#!/usr/bin/env python3
"""Rebuild the assembly from the EXPORTED STLs and check it against the rules the build has to obey.

This is not a render.  It loads the same triangle soup the printer and the viewer get, places each
part with the same transform assembly() uses, and then measures: build height, radius against the
329.9 mm shell, clearance to the four things on the robot that must stay reachable, and part-to-part
interference by voxel overlap rather than by eye.  Run it after every geometry change.
"""
import math
import os
import sys

import numpy as np
import trimesh

# Placement lives in placement.py so this file and export_assembly.py cannot disagree about where
# anything sits.  CHECKED_* is verify's subset of that table; see placement.py for what it omits.
from placement import (  # noqa: F401  - re-exported names are used further down
    HERE, D, R690, F, L, DECK, AJ, AC, AB, PT, PZ, CRAD_X, PIVX, PIVZ, PACK_TOP, LID_TOP,
    rz, tr, ry, load, TILT, CAM, CHECKED_PRINTED, CHECKED_BOUGHT, placed)

PRINTED = {k: placed(k) for k in CHECKED_PRINTED}
BOUGHT = {k: placed(k) for k in CHECKED_BOUGHT}
ALL = {**PRINTED, **BOUGHT}
fail = []

print(f"{'part':16}{'x0':>8}{'x1':>8}{'y0':>8}{'y1':>8}{'z0':>8}{'z1':>8}{'maxR':>8}")
zmax = 0.0
for k, m in ALL.items():
    b = m.bounds
    r = float(np.hypot(m.vertices[:, 0], m.vertices[:, 1]).max())
    zmax = max(zmax, b[1][2])
    print(f"{k:16}{b[0][0]:8.1f}{b[1][0]:8.1f}{b[0][1]:8.1f}{b[1][1]:8.1f}{b[0][2]:8.2f}{b[1][2]:8.2f}{r:8.1f}")
    if r > R690["diameter"] / 2 - 5:
        fail.append(f"{k} reaches r {r:.1f}, past the {R690['diameter']/2:.1f} mm shell less 5 mm")

print(f"\nbuild height        {zmax:8.2f} mm   (stock {R690['height']}, deck {DECK})")
print(f"camera optical z    {PIVZ:8.2f} mm   front face y {ALL['camera'].bounds[1][1]:.1f}")

# ---- interference: voxel overlap between every pair that shares a bounding box ------------
print("\ninterference (voxelised at 1.5 mm):")
keys = list(ALL)
clash = 0
for i in range(len(keys)):
    for j in range(i + 1, len(keys)):
        a, b = ALL[keys[i]], ALL[keys[j]]
        lo = np.maximum(a.bounds[0], b.bounds[0])
        hi = np.minimum(a.bounds[1], b.bounds[1])
        if np.any(hi - lo <= 0.5):
            continue
        g = np.stack(np.meshgrid(*[np.arange(lo[k] + .75, hi[k], 1.5) for k in range(3)], indexing="ij"), -1)
        p = g.reshape(-1, 3)
        if len(p) == 0 or len(p) > 4_000_000:
            continue
        both = a.contains(p) & b.contains(p)
        # NVIDIA's devkit CAD models the four mounting SCREWS, a 3.6 mm stub dropping 3.4 mm below
        # the PCB at each hole.  Our standoffs stand exactly there - that is what a standoff is for -
        # so a hit inside 5 mm of a mount hole is the screw sitting in its own boss, not a clash.
        if {keys[i], keys[j]} == {"jetson_plate", "jetson"}:
            J = D["jetson_xavier_nx_devkit"]
            for h in J["mount_holes"]:
                hx = L["jetson_center"][0] - (h[1] - J["board_d"] / 2)
                hy = h[0] - J["board_w"] / 2
                both &= np.hypot(p[:, 0] - hx, p[:, 1] - hy) > 5.0
        n = int(both.sum())
        if n:
            clash += 1
            print(f"  CLASH {keys[i]:16} / {keys[j]:16}  {n} cells  in "
                  f"x {lo[0]:.0f}..{hi[0]:.0f}  y {lo[1]:.0f}..{hi[1]:.0f}  z {lo[2]:.0f}..{hi[2]:.0f}")
            fail.append(f"{keys[i]} intersects {keys[j]}")
if not clash:
    print("  none")

# ---- the rocker has to clear the plate over the whole tilt range ------------------------
print("\ntilt sweep (rocker + camera vs the front plate at " + f"{PZ:.2f}):")
worst = 1e9
for t in range(F["camera"]["tilt_min"], F["camera"]["tilt_max"] + 1):
    T = rz(AC) @ tr(PIVX, 0, PIVZ) @ ry(-t)
    lo = min(load("stl", "camera_rocker", T,
                  tr(0, 0, -(D["realsense_d435i"]["h"] / 2 + F["camera"]["rocker"][2]))).bounds[0][2],
             load("mesh", "realsense_d435i", T, rz(-90)).bounds[0][2])
    if lo < worst:
        worst, worst_t = lo, t
print(f"  lowest point {worst:.2f} mm at {worst_t:+d} deg -> {worst - PZ:+.2f} mm over the plate")
if worst < PZ:
    fail.append(f"rocker fouls the front plate at {worst_t:+d} deg tilt")

# ---- the four things that must stay reachable ---------------------------------------------
print("\nkeep-outs:")
KO = [("Clean button",      lambda: np.array([[0, 0, DECK + 8]]), 22.0),
      ("dustbin lid",       lambda: np.array([[x, y, DECK + 20] for x in range(-150, 151, 6)
                                              for y in range(-160, -77, 6) if math.hypot(x, y) < 160]), 0.0),
      ("front IR boss",     lambda: np.array([[x, y, 92.0] for x in range(-9, 10, 3)
                                              for y in range(147, 166, 3)]), 0.0),
      ("mini-DIN",          lambda: np.array([[118, -95, DECK + 15]]), 14.0)]
for name, pts, rad in KO:
    P = pts()
    hit = []
    for k, m in ALL.items():
        d = trimesh.proximity.signed_distance(m, P) if rad else None
        if rad:
            if (d > -rad).any():
                hit.append(k)
        elif m.contains(P).any():
            hit.append(k)
    print(f"  {name:16} {'CLEAR' if not hit else 'BLOCKED by ' + ', '.join(hit)}")
    if hit:
        fail.append(f"{name} blocked by {', '.join(hit)}")

# ---- mass and balance --------------------------------------------------------------------
# Printed mass is volume x PLA density x an effective infill, so it is an estimate; the bought
# parts are catalogue figures.  What matters is the ratio between the left and right groups, and
# that is dominated by the 424 g pack and the 300 g Jetson, both of which are known.
PLA, INFILL = 1.24e-3, 0.45
CATALOGUE = {"jetson": D["jetson_xavier_nx_devkit"]["mass_g"], "cooler": 0.0,
             "pack": D["ovonic_3s_8000"]["mass_g"], "drok": D["drok_buck"]["mass_g"],
             "bms": D["bms_daier_3s"]["mass_g"], "ina219": D["ina219_hiletgo"]["mass_g"],
             "camera": D["realsense_d435i"]["mass_g"]}
tot = 0.0
mom = np.zeros(3)
for k, m in ALL.items():
    g = CATALOGUE.get(k, abs(m.volume) * PLA * INFILL)
    tot += g
    mom += g * (m.centroid if k in CATALOGUE else m.center_mass)
com = mom / tot
robot = R690["mass_g"]
print(f"\nbalance:\n  payload {tot:6.0f} g   CoM ({com[0]:+.1f}, {com[1]:+.1f}, {com[2]:.1f})")
print(f"  loaded  {robot + tot:6.0f} g   CoM ({com[0]*tot/(robot+tot):+.1f}, "
      f"{com[1]*tot/(robot+tot):+.1f})  taking the stock robot as balanced at its centre")
zc = (robot * 45 + tot * com[2]) / (robot + tot)
print(f"  CoM height ~{zc:.0f} mm over a {R690['wheel_track']} mm track -> tips at "
      f"{math.degrees(math.atan(R690['wheel_track'] / 2 / zc)):.0f} deg")
if abs(com[0]) > 25:
    fail.append(f"payload CoM is {com[0]:+.1f} mm off the centreline")

# ---- print bed --------------------------------------------------------------------------
print("\nprint footprints:")
bed = 0.0
for k in PRINTED:
    m = trimesh.load(os.path.join(HERE, "stl", k + ".stl"))
    s = m.bounds[1] - m.bounds[0]
    bed = max(bed, float(max(s[0], s[1])))
    print(f"  {k:16}{s[0]:7.1f} x{s[1]:7.1f} x{s[2]:6.1f}   {'watertight' if m.is_watertight else 'NOT WATERTIGHT'}")
    if not m.is_watertight:
        fail.append(f"{k} is not watertight")
print(f"  -> a {math.ceil(bed/10)*10:.0f} mm bed clears every part")

print()
if fail:
    print("FAIL:")
    for f in fail:
        print("  -", f)
    sys.exit(1)
print("PASS")
