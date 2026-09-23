#!/usr/bin/env python3
"""Rebuild the assembly from the EXPORTED STLs and check it against the rules the build has to obey.

This is not a render.  It loads the same triangle soup the printer and the viewer get, places each
part with the same transform assembly() uses, and then measures: build height, every part against
the MEASURED top deck (dimensions.json deck_measured - radius limits, the raised ring, screw A, the
hub as the only attachment), clearance to the things on the robot that must stay reachable, and
part-to-part interference by voxel overlap rather than by eye.  Run it after every geometry change.
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

print(f"{'part':16}{'x0':>8}{'x1':>8}{'y0':>8}{'y1':>8}{'z0':>8}{'z1':>8}")
zmax = 0.0
for k, m in ALL.items():
    b = m.bounds
    zmax = max(zmax, b[1][2])
    print(f"{k:16}{b[0][0]:8.1f}{b[1][0]:8.1f}{b[0][1]:8.1f}{b[1][1]:8.1f}{b[0][2]:8.2f}{b[1][2]:8.2f}")

print(f"\nbuild height        {zmax:8.2f} mm   (stock {R690['height']}, deck {DECK})")
if "camera" in ALL:
    print(f"camera optical z    {PIVZ:8.2f} mm   front face y {ALL['camera'].bounds[1][1]:.1f}")

# ---- the measured deck ------------------------------------------------------------------
# Every number here is read from deck_measured, which was measured off Kieran's own robot and
# supersedes the Create 2 CAD for anything on the top deck.  Heights there are ABOVE the deck flat.
DM = D["deck_measured"]
LIM = DM["limits"]
R_ON, UNDER = LIM["on_deck_r_max"], LIM["over_ring_underside_min"]
R_FRONT, R_REAR = LIM["front_r_max"], LIM["rear_r_max"]
SA, SA_R = np.array(DM["screw_a"]), DM["screw_a_berth"]
SA_TOP = DECK + DM["screw_a_h"] + 0.5
HUB = DM["hub"]

# STL vertices are sparse on big flat faces, and the limits here are about where a FACE is: the
# underside of a plate crossing the ring, a strap lying over screw A.  So every part is checked as
# its vertices plus a seeded surface sample, dense enough (2 per mm^2) that nothing mid-face hides.
DENSITY = 2.0


def cloud(m, seed):
    n = max(int(m.area * DENSITY), 1000)
    return np.vstack([m.vertices, trimesh.sample.sample_surface(m, n, seed=seed)[0]])


def xf(T, P):
    return P @ T[:3, :3].T + T[:3, 3]


# The camera and its rocker are checked over the WHOLE tilt range, not just the nominal angle: at
# the ends of the arc they reach further forward and lower than at -8.  Their clouds are sampled
# once where placement.py puts them and moved by (arc pose) x (nominal pose)^-1 for each degree.
TILTS = range(F["camera"]["tilt_min"], F["camera"]["tilt_max"] + 1)
SWEPT = {"camera_rocker", "camera"}
UNCAM = np.linalg.inv(CAM)


def poses(k, P):
    if k not in SWEPT:
        yield None, P
        return
    for t in TILTS:
        yield t, xf(rz(AC) @ tr(PIVX, 0, PIVZ) @ ry(-t) @ UNCAM, P)


print(f"\nmeasured deck (deck {DECK}; front r <= {R_FRONT:.2f}, rear r <= {R_REAR:.2f}, "
      f"past r {R_ON:.2f} underside >= +{UNDER:.2f}, screw A berth {SA_R} below +{SA_TOP - DECK:.2f}):")
print(f"  {'part':16}{'r front':>9}{'margin':>8}{'r rear':>9}{'margin':>8}{'z past':>9}{'margin':>8}"
      f"{'z0':>8}{'A dist':>8}{'margin':>8}  verdict")
CLOUD = {}
for i, (k, m) in enumerate(ALL.items()):
    P0 = CLOUD[k] = cloud(m, seed=i)
    rf = rr = -math.inf                    # furthest reach, front and rear half
    zp = math.inf                          # lowest point past on_deck_r_max, above the deck
    z0 = math.inf                          # lowest point overall, above the deck
    da = math.inf                          # nearest approach to screw A, among points below SA_TOP
    at = {}                                # tilt at which each worst case happens
    for t, P in poses(k, P0):
        r = np.hypot(P[:, 0], P[:, 1])
        fr = P[:, 1] >= 0
        if fr.any() and r[fr].max() > rf:
            rf, at["rf"] = float(r[fr].max()), t
        if (~fr).any() and r[~fr].max() > rr:
            rr, at["rr"] = float(r[~fr].max()), t
        past = r > R_ON
        if past.any() and P[past, 2].min() - DECK < zp:
            zp, at["zp"] = float(P[past, 2].min() - DECK), t
        if P[:, 2].min() - DECK < z0:
            z0, at["z0"] = float(P[:, 2].min() - DECK), t
        lo = P[:, 2] < SA_TOP
        if lo.any():
            d = float(np.hypot(P[lo, 0] - SA[0], P[lo, 1] - SA[1]).min())
            if d < da:
                da, at["da"] = d, t
    # A surface sample cannot see a solid that swallows the berth whole, so also probe the berth's
    # own volume.  Only at the nominal pose: the swept parts are nowhere near the rear-left ring.
    grid = np.array([[SA[0] + rr_ * math.cos(a), SA[1] + rr_ * math.sin(a), z]
                     for rr_ in (0.0, SA_R / 2, SA_R - 0.05) for a in np.radians(range(0, 360, 45))
                     for z in np.arange(DECK + 0.25, SA_TOP, 0.5)])
    if (np.all(m.bounds[0] <= grid.max(0)) and np.all(m.bounds[1] >= grid.min(0))
            and m.contains(grid).any()):
        da = 0.0
    bad = []
    if rf > R_FRONT:
        bad.append(f"reaches r {rf:.1f} in the FRONT half, past {R_FRONT:.2f}")
    if rr > R_REAR:
        bad.append(f"reaches r {rr:.1f} in the rear half, past {R_REAR:.2f}")
    if zp < UNDER:
        bad.append(f"is only {zp:+.2f} over the deck past r {R_ON:.2f} (ring needs {UNDER:+.2f})")
    if z0 < -0.01:
        bad.append(f"goes {z0:.2f} below the deck")
    if da < SA_R:
        bad.append(f"comes {da:.2f} from screw A below deck +{SA_TOP - DECK:.2f} (berth {SA_R})")
    tilt = lambda key: f"@{at[key]:+d}" if at.get(key) is not None else ""
    f = lambda v: f"{v:.1f}" if math.isfinite(v) else "-"
    mg = lambda v, lim, sign: f"{sign * (lim - v):+.1f}" if math.isfinite(v) else "-"
    print(f"  {k:16}{f(rf):>9}{mg(rf, R_FRONT, 1):>8}{f(rr):>9}{mg(rr, R_REAR, 1):>8}"
          f"{('-' if not math.isfinite(zp) else f'{zp:+.2f}'):>9}{mg(zp, UNDER, -1):>8}"
          f"{z0:+8.2f}{f(da):>8}{mg(da, SA_R, -1):>8}  {'FAIL' if bad else 'PASS'}"
          + (f"   worst tilt: r {tilt('rf')} z {tilt('zp')}" if k in SWEPT else ""))
    for b in bad:
        fail.append(f"{k} {b}" + (" (over the tilt range)" if k in SWEPT else ""))

# ---- hub-only mounting ------------------------------------------------------------------
# The hub is the ONLY thing bolted to the robot, and every carrier bolts to the hub through the two
# holes in its pad.  So each carrier must (a) lie on a pad the hub actually has, (b) be empty
# straight down through both hub holes - a vertical ray at the hole centre and on a ring just
# inside the hole wall must miss it entirely - and (c) have material around each hole for the bolt
# to clamp, or the "open hole" is just the carrier missing the hub.
print("\nhub-only mounting:")
CARRIERS = {"jetson_plate": AJ, "front_plate": AC, "battery_cradle": AB}
hr = HUB["hole_d"] / 2
ring = lambda x, y, rad: np.array([[x + rad * math.cos(a), y + rad * math.sin(a)]
                                   for a in np.radians(range(0, 360, 45))])
down = lambda xy: (np.column_stack([xy, np.full(len(xy), DECK + 500)]),
                   np.tile([0.0, 0.0, -1.0], (len(xy), 1)))
for k, a in CARRIERS.items():
    if k not in ALL:
        print(f"  {k:16} not in CHECKED_PRINTED - skipped")
        continue
    m = ALL[k]
    ray = trimesh.ray.ray_triangle.RayMeshIntersector(m)
    if not any(abs((a - p + 180) % 360 - 180) < 0.5 for p in HUB["pad_angles"]):
        print(f"  {k:16} at {a} deg: NO hub pad there (pads at {HUB['pad_angles']})")
        fail.append(f"{k} sits at {a} deg, where the hub has no pad")
        continue
    notes = []
    for r in HUB["hole_r"]:
        x, y = r * math.cos(math.radians(a)), r * math.sin(math.radians(a))
        hole = np.vstack([[x, y], ring(x, y, hr - 0.25)])
        blocked = int(ray.intersects_any(*down(hole)).sum())
        around = int(ray.intersects_any(*down(ring(x, y, hr + 2.5))).sum())
        notes.append(f"r{r}: {'open' if not blocked else f'BLOCKED ({blocked}/9 rays)'}, "
                     f"{'seated' if around == 8 else f'NO SEAT ({around}/8 rays)'}")
        if blocked:
            fail.append(f"{k} covers hub hole r{r} at ({x:.1f}, {y:.1f})")
        if around < 8:
            fail.append(f"{k} has no material round hub hole r{r} - it cannot bolt to the hub there")
    print(f"  {k:16} {'   '.join(notes)}")

# ... and no ears.  The old carriers also screwed down to the chassis bosses under the faceplate.
# An ear is recognised by what makes it one - a SCREW HOLE at a boss: the vertical ray down the
# boss's own axis passes clean through the part, and the part has material all round it 2.5-4 mm
# out for the screw head to seat on.  Plain floor lying over a boss position (no hole) is fine -
# the cradle is meant to sit flush on the deck - and so is a lightening hole too big to seat a
# screw.  Checked on the whole part, not just near the deck: a screw hole at a boss is an ear at
# any height.
BOSS = np.array(R690["screw_bosses"])
SEAT = (2.5, 3.25, 4.0)
for k in CHECKED_PRINTED:
    if k == "hub":
        continue
    ray = trimesh.ray.ray_triangle.RayMeshIntersector(ALL[k])
    hit = []
    for bx, by in BOSS:
        if ray.intersects_any(*down(np.array([[bx, by]])))[0]:
            continue                                           # no hole: floor, or not there at all
        seat = sum(int(ray.intersects_any(*down(ring(bx, by, r))).sum()) for r in SEAT)
        if seat >= 0.9 * 8 * len(SEAT):
            hit.append(f"({bx:g}, {by:g})")
    if hit:
        print(f"  {k:16} EARS: screw hole at chassis boss {', '.join(hit)}")
        fail.append(f"{k} has a screw hole at chassis boss {', '.join(hit)} (ears; the hub is the only mount)")

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
        # NVIDIA's devkit CAD models the four mounting SCREWS, a 3.6 mm stub dropping 3.4 mm below
        # the PCB at each hole.  Our standoffs stand exactly there - that is what a standoff is for -
        # so a hit inside 5 mm of a mount hole is the screw sitting in its own boss, not a clash.
        if {keys[i], keys[j]} == {"jetson_plate", "jetson"}:
            J = D["jetson_xavier_nx_devkit"]
            for h in J["mount_holes"]:
                hx = L["jetson_center"][0] - (h[1] - J["board_d"] / 2)
                hy = h[0] - J["board_w"] / 2
                p = p[np.hypot(p[:, 0] - hx, p[:, 1] - hy) > 5.0]
        # Ask the smaller mesh first and the other only about the cells the first one owns: same
        # answer as testing both everywhere, a fraction of the ray casts.
        a, b = sorted((a, b), key=lambda m: len(m.faces))
        p = p[a.contains(p)] if len(p) else p
        n = int(b.contains(p).sum()) if len(p) else 0
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
for t in TILTS:
    T = rz(AC) @ tr(PIVX, 0, PIVZ) @ ry(-t)
    lo = min(load("stl", "camera_rocker", T,
                  tr(0, 0, -(D["realsense_d435i"]["h"] / 2 + F["camera"]["rocker"][2]))).bounds[0][2],
             load("mesh", "realsense_d435i", T, rz(-90)).bounds[0][2])
    if lo < worst:
        worst, worst_t = lo, t
print(f"  lowest point {worst:.2f} mm at {worst_t:+d} deg -> {worst - PZ:+.2f} mm over the plate")
if worst < PZ:
    fail.append(f"rocker fouls the front plate at {worst_t:+d} deg tilt")

# ---- the things that must stay reachable --------------------------------------------------
# The front IR boss used to be a keep-out here.  It stands at y 147-165, which is past front_r_max
# everywhere: the measured-deck radius check above already forbids any part from reaching it, so a
# separate containment probe could only ever fire alongside that failure.  It is dropped, not lost.
print("\nkeep-outs:")
KO = [("Clean button",      lambda: np.array([[0, 0, DECK + 8]]), 22.0),
      ("dustbin lid",       lambda: np.array([[x, y, DECK + 20] for x in range(-150, 151, 6)
                                              for y in range(-160, -77, 6) if math.hypot(x, y) < 160]), 0.0),
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
