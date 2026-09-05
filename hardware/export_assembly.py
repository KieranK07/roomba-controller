#!/usr/bin/env python3
"""Bake every part of the robot into its own STL, already positioned in the robot frame.

stl/ holds printed parts in their print-ready part-local frames and mesh/ holds vendor models in
whatever frame the vendor drew them in; neither is any use to someone who just wants to look at the
machine.  This writes export/: the same triangles, one file per part, each pre-multiplied by its
placement so that dropping the whole directory into one scene - a slicer, a viewer, Blender, a
CAD import - reassembles the robot with nothing left to line up by hand.

The transforms come from placement.py, which is also what verify.py measures, so the exported set
and the checked set are the same robot by construction.

The robot is LEVEL here.  assembly() in roomba_nx.scad wraps everything in rotate([DOCK_PITCH,0,0])
to stand it nose-up on the Home Base ramp; that is a docking pose, not the machine, and it is
dropped.  Excluded for the same reason: the Home Base itself, and the cable harness, which has no
geometry in this repo at all.

    python3 hardware/export_assembly.py
"""
import os

import numpy as np
import trimesh

import placement as P

OUT = os.path.join(P.HERE, "export")

# (placement name, output stem, what it is).  Prefixes group the listing: what you print, what you
# buy, and the robot they both bolt to.
EXPORT = [
    ("roomba_shell",   "shell_roomba_690",           "stock 690 chassis and bumper - the datum"),
    ("hub",            "printed_hub",                "hub ring around the Clean button"),
    ("jetson_plate",   "printed_jetson_plate",       "right carrier, under the Jetson"),
    ("front_plate",    "printed_front_plate",        "front carrier, DROK + camera cheeks"),
    ("camera_rocker",  "printed_camera_rocker",      "D435i rocker, at the -8 deg design tilt"),
    ("battery_cradle", "printed_battery_cradle",     "left carrier, the pack cradle"),
    ("battery_lid",    "printed_battery_lid",        "cradle lid, carries the BMS and INA219"),
    ("pad_carrier",    "printed_pad_carrier",        "charge pads, underside, pockets down"),
    ("dock_pin_block", "printed_dock_pin_block",     "dock-side pogo block - see the note below"),
    ("jetson",         "bought_jetson_devkit_board", "Jetson Xavier NX devkit carrier board"),
    ("cooler",         "bought_jetson_devkit_cooler", "the devkit's heatsink and fan"),
    ("camera",         "bought_realsense_d435i",     "Intel RealSense D435i"),
    ("pack",           "bought_lipo_3s",             "Ovonic 3S 8000 mAh pack"),
    ("drok",           "bought_drok_buck",           "DROK buck converter"),
    ("bms",            "bought_bms_3s",              "Daier 3S BMS"),
    ("ina219",         "bought_ina219",              "INA219 current monitor"),
]

NOTES = """\
## Frame

Every file is in the **robot frame** of `dimensions.json` `_frame`: origin at the Roomba's centre
**on the floor plane**, +X right, +Y forward (towards the bumper), +Z up, millimetres.  Coordinates
are baked into the vertices, so the correct import transform for all of these is the identity.

The robot is **level**.  `assembly()` in `roomba_nx.scad` wraps the whole machine in
`rotate([DOCK_PITCH, 0, 0])` = {pitch:.2f} deg to stand it nose-up on the Home Base ramp; that is a
docking pose rather than part of the machine, so it is not applied here.  The only rotations left
about a horizontal axis are the two the parts genuinely have: the camera rocker and the D435i at
their {tilt:+.0f} deg design tilt, and the pad carrier flipped 180 deg because it prints
pockets-up and installs pockets-down.

## Files

| file | x0 | x1 | y0 | y1 | z0 | z1 | solid | what it is |
|---|---:|---:|---:|---:|---:|---:|:-:|---|
{rows}

Overall bounding box  x {b[0][0]:.1f} .. {b[1][0]:.1f}   y {b[0][1]:.1f} .. {b[1][1]:.1f}   \
z {b[0][2]:.1f} .. {b[1][2]:.1f} mm.
Nothing reaches below z = 0: the shell stands on the floor plane and is the lowest thing in the set.

"solid" is trimesh's watertight test after the transform.  The printed parts are all closed.  The
four that are not - the Jetson board, the D435i, the INA219 and the Roomba shell - are vendor and
scan meshes that arrived open; they are reference bodies, not printed geometry, and this export
does not change them.

## Deliberately excluded

- **Home Base dock.** `ref_dock()` in the SCAD.  This is a set of ROBOT parts; the charging station
  is furniture the robot drives onto.  Keeping it would also have forced the dock pitch back in and
  put a 114 mm tall box in front of the machine.
- **Cable harness.** Not excluded so much as absent: there is no harness geometry anywhere in the
  repo.  Routing is described in the build sheet, not modelled.
- `ref_roomba()`'s Clean-button disc and screw-boss studs.  Those are OpenSCAD annotations drawn on
  top of the shell mesh to show where things land, not objects.

The **pad carrier and the dock pin block are both IN** despite being dock-related.  They bolt to
the robot / mate with it and are printed here; only the station itself is out.

## Where the dock pin block went, and why

On the real machine the pin block does not belong to the robot at all - `assembly()` bolts it to
the Home Base ramp, {ramp:.1f} deg nose-up, and the robot drives its pads down onto it.  Deleting
the dock leaves it with no host.

It is exported standing **flat on the floor plane at (0, {pads:.0f}), pins up** - its own plan
position from `assembly()`, with the ramp's {ramp:.1f} deg tilt and its {rise:.3f} mm rise dropped
along with the ramp.  So it sits exactly under the pad carrier it mates with, in the orientation it
mates in, and it reads as what it is: the other half of the contact pair, waiting on the ground.
Two properties made that the choice over its own origin: it stays out of the shell (the origin is
inside the robot), and its relationship to the pads - the only thing about it anyone needs to see -
survives.

Consequence worth knowing: **the block's top face at z 6.0 overlaps the pad carrier's face at
z 4.5 by 1.5 mm.**  That is not a modelling error, it is the missing dock pitch.  A 6 mm block plus
a 3.5 mm pad carrier is 9.5 mm and the chassis clearance is only {z0:.0f} mm, so the pair only fits
when the robot is nose-up on the ramp and its underside at y {pads:.0f} has lifted about 4 mm.  Level
and dockless, they must interfere.  `dimensions.json` `contact_geometry._BLOCKED` already warns that
this contact geometry is unresolved pending a measurement of the real ramp; this is the same problem
seen from the other side.  Do not read the overlap as a fit check.
"""


def main():
    os.makedirs(OUT, exist_ok=True)
    rows, scene = [], []
    for name, stem, what in EXPORT:
        m = P.placed(name)
        m.export(os.path.join(OUT, stem + ".stl"))
        b = m.bounds
        scene.append(m)
        rows.append(f"| `{stem}.stl` | {b[0][0]:.1f} | {b[1][0]:.1f} | {b[0][1]:.1f} | {b[1][1]:.1f} "
                    f"| {b[0][2]:.2f} | {b[1][2]:.2f} | {'yes' if m.is_watertight else 'no'} | {what} |")
        print(f"{stem:30}{b[0][0]:8.1f}{b[1][0]:8.1f}{b[0][1]:8.1f}{b[1][1]:8.1f}{b[0][2]:8.2f}{b[1][2]:8.2f}")

    lo = np.min([m.bounds[0] for m in scene], axis=0)
    hi = np.max([m.bounds[1] for m in scene], axis=0)
    ramp = np.degrees(np.arctan(P.D["home_base"]["ramp_h"] / P.D["home_base"]["ramp_d"]))
    dock_y = P.R690["diameter"] / 2 - P.D["home_base"]["ramp_d"] + 10
    py = P.L["pads_center_y"] - dock_y
    caster_y = P.R690["caster_position"][1]
    pitch = np.degrees(np.arcsin(((caster_y - dock_y) / P.D["home_base"]["ramp_d"]
                                  * P.D["home_base"]["ramp_h"]) / caster_y))

    with open(os.path.join(OUT, "MANIFEST.md"), "w") as f:
        f.write("# Robot-frame STL export\n\nWritten by `hardware/export_assembly.py`. "
                "Do not edit by hand - rerun the script.\n\n")
        f.write(NOTES.format(rows="\n".join(rows), b=(lo, hi), pitch=pitch, tilt=P.L["camera_tilt_deg"],
                             ramp=ramp, pads=P.L["pads_center_y"], z0=P.Z0,
                             rise=py / P.D["home_base"]["ramp_d"] * P.D["home_base"]["ramp_h"]))

    print(f"\n{len(EXPORT)} STLs + MANIFEST.md -> {OUT}")
    print(f"overall  x {lo[0]:.1f}..{hi[0]:.1f}  y {lo[1]:.1f}..{hi[1]:.1f}  z {lo[2]:.2f}..{hi[2]:.2f}")
    assert lo[2] >= -1e-6, "something is below the floor plane"


if __name__ == "__main__":
    main()
