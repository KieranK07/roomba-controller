#!/usr/bin/env python3
"""Build hardware/roomba-nx-printed-parts.zip: every printed STL, oriented for the bed, plus a README.

    arch -arm64 .venv/bin/python hardware/package_print.py

Reads hardware/stl/ (export those first - see the header of roomba_nx.scad) and dimensions.json.
Nothing here changes geometry.  It only decides how each part lies on the bed, and writes down what
a person holding the zip needs to know before they spend filament.

Orientation is measured, not assumed.  Each part is tried as exported and flipped 180 deg about X,
and the flip is kept only when it both puts more area on the bed and sheds at least 1 cm^2 of
overhang steeper than 45 deg.  This exists because battery_lid once shipped standing on its skirt
with a 74 cm^2 bridge over it: the SCAD comment said it "prints flat", and it did not.

The zip is a GitHub release asset, never a tracked file - see .gitignore.
"""
import datetime
import json
import os
import zipfile

import numpy as np
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
D = json.load(open(os.path.join(HERE, "dimensions.json")))
OUT = os.path.join(HERE, "roomba-nx-printed-parts.zip")

PLA, INFILL = 1.24e-3, 0.45          # the same constants verify.py balances the robot with

# (part, what it is, anything a printer must be told).  Order is build order, hub first.
PARTS = [
    ("hub", "Kieran's hub, reproduced from his CAD person's model (roomba_top_deck.step) - the ONLY "
            "attachment between the frame and the robot.",
     "He already has one printed and drilled. Only print this as a replacement, and note the four "
     "Roomba-mount holes on the diagonals are an ESTIMATE until his CAD person models them."),
    ("jetson_plate", "Right carrier, under the Jetson on 9 mm standoffs. Bolts to the hub's right pad.", ""),
    ("front_plate", "Front carrier: the DROK and the camera cheeks. Bolts to the hub's front pad.", ""),
    ("camera_rocker", "D435i friction-clamp rocker; the pivot runs through the camera's optical centre.",
     "THREE separate pieces by design - a strap and two ears 1.5 mm off it, joined by the camera and "
     "its screws. Each ear is a 21.5 mm tower on an 8 x 23 mm footprint: use a brim."),
    ("battery_cradle", "Left carrier, the pack cradle. Sits flush on the deck; bolts to the hub's left pad.",
     "The pocket is cut exactly to the owned pack's PADDED envelope - no extra clearance."),
    ("battery_lid", "Cradle lid; carries the BMS and INA219.", ""),
    ("pad_carrier", "Charge pads, underside.",
     "Do NOT print yet: contact_geometry is blocked on a measurement of the real dock ramp."),
    ("dock_pin_block", "Dock-side pogo block.",
     "Do NOT print yet: contact_geometry is blocked on a measurement of the real dock ramp."),
]


def scored(m):
    """(area on the bed cm^2, overhang steeper than 45 deg cm^2) with the part dropped to z = 0."""
    m = m.copy()
    m.apply_translation([0, 0, -m.bounds[0][2]])
    n, c, a = m.face_normals, m.triangles_center, m.area_faces
    bed = a[np.abs(c[:, 2]) < 0.05].sum() / 100
    over = a[(n[:, 2] < -np.cos(np.radians(45))) & (c[:, 2] > 0.4)].sum() / 100
    return m, bed, over


def main():
    rows = []
    with zipfile.ZipFile(OUT, "w", zipfile.ZIP_DEFLATED, compresslevel=9) as z:
        for name, what, warn in PARTS:
            src = os.path.join(HERE, "stl", name + ".stl")
            if not os.path.exists(src):
                raise SystemExit(f"missing {src} - export the STLs first")
            as_is, b0, o0 = scored(trimesh.load(src))
            flip = trimesh.load(src)
            flip.apply_transform(trimesh.transformations.rotation_matrix(np.pi, [1, 0, 0]))
            flipped, b1, o1 = scored(flip)
            use_flip = b1 > b0 and o1 < o0 - 1.0
            m, bed, over = (flipped, b1, o1) if use_flip else (as_is, b0, o0)
            if not (m.is_watertight and m.is_winding_consistent):
                raise SystemExit(f"{name} is not a closed solid")
            z.writestr(name + ".stl", trimesh.exchange.stl.export_stl(m))
            e = m.extents
            rows.append(dict(name=name, what=what, warn=warn, flip=use_flip, x=e[0], y=e[1], z=e[2],
                             vol=m.volume / 1000, g=m.volume * PLA * INFILL, bed=bed, over=over,
                             bodies=len(m.split(only_watertight=False))))
            print(f"{name:16} {'FLIPPED' if use_flip else 'as exported':12} "
                  f"{e[0]:6.1f} x {e[1]:6.1f} x {e[2]:5.1f}  bed {bed:5.1f} cm2  overhang {over:5.1f} cm2")
        z.writestr("README.md", readme(rows))
    print(f"\n{OUT}  {os.path.getsize(OUT) / 1e6:.2f} MB")


def readme(rows):
    pr = D["print"]
    bed = max(max(r["x"], r["y"]) for r in rows)
    live = [r for r in rows if "Do NOT print" not in r["warn"]]
    L = ["# Roomba NX - printed parts", "",
         f"Built {datetime.date.today().isoformat()} by `hardware/package_print.py` from `hardware/stl/`, "
         "which `roomba_nx.scad` exports from `dimensions.json`. Placement is from Kieran's measured "
         "robot top (`hardware/reference/roomba_top_deck.step`), and `hardware/verify.py` passes on "
         "this geometry.", "",
         f"**{len(live)} parts to print now, ~{sum(r['g'] for r in live):.0f} g** in PLA at 45 % effective "
         f"infill. The largest part needs a {bed:.0f} mm bed.", "",
         "## Orientation", "",
         "**Print every file as it arrives.** Each is already flat on z = 0 in the orientation that "
         "puts the most area on the bed with the least overhang. Files marked `*` are flipped relative "
         "to `hardware/stl/`; that is deliberate and measured.", "",
         "## Parts", "",
         "| file | footprint | height | ~mass | on bed | overhang >45 deg | bodies |",
         "|---|---|---:|---:|---:|---:|---:|"]
    for r in rows:
        L.append(f"| `{r['name']}.stl`{' *' if r['flip'] else ''} | {r['x']:.1f} x {r['y']:.1f} mm | "
                 f"{r['z']:.1f} mm | {r['g']:.0f} g | {r['bed']:.1f} cm2 | {r['over']:.1f} cm2 | {r['bodies']} |")
    L.append("")
    for r in rows:
        L.append(f"**`{r['name']}.stl`** - {r['what']}" + (f" **{r['warn']}**" if r["warn"] else ""))
        L.append("")
    L += ["## Mounting", "",
          "The hub is the only thing fixed to the robot, through four holes into the Roomba. Each "
          "carrier bolts to one hub pad with two M3 countersunk flat heads, flush with the tongue. "
          "**How those bolts hold in the hub is not settled** - its pad holes are "
          f"{D['deck_measured']['hub']['hole_d']} mm, too big to tap for M3, and a nut cannot fit "
          "under a plate lying on the deck. Decide that (heat-set inserts, a tap, or through into the "
          "shell) before printing the carriers.", "",
          "## Design rules baked in", "", "| | |", "|---|---|",
          f"| wall | {pr['wall']} mm |", f"| floor | {pr['floor']} mm |",
          f"| M3 clearance | {pr['m3_clearance_d']} mm |", f"| M3 tap | {pr['m3_tap_d']} mm |",
          f"| M3 heat-set | {pr['m3_heatset_d']} mm |", f"| fit clearance | {pr['fit_clearance']} mm |", "",
          "From the `print` block of `dimensions.json`. If your printer runs tight or loose, change them "
          "there and re-export rather than scaling STLs.", "",
          "## Regenerating", "", "```sh", "cd hardware", "python3 gen.py",
          "for p in hub jetson_plate front_plate camera_rocker battery_cradle battery_lid \\",
          "         pad_carrier dock_pin_block; do",
          "  openscad -D \"part=\\\"$p\\\"\" -o \"stl/$p.stl\" roomba_nx.scad", "done",
          "python3 verify.py", "python3 package_print.py", "```", ""]
    return "\n".join(L)


if __name__ == "__main__":
    main()
