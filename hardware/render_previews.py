#!/usr/bin/env python3
"""Regenerate every PNG in hardware/preview/ from roomba_nx.scad.

Five assembly views (the robot pitched nose-up on the dock, exactly as assembly() draws it) and one
view per printed part in its print orientation.  README.md embeds assembly_iso, assembly_exploded
and assembly_top, so re-run this whenever the geometry moves:

    python3 hardware/render_previews.py            # everything
    python3 hardware/render_previews.py top hub    # just the named views / parts

OpenSCAD's preview renderer (OpenCSG) is used, not --render: it keeps the per-part colours and the
translucent shell, and it draws the vendor meshes (the Jetson CAD is not a closed solid, and a full
CGAL/Manifold render drops it).  Cameras are fixed gimbal cameras, not --viewall, so a moved part
never silently re-frames a picture; the part shots use --viewall because each is one small solid.
"""
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SCAD = os.path.join(HERE, "roomba_nx.scad")
OUT = os.path.join(HERE, "preview")
OPENSCAD = os.environ.get("OPENSCAD", "/usr/local/bin/openscad")
if not os.path.exists(OPENSCAD):
    OPENSCAD = "openscad"
SCHEME = "Cornfield"

# name -> (image size, gimbal camera tx,ty,tz,rx,ry,rz,dist, explode mm).  rx 55 / rz 25 is
# OpenSCAD's own default view angle; front and side are its "Front" (looking +Y) and "Right" presets.
ASSEMBLY = {
    "iso":      ((1400, 1000), (0, 10, 75, 55, 0, 25, 760), 0),
    "exploded": ((1400, 1000), (0, 20, 180, 55, 0, 25, 960), 40),
    "top":      ((1040, 1040), (0, 0, 80, 0, 0, 0, 830), 0),
    "front":    ((1400, 700),  (0, 0, 70, 90, 0, 0, 610), 0),
    "side":     ((1400, 700),  (0, 55, 70, 90, 0, 90, 700), 0),
}
# printed parts, in the orientation they come off the printer; the two widest get a larger canvas
PARTS = {
    "hub": (900, 700), "jetson_plate": (900, 700), "battery_cradle": (900, 700),
    "battery_lid": (900, 700), "front_plate": (1000, 760), "camera_rocker": (1000, 760),
    "pad_carrier": (900, 700), "dock_pin_block": (900, 700),
}


def openscad(out, size, part, camera=None, explode=0):
    cmd = [OPENSCAD, "-o", out, f"--imgsize={size[0]},{size[1]}", f"--colorscheme={SCHEME}",
           "-D", f'part="{part}"', "-D", f"explode={explode}"]
    cmd += [f"--camera={','.join(str(c) for c in camera)}"] if camera else ["--viewall", "--autocenter"]
    r = subprocess.run(cmd + [SCAD], capture_output=True, text=True)
    bad = [l for l in (r.stdout + r.stderr).splitlines() if l.startswith(("ERROR", "WARNING"))]
    if r.returncode or bad or not os.path.exists(out):
        sys.exit(f"openscad failed for {out}:\n" + "\n".join(bad or [r.stderr[-2000:]]))


def main(only):
    jobs = [(f"assembly_{n}.png", s, "assembly", c, e) for n, (s, c, e) in ASSEMBLY.items()]
    jobs += [(f"part_{n}.png", s, n, None, 0) for n, s in PARTS.items()]
    if only:
        jobs = [j for j in jobs if any(k in j[0] for k in only)]
        if not jobs:
            sys.exit(f"nothing matches {only}; views are {list(ASSEMBLY)}, parts are {list(PARTS)}")
    for name, size, part, cam, ex in jobs:
        openscad(os.path.join(OUT, name), size, part, cam, ex)
        print(f"  {name:28s} {size[0]}x{size[1]}")


if __name__ == "__main__":
    main(sys.argv[1:])
