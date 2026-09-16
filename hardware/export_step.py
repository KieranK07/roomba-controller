#!/usr/bin/env python3
"""Regenerate hardware/cad-export/*.step - one analytic B-rep solid per printed part.

    FC=/Applications/FreeCAD.app/Contents/Resources/bin/FreeCADCmd
    $FC hardware/export_step.py                                  # all eight
    $FC hardware/export_step.py dock_pin_block                   # just one
    $FC hardware/export_step.py --pass --part hub --out /tmp/try

This has to run inside FreeCAD's interpreter (it needs Part and the OpenSCAD workbench), which is
why it is not executable on its own and why there is no argparse: FreeCADCmd claims every argument
that starts with '-' for itself unless it comes after `--pass`, and then hands the leftovers to
both the script and its own document opener - so any form carrying a real path prints one
"File format not supported" line at FreeCAD's expense.  Harmless; the bare-part-name form is quiet.

STL is what you print; STEP is what you open in SolidWorks or Fusion and put a dimension on.  An
STL of a 3 mm fillet is a fan of flat strips, so a STEP that was round-tripped through a mesh is
no better - you cannot select "the cylinder" because there isn't one.  Everything here exists to
make each part arrive as the primitives it was drawn from.

Two routes, and the reason for the second one is the interesting part:

  * Seven parts go OpenSCAD -> .csg -> FreeCAD importCSG -> STEP.  A .csg file is the CSG tree
    after OpenSCAD has evaluated all the modules and loops, and importCSG maps its primitives
    straight onto Part primitives, so a cylinder() lands as a real Part::Cylinder.  roomba_nx.scad
    is written to keep that true - see the notes on slot() and rounded_box() there, which spell out
    unions of primitives rather than hull() precisely so this route stays analytic.

  * dock_pin_block is built here, in OpenCASCADE, by hand.  See build_dock_pin_block().

Both routes are checked against hardware/stl/, which is generated from the same .scad and is what
actually gets printed.  Every STEP volume lands within 0.012 % of its STL, and sampling the two
surfaces against each other puts dock_pin_block - the one that did not come from the .scad at all -
within 0.003 mm.  That is not slop: a $fn = 72 polygon inscribed in an r = 3 arc sits 0.0029 mm
inside it, so the STEP is the exact version of the shape the STL approximates, as intended.
"""
import json
import math
import os
import subprocess
import sys
import tempfile

import FreeCAD
import Import
import Part
from FreeCAD import Vector as V

HERE = os.path.dirname(os.path.abspath(__file__))
SCAD = os.path.join(HERE, "roomba_nx.scad")
DIMS = os.path.join(HERE, "dimensions.json")
OUT = os.path.join(HERE, "cad-export")

OPENSCAD = os.environ.get("OPENSCAD") or next(
    (p for p in ("/usr/local/bin/openscad",
                 "/opt/homebrew/bin/openscad",
                 "/Applications/OpenSCAD.app/Contents/MacOS/OpenSCAD",
                 "/usr/bin/openscad") if os.path.exists(p)), "openscad")

# Everything printed.  dock_pin_block is in the list but not on the .csg route; see below.
PARTS = ["hub", "jetson_plate", "front_plate", "camera_rocker",
         "battery_cradle", "battery_lid", "pad_carrier", "dock_pin_block"]

# Volume in mm^3 of the matching hardware/stl/*.stl, as a regression tripwire.  These are the
# faceted volumes, so an exact solid is expected to land a few thousandths of a percent off.
STL_VOLUME = {"hub": 16436, "jetson_plate": 32227, "front_plate": 49468, "camera_rocker": 15908,
              "battery_cradle": 66471, "battery_lid": 26883, "pad_carrier": 9688,
              "dock_pin_block": 16612}

# Surface types a part is allowed to reach STEP as.  The point of the whole exercise is that
# nothing here is a BSplineSurface: a spline fitted through a circle is not a circle, and a CAD
# package cannot put a radius on it or offset it cleanly.  Everything in this set is a closed-form
# surface with real parameters on the other side of the import.
ANALYTIC = {"Plane", "Cylinder", "Cone", "Sphere", "Toroid", "SurfaceOfExtrusion"}


def fail(message):
    """Stop, loudly and with a non-zero status.

    FreeCADCmd swallows a SystemExit's message but honours its exit code, and prints an
    exception's message but then exits 0 regardless.  Neither is a usable failure on its own,
    so write the reason out first and exit second.
    """
    sys.stderr.write("export_step: %s\n" % message)
    sys.stderr.flush()
    raise SystemExit(1)


# ------------------------------------------------------------------ FreeCAD setup --
def configure():
    """Pin the two preferences the import and the export actually read.

    useMaxFN is importCSG's threshold for "this $fn is too fine to be worth emitting as a real
    polygon": below it a cylinder() becomes an n-sided prism, at or above it a Part::Cylinder.
    The model runs $fn = 72, so anything up to 72 keeps the analytic form; 16 is the shipped
    default and is set here so a stray value in the user's preference file cannot quietly turn
    every fillet in the export into a 72-sided prism.

    AP214IS is the STEP application protocol.  FreeCAD's default AP203 carries no colour and, more
    to the point, importers treat AP214 as the better-tested path for solids with non-elementary
    surfaces - which is exactly what dock_pin_block's lead-ins are.
    """
    FreeCAD.ParamGet("User parameter:BaseApp/Preferences/Mod/OpenSCAD").SetInt("useMaxFN", 16)
    for group in ("User parameter:BaseApp/Preferences/Mod/Import",
                  "User parameter:BaseApp/Preferences/Mod/Import/hSTEP"):
        FreeCAD.ParamGet(group).SetString("Scheme", "AP214IS")


def patch_importcsg():
    """Make importCSG recompute deep enough to survive a union of three or more children.

    Stock checkObjShape() recomputes only the node it was handed.  A Part::MultiFuse - which is
    what any union() of 3+ children becomes, and rounded_box() emits six - is still Null at that
    point, so the parent boolean reads an empty .Shape and the import either loses geometry or
    dies outright.  battery_lid does not import at all without this.  Recomputing the whole
    document instead is the blunt fix, and it is cheap: these are 30-object documents.
    """
    import importCSG

    def checkObjShape(obj):
        if isinstance(obj, (list, tuple)):
            for o in obj:
                checkObjShape(o)
            return
        if hasattr(obj, "Shape") and obj.Shape.isNull():
            obj.Document.recompute()

    importCSG.checkObjShape = checkObjShape
    return importCSG


# -------------------------------------------------------------------- the CSG route --
def bake_placement(shape):
    """Fold a shape's Placement into its own geometry, so what it says is where it is.

    Two parts are wrapped in a translate() by the part selector at the bottom of roomba_nx.scad -
    battery_lid by its skirt, camera_rocker by the pivot height - to stand them on z = 0 the way
    they print, and importCSG lands that as a Placement on the root object rather than in the
    vertices.  A Placement is a perfectly good answer right up until the exporter: FreeCAD's STEP
    writer puts it on the assembly instance instead of the geometry, and anything that reads the
    solid rather than the assembly tree then gets the part 6 or 16.5 mm out of position.  Baking
    it is also just truer to the rest of the set, where every coordinate in the file is the real
    coordinate.  BRepBuilderAPI transforms are rigid, so cylinders stay cylinders; the only cost
    is that adding and subtracting 16.5 in binary floating point turns the odd -0.01 into
    -0.009999999999998, two femtometres out and a long way inside any tolerance in this build.
    """
    if shape.Placement.isIdentity():
        return shape
    matrix = shape.Placement.toMatrix()
    local = shape.copy()
    local.Placement = FreeCAD.Placement()
    return local.transformed(matrix, copy=True)


def csg_shape(part, workdir):
    """Evaluate one part to .csg with OpenSCAD, import it, and hand back the finished Shape."""
    csg = os.path.join(workdir, part + ".csg")
    subprocess.run([OPENSCAD, "-D", 'part="%s"' % part, "-o", csg, SCAD],
                   check=True, capture_output=True)

    importCSG = patch_importcsg()
    doc = importCSG.open(csg)
    try:
        # The part we want is the one nothing else consumes.  Filtering on InList rather than
        # taking doc.Objects[0] matters: when importCSG hits a multmatrix it cannot express as a
        # Placement it falls back to Part::Feature + transformGeometry, which leaves the
        # untransformed original in the document as well - and it sorts first.  Two roots means
        # the .csg grew a second top-level body and the guess would be silent, so refuse instead.
        roots = [o for o in doc.Objects if not o.InList and hasattr(o, "Shape")]
        if len(roots) != 1:
            fail("%s: expected 1 root object, found %d (%s)"
                 % (part, len(roots), ", ".join(o.Name for o in roots)))
        return bake_placement(roots[0].Shape.copy())
    finally:
        FreeCAD.closeDocument(doc.Name)


# --------------------------------------------------- dock_pin_block, built in OCC --
def rounded_rect(x, y, r, z):
    """The wire rounded_box_c([x, y, *], r) traces: centred on the origin, flat at height z.

    Four quarter-arcs and four tangent lines, in that order round the outside.  Each arc is given
    by three points so it lands as a true Part.Arc, not an approximation.
    """
    hx, hy = x / 2 - r, y / 2 - r
    centres = [(hx, hy, 0.0), (-hx, hy, 90.0), (-hx, -hy, 180.0), (hx, -hy, 270.0)]
    edges, ends = [], []
    for cx, cy, a0 in centres:
        pt = lambda a: V(cx + r * math.cos(math.radians(a)), cy + r * math.sin(math.radians(a)), z)
        edges.append(Part.Arc(pt(a0), pt(a0 + 45), pt(a0 + 90)).toShape())
        ends.append((pt(a0), pt(a0 + 90)))
    for i in range(4):
        edges.append(Part.LineSegment(ends[i][1], ends[(i + 1) % 4][0]).toShape())
    return Part.Wire(Part.__sortEdges__(edges))


def build_dock_pin_block(d):
    """The dock-side pogo block, as an exact solid.  This is the one part not on the .csg route.

    WHY IT IS DIFFERENT.  Its 45-degree lead-ins are a hull() of two rounded boxes of different
    depth (roomba_nx.scad, dock_pin_block()).  X and the corner radius are the same in both, so
    only the corner centres move, in Y, as z rises - which makes each corner surface an OBLIQUE
    CIRCULAR CYLINDER: circular in every horizontal section, but with its axis tilted.  No
    OpenSCAD primitive produces one (a rotated cylinder() has elliptical horizontal sections; a
    scaled linear_extrude() would shrink X and the radius too), so the .scad has to say hull(),
    and importCSG answers every hull() by meshing it.  That is why this part alone used to arrive
    as 362 faces, 354 of them flat, with 132 distinct vertical-wall bearings 1.41 degrees apart -
    the tessellation step, showing through - while the other seven came in analytic.

    WHY NOT makeLoft.  Part.makeLoft() between the two wires is the obvious move and it is wrong:
    ruled and unruled both hand back six BSplineSurface faces, because a general lofter fits a
    surface through the sections instead of recognising what they are.  A spline through a circle
    is not a circle, and downstream CAD cannot put a radius dimension on it.

    WHAT IS BUILT INSTEAD.  Extruding a shape along a vector is exactly what an oblique cylinder
    is, and OCC has a surface type for it: extruding the bottom profile along the slant vector
    gives Plane faces on the flats and SurfaceOfExtrusion on the corners
    (STEP SURFACE_OF_LINEAR_EXTRUSION), all exact.  The slanted band is then the intersection of
    two such prisms - one leaning to -Y, one to +Y - because the block narrows from BOTH sides at
    once and a single prism only leans one way.  Their common volume is the rounded rectangle
    that shrinks in Y but not in X, which is precisely the hull's cross-section.

    THE 0.01 mm THAT IS NOT A ROUNDING ERROR.  The hull is taken between a slab spanning
    z 0..E and one spanning z 0..BZ, so the wide profile is still full width at z = E and the
    slope only starts there.  The lead-in therefore runs over BZ - E = 5.99 mm, not 6, and stands
    at 45.05 degrees, and the block carries a 0.01 mm vertical band round its foot.  It is a
    modelling artefact of how the hull was written, but it is IN the printed geometry and in the
    STL, so it is reproduced here rather than tidied away.  Sloping cleanly from z = 0 instead
    loses exactly 7.8 mm^3 - 0.047 %, just under the 0.05 % this export is checked to, and no
    section is more than 0.01 mm out in Y.  So the tidy version passes every dimensional check
    and is still the wrong part; volume is the only measurement that sees it at all.
    """
    bx, by, bz = d["frame"]["dock_block"]                  # [130, 28, 6]
    pin_d = d["frame"]["pin_hole_d"]
    gs = d["contact_geometry"]["group_spacing"]
    pp = d["contact_geometry"]["pin_pitch_in_group"]
    m3_d = d["print"]["m3_clearance_d"]
    m3_head_d = d["print"]["m3_head_d"]

    # Literals that are literals in the .scad too, named here so the two read alike.
    r = 3.0                       # rounded_box_c(..., 3)
    e = 0.01                      # E
    bolt_x = (-55.0, 55.0)        # for (x = [-55, 55])
    slot_w, slot_h = 4.0, 3.0     # cube([4, ..., 3]) cable exits
    cbore_h = 3.0                 # counterbore starts 3 mm below the top face
    over = 0.5                    # how far cutters poke out of the solid; any value > 0 will do

    # -- outer envelope: the 0.01 mm foot, then the lead-in band ------------------------------
    foot = Part.Face(rounded_rect(bx, by, r, 0.0)).extrude(V(0, 0, e))
    base = Part.Face(rounded_rect(bx, by, r, e))
    rise = bz - e                                          # 5.99, see the docstring
    lean = bz                                              # each side draws in by bz over that rise
    band = base.extrude(V(0, -lean, rise)).common(base.extrude(V(0, lean, rise)))
    shape = foot.fuse(band).removeSplitter()               # merges the two coplanar x = +/-65 walls

    # -- cutters, in the order roomba_nx.scad applies them -----------------------------------
    cutters = []
    for gx in (-gs / 2, gs / 2):                           # four pogo pin bores, right through
        for px in (-pp / 2, pp / 2):
            cutters.append(Part.makeCylinder(pin_d / 2, bz + 2 * over, V(gx + px, 0, -over)))

    for gx in (-gs / 2, gs / 2):
        # Cable exit: in from the -Y edge to 0.01 PAST the centreline, and stopping 0.01 SHORT of
        # slot_h. Both offsets come from the E in the .scad and both land inside the solid, so
        # they are real faces of the part and are kept exactly.
        cutters.append(Part.makeBox(slot_w, by / 2 + 2 * e, slot_h, V(gx - 2, -by / 2 - e, -e)))

    for x in bolt_x:
        cutters.append(Part.makeCylinder(m3_d / 2, bz + 2 * over, V(x, 0, -over)))
        # The counterbore is a cone that reaches m3_head_d exactly at the top face.  Continuing it
        # past z = bz along its own slope changes nothing that is cut (everything above the top
        # face is outside the solid already) and avoids asking OCC to cut with a plane that is
        # coincident with the face it is cutting, which is where slivers come from.
        z0 = bz - cbore_h - e
        h = bz + over - z0
        slope = (m3_head_d - m3_d) / 2 / (cbore_h + e)
        cutters.append(Part.makeCone(m3_d / 2, m3_d / 2 + slope * h, h, V(x, 0, z0)))

    for cutter in cutters:                                 # one at a time; more robust than a fuse
        shape = shape.cut(cutter)
    return shape.removeSplitter()


# ------------------------------------------------------------------------ reporting --
def audit(part, shape):
    """One line per part: what a downstream CAD package is actually going to see.

    The azimuth count is the facetting tell.  Every vertical planar wall is reduced to the compass
    bearing of its normal; a part drawn from primitives has a handful of them, widely spaced, while
    a meshed one has a bearing per facet at the angular step of the tessellation.
    """
    kinds, azimuths = {}, set()
    for f in shape.Faces:
        name = type(f.Surface).__name__
        kinds[name] = kinds.get(name, 0) + 1
        if name == "Plane" and abs(f.Surface.Axis.z) < 1e-7:
            azimuths.add(round(math.degrees(math.atan2(f.Surface.Axis.y, f.Surface.Axis.x)) % 360, 2))
    a = sorted(azimuths)
    gaps = [g for g in ((a[(i + 1) % len(a)] - a[i]) % 360 for i in range(len(a))) if g > 1e-3]
    bb = shape.BoundBox
    ref = STL_VOLUME[part]
    delta = (shape.Volume - ref) / ref * 100
    stray = sorted(k for k in kinds if k not in ANALYTIC)
    good = shape.isValid() and shape.isClosed() and abs(delta) < 0.05 and not stray
    print("%-15s %d solid%s %4d faces  %-44s %3d azim @ %6.2f deg  "
          "%7.2f x %6.2f x %5.2f  %10.2f mm3 %+8.4f%%  %s"
          % (part, len(shape.Solids), " " if len(shape.Solids) == 1 else "s", len(shape.Faces),
             " ".join("%s:%d" % (k.replace("Surface", "Srf"), v) for k, v in sorted(kinds.items())),
             len(a), min(gaps) if gaps else 360.0,
             bb.XLength, bb.YLength, bb.ZLength, shape.Volume, delta,
             "OK" if good else "FAIL"))
    if stray:
        print("%-15s   not analytic: %s" % ("", ", ".join(stray)))
    return good


# ------------------------------------------------------------------------------ main --
def main():
    # FreeCADCmd puts its own binary in argv[0] and this script in argv[1], and it consumes
    # anything starting with '-' for itself unless it comes after '--pass'.  So: drop everything
    # up to and including this file, then drop a leading '--pass' if there is one.
    argv = sys.argv[:]
    me = os.path.basename(__file__)
    for i, a in enumerate(argv):
        if os.path.basename(a) == me:
            argv = argv[i + 1:]
            break
    if argv and argv[0] == "--pass":
        argv = argv[1:]

    picked, out = [], OUT
    i = 0
    while i < len(argv):
        a = argv[i]
        if a.startswith("--part=") or a.startswith("--out="):
            key, value = a.split("=", 1)
            i += 1
        elif a in ("--part", "-p", "--out", "-o"):
            if i + 1 >= len(argv):
                fail("%s needs a value" % a)
            key, value = a, argv[i + 1]
            i += 2
        elif not a.startswith("-"):
            key, value = "--part", a
            i += 1
        else:
            fail("unknown option %r" % a)
        if key in ("--part", "-p"):
            picked.append(value)
        else:
            out = value
    wanted = picked or list(PARTS)
    for p in wanted:
        if p not in PARTS:
            fail("unknown part %r; known: %s" % (p, " ".join(PARTS)))

    configure()
    os.makedirs(out, exist_ok=True)
    with open(DIMS) as f:
        dims = json.load(f)

    ok = True
    with tempfile.TemporaryDirectory() as workdir:
        for part in wanted:
            if part == "dock_pin_block":
                shape = build_dock_pin_block(dims)
            else:
                shape = csg_shape(part, workdir)
            # Export through a document object and Import.export, not Shape.exportStep(): only
            # that path carries a name into STEP's PRODUCT entity, so the part arrives in
            # SolidWorks called 'hub' rather than 'Open CASCADE STEP translator 7.8 1'.
            doc = FreeCAD.newDocument(part)
            try:
                obj = doc.addObject("Part::Feature", part)
                obj.Shape = shape
                obj.Label = part
                doc.recompute()
                Import.export([obj], os.path.join(out, part + ".step"))
            finally:
                FreeCAD.closeDocument(doc.Name)
            ok = audit(part, shape) and ok

    print("\n%d STEP file%s -> %s" % (len(wanted), "" if len(wanted) == 1 else "s", out))
    if not ok:
        fail("at least one part failed its check")


main()
