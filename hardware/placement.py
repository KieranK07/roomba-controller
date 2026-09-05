#!/usr/bin/env python3
"""Where every part sits on the robot, as 4x4 transforms.  One table, two consumers.

assembly() in roomba_nx.scad is the authority on placement, but it is OpenSCAD: nothing outside
OpenSCAD can read it.  This mirrors it in numpy so verify.py can measure the placed geometry and
export_assembly.py can bake it into world-frame STLs without either re-deriving the numbers, or
drifting away from the other.  Every transform below is the same product of rotations and
translations assembly() applies, in the same order.

Frame: dimensions.json's _frame - origin at the Roomba's centre ON THE FLOOR, +X right, +Y forward,
+Z up.  assembly() additionally spins the robot by DOCK_PITCH so it stands nose-up on the Home Base
ramp.  That is a pose, not geometry: it is deliberately not reproduced here, so the robot this
module describes is LEVEL and the numbers can be read straight off a ruler.
"""
import json
import math
import os

import numpy as np
import trimesh

HERE = os.path.dirname(os.path.abspath(__file__))
D = json.load(open(os.path.join(HERE, "dimensions.json")))
R690, F, L = D["roomba_690"], D["frame"], D["layout"]
DECK = R690["deck_height"]
AJ, AC, AB = L["carrier_angles"]
PT = F["carrier"]["plate_t"]
PZ = DECK + PT
CRAD_X = F["battery"]["r0"] + F["battery"]["outer"][0] / 2
PIVX, PIVZ = F["camera"]["pivot_y"], DECK + F["camera"]["pivot_z"]
PACK_TOP = DECK + F["battery"]["floor"] + D["ovonic_3s_8000"]["design_h"]
LID_TOP = PACK_TOP + F["battery"]["lid_t"]
Z0 = L["chassis_clearance"]                       # underside of the chassis


def rz(a):
    a = math.radians(a)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])


def tr(x, y, z):
    T = np.eye(4)
    T[:3, 3] = [x, y, z]
    return T


def ry(a):
    return trimesh.transformations.rotation_matrix(math.radians(a), [0, 1, 0])


def load(sub, name, *M):
    m = trimesh.load(os.path.join(HERE, sub, name + ".stl"))
    T = np.eye(4)
    for X in M:
        T = T @ X
    m.apply_transform(T)
    return m


TILT = ry(-L["camera_tilt_deg"])
CAM = rz(AC) @ tr(PIVX, 0, PIVZ) @ TILT           # the rocker's frame, tilt included

# name -> (source dir, STL stem, transforms applied left to right).  The dir says which frame the
# file is in: stl/ is print-ready part-local, mesh/ is vendor-local.
PRINTED = {
    "hub":            ("stl", "hub", [tr(0, 0, DECK)]),
    "jetson_plate":   ("stl", "jetson_plate", [rz(AJ), tr(0, 0, DECK)]),
    "battery_cradle": ("stl", "battery_cradle", [rz(AB), tr(0, 0, DECK)]),
    # the lid STL is exported lifted by its lip so it prints flat; undo that, then cap the walls
    "battery_lid":    ("stl", "battery_lid", [rz(AB), tr(0, 0, PACK_TOP - F["battery"]["lid_skirt"])]),
    "front_plate":    ("stl", "front_plate", [rz(AC), tr(0, 0, DECK)]),
    "camera_rocker":  ("stl", "camera_rocker",
                       [CAM, tr(0, 0, -(D["realsense_d435i"]["h"] / 2 + F["camera"]["rocker"][2]))]),
    # underside: it prints pockets-up and installs pockets-down, hence the flip
    "pad_carrier":    ("stl", "pad_carrier", [tr(0, L["pads_center_y"], Z0), ry(180)]),
    # dock-side half of the contact pair.  assembly() bolts it to the Home Base ramp; with the dock
    # gone it keeps its plan position and its ramp tilt is dropped, so it stands flat on the floor
    # directly under the pads it mates with.  See export_assembly.py for the reasoning.
    "dock_pin_block": ("stl", "dock_pin_block", [tr(0, L["pads_center_y"], 0)]),
}
BOUGHT = {
    "jetson":  ("mesh", "jetson_devkit_board",
                [rz(AJ), tr(L["jetson_center"][0], 0, PZ + F["jetson"]["standoff_h"]), rz(L["jetson_rotation"])]),
    "cooler":  ("mesh", "jetson_devkit_cooler",
                [rz(AJ), tr(L["jetson_center"][0], 0, PZ + F["jetson"]["standoff_h"]), rz(L["jetson_rotation"])]),
    "pack":    ("mesh", "lipo_3s", [rz(AB), tr(CRAD_X, 0, DECK + F["battery"]["floor"]), rz(90)]),
    "drok":    ("mesh", "drok", [rz(AC), tr(F["front"]["drok_center_y"], 0, PZ), rz(L["drok_rotation"])]),
    "bms":     ("mesh", "bms", [rz(AB), tr(CRAD_X, F["battery"]["bms_center_y"], LID_TOP)]),
    "ina219":  ("mesh", "ina219", [rz(AB), tr(CRAD_X, F["battery"]["ina_center_y"], LID_TOP)]),
    "camera":  ("mesh", "realsense_d435i", [CAM, rz(-90)]),
}
# The stock robot.  It IS the frame - modelled centred on the origin, standing on z = 0 - so its
# transform is the identity and everything above is measured against it.
SHELL = {"roomba_shell": ("mesh", "roomba_690_body", [])}

PLACED = {**PRINTED, **BOUGHT, **SHELL}

# What verify.py measures: the parts that are bolted to the robot and can foul each other.  The
# shell is the datum rather than a payload, and the two contact parts (pad_carrier, dock_pin_block)
# are held out because their fit is a robot-to-DOCK question that dimensions.json flags _BLOCKED
# pending a measurement of the real ramp - verify.py has no dock to check them against.
CHECKED_PRINTED = ["hub", "jetson_plate", "battery_cradle", "battery_lid", "front_plate", "camera_rocker"]
CHECKED_BOUGHT = list(BOUGHT)


def matrix(name):
    """The single 4x4 taking `name` from its own file's frame into the robot frame."""
    T = np.eye(4)
    for X in PLACED[name][2]:
        T = T @ X
    return T


def placed(name):
    """The part's mesh, loaded from disk and moved into the robot frame."""
    sub, stem, M = PLACED[name]
    return load(sub, stem, *M)
