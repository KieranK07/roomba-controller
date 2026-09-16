// Roomba 690 + Jetson Xavier NX + RealSense D435i  --  printed frame and fittings
//
// Every dimension comes from dimensions.json via gen.py -> dims.scad.
// Robot frame: origin at the Roomba centre on the floor, +X right, +Y forward, +Z up.
//
// CARRIER FRAME.  The three carriers are each drawn in their own local frame: origin still at
// the robot centre, but on the DECK plane (z = 0 means z = 82.94), and +X pointing outboard
// along that carrier's own axis.  assembly() just rotates each one by its layout angle.  So
// every r0 / r1 below is a radius from the robot's centre and can be read straight against the
// chassis screw-boss table, and the exported STL of, say, the battery cradle sits 62 mm off its
// own origin.  That is deliberate: the number in the file is the number on the robot.
//
// Export one part:   openscad -D 'part="hub"' -o stl/hub.stl roomba_nx.scad
// Full assembly:     openscad -D 'part="assembly"' -D explode=40 ...
//
// parts: assembly hub jetson_plate battery_cradle battery_lid front_plate camera_rocker
//        dock_pin_block pad_carrier

include <dims.scad>

part    = "assembly";
explode = 0;          // mm of separation in the assembly view
$fn     = 72;

// ---------------------------------------------------------------- shorthand --
R    = roomba_690_diameter / 2;
DECK = roomba_690_deck_height;          // the flat top the frame lies on; 92.25 is only the IR boss
Z0   = layout_chassis_clearance;        // underside of the chassis
M3   = print_m3_clearance_d;
M3T  = print_m3_tap_d;
CL   = print_fit_clearance;
E    = 0.01;

HT   = frame_hub_t;                     // hub ring thickness; the carrier tongues lie on top of it
PT   = frame_carrier_plate_t;           // carrier plate, flat on the deck
TW   = frame_carrier_tongue_w;
TR0  = frame_carrier_tongue_r0;
TR1  = frame_carrier_tongue_r1;
TT   = frame_carrier_tongue_t;
RR1  = frame_carrier_ramp_r1;           // where the ramp finishes and the deck-level plate starts
CR   = frame_carrier_corner_r;

PIV_X = frame_camera_pivot_y;           // carrier-local: outboard distance to the pivot axis
PIV_Z = frame_camera_pivot_z;           // above the deck.  DECK + this = the D435i's optical centre

// docked: dock origin at the front of its ramp, front caster up on the ramp, robot pitched nose-up
DOCK_Y     = R - home_base_ramp_d + 10;
RAMP_ANG   = atan(home_base_ramp_h / home_base_ramp_d);
CASTER_Y   = roomba_690_caster_position[1];
DOCK_PITCH = asin(((CASTER_Y - DOCK_Y) / home_base_ramp_d * home_base_ramp_h) / CASTER_Y);

// pack pocket, cradle-local: long axis along local Y
PACK_L = ovonic_3s_8000_design_l;
PACK_W = ovonic_3s_8000_design_w;
PACK_H = ovonic_3s_8000_design_h;
CRAD_W = frame_battery_outer[0];        // 52, local X
CRAD_L = frame_battery_outer[1];        // 149, local Y
CRAD_X = frame_battery_r0 + CRAD_W/2;   // 88: outboard centre of the cradle

module m3(h, d = M3) { cylinder(h = h, d = d); }

// A stadium along X, centred.  Written as waist + two end caps rather than hull() of the caps:
// the two forms are identical by construction, but FreeCAD's importCSG turns every hull() into a
// mesh, so a hulled stadium exports to STEP as facets instead of two cylinders and a box.
module slot(len, w, h) {
  for (x = [-len/2 + w/2, len/2 - w/2]) translate([x, 0, 0]) cylinder(h = h, d = w);
  translate([-(len - w)/2, -w/2, 0]) cube([len - w, w, h]);
}

module ziptie_pair(gap, len = 40) {      // two slots either side of a footprint, cut right through
  w = frame_ziptie_slot[0]; t = frame_ziptie_slot[1];
  for (s = [-1, 1]) translate([s * gap/2, 0, 0]) cube([t, w, len], center = true);
}

// A wall of thickness t whose INNER face sits at |y| = x0.  cube() only ever grows in +y, which
// silently put every mirrored feature on the wrong side of centre until this existed.
module wall_at(s, x0, t, size_x, size_z, z0 = 0) {
  translate([0, s > 0 ? x0 : -x0 - t, z0]) cube([size_x, t, size_z]);
}

// Same deal as slot(): the four corner pillars plus the two cross slabs that span between them are
// exactly the hull of those pillars, and unlike a hull() they survive the CSG -> STEP trip as four
// cylinders and two boxes.  This is the base primitive of nearly every part, so it matters most.
module rounded_box(size, r) {
  for (x = [r, size[0]-r], y = [r, size[1]-r]) translate([x, y, 0]) cylinder(h = size[2], r = r);
  translate([0, r, 0]) cube([size[0],       size[1] - 2*r, size[2]]);
  translate([r, 0, 0]) cube([size[0] - 2*r, size[1],       size[2]]);
}
module rounded_box_c(size, r) { translate([-size[0]/2, -size[1]/2, 0]) rounded_box(size, r); }

// A prism along +Y: the XZ outline `pts`, given counter-clockwise in (x, z), t deep, starting at
// y = y0.  Both ramps in this file are a hull() of two thin slabs that share a Y extent, so each is
// really a prism over a fixed polygon - all flat faces, no curvature, so an explicit polyhedron is
// EXACT, not an approximation.  Spelled as a polyhedron rather than linear_extrude(polygon()) on
// purpose: FreeCAD's importCSG turns a polyhedron straight into planar B-rep faces, whereas it
// turns linear_extrude into a parametric feature it then fails to evaluate inside a nested tree.
// Faces are wound so the right-hand normal points inward, which is what polyhedron() asks for.
module prism_y(pts, y0, t) {
  n = len(pts);
  polyhedron(points = concat([for (p = pts) [p[0], y0,     p[1]]],
                             [for (p = pts) [p[0], y0 + t, p[1]]]),
             faces  = concat([[for (i = [n - 1 : -1 : 0]) i]],                 // the y0 cap
                             [[for (i = [n : 2*n - 1]) i]],                    // the y0 + t cap
                             [for (i = [0 : n - 1]) [i, (i+1)%n, n + (i+1)%n, n + i]]));
}

// ================================================================= PRINTED ==

// --- hub: a plain ring clearing the Clean button, with three radial pads the carrier tongues
//     bolt down onto.  The nuts live in hex pockets in the pads' undersides, trapped between the
//     hub and the Roomba's shell, so no printed thread carries the joint. --
module hub() {
  od = frame_hub_od; id = frame_hub_id; pw = frame_hub_pad_w; pr = frame_hub_pad_r1;
  difference() {
    union() {
      cylinder(h = HT, d = od);
      for (a = layout_carrier_angles) rotate(a)
        translate([0, -pw/2, 0]) cube([pr, pw, HT]);
    }
    translate([0, 0, -E]) cylinder(h = HT + 2*E, d = id);
    for (a = layout_carrier_angles) rotate(a) for (r = frame_hub_hole_r) {
      translate([r, 0, -E]) m3(HT + 2*E);
      translate([r, 0, -E]) cylinder(h = frame_hub_nut_h + E, d = frame_hub_nut_af / cos(30), $fn = 6);
    }
  }
}

// --- the tongue every carrier starts with: a 4 mm strap lying on the hub pad, then a 45 degree
//     ramp down to the plate that lies flat on the deck.  That step is the whole design. --
module carrier_tongue() {
  difference() {
    union() {
      translate([TR0, -TW/2, HT]) cube([TR1 - TR0, TW, TT]);
      // Ramp, hub level -> deck level.  Was hull() of an E-wide slab at [TR1, HT..HT+TT] and one at
      // [RR1, 0..PT]; that hull is exactly this hexagon swept the tongue's width.  The two E-long
      // steps are not slop: they are the back faces of those slabs, and the hull kept them too.
      prism_y([[RR1, 0], [RR1, PT], [TR1, HT + TT], [TR1 - E, HT + TT], [TR1 - E, HT], [RR1 - E, 0]],
              -TW/2, TW);
    }
    for (r = frame_hub_hole_r) translate([r, 0, HT - E]) m3(TT + 2*E);
  }
}

// --- a diagonal ear from the plate edge out to a chassis screw boss.  t = [bx, by, ax, ay]. --
// An obround: the two end discs plus the w-wide bar between their centres, laid along the tab's own
// angle.  Same shape as hull() of the two discs, but analytic, so it exports as cylinders and a box.
// Several tabs have both ends at the same point (the ear IS the boss); those are a single disc.
module boss_tab(t) {
  w = frame_carrier_boss_tab_w;
  L = norm([t[2] - t[0], t[3] - t[1]]);          // centre distance
  difference() {
    union() {
      translate([t[0], t[1], 0]) cylinder(h = PT, d = w);
      if (L > 0) {
        translate([t[2], t[3], 0]) cylinder(h = PT, d = w);
        translate([t[0], t[1], 0]) rotate(atan2(t[3] - t[1], t[2] - t[0]))
          translate([0, -w/2, 0]) cube([L, w, PT]);
      }
    }
    translate([t[0], t[1], -E]) m3(PT + 2*E);
  }
}
module boss_holes(tabs) { for (t = tabs) translate([t[0], t[1], -E]) m3(PT + 2*E); }

// --- jetson plate: the carrier board on four tapped standoffs.  Board 100 mm axis along robot Y,
//     I/O edge facing inboard so every connector presents to the open deck. --
module jetson_plate() {
  r0 = frame_jetson_r0; r1 = frame_jetson_r1; hw = frame_jetson_half_w;
  bw = jetson_xavier_nx_devkit_board_w; bd = jetson_xavier_nx_devkit_board_d;
  bx = layout_jetson_center[0]; sh = frame_jetson_standoff_h; v = frame_jetson_vent;
  // board hole (u along the 100 mm edge, v along the 79) -> carrier-local, board turned 90 deg
  function jm(h) = [bx - (h[1] - bd/2), h[0] - bw/2];
  difference() {
    union() {
      carrier_tongue();
      translate([r0, -hw, 0]) rounded_box([r1 - r0, 2*hw, PT], 12);
      for (t = frame_jetson_boss_tabs) boss_tab(t);
      for (h = jetson_xavier_nx_devkit_mount_holes)
        translate(jm(h)) cylinder(h = PT + sh, d = print_standoff_od);
    }
    for (h = jetson_xavier_nx_devkit_mount_holes)
      translate(concat(jm(h), [PT])) cylinder(h = sh + E, d = M3T);
    boss_holes(frame_jetson_boss_tabs);
    for (s = [-1, 1]) translate([bx + s * 22, 0, -E])                     // airflow under the module
      rounded_box_c([v[0], v[1], PT + 2*E], 6);
  }
}

// --- battery cradle: open-top box cut for the HARD-case envelope so either pack drops in.
//     Long axis local Y; leads leave the inboard-front end, right under the lid-mounted BMS. --
module battery_cradle() {
  w = frame_battery_wall; fl = frame_battery_floor; hh = frame_battery_wall_h;
  PL = CRAD_L - 2*w; PW = CRAD_W - 2*w; ss = frame_battery_strap_slot;
  difference() {
    union() {
      carrier_tongue();
      translate([CRAD_X, 0, 0]) rounded_box_c([CRAD_W, CRAD_L, fl + hh], CR);
      for (t = frame_battery_boss_tabs) boss_tab(t);
      // Strap lugs.  The cradle lies flat on the deck, so there is nothing for a strap to pass
      // under and the usual floor slots are useless here; each strap goes over the lid, down the
      // outside of both walls and through one of these instead.
      for (y = [-1, 1], s = [-1, 1])
        translate([CRAD_X + s * CRAD_W/2 - (s > 0 ? 0 : 5), y * 40 - 13, 2])
          cube([5, 26, 13]);
    }
    translate([CRAD_X, 0, fl]) rounded_box_c([PW, PL, hh + E], 2);        // the pocket
    for (y = [-1, 1], s = [-1, 1])                                       // the strap slot in each lug
      translate([CRAD_X + s * (CRAD_W/2 + 3) - 5, y * 40 - ss[0]/2, 6])
        cube([10, ss[0], ss[1]]);
    translate([CRAD_X - 11, -CRAD_L/2 - E, fl + hh - 10])                 // notch for the pack leads
      cube([22, w + 2*E, 10 + E]);
    boss_holes(frame_battery_boss_tabs);
    for (y = [-45, 0, 45]) translate([CRAD_X, y, -E]) cylinder(h = fl + 2*E, d = 14);   // lightening
  }
}

// --- battery lid: lies on the pack, located by end skirts, clamped by the same two straps.
//     Carries the BMS and the INA219, which puts both boards within 40 mm of the cells. --
module battery_lid() {
  t = frame_battery_lid_t; sk = frame_battery_lid_skirt; lp = frame_battery_lid_lip;
  cw = CRAD_W + 2*(lp + CL); cl = CRAD_L + 2*(lp + CL);
  difference() {
    union() {
      translate([CRAD_X, 0, 0]) rounded_box_c([cw, cl, t], CR);
      difference() {                                                      // a lip capping the walls
        translate([CRAD_X, 0, -sk]) rounded_box_c([cw, cl, sk + E], CR);
        translate([CRAD_X, 0, -sk - E]) rounded_box_c([CRAD_W + 2*CL, CRAD_L + 2*CL, sk + 2*E], CR);
      }
    }
    translate([CRAD_X, frame_battery_bms_center_y, 0]) ziptie_pair(bms_daier_3s_w + 3);
    translate([CRAD_X, frame_battery_ina_center_y, 0]) ziptie_pair(ina219_hiletgo_w + 6);
    for (x = [-10, 10]) translate([CRAD_X + x, frame_battery_ina_center_y, -E]) m3(t + 2*E);
    for (y = [-1, 1], s = [-1, 1])                                        // straps pass through
      translate([CRAD_X + s * (CRAD_W/2 - 4 - frame_battery_wall) - frame_battery_strap_slot[1]/2,
                 y * 40 - frame_battery_strap_slot[0]/2, -E])
        cube([frame_battery_strap_slot[1], frame_battery_strap_slot[0], t + 2*E]);
    translate([CRAD_X, frame_battery_bms_center_y, -E]) cylinder(h = t + 2*E, d = 16);
    translate([CRAD_X - 11, -cl/2 - E, -sk - E]) cube([22, lp + CL + E, sk + E]);   // lead notch
  }
}

// --- front plate: DROK at the inboard end, camera cheeks at the outboard end.  Stops 3 mm short
//     of the front IR boss, and nothing on it stands in front of that boss. --
module front_plate() {
  r0 = frame_front_r0; r1 = frame_front_r1; hw = frame_front_half_w;
  cx = frame_camera_cheek_x; ct = frame_camera_cheek_t; cy = frame_camera_cheek_y;
  difference() {
    union() {
      carrier_tongue();
      translate([r0, -hw, 0]) rounded_box([r1 - r0, 2*hw, PT], 10);
      for (s = [-1, 1]) translate([cy[0], 0, 0])                          // the two cheeks
        wall_at(s, cx, ct, cy[1] - cy[0], frame_camera_cheek_top);
      // Gusset behind each cheek.  Was hull() of two E-wide slivers of wall_at at the same |y|;
      // that hull is exactly this pentagon swept the cheek's thickness, and prism_y keeps it
      // analytic.  The lone E step at the top is the back face of the tall sliver, as before.
      for (s = [-1, 1])
        prism_y([[cy[0] - 22, 0], [cy[0] + E, 0], [cy[0] + E, frame_camera_cheek_top - 8],
                 [cy[0], frame_camera_cheek_top - 8], [cy[0] - 22, PT]],
                s > 0 ? cx : -cx - ct, ct);
    }
    translate([frame_front_drok_center_y, 0, 0]) ziptie_pair(drok_buck_w + 6);
    boss_holes(frame_front_boss_tabs);
    // Pivot: two M3 stubs, one per side, NOT one long rod - the bracket is 116 mm across and
    // the hardware on hand stops at M3x30.  Clearance in the cheeks, a pilot in the ears.
    translate([PIV_X, 0, PIV_Z]) rotate([90, 0, 0]) translate([0, 0, -100]) m3(200, d = M3);
    for (a = [frame_camera_tilt_min : 1 : frame_camera_tilt_max])         // swept arc slot
      translate([PIV_X, 0, PIV_Z]) rotate([0, -a, 0])
        translate(concat([frame_camera_lock_offset[0]], [0], [frame_camera_lock_offset[1]]))
          rotate([90, 0, 0]) cylinder(h = 200, d = frame_camera_lock_d, center = true);
    translate([frame_front_drok_center_y, 0, -E]) cylinder(h = PT + 2*E, d = 16);
  }
}

// --- camera rocker: the D435i bolts on top, the pivot runs THROUGH its optical centre, so
//     tilting the camera does not move it.  Drawn about the pivot, in the front plate's frame. --
module camera_rocker() {
  p = frame_camera_rocker; ex = frame_camera_ear_x; et = frame_camera_ear_t;
  zt = frame_camera_ear_top - PIV_Z;                       // ear top, relative to the pivot
  zb = -realsense_d435i_h/2;                               // plate top = camera underside
  lo = frame_camera_lock_offset;
  difference() {
    union() {
      translate([0, 0, zb - p[2]]) rounded_box_c([p[1], p[0], p[2]], 3);
      for (s = [-1, 1]) translate([-p[1]/2, 0, 0])
        wall_at(s, ex, et, p[1], zt - (zb - p[2]), zb - p[2]);
    }
    rotate([90, 0, 0]) translate([0, 0, -100]) m3(200, d = M3T, $fn = 40);   // pivot, through both ears
    // Clamp bolt, bored through each EAR only.  A full-width bore would run the length of the
    // 88 mm plate 2 mm from its rear edge, through the middle of its 4 mm thickness.
    for (s = [-1, 1]) translate([lo[0], s * (ex + et/2), lo[1]]) rotate([90, 0, 0])
      cylinder(h = et + 2, d = M3, center = true);
    for (s = [-1, 1]) translate([lo[0], s * (ex + et), lo[1]]) rotate([90, 0, 0])
      cylinder(h = frame_hub_nut_h * 2, d = frame_hub_nut_af / cos(30), center = true, $fn = 6);
    for (y = [-1, 1]) translate([0, y * realsense_d435i_mount_m3_spacing/2, zb - p[2] - E])
      m3(p[2] + 2*E);
    translate([0, 0, zb - p[2] - E]) cylinder(h = p[2] + 2*E, d = frame_camera_tripod_clearance_d);
    translate([-p[1]/2 + 4, -28, zb - p[2] - E]) rotate(90) slot(16, 4, p[2] + 2*E);  // USB tail-down
  }
}

// --- dock pin block: four P75-B1 pins at the build-sheet geometry, wire channels --
module dock_pin_block() {
  b = frame_dock_block; gs = contact_geometry_group_spacing; pp = contact_geometry_pin_pitch_in_group;
  difference() {
    // 45-degree lead-ins.  This one stays a hull(), on purpose.  It is a loft between two rounded
    // rectangles of different depth: X and the corner radius are constant, but each corner's centre
    // slides in Y as z rises, so the corner surfaces are OBLIQUE cylinders.  No OpenSCAD primitive
    // makes one (a tilted cylinder has elliptical horizontal sections, not circular; a scaled
    // linear_extrude would shrink X and the radius too), and a sheared multmatrix is silently
    // dropped by FreeCAD's importCSG.  So this is the one part whose STEP is NOT imported from
    // this file: export_step.py builds it directly in OpenCASCADE, where an oblique cylinder is a
    // surface of linear extrusion and comes out exact.  The two are checked against each other to
    // 0.003 mm, which is just this file's $fn = 72 corners.  This hull() remains the source of
    // truth for the printed part, so change it here and the STEP follows - but read the note in
    // export_step.py first, because the E-thick lower slab is load-bearing: it holds the outline
    // at full width up to z = E, so the lead-in rises over b[2] - E and is 45.05 deg, not 45.
    hull() {
      rounded_box_c([b[0], b[1], E], 3);
      rounded_box_c([b[0], b[1] - 2*b[2], b[2]], 3);
    }
    for (gx = [-gs/2, gs/2], px = [-pp/2, pp/2])
      translate([gx + px, 0, -E]) cylinder(h = b[2] + 2*E, d = frame_pin_hole_d);
    for (gx = [-gs/2, gs/2]) translate([gx - 2, -b[1]/2 - E, -E]) cube([4, b[1]/2 + 2*E, 3]);
    for (x = [-55, 55]) translate([x, 0, -E]) {
      m3(b[2] + 2*E);
      translate([0, 0, b[2] - 3]) cylinder(h = 3 + E, d1 = M3, d2 = print_m3_head_d);
    }
  }
}

// --- pad carrier: two copper-clad pads recessed so they stand 0.4 mm proud --
module pad_carrier() {
  c = frame_pad_carrier; gs = contact_geometry_group_spacing;
  pw = contact_geometry_pad_w + 2*CL; pd = contact_geometry_pad_d + 2*CL;
  difference() {
    rounded_box_c(c, 3);
    for (x = [-gs/2, gs/2]) {
      translate([x - pw/2, -pd/2, c[2] - frame_pad_pocket_depth]) cube([pw, pd, frame_pad_pocket_depth + E]);
      translate([x - 1.5, -c[1]/2 - E, c[2] - frame_pad_pocket_depth - 2]) cube([3, c[1]/2, 4]);
    }
    for (x = [-55, 55]) translate([x, 0, -E]) m3(c[2] + 2*E);
  }
}

// =============================================================== REFERENCE ==
// Bought parts at catalogue dimensions, so fit and clearance can be judged.

module ref_roomba() {
  color("#B9BFC4", 0.55) import("mesh/roomba_690_body.stl");
  color("#E8E8E8") translate([0, 0, DECK - 0.5]) cylinder(h = 1.5, d = roomba_690_clean_button_diameter - 2);
  for (b = roomba_690_screw_bosses) color("#8C5A2A") translate([b[0], b[1], DECK]) cylinder(h = 1, d = 4.4);
}

module ref_jetson() {
  color("#1E6B3A") import("mesh/jetson_devkit_board.stl");
  color("#2A2E33") import("mesh/jetson_devkit_cooler.stl");
}

// --- Ovonic 3S soft pack, at the ON-HAND soft-case figures.  The cradle is cut for the larger
//     hard-case envelope, so what you see here is the pack with its end play. --
module ref_battery() {
  L = ovonic_3s_8000_l; W = ovonic_3s_8000_w; H = ovonic_3s_8000_h;
  r = ovonic_3s_8000_corner_r;
  color("#2C5BB5") hull() for (x = [-1, 1], y = [-1, 1], z = [0, 1])
    translate([x * (L/2 - r), y * (W/2 - r), r + z * (H - 2*r)]) sphere(r = r, $fn = 40);
}
module ref_camera() { color("#D9DBDD") import("mesh/realsense_d435i.stl"); }

module ref_drok() {
  L = drok_buck_l; W = drok_buck_w; H = drok_buck_h; pcb = 1.6;
  color("#3A3F45") translate([-W/2, -L/2, 0]) cube([W, L, pcb]);
  color("#1A1D20") translate([-W/2 + 3, -L/2 + 6, pcb]) cube([W - 6, 26, H - pcb]);
  color("#B03030") for (y = [-6, 6]) translate([0, y + 22, pcb]) cylinder(h = 7, d = 9.5);
  color("#2A2E33") for (y = [-L/2 + 4, L/2 - 4])
    translate([-W/2 + 5, y - 4, pcb]) cube([W - 10, 8, 10]);
}

module ref_bms() {
  L = bms_daier_3s_l; W = bms_daier_3s_w; T = bms_daier_3s_t;
  module at(px, py) translate([py - W/2, px - L/2, T]) children();
  color("#1E6B3A") translate([-W/2, -L/2, 0]) cube([W, L, T]);
  color("#C8A24A") intersection() {
    translate([-W/2, -L/2, T - E]) cube([W, L, 0.06 + E]);
    union() {
      for (q = [bms_daier_3s_pads_Bminus, bms_daier_3s_pads_B1,
                bms_daier_3s_pads_Bplus,  bms_daier_3s_pads_B2])
        at(q[0], q[1]) linear_extrude(0.06) square([q[3], q[2]], center = true);
      for (q = [bms_daier_3s_pads_round_Pplus, bms_daier_3s_pads_round_Pminus])
        at(q[0], q[1]) cylinder(h = 0.06, d = q[2]);
    }
  }
  mb = bms_daier_3s_mosfet_bank;
  color("#2A2E33") at(mb[0], mb[1]) translate([-mb[3]/2, -mb[2]/2, 0]) cube([mb[3], mb[2], mb[4]]);
  ir = bms_daier_3s_ic_row;
  color("#2A2E33") at(ir[0], ir[1]) translate([-ir[3]/2, -ir[2]/2, 0]) cube([ir[3], ir[2], ir[4]]);
}
module ref_ina() { color("#6A2FB5") import("mesh/ina219.stl"); }

module ref_dock() {
  w = home_base_w; d = home_base_d; h = home_base_h; rh = home_base_ramp_h; rd = home_base_ramp_d;
  color("#4A4F55", 0.8) {
    translate([-w/2, 0, 0]) rotate([90, 0, 90]) linear_extrude(w) polygon([[0, 0], [rd, 0], [rd, rh]]);
    translate([-w/2, rd, 0]) cube([w, d - rd, h]);
  }
  for (s = [-1, 1]) color("#C8A24A") translate([s * home_base_stock_contact_spacing/2 - 8, rd - 45, (rd - 45)/rd*rh])
    rotate([RAMP_ANG, 0, 0]) cube([16, 40, 0.5]);
}

// ================================================================ ASSEMBLY ==
module assembly() {
  ex = explode;
  rotate([DOCK_PITCH, 0, 0]) {                          // nose-up: the front caster is on the ramp
  ref_roomba();
  AJ = layout_carrier_angles[0];                        // 0   right, Jetson
  AC = layout_carrier_angles[1];                        // 90  front, camera + DROK
  AB = layout_carrier_angles[2];                        // 180 left,  battery
  PZ = DECK + PT;                                       // top of any carrier plate

  color("#E9A055") translate([0, 0, DECK + ex]) hub();

  // ---- right: Jetson -------------------------------------------------------------------
  rotate(AJ) {
    color("#E9A055") translate([0, 0, DECK + ex * 2]) jetson_plate();
    translate([layout_jetson_center[0], 0, PZ + frame_jetson_standoff_h + ex * 3])
      rotate(layout_jetson_rotation) ref_jetson();
  }

  // ---- left: battery, lid, BMS, INA219 -------------------------------------------------
  rotate(AB) {
    color("#E9A055") translate([0, 0, DECK + ex * 2]) battery_cradle();
    translate([CRAD_X, 0, DECK + frame_battery_floor + ex * 3]) rotate(90) ref_battery();
    translate([0, 0, DECK + frame_battery_floor + PACK_H + ex * 4]) {
      color("#F0B06A") battery_lid();
      translate([CRAD_X, frame_battery_bms_center_y, frame_battery_lid_t + ex]) ref_bms();
      translate([CRAD_X, frame_battery_ina_center_y, frame_battery_lid_t + ex]) ref_ina();
    }
  }

  // ---- front: DROK, then the camera on its rocker --------------------------------------
  rotate(AC) {
    color("#E9A055") translate([0, 0, DECK + ex * 2]) front_plate();
    translate([frame_front_drok_center_y, 0, PZ + ex * 3]) ref_drok();
    translate([PIV_X, 0, DECK + PIV_Z + ex * 3]) rotate([0, -layout_camera_tilt_deg, 0]) {
      color("#F0B06A") camera_rocker();
      rotate([0, 0, -90]) ref_camera();
    }
  }

  color("#E9A055") translate([0, layout_pads_center_y, Z0 - ex]) rotate([0, 180, 0]) pad_carrier();
  }

  translate([0, DOCK_Y, -ex * 2]) {                                         // docked position
    ref_dock();
    py = layout_pads_center_y - DOCK_Y;
    color("#E9A055") translate([0, py, py / home_base_ramp_d * home_base_ramp_h]) rotate([RAMP_ANG, 0, 0]) dock_pin_block();
  }
}

// ================================================================= DISPATCH ==
if      (part == "assembly")         assembly();
else if (part == "hub")              hub();
else if (part == "jetson_plate")     jetson_plate();
else if (part == "battery_cradle")   battery_cradle();
else if (part == "battery_lid")      translate([0, 0, frame_battery_lid_skirt]) battery_lid();
else if (part == "front_plate")      front_plate();
else if (part == "camera_rocker")    translate([0, 0, realsense_d435i_h/2 + frame_camera_rocker[2]]) camera_rocker();
else if (part == "dock_pin_block")   dock_pin_block();
else if (part == "pad_carrier")      pad_carrier();
else if (part == "ref_battery")      ref_battery();
else if (part == "ref_bms")          ref_bms();
else if (part == "ref_drok")         ref_drok();
else echo(str("unknown part: ", part));
