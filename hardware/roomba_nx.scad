// Roomba 690 + Jetson Xavier NX + RealSense D435i  --  printed frame and fittings
//
// Every dimension comes from dimensions.json via gen.py -> dims.scad.
// Robot frame: origin at the Roomba centre on the floor, +X right, +Y forward, +Z up.
//
// WHERE THE GEOMETRY COMES FROM.  The top deck is MEASURED (deck_measured: Kieran's CAD person
// modelled his own 690, faceplate off).  The deck is flat out to r 135.26, a 3.3 mm ring runs
// out to r 144.42, and past that is the bumper, which moves.  The frame fastens to the robot at
// ONE place - Kieran's hub, reproduced exactly from that CAD - and every carrier bolts to the hub
// and nothing else.  No chassis screw bosses, no Create 2 CAD placement.
//
// CARRIER FRAME.  The three carriers are each drawn in their own local frame: origin still at
// the robot centre, but on the DECK plane (z = 0 means z = 82.94), and +X pointing outboard
// along that carrier's own axis.  assembly() just rotates each one by its layout angle.  So
// every r0 / r1 below is a radius from the robot's centre and can be read straight against
// deck_measured.flat_r and the limits, and the exported STL of, say, the battery cradle sits
// 56 mm off its own origin.  That is deliberate: the number in the file is the number on the robot.
//
// LEGAL ENVELOPE.  Every carrier is intersected with legal_envelope() - deck_measured.limits
// shrunk by frame.envelope_margin - so whatever the plates are drawn as, nothing that rests on
// the deck passes the flat, nothing passes the ring low enough to touch it, and nothing passes
// the bumper line.  The rocker tilts, so it cannot be clipped; it is sized by hand instead and
// checked over its whole tilt range.
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
DECK = roomba_690_deck_height;          // the flat top the frame lies on
Z0   = layout_chassis_clearance;        // underside of the chassis
M3   = print_m3_clearance_d;
M3T  = print_m3_tap_d;
CSK  = print_m3_csk_d;                  // countersink for an M3 flat head
CL   = print_fit_clearance;
E    = 0.01;

HT   = deck_measured_hub_top_h;         // Kieran's hub plate: the carrier tongues lie on top of it
PT   = frame_carrier_plate_t;           // carrier plate, flat on the deck
TW   = frame_carrier_tongue_w;
TR0  = frame_carrier_tongue_r0;
TT   = frame_carrier_tongue_t;
CR   = frame_carrier_corner_r;
PAD_SIDE  = deck_measured_hub_side_pad_r1;     // where each tongue ends: the end of its hub pad
PAD_FRONT = deck_measured_hub_front_pad_r1;
function ramp_end(pad_r1) = pad_r1 + HT;       // 45 deg ramp, down the hub's own thickness

assert(layout_carrier_angles == deck_measured_hub_pad_angles, "carriers must sit on the hub's pads");
assert(abs(frame_jetson_r0  - ramp_end(PAD_SIDE))  < 1e-3, "jetson r0 must be the side ramp's end");
assert(abs(frame_battery_r0 - ramp_end(PAD_SIDE))  < 1e-3, "battery r0 must be the side ramp's end");
assert(abs(frame_front_r0   - ramp_end(PAD_FRONT)) < 1e-3, "front r0 must be the front ramp's end");

PIV_X = frame_camera_pivot_y;           // carrier-local: outboard distance to the pivot axis
PIV_Z = frame_camera_pivot_z;           // above the deck.  DECK + this = the D435i's optical centre

// docked: dock origin at the front of its ramp, front caster up on the ramp, robot pitched nose-up
DOCK_Y     = R - home_base_ramp_d + 10;
RAMP_ANG   = atan(home_base_ramp_h / home_base_ramp_d);
CASTER_Y   = roomba_690_caster_position[1];
DOCK_PITCH = asin(((CASTER_Y - DOCK_Y) / home_base_ramp_d * home_base_ramp_h) / CASTER_Y);

// pack pocket, cradle-local: long axis along local Y.  Kieran's padded pack, cut to exactly.
PACK_L = ovonic_3s_8000_design_l;       // 147.32, local Y
PACK_W = ovonic_3s_8000_design_w;       // 34.65, across the cradle (local X)
PACK_H = ovonic_3s_8000_design_h;       // 42.33, tall
CRAD_W = frame_battery_outer[0];        // 40.65, local X
CRAD_L = frame_battery_outer[1];        // 153.32, local Y
CRAD_X = frame_battery_r0 + CRAD_W/2;   // 76.44: outboard centre of the cradle
assert(abs(CRAD_W - (PACK_W + 2*frame_battery_wall)) < 1e-3 && abs(CRAD_L - (PACK_L + 2*frame_battery_wall)) < 1e-3,
       "frame.battery.outer must be the pack plus two walls");
assert(abs(frame_battery_wall_h - PACK_H) < 1e-3, "the walls stand exactly as tall as the pack");

// the limits, shrunk by the margin (deck-plane heights)
ENV_ON = deck_measured_limits_on_deck_r_max - frame_envelope_margin;          // resting on the flat
ENV_Z  = deck_measured_limits_over_ring_underside_min + frame_envelope_margin; // lowest a part may be past ENV_ON
ENV_F  = deck_measured_limits_front_r_max - frame_envelope_margin;           // y >= 0
ENV_R  = deck_measured_limits_rear_r_max  - frame_envelope_margin;           // y <  0

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

// --- the legal envelope, robot frame, deck plane at z = 0.  Out to ENV_ON a part may rest on the
//     flat.  Past it the part has to be at least ENV_Z up (clear of the 3.3 mm ring), and from
//     there a 45 deg cone widens it out to the front or rear radius limit - the cone rather than
//     a step so a clipped part never has a flat overhang to print.  Cylinders and cones only, so
//     it survives the CSG -> STEP trip as analytic faces like everything else. --
module envelope_half(rmax) {
  cylinder(h = 300, r = ENV_ON, $fn = 360);
  translate([0, 0, ENV_Z]) cylinder(h = rmax - ENV_ON, r1 = ENV_ON, r2 = rmax, $fn = 360);
  translate([0, 0, ENV_Z + rmax - ENV_ON]) cylinder(h = 300, r = rmax, $fn = 360);
}
module legal_envelope() {
  intersection() { envelope_half(ENV_F); translate([-400, 0,    -1]) cube([800, 400, 400]); }
  intersection() { envelope_half(ENV_R); translate([-400, -400, -1]) cube([800, 400, 400]); }
}
// A carrier drawn in its local frame, clipped to the envelope turned into that same frame.
module clip_to_envelope(a) { intersection() { children(); rotate(-a) legal_envelope(); } }

// --- hub: Kieran's part, reproduced exactly from roomba_top_deck.step - do not redesign it.  A
//     3.26 mm plate standing on the deck: a ring clear of the Clean button and three flat-ended
//     pads the carrier tongues lie on, two holes each.  The four diagonal holes are where it is
//     screwed to the Roomba; their positions are an ESTIMATE until his CAD models them.  No nut
//     pockets: a 3.26 mm plate on the deck has no room for one. --
module hub() {
  h = deck_measured_hub_top_h;
  difference() {
    union() {
      cylinder(h = h, d = deck_measured_hub_od);
      for (a = deck_measured_hub_pad_angles) rotate(a) {
        w  = a == 90 ? deck_measured_hub_front_pad_w  : deck_measured_hub_side_pad_w;
        r1 = a == 90 ? deck_measured_hub_front_pad_r1 : deck_measured_hub_side_pad_r1;
        translate([0, -w/2, 0]) cube([r1, w, h]);
      }
    }
    translate([0, 0, -E]) cylinder(h = h + 2*E, d = deck_measured_hub_id);
    for (a = deck_measured_hub_pad_angles, r = deck_measured_hub_hole_r)
      rotate(a) translate([r, 0, -E]) cylinder(h = h + 2*E, d = deck_measured_hub_hole_d);
    for (a = deck_measured_hub_mount_holes_angles)
      rotate(a) translate([deck_measured_hub_mount_holes_r, 0, -E])
        cylinder(h = h + 2*E, d = deck_measured_hub_mount_hole_d);
  }
}

// --- the tongue every carrier starts with: a 4 mm strap lying on its hub pad out to the pad's
//     end, then a 45 degree ramp down the hub's 3.26 mm to the plate lying flat on the deck.  The
//     two bolts are countersunk flat heads so the tongue's top stays flush at deck + 7.26 - the
//     Jetson's I/O overhang passes 1.2 mm over the r 49 one. --
module carrier_tongue(pad_r1) {
  rr1 = ramp_end(pad_r1);
  cs  = (CSK - M3) / 2;                                  // depth of a 90 deg countersink
  difference() {
    union() {
      translate([TR0, -TW/2, HT]) cube([pad_r1 - TR0, TW, TT]);
      // Ramp, hub level -> deck level: the hexagon hull() of an E-wide slab at [pad_r1, HT..HT+TT]
      // and one at [rr1, 0..PT] would make, swept the tongue's width.  The two E-long steps are
      // the back faces of those slabs.
      prism_y([[rr1, 0], [rr1, PT], [pad_r1, HT + TT], [pad_r1 - E, HT + TT], [pad_r1 - E, HT], [rr1 - E, 0]],
              -TW/2, TW);
    }
    for (r = deck_measured_hub_hole_r) translate([r, 0, HT - E]) {
      m3(TT + 2*E);
      translate([0, 0, TT + E - cs]) cylinder(h = cs + E, d1 = M3, d2 = CSK);
    }
  }
}

// --- jetson plate: the carrier board on four tapped standoffs.  Board 100 mm axis along robot Y,
//     I/O edge facing inboard so every connector presents to the open deck.  The board is as far
//     outboard as the front limit lets its corner go; that puts the I/O overhang over the tongue,
//     hence 9 mm standoffs.  Hangs off the hub's two bolts and rests on the deck. --
module jetson_plate() {
  r0 = frame_jetson_r0; r1 = frame_jetson_r1; hw = frame_jetson_half_w;
  bw = jetson_xavier_nx_devkit_board_w; bd = jetson_xavier_nx_devkit_board_d;
  bx = layout_jetson_center[0]; sh = frame_jetson_standoff_h; v = frame_jetson_vent;
  // board hole (u along the 100 mm edge, v along the 79) -> carrier-local, board turned 90 deg
  function jm(h) = [bx - (h[1] - bd/2), h[0] - bw/2];
  clip_to_envelope(layout_carrier_angles[0]) difference() {
    union() {
      carrier_tongue(PAD_SIDE);
      translate([r0, -hw, 0]) rounded_box([r1 - r0, 2*hw, PT], 12);
      for (h = jetson_xavier_nx_devkit_mount_holes)
        translate(jm(h)) cylinder(h = PT + sh, d = print_standoff_od);
    }
    for (h = jetson_xavier_nx_devkit_mount_holes)
      translate(concat(jm(h), [PT])) cylinder(h = sh + E, d = M3T);
    for (x = frame_jetson_vent_x) translate([bx + x, 0, -E])                // airflow under the module
      rounded_box_c([v[0], v[1], PT + 2*E], 6);
  }
}

// --- battery cradle: open-top box cut to exactly Kieran's padded pack, standing on its side.
//     Long axis local Y; leads leave the inboard-front end, right under the lid-mounted BMS. --
module battery_cradle() {
  w = frame_battery_wall; fl = frame_battery_floor; hh = frame_battery_wall_h;
  ss = frame_battery_strap_slot; sy = frame_battery_strap_y;
  lt = 8;                                                // lug thickness, outboard of the wall
  difference() {
    union() {
      carrier_tongue(PAD_SIDE);
      translate([CRAD_X, 0, 0]) rounded_box_c([CRAD_W, CRAD_L, fl + hh], CR);
      // Strap lugs, one per strap per side.  The cradle lies flat on the deck, so a strap cannot
      // pass under it; each strap comes over the lid, down the outside of the wall and through the
      // vertical slot in one of these, round under its outer bar (3 mm off the deck) and back up.
      for (y = [-1, 1], s = [-1, 1])
        translate([CRAD_X + s * CRAD_W/2 - (s > 0 ? 0 : lt), y * sy - 13, 3]) cube([lt, 26, 12]);
    }
    translate([CRAD_X, 0, fl]) rounded_box_c([PACK_W, PACK_L, hh + E], frame_battery_pocket_r);  // the pocket
    for (y = [-1, 1], s = [-1, 1])                                       // the strap slot in each lug
      translate([CRAD_X + s * (CRAD_W/2 + 3 + ss[1]/2) - ss[1]/2, y * sy - ss[0]/2, 0])
        cube([ss[1], ss[0], 20]);
    translate([CRAD_X - 11, -CRAD_L/2 - E, fl + hh - 10])                 // notch for the pack leads
      cube([22, w + 2*E, 10 + E]);
    for (y = [-45, 0, 45]) translate([CRAD_X, y, -E]) cylinder(h = fl + 2*E, d = 14);   // lightening
  }
}

// --- battery lid: lies on the pack and the walls, located by a skirt round the wall tops, clamped
//     by the two straps.  Its top is wider than the skirt: the 45 mm BMS and its ties need 58, and
//     the overhang carries the strap slots just outside the skirt.  Carries the BMS and the INA219
//     between the straps, which keeps both boards within 40 mm of the cells. --
module battery_lid() {
  t = frame_battery_lid_t; sk = frame_battery_lid_skirt; lp = frame_battery_lid_lip;
  ss = frame_battery_strap_slot; sy = frame_battery_strap_y;
  cw = CRAD_W + 2*(lp + CL); cl = CRAD_L + 2*(lp + CL); lw = frame_battery_lid_w;
  difference() {
    union() {
      translate([CRAD_X, 0, 0]) rounded_box_c([lw, cl, t], CR);
      difference() {                                                      // the skirt capping the walls
        translate([CRAD_X, 0, -sk]) rounded_box_c([cw, cl, sk + E], CR);
        translate([CRAD_X, 0, -sk - E]) rounded_box_c([CRAD_W + 2*CL, CRAD_L + 2*CL, sk + 2*E], CR);
      }
    }
    translate([CRAD_X, frame_battery_bms_center_y, 0]) ziptie_pair(bms_daier_3s_w + 3);
    translate([CRAD_X, frame_battery_ina_center_y, 0]) ziptie_pair(ina219_hiletgo_w + 6);
    for (x = [-10, 10]) translate([CRAD_X + x, frame_battery_ina_center_y, -E]) m3(t + 2*E);
    for (y = [-1, 1], s = [-1, 1])                                        // straps pass through
      translate([CRAD_X + s * (cw/2 + 0.6 + ss[1]/2) - ss[1]/2, y * sy - ss[0]/2, -E])
        cube([ss[1], ss[0], t + 2*E]);
    translate([CRAD_X, frame_battery_bms_center_y, -E]) cylinder(h = t + 2*E, d = 16);
    translate([CRAD_X - 11, -cl/2 - E, -sk - E]) cube([22, lp + CL + E, sk + E]);   // lead notch
  }
}

// --- front plate: DROK at the inboard end, camera cheeks at the outboard end.  The cheeks' lower
//     front corners are where the envelope bites: they stand on the flat and lean out over the
//     ring above ENV_Z, which is where the pivot is. --
module front_plate() {
  r0 = frame_front_r0; hw = frame_front_half_w;
  cx = frame_camera_cheek_x; ct = frame_camera_cheek_t; cy = frame_camera_cheek_y;
  r1 = cy[1];
  clip_to_envelope(layout_carrier_angles[1]) difference() {
    union() {
      carrier_tongue(PAD_FRONT);
      // narrow inboard, where the battery cradle's front end is; full width from the gussets out
      translate([r0, -frame_front_half_w_inboard, 0]) rounded_box([r1 - r0, 2*frame_front_half_w_inboard, PT], 10);
      translate([cy[0] - 22, -hw, 0]) rounded_box([r1 - cy[0] + 22, 2*hw, PT], 10);
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
    // DROK ties run round its 65.6 mm length, so the slots sit off its ends (robot +/-x) rather
    // than fore and aft, where the inboard one would cut the tongue's ramp.
    translate([frame_front_drok_center_y, 0, 0]) rotate(90) ziptie_pair(drok_buck_l + 6);
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
//     tilting the camera does not move it.  Drawn about the pivot, in the front plate's frame.
//     Ahead of the pivot each ear is a disc about the pivot, so tilting cannot swing an ear corner
//     forward into the bumper line; behind it the ear stays square to hold the clamp bolt. --
module camera_rocker() {
  p = frame_camera_rocker; ex = frame_camera_ear_x; et = frame_camera_ear_t;
  zt = frame_camera_ear_top - PIV_Z;                       // ear top, relative to the pivot
  zb = -realsense_d435i_h/2;                               // plate top = camera underside
  lo = frame_camera_lock_offset; nr = frame_camera_ear_nose_r;
  eh = zt - (zb - p[2]);                                   // ear height
  difference() {
    union() {
      translate([0, 0, zb - p[2]]) rounded_box_c([p[1], p[0], p[2]], 3);
      for (s = [-1, 1]) {
        translate([-p[1]/2, 0, 0]) wall_at(s, ex, et, p[1]/2, eh, zb - p[2]);        // rear: square
        intersection() {                                                               // front: disc
          wall_at(s, ex, et, nr, eh, zb - p[2]);
          rotate([90, 0, 0]) cylinder(h = 200, r = nr, center = true);
        }
      }
    }
    rotate([90, 0, 0]) translate([0, 0, -100]) m3(200, d = M3T, $fn = 40);   // pivot, through both ears
    // Clamp bolt, bored through each EAR only.  A full-width bore would run the length of the
    // 88 mm plate 2 mm from its rear edge, through the middle of its 4 mm thickness.
    for (s = [-1, 1]) translate([lo[0], s * (ex + et/2), lo[1]]) rotate([90, 0, 0])
      cylinder(h = et + 2, d = M3, center = true);
    for (s = [-1, 1]) translate([lo[0], s * (ex + et), lo[1]]) rotate([90, 0, 0])
      cylinder(h = print_m3_nut_h * 2, d = print_m3_nut_af / cos(30), center = true, $fn = 6);
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
  // the MEASURED raised ring (the Create CAD shell does not have it) and point A's screw on it
  color("#8F969C") translate([0, 0, DECK]) difference() {
    cylinder(h = deck_measured_ring_h, r = deck_measured_ring_r1, $fn = 180);
    translate([0, 0, -E]) cylinder(h = deck_measured_ring_h + 2*E, r = deck_measured_flat_r, $fn = 180);
  }
  color("#8C5A2A") translate(concat(deck_measured_screw_a, [DECK])) cylinder(h = deck_measured_screw_a_h, d = 5);
}

module ref_jetson() {
  color("#1E6B3A") import("mesh/jetson_devkit_board.stl");
  color("#2A2E33") import("mesh/jetson_devkit_cooler.stl");
}

// --- Kieran's pack as he measured it, padding included, standing on its side: exactly the
//     pocket.  L along x, W (34.65) across, H (42.33) up. --
module ref_battery() {
  L = PACK_L; W = PACK_W; H = PACK_H;
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
