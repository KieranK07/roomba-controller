# Scans and vendor CAD drop folder

Put anything you capture or download in here and tell Claude. Everything below
feeds `hardware/dimensions.json`, which drives both the OpenSCAD parts and the
web viewer.

## 1. Downloads that need your login (5 minutes, highest value)

**Jetson Xavier NX 3D STEP model** — needs a free NVIDIA developer account.
  https://developer.nvidia.com/embedded/jetson-xavier-nx-3d-cad-step-model
  Grab `Jetson_Xavier_NX_Module_3D_STEP_Model_nv.zip`. That is the *module*.
  While logged in, also check the Jetson Download Center for the
  **Xavier NX Developer Kit Carrier Board design package (P3509)** — if a STEP
  or mechanical DXF is in there, that is the carrier board we actually bolt to.
  https://developer.nvidia.com/embedded/downloads

Drop the .zip in here as-is. No need to extract.

## 2. Photogrammetry (the parts nobody publishes)

Only two things need scanning: the **Roomba 690** and the **Home Base**.
Everything else is either vendor CAD or a rectangular box that calipers settle
in ten seconds.

- App: RealityScan (Epic, free) or Polycam in **Photo** mode. Plain
  photogrammetry, not the LiDAR mode — LiDAR is only good to 3-5 mm.
- **Print or find a scale bar** and include it in every scan. A ruler works.
  Without a known length in frame the mesh has no absolute scale and I cannot
  use it for fit.
- Matte the shiny top: painter's tape torn into patches, or a light dusting of
  dry shampoo. Photogrammetry fails on gloss.
- Soft even light, no flash. 60-100 photos per scan, overlapping by half.

Three scans:
1. Roomba upright on the floor (top, sides, bumper).
2. Roomba flipped on a towel (**the important one** — underside, caster,
   cliff sensors, wheels).
3. Home Base alone, with the scale bar lying on the ramp.

Export **OBJ or PLY**. Drop the export and the source photos in here.

## 3. Three measurements a scan will not give me

Do these with calipers or a ruler and just write the numbers in a text file:

- **Ground clearance at the contact pads.** Dock the robot, then slide a stack
  of business cards under the front until they touch. Measure the stack.
  My model currently assumes 8 mm and that number sets the whole dock design.
- **Home Base ramp profile.** One photo, camera on the floor, level, side-on,
  with a ruler standing next to the ramp. That silhouette sets pin height.
- **Pad location.** Where on the underside is there flat space about 120 x 28 mm,
  behind the bumper, clear of the cliff sensors? A straight-down photo of the
  underside with a ruler across it is enough.

## What is already here

- `hardware/vendor-cad/` — RealSense D400 CAD from Intel (SolidWorks .SLDPRT,
  not readable on this Mac), Intel's own `d435.dae` ROS mesh (usable, verified
  89.91 x 25.00 x 25.05 mm against the datasheet), and a Create 2 body mesh
  from the AutonomyLab ROS driver (visual only, off by ~12 mm in height).
- `hardware/mesh/realsense_d435i.stl` — the D435i display mesh, derived from
  Intel's file, oriented into the robot frame.
