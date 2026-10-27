# matlab/assets - MiR100 + hard_world assets
Source: https://github.com/muffedd/ROS_mir100 commit 6450a77 (files copied unmodified under mir_description/).

## Contents
- mir_description/urdf/mir100_ref.urdf.xacro (+ include/): ORIGINAL xacro, preserved. Used by the repo's gazebo.launch.py.
- mir_description/urdf/mir100_flat.urdf: xacro-expanded (xacro 2.1.1), package://mir_description/ -> ../ so meshes resolve relative to the URDF. Still has Gazebo <gazebo>/<plugin> blocks.
- mir_description/urdf/mir100_matlab.urdf: same, <gazebo> blocks stripped. Use this with importrobot (relative mesh paths, e.g. ../meshes/visual/wheel.stl). Keep the urdf/ and meshes/ folders side by side.
- mir_description/meshes/ (visual + collision STL, 8 visual meshes referenced), mir_description/worlds/hard_world.sdf, gen_hard_world.py.
- hard_world_boxes.csv: 65 walls, columns id,xc,yc,zc,sx,sy,sz,yaw (m, rad, world frame, header row). Made by sdf_to_boxes.py.
- mir100_footprint.csv: Nav2 footprint polygon (base_footprint frame, m), from nav2 config values given by project.
- verify_assets.py: static checks (mesh paths, box count, footprint vs URDF collision). Result: all 8 mesh files exist, 65 boxes, polygon contains every URDF collision point within 0.05 mm rounding.
- MODEL_VALIDATION.txt, HARD_WORLD_VALIDATION.txt: repo docs (Gazebo-side, not MATLAB evidence).

## Inventory / units (all SI, meters/radians)
- World: 65 boxes named w0..w64, all static, yaw=0 for every one (axis-aligned), zc=1.1, sz=2.2, thickness mostly 0.375 (grid cell 0.375). Extent x[-6.3755,6.3755] y[-4.8755,4.8755]. SDF values are 3 decimals (e.g. 4.688 vs grid 4.6875), CSV preserves them exactly.
- Excluded by identity: ground_plane (plane geometry, no box) and observer_cam (sensor only, no collision). Light also skipped.
- Robot: URDF name mir100_ref, base_footprint -> base_link (zero offset). wheel radius 0.0625, separation 2*0.222604=0.445208, total mass 69.239 kg (58 base + 11 x 1 kg). Actuated: left_wheel_joint, right_wheel_joint (continuous, axis y). Casters: 4 swivel (z) + 4 wheel joints (continuous) - passive.
- Collision in URDF is primitives, NOT the STL: base box 0.9x0.58x0.25 at (0.0376,0,0.20); lasers boxes 0.16x0.16x0.08 yawed 45deg at (0.4288,0.2358) and (-0.3548,-0.2352); wheel cylinders. The STL meshes are visual (collision/*.stl exist but unreferenced by mir100_ref).
- Laser frames: front_laser_link (0.4288,0.2358,0.1914) yaw +pi/4; back_laser_link (-0.3548,-0.2352,0.1914) yaw -3pi/4.
- Footprint polygon max radius 0.5988; URDF collision max radius 0.5910 (polygon is slightly conservative).

## Caveats
- NOT verified in MATLAB: importrobot on these URDFs was not run (no MATLAB here). importrobot may warn about continuous joints/ignored gazebo tags; check mesh resolution there. STL visuals are large (mir_100_base.stl 37k triangles).
- If importrobot fails on meshes, fall back to loading STLs with stlread, or drop visuals.
- Footprint numbers come from the repo's Nav2 config, cross-checked against URDF geometry only.
