# MiR100 conservative footprint clearance

Source: https://github.com/muffedd/ROS_mir100
Verified checkout: 6450a77ed93cbc0141a53e036c6a9b31feabf7d9.

## Integration

Add this directory to the MATLAB path. Call:

```matlab
clearLog = check_run_clearance(trajLog, boxes);
assert(clearLog.pass, clearLog.status);
```

`trajLog` is a scalar struct or table with equally sized finite vectors `t`, `x`, `y`, `yaw`. `t` is seconds and strictly increasing. XY is world metres, not route/map coordinates; yaw is continuous/unwrapped radians. `boxes` is N-by-7 `[xc,yc,zc,sx,sy,sz,yaw]`, with positive sizes, world metres/radians. No argument reordering. It accepts 65 wall boxes but does not hardcode their count, allowing synthetic tests. All supplied boxes are conservatively projected into XY; this is appropriate for hard_world's floor-to-2.2 m walls, not a general exact 3-D collision test.

Canonical locked integration fields:
- `t` and `minDist` are N-by-1 original logged timestamps and minimum wall distances.
- `minOverall` is the scalar minimum over original AND refined samples.
- `minAt` has `x`, `y`, `t`, `box` (1-based input box row).
- `collision` is a scalar true if any original OR refined sample contacts a wall.

Additional outputs and aliases:
- `clearance` and `collisionFlags`: distances/contact at the original logged poses.
- `refined`: adaptive linear-XY, linear-yaw samples with nearest wall, robot/wall witness points and flags. Translation step defaults 4 cm and yaw step 3 degrees.
- `anyCollision`: contact/overlap detected at ANY original or refined sample.
- `minClearance` / `minClearanceM`: smallest sampled unsigned distance. Zero means contact or overlap, NOT penetration depth.
- `minLocation`: time, full XY/yaw pose, 1-based row index into `boxes`, and closest pair of points. Containment witness is a common point, not an exit-distance pair.
- `continuousClearanceLowerBound`: conservative bound on the entire interpolated path.
- `isCollisionFree` / `pass`: true only if no contact was found and the continuous lower bound is above tolerance.
- `status`: `collision_or_contact_detected`, `certified_clear_under_interpolation_model`, or `inconclusive_between_samples`.

A zero lower bound with no detected contact is INCONCLUSIVE, not a collision and not a pass. There is a hard sample cap that errors rather than silently dropping refinement. Empty wall sets produce infinite clearance and wall index 0; a single pose is allowed.

## Geometry and the corrected envelope

The original supplied polygon is slightly too small to claim all-caster-angle containment. This exact URDF uses `meshes/visual/wheel.stl` in both visual AND collision elements for drive and caster wheels. Its `cylinder_inertial` call is not a collision cylinder. Caster collision hub uses `meshes/collision/caster_wheel_base.stl`. Scanner collision uses `meshes/visual/sick_lms-100.stl`.

The original polygon was expanded OUTWARD by 6 mm on every supporting edge, rather than reducing/changing source collision geometry. Corrected default vertices in base frame:

```text
-0.474026970 -0.236085370
-0.439392950 -0.319792950
-0.355791680 -0.354382980
 0.493049750 -0.295637190
 0.548024070  0.236692380
 0.513392950  0.320392950
 0.429789310  0.354983960
-0.417838050  0.295633920
```

Radius of these rounded vertices: 0.6051644103211994 m. Use 0.6052 m if a rounded-up enclosing circle is required. Keep this SAME envelope in planning, control and validation. This is conservative geometry, not an exact mesh union.

Proof in the executed Python reference:
- All 74,844 fixed collision mesh vertices (base with URDF x offset +0.037646, scanners transformed by roll pi and yaw +/-45/135 degrees) lie within the corrected polygon's halfspaces.
- Drive wheel collision vertices under arbitrary spin are enclosed by the wheel radial XZ maximum and Y halfwidth, then checked against all polygon halfspaces.
- Caster pivots are at x=0.341346 or -0.270154, y=+/-0.203. Wheel joint offset is x=-0.0382, y=0, z=-0.094. Arbitrary wheel spin and caster yaw have maximal XY radius 0.10196318885249485 m about each caster pivot. This bound is attainable by an actual wheel vertex under those joint rotations, not merely a loose support circle.
- Original polygon's worst caster margin is -0.005240073803847625 m. Corrected polygon's minimum margin is +0.0007599235526160181 m. This correction is therefore necessary for an all-caster-angle claim.

## Inter-sample certificate

For each refined interval, let DeltaXY be center displacement, DeltaYaw the absolute supplied yaw change, and R the maximum footprint vertex radius. A point at any intermediate pose is within

`||DeltaXY||/2 + 2*R*sin(min(|DeltaYaw|/4, pi/2))`

of its corresponding point at the nearer endpoint. Distance to a fixed obstacle is 1-Lipschitz under that displacement. Hence

`max(0, min(endpointClearances) - residualBound)`

is a valid interval clearance lower bound. The rotation term caps at 2R for long rotations. This certificate assumes straight-line XY and linear yaw between the logged poses. It says nothing about unlogged physical excursions, wheel slip, dynamic obstacles, out-of-plane motion, tracking accuracy, collision meshes changing, execution rate, or real-time deadlines. A supplied wrapped yaw jump is interpreted literally as a long rotation; the checker deliberately does not replace it with a possibly false shortest turn.

## Tests and execution status

`test_collision.m` supplies native MATLAB assertions for edge touch, corner touch, strict containment in both directions, crossing edges with no vertex containment, repeated vertices, rigid transform invariance, tunneling, clear/inconclusive intervals, empty worlds, and a full 2*pi rotation. These assertions were NOT executed here: no MATLAB or Octave binary was available.

`reference_collision.py` is a separately executed Python reference with seven synthetic edge cases, 400 independently cross-checked convex SAT and rigid-transform cases, 80 interpolation lower-bound checks with 31 dense probes each, mesh/caster verification, and the exact world's 65-wall count/start-pose check. `reference_results.json` records its results. Python requirements are NumPy and SciPy, already available in the execution environment. No MATLAB/physics validation is implied.

`reference_route.py` independently checks the supplied route as straight segments plus in-place shortest turns at waypoints, initially yaw 0. It does not pretend to be the MATLAB follower. Its 999 samples have minimum footprint clearance 0.1453413302 m at world pose [-4.31,0,1.0172219679], nearest wall w24; the conservative continuous lower bound is 0.1393591492 m. That bound certifies only this explicitly constructed kinematic route model. Actual follower logs must go through `check_run_clearance` separately.

Reproduce with:

```text
python3 reference_collision.py --repo /path/to/ROS_mir100 --output reference_results.json
python3 reference_route.py --repo /path/to/ROS_mir100 --output route_reference_results.json
```

`hard_world.sdf` contains an `observer_cam` model with pitch pi/2 and no collision box. A wall reader must select collision box geometry and exclude the ground by identity, rather than assuming every non-ground model is a yaw-only box.

## MATLAB API/product grounding

This module uses base MATLAB numerical functions and `inpolygon`; it does not require Robotics, Mapping, Navigation, or Parallel Computing Toolbox. This does not waive product requirements of the rest of the project. No MATLAB execution is claimed.

Official pages inspected:
- https://www.mathworks.com/help/matlab/ref/inpolygon.html : inside/on-boundary syntax and semantics; introduced before R2006a.
- https://www.mathworks.com/help/matlab/ref/validateattributes.html : numeric, real, finite, vector, ncols and scalar validation; introduced R2007b.
- https://www.mathworks.com/help/matlab/ref/repmat.html : explicit row replication; introduced before R2006a.

The module avoids polyshape and implicit array expansion in its MATLAB code. Whole-project supported release is owned by the integration project's Robotics Toolbox APIs, not these function-introduction dates; do not infer an R2007b project requirement from this module.

Static parse/lint evidence: MISS_HIT analyzed all three `.m` files successfully (`static_lint_results.txt`). This is a MATLAB syntax/static check, not a MATLAB execution or numerical test.
