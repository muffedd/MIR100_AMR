# matlab/reaction - lidar_react

Files: lidar_react.m, react_capabilities.m, run_react_demo.m, ref/react_ref.py (Python mirror), ref/hard_world_boxes_from_sdf.csv (65 walls parsed from hard_world.sdf @6450a77, ground plane excluded by identity; all yaw 0 in the fetched text - project's read_world_boxes is authoritative).

## Evidence status
- MATLAB: NOT executed (MATLAB/Octave execution has not been performed for this build). lidar_react.m is statically reviewed only; syntax/runtime unverified.
- Python mirror (ref/react_ref.py) of the same algorithm on the real hard_world walls and route v2 (world = map + [-4.31 -2.81]) with one 0.3-0.5 m unseen box. Kinematic point sim, NOT physics, NOT full-robot proof, NOT MATLAB evidence:
  | case | result |
  | no unseen box | success, 31.6 s |
  | box (-4.31,-1.7) 0.5 m, blocks corridor mouth | FAILED: no certified footprint path (replan failed repeatedly), no collision |
  | box (-3.9,-1.9) 0.5 m yaw .3 | success, 2 replans, no collision |
  | box (-4.0,-1.5) 0.4 m | success, 3 replans, no collision |
  | box (-4.6,-1.4) 0.3 m yaw .7 | FAILED: replan found no certified path, no collision |
  | box (-5.0,-2.0), (-3.5,-1.2) off route | success, 0 replans |
  Earlier runs that lowered inflation (0.5/0.45/0.4) were removed; those pass counts are void.

## Design
Core needs no toolbox: default lidarMode='ray' own 2-D ray caster (a distinct simulation, used for the Python/logic reference and as an explicit non-MathWorks mode; the MathWorks sensor path is the separate real mode below), grid from known boxes, lidar hits not explained by the known map become unseen cells, look-ahead footprint check, replan with an orientation-aware SE2 lattice A* (8 headings, 2*res grid, exact footprint-polygon cell masks, no inflation radius shrinking or tuning), pure-pursuit tracking. EVERY command's next pose gets a full-polygon check against known polygons + detected unseen cells (candidates: nominal, half speed, rotate, reverse, stop). A ground-truth full-polygon collision check runs every step (collided flag).
Optional, gated, UNTESTED: opts.useTEB (controllerTEB, R2023a+, Navigation Toolbox). TEB init/step errors are recorded in reactOut.tebErrors and notes, reactOut.tebSteps/coreSteps show who drove; no claim TEB worked unless tebSteps>0 and tebErrors empty. opts.movePlatform: a failed platform move aborts the run with success=false, platformMoveFailed=true.
robotSensor / robotLidarPointCloudGenerator: implemented (static-only, never run) as lidarMode='robotSensor' - see below. Real mode is only entered explicitly by run_real_sensor_reaction; it never falls back to the ray caster.

## Contract notes / inconsistencies
1. lidar_react(plat,scen,path,opts): known boxes cannot be recovered from a robotScenario, so opts.boxes (Nx7 world, from read_world_boxes) is REQUIRED. plat/scen may be [] for the core.
2. plan_path frame is unspecified; lidar_react assumes path is WORLD frame (map + [-4.31 -2.81]).
3. opts.unseenBoxes (or opts.autoUnseen=true) supplies the obstacles the planner does not know.
4. Output: success, collided, goalReached, replanCount, unseenDetected, log(t,x,y,yaw), pathFinal, minLidarRange (lidar-centre proxy, not footprint clearance - use check_run_clearance), notes, caps.
5. Footprint-based only: conservative polygon from supplied model leads; verify_robot should confirm.
6. Known limits: circular inflation conservative; may deadlock in narrow corridors (seen in Python); kinematic only, no wheel angles in log, no slip, no real-time guarantee.

## Real MathWorks sensor path (STATIC-ONLY, never executed)
Entrypoint: run_real_sensor_reaction(rootDir) = build_world (65 wall meshes, 100 Hz) -> load_mir100 -> spawn_mir100 -> robot_lidar_sensor('attach') -> build_map(boxes,20,[]) -> plan_path -> setup(scen) -> lidar_react(lidarMode='robotSensor').
- Sensor: robotLidarPointCloudGenerator (3-D organised point cloud, 270 deg az x 3 elevation channels 0..2 deg, 0.5 deg, 8 m, 100 Hz, noise off) via robotSensor mounted on body "front_laser_link" (URDF joint origin xyz .4288 .2358 .1914, yaw 45 deg; transform to base taken from getTransform at home config). Elevation >= 0 so the ground plane is never hit.
- The controller consumes the planar nearest-return-per-azimuth projection: detected cells not explained by the known map -> look-ahead block check -> SE2 footprint replan -> per-command full-polygon check -> pure pursuit. Unseen box exists only in the scenario (addMesh) and ground truth, not in the planner map.
- Gates: R2022a+, Robotics System Toolbox, Navigation Toolbox, Computer Vision Toolbox, robotSensor; failure throws, no ray-caster fallback in this mode. Only the front laser is mounted (rear blind sector; reverse candidates disabled in this mode).
- Each control step (dt 0.1) runs round(dt*scen.UpdateRate)=10 advance/updateSensors steps.
Unverified without MATLAB: MountingAngles/MountingLocation semantics, robotSensor read output shape (organised Location, NaN = no return), move(plat,"base",...) 16-vector, whether default Collision/mesh options let the generator hit walls, runtime performance (1,623 rays x 100 Hz).
Footprint default is polygon v2 from the integration contract file (not re-verified by this module).

Mount/elevation notes: the sensor is mounted with MountingBodyName="front_laser_link" and zero offset; the link frame already contains the URDF joint (xyz [0.4288 0.2358 0.1914], rpy yaw 45 deg), and the same transform is applied from getTransform for the world conversion, so the 45 deg yaw and offset are counted once (no separate MountingAngles). If the MathWorks mount semantics differ, the equivalent form is base-body mount with MountingLocation [0.4288 0.2358 0.1914], MountingAngles [45 0 0] (ZYX deg). Elevation limits are [0 2] deg x 1 deg (3 channels): never below horizontal, so the floor (hit at 2.19 m with [-5 5] from 0.1914 m height) is not seen. A [0 0] single-channel setting is NOT used because the docs read do not state that equal bounds are allowed; 3-D points are projected to 2-D for control. Update rate 100 Hz equals the scenario rate.

Fail-closed sensing (real mode): lidar_react never fabricates readings. No platform motion until a real scan arrives and none while the newest scan is older than opts.maxScanAge (0.5 s); the scenario is advanced without moving to obtain scans; after opts.scanTimeout (5 s) the run stops with reactOut.stopReason and success=false.

## Controller change (5:50): rotate-then-drive (replaces pure pursuit)
Chosen because the SE2 lattice path is piecewise straight at 45 deg multiples, and rotate-then-drive follows exactly those segments (pure pursuit cut corners and the per-command polygon check then stalled). MiR100 is differential drive (wheel separation 0.4452 m), so turning in place is feasible. Tradeoff: slower (about 49 s vs 31.6 s baseline in the mirror). Planning keeps the 0.6052-class conservative footprint masks (exact v2 polygon, no radius shrink); this may reject tight passages, and then the run stops with no motion into contact.
Python mirror (kinematic, NOT MATLAB, NOT physics), v2 footprint, route v2:
| case | result |
| no unseen box | PASS 49.0 s, 0 contact |
| shipped demo box [-3.9,-1.9] 0.5 yaw .3 | PASS, 2 replans, 0 contact |
| box [-4.0,-1.5] 0.4 | PASS, 1-4 replans, 0 contact |
| corridor mouth box [-4.31,-0.91] 0.5 | STOP (no certified path; gap 0.69 m < footprint width 0.709 m), 0 contact |
| box [-4.31,-1.7] 0.5 | STOP, replan failed repeatedly, 0 contact: UNRESOLVED whether true infeasibility or planner conservatism (0.1 m lattice, 45 deg headings) |
Avoidance is NOT proven. MATLAB version of rtdControl is statically written only.
