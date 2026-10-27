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
Core needs no toolbox: own 2-D ray caster (simulation distinct from robotLidarPointCloudGenerator; the optional real-sensor path is in robot_lidar_sensor.m / run_real_sensor_reaction.m, see README_reaction.md, NOTRUN), grid from known boxes, lidar hits not explained by the known map become unseen cells, look-ahead footprint check, replan with an orientation-aware SE2 lattice A* (8 headings, 2*res grid, exact footprint-polygon cell masks, no inflation radius shrinking or tuning), pure-pursuit tracking. EVERY command's next pose gets a full-polygon check against known polygons + detected unseen cells (candidates: nominal, half speed, rotate, reverse, stop). A ground-truth full-polygon collision check runs every step (collided flag).
Optional, gated, UNTESTED: opts.useTEB (controllerTEB, R2023a+, Navigation Toolbox). TEB init/step errors are recorded in reactOut.tebErrors and notes, reactOut.tebSteps/coreSteps show who drove; no claim TEB worked unless tebSteps>0 and tebErrors empty. opts.movePlatform: a failed platform move aborts the run with success=false, platformMoveFailed=true.
robotSensor / robotLidarPointCloudGenerator read path: OPTIONAL, written static-only, MATLAB NOT RUN (needs Robotics System Toolbox + Navigation Toolbox + Computer Vision Toolbox). Entry: run_real_sensor_reaction.m; details README_reaction.md.

## Contract notes / inconsistencies
1. lidar_react(plat,scen,path,opts): known boxes cannot be recovered from a robotScenario, so opts.boxes (Nx7 world, from read_world_boxes) is REQUIRED. plat/scen may be [] for the core.
2. plan_path frame is unspecified; lidar_react assumes path is WORLD frame (map + [-4.31 -2.81]).
3. opts.unseenBoxes (or opts.autoUnseen=true) supplies the obstacles the planner does not know.
4. Output: success, collided, goalReached, replanCount, unseenDetected, log(t,x,y,yaw), pathFinal, minLidarRange (lidar-centre proxy, not footprint clearance - use check_run_clearance), notes, caps.
5. Footprint-based only: conservative polygon from supplied model leads; verify_robot should confirm.
6. Known limits: circular inflation conservative; may deadlock in narrow corridors (seen in Python); kinematic only, no wheel angles in log, no slip, no real-time guarantee.

## Reference cases before and after the rotate-then-drive controller (Python mirror `ref/react_ref.py`, footprint v2; MATLAB NOT RUN)

Obstacle avoidance is NOT proven. Mirror is kinematic, not physics. Run from a fresh tree: `python3 matlab/ref/react_ref.py`.

| case (unseen box x,y,size) | before: lattice + pure pursuit | after: lattice + rotate-then-drive |
|---|---|---|
| none | PASS 31.6 s | PASS 49.0 s |
| shipped demo [-3.9,-1.9] 0.5 yaw .3 | FAIL stuck, 21 replans | PASS, 2 replans, 50.3 s |
| corridor mouth [-4.31,-0.91] 0.5 | FAIL, replan failed x15 (infeasible: 0.69 m free gap < 0.709 m footprint width at heading 90 deg) | same: no path, stops, no contact |
| room [-4.31,-1.7] 0.5 | FAIL, replan failed x201 | same: stop, UNRESOLVED (true infeasibility or planner conservatism of 0.1 m lattice / 45 deg headings) |
| small [-4.0,-1.5] 0.4 | PASS, 3 replans | PASS, 4 replans, 53.0 s |

All cases: 0 contact in the mirror. Earlier "3 of 4 pass" claims used footprint v1 and are void.

Caveats (reviewer-accepted wording):
- Rotate-then-drive is a LOW-SPEED reference baseline, slower than pure pursuit. It is not a high-speed navigation solution; high speed is an open, unevaluated item.
- The planner is still the SE2 lattice, not plain grid A*, for detours.
- Endpoint checks at dt 0.1 s grown 0.1 m cover step translation 0.05 m and rotation arc 0.048 m under current limits. This is an argument, not a certificate.
- SE2 planned 45 deg rotation actions check discrete headings only. There is NO swept-rotation proof; a circumradius rotation mask fix is pending.
