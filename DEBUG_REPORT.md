# MATLAB R2025b debugging report

Date: 27 October 2026. All execution results below are from installed MATLAB
25.2.0.2998904 on Windows 11. Python was not used to validate the simulation.
The user's later instruction restricted the work to code and MATLAB execution;
no further computer-use actions were performed after that instruction.

## Core result

`results = run_all;` completed. Final recorded core run:

| Measurement | Actual MATLAB result |
|---|---:|
| T1-T6 | All PASS |
| Commanded waypoints reached | 277/277 |
| Final goal error | 0.099760 m |
| Maximum / RMS path tracking error | 0.114 / 0.022 m |
| Minimum full-footprint wall clearance | 0.221322 m |
| Between-sample clearance lower bound | 0.211075 m |
| Detected collisions | 0 |
| Simulated route duration | 16.38 s |
| Measured follower wall time, final core run | 40.59 s |
| Imported and source-declared mass | 69.2390156076 kg |
| Model inventory | 20 links, 19 joints, 10 movable joints |
| Planner inflation | 0.6052 m, unchanged |

Wall time is machine/load dependent. These are kinematic results, not physics,
braking, wheel-slip or real-time validation. The clearance certificate assumes
linear XY and continuous yaw interpolation between recorded poses.

## Root causes and corrections

| File | Root cause | Correction |
|---|---|---|
| `robot/verify_robot.m` | Cell-array fields expanded `struct(...)` into ten structs, which could not be assigned to one array element. | Wrap each field payload in a scalar cell. The actual original error was reproduced before editing. |
| `robot/urdf_link_masses.m` | MATLAB regexp `\b` is a backspace, so no links were found. The alternative nested capture also failed to retain paired-link contents. | Parse opening tags and their bounded link contents explicitly, handle self-closing tags, reject an empty parse. MATLAB now finds 20 links, seven bare links and the correct mass. |
| `robot/load_mir100.m` | After fixing parsing, writing `robot.Base.Mass` threw because the tree base is a reference frame without a writable Mass property. | Normalize source-bare bodies only; record that the source-bare base is excluded from body mass. |
| `robot/verify_robot.m` | Expected URDF type `continuous` literally, but MATLAB imports these as `revolute`. | Require both `revolute` and exactly `[-Inf Inf]` position limits for each continuous joint. Inventory and geometry gates remain intact. |
| `control/follow_route.m` | `nrows,2` means exactly two rows, contradicting the documented Nx2/Nx3 route contract. | Require at least two rows and two or three columns. Include the initial pose and retain continuous yaw for clearance interpolation. |
| `planning/plan_path.m` | Struct maps were treated as bottom-row-first, while `build_map` stores row 1 at maximum Y. | Flip the struct grid when converting to the planner's internal ascending-Y convention. A native map/struct map equivalence test failed before and passes after. |
| `world/read_world_boxes.m` | Default path looked for a CSV inside `world`, although the canonical CSV is in `assets`. | Resolve the shipped asset relative to the function. |
| `reaction/lidar_react.m` | Two outputs from `ind2sub([rows cols headings],...)` merged headings into the column index. | Request all three outputs and discard heading explicitly when constructing XY coordinates. |
| `reaction/lidar_react.m` | `poseSafe` was called but never defined. Actual robotSensor execution reached this error. | Add a full-polygon command guard, including a displacement bound between endpoints and full squares for detected occupied cells. This is conditional geometric protection against known and detected obstacles. |
| `reaction/run_real_sensor_reaction.m` | Its default unseen box had yaw 0.3, but the entry point explicitly rejected any nonzero yaw. Required sibling functions were not added to the path. | Build rotated obstacles as world-coordinate custom triangle meshes and add the required sibling folders. Forward caller control/time options. |
| `reaction/robot_lidar_sensor.m`, `reaction/WorldLidarAdaptor.m` | The stock lidar adapter ray-traced the host robot, including its opaque scanner housing. At the URDF laser origin, every ray returned approximately 0.034 m. | Use the documented `robotics.SensorAdaptor` interface with the actual `robotLidarPointCloudGenerator`, passing static scenario meshes and excluding the host from ray targets. No readings are invented or discarded after generation. Host geometry and collision footprint remain intact. Explicitly reject additional dynamic platforms. |
| `reaction/lidar_react.m` | Look-ahead inspected only path vertices, missing an obstacle inside a long segment. A real feasible detour safely stalled without replanning. | Sample the interior of the requested look-ahead distance. Apply the replan cooldown to failed searches too. |
| `reaction/lidar_react.m` | The lattice admitted arbitrary initial headings, and reconstruction discarded heading-only actions. The controller could then take a different rotation from the planned route. A single coarse clearance bound could also reject a clear short sweep. | Start at the measured heading, check intermediate headings on rotation edges, preserve rotation actions through tracking, and adaptively subdivide inconclusive command-clearance intervals. Refinement still fails closed. |
| `reaction/lidar_react.m` | The speed limiter used a base-relative threshold with ranges measured from the offset front scanner. The real detour stalled before its planned turn. | Convert measured return positions to base-relative distances for the speed limiter. Keep original sensor ranges for sensor reporting. |
| `reaction/lidar_react.m` | Goal, no-path, timeout, stuck and platform-error exits could have empty stop reasons. | Populate explicit stop reasons and final pose/error/real return counters. |
| `roslink/roslink_goto_goal.m` | A callback or publish exception could bypass the final stop call. | Add an `onCleanup` stop attempt. An in-memory callback-exception test verifies this without creating ROS nodes or sending commands. |
| `demo/make_demo.m`, `run_all.m` | Default recording sampled every 100 Hz integration step at 15 fps; root status still said MATLAB had never run and did not expose aggregate success. | Derive the default render stride from dt/fps, return explicit core/validation status and evidence, use the actual footprint option, and withhold demo generation when navigation/clearance fails. |
| `validation/run_validation.m` | Optional lidar smoke did not receive the actual wall geometry. | Pass the same known wall boxes and record the returned success/stop reason separately from the smoke status. |

## Sensor execution

The actual robotSensor/robotLidarPointCloudGenerator chain was executed.
Before the host-exclusion correction, a one-obstacle probe produced 540
self-housing returns at 0.0337-0.0340 m. After correction it produced 41
real obstacle returns, minimum range 1.3212 m, with world X=1.75 m as expected.
After moving and rotating the real robotPlatform, 56 measured obstacle returns
still mapped to that face, with maximum X error 0.000000186 m.

The sensor remains a three-elevation-channel approximation, not an exact planar
SICK scanner. The adapter supports a single host robot with static world meshes.
There is a front-only blind sector. Sensor/control updates consumed by the
controller occur at its existing 0.1 s step; unused intermediate scans are no
longer generated at every 0.01 s scenario advance. No ray-caster fallback occurs
in `lidarMode='robotSensor'`.

Final native MATLAB robotSensor fixtures:

| Case | Goal reached | Goal error m | Collisions | Min clearance m | Continuous lower bound m | Replans | Stop reason | Real scans |
|---|---|---:|---:|---:|---:|---:|---|---:|
| Clear route | Yes | 0.1412 | 0 | 1.4760 | 1.4635 | 0 | goal_reached | 83 |
| Feasible 0.5 m obstacle detour | Yes | 0.1396 | 0 | 0.0887 | 0.0766 | 2 | goal_reached | 219 |
| Blocked corridor | No | 2.9000 | 0 | 0.1020 | 0.1020 | 0 | no_path | 204 |

The blocked case passes the safe-stop test; it is **not successful goal arrival**.
Actual finite point returns were 44,820 / 118,260 / 110,160 respectively.
The fixtures use the original imported MiR100 at its real scale and simple test
walls. They do not replace or modify the shipped 65-wall world. Full results are
in `../evidence/navigation_final_source/*.mat` and `../evidence/final_sensor_run.log`.

The separate **original 65-wall entry point also reached its goal** with the
default rotated unseen obstacle: goal error 0.140794 m, three replans,
613 real scans and 330,799 finite returns over 61.2 simulated seconds.
Stop reason was `goal_reached`, with zero detected collisions. With the actual
continuous-yaw trajectory, minimum full-footprint clearance was **0.060412 m**
and the between-sample lower bound was **0.055337 m**. The original wrapped-yaw
log was retained; its artificial long rotations gave 0.041767 m. The final
continuous-yaw logging correction and actual trajectory clearance calculation
are independently checked in `final_log_check.log`.
This supersedes the earlier `stuck` results recorded during debugging.

## Evidence and reproduction

All evidence is in `../evidence/` relative to this project folder.

| Evidence | Meaning |
|---|---|
| `../matlab_environment.log` | Actual MATLAB version and installed products |
| `../evidence/01_original.log` | Original failure reproduced |
| `../evidence/02_scalar_fix.log` | Next robot-verification failures |
| `../evidence/job_01.log` through `job_09.log` | Diagnostic failures, source/import measurements, collision assertions and MATLAB Code Analyzer output |
| `../evidence/job_10.log` | First completed run_all |
| `../evidence/job_11.log` | Map equivalence, 14 native collision assertions, mocked ROS exception cleanup |
| `../evidence/job_12.log`, `final_results.mat` | Final core run and focused regression test, all T1-T6 PASS |
| `../evidence/final_core_run.log`, `final_verified_core.mat` | Fresh-process full run_all rerun after the reactive planner corrections, all T1-T6 PASS |
| `../evidence/job_13.log`, `job_14.log` | Real sensor obstacle and moving-platform coordinate probes |
| `../evidence/job_15.log`, `job_16.log` | Explicit ray-mode MATLAB diagnostics for the look-ahead bug; these are not real-sensor evidence |
| `../evidence/job_17.log` | Earlier real-sensor clear/detour/blocked tests, including recorded detour failure |
| `../evidence/job_18.log` | Final real-sensor navigation fixtures and separate default 65-wall entry point |
| `../evidence/final_sensor_run.log` | Fresh-process execution of the final heading-preserving sensor implementation |
| `../evidence/final_reaction_logic.log` | Final-source native MATLAB ray-mode detour regression, success with zero collisions; not sensor evidence |
| `../evidence/final_log_check.log`, `final_sensor_world_continuous.mat` | Yaw-boundary regression and actual final-world trajectory clearance with continuous yaw |
| `../evidence/asset_integrity.log` | 36 asset files compared with the original ZIP, zero mismatches |
| `debug_fixes.patch`, `../evidence/patch_validation.log` | Source patch checked against the original ZIP source snapshot |
| `matlab/validation/validation_results.txt` | Human-readable T1-T6 results |

The diagnostic command scripts are retained beside their logs. The installed
executable was invoked from PowerShell as follows, with a different log per run:

```powershell
& 'C:\Program Files\MATLAB\R2025b\bin\matlab.exe' -batch "cd('MATLAB-AMR-MiR100/matlab'); results=run_all;" -logfile evidence\01_original.log
& 'C:\Program Files\MATLAB\R2025b\bin\matlab.exe' -batch "run('evidence/matlab_worker.m')" -logfile evidence\worker.log
```

`matlab_worker.m` executes the numbered local MATLAB diagnostic scripts, captures
each complete transcript and preserves the actual MATLAB result data. No script
creates ROS publishers. Run the public tests directly from MATLAB:

```matlab
cd('C:\Users\maxil\Downloads\MATLAB-AMR-MiR100-1da4a1f\MATLAB-AMR-MiR100\matlab')
results = run_all;
regressions = test_regressions;
sensorCases = run_navigation_checks;
realSensor = run_real_sensor_reaction(pwd);
```

For a fresh MATLAB session, add `validation` before calling its tests and
`reaction` before calling its entry point. `run_all` adds the core paths; the
validation runner adds the remaining project paths.

## Limits and untested scope

- No live ROS 2/Gazebo environment was used. The ROS module was inspected and
  exception cleanup was tested with an in-memory mock only. No physical hardware
  was commanded. ROS topic interoperability, DDS, callback behavior under load,
  deceleration assumptions and live stopping distances remain untested.
- The global planner retains 0.6052 m inflation. The reactive SE2 lattice is
  coarse and does not itself certify every planned rotation sweep. The command
  guard checks the motion that is actually executed against known and detected
  obstacles; unknown geometry, blind sectors and finite beams remain limitations.
- Existing speed limits were not increased. Acceleration, measured braking,
  physical sensor latency and higher-speed control were not implemented or
  validated. The optional controllerTEB branch was not executed or validated.
- Headless MATLAB generated 3D keyframes and robot views through the existing
  demo module. These are rendered output, not live UI inspection. No video was
  generated, and no visual success claim is made.

## Passed / failed / not tested

- **Passed:** final complete `run_all`, T1-T6, model/mass checks, native collision
  assertions, struct/native map equivalence, actual wheel-joint command readback,
  real lidar coordinate probes, real-sensor clear/detour/blocked fixtures, and
  goal arrival in the original 65-wall real-sensor scenario.
- **Expected navigation failure:** the blocked fixture returns `success=false`,
  `goalReached=false`, `stopReason='no_path'`; its collision-free safe-stop
  requirement passes. Earlier genuine failures remain in the diagnostic logs
  and are superseded by the final passing runs.
- **Not tested:** live ROS/Gazebo integration, physical hardware, dynamics,
  calibrated braking, higher speeds, optional TEB controller, and live visual
  verification. No claim of physical safety or universal avoidance is made.

The fixed sources are already installed in this local project. The patch is
for an original unmodified copy of the ZIP, using `git apply --ignore-space-change
debug_fixes.patch`. Do not apply it again to this already-fixed folder.

## Built-in desktop playback

`run_all` now automatically shows the existing 3D replay in interactive MATLAB.
Figure visibility uses `~batchStartupOptionUsed`, independently of Java desktop
availability. Playback is paced using the recorded timestamps, includes the
final frame and leaves the view open. Closing the view ends playback. The
planner and controller are unchanged by this display correction.

`../evidence/animation_entry_test.log` records the native MATLAB short-trajectory
render regression. Live desktop appearance was not inspected with computer use.

## Custom goals, fresh plans and navigation graphs

`run_all` now prompts for goal X/Y in interactive MATLAB. `run_all([x y])`
sets the destination directly, while the existing `run_all(rootDir)` call remains
supported. Batch calls without a goal keep the original [4.31 0.56] destination.
Each invocation calls A* again. No prior path/result is loaded. Repeated routes
to the same goal are expected because the planner is deterministic.

Requested-goal failure commands no motion. The previous diagnostic-waypoint
substitution was removed. The unchanged canonical T1-T6 battery still runs in
its own scenario and is explicitly labelled as reference-route validation;
its standard goal does not replace the requested mission goal.

Every invocation gets a unique `matlab/demo/runs/run_...` output folder containing
`navigation_graphs.png`, `navigation_metrics.csv`, `navigation_results.mat`,
reference validation results and any rendered demo. Graphs cover the XY route,
X/Y versus time, distance to goal and cross-track error, kinematic linear/angular
velocity, heading and full-footprint clearance. Cross-track error is distance to
the planned polyline, not distance to the final goal. Blocked-path tracking
error is undefined/NaN rather than a fabricated zero. Graphs are displayed before
the separate reference battery. The 3D replay now overlays its current commanded
path, actual trace and chosen goal. The algorithm chain is shown on the graph:
known static walls -> 0.6052 m inflation -> A* -> pure pursuit -> kinematics.

Native MATLAB evidence:

- `custom_goal_before.log`: reproduces the missing custom-goal API.
- `custom_goal_after.log`: requested [-4.31 -1.69], successful arrival with
  0.098314 m error, distinct fresh output directory and all canonical tests PASS.
- `blocked_goal_verified.log`: blocked requested [0 4.688], no fallback path,
  no wheel commands and zero mission movement; separate canonical tests remain.
- `navigation_plots_verified.log`: actual CSV and six-panel graphics, with
  geometric cross-track and goal-distance values checked against known inputs.
- `custom_goal_overlay.log`: actual MATLAB 3D render checked for the chosen
  goal coordinates, complete executed trace and wheel animation.

The final patch includes these changes. No robot/world assets, velocity limits,
planner inflation or validation thresholds were changed. The coordinate dialog
was not manually inspected; numerical goal input and rendering were executed
in MATLAB without computer-use tools.
