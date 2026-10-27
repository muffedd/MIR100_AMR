# MATLAB-AMR-MiR100

Modelling, simulation and control of the MiR100 wheeled robot in MATLAB
(college project, Team 8). Uses the SAME robot model (URDF + meshes) and
the SAME custom 65-wall world as the companion ROS 2 project
github.com/muffedd/ROS_mir100 (commit 6450a77).

## HONEST STATUS (read first)

**The core now completes in MATLAB R2025b, with T1-T6 passing.**
See [DEBUG_REPORT.md](DEBUG_REPORT.md) for measured results, source fixes,
actual MATLAB logs, sensor results and limitations. This remains a kinematic
simulation, not physical validation. The URDF, meshes and 65-wall assets
match the original archive byte-for-byte. No physical hardware was commanded.

The following describes the original authoring evidence, before this debug run:

- Statically reviewed MATLAB source for the full core pipeline
  (world -> robot -> map -> planning -> control -> collision -> demo
  -> validation battery).
- Python reference checks that DID run (algorithm-level evidence only,
  not MATLAB, not physics): route follower reaches all 16 waypoints
  (final error 0.0997 m, 21.93 s), full smooth trajectory collision-free
  against all 65 walls with the corrected footprint (minimum clearance
  15.8 mm), footprint polygon v2 verified against the URDF collision
  geometry, 65/65 wall CSV matches the source SDF, 30/30 model reference
  checks.
- A footprint correction found during this build: the previous polygon
  (v1, radius 0.5988 m) left the swivelling caster wheels up to 5.24 mm
  outside. Polygon v2 (radius 0.6052 m) fixes this. See
  collision/README.md. This correction is NOT retro-applied to the
  earlier Gazebo work, which stays as recorded.

To run in MATLAB, cd to `matlab/`, then:

```matlab
results = run_all;
```

In the MATLAB desktop this asks for your goal X/Y in world metres, computes a
fresh path, and opens the original robot/world in 3D with wheel animation.
The view shows the commanded path, executed trace, a footprint outline that
follows the robot, and the requested goal. Walls are drawn 0.6 m tall in this
3D view only (real 2.2 m walls hid the robot); planning and collision checks
still use the full walls. It is a
replay of the freshly computed trajectory. Batch runs stay hidden.

For presentation slides, run once from `matlab/`:

```matlab
pptDir = make_ppt_figures;            % standard goal [4.31 0.56]
pptDir = make_ppt_figures([-4.31 -1.69]);  % or your own goal
```

It does one real run and writes numbered PNGs (200 dpi) to `<run folder>/ppt/`.
They cover the robot model, the occupancy map with the A* path, six 3D and
six 2D frames of the robot moving, the route overview, the graphs (errors,
error calibration, speed, heading, clearance), a results table and the T1-T6
table. It also writes an MP4 of the 3D motion (Windows/macOS MATLAB).

Slide-by-slide plan and viva answers: [docs/PRESENTATION_GUIDE.md](docs/PRESENTATION_GUIDE.md).

To set a goal directly without a dialog:

```matlab
results = run_all([-4.31 -1.69]);
```

Each call saves its own folder under `matlab/demo/runs/`, printed in the Command
Window and returned as `results.outputDir`. Open `navigation_graphs.png` for six
panels: route, X/Y versus time, goal distance and tracking error, linear/angular
speed, heading, and full-footprint clearance. Numeric data is in
`navigation_metrics.csv`; complete results are in `navigation_results.mat`.
Previous outputs are reports only; the runner never loads them to plan a path.
An identical goal produces the same deterministic A* route. A blocked goal
commands no motion and reports the reason without substituting another route.

Core algorithms: known static walls -> occupancy grid -> 0.6052 m inflation ->
8-connected A* -> pure-pursuit steering -> differential-drive kinematics. The
core does not use live lidar. The independent T1-T6 battery remains a separate
check against the standard reference goal, clearly labelled in the output.

## Requirements (core)

- MATLAB R2022a or newer
- Robotics System Toolbox (robotScenario, robotPlatform, importrobot)
- Navigation Toolbox (occupancyMap / binaryOccupancyMap)

Optional modules (NOT needed for the core run): reaction/ (reactive
replanning demo), roslink/ (ROS 2 client for a separately running Gazebo MiR100,
written, static review only, MATLAB NOT RUN, never used by the core run;
versions/ros_link_skeleton.m is the older gated-off skeleton). See versions/VERSION_MATRIX.txt for per-API
release notes and MATLAB Online notes.

## Layout

- `run_all.m` - root entry point (core pipeline + validation battery)
- `world/ robot/ map/ planning/ control/ collision/ demo/ validation/` - modules
- `assets/` - canonical MiR100 URDF + meshes (from ROS_mir100), wall CSV
- `versions/` - version matrix, older ROS 2 skeleton (gated off); real optional link is in roslink/ (NOT RUN)
- `ref/` - Python reference checks (evidence; NOT MATLAB)
- `validation/TESTS.txt` - the engineering test battery T1-T6
- `INTEGRATION_NOTES.txt` - per-module status, fixes applied, open items

## Safety rules baked in

- The planner fails closed: it never shrinks inflation to force a path.
- Collision checking uses the actual footprint polygon, not a point.
- Diagnostic route-follows are labeled and never counted as a
  global-planner pass.

## License / provenance

mir_description assets are from the DFKI MiR robot description
(see assets/mir_description/LICENSE.dfki-mir_robot.txt) via the user's
ROS_mir100 repository. World geometry from the user's hard_world.sdf.

## Optional modules added after the core (static review only, MATLAB NOT RUN)

- `matlab/reaction/robot_lidar_sensor.m`, `run_real_sensor_reaction.m`: manual entry using `robotSensor` with `robotLidarPointCloudGenerator` mounted on `front_laser_link`. 3-channel 0..2 deg upward cloud projected to planar; not an exact physical 2-D SICK S300. Needs Robotics System Toolbox, Navigation Toolbox, Computer Vision Toolbox. Moves nothing until a fresh real scan arrives. See `matlab/reaction/README_reaction.md`.
- `matlab/roslink/`: ROS 2 client (MATLAB ROS Toolbox) publishing /cmd_vel and subscribing to scans and /odom of a separately running Gazebo MiR100. Base-frame laser transforms, both scans fresh for turns, braking-distance bound with an ASSUMED deceleration that is not validated; zero Twist on every exit path. Not a collision-free guarantee. Runtime needs a reachable DDS network and physical deceleration must be calibrated before any motion. See `matlab/roslink/README.md`.
- Python reference checks in `matlab/ref/` run without MATLAB. No MATLAB output exists for any module.

- `docs/ALGORITHM_NOTES.md` - planner/controller selection rationale (Rev B). High-speed TEB is a candidate only, not integrated, not run.
