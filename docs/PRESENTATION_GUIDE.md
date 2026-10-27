# Presentation guide: MiR100 AMR in MATLAB

Slide plan, matching figures and viva answers. Figure numbers refer to the
files `make_ppt_figures` writes to `<run folder>/ppt/`. Numbers below come from
the R2025b validation run (standard goal `[4.31 0.56]`,
`matlab/validation/validation_results.txt`). If your own run differs, use the
values in `25_results_summary.png`.

## Slides

| # | Title | Figure | Say this (key points) |
|---|---|---|---|
| 1 | Autonomous navigation of a MiR100 in MATLAB | `07_3d_motion_...` | Team 8. Real robot model, known 65-wall world, plan and drive start to goal. |
| 2 | Problem | `04_occupancy_map...` | Get from start to goal through a maze without touching a wall. The robot is not a point: its footprint is 1.02 m x 0.71 m including caster swing. |
| 3 | Robot model | `01`-`03` | Official MiR100 URDF + meshes, 20 links, 19 joints. Differential drive: wheel radius 0.0625 m, wheel gap 0.4452 m. Mass 69.24 kg. |
| 4 | World and map | `04` | 65 walls from Gazebo `hard_world.sdf`. 5 cm occupancy grid. Walls grown by 0.6052 m (the robot's outer radius) so A* can treat the robot as a point. |
| 5 | Pipeline | (draw) | Walls -> occupancy grid -> inflation -> A* -> pure pursuit -> differential-drive kinematics -> clearance check. |
| 6 | Path planning: A* | `04`, `17` | 8-connected A*, no corner cutting. Deterministic: same goal gives the same path. Fails closed: if no safe path exists, the robot does not move. |
| 7 | Path following: pure pursuit | `20` | Look 0.6 m ahead on the path, steer towards it. Max 1.0 m/s, 2.0 rad/s. Each waypoint must be reached within 0.15 m; the goal within 0.10 m. |
| 8 | Robot moving in 3D | `05`-`10` + MP4 | Real meshes, wheels turning with the computed wheel speeds. Play the video here. |
| 9 | Results | `25` | 277/277 waypoints, final goal error 0.100 m, 14.82 m in 16.38 s (about 0.9 m/s average). |
| 10 | Accuracy: error calibration | `21`, `20` | Tracking error max 0.114 m, RMS 0.022 m, limit 0.20 m. Goal error falls to 0.100 m. |
| 11 | Safety: clearance | `24` | Full robot outline checked against every wall at every sample. Minimum gap 0.221 m, 0 collisions. |
| 12 | Validation | `26` | T1-T6 all pass: waypoints, clearance, tracking, model units, straight 2 m / turn pi accuracy, timing. |
| 13 | Limits and next steps | - | Kinematic simulation: no slip, inertia or motor limits. No live lidar in the core run (an optional reactive module exists). Next: ROS 2 / Gazebo link, real sensor obstacles. |

## Likely viva questions

**Why A* and not RRT or PRM?**
The world is fully known and already a grid. A* is deterministic, finds the
shortest grid path, and says clearly when no path exists. RRT/PRM add
randomness and need smoothing; on a small 2D grid they add nothing.

**What is inflation and why 0.6052 m?**
Every wall is grown by the robot's outer radius so the planner can treat the
robot as a single point. 0.6052 m is the furthest point of the footprint
polygon from the centre, including the swivelling casters. The planner never
shrinks it to force a path.

**Why does the robot not drive exactly on the path?**
Pure pursuit steers towards a point 0.6 m ahead, so it cuts corners slightly.
Measured: RMS 0.022 m, max 0.114 m, well inside the 0.20 m limit.

**Why is the goal error 0.10 m, not zero?**
The goal acceptance radius is 0.10 m. The robot stops as soon as it is
inside it.

**How do you know it never hit a wall?**
`check_run_clearance` rotates the full 8-point footprint polygon to every
logged pose and measures the distance to all 65 walls. Minimum 0.221 m. A
between-sample lower bound (0.211 m) covers the motion between samples.

**What does "kinematic" mean here?**
Position is integrated from commanded speeds. There is no mass, friction,
wheel slip or motor model. It shows the planning and control logic is
correct, not how the real robot brakes.

**How are the wheels animated?**
Wheel speeds come from the drive equations: left = (v - w*L/2)/r,
right = (v + w*L/2)/r. They are integrated to angles and applied to the
URDF wheel joints.

**What happens with an unreachable goal?**
The planner returns the reason (for example "goal is within inflation radius")
and the robot gets no motion command. It never falls back to an old route.

**Why are the walls shorter in the 3D view?**
Display only. The real 2.2 m walls hid the 0.35 m robot from every camera
angle. Planning and collision checks use the full walls.
