# matlab/roslink - optional ROS 2 link module

Owns the (optional, deferred) connection between the MATLAB headless pipeline
and the ROS 2 / Gazebo Harmonic simulation of github.com/muffedd/ROS_mir100
(commit 6450a77, ROS 2 Jazzy; README-verified 2026-10-27).

## Files
- `ros_link_skeleton.m` - superseded by `matlab/roslink/` (written, MATLAB NOT RUN); this file remains the older gated-off skeleton. NOT EXECUTED. Commented ROS 2 skeleton
  (ros2node / ros2subscriber / ros2publisher / ros2message), hard-gated OFF.
  Subscribe-first design; the /cmd_vel publisher block is an explicit override
  mode only (Nav2 owns control). ROS 1 (rosinit) intentionally absent - the
  repo is ROS 2 Jazzy, ROS 1 does not apply.
- `VERSION_MATRIX.txt` - doc-verified release/toolbox requirements for every
  MathWorks API in the integration contract plus the ROS 2 link APIs, against
  the R2022a baseline, fail-closed on anything newer.

## Hard facts
- No MATLAB executable exists in the authoring environment (checked
  `command -v matlab`, /usr/local/MATLAB, /opt/MATLAB, /Applications on
  2026-10-27): nothing here is execution-verified.
- Version claims come from live official MathWorks doc pages (Version History
  sections), URLs cited in VERSION_MATRIX.txt.
- Headline flags: controllerTEB is R2023a+ (not at R2022a baseline, F1);
  controllerPurePursuit's second output is CURVATURE not angular velocity from
  R2026a (F3, exact quotes in matrix); robotPlatform can move without
  BaseTrajectory via move() (R2022a), making Sensor Fusion and Tracking
  Toolbox optional (row 2a).
- The skeleton keeps the real MiR100 URDF/meshes (mir_description) and the
  Gazebo world as the world source; no point-robot substitution.
