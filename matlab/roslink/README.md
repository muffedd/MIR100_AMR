# matlab/roslink - ROS 2 client link to the ROS_mir100 Gazebo sim

Stage: ROS integration. MATLAB acts as a CLIENT of the existing Gazebo
Harmonic sim from ROS_mir100 @ 6450a77: it PUBLISHES /cmd_vel and
SUBSCRIBES to the sim's /scan, /back_scan, /odom (+ optional ground truth).
Nothing here synthesizes sensor data or republishes MATLAB state.

## Verified topic map (from the repo, not assumed)
Source: src/mir100_navigation/launch/slam_navigation_hardtest.launch.py,
bridge nodes mir100_gz_bridge and mir100_front_scan_bridge:

| ROS 2 topic | Type             | Direction vs MATLAB | Bridge entry (repo)                    |
|-------------|------------------|---------------------|----------------------------------------|
| /cmd_vel    | geometry_msgs/Twist    | MATLAB publishes | /cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist (ROS -> gz) |
| /odom       | nav_msgs/Odometry      | MATLAB subscribes | /odom@nav_msgs/msg/Odometry[gz.msgs.Odometry |
| /scan       | sensor_msgs/LaserScan  | MATLAB subscribes | /scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan, override_frame_id front_laser_link |
| /back_scan  | sensor_msgs/LaserScan  | MATLAB subscribes | /back_scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan |
| /joint_states | sensor_msgs/JointState | (not used here) | /joint_states@sensor_msgs/msg/JointState[gz.msgs.Model |
| /clock      | rosgraph_msgs/Clock      | (not used here) | /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock |

Model plugin sources (urdf/mir100_ref.urdf.xacro): DiffDrive topic
cmd_vel, odom_topic odom (frame_id odom, child_frame_id base_footprint,
max_linear_acceleration 1.0, max_angular_acceleration 1.5); gpu_lidar
topics scan / back_scan. Ground truth: the model's gz PosePublisher emits
link poses on gz transport, but the repo does NOT bridge them to ROS 2.
link.latestGroundTruth() therefore needs a user-side extra bridge argument
(standard mapping, unverified live):
  /pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V
and degrades to available=false without failing the link.

## Two separate runtime paths - do not conflate
1. MATLAB Online core (import -> world -> plan -> follow -> validate):
   needs NO ROS 2, NO ROS Toolbox, NO Gazebo. This folder is never loaded
   by the core (run_all.m does not call it).
2. THIS external Gazebo path: needs a real MATLAB installation with ROS
   Toolbox (R2019b+ API; project baseline R2022a) on a host with DDS
   reachability to the Gazebo machine. The sim side is ROS 2 Jazzy +
   Gazebo Harmonic on Linux (repo README: Ubuntu 24.04 native or
   robostack). Current user machine status at authoring time: Windows, no
   MATLAB installed - this path cannot run until MATLAB + ROS Toolbox is
   installed and network reachability to a Linux/WSL2 Gazebo host exists.
   MATLAB Online is NOT assumed to reach a local Gazebo.

## Launch order on the sim side
- With repo navigation stack (slam_navigation_hardtest.launch.py): bridge
  nodes are already launched, BUT Nav2's controller also publishes
  /cmd_vel - do NOT drive from MATLAB at the same time. Use this link
  against the plain sim (mir_description gazebo.launch.py) and run the
  bridge nodes from the repo launch file, or disable Nav2's controller.
- With plain gazebo.launch.py: it starts NO ros_gz_bridge (verified -
  only gz sim, robot_state_publisher, spawn). Run the bridge yourself
  with the exact repo arguments listed above, e.g.:
    ros2 run ros_gz_bridge parameter_bridge \
      /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock \
      /cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist \
      /odom@nav_msgs/msg/Odometry[gz.msgs.Odometry \
      /back_scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan \
      /joint_states@sensor_msgs/msg/JointState[gz.msgs.Model
  plus the separate /scan bridge (override_frame_id front_laser_link).

## Files
- mir100_gazebo_link.m - capability-gated connection: cmd_vel publisher,
  scan/odom/(optional ground truth) subscribers with per-topic received
  counts and wall-clock freshness, bounded sendCmdVel (limits exposed in
  link.limits), immediate stop(). Scan readings are transformed from the
  canonical laser mounts (front [0.4288 0.2358] yaw +45 deg, back
  [-0.3548 -0.2352] yaw -135 deg, verified from the flattened ref URDF
  joint origins; sensor pose zero) into BASE-frame hit points - raw ranges
  are laser-frame distances, not base clearance. groundTruthSubscribed
  means the subscriber was created; latestGroundTruth().fresh gates on
  actual data.
- roslink_goto_goal.m - closed-loop go-to-goal client controller consuming
  only observed odom/scans (goalXY declared in the /odom frame). Forward
  motion gates on base-frame hits with x_base > 0; turning requires BOTH
  scans fresh + full 360 deg ray coverage + all-direction clearance
  (full swept footprint 0.6052 + margin), and stops with blind_coverage /
  turn_blocked / unknown_returns when clearance cannot be established.
  NaN returns are unknown - the robot stops. Inf = no return within max
  range (clear). Safe zero-Twist stop on every exit; measurement log
  only. Discrete scans are NOT a collision-free guarantee.

## Braking model (why the speed cap is distance-dependent)
Fixed 0.75 m gating at 1.0 m/s was shown insufficient: stopping at the
plugin's 1.0 m/s^2 needs 0.5 m plus ~0.18 m of latency travel, more than
the 0.145 m gap between gate and footprint edge. The controller therefore
caps v so v*L + v^2/(2*aDec) <= D, with D = nearest forward hit -
footprint radius (0.6052) - safety buffer (0.15), and L = control period
+ measured scan age. Verified sim facts used: DiffDrive sets
max_linear_acceleration 1.0 / max_angular_acceleration 1.5 and NO velocity
limits; lidar update_rate 12.5 Hz (0.08 s), range [0.05, 29], 541 samples
(mir100_ref.urdf.xacro). Assumptions, stated not guaranteed: aDec = 0.5
m/s^2 (50% derated - gz-sim decel symmetry is not verifiable from the
repo); Inf returns treated as clear; rotation does not change radial
obstacle distance (turn gate is radial + coverage).

## Deceleration calibration (MANDATORY before any motion)
roslink_goto_goal ERRORS unless opts.DecelAssumptionAccepted=true, because
the braking bound rests on an UNVERIFIED deceleration assumption
(AssumedLinearDecel, default 0.5 m/s^2 derated from the plugin's
max_linear_acceleration 1.0; gz-sim decel symmetry unverifiable from the
repo). Calibrate on the live system:
1. Open floor: confirm via link.latestScan('front'/'back') that no finite
   return is within 3 m in any direction.
2. Command a steady speed, e.g. link.sendCmdVel(0.5, 0) for ~2 s, then
   link.stop(). Record /odom twist (link.latestOdom().v) and pose over
   time: stopping distance s from the stop command to v=0.
3. measured decel ~ v^2/(2s) (or v / stop_time). Repeat at 0.25 / 0.75 /
   1.0 m/s and take the WORST (lowest) value.
4. Set opts.AssumedLinearDecel to that worst value x 0.8 safety factor,
   then set opts.DecelAssumptionAccepted=true.
Until calibration is done, treat every motion result as unverified
(log.decelVerified is always false from this code).

## Verification status
NOT runtime-verified. No MATLAB and no ROS 2 existed in the authoring
environment. Grounding: MathWorks reference pages (ros2node, ros2publisher,
ros2subscriber, ros2message - Introduced R2019b) and repo source files
cited above. First live run must validate: ros2 topic list shows the
bridged topics, a zero cmd_vel is accepted, odom/scan callbacks fire
(link.isFresh true), before any motion. No navigation, real-time or
collision-avoidance success is claimed by this code.
