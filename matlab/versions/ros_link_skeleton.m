%% ros_link_skeleton.m
%  OPTIONAL ROS 2 LINK SKELETON - MiR100 AMR project (module: matlab/roslink)
%
%  STATUS: NOT BUILT / NOT EXECUTED. Hard gate OFF; every ROS 2 call is
%  commented out. No MATLAB, no ROS Toolbox, and no ROS 2 / Gazebo system were
%  available in the authoring environment on 2026-10-27. Do not describe this
%  file as tested.
%
%  TARGET: the source project is ROS 2 Jazzy + Gazebo Harmonic (verified from
%  the live README, https://github.com/muffedd/ROS_mir100, 2026-10-27; parent
%  identifies commit 6450a77). Packages: mir_description, mir100_navigation
%  (colcon). The sim runs the Nav2 stack (A* + regulated pure pursuit, SLAM
%  Toolbox, robot_localization).
%
%  ROS 1 DOES NOT APPLY: rosinit/rospublisher/rossubscriber are ROS 1 and are
%  intentionally absent. All calls below are the ROS 2 interface of ROS Toolbox
%  (ros2node/ros2publisher/ros2subscriber/ros2message), each verified as
%  "Introduced in R2019b" on the official pages - so this link is compatible
%  with the R2022a baseline. See VERSION_MATRIX.txt rows 14-17.
%
%  REQUIREMENTS IF ENABLED LATER:
%    - MATLAB R2022a+ with ROS Toolbox (ROS 2 interface OK since R2019b)
%    - Robotics System Toolbox (importrobot, differentialDriveKinematics,
%      controllerPurePursuit) if the model or the loop below is used
%    - The ROS 2 Jazzy sim running on the SAME HOST (the repo launcher sets
%      ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST) or a DDS layer reconfigured
%      for cross-host discovery; matching ROS_DOMAIN_ID
%    - Ubuntu 24.04 / ROS 2 Jazzy side per the repo README

%% 0. Hard gate - leave false until requirements above are verified on the
%%    target machine (`ver`, and a live `ros2 node list` from this host).
ENABLE_ROS_LINK = false;
if ~ENABLE_ROS_LINK
    disp('ros_link_skeleton: gate OFF - NOT BUILT, nothing executed.');
    return;
end

%% 1. Environment (set BEFORE the first ros2 call in the MATLAB session) -----
% setenv('ROS_DOMAIN_ID', '0');   % must match the sim's domain
%  Note: the repo launcher pins ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST, so
%  run MATLAB on the same host unless the launch is changed.

%% 2. Create a ROS 2 node -----------------------------------------------------
% node = ros2node('/mir100_matlab_link');   % Introduced R2019b (ROS Toolbox)

%% 3. Subscribe-first (observe the sim; Nav2 keeps ownership of control) -----
% odomSub = ros2subscriber(node, '/odom', 'nav_msgs/Odometry');
% scanSub = ros2subscriber(node, '/scan', 'sensor_msgs/LaserScan');
% mapSub  = ros2subscriber(node, '/map',  'nav_msgs/OccupancyGrid');
%  Topic names are the Nav2/repo defaults - CONFIRM with `ros2 topic list`
%  on the running system before enabling; do not assume.

%% 4. OPTIONAL velocity publisher (only when Nav2 is NOT commanding) ---------
%  Publishing /cmd_vel while Nav2's regulated pure pursuit is active creates
%  two commanders fighting one base. Enable this block only for a manual
%  override mode with Nav2 stopped or the controller server inactive.
% cmdPub = ros2publisher(node, '/cmd_vel', 'geometry_msgs/Twist');
% cmdMsg = ros2message(cmdPub);

%% 5. SAME MiR100 model for MATLAB-side reasoning ----------------------------
%  URDF/meshes come from the colcon workspace checkout; do NOT remodel.
% mirUrdf = fullfile('<path-to>/ROS_mir100/src/mir_description/urdf/mir100.urdf');
% robot   = importrobot(mirUrdf, 'MeshPath', '<path-to>/ROS_mir100/src/mir_description');
% robot.DataFormat = 'row';
% showdetails(robot);            % sanity-check joint tree against the URDF

%% 6. World handling (unchanged contract) ------------------------------------
%  The Gazebo Harmonic world (hard_world.sdf) is NOT imported into MATLAB.
%  MATLAB-side planning ground truth stays the shared contract:
%      boxes = read_world_boxes(csv);   % Nx [xc yc zc sx sy sz yaw], m/rad
%      map   = build_map(scen, res, originXY);
%  Gazebo keeps simulating the world; MATLAB only exchanges ROS 2 topics.

%% 7. Control loop sketch (override mode only; see section 4 warning) --------
%  wheelRadius = 0.0625;   % m  (MiR100 build parameters)
%  trackWidth  = 0.4452;   % m  (wheel separation)
%  kin = differentialDriveKinematics('WheelRadius', wheelRadius, ...
%                                  'TrackWidth',  trackWidth, ...
%                                  'VehicleInputs', 'VehicleSpeedHeadingRate');
%  pp = controllerPurePursuit('Waypoints', wpsWorld, ...
%                             'DesiredLinearVelocity', 0.5, ...
%                             'LookaheadDistance', 0.35, ...
%                             'MaxAngularVelocity', 1.5);   % R2022a naming
%  while ~goalReached
%      odom = receive(odomSub, 2);
%      pose = odomToPoseXYYaw(odom);        % local helper, TODO
%      [v, omega] = pp(pose);               % R2022a: 2nd output IS angular vel
%      % WARNING (verified, see matrix F3): from R2026a the property is
%      % MaxCurvature and the 2nd output is CURVATURE, not angular velocity.
%      % On R2026a+: omega = curvature * v. Branch on version or convert.
%      cmdMsg.linear.x  = v;
%      cmdMsg.angular.z = omega;
%      send(cmdPub, cmdMsg);
%      pause(0.1);                          % soft rate only, NOT real-time
%  end

%% 8. Custom messages ---------------------------------------------------------
%  OMITTED: the repo's interfaces are standard msg/srv types (Nav2, sensor,
%  geometry). If a custom type ever appears, ROS 2 custom messages need
%  ros2genmsg - its release requirement is NOT verified here; verify on the
%  official page before relying on it at any baseline.

%% 9. Cleanup -----------------------------------------------------------------
%  ROS 2 has no rosshutdown (that is ROS 1). Clear the ROS 2 entities:
% clear cmdPub odomSub scanSub mapSub node
