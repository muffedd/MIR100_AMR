function plat = spawn_mir100(scen, robot, opts)
%SPAWN_MIR100 Add the real MiR100 rigidBodyTree to a robotScenario.
%
%   plat = SPAWN_MIR100(scen, robot) creates a robotPlatform named "mir100"
%   in the robotScenario scen, carrying the actual imported rigidBodyTree
%   from LOAD_MIR100 - the scenario renders the MiR100 STL meshes, not a
%   generic marker - at the project contract spawn pose:
%   world [-4.31 -2.81 0] m, yaw 0 rad (world = map frame + [-4.31 -2.81];
%   scenario ENU x-y plane, yaw CCW from +x, quaternion [w x y z]).
%
%   plat = SPAWN_MIR100(scen, robot, opts) overrides via struct fields:
%     Name        - platform name, default "mir100"
%     Position    - [x y z] world spawn, default [-4.31 -2.81 0]
%     Yaw         - heading rad CCW from +x, default 0
%     Collision   - robotPlatform Collision option, default "default"
%                   (keeps the collision meshes from the URDF so clearance
%                   checks see the real geometry)
%     JointConfig - initial joint configuration vector in robot.DataFormat
%                   order (default: robotPlatform default = homeConfiguration).
%                   Use verify_robot(robot).movableJoints for name<->index.
%
%   Joint animation contract (for follow_route / lidar_react):
%   per-step wheel and caster angles are applied with
%       move(plat, "joint", q)      % q in robot.DataFormat ("row") order
%   Mapping must be BY NAME against verify_robot(robot).movableJoints:
%   drive wheels are 'left_wheel_joint' and 'right_wheel_joint'; the 10
%   caster joints are passive and may be held at 0 in a kinematic demo.
%   Base motion uses move(plat, "base", motion) or a BaseTrajectory
%   (waypointTrajectory); see the robotPlatform move reference page for the
%   motion argument shape. ReferenceFrame "ENU" (robotScenario default).
%
%   Requires: Robotics System Toolbox (robotScenario / robotPlatform /
%   move). APIs verified against the current MathWorks reference pages.

    if nargin < 3 || isempty(opts), opts = struct(); end
    if ~isfield(opts, 'Name'),      opts.Name      = "mir100"; end
    if ~isfield(opts, 'Position'),  opts.Position  = [-4.31 -2.81 0]; end
    if ~isfield(opts, 'Yaw'),       opts.Yaw       = 0; end
    if ~isfield(opts, 'Collision'), opts.Collision = "default"; end
    if ~isfield(opts, 'JointConfig'), opts.JointConfig = []; end

    if ~isa(scen, 'robotScenario')
        error('spawn_mir100:badScenario', 'scen must be a robotScenario object, got %s.', class(scen));
    end
    if ~isa(robot, 'rigidBodyTree')
        error('spawn_mir100:badRobot', 'robot must be a rigidBodyTree from load_mir100, got %s.', class(robot));
    end

    q = [cos(opts.Yaw/2), 0, 0, sin(opts.Yaw/2)]; % [w x y z], yaw about z (ENU up)

    args = {"RigidBodyTree", robot, ...
            "InitialBasePosition", opts.Position, ...
            "InitialBaseOrientation", q, ...
            "Collision", opts.Collision};
    if ~isempty(opts.JointConfig)
        args = [args, {"InitialJointConfiguration", opts.JointConfig}];
    end
    plat = robotPlatform(opts.Name, scen, args{:});
end
