function out = run_real_sensor_reaction(rootDir, opts)
%RUN_REAL_SENSOR_REACTION  End-to-end reaction run with the REAL MathWorks scenario sensor.
%   MATLAB R2025b runtime evidence and limits: see DEBUG_REPORT.md.
%   Chain: build_world (65 wall meshes) -> load_mir100 -> spawn_mir100 ->
%   robotSensor(robotLidarPointCloudGenerator) at front_laser_link -> build_map ->
%   plan_path -> setup(scen) -> lidar_react(lidarMode='robotSensor').
%   The planner map comes from the SAME boxes; an UNSEEN box is added to the scenario
%   (and passed as ground truth) but is NOT in the map, so only the sensor can reveal it.
%   Hard gates (errors, no ray-caster fallback): R2022a+, Robotics System Toolbox,
%   Navigation Toolbox, Computer Vision Toolbox, robotSensor.
%   Read ranges drive: unseen-cell detection -> look-ahead block check -> SE2 footprint
%   replan -> per-command full-polygon check (see lidar_react).
if nargin < 1 || isempty(rootDir), rootDir = fileparts(fileparts(mfilename('fullpath'))); end
if nargin < 2, opts = struct(); end
for folder = {'world','robot','map','planning','collision','reaction'}
    addpath(fullfile(rootDir,folder{1}));
end
if ~isfield(opts,'unseen'), opts.unseen = [-3.9 -1.9 0.25 0.5 0.5 0.5 0.3]; end  % [xc yc zc sx sy sz yaw]
caps = react_capabilities();
need = {'robotScenario','robotSensor','robotLidarPointCloudGenerator'};
ok = [caps.robotScenario, caps.robotSensor, caps.robotLidarPointCloudGenerator];
if isMATLABReleaseOlderThan("R2022a") || ~caps.navToolbox || ~caps.computerVisionToolbox || ~all(ok)
    error('run_real_sensor_reaction:unavailable', ...
        'Real-sensor mode needs R2022a+, Robotics System Toolbox, Navigation Toolbox, Computer Vision Toolbox. caps: %s', jsonencode(caps));
end
csv  = fullfile(rootDir,'assets','hard_world_boxes.csv');
urdf = fullfile(rootDir,'assets','mir_description','urdf','mir100_matlab.urdf');
[scen, boxes] = build_world(csv);                         % UpdateRate 100 Hz
ub = opts.unseen;
if ~isempty(ub)                                          % unseen obstacle: scenario only, NOT in map
    if abs(ub(7)) < 1e-9
        addMesh(scen, "Box", Position=ub(1:3), Size=ub(4:6), Color=[0.9 0.2 0.2], ...
            Collision="mesh", IsBinaryOccupied=true, Name="unseen_box");
    else
        v = [-1 -1 -1;1 -1 -1;1 1 -1;-1 1 -1;-1 -1 1;1 -1 1;1 1 1;-1 1 1].*(ub(4:6)/2);
        R = [cos(ub(7)) -sin(ub(7)) 0;sin(ub(7)) cos(ub(7)) 0;0 0 1];
        v = v*R.' + ub(1:3);
        f = [1 3 2;1 4 3;5 6 7;5 7 8;1 2 6;1 6 5;2 3 7;2 7 6;3 4 8;3 8 7;4 1 5;4 5 8];
        addMesh(scen,"Custom",Vertices=v,Faces=f,Position=[0 0 0],Color=[0.9 0.2 0.2], ...
            Collision="mesh",IsBinaryOccupied=true,Name="unseen_box");
    end
end
robot = load_mir100(urdf);
plat  = spawn_mir100(scen, robot);                       % world [-4.31 -2.81], yaw 0
h     = robot_lidar_sensor('attach', plat, struct('robot', robot));
map   = build_map(boxes, 20, []);                        % cells/m, auto origin
[path, pinfo] = plan_path(map, [-4.31 -2.81], [4.31 0.56], 0.6052);
if ~pinfo.ok || isempty(path), error('run_real_sensor_reaction:noPath','global plan failed: %s', pinfo.reason); end
setup(scen);                                             % required before read/advance
ro = opts; % Honor caller time limits and control options.
ro.boxes = boxes; ro.unseenBoxes = ub; ro.movePlatform = true;
ro.lidarMode = 'robotSensor'; ro.robotLidar = h; ro.lidarRange = 8;
ro.startPose = [-4.31 -2.81 0];
out = lidar_react(plat, scen, path, ro);
out.globalPathInfo = pinfo;
fprintf('real-sensor reaction: success=%d collided=%d replans=%d sensorUpdates=%d simTime=%.1f\n', ...
    out.success, out.collided, out.replanCount, out.robotSensorUpdates, out.simTime);
cellfun(@disp, out.notes);
end
