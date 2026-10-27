function out = robot_lidar_sensor(action, varargin)
%ROBOT_LIDAR_SENSOR  Isolated adapter to the MathWorks scenario lidar:
%   robotLidarPointCloudGenerator mounted with robotSensor on the spawned MiR100
%   robotPlatform at the canonical URDF link "front_laser_link".
%   Runtime-tested on MATLAB R2025b; see DEBUG_REPORT.md for scope.
%   WorldLidarAdaptor excludes the host robot's opaque scanner housing
%   from sensor targets. World meshes and robot geometry remain intact.
%   Release/products (MathWorks docs): robotSensor + robotPlatform Introduced R2022a;
%   Robotics System Toolbox; robotLidarPointCloudGenerator REQUIRES Computer Vision
%   Toolbox. Errors are NOT swallowed: missing product/API throws
%   'robot_lidar_sensor:unavailable'. No ray-caster fallback exists in this file.
%
%   Sensor model: 3-D organised point cloud (azimuth x few elevation channels, sensor
%   frame). The controller consumes its planar (x,y) projection (nearest return per
%   azimuth column). Default: azimuth [-135 135] deg (270 deg, SICK LMS100-like front
%   laser), 0.5 deg, elevation [0 2] deg x 1 deg (floor plane never hit: sensor is
%   upward of horizontal), MaxRange 8 m, UpdateRate 100 Hz (= build_world scenario rate; must be an integer
%   multiple of the scenario step).
%   Only the FRONT laser is mounted: rear-left blind sector exists (no back laser).
%   The supported scene has one host robot and static world meshes.
%
%   h    = robot_lidar_sensor('attach', plat, cfg)   cfg.robot (rigidBodyTree) REQUIRED
%   scan = robot_lidar_sensor('read', h, pose)       pose=[x y yaw] platform base (world)
%     scan.isUpdated, scan.rngs (Naz x1, planar range from sensor; maxRange=no return),
%     scan.pts (Naz x2 WORLD, NaN no return), scan.rel (azimuth in ROBOT frame, rad),
%     scan.originXY (sensor origin, world)
switch lower(action)
    case 'attach', out = doAttach(varargin{:});
    case 'read',   out = doRead(varargin{:});
    otherwise, error('robot_lidar_sensor:badAction','unknown action %s', action);
end
end

function h = doAttach(plat, cfg)
if nargin < 2 || ~isstruct(cfg) || ~isfield(cfg,'robot') || isempty(cfg.robot)
    error('robot_lidar_sensor:cfg','cfg.robot (rigidBodyTree from load_mir100) is required');
end
d = struct('maxRange',8,'azLimits',[-135 135],'azRes',0.5,'elLimits',[0 2],'elRes',1, ...
    'updateRate',100,'mountBody',"front_laser_link",'hasNoise',false);
for f = fieldnames(d)', if ~isfield(cfg,f{1}) || isempty(cfg.(f{1})), cfg.(f{1}) = d.(f{1}); end, end
req = {'robotSensor','robotLidarPointCloudGenerator','robotPlatform'};
for i = 1:numel(req)
    if isempty(which(req{i})) && exist(req{i},'class') ~= 8
        error('robot_lidar_sensor:unavailable', ...
            '%s not found (needs Robotics System Toolbox R2022a+ and Computer Vision Toolbox).', req{i});
    end
end
if isempty(ver('vision'))
    error('robot_lidar_sensor:unavailable','Computer Vision Toolbox not installed (required by robotLidarPointCloudGenerator).');
end
names = string(cfg.robot.BodyNames);
if ~any(names == string(cfg.mountBody))
    error('robot_lidar_sensor:mount','body %s not in rigidBodyTree', cfg.mountBody);
end
% laser frame -> robot base frame, from the URDF (joint origin rpy/xyz), home config
Tb = getTransform(cfg.robot, homeConfiguration(cfg.robot), char(cfg.mountBody));  % body -> base
model = robotLidarPointCloudGenerator( ...
    UpdateRate=cfg.updateRate, MaxRange=cfg.maxRange, ...
    AzimuthLimits=cfg.azLimits, AzimuthResolution=cfg.azRes, ...
    ElevationLimits=cfg.elLimits, ElevationResolution=cfg.elRes, ...
    HasNoise=cfg.hasNoise, HasOrganizedOutput=true);
% MountingBodyName/Location per robotSensor doc: sensor frame == laser link frame
adaptor = WorldLidarAdaptor(model,Tb);
sensor = robotSensor("front_lidar", plat, adaptor, MountingBodyName=cfg.mountBody, ...
    MountingLocation=[0 0 0], MountingAngles=[0 0 0]);
h = struct('sensor', sensor, 'cfg', cfg, 'Tb', Tb, ...
    'selfExclusion','host robot excluded from ray targets; static meshes retained');
end

function scan = doRead(h, pose)
cfg = h.cfg;
[isUpdated, ~, pc] = read(h.sensor);
scan = struct('isUpdated', logical(isUpdated), 'rngs', [], 'pts', [], 'rel', [], 'originXY', []);
if ~isUpdated, return, end
L = pc.Location;
if ndims(L) ~= 3, error('robot_lidar_sensor:format','expected organized MxNx3 Location'); end
X = L(:,:,1); Y = L(:,:,2); Z = L(:,:,3);
R = hypot(X, Y); R(~isfinite(R)) = inf;          % NaN = no return
[rmin, idx] = min(R, [], 1);                     % nearest return per azimuth column
lin = sub2ind(size(R), idx, 1:size(R,2));
P = [X(lin); Y(lin); Z(lin); ones(1,size(R,2))];
miss = ~isfinite(rmin) | rmin > cfg.maxRange;
P(~isfinite(P)) = 0;
Pr = h.Tb * P;                                   % laser -> robot base frame
c = cos(pose(3)); s = sin(pose(3));
wx = pose(1) + c*Pr(1,:) - s*Pr(2,:);  wy = pose(2) + s*Pr(1,:) + c*Pr(2,:);
o = h.Tb(1:3,4);
scan.originXY = [pose(1) + c*o(1) - s*o(2), pose(2) + s*o(1) + c*o(2)];
scan.rngs = rmin(:); scan.rngs(miss) = cfg.maxRange;
scan.pts  = [wx(:) wy(:)]; scan.pts(miss,:) = NaN;
scan.rel  = atan2(Pr(2,:)', Pr(1,:)');           % direction of return in ROBOT frame
end
