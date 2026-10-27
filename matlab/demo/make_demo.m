function manifest = make_demo(csvPath, urdfPath, outDir, opts)
%MAKE_DEMO Render the MiR100 AMR demo from actual contract outputs.
%   manifest = MAKE_DEMO(csvPath, urdfPath, outDir, opts) builds the world
%   and robot through the project's integration contracts, runs the
%   kinematic route follower, and renders:
%     * PRIMARY: the ACTUAL imported rigidBodyTree (visual meshes) animated
%       INSIDE the actual hard_world boxes via robotScenario/robotPlatform/
%       show3D: per executed trajectory pose, move("base",motion16) for the
%       chassis AND move("joint",config) for the name-mapped wheel joints
%       driven by trajLog.wheelAngles(:,[left right]);
%     * a top-down 2D animation (world boxes, planned route v2, executed
%       trajectory, v2 footprint + heading, clearance status);
%     * static rigidBodyTree proof views;
%   all video written with LOSSLESS VideoWriter profiles. No user media is
%   recompressed anywhere in this pipeline.
%
%   Integration contracts consumed (owned by sibling modules, NOT by demo):
%     boxes    = read_world_boxes(csvPath)   % Nx7 [xc yc zc sx sy sz yaw], m/rad
%     scen     = build_world(csvPath)
%     robot    = load_mir100(urdfPath)       % canonical flat asset, matlab/assets
%     plat     = spawn_mir100(scen, robot)
%     trajLog  = follow_route(plat, wpsWorld, simOpts)
%                -> struct with fields t, x, y, yaw, wheelAngles(:,[left right])
%                simOpts is FLAT: dt, maxLinVel, maxAngVel, wpRadius,
%                goalRadius, maxSimTime, scenario (handle). No nesting.
%     clearLog = check_run_clearance(trajLog, boxes)  % optional overlay
%                canonical schema: minOverall, minDist, t, minAt, collision
%
%   Robot asset: pass urdfPath (argument or opts.urdfPath) pointing at the
%   canonical flat URDF under the matlab/assets tree (project
%   supplies it). make_demo ERRORS when no explicit asset is given; it
%   never picks an arbitrary file.
%
%   STATUS: authored, statically reviewed, MISS_HIT-linted; NOT yet
%   executed in MATLAB (no MATLAB runtime available to this module author).
%   See README.txt. Headless runs produce NO video - keyframe PNGs only,
%   stated bluntly in the manifest.
%
%   Verified against MathWorks docs (2026-10-27, URLs in README.txt):
%     VideoWriter 'Uncompressed AVI' = uncompressed RGB24 (lossless);
%     'Archival' = lossless Motion JPEG 2000. MPEG-4 not on Linux: unused.
%     importrobot: URDF/Xacro/SDF -> rigidBodyTree (Robotics System Toolbox).
%     robotScenario/robotPlatform/addMesh/show3D/move (R2022a+):
%       addMesh(scenario,"Box",Size=[sx sy sz],Position=[xc yc zc],Color=..)
%       addMesh(scenario,"Custom",Vertices=V,Faces=F,Color=..)
%       platform = robotPlatform(name,scenario,RigidBodyTree=robot)
%       move(platform,"base",motion16), motion16 = [x y z vx vy vz
%         ax ay az qw qx qy qz wx wy wz]  (quaternion w-first)
%       move(platform,"joint",config)
%       [ax,~] = show3D(scenario); show3D(scenario,FastUpdate=true,Parent=ax)

% ---------------- arguments and defaults --------------------------------
demoDir     = fileparts(mfilename('fullpath'));
projectRoot = fileparts(demoDir);
if nargin < 1 || isempty(csvPath)
    csvPath = fullfile(projectRoot, 'assets', 'hard_world_boxes.csv');
end
if nargin < 2
    urdfPath = '';
end
if nargin < 3 || isempty(outDir)
    outDir = fullfile(demoDir, 'out');
end
if nargin < 4 || isempty(opts)
    opts = struct();
end
% Robot asset: explicit argument wins, then opts.urdfPath (lead-supplied,
% canonical flat URDF under the matlab/assets tree). Never auto-pick.
if isempty(urdfPath) && isfield(opts, 'urdfPath') && ~isempty(opts.urdfPath)
    urdfPath = opts.urdfPath;
end
if isempty(urdfPath)
    error('make_demo:noRobotAsset', ['No robot asset given. Supply the ' ...
        'canonical flat URDF from the matlab/assets tree via the urdfPath ' ...
        'argument or opts.urdfPath (project owns it).']);
end
opts = withDefaults(opts);
if ~exist(outDir, 'dir'), mkdir(outDir); end

manifest = struct();
manifest.created       = datestr(now, 31);
manifest.matlabVersion = version;
manifest.outDir        = outDir;
manifest.urdfPath      = urdfPath;
manifest.footprintUsed = opts.footprintVersion;
manifest.notes         = {};

% ---------------- contract dependency check ------------------------------
required = {'read_world_boxes', 'build_world', 'load_mir100', ...
            'spawn_mir100', 'follow_route'};
missing = required(cellfun(@(f) exist(f, 'file') ~= 2, required));
if ~isempty(missing)
    error('make_demo:missingContracts', ...
        ['Missing contract function(s): %s.\nRun from the integrated ' ...
         'project root with all modules on the path.'], strjoin(missing, ', '));
end
haveClearance = exist('check_run_clearance', 'file') == 2;
if ~haveClearance
    manifest.notes{end+1} = ...
        'check_run_clearance not on path: clearance overlay skipped (optional).';
end

% ---------------- run the actual pipeline --------------------------------
fprintf('make_demo: reading world boxes from %s\n', csvPath);
boxes = read_world_boxes(csvPath);
assert(isnumeric(boxes) && size(boxes, 2) == 7, ...
    'make_demo:badBoxes', 'read_world_boxes must return Nx7 [xc yc zc sx sy sz yaw].');
manifest.nBoxes = size(boxes, 1);

scen  = build_world(csvPath);
robot = load_mir100(urdfPath);
plat  = spawn_mir100(scen, robot);
setup(scen);   % documented lifecycle: setup BEFORE any advance/read

if isfield(opts, 'trajLog') && ~isempty(opts.trajLog)
    % CORE mode: render the ACTUAL trajectory the caller ran. Never rerun.
    trajLog = opts.trajLog;
    assert(isstruct(trajLog) && all(isfield(trajLog, {'t','x','y','yaw'})), ...
        'make_demo:badTrajLog', 'opts.trajLog must have fields t/x/y/yaw.');
    manifest.notes{end+1} = 'Rendered caller-supplied trajLog (core run).';
else
    % STANDALONE mode only: run its own follow_route on default waypoints.
    simOpts = struct('dt', opts.dt, 'maxLinVel', opts.maxLinVel, ...
        'maxAngVel', opts.maxAngVel, 'wpRadius', opts.wpRadius, ...
        'goalRadius', opts.goalRadius, 'maxSimTime', opts.maxSimTime, ...
        'scenario', scen);
    fprintf('make_demo: STANDALONE run, following %d default waypoints\n', ...
        size(opts.wpsWorld, 1));
    trajLog = follow_route(plat, opts.wpsWorld, simOpts);
    manifest.notes{end+1} = ['STANDALONE: demo executed its own ' ...
        'follow_route on default waypoints (not a core-run trajectory).'];
end
% Regression guard: a caller-supplied trajLog must never be overwritten.
if isfield(opts, 'trajLog') && ~isempty(opts.trajLog)
    assert(isequal(trajLog, opts.trajLog), 'make_demo:trajLogReplaced', ...
        'internal error: caller trajLog was replaced.');
end
need = {'t', 'x', 'y', 'yaw'};
for k = 1:numel(need)
    assert(isfield(trajLog, need{k}), 'make_demo:badTrajLog', ...
        'follow_route output missing required field ''%s''.', need{k});
end
t = trajLog.t(:); x = trajLog.x(:); y = trajLog.y(:); th = trajLog.yaw(:);
nS = numel(t);
manifest.nTrajSamples = nS;
manifest.simTime      = t(end) - t(1);
manifest.routeLength  = sum(hypot(diff(x), diff(y)));

% canonical clearLog schema: minOverall, minDist, t, minAt, collision
minClear = NaN; collided = false;
if haveClearance
    clearLog = check_run_clearance(trajLog, boxes);
    if isstruct(clearLog)
        if isfield(clearLog, 'minOverall') && ~isempty(clearLog.minOverall)
            minClear = clearLog.minOverall;
        elseif isfield(clearLog, 'minDist') && ~isempty(clearLog.minDist)
            minClear = min(clearLog.minDist(:));
        else
            manifest.notes{end+1} = ...
                'clearLog has neither minOverall nor minDist: no clearance number rendered.';
        end
        if isfield(clearLog, 'collision'), collided = logical(clearLog.collision); end
        if isfield(clearLog, 'minAt'), manifest.clearanceMinAt = clearLog.minAt; end
        if isfield(clearLog, 't'),      manifest.clearanceT     = clearLog.t;     end
    else
        manifest.notes{end+1} = sprintf( ...
            'clearLog is not a struct (class %s): clearance overlay skipped.', class(clearLog));
    end
end
manifest.minClearance = minClear;
manifest.collision    = collided;

% Grounded base height: preserve the spawn height, never assume it.
[baseZ, baseZSource] = resolveBaseZ(robot, plat, opts);
manifest.baseZ       = baseZ;
manifest.baseZSource = baseZSource;

canVideo = usejava('desktop');
% The newer desktop can display figures without a Java desktop.
manifest.showAnimation = opts.showAnimation;
figVis = 'on';
if ~opts.showAnimation, figVis = 'off'; end
if canVideo
    manifest.captureMode = 'lossless video + PNG keyframes';
else
    if opts.showAnimation
        manifest.captureMode = 'Interactive playback + PNG keyframes; no video capture available.';
    else
        manifest.captureMode = ['HEADLESS: NO video produced; keyframe PNGs only. ' ...
            'Do not present keyframes as the video demo.'];
    end
    manifest.notes{end+1} = manifest.captureMode;
end

% ---------------- top-down 2D animation ----------------------------------
fig = figure('Visible', figVis, 'Color', 'w', 'Position', [50 50 1100 820]);
ax  = axes('Parent', fig); hold(ax, 'on'); axis(ax, 'equal');
pad = 1.0;
xlim(ax, [min(boxes(:,1)) - pad, max(boxes(:,1)) + pad]);
ylim(ax, [min(boxes(:,2)) - pad, max(boxes(:,2)) + pad]);
xlabel(ax, 'x (m, world)'); ylabel(ax, 'y (m, world)');
grid(ax, 'on');

hWall = [];
for i = 1:size(boxes, 1)
    h = drawBox(ax, boxes(i, :), [0.78 0.78 0.82]);
    if isempty(hWall), hWall = h; end   % first wall patch = legend proxy
end
routeXY = opts.wpsWorld;
if isfield(opts, 'trajLog') && ~isempty(opts.trajLog) && ...
        isfield(opts.trajLog, 'waypoints') && ~isempty(opts.trajLog.waypoints)
    routeXY = opts.trajLog.waypoints;   % the route actually followed
end
hRoute = plot(ax, routeXY(:,1), routeXY(:,2), 'k--', 'LineWidth', 1.0);
hSpawn = plot(ax, opts.wpsWorld(1,1),  opts.wpsWorld(1,2),  'go', ...
    'MarkerFaceColor', 'g', 'MarkerSize', 9);
hGoal  = plot(ax, opts.wpsWorld(end,1), opts.wpsWorld(end,2), 'rp', ...
    'MarkerFaceColor', 'r', 'MarkerSize', 12);
trace = plot(ax, x(1), y(1), 'b-', 'LineWidth', 1.6);
foot  = patch(ax, 'XData', nan, 'YData', nan, ...
    'FaceColor', [0.2 0.5 0.9], 'FaceAlpha', 0.45, 'EdgeColor', 'b');
head  = plot(ax, [0 0], [0 0], 'b-', 'LineWidth', 1.6);
legend(ax, [hWall hRoute hSpawn hGoal trace foot head], ...
    {'walls', 'planned route', 'spawn', 'goal', ...
     'executed trajectory', 'robot footprint (v2)', 'heading'}, ...
    'Location', 'best');

[vw2, vf2, prof2] = openLosslessWriter(outDir, 'mir100_demo_topdown', opts, canVideo);
manifest.topdownVideo = vf2;
manifest.keyframes2D  = {};

stride = max(1, round(opts.frameStride));
for k = 1:stride:nS
    R  = [cos(th(k)) -sin(th(k)); sin(th(k)) cos(th(k))];
    fp = (R * opts.footprint.').' + [x(k) y(k)];
    set(foot, 'XData', fp(:,1), 'YData', fp(:,2));
    set(head, 'XData', [x(k), x(k) + cos(th(k))*opts.footprintRadius], ...
              'YData', [y(k), y(k) + sin(th(k))*opts.footprintRadius]);
    set(trace, 'XData', x(1:k), 'YData', y(1:k));
    if ~isnan(minClear)
        if collided, cstr = 'COLLISION'; else cstr = 'clear'; end
        title(ax, sprintf('MiR100 route following  t = %.2f s  (min clearance %.3f m, %s)', ...
            t(k), minClear, cstr));
    else
        title(ax, sprintf('MiR100 route following  t = %.2f s', t(k)));
    end
    drawnow;
    if canVideo, writeVideo(vw2, getframe(fig)); end
    if opts.saveKeyframes && (k == 1 || mod(k, opts.keyframeEvery) == 0 || k + stride > nS)
        png = sprintf('keyframe2d_%05d.png', k);
        print(fig, fullfile(outDir, png), '-dpng', '-r120');
        manifest.keyframes2D{end+1} = png;
    end
end
if ~isempty(vw2), close(vw2); end

% final overview still (full executed trajectory)
set(trace, 'XData', x, 'YData', y);
set(foot, 'XData', nan, 'YData', nan);
set(head, 'XData', nan, 'YData', nan);
title(ax, 'MiR100 hard-world route: planned (dashed) vs executed (blue)');
print(fig, fullfile(outDir, 'trajectory_overview.png'), '-dpng', '-r150');
manifest.overviewPng = 'trajectory_overview.png';
close(fig);

% ---------------- 3D scene: actual robot meshes inside actual world ------
[vf3, kf3, note3, prof3, wheelForm] = renderScene3D(boxes, robot, trajLog, ...
    t, x, y, th, outDir, opts, canVideo, figVis, minClear, collided, baseZ);
manifest.sceneVideo   = vf3;
manifest.sceneProfile = prof3;
manifest.keyframes3D  = kf3;
manifest.wheelAnim    = wheelForm;
manifest.notes        = [manifest.notes, note3];
manifest.videoProfile = prof2;

% ---------------- imported-robot proof stills -----------------------------
manifest.robotPngs = renderRobotViews(robot, outDir, figVis);

% ---------------- manifest -----------------------------------------------
tb = ver;
manifest.toolboxes = {tb.Name};
fid = fopen(fullfile(outDir, 'demo_manifest.txt'), 'w');
if fid >= 0
    fprintf(fid, 'make_demo manifest\n');
    fprintf(fid, 'created:          %s\n', manifest.created);
    fprintf(fid, 'matlab version:   %s\n', manifest.matlabVersion);
    fprintf(fid, 'capture mode:     %s\n', manifest.captureMode);
    fprintf(fid, 'csv:              %s\n', csvPath);
    fprintf(fid, 'robot asset:      %s\n', manifest.urdfPath);
    fprintf(fid, 'footprint:        %s\n', manifest.footprintUsed);
    fprintf(fid, 'base z (m):       %.4f  [%s]\n', manifest.baseZ, manifest.baseZSource);
    fprintf(fid, 'boxes:            %d\n', manifest.nBoxes);
    fprintf(fid, 'traj samples:     %d\n', manifest.nTrajSamples);
    fprintf(fid, 'sim time (s):     %.3f\n', manifest.simTime);
    fprintf(fid, 'route length (m): %.3f\n', manifest.routeLength);
    fprintf(fid, 'min clearance (m): %.4f\n', manifest.minClearance);
    fprintf(fid, 'collision:        %d\n', manifest.collision);
    fprintf(fid, 'wheel animation:  %s\n', manifest.wheelAnim);
    fprintf(fid, '3D scene video:   %s\n', charOrNone(manifest.sceneVideo));
    fprintf(fid, 'topdown video:    %s\n', charOrNone(manifest.topdownVideo));
    fprintf(fid, 'video profiles:   3D=%s topdown=%s (lossless)\n', ...
        charOrNone(manifest.sceneProfile), charOrNone(manifest.videoProfile));
    fprintf(fid, 'robot views:      %d\n', numel(manifest.robotPngs));
    for i = 1:numel(manifest.notes)
        fprintf(fid, 'note:             %s\n', manifest.notes{i});
    end
    fclose(fid);
end
fprintf('make_demo: done. Outputs in %s\n', outDir);
end

% ======================= local functions =================================

function [baseZ, source] = resolveBaseZ(robot, plat, opts)
% Grounded base height, in priority order:
%   1. opts.baseZ            - explicit project override;
%   2. plat.InitialBasePosition(3) - the ACTUAL spawn base height set by
%      spawn_mir100 (preserve it);
%   3. URDF-derived: lowest body frame in home configuration minus one
%      wheel radius (wheel/caster axles sit one radius above contact).
% Never a bare assumed constant.
if isfield(opts, 'baseZ') && ~isempty(opts.baseZ)
    baseZ = opts.baseZ; source = 'opts.baseZ override';
    return;
end
if isobject(plat) && isprop(plat, 'InitialBasePosition')
    p0 = plat.InitialBasePosition;
    if isnumeric(p0) && numel(p0) >= 3
        baseZ = p0(3); source = 'spawn base height (plat.InitialBasePosition)';
        return;
    end
end
baseZ = 0; source = 'URDF-derived (drive-wheel axle height minus wheel radius)';
try
    % Only the drive-wheel axles sit one wheel radius above the floor;
    % base_link and sensor frames would wrongly lift the robot.
    cfg = homeConfiguration(robot);
    minZ = inf;
    for i = 1:numel(robot.Bodies)
        nm = lower(robot.Bodies{i}.Name);
        if ~contains(nm, 'wheel') || contains(nm, 'caster'), continue; end
        Tb = getTransform(robot, cfg, robot.Bodies{i}.Name);
        minZ = min(minZ, Tb(3,4));
    end
    derived = -(minZ - opts.wheelRadius);
    if isfinite(derived) && abs(derived) <= 0.5
        baseZ = derived;
    end
catch
    % keep 0 and the source string; recorded in the manifest
end
end

function [cfg0, iL, iR, note] = wheelJointMap(robot, opts)
% Map trajLog.wheelAngles(:,[left right]) onto the actual URDF joint
% names. Upstream names (empty xacro prefix) are 'left_wheel_joint' and
% 'right_wheel_joint' (confirmed in mir.urdf.xacro's diff-controller
% comment); opts.wheelJointNames = {left,right} overrides explicitly.
% DataFormat is 'row': homeConfiguration is a NUMERIC vector; collect the
% non-fixed joint names in body order (the 'row' config order) instead.
cfg0 = homeConfiguration(robot);
names = {};
for bi = 1:numel(robot.Bodies)
    jb = robot.Bodies{bi}.Joint;
    if ~strcmp(jb.Type, 'fixed'), names{end+1} = lower(char(jb.Name)); end %#ok<AGROW>
end
assert(numel(cfg0) == numel(names), 'make_demo:configOrder', ...
    'row-format config length must equal the non-fixed joint count.');
iL = []; iR = []; note = '';
if isfield(opts, 'wheelJointNames') && numel(opts.wheelJointNames) == 2
    iL = find(strcmp(names, lower(char(opts.wheelJointNames{1}))), 1);
    iR = find(strcmp(names, lower(char(opts.wheelJointNames{2}))), 1);
    if isempty(iL) || isempty(iR)
        note = 'opts.wheelJointNames did not match URDF joints: wheel animation off.';
        iL = []; iR = [];
    end
else
    iL = find(strcmp(names, 'left_wheel_joint'), 1);
    iR = find(strcmp(names, 'right_wheel_joint'), 1);
    if isempty(iL)
        iL = find(contains(names, 'left') & contains(names, 'wheel') & ...
            ~contains(names, 'caster'), 1);
    end
    if isempty(iR)
        iR = find(contains(names, 'right') & contains(names, 'wheel') & ...
            ~contains(names, 'caster'), 1);
    end
    if isempty(iL) || isempty(iR)
        note = 'wheel joints not found by name in URDF: wheel animation off.';
    end
end
end

function opts = withDefaults(opts)
d = struct( ...
    'showAnimation',  ~batchStartupOptionUsed, ...
    'fps',            15, ...
    'frameStride',    [], ...
    'saveKeyframes',  true, ...
    'keyframeEvery',  30, ...
    'videoProfile',   'Uncompressed AVI', ...
    'dt',             0.01, ...   % matches world UpdateRate=100
    'maxLinVel',      0.9, ...
    'maxAngVel',      1.5, ...
    'wpRadius',       0.15, ...
    'goalRadius',     0.05, ...
    'maxSimTime',     120, ...
    'wheelRadius',    0.0625, ... % xacro-verified; baseZ derivation only
    'wallDisplayHeight', 0.6, ... % 3D view only: real 2.2 m walls hide the 0.35 m robot
    'view3D',         [-30 55], ...
    'footprintRadius', 0.6052, ...
    'footprintVersion', 'v2 caster-full-coverage (project, 2026-10-27); radius 0.6052', ...
    'footprint', [-0.474026970 -0.236085370; -0.439392950 -0.319792950; ...
                 -0.355791680 -0.354382980;  0.493049750 -0.295637190; ...
                  0.548024070  0.236692380;  0.513392950  0.320392950; ...
                  0.429789310  0.354983960; -0.417838050  0.295633920], ...
    'wpsWorld', routeV2World());
f = fieldnames(d);
for i = 1:numel(f)
    if ~isfield(opts, f{i}), opts.(f{i}) = d.(f{i}); end
end
if isempty(opts.frameStride)
    opts.frameStride = max(1,round(1/(opts.dt*opts.fps)));
end
end

function wps = routeV2World()
% Route v2, map frame (contract), converted with world = map + [-4.31 -2.81].
wpsMap = [0 0; 0 1.12; 0 2.81; 0.56 3.09; 1.12 3.37; 1.12 4.5; ...
          1.12 5.62; 2.25 5.62; 3.37 5.62; 4.5 5.62; 5.25 5.25; ...
          5.25 4.12; 5.25 3.37; 6.75 3.37; 7.87 3.37; 8.62 3.37];
wps = wpsMap + [-4.31 -2.81];
end

function h = drawBox(ax, row, faceColor)
xc = row(1); yc = row(2); sx = row(4); sy = row(5); yaw = row(7);
c = [ sx/2  sx/2 -sx/2 -sx/2; ...
      sy/2 -sy/2 -sy/2  sy/2];
R = [cos(yaw) -sin(yaw); sin(yaw) cos(yaw)];
p = R * c;
h = patch(ax, 'XData', xc + p(1,:), 'YData', yc + p(2,:), ...
    'FaceColor', faceColor, 'EdgeColor', [0.4 0.4 0.45]);
end

function [V, F] = yawBoxMesh(b)
% 8-vertex 12-triangle box, yaw-rotated about z (addMesh "Custom" path;
% the current hard_world has yaw = 0 on all 65 boxes, so the plain "Box"
% path is the one actually exercised today).
xc = b(1); yc = b(2); zc = b(3); sx = b(4); sy = b(5); sz = b(6); yaw = b(7);
hx = sx/2; hy = sy/2; hz = sz/2;
V0 = [-hx -hy -hz;  hx -hy -hz;  hx  hy -hz; -hx  hy -hz; ...
      -hx -hy  hz;  hx -hy  hz;  hx  hy  hz; -hx  hy  hz];
Rz = [cos(yaw) -sin(yaw) 0; sin(yaw) cos(yaw) 0; 0 0 1];
V = (Rz * V0.').' + [xc yc zc];
F = [1 2 3; 1 3 4;  5 7 6; 5 8 7;  1 6 2; 1 5 6; ...
     2 7 3; 2 6 7;  3 8 4; 3 7 8;  4 5 1; 4 8 5];
end

function [videoFile, keyframes, notes, profileUsed, wheelForm] = renderScene3D( ...
    boxes, robot, trajLog, t, x, y, th, outDir, opts, canVideo, figVis, minClear, collided, baseZ)
% Animate the ACTUAL imported rigidBodyTree (visual meshes) INSIDE the
% actual world boxes: move("base",motion16) per executed pose plus
% move("joint",config) for the name-mapped wheel joints.
videoFile = ''; keyframes = {}; notes = {}; profileUsed = ''; wheelForm = 'off';
if isempty(which('robotScenario'))
    notes{end+1} = ['robotScenario unavailable (needs Robotics System ' ...
        'Toolbox R2022a+): 3D scene animation skipped. 2D top-down and ' ...
        'robot stills still produced.'];
    return;
end
try
    scenario = robotScenario(UpdateRate = 1 / max(opts.dt, eps));
    addMesh(scenario, "Plane", Size = [14.5 11.5], ...
        Color = [0.88 0.88 0.88], Position = [0 0 -0.02]);
    % Display-only wall height. Full-height walls occlude the robot, plan and
    % trace from every useful camera angle; planning/collision are 2D and
    % unaffected.
    for i = 1:size(boxes, 1)
        b = boxes(i, :);
        if ~isempty(opts.wallDisplayHeight) && opts.wallDisplayHeight > 0
            b(6) = min(b(6), opts.wallDisplayHeight); b(3) = b(6) / 2;
        end
        if abs(b(7)) < 1e-9
            addMesh(scenario, "Box", Size = [b(4) b(5) b(6)], ...
                Position = [b(1) b(2) b(3)], Color = [0.55 0.57 0.62]);
        else
            [V, F] = yawBoxMesh(b);
            addMesh(scenario, "Custom", Vertices = V, Faces = F, ...
                Color = [0.55 0.57 0.62]);
        end
    end
    platform = robotPlatform("mir100", scenario, ...
        RigidBodyTree = robot, ReferenceFrame = "ENU");
    try
        updateMesh(platform, "RigidBodyTree", Object = robot);
    catch
        % RigidBodyTree was already attached at construction.
    end
    setup(scenario);   % this scenario is created HERE - set it up before
                       % any move()/show3D (caller set up a DIFFERENT one)

    [cfg0, iL, iR, wheelNote] = wheelJointMap(robot, opts);
    if ~isempty(wheelNote), notes{end+1} = wheelNote; end
    wa = [];
    if isfield(trajLog, 'wheelAngles') && isnumeric(trajLog.wheelAngles) && ...
            size(trajLog.wheelAngles, 2) >= 2 && ~isempty(iL) && ~isempty(iR)
        wa = trajLog.wheelAngles;   % columns [left right] per contract
        wheelForm = 'vector';   % DataFormat 'row': numeric joint config
    elseif ~isempty(iL)
        notes{end+1} = 'trajLog.wheelAngles missing/invalid: wheel animation off.';
    end

    ax3 = show3D(scenario);
    fig3 = ancestor(ax3, 'figure');
    set(fig3, 'Visible', figVis, 'Color', 'w', 'Position', [70 70 1100 820], ...
        'Name', 'MiR100 3D navigation', 'NumberTitle', 'off');
    axis(ax3, 'equal');
    zTop = 2.4;
    if ~isempty(opts.wallDisplayHeight) && opts.wallDisplayHeight > 0
        zTop = max(1.0, opts.wallDisplayHeight + 0.4);
    end
    xlim(ax3, [-6.6 6.6]); ylim(ax3, [-5.1 5.1]); zlim(ax3, [-0.05 zTop]);
    view(ax3, opts.view3D);
    camlight(ax3, 'headlight');
    hold(ax3,'on');
    route=opts.wpsWorld;
    if isfield(trajLog,'waypoints') && ~isempty(trajLog.waypoints),route=trajLog.waypoints;end
    ov = drawNavOverlay3D(ax3, route, x, y, th, 1, opts);
    % Readable in a dark MATLAB desktop theme as well.
    set(ax3, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'ZColor', 'k');
    if ~isempty(opts.wallDisplayHeight) && opts.wallDisplayHeight > 0
        notes{end+1} = sprintf(['3D view draws walls %.2f m tall (real 2.20 m) ' ...
            'so the robot and path stay visible; display only.'], opts.wallDisplayHeight);
    end

    [vw3, videoFile, profileUsed] = openLosslessWriter(outDir, ...
        'mir100_demo_3d', opts, canVideo);

    nS = numel(t);
    stride = max(1, round(opts.frameStride));
    playbackTimer = tic;
    for k = unique([1:stride:nS, nS])
        if ~isgraphics(ax3), break; end % Closing the view stops playback.
        qw = cos(th(k)/2); qz = sin(th(k)/2);
        motion = [x(k) y(k) baseZ, zeros(1,3), zeros(1,3), ...
                  qw 0 0 qz, zeros(1,3)];   % 16-elem, quaternion w-first
        move(platform, "base", motion);
        if ~strcmp(wheelForm, 'off') && k <= size(wa, 1)
            if strcmp(wheelForm, 'struct') || strcmp(wheelForm, 'vector')
                % row format: one numeric config vector, wheels by index
                try
                    cfgv = cfg0;
                    cfgv(iL) = wa(k, 1); cfgv(iR) = wa(k, 2);
                    move(platform, "joint", cfgv);
                catch
                    notes{end+1} = ['move("joint",...) rejected struct and ' ...
                        'vector configs: body motion only, wheels static.'];
                    wheelForm = 'off';
                end
            end
        end
        show3D(scenario, FastUpdate = true, Parent = ax3);
        if ~isgraphics(ax3), break; end
        % show3D may rebuild the axes contents; redraw the overlay if so.
        if ~all(isgraphics([ov.plan ov.goal ov.trace ov.foot ov.head]))
            hold(ax3, 'on');
            ov = drawNavOverlay3D(ax3, route, x, y, th, k, opts);
        else
            updateNavOverlay3D(ov, x, y, th, k, opts);
        end
        if ~isnan(minClear)
            if collided, cstr = 'COLLISION'; else cstr = 'clear'; end
            title(ax3, sprintf('MiR100 3D navigation  t = %.2f s  (min clearance %.3f m, %s)', ...
                t(k), minClear, cstr), 'Color', 'k');
        else
            title(ax3, sprintf('MiR100 3D navigation  t = %.2f s', t(k)), 'Color', 'k');
        end
        drawnow;
        if opts.showAnimation
            pause(max(0,t(k)-t(1)-toc(playbackTimer)));
        end
        if canVideo, writeVideo(vw3, getframe(fig3)); end
        if opts.saveKeyframes && (k == 1 || mod(k, opts.keyframeEvery) == 0 || k + stride > nS)
            png = sprintf('keyframe3d_%05d.png', k);
            print(fig3, fullfile(outDir, png), '-dpng', '-r120');
            keyframes{end+1} = png;
        end
    end
    if ~isempty(vw3), close(vw3); end
catch ME
    notes{end+1} = sprintf( ...
        '3D scene animation failed (%s): %s. 2D top-down and robot stills remain.', ...
        ME.identifier, ME.message);
    % Do not fail silently: the 3D view is the primary demo.
    warning('make_demo:scene3DFailed', '%s', notes{end});
    videoFile = ''; profileUsed = ''; wheelForm = 'off';
end
end

function ov = drawNavOverlay3D(ax, route, x, y, th, k, opts)
% Plan, goal, start, executed trace and a footprint outline that follows
% the robot, drawn just above the floor so they show between the walls.
z = 0.03;
ov.plan  = plot3(ax, route(:,1), route(:,2), z*ones(size(route,1),1), 'k--', ...
    'LineWidth', 1.5, 'Tag', 'NavigationPlan');
ov.start = plot3(ax, x(1), y(1), z, 'go', 'MarkerFaceColor', 'g', ...
    'MarkerSize', 9, 'Tag', 'NavigationStart');
ov.goal  = plot3(ax, route(end,1), route(end,2), z, 'rp', 'MarkerFaceColor', 'r', ...
    'MarkerSize', 16, 'Tag', 'NavigationGoal');
ov.goalPole = plot3(ax, route(end,1)*[1 1], route(end,2)*[1 1], [0 1.0], 'r-', ...
    'LineWidth', 2, 'Tag', 'NavigationGoalPole');
ov.trace = plot3(ax, x(1), y(1), z, 'b-', 'LineWidth', 2.2, 'Tag', 'NavigationTrace');
ov.foot  = plot3(ax, nan, nan, nan, '-', 'Color', [0.1 0.4 0.95], ...
    'LineWidth', 1.6, 'Tag', 'NavigationFootprint');
ov.head  = plot3(ax, nan, nan, nan, '-', 'Color', [0.1 0.4 0.95], ...
    'LineWidth', 2, 'Tag', 'NavigationHeading');
legend(ax, [ov.plan ov.trace ov.foot ov.start ov.goal], ...
    {'commanded path', 'executed motion', 'robot footprint', 'start', 'requested goal'}, ...
    'Location', 'northeast', 'TextColor', 'k', 'Color', 'w', 'AutoUpdate', 'off');
updateNavOverlay3D(ov, x, y, th, k, opts);
end

function updateNavOverlay3D(ov, x, y, th, k, opts)
z = 0.03;
R  = [cos(th(k)) -sin(th(k)); sin(th(k)) cos(th(k))];
fp = (R * opts.footprint.').' + [x(k) y(k)];
fp = [fp; fp(1,:)];
set(ov.trace, 'XData', x(1:k), 'YData', y(1:k), 'ZData', z*ones(k,1));
set(ov.foot, 'XData', fp(:,1), 'YData', fp(:,2), 'ZData', z*ones(size(fp,1),1));
set(ov.head, 'XData', [x(k), x(k) + cos(th(k))*opts.footprintRadius], ...
             'YData', [y(k), y(k) + sin(th(k))*opts.footprintRadius], 'ZData', [z z]);
end

function [vw, videoFile, profileUsed] = openLosslessWriter(outDir, baseName, opts, canVideo)
vw = []; videoFile = ''; profileUsed = '';
if ~canVideo, return; end
candidates = {opts.videoProfile, 'Uncompressed AVI', 'Archival'};
candidates = candidates(~cellfun('isempty', candidates));
tried = {};
for i = 1:numel(candidates)
    prof = candidates{i};
    if any(strcmp(prof, tried)), continue; end
    tried{end+1} = prof;
    if any(strcmp(prof, {'Archival', 'Motion JPEG 2000'}))
        videoFile = fullfile(outDir, [baseName '.mj2']);
    else
        videoFile = fullfile(outDir, [baseName '.avi']);
    end
    try
        vw = VideoWriter(videoFile, prof);
        if strcmp(prof, 'Motion JPEG 2000'), vw.LosslessCompression = true; end
        vw.FrameRate = opts.fps;
        open(vw);
        profileUsed = prof;
        return;
    catch ME
        warning('make_demo:videoProfile', ...
            'VideoWriter profile ''%s'' unavailable (%s); trying next.', ...
            prof, ME.message);
        vw = []; videoFile = ''; profileUsed = '';
    end
end
warning('make_demo:noVideo', ...
    'No lossless VideoWriter profile available. PNG keyframes only.');
end

function robotPngs = renderRobotViews(robot, outDir, figVis)
robotPngs = {};
if isempty(ver('robotics')) || ~isa(robot, 'rigidBodyTree')
    return;
end
try
    f = figure('Visible', figVis, 'Color', 'w', 'Position', [60 60 900 700]);
    views = {[-35 20], [90 0], [0 90]};
    names = {'robot_iso.png', 'robot_side.png', 'robot_top.png'};
    for i = 1:3
        clf(f);
        show(robot, 'Visuals', 'on', 'Collisions', 'off', 'Frames', 'off');
        axis equal; grid on; view(views{i});
        title('MiR100 rigidBodyTree imported by load\_mir100 (actual meshes)');
        print(f, fullfile(outDir, names{i}), '-dpng', '-r150');
    end
    robotPngs = names;
    close(f);
catch ME
    warning('make_demo:robotView', ...
        'Robot 3D still views failed (mesh resolution?): %s', ME.message);
end
end

function s = charOrNone(s)
if isempty(s), s = '(none)'; end
end
