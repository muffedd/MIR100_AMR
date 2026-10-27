function results = run_all(rootDir, opts)
% RUN_ALL Root entry point for the MiR100 MATLAB AMR simulation.
%   results = RUN_ALL(rootDir) runs the full core pipeline:
%   world -> robot -> map -> planning -> control -> collision -> demo,
%   then the independent validation battery (validation/run_validation.m,
%   tests T1-T6 in validation/TESTS.txt), which re-executes the stages
%   with its own checks (deliberate independence, not reuse of this
%   function's variables).
%
%   run_all([x y]) selects a goal in world metres and computes a fresh path.
%   In desktop MATLAB, run_all opens a map and lets you click a goal;
%   batch uses [4.31 0.56].
%   run_all(rootDir,opts): opts.goal, showAnimation, renderDemo.
%
%   Executed on MATLAB R2025b. See DEBUG_REPORT.md in the project root
%   for actual logs, measured results and remaining limitations.
%   In interactive MATLAB, make_demo opens and plays the executed route
%   in 3D automatically. Batch runs render without opening a window.
%
%   Requires on path: world/, robot/, map/, planning/, control/,
%   collision/, validation/, demo/ (added below).

if nargin < 2 || isempty(opts), opts=struct(); end
if nargin>=1 && isnumeric(rootDir) && ~isempty(rootDir)
    opts.goal=rootDir;rootDir=[];
elseif nargin>=1 && isstruct(rootDir)
    opts=rootDir;rootDir=[];
end
if nargin < 1 || isempty(rootDir), rootDir = fileparts(mfilename('fullpath')); end
validateattributes(opts,{'struct'},{'scalar'});
if ~isfield(opts,'showAnimation'),opts.showAnimation=~batchStartupOptionUsed;end
if ~isfield(opts,'renderDemo'),opts.renderDemo=true;end
goalNeedsGui = ~isfield(opts,'goal') || isempty(opts.goal);
if goalNeedsGui
    opts.goal=[4.31 .56];
end
validateattributes(opts.goal,{'numeric'},{'real','finite','vector','numel',2});
opts.goal=double(opts.goal(:)');
for d = {'world','robot','map','planning','control','collision','validation','demo'}
    addpath(fullfile(rootDir, d{1}));
end

csvPath  = fullfile(rootDir, 'assets', 'hard_world_boxes.csv');
urdfPath = fullfile(rootDir, 'assets', 'mir_description', 'urdf', 'mir100_matlab.urdf');

% Hard facts (verified against the source model - assets/MODEL_VALIDATION.txt):
RES     = 0.05;              % occupancy map cell size, METRES PER CELL
SPAWN   = [-4.31, -2.81];    % world-frame spawn (route v2 start)
GOAL    = opts.goal;         % requested world-frame destination
if goalNeedsGui && ~batchStartupOptionUsed
    GOAL = select_goal_on_map(csvPath, SPAWN, GOAL);
    opts.goal = GOAL;
end
INFLATE = 0.6052;            % planner inflation = footprint polygon v2 radius
                             % (caster-sweep-corrected; do NOT shrink)
% Footprint polygon v2 (base frame), caster-sweep corrected, max radius
% 0.6052 m - see assets/mir100_footprint.csv and collision/README.md:
FOOT_V2 = [-0.474027 -0.236085; -0.439393 -0.319793; -0.355792 -0.354383; ...
            0.493050 -0.295637;  0.548024  0.236692;  0.513393  0.320393; ...
            0.429789  0.354984; -0.417838  0.295634];
% Independent output folder; no prior path or result is read by this run.
runsDir=fullfile(rootDir,'demo','runs');
if ~isfolder(runsDir),mkdir(runsDir);end
[~,runToken]=fileparts(tempname(runsDir));
outDir=fullfile(runsDir,['run_' char(datetime('now','Format','yyyyMMdd_HHmmss')) '_' runToken]);
mkdir(outDir);
fprintf('Fresh navigation run: goal=[%.3f %.3f] m\nOutputs: %s\n',GOAL,outDir);
fprintf('Known walls -> occupancy map -> 0.6052 m inflation -> A* -> pure pursuit -> differential-drive kinematics.\n');

step('read_world_boxes');  boxes   = read_world_boxes(csvPath);
step('build_world');       scen    = build_world(csvPath);
step('load_mir100');       [robot, importInfo] = load_mir100(urdfPath);
step('verify_robot');      rreport = verify_robot(robot, struct( ...
                           'UrdfPath', urdfPath, 'ImportInfo', importInfo));
                           disp(rreport);
step('spawn_mir100');      plat    = spawn_mir100(scen, robot);
setup(scen);   % documented lifecycle: setup BEFORE any advance/read
step('build_map');         map     = build_map(boxes, RES, []);  % [] = auto
                             % origin: map LOWER-LEFT from wall extents
                             % (NOT the robot spawn - walls exist left and
                             % below the spawn; the map must contain them)
step('check_map');         assert(check_map(map, boxes, RES, []), ...
                                   'map check failed - see map/check_map.m');
% Full-world bounds are covered by check_map's wall probes (every wall
% must be inside and occupied), not only start/goal.
step('plan_path');         [path, planInfo] = plan_path(map, SPAWN, GOAL, INFLATE);
% GLOBAL planner status is tracked separately from the follower's own
% plannerStatus field (which only reports follow success/timeout):
if isempty(path)
    % A requested goal must never be replaced by an old diagnostic route.
    plannerStatusGlobal = ['BLOCKED: ' planInfo.reason];
    warning('run_all:noPath','%s. No motion commanded.',plannerStatusGlobal);
    routeWps = zeros(0,2);
    diagnosticOnly = false;
else
    plannerStatusGlobal = 'PATH FOUND';
    routeWps = path;
    diagnosticOnly = false;
end
fprintf('Global planner: %s (diagnosticOnly=%d)\n', plannerStatusGlobal, diagnosticOnly);
if isempty(path)
    trajLog=struct('t',0,'x',SPAWN(1),'y',SPAWN(2),'yaw',0, ...
        'wheelAngles',[0 0],'waypoints',path,'reachedIdx',zeros(0,1), ...
        'status','blocked','plannerStatus','blocked','reason',planInfo.reason, ...
        'finalError',norm(GOAL-SPAWN),'wheelsCommanded',false);
else
    if size(routeWps,1)==1,routeWps=[SPAWN;GOAL];end
    step('follow_route');trajLog = follow_route(plat, routeWps, ...
        struct('scenario', scen, 'maxSimTime', 600));
end
trajLog.diagnosticOnly = diagnosticOnly;   % ACTUAL status, set by root
trajLog.plannerStatusGlobal = plannerStatusGlobal;  % kept separate from
                                           % follow-status plannerStatus
step('check_clearance');   clearLog = check_run_clearance(trajLog, boxes);
fprintf('Clearance: minOverall = %.4f m, collision = %d\n', ...
        clearLog.minOverall, clearLog.collision);
corePass = ~diagnosticOnly && strcmp(trajLog.status,'success') && clearLog.pass;
[metrics,algorithms]=navigation_diagnostics(trajLog,path,GOAL, ...
    clearLog,boxes,outDir,opts.showAnimation);
if corePass && opts.renderDemo
step('make_demo');         manifest = make_demo(csvPath, urdfPath, ...
                               outDir, ...
                               struct('footprint', FOOT_V2, ...
                                      'scenario', scen, ...
                                      'trajLog', trajLog, ...
                                      'wpsWorld', routeWps, ...
                                      'showAnimation',opts.showAnimation, ...
                                      'diagnosticOnly', diagnosticOnly));
elseif ~corePass
    manifest = struct('status','not recorded: navigation or clearance failed');
    warning('run_all:coreFailed','Core navigation failed; no demo recorded.');
else
    manifest=struct('status','renderDemo=false; simulation still executed');
end
step('reference_route_validation');
fprintf('Independent standard tests use goal [4.31 0.56], separate from your requested goal [%.3f %.3f].\n',GOAL);
rv = run_validation(fileparts(rootDir), ...
                               struct('csvPath', csvPath, ...
                                      'urdfPath', urdfPath, ...
                                      'res', RES, ...
                                      'inflateM', INFLATE, ...
                                      'outDir',outDir, ...
                                      'maxSimTime', 600));
% rv is a struct ARRAY (one entry per test) - wrap deliberately:
results = struct('tests', {rv}, ...
                 'plannerStatusGlobal', plannerStatusGlobal, ...
                 'diagnosticOnly', diagnosticOnly);
results.corePass = corePass;
results.pass = corePass && all(strcmp({rv(ismember({rv.id},{'T1','T2','T3','T4','T5','T6'})).status},'PASS'));
results.trajectory = trajLog;
results.clearance = clearLog;
results.demo = manifest;
results.goal=GOAL;results.start=SPAWN;results.path=path;
results.planInfo=planInfo;results.outputDir=outDir;
results.validationScope='T1-T6 independently validate the canonical reference goal [4.31 0.56].';
results.metrics=metrics;results.algorithms=algorithms;
save(fullfile(outDir,'navigation_results.mat'),'results');
fprintf('Requested goal: [%.3f %.3f], stop=%s, error=%.4f m\n',GOAL,trajLog.status,trajLog.finalError);
fprintf('%s\nGraphs and numeric logs: %s\n',results.validationScope,outDir);

fprintf('RUN_ALL complete. corePass=%d validationPass=%d (kinematic simulation only).\n',corePass,results.pass);
disp(struct2table(rv,'AsArray',true));
end

function step(name)
fprintf('\n===== %s =====\n', name);
end
