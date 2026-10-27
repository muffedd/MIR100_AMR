function results = run_validation(repoRoot, opts)
%RUN_VALIDATION Validation suite for the MiR100 AMR MATLAB project.
%   results = RUN_VALIDATION() locates the repository root, runs tests
%   T1..T6 predeclared in matlab/validation/TESTS.txt, returns a struct
%   array with one entry per test, and writes
%   matlab/validation/validation_results.txt and validation_results.mat.
%
%   results = RUN_VALIDATION(repoRoot, opts) overrides the repo root and
%   options. opts fields (all optional):
%     csvPath   - world box CSV consumed by read_world_boxes/build_world
%     urdfPath  - MiR100 URDF consumed by load_mir100
%     res       - occupancy map resolution (m/cell), default 0.05
%     (map origin is NOT an option: build_map auto-derives it from the
%     wall extents when passed [], matching run_all.m)
%     inflateM  - planner inflation radius, default 0.6052 m (polygon v2)
%     outDir    - where result files are written (default: this folder)
%
%   Per-test status: PASS | FAIL | SKIP | NOTRUN | ERROR.
%   No test reports PASS unless every measured value came from the live
%   run in this invocation. Missing MATLAB, toolboxes, contract functions,
%   or data files yield NOTRUN; thrown errors yield ERROR; unmet
%   thresholds on real measurements yield FAIL. Nothing is fabricated.
%
%   Validation module, matlab/validation. Contracts come from the project:
%     boxes    = read_world_boxes(csv)          Nx7 [xc,yc,zc,sx,sy,sz,yaw] m/rad
%     scen     = build_world(csv)
%     robot    = load_mir100(urdfPath)
%     plat     = spawn_mir100(scen,robot)
%     map      = build_map(boxes,res,[])   % boxes Nx7 or CSV path; [] = auto origin
%     ok       = check_map(map,boxes,res,[])
%     report   = verify_robot(robot)
%     trajLog  = follow_route(plat,wpsWorld,opts)   fields: t,x,y,yaw,wheelAngles
%     clearLog = check_run_clearance(trajLog,boxes)
%     path     = plan_path(map,startXY,goalXY,inflateM)
%     reactOut = lidar_react(plat,scen,path,opts)
%   LOCKED project schemas (supersede all earlier aliases):
%     report   struct: linkCount, jointCount, massTotal, wheelRadius,
%              wheelSeparation, footprintRadius, footprintPolygon (Nx2,
%              base frame, m), checks (struct array: name, value, expected,
%              tol, pass), pass
%     trajLog  struct: t, x, y, yaw, wheelAngles (Nx2), waypoints,
%              reachedIdx, diagnosticOnly, plannerStatus
%     clearLog struct: t, minDist (Nx1), minOverall, minAt (x,y,t,box),
%              collision
%   opts.maxSimTime is honored by follow_route (default 600 s).

    results = struct('id', {}, 'name', {}, 'status', {}, 'measured', {}, ...
                     'thresholds', {}, 'details', {});

    % ---------------- environment + inputs ----------------
    meta = struct();
    meta.matlabVersion = version;
    v = ver;
    meta.installedProducts = {v.Name};
    meta.hasRST = exist('differentialDriveKinematics', 'class') == 8 && ...
                  exist('robotScenario', 'class') == 8;
    meta.hasNavToolbox = exist('binaryOccupancyMap', 'class') == 8 && ...
                         exist('plannerRRTStar', 'class') == 8;
    meta.timestamp = datestr(now, 'yyyy-mm-dd HH:MM:SS');

    thisDir = fileparts(mfilename('fullpath'));
    if nargin < 1 || isempty(repoRoot)
        repoRoot = fileparts(fileparts(thisDir)); % matlab/validation -> repo root
    end
    if nargin < 2, opts = struct(); end
    if ~isfield(opts, 'res'),      opts.res = 0.05; end
    if ~isfield(opts, 'inflateM'), opts.inflateM = 0.6052; end  % polygon v2
    if ~isfield(opts, 'maxSimTime'), opts.maxSimTime = 600; end
    if ~isfield(opts, 'outDir'),   opts.outDir = thisDir; end
    matlabDir = fullfile(repoRoot, 'matlab');
    if exist(matlabDir, 'dir') == 7
        addpath(genpath(matlabDir));
    end

    % Contract functions this suite consumes.
    contractFns = {'read_world_boxes', 'build_world', 'load_mir100', ...
                   'spawn_mir100', 'build_map', 'check_map', 'verify_robot', ...
                   'follow_route', 'check_run_clearance', 'plan_path', ...
                   'lidar_react'};
    have = struct();
    for i = 1:numel(contractFns)
        have.(contractFns{i}) = exist(contractFns{i}, 'file') == 2;
    end
    meta.contractFnsPresent = have;

    % Data files.
    if ~isfield(opts, 'csvPath') || isempty(opts.csvPath)
        opts.csvPath = find_first({fullfile(repoRoot, 'matlab', 'assets', 'hard_world_boxes.csv'), ...
                                   fullfile(repoRoot, 'matlab', 'world', '*.csv'), ...
                                   fullfile(repoRoot, 'matlab', '**', 'world*.csv'), ...
                                   fullfile(repoRoot, 'matlab', '**', '*boxes*.csv')});
    end
    if ~isfield(opts, 'urdfPath') || isempty(opts.urdfPath)
        opts.urdfPath = find_first({fullfile(repoRoot, 'matlab', 'assets', ...
                                        'mir_description', 'urdf', 'mir100_matlab.urdf'), ...
                                    fullfile(repoRoot, '**', 'mir100_matlab.urdf')});
        % Never auto-glob any *.urdf: the tree also carries mir100_flat.urdf
        % (Gazebo XML) and legacy xacros - only the canonical flat asset is
        % the import target.
    end
    meta.csvPath = opts.csvPath;
    meta.urdfPath = opts.urdfPath;

    % Declared model constants (from MiR100 model leads; verified by T4).
    C = struct();
    C.wheelRadius     = 0.0625;   % m
    C.wheelSeparation = 0.4452;   % m (track width)
    C.footprintRadius = 0.6052;   % m (6 mm edge-expanded hull, covers swivel casters)
    C.footprint = [-0.474026970 -0.236085370; -0.439392950 -0.319792950; ...
                   -0.355791680 -0.354382980;  0.493049750 -0.295637190; ...
                    0.548024070  0.236692380;  0.513392950  0.320392950; ...
                    0.429789310  0.354983960; -0.417838050  0.295633920]; % base frame, m
    C.frameOffset = [-4.31, -2.81]; % world = map + offset
    C.nWallBoxes  = 65;             % declared wall boxes, ground excluded

    % Route v2, map frame (16 waypoints).
    wpsMap = [0 0; 0 1.12; 0 2.81; 0.56 3.09; 1.12 3.37; 1.12 4.5; ...
              1.12 5.62; 2.25 5.62; 3.37 5.62; 4.5 5.62; 5.25 5.25; ...
              5.25 4.12; 5.25 3.37; 6.75 3.37; 7.87 3.37; 8.62 3.37];
    wpsWorld = [wpsMap(:,1) + C.frameOffset(1), wpsMap(:,2) + C.frameOffset(2)];
    spawnWorld = wpsWorld(1, :);          % [-4.31, -2.81]
    goalWorld  = [4.31, 0.56];            % == wpsWorld(end,:)

    % Shared run state so T1/T2/T3/T6 consume the SAME trajectory.
    S = struct('boxes', [], 'scen', [], 'robot', [], 'plat', [], ...
               'trajLog', [], 'wallTime', NaN, 'map', [], 'path', [], ...
               'importInfo', []);
    if ~isempty(opts.csvPath) && have.read_world_boxes
        try
            S.boxes = read_world_boxes(opts.csvPath);
        catch ME
            results(end+1) = mk('T0', 'load world boxes', 'ERROR', ...
                struct(), struct(), ['read_world_boxes threw: ' ME.message]);
        end
    end

    % ---------------- T4: unit / frame / wheel checks ----------------
    results(end+1) = test_t4(C, have, opts, spawnWorld, goalWorld, wpsWorld, S.boxes);

    % ---------------- T5: open-loop kinematic accuracy ----------------
    results(end+1) = test_t5(C, meta);

    % ---------------- build scenario + run route (feeds T1, T2, T3, T6) --
    needRun = {'build_world', 'load_mir100', 'spawn_mir100', 'follow_route'};
    canRun = meta.hasRST && ~isempty(opts.csvPath) && ~isempty(opts.urdfPath) ...
             && all(cellfun(@(f) have.(f), needRun));
    runNote = '';
    if ~canRun
        miss = needRun(~cellfun(@(f) have.(f), needRun));
        runNote = sprintf(['route run not possible (RST=%d, csv=%d, urdf=%d, ', ...
            'missing fns: %s)'], meta.hasRST, ~isempty(opts.csvPath), ...
            ~isempty(opts.urdfPath), strjoin(miss, ', '));
    else
        try
            S.scen  = build_world(opts.csvPath);
            [S.robot, S.importInfo] = load_mir100(opts.urdfPath);
            S.plat  = spawn_mir100(S.scen, S.robot);
            setup(S.scen);   % documented lifecycle: setup BEFORE advance/read
            % Global planner FIRST (audit fix): follow the planned path
            % when one exists; the predeclared route is a labeled
            % diagnostic only.
            routeWps = wpsWorld; diagFlag = true;
            if all(cellfun(@(f) have.(f), {'build_map', 'plan_path'}))
                mapSrc = S.boxes;            % build_map takes
                if isempty(mapSrc), mapSrc = opts.csvPath; end
                                             % Nx7 boxes or a CSV path,
                                             % NOT the scenario handle;
                                             % [] = auto origin (the
                                             % opts.originXY field never
                                             % existed)
                S.map  = build_map(mapSrc, opts.res, []);
                S.path = plan_path(S.map, spawnWorld, goalWorld, opts.inflateM);
                if ~isempty(S.path)
                    routeWps = S.path; diagFlag = false;
                end
            end
            tStart = tic;
            fopts = opts;
            fopts.scenario = S.scen;              % live scenario handle:
                                                  % advance() drives the clock
            fopts.diagnosticOnly = diagFlag;      % ACTUAL status
            if ~isfield(fopts, 'maxSimTime'), fopts.maxSimTime = 600; end
            S.trajLog = follow_route(S.plat, routeWps, fopts);
            S.wallTime = toc(tStart);
        catch ME
            canRun = false;
            runNote = ['route run threw: ' ME.message];
        end
    end

    % ---------------- T1: waypoint capture + footprint collision -------
    results(end+1) = test_t1(C, wpsWorld, S, canRun, runNote);

    % ---------------- T2: min clearance > 0 ----------------------------
    results(end+1) = test_t2(C, have, S, canRun, runNote);

    % ---------------- T3: tracking error vs planned path ---------------
    [rT3, S] = test_t3(have, opts, S, canRun, runNote, spawnWorld, goalWorld);
    results(end+1) = rT3;

    % ---------------- T6: sim/wall ratio report ------------------------
    results(end+1) = test_t6(S, canRun, runNote);

    % ---------------- aux (non-gating): lidar_react smoke --------------
    results(end+1) = aux_lidar(have, opts, S, canRun);

    % ---------------- write outputs ------------------------------------
    meta.note = ['Core kinematic demo evidence only: NOT physical validation ', ...
                 'and NOT a realtime guarantee.'];
    out = struct('meta', meta, 'constants', C, 'results', results);
    try
        write_report(fullfile(opts.outDir, 'validation_results.txt'), out);
        save(fullfile(opts.outDir, 'validation_results.mat'), 'out');
    catch ME
        warning('run_validation:io', 'could not write result files: %s', ME.message);
    end
end

% ======================================================================
function r = test_t4(C, have, opts, spawnWorld, goalWorld, wpsWorld, boxes)
    r = mk('T4', 'unit/frame/wheel checks', 'NOTRUN', struct(), ...
           struct('wheelRadiusTol', 1e-3, 'separationTol', 1e-3, ...
                  'massTol', 1e-2, 'footprintRadiusTol', 5e-3), '');
    if ~meta_ok(have, {'load_mir100', 'verify_robot'}) || isempty(opts.urdfPath)
        r.details = 'load_mir100/verify_robot or URDF missing';
        return;
    end
    try
        [robot, importInfo] = load_mir100(opts.urdfPath);
        report = verify_robot(robot, struct('UrdfPath', opts.urdfPath, ...
            'ImportInfo', importInfo, 'ThrowOnFail', false));
    catch ME
        r.status = 'ERROR';
        r.details = ME.message;
        return;
    end
    checks = {};
    % -- wheel radius
    val = first_field(report, {'wheelRadius'});
    checks{end+1} = subcheck('wheelRadius', val, C.wheelRadius, 1e-3);
    % -- wheel separation
    val = first_field(report, {'wheelSeparation'});
    checks{end+1} = subcheck('wheelSeparation', val, C.wheelSeparation, 1e-3);
    % -- mass: ACTUAL imported tree mass vs URDF-
    %    declared source mass, both derived at runtime - no hardcoded
    %    constant, no subtract scheme. massTotal recorded as evidence.
    val = first_field(report, {'massTotal'});
    valImp = first_field(report, {'importedMass'});
    valDecl = first_field(report, {'declaredSourceMass'});
    checks{end+1} = subcheck('importedMassVsDeclaredSource', valImp, valDecl, 1e-3);
    checks{end+1} = struct('name', 'massTotal_actualEvidence', 'measured', val, ...
        'expected', '== declaredSourceMass', 'tol', NaN, 'pass', true);
    % -- footprint radius
    val = first_field(report, {'footprintRadius'});
    checks{end+1} = subcheck('footprintRadius', val, C.footprintRadius, 5e-3);
    % -- footprint polygon vs locked geometry (vertex-wise)
    val = first_field(report, {'footprintPolygon'});
    if isnumeric(val) && isequal(size(val), size(C.footprint))
        polyErr = max(abs(val(:) - C.footprint(:)));
    else
        polyErr = NaN;
    end
    checks{end+1} = subcheck('footprintPolygonMaxVertexErr', polyErr, 0, 1e-6);
    % -- boxes: shape, units, count
    if ~isempty(boxes)
        okShape = size(boxes, 2) == 7 && all(isfinite(boxes(:)));
        checks{end+1} = subcheck('boxesNx7finite', double(okShape), 1, 0);
        okSizes = all(boxes(:, 4) > 0) && all(boxes(:, 5) > 0) && all(boxes(:, 6) > 0);
        checks{end+1} = subcheck('boxSizesPositive_m', double(okSizes), 1, 0);
        okYaw = all(abs(boxes(:, 7)) <= pi + 1e-9);
        checks{end+1} = subcheck('boxYawRange_rad', double(okYaw), 1, 0);
        checks{end+1} = subcheck('wallBoxCount', size(boxes, 1), C.nWallBoxes, 0);
    else
        checks{end+1} = subcheck('boxesLoaded', NaN, 1, 0);
    end
    % -- frame check: world = map + offset
    okFrame = norm(wpsWorld(1,:) - [-4.31 -2.81]) < 1e-9 && ...
              norm(wpsWorld(end,:) - goalWorld) < 1e-9 && ...
              norm(spawnWorld - [-4.31 -2.81]) < 1e-9;
    checks{end+1} = subcheck('mapToWorldFrameOffset', double(okFrame), 1, 0);

    r.measured = checks;
    % Robot module's own declared checks are GATING (project fix): if the
    % module's internal inventory/geometry checks fail, T4 cannot pass.
    selfPass = true;
    if isfield(report, 'pass')
        selfPass = logical(report.pass);
        r.measured{end+1} = subcheck('reportSelfDeclaredPass', double(report.pass), 1, 0);
    end
    bad = find(cellfun(@(c) ~c.pass, checks), 1);
    if any(cellfun(@(c) isnan(c.measured) || isempty(c.measured), checks))
        r.status = 'ERROR';
        r.details = 'one or more contract report fields missing; see measured[]';
    elseif isempty(bad) && selfPass
        r.status = 'PASS';
        r.details = sprintf('all %d subchecks within tolerance', numel(checks));
    elseif isempty(bad)
        r.status = 'FAIL';
        r.details = 'contract fields match but verify_robot report.pass is false (its internal checks failed)';
    else
        r.status = 'FAIL';
        r.details = sprintf('%d of %d subchecks failed', ...
            nnz(~cellfun(@(c) c.pass, checks)), numel(checks));
    end
end

% ======================================================================
function r = test_t5(C, meta)
    r = mk('T5', 'straight 2 m / turn pi accuracy', 'NOTRUN', struct(), ...
           struct('straightTolM', 0.02, 'turnTolRad', 0.02, ...
                  'wheelInputTolM', 0.02), '');
    if ~meta.hasRST
        r.details = 'Robotics System Toolbox not installed';
        return;
    end
    try
        kin = differentialDriveKinematics;
        kin.WheelRadius = C.wheelRadius;
        kin.TrackWidth  = C.wheelSeparation;
        dt = 0.05;

        kin.VehicleInputs = 'VehicleSpeedHeadingRate';
        % Straight: 0.25 m/s for 8 s = 2.0 m.
        pose = [0; 0; 0]; v = 0.25; w = 0;
        for k = 1:round(8.0/dt)
            pose = pose + derivative(kin, pose, [v; w]) * dt;
        end
        m.straightDistM = pose(1);
        straightErr = abs(pose(1) - 2.0);

        % Turn in place: w = 0.5 rad/s for pi/0.5 s = pi rad.
        pose = [0; 0; 0]; v = 0; w = 0.5;
        for k = 1:round((pi/w)/dt)
            pose = pose + derivative(kin, pose, [v; w]) * dt;
        end
        m.turnRad = wrap_pi(pose(3));
        turnErr = abs(wrap_pi(pose(3) - pi));

        % Wheel-speed input consistency: [v/r, v/r] must reproduce v.
        kin.VehicleInputs = 'WheelSpeeds';
        pose = [0; 0; 0]; ws = 0.25 / C.wheelRadius;
        for k = 1:round(8.0/dt)
            pose = pose + derivative(kin, pose, [ws; ws]) * dt;
        end
        m.wheelInputDistM = pose(1);
        wheelErr = abs(pose(1) - 2.0);

        r.measured = m;
        if straightErr <= 0.02 && turnErr <= 0.02 && wheelErr <= 0.02
            r.status = 'PASS';
            r.details = sprintf('straight err %.4f m, turn err %.4f rad, wheel-input err %.4f m', ...
                                straightErr, turnErr, wheelErr);
        else
            r.status = 'FAIL';
            r.details = sprintf('straight err %.4f m (tol 0.02), turn err %.4f rad (tol 0.02), wheel-input err %.4f m (tol 0.02)', ...
                                straightErr, turnErr, wheelErr);
        end
    catch ME
        r.status = 'ERROR';
        r.details = ME.message;
    end
end

% ======================================================================
function r = test_t1(C, wpsWorld, S, canRun, runNote)
    r = mk('T1', 'commanded-waypoint capture + final goal; zero footprint collision', ...
           'NOTRUN', struct(), struct('waypointTolM', 0.15, 'maxCollisions', 0), '');
    if ~canRun
        r.details = runNote;
        return;
    end
    [xy, yaw, okFields] = traj_fields(S.trajLog);
    if ~okFields
        r.status = 'ERROR';
        r.details = 'trajLog missing required fields (t,x,y,yaw)';
        return;
    end
    % Commanded route = the waypoints actually handed to follow_route:
    % the global plan when the planner found one, else the predeclared
    % v2 route in a labeled diagnostic run. SPEC CHANGE (recorded in
    % TESTS.txt): grading a planned-path run
    % against the predeclared v2 route false-fails a valid plan - 9 of
    % 16 v2 waypoints lie more than 0.15 m off the 0.6052-inflation
    % plan (worst 0.658 m). The v2 distance is diagnostic evidence only.
    commanded = wpsWorld;
    if isfield(S.trajLog, 'waypoints') && ~isempty(S.trajLog.waypoints)
        commanded = S.trajLog.waypoints;
    end
    nCmd = size(commanded, 1);
    % Capture evidence: the follower contract's reachedIdx bookkeeping,
    % plus an independent min-distance from the logged trajectory.
    if isfield(S.trajLog, 'reachedIdx') && ~isempty(S.trajLog.reachedIdx)
        reachedCount = numel(S.trajLog.reachedIdx);
    else
        reachedCount = NaN;
    end
    dmin = zeros(nCmd, 1);
    for i = 1:nCmd
        dmin(i) = min(sqrt((xy(:,1) - commanded(i,1)).^2 + (xy(:,2) - commanded(i,2)).^2));
    end
    goalDist = min(sqrt((xy(:,1) - commanded(end,1)).^2 + (xy(:,2) - commanded(end,2)).^2));
    % Diagnostic-only evidence: predeclared v2 route distance (NOT gated).
    dminV2 = zeros(size(wpsWorld, 1), 1);
    for i = 1:size(wpsWorld, 1)
        dminV2(i) = min(sqrt((xy(:,1) - wpsWorld(i,1)).^2 + (xy(:,2) - wpsWorld(i,2)).^2));
    end
    % Independent footprint-vs-boxes SAT overlap check (does not trust
    % the module under test; consumes trajLog + boxes directly).
    nColl = 0;
    worst = struct('t', NaN, 'box', NaN);
    for k = 1:size(xy, 1)
        fp = transform_poly(C.footprint, xy(k,1), xy(k,2), yaw(k));
        for b = 1:size(S.boxes, 1)
            box = box_poly(S.boxes(b,:));
            if polys_overlap(fp, box)
                nColl = nColl + 1;
                worst.t = S.trajLog.t(k); worst.box = b;
            end
        end
    end
    if isnan(reachedCount)   % no contract bookkeeping: min-distance fallback
        captureOk = all(dmin <= 0.15);
        captureBasis = 'min-distance fallback (reachedIdx absent)';
    else
        captureOk = reachedCount == nCmd;
        captureBasis = sprintf('reachedIdx %d/%d commanded', reachedCount, nCmd);
    end
    goalOk = goalDist <= 0.15;
    r.measured = struct('commandedCount', nCmd, 'reachedCount', reachedCount, ...
                        'commandedMinDistM_max', max(dmin), ...
                        'goalDistM', goalDist, ...
                        'v2RouteMinDistM_max_DIAGNOSTIC', max(dminV2), ...
                        'collisionCount', nColl, 'lastCollision', worst);
    if isfield(S.trajLog, 'plannerStatus')
        r.measured.plannerStatus = S.trajLog.plannerStatus;
    end
    diagNote = '';
    isDiag = isfield(S.trajLog, 'diagnosticOnly') && S.trajLog.diagnosticOnly;
    if isDiag
        diagNote = ' WARNING: trajLog.diagnosticOnly is true (diagnostic trajectory).';
    end
    if captureOk && goalOk && nColl == 0 && isDiag
        r.status = 'DIAGNOSTIC-PASS';
        r.details = sprintf(['ROUTE-FOLLOW DIAGNOSTIC ONLY (NOT a global-navigation pass): ' ...
            '%s, goal %.3f m, 0 collisions over %d poses.%s'], ...
            captureBasis, goalDist, size(xy,1), diagNote);
    elseif captureOk && goalOk && nColl == 0
        r.status = 'PASS';
        if isnan(reachedCount)   % fallback basis: no count to display
            r.details = sprintf('commanded waypoints captured (%s), goal %.3f m, 0 collisions over %d poses.%s', ...
                                captureBasis, goalDist, size(xy,1), diagNote);
        else
            r.details = sprintf('%d/%d commanded waypoints captured (%s), goal %.3f m, 0 collisions over %d poses.%s', ...
                                reachedCount, nCmd, captureBasis, goalDist, size(xy,1), diagNote);
        end
    else
        r.status = 'FAIL';
        r.details = sprintf('capture=%d (%s), goal %.3f m (tol 0.15), collisions %d.%s', ...
                            captureOk, captureBasis, goalDist, nColl, diagNote);
    end
end

% ======================================================================
function r = test_t2(C, have, S, canRun, runNote)
    r = mk('T2', 'min clearance > 0', 'NOTRUN', struct(), ...
           struct('minClearanceM', 0), '');
    if ~canRun
        r.details = runNote;
        return;
    end
    [xy, yaw, okFields] = traj_fields(S.trajLog);
    if ~okFields
        r.status = 'ERROR';
        r.details = 'trajLog missing required fields (t,x,y,yaw)';
        return;
    end
    % Primary: independent polygon-to-box distance (0 on overlap).
    dmin = inf;
    for k = 1:size(xy, 1)
        fp = transform_poly(C.footprint, xy(k,1), xy(k,2), yaw(k));
        for b = 1:size(S.boxes, 1)
            dmin = min(dmin, poly_dist(fp, box_poly(S.boxes(b,:))));
        end
    end
    % Cross-check against contract check_run_clearance when available.
    cross = NaN;
    if have.check_run_clearance
        try
            clearLog = check_run_clearance(S.trajLog, S.boxes);
            cross = min_clearance_from(clearLog);
        catch ME
            r.details = ['check_run_clearance threw: ' ME.message];
        end
    end
    r.measured = struct('minClearanceM_independent', dmin, ...
                        'minOverall_contract', cross);
    if dmin > 0
        r.status = 'PASS';
        r.details = sprintf('min clearance %.4f m (independent); contract fn: %s', ...
            dmin, num2str_or_na(cross));
    else
        r.status = 'FAIL';
        r.details = sprintf('min clearance %.4f m <= 0 (footprint touches/penetrates)', dmin);
    end
end

% ======================================================================
function [r, S] = test_t3(have, opts, S, canRun, runNote, startXY, goalXY)
    r = mk('T3', 'max/RMS tracking error <= 0.20 m', 'NOTRUN', struct(), ...
           struct('maxErrTolM', 0.20, 'rmsErrTolM', 0.20, 'inflateM', opts.inflateM), '');
    if ~canRun
        r.details = runNote;
        return;
    end
    if ~all(cellfun(@(f) have.(f), {'build_map', 'plan_path'}))
        r.details = 'build_map/plan_path missing';
        return;
    end
    [xy, ~, okFields] = traj_fields(S.trajLog);
    if ~okFields
        r.status = 'ERROR';
        r.details = 'trajLog missing required fields';
        return;
    end
    try
        % Reuse the shared run's map/path when already computed with the
        % same params. build_map takes Nx7 boxes or a CSV path, NOT a
        % robotScenario handle; [] selects the auto origin.
        if isempty(S.map)
            mapSrc = S.boxes;
            if isempty(mapSrc), mapSrc = opts.csvPath; end
            S.map = build_map(mapSrc, opts.res, []);
        end
        if have.check_map && ~isempty(S.boxes)
            r.measured.checkMapOk = check_map(S.map, S.boxes, opts.res, []);
        end
        if isempty(S.path)
            S.path = plan_path(S.map, startXY, goalXY, opts.inflateM);
        end
    catch ME
        r.status = 'ERROR';
        r.details = ME.message;
        return;
    end
    if isempty(S.path) || size(S.path, 1) < 2
        r.status = 'FAIL';
        r.measured.maxErrM = NaN; r.measured.rmsErrM = NaN;
        r.details = 'plan_path returned an empty path; tracking error undefined, criterion unmet';
        return;
    end
    d = point_polyline_dist(xy, S.path);
    r.measured.maxErrM = max(d);
    r.measured.rmsErrM = sqrt(mean(d.^2));
    if r.measured.maxErrM <= 0.20 && r.measured.rmsErrM <= 0.20
        r.status = 'PASS';
        r.details = sprintf('max %.3f m, RMS %.3f m vs planned path (%d poses)', ...
                            r.measured.maxErrM, r.measured.rmsErrM, numel(d));
    else
        r.status = 'FAIL';
        r.details = sprintf('max %.3f m, RMS %.3f m (tol 0.20)', ...
                            r.measured.maxErrM, r.measured.rmsErrM);
    end
end

% ======================================================================
function r = test_t6(S, canRun, runNote)
    r = mk('T6', 'sim/wall time ratio (report only, NOT a realtime guarantee)', ...
           'NOTRUN', struct(), struct(), '');
    if ~canRun
        r.details = runNote;
        return;
    end
    [~, ~, okFields] = traj_fields(S.trajLog);
    if ~okFields
        r.status = 'ERROR';
        r.details = 'trajLog missing field t';
        return;
    end
    simTime = S.trajLog.t(end) - S.trajLog.t(1);
    ratio = simTime / S.wallTime;
    r.measured = struct('simTimeS', simTime, 'wallTimeS', S.wallTime, ...
                        'simOverWall', ratio);
    r.status = 'PASS'; % PASS = measured and reported; no threshold by design
    r.details = sprintf(['sim %.2f s in %.2f s wall (ratio %.3f). ', ...
        'Single-machine measurement; NOT a realtime guarantee.'], ...
        simTime, S.wallTime, ratio);
end

% ======================================================================
function r = aux_lidar(have, opts, S, canRun)
    r = mk('AUX-LIDAR', 'lidar_react smoke (non-gating)', 'SKIP', struct(), struct(), '');
    if ~have.lidar_react
        r.details = 'lidar_react not present; not required by T1..T6';
        return;
    end
    if ~canRun || isempty(S.path)
        r.details = 'no run/path available to feed lidar_react';
        return;
    end
    try
        reactOpts = opts;
        reactOpts.boxes = S.boxes;
        reactOut = lidar_react(S.plat, S.scen, S.path, reactOpts);
        r.status = 'PASS';
        r.measured = struct('outputClass', class(reactOut), ...
            'success',reactOut.success,'stopReason',reactOut.stopReason, ...
            'lidarSource',reactOut.lidarSource);
        r.details = 'lidar_react executed without error (smoke only, no criteria)';
    catch ME
        r.status = 'ERROR';
        r.details = ['lidar_react threw: ' ME.message];
    end
end

% ======================== helpers =====================================
function r = mk(id, name, status, measured, thresholds, details)
    r = struct('id', id, 'name', name, 'status', status, ...
               'measured', measured, 'thresholds', thresholds, 'details', details);
end

function tf = meta_ok(have, fns)
    tf = all(cellfun(@(f) have.(f), fns));
end

function p = find_first(patterns)
    p = '';
    for i = 1:numel(patterns)
        d = dir(patterns{i});
        if ~isempty(d)
            p = fullfile(d(1).folder, d(1).name);
            return;
        end
    end
end

function val = first_field(s, names)
    val = NaN;
    for i = 1:numel(names)
        if isstruct(s) && isfield(s, names{i})
            val = s.(names{i});
            return;
        end
    end
end

function c = subcheck(name, measured, expected, tol)
    c = struct('name', name, 'measured', measured, 'expected', expected, 'tol', tol);
    if isempty(measured) || (isscalar(measured) && isnan(measured))
        c.pass = false; % missing contract field => not a pass
    else
        c.pass = abs(measured - expected) <= tol;
    end
end

function [xy, yaw, ok] = traj_fields(trajLog)
    xy = []; yaw = []; ok = false;
    if ~isstruct(trajLog), return; end
    if ~all(isfield(trajLog, {'t', 'x', 'y', 'yaw'})), return; end
    xy = [trajLog.x(:), trajLog.y(:)];
    yaw = trajLog.yaw(:);
    ok = true;
end

function p = transform_poly(poly, x, y, yaw)
    R = [cos(yaw) -sin(yaw); sin(yaw) cos(yaw)];
    p = (R * poly.').' + [x y];
end

function p = box_poly(row)
    % row = [xc,yc,zc,sx,sy,sz,yaw] -> 4x2 world-frame corners (2-D).
    hx = row(4)/2; hy = row(5)/2;
    local = [ hx  hy;  hx -hy; -hx -hy; -hx  hy];
    p = transform_poly(local, row(1), row(2), row(7));
end

function tf = polys_overlap(a, b)
    % 2-D SAT for convex polygons (OBB vs footprint hull).
    tf = true;
    for poly = {a, b}
        q = poly{1};
        for i = 1:size(q,1)
            j = mod(i, size(q,1)) + 1;
            n = [-(q(j,2)-q(i,2)), q(j,1)-q(i,1)];
            pa = a * n.'; pb = b * n.';
            if max(pa) < min(pb) || max(pb) < min(pa)
                tf = false;
                return;
            end
        end
    end
end

function d = poly_dist(a, b)
    if polys_overlap(a, b)
        d = 0;
        return;
    end
    d = inf;
    for poly = {a, b}
        q = poly{1}; other = b; if isequal(q, b), other = a; end
        for i = 1:size(q,1)
            j = mod(i, size(q,1)) + 1;
            d = min(d, point_seg_dist(other, q(i,:), q(j,:)));
        end
    end
end

function dmin = point_seg_dist(pts, a, b)
    ab = b - a;
    L2 = sum(ab.^2);
    if L2 == 0
        dmin = min(sqrt(sum((pts - a).^2, 2)));
        return;
    end
    t = ((pts(:,1)-a(1))*ab(1) + (pts(:,2)-a(2))*ab(2)) / L2;
    t = max(0, min(1, t));
    proj = a + t .* ab;
    dmin = min(sqrt(sum((pts - proj).^2, 2)));
end

function d = point_polyline_dist(pts, path)
    d = inf(size(pts,1), 1);
    for i = 1:size(path,1)-1
        a = path(i,1:2); b = path(i+1,1:2);
        ab = b - a; L2 = sum(ab.^2);
        if L2 == 0, continue; end
        t = ((pts(:,1)-a(1))*ab(1) + (pts(:,2)-a(2))*ab(2)) / L2;
        t = max(0, min(1, t));
        proj = a + t .* ab;
        d = min(d, sqrt(sum((pts - proj).^2, 2)));
    end
end

function m = min_clearance_from(clearLog)
    if isnumeric(clearLog)
        m = min(clearLog(:));
        return;
    end
    if isstruct(clearLog)
        if isfield(clearLog, 'minOverall') % locked canonical field
            m = clearLog.minOverall;
            return;
        end
        if isfield(clearLog, 'minDist') % locked per-step vector; derive
            m = min(clearLog.minDist(:));
            return;
        end
    end
    m = NaN;
end

function s = num2str_or_na(x)
    if isnan(x), s = 'n/a'; else, s = sprintf('%.4f m', x); end
end

function a = wrap_pi(a)
    a = mod(a + pi, 2*pi) - pi;
end

function write_report(fname, out)
    fid = fopen(fname, 'w');
    if fid < 0, return; end
    c = onCleanup(@() fclose(fid));
    fprintf(fid, 'run_validation report - %s\n', out.meta.timestamp);
    fprintf(fid, 'MATLAB %s | RST present: %d | Navigation Toolbox present: %d\n', ...
        out.meta.matlabVersion, out.meta.hasRST, out.meta.hasNavToolbox);
    fprintf(fid, 'csv: %s\nurdf: %s\n\n', out.meta.csvPath, out.meta.urdfPath);
    for i = 1:numel(out.results)
        r = out.results(i);
        fprintf(fid, '[%-6s] %-9s %s\n', r.status, r.id, r.name);
        if ~isempty(r.details)
            fprintf(fid, '          %s\n', r.details);
        end
    end
    fprintf(fid, '\n%s\n', out.meta.note);
end
