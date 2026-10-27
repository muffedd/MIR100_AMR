function reactOut = lidar_react(plat, scen, path, opts)
%LIDAR_REACT  Follow a world-frame path with simulated 2-D lidar, detect obstacles
%   that are NOT in the known map, check collision explicitly and replan.
%
%   reactOut = lidar_react(plat, scen, path, opts)
%
%   plat : robot platform from spawn_mir100 (only used when opts.movePlatform=true;
%          the core loop does not need it).  scen: robotScenario (only used for
%          optional advance/updateSensors; may be []).
%   path : Nx2 (or Nx3) WORLD-frame waypoints [x y (yaw)] in metres.
%          NOTE: plan_path contract does not state its frame; this function
%          assumes WORLD frame. Convert map->world (+[-4.31 -2.81]) before calling.
%   opts (struct, all optional except boxes):
%     boxes        Nx7 known boxes [xc yc zc sx sy sz yaw] (read_world_boxes)
%     unseenBoxes  Mx7 obstacles the planner does not know about (default [])
%     autoUnseen   true -> add one 0.5x0.5 unseen box on the path at ~40% length
%     startPose    [x y yaw] (default path(1,:), heading toward path(2,:))
%     footprint    Kx2 polygon in robot frame (default MiR100 conservative polygon)
%     res          grid cell size m (default 0.05);  originXY / sizeXY grid bounds
%                  (default: bounding box of boxes+path, 2 m margin)
%     (planner: orientation-aware SE2 footprint lattice on a 2*res grid, exact footprint
%      cell masks; no inflation radius is shrunk or tuned to force a pass)
%     lidarRange (8), nRays (360), fovDeg (360), noiseStd (0)
%     vmax (0.5), wmax (0.8), lookahead (0.6), goalTol (0.15), dt (0.1), maxTime (400)
%     checkAhead (1.5 m) , safetyMargin (0.10 m)
%     movePlatform false; useTEB false (optional, UNTESTED, gated)
%   Output reactOut fields: success, collided, goalReached, replanCount,
%     unseenDetected, log (t,x,y,yaw), pathFinal, minLidarRange, caps, notes.
%
%   MATLAB R2025b execution evidence: see DEBUG_REPORT.md. Kinematic only.

if nargin < 4 || isempty(opts), opts = struct(); end
o = defaults(opts, path);
notes = {};
caps  = react_capabilities();
path  = path(:,1:2);

if strcmp(o.lidarMode,'robotSensor')
    if ~o.movePlatform || isempty(o.robotLidar) || isempty(scen)
        error('lidar_react:robotSensorSetup','lidarMode=robotSensor needs movePlatform=true, a robotScenario scen, and opts.robotLidar=robot_lidar_sensor(''attach'',plat,cfg)');
    end
end
% ---- world geometry ----------------------------------------------------
knownPolys  = boxPolys(o.boxes);
if o.autoUnseen && isempty(o.unseenBoxes)
    o.unseenBoxes = autoUnseenBox(path);
    notes{end+1} = 'autoUnseen: synthetic unseen box added on route';
end
truePolys   = [knownPolys, boxPolys(o.unseenBoxes)];   % what the lidar/collision see
[segA, segB] = polySegments(truePolys);

% ---- grids ---------------------------------------------------------------
G = makeGrid(o);
staticOcc = rasterPolys(G, knownPolys);
staticNear = conv2(double(staticOcc), ones(3), 'same') > 0;   % 1-cell tolerance
unseenOcc = false(size(staticOcc));

pose = o.startPose(:)';
curPath = path;
tNow = 0; replans = 0; collided = false; goalReached = false; platFail = false; lastScan = []; lastScanT = -inf; noScanT = 0; stopReason = ''; rsUpdates = 0; tebErrors = {}; tebSteps = 0; coreSteps = 0;
minRange = inf; logT = [0 pose];
sensorReturnCount = 0;
pIdx = 1; stuckT = 0; lastPose = pose; blockedSince = NaN; lastReplanT = -inf; C = cornersOf(curPath); wi = min(2,size(C,1));

teb = []; useTEB = o.useTEB && caps.controllerTEB;
if o.useTEB && ~caps.controllerTEB
    tebErrors{end+1} = 'controllerTEB unavailable'; notes{end+1} = 'TEB REQUESTED BUT UNAVAILABLE: core controller used, TEB not run';
end
if useTEB
    [teb, tebNote] = tebInit(G, staticOcc, curPath, o);
    if isempty(teb), useTEB = false; tebErrors{end+1} = tebNote; end
    notes{end+1} = tebNote;
end
goal = path(end,:);
fpBody = o.footprint;
growFp = fpBody + 0.03*fpBody./max(vecnorm(fpBody,2,2),1e-9);

while tNow < o.maxTime
    % --- simulated lidar (own ray caster against true geometry) -----------
    if strcmp(o.lidarMode,'robotSensor')
        % MathWorks robotSensor + robotLidarPointCloudGenerator path (STATIC-ONLY, UNTESTED).
        % Errors propagate on purpose. Requires movePlatform=true and a real scen.
        sc = robot_lidar_sensor('read', o.robotLidar, pose);
        if sc.isUpdated
            lastScan = sc; lastScanT = tNow; rsUpdates = rsUpdates + 1; noScanT = 0;
            sensorReturnCount = sensorReturnCount + nnz(isfinite(sc.pts(:,1)));
        end
        % FAIL CLOSED: never invent readings. No motion until a real scan exists, and
        % none while the newest scan is older than o.maxScanAge. Advance the scenario
        % (no platform move) to obtain a measurement, bounded by o.scanTimeout.
        if isempty(lastScan) || (tNow - lastScanT) > o.maxScanAge
            noScanT = noScanT + o.dt;
            if noScanT > o.scanTimeout
                if isempty(lastScan), stopReason = 'STOP: no sensor scan received within scanTimeout';
                else, stopReason = 'STOP: sensor scans stale (updates stopped) beyond scanTimeout'; end
                notes{end+1} = stopReason; break %#ok<AGROW>
            end
            nSub = max(1, round(o.dt*scen.UpdateRate));
            for qq = 1:nSub, advance(scen); end
            updateSensors(scen);
            tNow = tNow + o.dt;
            continue
        end
        rng_ = lastScan.rngs; pts = lastScan.pts; rel = lastScan.rel;
    else
        [rng_, pts, rel] = lidarScan(pose, segA, segB, o);
    end
    minRange = min(minRange, min(rng_));
    rayIdx = find(~isnan(pts(:,1)));
    hit = pts(rayIdx,:);
    rngU = o.lidarRange * ones(size(rng_));       % ranges of hits NOT explained by known map
    if ~isempty(hit)
        ij = worldToIdx(G, hit);
        ok = ij(:,1)>=1 & ij(:,1)<=size(G.shape,1) & ij(:,2)>=1 & ij(:,2)<=size(G.shape,2);
        lin = sub2ind(size(G.shape), ij(ok,1), ij(ok,2));
        isNew = ~staticNear(lin);
        unseenOcc(lin(isNew)) = true;
        okRay = rayIdx(ok); newRay = okRay(isNew);
        rngU(newRay) = rng_(newRay);
        if strcmp(o.lidarMode,'robotSensor')
            % speedLimit's distance threshold is referenced to the BASE.
            % Raw lidar ranges start at the offset laser, not the base.
            rngU(newRay) = hypot(pts(newRay,1)-pose(1),pts(newRay,2)-pose(2));
        end
    end

    % --- explicit collision check against ground truth ---------------------
    fp = transformPoly(fpBody, pose);
    if polyHitsAny(fp, truePolys)
        collided = true; stopReason = 'collision'; notes{end+1} = sprintf('COLLISION at t=%.1f pose=[%.2f %.2f]',tNow,pose(1:2)); break
    end

    % --- goal ---------------------------------------------------------------
    if hypot(pose(1)-goal(1), pose(2)-goal(2)) < o.goalTol
        goalReached = true; stopReason = 'goal_reached'; break
    end

    % --- look-ahead blockage check vs *detected unseen* cells ----------------
    [blocked, ~] = pathBlocked(curPath, pIdx, pose, unseenOcc, G, fpBody, o);
    if blocked && (tNow - lastReplanT >= 1.5)
        if isnan(blockedSince), blockedSince = tNow; end
        lastReplanT = tNow; % The cooldown also applies to failed searches.
        [newPath, usedInfl] = replan(G, staticOcc, unseenOcc, pose, goal, o);
        if ~isempty(newPath)
            curPath = newPath(:,1:2); pIdx = 1; C = newPath; wi = min(2,size(C,1)); replans = replans + 1; blockedSince = NaN; lastReplanT = tNow;
            notes{end+1} = sprintf('t=%.1f replan #%d (%s, %d pts)',tNow,replans,usedInfl,size(newPath,1)); %#ok<AGROW>
            if useTEB, teb.ctrl.ReferencePath = [curPath, headingOf(curPath)]; end
        else
            notes{end+1} = sprintf('t=%.1f replan FAILED (no path)',tNow); %#ok<AGROW>
            if tNow - blockedSince > 20, stopReason = 'no_path'; break, end
        end
    end

    % --- control -------------------------------------------------------------
    v = 0; w = 0; usedTeb = false;
    if useTEB
        try
            [v, w] = tebCommand(teb, pose, unseenOcc, G, o);
            usedTeb = true; tebSteps = tebSteps + 1;
        catch ME
            tebErrors{end+1} = sprintf('t=%.1f %s', tNow, ME.message); %#ok<AGROW>
            notes{end+1} = ['TEB ERROR (core controller took over, TEB NOT working): ' ME.message]; %#ok<AGROW>
            useTEB = false;
        end
    end
    if ~usedTeb
        coreSteps = coreSteps + 1;
        pIdx = nearestIdx(curPath, pIdx, pose);
        [v, w, wi] = rtdControl(C, wi, pose, o);   % rotate-then-drive waypoint tracker
        % slow for unseen cells directly ahead (range-based speed limit)
        v = min(v, speedLimit(rngU, rel, o));
    end
    % --- canonical full-polygon check of EVERY command (next pose) vs known polygons
    %     + lidar-detected unseen cells; no command executes unless it passes ---
    %     (known polygons + lidar-detected unseen cells). Picks first safe command.
    if any(unseenOcc(:))
        [rr, cc] = find(unseenOcc);
        UC = [(cc-0.5)*G.res + G.origin(1), (rr-0.5)*G.res + G.origin(2)];
    else
        UC = zeros(0,2);
    end
    cands = [v w; v/2 w; 0 w; 0 -w; -0.2 0; -0.2 w; -0.2 -w; 0 0];
    if strcmp(o.lidarMode,'robotSensor'), cands = cands(cands(:,1) >= 0, :); end   % front laser only: rear is blind, no reversing
    nextPose = pose;
    for ci = 1:size(cands,1)
        q = [pose(1) + cands(ci,1)*cos(pose(3))*o.dt, pose(2) + cands(ci,1)*sin(pose(3))*o.dt, ...
             wrapToPiLocal(pose(3) + cands(ci,2)*o.dt)];
        if commandSafe(pose, q, growFp, knownPolys, UC, G.res), nextPose = q; break, end
    end
    pose = nextPose;
    tNow = tNow + o.dt;
    logT(end+1,:) = [tNow pose]; %#ok<AGROW>

    if o.movePlatform
        try
            movePlat(plat, scen, pose);
            if strcmp(o.lidarMode,'robotSensor')
                nSub = max(1, round(o.dt*scen.UpdateRate));   % control dt -> scenario steps
                for qq = 1:nSub, advance(scen); end
                updateSensors(scen);
            end
        catch ME
            platFail = true;
            stopReason = 'platform_error';
            notes{end+1} = ['PLATFORM MOVE FAILED (success withheld): ' ME.message]; %#ok<AGROW>
            break
        end
    end
    if norm(pose(1:2)-lastPose(1:2)) > 0.02, lastPose = pose; stuckT = tNow; end
    if tNow - stuckT > 30
        stopReason = 'stuck'; notes{end+1} = 'STUCK >30 s without progress'; break
    end
end

if isempty(stopReason), stopReason = 'timeout'; end
if strcmp(o.lidarMode,'robotSensor')
    lidarSrc = 'robotSensor + robotLidarPointCloudGenerator';
else
    lidarSrc = 'own 2-D ray caster (distinct simulation, not the MathWorks sensor)';
end
reactOut = struct('success', goalReached && ~collided && ~platFail, 'collided', collided, ...
    'goalReached', goalReached, 'replanCount', replans, ...
    'unseenDetected', nnz(unseenOcc) > 0, 'unseenCellCount', nnz(unseenOcc), ...
    'log', struct('t',logT(:,1),'x',logT(:,2),'y',logT(:,3),'yaw',logT(:,4)), ...
    'pathFinal', curPath, 'minLidarRange', minRange, 'simTime', tNow, ...
    'caps', caps, 'notes', {notes}, 'platformMoveFailed', platFail, ...
    'tebRequested', o.useTEB, 'tebSteps', tebSteps, 'coreSteps', coreSteps, 'tebErrors', {tebErrors}, ...
    'lidarSource', lidarSrc, 'stopReason', stopReason, 'robotSensorUpdates', rsUpdates, 'planner', 'orientation-aware SE2 footprint lattice (no inflation shrinking)',  'unseenBoxes', o.unseenBoxes);
reactOut.robotSensorFiniteReturns = sensorReturnCount;
reactOut.log.yaw = unwrap(reactOut.log.yaw); % Clearance consumes continuous yaw.
reactOut.finalPose = pose;
reactOut.goalError = norm(pose(1:2)-goal);
end

% =========================================================================
function safe = commandSafe(p, q, fp, knownPolys, cells, cellSize)
% Certify linear XY / shortest-yaw interpolation of the executed command.
% Distance to any point on the footprint changes by at most this bound
% from the nearer endpoint. Detected cells are full squares, not points.
obstacles = knownPolys;
for u = 1:size(cells,1)
    obstacles{end+1} = cells(u,:) + [-1 -1;1 -1;1 1;-1 1]*cellSize/2; %#ok<AGROW>
end
safe = intervalSafe(p,q,fp,obstacles,0);
end

function safe = intervalSafe(p,q,fp,obstacles,depth)
radius = max(vecnorm(fp,2,2));
turn = wrapToPiLocal(q(3)-p(3));
bound = norm(q(1:2)-p(1:2))/2 + 2*radius*sin(abs(turn)/4);
safe = false; uncertain = false;
for endpoint = [p; q]'
    poly = transformPoly(fp,endpoint');
    for u = 1:numel(obstacles)
        wall = obstacles{u};
        gap = max(0,max(min(wall)-max(poly),min(poly)-max(wall)));
        if norm(gap) > bound + 1e-9, continue; end
        distance = polygon_clearance(poly,wall);
        if distance <= 1e-9, return; end
        if distance <= bound + 1e-9, uncertain = true; end
    end
end
if ~uncertain, safe = true; return; end
% A coarse endpoint bound can be inconclusive even with positive clearance.
% Refine the sweep instead of mistaking that uncertainty for contact.
% Reaching the refinement cap still fails closed.
if depth >= 8, return; end
mid = [(p(1:2)+q(1:2))/2, p(3)+turn/2];
safe = intervalSafe(p,mid,fp,obstacles,depth+1) && ...
       intervalSafe(mid,q,fp,obstacles,depth+1);
end

function o = defaults(in, path)
o = in;
d = struct('boxes',zeros(0,7),'unseenBoxes',zeros(0,7),'autoUnseen',false, ...
    'footprint',[-0.474027 -0.236085; -0.439393 -0.319793; -0.355792 -0.354383; ...
                  0.493050 -0.295637;  0.548024  0.236692;  0.513393  0.320393; ...
                  0.429789  0.354984; -0.417838  0.295634], ...   % polygon v2 (integration contract; verify_robot owns verification)
    'res',0.05, ...
    'lidarRange',8,'nRays',360,'fovDeg',360,'noiseStd',0, ...
    'vmax',0.5,'wmax',0.8,'lookahead',0.6,'goalTol',0.15,'dt',0.1,'maxTime',400, ...
    'checkAhead',1.5,'safetyMargin',0.10,'movePlatform',false,'useTEB',false,'lidarMode','ray','robotLidar',[],'maxScanAge',0.5,'scanTimeout',5);
f = fieldnames(d);
for k = 1:numel(f)
    if ~isfield(o,f{k}) || isempty(o.(f{k})), o.(f{k}) = d.(f{k}); end
end
if ~isfield(o,'startPose') || isempty(o.startPose)
    if size(path,1) > 1
        h = atan2(path(2,2)-path(1,2), path(2,1)-path(1,1));
    else, h = 0; end
    o.startPose = [path(1,1:2) h];
end
allpts = [path(:,1:2); o.startPose(1:2)];
if ~isempty(o.boxes)
    r = 0.5*hypot(o.boxes(:,4),o.boxes(:,5));
    allpts = [allpts; o.boxes(:,1:2)+r; o.boxes(:,1:2)-r];
end
if ~isfield(o,'originXY') || isempty(o.originXY), o.originXY = min(allpts,[],1) - 2; end
if ~isfield(o,'sizeXY')   || isempty(o.sizeXY),   o.sizeXY = max(allpts,[],1) + 2 - o.originXY; end
end

function G = makeGrid(o)
nx = ceil(o.sizeXY(1)/o.res); ny = ceil(o.sizeXY(2)/o.res);
G.res = o.res; G.origin = o.originXY(:)'; G.shape = false(ny, nx);
[cx, cy] = meshgrid((0.5:nx-0.5)*o.res + G.origin(1), (0.5:ny-0.5)*o.res + G.origin(2));
G.cx = cx; G.cy = cy;
end

function ij = worldToIdx(G, xy)   % returns [row col]
ij = [floor((xy(:,2)-G.origin(2))/G.res)+1, floor((xy(:,1)-G.origin(1))/G.res)+1];
end

function P = boxPolys(b)
P = cell(1, size(b,1));
for i = 1:size(b,1)
    hx = b(i,4)/2; hy = b(i,5)/2; c = cos(b(i,7)); s = sin(b(i,7));
    L = [-hx -hy; hx -hy; hx hy; -hx hy];
    P{i} = [b(i,1) + c*L(:,1) - s*L(:,2), b(i,2) + s*L(:,1) + c*L(:,2)];
end
end

function [A, B] = polySegments(P)
A = zeros(0,2); B = zeros(0,2);
for i = 1:numel(P)
    q = P{i}; A = [A; q]; B = [B; q([2:end 1],:)]; %#ok<AGROW>
end
end

function occ = rasterPolys(G, P)
occ = false(size(G.shape));
for i = 1:numel(P)
    q = P{i};
    sel = G.cx >= min(q(:,1))-G.res & G.cx <= max(q(:,1))+G.res & ...
          G.cy >= min(q(:,2))-G.res & G.cy <= max(q(:,2))+G.res;
    in = inpolygon(G.cx(sel), G.cy(sel), q(:,1), q(:,2));
    tmp = false(size(occ)); tmp(sel) = in; occ = occ | tmp;
end
% also mark cells touched by thin boxes (edges): dilate by 1 cell
occ = conv2(double(occ), ones(3), 'same') > 0;
end

function b = autoUnseenBox(path)
d = [0; cumsum(hypot(diff(path(:,1)),diff(path(:,2))))];
i = find(d >= 0.4*d(end), 1);
b = [path(i,1) path(i,2) 0.25 0.5 0.5 0.5 0];
end

function [rngs, pts, rel] = lidarScan(pose, A, B, o)
rel = deg2rad(linspace(-o.fovDeg/2, o.fovDeg/2, o.nRays+(o.fovDeg<360)))';
if o.fovDeg >= 360, rel = rel(1:o.nRays); end
ang = pose(3) + rel;
dx = cos(ang); dy = sin(ang);
ex = (B(:,1)-A(:,1))'; ey = (B(:,2)-A(:,2))';
wx = (A(:,1)-pose(1))'; wy = (A(:,2)-pose(2))';
den = dx.*ey - dy.*ex;
t = (wx.*ey - wy.*ex) ./ den;
u = (wx.*dy - wy.*dx) ./ den;
bad = abs(den) < 1e-12 | t <= 0 | u < 0 | u > 1;
t(bad) = inf;
rngs = min(t, [], 2);
if isempty(rngs), rngs = inf(numel(ang),1); end
if o.noiseStd > 0, rngs = rngs + o.noiseStd*randn(size(rngs)); end
rngs(rngs > o.lidarRange) = inf;
pts = [pose(1)+rngs.*dx, pose(2)+rngs.*dy];
pts(isinf(rngs),:) = NaN;
rngs(isinf(rngs)) = o.lidarRange;
end

function Q = transformPoly(F, pose)
c = cos(pose(3)); s = sin(pose(3));
Q = [pose(1) + c*F(:,1) - s*F(:,2), pose(2) + s*F(:,1) + c*F(:,2)];
end

function hit = polyHitsAny(fp, P)
hit = false;
for i = 1:numel(P)
    q = P{i};
    if max(q(:,1)) < min(fp(:,1)) || min(q(:,1)) > max(fp(:,1)) || ...
       max(q(:,2)) < min(fp(:,2)) || min(q(:,2)) > max(fp(:,2)), continue, end
    % vertices inside each other, or any edge crossing
    if any(inpolygon(q(:,1),q(:,2),fp(:,1),fp(:,2))) || any(inpolygon(fp(:,1),fp(:,2),q(:,1),q(:,2)))
        hit = true; return
    end
    if edgesCross(fp, q), hit = true; return, end
end
end

function c = edgesCross(P, Q)
c = false;
n = size(P,1); m = size(Q,1);
for i = 1:n
    a = P(i,:); b = P(mod(i,n)+1,:);
    for j = 1:m
        cc = Q(j,:); d = Q(mod(j,m)+1,:);
        o1 = cr(a,b,cc); o2 = cr(a,b,d); o3 = cr(cc,d,a); o4 = cr(cc,d,b);
        if o1*o2 < 0 && o3*o4 < 0, c = true; return, end
    end
end
end
function z = cr(a,b,c), z = (b(1)-a(1))*(c(2)-a(2)) - (b(2)-a(2))*(c(1)-a(1)); end

function [blocked, where] = pathBlocked(path, idx, pose, unseenOcc, G, fpBody, o)
blocked = false; where = [];
if ~any(unseenOcc(:)), return, end
[r, c] = find(unseenOcc);
U = [(c-0.5)*G.res + G.origin(1), (r-0.5)*G.res + G.origin(2)];
% walk along path for checkAhead metres, test footprint (grown by margin)
s = 0; k = max(idx,1); p = pose(1:2);
grow = fpBody + o.safetyMargin * fpBody ./ max(vecnorm(fpBody,2,2),1e-9);
while s <= o.checkAhead && k < size(path,1)
    q = path(k+1,:); seg = q - p; L = norm(seg);
    h = atan2(seg(2), seg(1)); if L < 1e-6, h = pose(3); end
    travel = min(L,max(0,o.checkAhead-s));
    samples = max(1,ceil(travel/(G.res/2)));
    for distance = linspace(0,travel,samples+1)
        probe = p + seg*(distance/max(L,eps));
        fp = transformPoly(grow, [probe h]);
        near = U(U(:,1) >= min(fp(:,1)) & U(:,1) <= max(fp(:,1)) & ...
                 U(:,2) >= min(fp(:,2)) & U(:,2) <= max(fp(:,2)), :);
        if ~isempty(near) && any(inpolygon(near(:,1), near(:,2), fp(:,1), fp(:,2)))
            blocked = true; where = probe; return
        end
    end
    if travel < L, break; end
    s = s + max(L, 0.05); p = q; k = k + 1;
end
end

function [path, usedInfl] = replan(G, staticOcc, unseenOcc, pose, goalXY, o)
% Orientation-aware footprint lattice planner (see plan_se2). No radius shrinking.
usedInfl = 'SE2 footprint lattice';
path = plan_se2(G, staticOcc | unseenOcc, pose, goalXY, o.footprint);
% Preserve heading-only actions; collapsing them forces the controller to
% take a different (possibly obstructed) rotation from the planned sweep.
end

function path = plan_se2(G, occ, pose, goalXY, FP)
% State (row,col,heading) on a 2*res grid, K=8 headings, exact footprint cell mask per
% heading (polygon grown by 0.71 cell). Actions: rotate +-45 deg (cost .4), step
% forward along heading. Start = actual pose, any heading with a free footprint.
K = 8; f = 2; tol = 0.2; path = [];
[ny, nx] = size(occ); py = ceil(ny/f); px = ceil(nx/f);
pad = false(py*f, px*f); pad(1:ny,1:nx) = occ;
oc = reshape(any(any(reshape(pad, f, py, f, px), 1), 3), py, px);
rp = G.res*f; r = ceil(0.75/rp); [xx, yy] = meshgrid(-r:r);
pts = [xx(:)*rp, yy(:)*rp];
mDi = cell(1,K); mDj = cell(1,K);
Gp = FP + 0.71*rp*FP./max(vecnorm(FP,2,2),1e-9);
for k = 1:K
    q = transformPoly(Gp, [0 0 2*pi*(k-1)/K]);
    in = inpolygon(pts(:,1), pts(:,2), q(:,1), q(:,2));
    mDi{k} = yy(in); mDj{k} = xx(in);
end
% Rotation edges must cover intermediate headings, not only endpoints.
rotDi=cell(1,2*K);rotDj=cell(1,2*K);
for k=1:K
    for direction=[1 -1]
        covered=false(size(xx));
        for fraction=linspace(0,1,17)
            poly=transformPoly(Gp,[0 0 2*pi*(k-1)/K+direction*fraction*2*pi/K]);
            covered=covered|reshape(inpolygon(pts(:,1),pts(:,2),poly(:,1),poly(:,2)),size(xx));
        end
        slot=k+(direction<0)*K;
        rotDi{slot}=yy(covered);rotDj{slot}=xx(covered);
    end
end
cellOf = @(xy) [floor((xy(2)-G.origin(2))/rp)+1, floor((xy(1)-G.origin(1))/rp)+1];
s0 = cellOf(pose(1:2)); g0 = cellOf(goalXY(:)');
gS = inf(py, px, K); par = zeros(py, px, K); closed = false(py, px, K);
openL = zeros(1,0); openF = zeros(1,0);
startHeading = mod(round(pose(3)*K/(2*pi)),K)+1;
for k = startHeading
    if footFree(oc, mDi, mDj, s0(1), s0(2), k, py, px)
        d = abs(wrapToPiLocal(pose(3) - 2*pi*(k-1)/K));
        c0 = 0.4*d/(pi/4); li = sub2ind([py px K], s0(1), s0(2), k);
        gS(li) = c0; openL(end+1) = li; openF(end+1) = c0; %#ok<AGROW>
    end
end
if isempty(openL), return, end
endL = 0;
while ~isempty(openL)
    [~, m] = min(openF); cur = openL(m); openL(m) = []; openF(m) = [];
    if closed(cur), continue, end
    closed(cur) = true;
    [i, j, k] = ind2sub([py px K], cur);
    if hypot(i-g0(1), j-g0(2))*rp <= tol, endL = cur; break, end
    th = 2*pi*(k-1)/K; di = sign(round(sin(th))); dj = sign(round(cos(th)));
    nbs = [i j mod(k,K)+1 0.4; i j mod(k-2,K)+1 0.4; i+di j+dj k rp*hypot(di,dj)];
    for q = 1:3
        ni = nbs(q,1); nj = nbs(q,2); nk = nbs(q,3);
        if ni < 1 || nj < 1 || ni > py || nj > px, continue, end
        li = sub2ind([py px K], ni, nj, nk);
        if closed(li) || ~footFree(oc, mDi, mDj, ni, nj, nk, py, px), continue, end
        if q<=2
            slot=k+(q==2)*K;
            if ~footFree(oc,rotDi,rotDj,i,j,slot,py,px),continue,end
        end
        ng = gS(cur) + nbs(q,4);
        if ng < gS(li)
            gS(li) = ng; par(li) = cur; openL(end+1) = li; %#ok<AGROW>
            openF(end+1) = ng + hypot(ni-g0(1), nj-g0(2))*rp; %#ok<AGROW>
        end
    end
end
if endL == 0, return, end
idx = endL;
while par(idx(end)) ~= 0, idx(end+1) = par(idx(end)); end %#ok<AGROW>
idx = fliplr(idx);
[I, J, H] = ind2sub([py px K], idx); % Two outputs merge heading pages into J.
xy = [(J(:)-0.5)*rp + G.origin(1), (I(:)-0.5)*rp + G.origin(2)];
keep=true(numel(H),1);
for n=2:numel(H)-1
    if H(n-1)==H(n) && H(n)==H(n+1) && ...
            norm(xy(n,:)-xy(n-1,:))>eps && norm(xy(n+1,:)-xy(n,:))>eps
        keep(n)=false;
    end
end
headings=2*pi*(H(:)-1)/K;
path = [pose;xy(keep,:),headings(keep);goalXY(:)',headings(end)];
end

function ok = footFree(oc, mDi, mDj, i, j, k, py, px)
ii = i + mDi{k}; jj = j + mDj{k};
if any(ii < 1 | jj < 1 | ii > py | jj > px), ok = false; return, end
ok = ~any(oc(sub2ind([py px], ii, jj)));
end

function out = densify(q)
out = q(1,:);
for k = 2:size(q,1)
    L = norm(q(k,:)-q(k-1,:)); m = max(1, ceil(L/0.25));
    for a = 1:m, out(end+1,:) = q(k-1,:) + (q(k,:)-q(k-1,:))*a/m; end %#ok<AGROW>
end
end

function C = cornersOf(p)
% corner waypoints of a piecewise-straight path (heading changes > 1e-3 rad)
C = p(1,:);
for k = 2:size(p,1)-1
    a = p(k,:)-p(k-1,:); b = p(k+1,:)-p(k,:);
    if abs(wrapToPiLocal(atan2(b(2),b(1)) - atan2(a(2),a(1)))) > 1e-3, C(end+1,:) = p(k,:); end %#ok<AGROW>
end
C(end+1,:) = p(end,:);
end

function idx = nearestIdx(path, idx, pose)
win = idx:min(size(path,1), idx+40);
[~, k] = min(hypot(path(win,1)-pose(1), path(win,2)-pose(2))); idx = win(k);
end

function [v, w, wi] = rtdControl(C, wi, pose, o)
% Rotate-then-drive: turn in place until aligned (|err|<=0.05 rad), then drive straight
% to the next corner; no corner cutting, so motion matches the lattice/straight segments.
if size(C,2)==3
    while wi < size(C,1) && norm(C(wi,1:2)-pose(1:2)) < 0.06 && ...
            abs(wrapToPiLocal(C(wi,3)-pose(3))) < 0.05
        wi=wi+1;
    end
end
while size(C,2)==2 && wi < size(C,1) && hypot(C(wi,1)-pose(1), C(wi,2)-pose(2)) < 0.06, wi = wi + 1; end
d = C(wi,1:2) - pose(1:2); dist = hypot(d(1), d(2));
if size(C,2)==3 && dist < 0.06
    err=wrapToPiLocal(C(wi,3)-pose(3));
    v=0;w=max(-o.wmax,min(o.wmax,2*err));return;
end
err = wrapToPiLocal(atan2(d(2),d(1)) - pose(3));
if abs(err) > 0.05
    v = 0; w = max(-o.wmax, min(o.wmax, 2*err));
    if abs(w) < 0.15, w = 0.15*sign(err); end
else
    v = min(o.vmax, max(0.1, dist)); w = max(-o.wmax, min(o.wmax, 2*err));
end
end

function vlim = speedLimit(rngs, rel, o)
% range-based speed cap from rays within +-15 deg of heading
front = min(rngs(abs(rel) <= deg2rad(15)));
if isempty(front), front = min(rngs); end
vlim = max(0, min(o.vmax, 0.6*(front - 0.65)));
if front > 1.5, vlim = o.vmax; end
end

function a = wrapToPiLocal(a), a = mod(a+pi, 2*pi) - pi; end
function h = headingOf(p)
d = diff(p,1,1); h = atan2(d(:,2), d(:,1)); h = [h; h(end)];
end

function movePlat(plat, scen, pose)   %#ok<INUSD>
% Optional; errors PROPAGATE to caller (success withheld). Signature follows the
% MathWorks example move(platform,"base",[x y z vel6 quat vel3]); UNTESTED here.
move(plat, "base", [pose(1) pose(2) 0 zeros(1,6) eul2quat([pose(3) 0 0]) zeros(1,3)]);
end

% ---- optional controllerTEB branch (R2024b+, Navigation Toolbox) UNTESTED ----
function [tc, note] = tebInit(G, staticOcc, path, o)
tc = [];
try
    cpm = 1/G.res;
    gm = occupancyMap(double(staticOcc), cpm);
    gm.GridLocationInWorld = G.origin;
    lm = occupancyMap(15, 15, cpm);
    ctrl = controllerTEB([path, headingOf(path)], lm);
    ctrl.MaxVelocity = [o.vmax o.wmax];
    ctrl.LookAheadTime = 3;
    ctrl.ObstacleSafetyMargin = o.safetyMargin;
    ctrl.GoalTolerance = [o.goalTol o.goalTol 0.3];
    ctrl.RobotInformation = struct(Dimension=[1.02 0.64], Shape="Rectangle");
    tc = struct('ctrl', ctrl, 'local', lm, 'global', gm);
    note = 'controllerTEB initialised (UNTESTED; MathWorks example pattern)';
catch ME
    tc = []; note = ['controllerTEB init failed -> core used: ' ME.message];
end
end

function [v, w] = tebCommand(tc, pose, unseenOcc, G, ~)
lm = tc.local; gm = tc.global;
move(lm, pose(1:2) - lm.XLocalLimits(end)/2);
syncWith(lm, gm);
[r, c] = find(unseenOcc);
if ~isempty(r)
    setOccupancy(lm, [(c-0.5)*G.res + G.origin(1), (r-0.5)*G.res + G.origin(2)], 1);
end
[cmd, ~, ~, info] = step(tc.ctrl, pose, [0 0]);
if info.ExitFlag == 1 || isempty(cmd), error('TEB trajectory invalid'); end
v = cmd(1,1); w = cmd(1,2);
end
