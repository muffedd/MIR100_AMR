function [path, info] = plan_path(map, startXY, goalXY, inflateM)
%PLAN_PATH Global grid A* planner on an occupancy map (world metres).
%   [path, info] = plan_path(map, startXY, goalXY, inflateM)
%   map      binaryOccupancyMap / occupancyMap (Robotics System Toolbox or
%            Navigation Toolbox), OR struct with fields
%            Grid (logical, row 1=max y, same as occupancyMatrix),
%            Resolution (cells/m), Origin (world lower-left corner).
%   startXY, goalXY  [x y] metres, world frame.
%   inflateM obstacle inflation radius in metres (default 0.6052, the
%            conservative circumscribed radius of the MiR100 footprint).
%   path     Nx2 world waypoints (metres), or zeros(0,2) on failure.
%   info     struct: ok, reason, lengthM, inflateM, res, minClearanceM,
%            footprintModel ('circle'), limitation (text).
%
% Uses only base MATLAB (no PRM/RRT). Any planner must fail closed; A* is
% chosen for determinism and resolution-completeness (no randomness), not
% because PRM/RRT cannot fail closed. Fails closed: it never
% shrinks inflateM to force a route.
% Circle limitation: a circle of radius r_circ=0.6052 m is wider than the
% real robot (about 0.58 m wide), so it may reject corridors the real
% footprint could pass (e.g. 1.125 m < 2*0.6052). An orientation-aware
% polygon check is not done here; use check_run_clearance on the trajectory.

if nargin < 4 || isempty(inflateM), inflateM = 0.6052; end  % polygon v2 radius; never shrink
info = struct('ok',false,'reason','','lengthM',NaN,'inflateM',inflateM, ...
    'res',NaN,'minClearanceM',NaN,'footprintModel','circle', ...
    'limitation',['Circular footprint is conservative; may fail where ' ...
    'orientation-aware polygon would pass. Not shrunk to force a route.']);
path = zeros(0,2);
startXY = double(startXY(:).'); goalXY = double(goalXY(:).');
if numel(startXY)~=2 || numel(goalXY)~=2 || any(~isfinite([startXY goalXY])) ...
        || ~isscalar(inflateM) || ~isfinite(inflateM) || inflateM < 0
    info.reason = 'invalid input: start/goal must be finite [x y], inflateM finite >= 0';
    return
end

% ---- extract occupancy matrix (row = y ascending, col = x ascending)
% Docs: checkOccupancy(map,xy) -> 0 free, 1 occupied, -1 unknown INCLUDING
% outside the map (mathworks.com/help/nav/ref/binaryoccupancymap.checkoccupancy.html).
% So anything ~= 0 is treated as NOT free. xy is world frame; Resolution is
% cells/m; XWorldLimits/YWorldLimits are [min max]; GridLocationInWorld is
% the bottom-left corner (mathworks.com/help/nav/ref/binaryoccupancymap.html).
% Point queries are used so the (top-left, row-1) matrix orientation of
% getOccupancy/checkOccupancy(map) never matters.
if isstruct(map)
    occ = flipud(logical(map.Grid)); res = map.Resolution; org = map.Origin(:).';
else
    res = map.Resolution;
    xl = map.XWorldLimits; yl = map.YWorldLimits;
    nx = map.GridSize(2); ny = map.GridSize(1);
    xs = xl(1) + ((1:nx)-0.5)/res; ys = yl(1) + ((1:ny)-0.5)/res;
    [X,Y] = meshgrid(xs,ys);
    st = checkOccupancy(map,[X(:) Y(:)]);
    occ = reshape(st ~= 0, ny, nx);   % occupied OR unknown -> blocked
    org = [xl(1) yl(1)];
end
info.res = res;
[ny,nx] = size(occ);

% ---- inflate with disk (outside-map counts as occupied)
% Raster pad: a point anywhere in a free cell vs a point anywhere in an
% occupied cell can be closer than their centre distance by up to two
% half-diagonals = sqrt(2) cells. Pad sqrt(2) cells (+0.05 cell for the
% sampled line-of-sight step) so every accepted cell has true clearance
% >= inflateM from every occupied cell, given the map raster itself is
% conservative (build_map/check_map responsibility).
rEff = inflateM + (sqrt(2)+0.05)/res;
rc = ceil(rEff*res);
[dx,dy] = meshgrid(-rc:rc);
disk = double((dx.^2+dy.^2) <= (rEff*res)^2);
padded = ones(ny+2*rc, nx+2*rc);
padded(rc+1:rc+ny, rc+1:rc+nx) = double(occ);
infl = conv2(padded, disk, 'same') > 0;
infl = infl(rc+1:rc+ny, rc+1:rc+nx);

w2c = @(p) [floor((p(1)-org(1))*res)+1, floor((p(2)-org(2))*res)+1]; % [col row]
s = w2c(startXY); g = w2c(goalXY);
inb = @(c) c(1)>=1 && c(1)<=nx && c(2)>=1 && c(2)<=ny;
if ~inb(s), info.reason = 'start outside map bounds'; return, end
if ~inb(g), info.reason = 'goal outside map bounds'; return, end
if occ(s(2),s(1)), info.reason = 'start lies inside an obstacle'; return, end
if occ(g(2),g(1)), info.reason = 'goal lies inside an obstacle'; return, end
if infl(s(2),s(1)), info.reason = sprintf('start is within inflation radius %.3f m (+raster pad) of an obstacle', inflateM); return, end
if infl(g(2),g(1)), info.reason = sprintf('goal is within inflation radius %.3f m (+raster pad) of an obstacle', inflateM); return, end

% ---- A* (8-connected, no corner cutting)
N = nx*ny; idx = @(c) (c(2)-1)*nx + c(1);
gcost = inf(N,1); parent = zeros(N,1); closed = false(N,1);
h = @(c) hypot(c(1)-g(1), c(2)-g(2));
open = zeros(0,3); % [f, index, tiebreak]
si = idx(s); gi = idx(g); gcost(si) = 0; open(1,:) = [h(s) si 0];
nb = [1 0;-1 0;0 1;0 -1;1 1;1 -1;-1 1;-1 -1];
found = false;
while ~isempty(open)
    [~,k] = min(open(:,1)); cur = open(k,2); open(k,:) = [];
    if closed(cur), continue, end
    closed(cur) = true;
    if cur == gi, found = true; break, end
    cx = mod(cur-1,nx)+1; cy = floor((cur-1)/nx)+1;
    for j = 1:8
        x = cx+nb(j,1); y = cy+nb(j,2);
        if x<1||x>nx||y<1||y>ny || infl(y,x), continue, end
        if j>4 && (infl(cy,x) || infl(y,cx)), continue, end
        ni = (y-1)*nx+x; if closed(ni), continue, end
        ng = gcost(cur) + hypot(nb(j,1),nb(j,2));
        if ng < gcost(ni)
            gcost(ni) = ng; parent(ni) = cur;
            open(end+1,:) = [ng + hypot(x-g(1),y-g(2)), ni, 0]; %#ok<AGROW>
        end
    end
end
if ~found
    info.reason = sprintf(['no collision-free route with inflation %.3f m ' ...
        '(circle footprint); a narrow gap (e.g. 1.125 m corridor < 2*radius) ' ...
        'may be blocking. Not shrinking footprint to force a path.'], inflateM);
    return
end
cells = zeros(0,2); c = gi;
while c ~= 0
    cells(end+1,:) = [mod(c-1,nx)+1, floor((c-1)/nx)+1]; %#ok<AGROW>
    c = parent(c);
end
cells = flipud(cells);

% ---- NO line-of-sight smoothing (project decision): the sampled LOS
% check is not an exact supercover of the inflated grid, so a smoothed
% chord could clip an inflated corner cell. The raw A* grid path is
% already certified collision-free on the inflated grid - keep it.
path = [org(1) + (cells(:,1)-0.5)/res, org(2) + (cells(:,2)-0.5)/res];
path(1,:) = startXY; path(end,:) = goalXY;
info.ok = true; info.reason = 'ok';
info.lengthM = sum(hypot(diff(path(:,1)),diff(path(:,2))));
end

