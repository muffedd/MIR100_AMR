function [ok, rep] = check_map(map, boxes, res, originXY, startXY, goalXY, routeXY)
%CHECK_MAP Independent checks of a map from build_map against the CSV boxes.
%   res: <1 = cell size in m (0.05), >=1 = cells/m (20); same rule as build_map
%   ok = check_map(map, boxes, res, originXY)
%   [ok,rep] = check_map(map, boxes, res, originXY, startXY, goalXY, routeXY)
%   Defaults: startXY=[-4.31 -2.81], goalXY=[4.31 0.56] (world frame),
%   routeXY = v2 route (map frame) + [-4.31 -2.81].
%   Checks (all queried in WORLD coords through the map, so a row-flip or
%   origin error fails):
%    1 resolution/origin match what was requested
%    2 every wall (z-band) : centre + 4 inner points occupied
%    3 points 2 cells outside a wall face are free unless inside another wall
%    4 free-space sample: cells > 2 cells from every wall are free
%    5 start/goal inside map bounds and free; route vertices free
if nargin < 5 || isempty(startXY), startXY = [-4.31 -2.81]; end
if nargin < 6 || isempty(goalXY),  goalXY  = [4.31 0.56]; end
if nargin < 7 || isempty(routeXY)
    routeXY = [0 0;0 1.12;0 2.81;.56 3.09;1.12 3.37;1.12 4.5;1.12 5.62;2.25 5.62; ...
        3.37 5.62;4.5 5.62;5.25 5.25;5.25 4.12;5.25 3.37;6.75 3.37;7.87 3.37;8.62 3.37] ...
        + [-4.31 -2.81];
end
if ischar(boxes) || isstring(boxes), boxes = read_world_boxes(boxes); end
rep = struct('msgs',{{}});
% 1
if res < 1, res = 1/res; end   % same rule as build_map: <1 = cell size (m), >=1 = cells/m
[mres, morg, w, hgt] = info(map);
if abs(mres-res) > 1e-9, rep.msgs{end+1} = sprintf('resolution %g ~= %g',mres,res); end
if ~isempty(originXY) && any(abs(morg-originXY(:).') > 1e-9), rep.msgs{end+1} = 'origin mismatch'; end
zb = [0.05 1.0];
b = boxes(boxes(:,3)-boxes(:,6)/2 < zb(2) & boxes(:,3)+boxes(:,6)/2 > zb(1),:);
d = 2/res; nIn = 0; nOut = 0;
for i = 1:size(b,1)
    c = cos(b(i,7)); s = sin(b(i,7)); R = [c -s; s c];
    loc = [0 0; .45 0; -.45 0; 0 .45; 0 -.45] .* [b(i,4) b(i,5)];
    inner = b(i,1:2) + loc*R.';
    % 2: only test points whose containing cell is fully inside is overkill;
    % inner points at 45% of size are inside the box by construction.
    o = occ_at(map, inner);
    if ~all(o), rep.msgs{end+1} = sprintf('wall %d interior not occupied',i); end
    nIn = nIn + numel(o);
    % 3: outside probes on the 4 faces (wall centre line)
    off = [ (b(i,4)/2+d) 0; -(b(i,4)/2+d) 0; 0 (b(i,5)/2+d); 0 -(b(i,5)/2+d)];
    out = b(i,1:2) + off*R.';
    for k = 1:4
        if ~in_any(b, out(k,:), 0.5/res+1e-6) && inside_map(map,out(k,:))
            nOut = nOut + 1;
            if occ_at(map,out(k,:)), rep.msgs{end+1} = sprintf('wall %d: cell outside face %d occupied',i,k); end
        end
    end
end
% 4: free sample on a coarse lattice
[xs,ys] = meshgrid(morg(1)+0.5/res : 0.25 : morg(1)+w, morg(2)+0.5/res : 0.25 : morg(2)+hgt);
P = [xs(:) ys(:)]; nf = 0;
for k = 1:size(P,1)
    if ~in_any(b, P(k,:), d)
        nf = nf + 1;
        if occ_at(map,P(k,:)), rep.msgs{end+1} = sprintf('free sample (%.2f,%.2f) occupied',P(k,:)); end
    end
end
% 5
pts = {startXY,'start'; goalXY,'goal'};
for k = 1:2
    p = pts{k,1};
    if ~inside_map(map,p), rep.msgs{end+1} = [pts{k,2} ' outside map'];
    elseif occ_at(map,p), rep.msgs{end+1} = [pts{k,2} ' cell occupied']; end
end
for k = 1:size(routeXY,1)
    if ~inside_map(map,routeXY(k,:)) || occ_at(map,routeXY(k,:))
        rep.msgs{end+1} = sprintf('route vertex %d (%.2f,%.2f) not free',k,routeXY(k,:)); end
end
ok = isempty(rep.msgs);
rep.nWalls = size(b,1); rep.nInteriorProbes = nIn; rep.nOutsideProbes = nOut; rep.nFreeProbes = nf;
rep.occupiedFraction = mean(occ_at(map,P));
end

function [res, org, w, h] = info(map)
if isstruct(map)
    res = map.Resolution; org = map.Origin;
    w = size(map.Grid,2)/res; h = size(map.Grid,1)/res;
else
    res = map.Resolution; org = map.GridLocationInWorld;
    w = map.XWorldLimits(2)-map.XWorldLimits(1); h = map.YWorldLimits(2)-map.YWorldLimits(1);
end
end
function tf = inside_map(map,p)
[~,org,w,h] = info(map);
tf = p(1)>=org(1) && p(1)<org(1)+w && p(2)>=org(2) && p(2)<org(2)+h;
end
function o = occ_at(map,xy)
if isstruct(map)
    r = map.Resolution; [nr,nc] = size(map.Grid);
    col = floor((xy(:,1)-map.Origin(1))*r)+1;
    row = nr - floor((xy(:,2)-map.Origin(2))*r);
    o = false(size(xy,1),1);
    v = col>=1 & col<=nc & row>=1 & row<=nr;
    o(v) = map.Grid(sub2ind([nr nc],row(v),col(v)));
else
    o = getOccupancy(map, xy) > 0.5;
end
end
function tf = in_any(b,p,m)
tf = false;
for i = 1:size(b,1)
    c = cos(b(i,7)); s = sin(b(i,7));
    dx = p(1)-b(i,1); dy = p(2)-b(i,2);
    if abs(c*dx+s*dy) <= b(i,4)/2+m && abs(-s*dx+c*dy) <= b(i,5)/2+m, tf = true; return; end
end
end
