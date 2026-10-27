function map = build_map(scen, res, originXY, opts)
%BUILD_MAP Occupancy map from the world wall boxes (2-D projection).
%   map = build_map(scen, res, originXY)
%   scen     : Nx7 [xc yc zc sx sy sz yaw] (m, rad), OR a struct/object with a
%              .Boxes (Nx7) field, OR a CSV path (read_world_boxes format).
%              (or pass opts.Boxes). NOTE: project contract says "scenario"; a robotics
%              drivingScenario/robotScenario has no Boxes field, so the
%              boxes must be passed (or attached as scen.Boxes).
%   res      : ONE rule for every module (see norm_res): 0<res<1 is a CELL SIZE
%              in metres (0.05 -> 20 cells/m); res>=1 is cells per metre
%              (20 -> 0.05 m cells). res=1 is the same either way. The map's
%              Resolution property is always cells/m (occupancyMap convention).
%   originXY : [x y] world coords of the map LOWER-LEFT corner. [] -> auto
%              (bbox min minus margin, snapped to a whole cell).
%   opts     : struct, optional: Boxes (Nx7/CSV; overrides scen), ZBand [zmin zmax] (default [0.05 1.0], boxes
%              not overlapping this band are ignored), Margin (m, 0.5),
%              Inflate (m, extra obstacle growth, 0), XMax/YMax (m, world
%              upper-right extent; default bbox max + margin).
%   Cell is occupied if it OVERLAPS a box (conservative). For yaw~=0 boxes
%   the overlap test is exact for the cell-centre point against the box grown
%   by half a cell on each box axis (slight over-approximation at corners).
%   Returns occupancyMap (Robotics System Toolbox) if available, else a
%   struct with Grid (row 1 = max y), Resolution, Origin, Boxes.
if nargin < 4 || isempty(opts), opts = struct; end
zb = getf(opts,'ZBand',[0.05 1.0]); mg = getf(opts,'Margin',0.5);
infl = getf(opts,'Inflate',0);
if nargin < 2 || isempty(res), res = 20; end
res = norm_res(res);   % -> cells per metre
if isfield(opts,'Boxes'), boxes = get_boxes(opts.Boxes); else, boxes = get_boxes(scen); end
keep = (boxes(:,3)-boxes(:,6)/2 < zb(2)) & (boxes(:,3)+boxes(:,6)/2 > zb(1));
b = boxes(keep,:);
if isempty(b), error('build_map:empty','no boxes intersect ZBand'); end
ext = 0.5*(abs(b(:,4).*cos(b(:,7)))+abs(b(:,5).*sin(b(:,7))));
eyx = 0.5*(abs(b(:,4).*sin(b(:,7)))+abs(b(:,5).*cos(b(:,7))));
lo = [min(b(:,1)-ext) min(b(:,2)-eyx)] - mg;
hi = [max(b(:,1)+ext) max(b(:,2)+eyx)] + mg;
if nargin < 3 || isempty(originXY), originXY = floor(lo*res)/res; end
originXY = originXY(:).';
if isfield(opts,'XMax'), hi(1) = opts.XMax; end
if isfield(opts,'YMax'), hi(2) = opts.YMax; end
nc = ceil((hi(1)-originXY(1))*res - 1e-9);
nr = ceil((hi(2)-originXY(2))*res - 1e-9);
if any(originXY > lo + 1e-9)
    warning('build_map:origin','originXY is inside the wall bounding box; walls will be clipped.');
end
xc = originXY(1) + ((1:nc)-0.5)/res;   % cell-centre x
yc = originXY(2) + ((1:nr)-0.5)/res;   % cell-centre y (bottom -> top)
[X,Y] = meshgrid(xc,yc);
occ = false(nr,nc);
h = 0.5/res + infl;
for i = 1:size(b,1)
    dx = X-b(i,1); dy = Y-b(i,2);
    c = cos(b(i,7)); s = sin(b(i,7));
    u =  c*dx + s*dy; v = -s*dx + c*dy;
    occ = occ | (abs(u) <= b(i,4)/2+h+1e-9 & abs(v) <= b(i,5)/2+h+1e-9);
end
grid = flipud(occ);   % row 1 = top (max y)
if exist('occupancyMap','class') || exist('occupancyMap','file')
    map = occupancyMap(double(grid), res);
    map.GridLocationInWorld = originXY;
else
    map = struct('Grid',grid,'Resolution',res,'Origin',originXY,'Boxes',b);
end
end

function boxes = get_boxes(scen)
if ischar(scen) || isstring(scen)
    boxes = read_world_boxes(scen);   % handles id|name header, never raw readmatrix
elseif isnumeric(scen)
    boxes = scen;
elseif isstruct(scen) && isfield(scen,'Boxes')
    boxes = scen.Boxes;
elseif isobject(scen) && isprop(scen,'Boxes')
    boxes = scen.Boxes;
else
    error('build_map:scen','scen must be Nx7 boxes, a CSV path, or have a .Boxes field.');
end
if size(boxes,2) ~= 7, error('build_map:boxes','boxes must be Nx7'); end
end

function r = norm_res(res)
if ~isscalar(res) || ~isnumeric(res) || ~(res > 0) || ~isfinite(res)
    error('build_map:res','res must be a positive scalar; got %s.',mat2str(res));
end
if res < 1, r = 1/res; else, r = res; end
end

function v = getf(s,f,d)
if isfield(s,f), v = s.(f); else, v = d; end
end
