function clearLog = check_run_clearance(trajLog, boxes, opts)
%CHECK_RUN_CLEARANCE MiR100 full-footprint distance to oriented world wall boxes.
% clearLog = check_run_clearance(trajLog, boxes)
% trajLog: scalar struct/table with equal-length t,x,y,yaw vectors. Units s,m,rad.
% boxes: N-by-7 [xc yc zc sx sy sz yaw], metres/radians, world frame.
% opts optional struct fields:
%   Footprint          N-by-2 base-frame envelope (default verified MiR100 hull).
%   MaxTranslationStep 0.04 m, MaxYawStep pi/60 rad, Tolerance 1e-9 m.
%   MaxRefinementSamples 2000000 (errors rather than silently under-sampling).
%
% Default footprint expands the original lead polygon outward by 6 mm on each
% supporting edge, to cover caster sweep (lead polygon was 5.24 mm too small).
% It is a conservative polygon enclosing the original collision
% meshes including laser housings and all caster angles, NOT a point or circle.
% Every supplied box is projected into XY. This is conservative for elevated
% objects; hard_world.sdf's 65 walls all span floor to 2.2 m.
%
% Inter-sample certificate is conditional on straight-line XY and linear yaw
% interpolation between logged poses. Yaw is used as supplied, WITHOUT wrapping:
% supply continuous/unwrapped yaw, otherwise a +/-pi crossing means a long turn.
% No finite log proves unobserved physical excursions or wheel-slip behavior.
if nargin < 3, opts = struct(); end
validateattributes(boxes,{'numeric'},{'2d','ncols',7,'real','finite'});
if any(any(boxes(:,4:6)<=0))
    error('mir100:InvalidBoxes','Box dimensions must be positive.');
end
if ~isstruct(opts) || ~isscalar(opts)
    error('mir100:InvalidOptions','opts must be a scalar struct.');
end
fp = getOption(opts,'Footprint',[-.474026970 -.236085370;...
    -.439392950 -.319792950;-.355791680 -.354382980;...
    .493049750 -.295637190;.548024070 .236692380;...
    .513392950 .320392950;.429789310 .354983960;...
    -.417838050 .295633920]);
validateattributes(fp,{'numeric'},{'2d','ncols',2,'real','finite'});
if size(fp,1)<3, error('mir100:InvalidFootprint','Need at least 3 vertices.'); end
stepXY = getOption(opts,'MaxTranslationStep',0.04);
stepYaw = getOption(opts,'MaxYawStep',pi/60);
tol = getOption(opts,'Tolerance',1e-9);
cap = getOption(opts,'MaxRefinementSamples',2000000);
validateattributes(stepXY,{'numeric'},{'scalar','finite','real','positive'});
validateattributes(stepYaw,{'numeric'},{'scalar','finite','real','positive'});
validateattributes(tol,{'numeric'},{'scalar','finite','real','nonnegative'});
validateattributes(cap,{'numeric'},{'scalar','finite','integer','positive'});
fields = {'t','x','y','yaw'}; data = cell(1,4);
for k=1:4
    v = trajLog.(fields{k});
    validateattributes(v,{'numeric'},{'vector','real','finite','nonempty'});
    data{k} = double(v(:));
end
n = numel(data{1});
if any(cellfun(@numel,data) ~= n)
    error('mir100:InvalidTrajectory','t,x,y,yaw must have matching lengths.');
end
if any(diff(data{1})<=0)
    error('mir100:InvalidTime','Times must be strictly increasing.');
end
t = data{1}; poses = [data{2:4}]; radius = max(sqrt(sum(fp.^2,2)));
subdiv = max(1,ceil(max([sqrt(sum(diff(poses(:,1:2),1,1).^2,2))/stepXY,...
    abs(diff(poses(:,3)))/stepYaw],[],2)));
count = 1+sum(subdiv);
if count>cap
    error('mir100:TooManySamples','Refinement exceeds MaxRefinementSamples.');
end
rt = zeros(count,1); rp = zeros(count,3); originalIndex = zeros(n,1);
rt(1)=t(1); rp(1,:)=poses(1,:); originalIndex(1)=1; at=1;
for k=1:n-1
    m = subdiv(k); f = (1:m)'/m; ids = at+(1:m);
    rt(ids) = t(k)+(t(k+1)-t(k))*f;
    rp(ids,:) = repmat(poses(k,:),m,1)+f*(poses(k+1,:)-poses(k,:));
    at=at+m; originalIndex(k+1)=at;
end
walls = cell(size(boxes,1),1); wallMin = zeros(size(boxes,1),2); wallMax=wallMin;
for j=1:size(boxes,1)
    c=cos(boxes(j,7)); s=sin(boxes(j,7));
    corners=[-1 -1;1 -1;1 1;-1 1].*repmat(boxes(j,4:5)/2,4,1);
    walls{j}=corners*[c s;-s c]+repmat(boxes(j,1:2),4,1);
    wallMin(j,:)=min(walls{j},[],1); wallMax(j,:)=max(walls{j},[],1);
end
dist = inf(count,1); nearestWall=zeros(count,1);
robotPoint=NaN(count,2); wallPoint=robotPoint;
for k=1:count
    c=cos(rp(k,3)); s=sin(rp(k,3));
    poly=fp*[c s;-s c]+repmat(rp(k,1:2),size(fp,1),1);
    % AABB gaps are lower bounds: pruning cannot discard a nearer wall.
    delta=max(0,max(wallMin-repmat(max(poly,[],1),size(boxes,1),1),...
        repmat(min(poly,[],1),size(boxes,1),1)-wallMax));
    [lower,order]=sort(sqrt(sum(delta.^2,2)));
    for jj=1:numel(order)
        if lower(jj)>dist(k)+tol, break; end
        j=order(jj); [d,p,q]=polygon_clearance(poly,walls{j},tol);
        if d<dist(k)
            dist(k)=d; nearestWall(k)=j; robotPoint(k,:)=p; wallPoint(k,:)=q;
        end
        if d<=tol, break; end
    end
end
% Nearest endpoint is <= half a subinterval away in interpolation parameter.
% Point displacement <= ||DeltaXY||/2 + 2*R*sin(min(|DeltaYaw|/4,pi/2)).
% For very large turns use 2*R, not a periodic sine that could underbound.
dxy=sqrt(sum(diff(rp(:,1:2),1,1).^2,2)); dyaw=abs(diff(rp(:,3)));
residual=dxy/2+2*radius*sin(min(dyaw/4,pi/2));
intervalLB=max(0,min(dist(1:end-1),dist(2:end))-residual);
[minD,idx]=min(dist);
if isempty(intervalLB), certifiedLB=minD; else, certifiedLB=min(intervalLB); end
clearLog=struct();
clearLog.t=t; clearLog.minDist=dist(originalIndex);
clearLog.clearance=clearLog.minDist;
clearLog.minOverall=minD;
clearLog.minAt=struct('x',rp(idx,1),'y',rp(idx,2),'t',rt(idx),...
    'box',nearestWall(idx));
clearLog.collisionFlags=clearLog.clearance<=tol;
clearLog.anyCollision=any(dist<=tol);
clearLog.collision=clearLog.anyCollision; % Scalar, true at any original/refined pose.
clearLog.minClearance=minD; clearLog.minClearanceM=minD;
clearLog.minLocation=struct('time',rt(idx),'pose',rp(idx,:),...
    'wallIndex',nearestWall(idx),'robotPoint',robotPoint(idx,:),...
    'wallPoint',wallPoint(idx,:),'refinedSampleIndex',idx);
clearLog.minClearanceXY=robotPoint(idx,:);
clearLog.minWallIndex=nearestWall(idx);
clearLog.refined=struct('t',rt,'poses',rp,'clearance',dist,...
    'wallIndex',nearestWall,'collisionFlags',dist<=tol,...
    'robotPoint',robotPoint,'wallPoint',wallPoint);
clearLog.originalSampleIndices=originalIndex;
clearLog.intervalResidualBound=residual;
clearLog.intervalClearanceLowerBound=intervalLB;
clearLog.continuousClearanceLowerBound=certifiedLB;
clearLog.isCollisionFree=~clearLog.anyCollision && certifiedLB>tol;
clearLog.pass=clearLog.isCollisionFree;
if clearLog.anyCollision
    clearLog.status='collision_or_contact_detected';
elseif clearLog.isCollisionFree
    clearLog.status='certified_clear_under_interpolation_model';
else
    clearLog.status='inconclusive_between_samples';
end
clearLog.footprint=fp; clearLog.footprintRadius=radius;
clearLog.wallCount=size(boxes,1); clearLog.tolerance=tol;
clearLog.interpolationAssumption='Linear XY and supplied continuous yaw between logged poses; static XY wall projections.';
clearLog.scope='Conservative 2-D footprint geometry only; not physics, slip, mesh-level 3-D or real-time validation.';
end

function value=getOption(opts,name,default)
if isfield(opts,name), value=opts.(name); else, value=default; end
end
