function results = test_collision()
%TEST_COLLISION Native MATLAB assertions. Run after addpath(genpath('matlab')).
% These tests are provided, not claimed executed on a MATLAB runtime here.
S=[-1 -1;1 -1;1 1;-1 1];
[d,p,q,hit]=polygon_clearance(S,S+[3 0]);
assert(abs(d-1)<1e-12 && ~hit && abs(norm(p-q)-d)<1e-12);
assert(polygon_clearance(S,S+[2 0])==0); % shared boundary
assert(polygon_clearance(S,S+[2 2])==0); % corner contact
assert(polygon_clearance(S,S/4)==0); % B wholly inside A
assert(polygon_clearance(S/4,S)==0); % A wholly inside B
H=[-2 -.1;2 -.1;2 .1;-2 .1];
V=[-.1 -2;.1 -2;.1 2;-.1 2];
assert(polygon_clearance(H,V)==0); % crossing edges, no contained vertices
D=[-1 -1;-1 -1;1 -1;1 1;-1 1];
assert(abs(polygon_clearance(D,S+[3 0])-1)<1e-12); % repeated vertex
R=[cos(.7) sin(.7);-sin(.7) cos(.7)];
assert(abs(polygon_clearance(S*R,(S+[3 0])*R)-1)<1e-12);
boxes=[0 0 1 0.1 5 2 0];
log=struct('t',[0;1],'x',[-1;1],'y',[0;0],'yaw',[0;0]);
small=[-.1 -.1;.1 -.1;.1 .1;-.1 .1];
r=check_run_clearance(log,boxes,struct('Footprint',small));
assert(~any(r.collisionFlags) && r.anyCollision && r.collision && ~r.pass);
assert(isequal(size(r.minDist),[2 1]) && r.minOverall==r.minClearance);
assert(all(isfield(r.minAt,{'x','y','t','box'}))); % tunneling
log.x=[-2;-2]; r=check_run_clearance(log,boxes,struct('Footprint',small));
assert(r.pass && r.continuousClearanceLowerBound>1.8);
log=struct('t',0,'x',0,'y',0,'yaw',0);
r=check_run_clearance(log,zeros(0,7));
assert(r.pass && isinf(r.minClearance));
% Coarse samples can be inconclusive without detected collision.
log=struct('t',[0;1],'x',[-1;1],'y',[0;0],'yaw',[0;0]);
r=check_run_clearance(log,boxes,struct('Footprint',small,'MaxTranslationStep',3));
assert(~r.anyCollision && ~r.pass && strcmp(r.status,'inconclusive_between_samples'));
% Wrapped yaw is deliberately NOT normalized: actual supplied 2*pi turn.
log=struct('t',[0;1],'x',[3;3],'y',[0;0],'yaw',[0;2*pi]);
r=check_run_clearance(log,boxes);
assert(numel(r.refined.t)>=121 && r.pass);
results=struct('pass',true,'testCount',14,'runtime','MATLAB native assertions');
end
