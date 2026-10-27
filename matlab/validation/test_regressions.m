function result = test_regressions()
% Focused MATLAB regression tests for the execution failures.
root=fileparts(fileparts(mfilename('fullpath')));addpath(genpath(root));
urdf=fullfile(root,'assets','mir_description','urdf','mir100_matlab.urdf');
[mass,hasInertial,names]=urdf_link_masses(urdf);
assert(numel(names)==20 && nnz(~cell2mat(values(hasInertial)))==7);
assert(abs(sum(cell2mat(values(mass)),'omitnan')-69.2390156076)<1e-9);
[robot,info]=load_mir100(urdf);
report=verify_robot(robot,struct('UrdfPath',urdf,'ImportInfo',info));
assert(isscalar(report) && report.pass);
check=report.checks(strcmp({report.checks.name},'movableJoints'));
assert(isscalar(check) && iscell(check.value) && numel(check.value)==10);
assert(info.baseExcludedFromMass && ~info.normalizedBase);
boxes=read_world_boxes();assert(isequal(size(boxes),[65 7]));
map=build_map(boxes,.05,[]);
sm=struct('Grid',occupancyMatrix(map)>.5,'Resolution',map.Resolution,'Origin',map.GridLocationInWorld);
path=plan_path(map,[-4.31 -2.81],[4.31 .56],.6052);
assert(~isempty(path) && isequal(path,plan_path(sm,[-4.31 -2.81],[4.31 .56],.6052)));
scen=robotScenario(UpdateRate=100,StopTime=10);
plat=spawn_mir100(scen,robot);setup(scen);
traj=follow_route(plat,[-4.31 -2.81;-4.21 -2.81;-4.01 -2.81], ...
    struct('scenario',scen,'maxSimTime',5));
assert(strcmp(traj.status,'success') && traj.t(1)==0 && traj.wheelsCommanded);
assert(max(abs(diff(traj.yaw)))<=.021);
joints=read(plat,'joint');assert(any(abs(joints)>0));
collision=test_collision();assert(collision.pass);
result=struct('pass',true,'matlabVersion',version,'robotMass',report.massTotal, ...
    'linkCount',numel(names),'wallCount',size(boxes,1),'pathPoints',size(path,1));
disp(result);
end
