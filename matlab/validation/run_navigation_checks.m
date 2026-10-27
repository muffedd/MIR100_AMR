function reports = run_navigation_checks(outDir)
% Native MATLAB robotSensor tests; no ROS publishers or hardware access.
root = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(root));
if nargin<1, outDir=fullfile(root,'validation','navigation_results'); end
if ~isfolder(outDir),mkdir(outDir);end
robot=load_mir100(); verify_robot(robot);
known=[-2 0 1 .1 4.2 2 0;6 0 1 .1 4.2 2 0;2 -2 1 8 .1 2 0;2 2 1 8 .1 2 0];
cases={zeros(0,7),[2 0 .5 .5 .5 1 0],[2 0 1 .5 4 2 0]};
labels={'clear','detour','blocked'};
reports=struct([]);
for ci=1:3
    fprintf('Starting real robotSensor case: %s\n',labels{ci});
    scen=robotScenario(UpdateRate=100,StopTime=80);
    allBoxes=[known;cases{ci}];
    for bi=1:size(allBoxes,1)
        b=allBoxes(bi,:);
        addMesh(scen,"Box",Position=b(1:3),Size=b(4:6),Collision="mesh",IsBinaryOccupied=true);
    end
    plat=spawn_mir100(scen,robot,struct('Position',[0 0 0]));
    sensor=robot_lidar_sensor('attach',plat,struct('robot',robot));
    setup(scen);
    o=struct('boxes',known,'unseenBoxes',cases{ci},'startPose',[0 0 0], ...
        'movePlatform',true,'lidarMode','robotSensor','robotLidar',sensor,'maxTime',60);
    out=lidar_react(plat,scen,[0 0;4 0],o);
    clearance=check_run_clearance(out.log,allBoxes);
    save(fullfile(outDir,[labels{ci} '.mat']),'out','clearance');
    expected=(ci<3 && out.success && clearance.pass) || ...
        (ci==3 && ~out.success && ~out.goalReached && clearance.pass);
    entry=struct('name',labels{ci},'pass',expected,'success',out.success, ...
        'goalError',out.goalError,'collision',clearance.collision,'clearance',clearance.minOverall, ...
        'continuousClearance',clearance.continuousClearanceLowerBound, ...
        'replans',out.replanCount,'stopReason',out.stopReason, ...
        'sensorUpdates',out.robotSensorUpdates,'finiteReturns',out.robotSensorFiniteReturns, ...
        'unseenDetected',out.unseenDetected);
    if isempty(reports),reports=entry;else,reports(ci)=entry;end
    disp(reports(ci));
    save(fullfile(outDir,[labels{ci} '.mat']),'out','clearance');
end
save(fullfile(outDir,'summary.mat'),'reports');
end
