function out = replanning_demo(out)
%REPLANNING_DEMO Run real MATLAB sensor simulation and replay its result.
% replanning_demo() runs the sensor; replanning_demo(out) replays its output.
root=fileparts(mfilename('fullpath'));
liveRun=nargin==0;
for d={'world','robot','map','planning','control','collision','reaction','demo'}
    addpath(fullfile(root,d{1}));
end
if liveRun
    fprintf('Running robotSensor obstacle detection and replanning. This may take several minutes.\n');
    out=run_real_sensor_reaction(root,struct('showLive',true));
    runs=fullfile(root,'demo','runs');
    if ~isfolder(runs), mkdir(runs); end
    folder=tempname(runs); mkdir(folder);
    save(fullfile(folder,'reaction_results.mat'),'out');
    fprintf('Sensor run saved: %s\n',folder);
end
needed={'log','pathFinal','unseenBoxes','stopReason','replanCount', ...
    'robotSensorUpdates','robotSensorFiniteReturns','goalError','collided','success','lidarSource'};
if ~isstruct(out) || ~isscalar(out) || ~all(isfield(out,needed)) || ...
        ~strcmp(out.lidarSource,'robotSensor + robotLidarPointCloudGenerator')
    error('replanning_demo:invalid','Expected an actual robotSensor result from run_real_sensor_reaction.');
end
traj=out.log;
traj.status=out.stopReason;
if isempty(traj.t) || isempty(out.pathFinal)
    error('replanning_demo:empty','The sensor run has no trajectory or path to replay.');
end
fprintf('Sensor: %s | scans: %d | finite returns: %d | replans: %d\n', ...
    out.lidarSource,out.robotSensorUpdates,out.robotSensorFiniteReturns,out.replanCount);
fprintf('Stop: %s | success: %d | goal error: %.3f m | collision: %d\n', ...
    out.stopReason,out.success,out.goalError,out.collided);
fprintf('Replay uses recorded poses; wheel angles were not logged in this sensor run.\n');
if liveRun, return; end
opts=struct('trajLog',traj,'wpsWorld',out.pathFinal,'goalWorld',out.pathFinal(end,1:2), ...
    'extraBoxes',out.unseenBoxes,'sceneOnly',true,'recordVideo',false,'saveKeyframes',false, ...
    'showAnimation',true,'dt',0.1, ...
    'sceneSubtitle',sprintf('Red obstacle: unmapped | Dashed: final plan | Blue: actual motion | Replans: %d',out.replanCount));
make_demo(fullfile(root,'assets','hard_world_boxes.csv'), ...
    fullfile(root,'assets','mir_description','urdf','mir100_matlab.urdf'),tempdir,opts);
end
