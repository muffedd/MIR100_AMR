function manifest = show_navigation_3d(source)
%SHOW_NAVIGATION_3D Replay a saved trajectory in the original 3D world.
%   show_navigation_3d opens a file picker for navigation_results.mat.
%   show_navigation_3d(results) uses the results already in your workspace.
%   No planning, controller run, validation battery or recording is started.
rootDir = fileparts(mfilename('fullpath'));
for d = {'world','robot','control','collision','demo'}
    addpath(fullfile(rootDir,d{1}));
end
if nargin < 1 || isempty(source)
    [file,folder] = uigetfile({'*.mat','Saved navigation results (*.mat)'}, ...
        'Choose navigation_results.mat from a completed run', ...
        [fullfile(rootDir,'demo','runs') filesep]);
    if isequal(file,0), manifest=struct('status','cancelled'); return; end
    source = fullfile(folder,file);
end
if ischar(source) || isstring(source)
    saved = load(source,'results');
    if ~isfield(saved,'results')
        error('show_navigation_3d:badFile','Choose a navigation_results.mat file written by run_all.');
    end
    result = saved.results;
    fprintf('Replaying saved run: %s\n',char(source));
else
    result = source;
end
if ~isstruct(result) || ~isscalar(result) || ...
        ~all(isfield(result,{'trajectory','path','goal'}))
    error('show_navigation_3d:badResults','Input must be the results returned by run_all.');
end
traj = result.trajectory;
if ~all(isfield(traj,{'t','x','y','yaw'})) || isempty(traj.t)
    error('show_navigation_3d:noTrajectory','This run contains no trajectory to replay.');
end
dt = 0.01;
if numel(traj.t)>1, dt = median(diff(traj.t)); end
opts = struct('trajLog',traj,'wpsWorld',result.path,'goalWorld',result.goal, ...
    'sceneOnly',true,'recordVideo',false,'saveKeyframes',false, ...
    'showAnimation',true,'dt',dt);
fprintf('3D replay: requested goal [%.3f %.3f] m; stop=%s.\n', ...
    result.goal,traj.status);
manifest = make_demo(fullfile(rootDir,'assets','hard_world_boxes.csv'), ...
    fullfile(rootDir,'assets','mir_description','urdf','mir100_matlab.urdf'), ...
    tempdir,opts);
end
