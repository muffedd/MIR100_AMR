function fig = compare_navigation_runs(first,second)
%COMPARE_NAVIGATION_RUNS Compare two explicitly selected saved missions.
root=fileparts(mfilename('fullpath'));
if nargin==0
    [name,folder]=uigetfile('*.mat','Choose the first navigation_results.mat', ...
        [fullfile(root,'demo','runs') filesep]);
    if isequal(name,0), fig=[]; return; end
    first=fullfile(folder,name);
    [name,folder]=uigetfile('*.mat','Choose the second navigation_results.mat', ...
        [fullfile(root,'demo','runs') filesep]);
    if isequal(name,0), fig=[]; return; end
    second=fullfile(folder,name);
elseif nargin~=2
    error('compare_navigation_runs:inputs','Provide two results or select two saved runs.');
end
runs={readRun(first),readRun(second)};
data=cell(2,7);
for i=1:2
    r=runs{i}; m=r.metrics;
    data(i,:)={sprintf('[%.2f, %.2f]',r.goal),char(m.stopReason), ...
        m.finalGoalErrorM,m.minClearanceM,logical(m.collision),m.simTimeS,logical(r.corePass)};
end
fig=figure('Name','Compare navigation runs','NumberTitle','off','MenuBar','none', ...
    'ToolBar','none','Color','w','Position',[170 180 980 260]);
uicontrol(fig,'Style','text','String','Measured mission results (kinematic simulation)', ...
    'FontSize',13,'FontWeight','bold','BackgroundColor','w','Position',[15 210 950 30]);
uitable(fig,'Tag','RunComparison','Data',data,'RowName',{'Run 1','Run 2'}, ...
    'ColumnName',{'Goal (m)','Stop reason','Goal error (m)','Clearance (m)', ...
    'Collision','Sim time (s)','Core passed'}, ...
    'Units','normalized','Position',[.02 .15 .96 .64]);
end

function r=readRun(value)
if ischar(value) || (isstring(value) && isscalar(value))
    loaded=load(value,'results');
    if ~isfield(loaded,'results'), error('Select a navigation_results.mat file from run_all.'); end
    value=loaded.results;
end
if ~isstruct(value) || ~isscalar(value) || ~all(isfield(value,{'goal','metrics','corePass'}))
    error('A selected run is missing its navigation results.');
end
r=value;
end
