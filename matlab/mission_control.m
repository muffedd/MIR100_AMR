function mission_control(result)
%MISSION_CONTROL Start a mission or inspect a saved run in one window.
root = fileparts(mfilename('fullpath'));
if nargin < 1, result = []; end
fig = figure('Name','MiR100 Mission Control','NumberTitle','off', ...
    'MenuBar','none','ToolBar','none','Color',[0.96 0.97 0.99], ...
    'Position',[200 120 760 440]);
uicontrol(fig,'Style','text','String','MiR100 Mission Control', ...
    'FontSize',20,'FontWeight','bold','BackgroundColor',fig.Color, ...
    'HorizontalAlignment','left','Position',[25 380 700 38]);
uicontrol(fig,'Style','text','String','Choose a destination, replay a run, and review its actual measurements.', ...
    'FontSize',11,'BackgroundColor',fig.Color,'HorizontalAlignment','left', ...
    'Position',[26 350 700 25]);
buttons = gobjects(6,1);
labels = {'New mission','Open saved run','Replay 3D','Show graphs','Compare runs','Sensor replan'};
callbacks = {@newMission,@openRun,@replay,@graphs,@compare,@sensorDemo};
for k=1:6
    buttons(k)=uicontrol(fig,'Style','pushbutton','String',labels{k}, ...
        'FontSize',10,'Position',[25+(k-1)*118 290 110 42], ...
        'Callback',callbacks{k});
end
summary = uicontrol(fig,'Style','text','Tag','MissionSummary', ...
    'String','No run selected. Start a new mission or open a saved run.', ...
    'FontSize',12,'BackgroundColor','w','HorizontalAlignment','left', ...
    'Position',[25 75 700 195]);
status = uicontrol(fig,'Style','text','String','Ready', ...
    'BackgroundColor',fig.Color,'HorizontalAlignment','left','Position',[25 35 700 25]);
refresh();

    function newMission(~,~)
        set(buttons,'Enable','off'); set(status,'String','Running mission; choose a goal on the map.'); drawnow;
        cleanup = onCleanup(@enableButtons); %#ok<NASGU>
        try
            result = run_all();
            refresh();
            if isgraphics(status), set(status,'String',['Run finished: ' result.trajectory.status]); end
        catch err
            set(status,'String',['Mission stopped: ' err.message]);
            if ~strcmp(err.identifier,'run_all:cancelled'), errordlg(err.message,'Mission stopped'); end
        end
    end

    function openRun(~,~)
        [name,folder]=uigetfile('*.mat','Choose navigation_results.mat', ...
            [fullfile(root,'demo','runs') filesep]);
        if isequal(name,0), return; end
        try
            saved=load(fullfile(folder,name),'results');
            if ~isfield(saved,'results') || ~validResult(saved.results)
                error('Choose a navigation_results.mat file produced by run_all.');
            end
            result=saved.results; refresh(); set(status,'String',['Opened: ' fullfile(folder,name)]);
        catch err
            errordlg(err.message,'Could not open run');
        end
    end

    function replay(~,~)
        if ~validResult(result), return; end
        set(buttons,'Enable','off'); set(status,'String','Playing the selected saved trajectory.'); drawnow;
        cleanup=onCleanup(@enableButtons); %#ok<NASGU>
        try
            show_navigation_3d(result);
            if isgraphics(status), set(status,'String','Replay finished.'); end
        catch err
            if isgraphics(status), set(status,'String',['Replay failed: ' err.message]); end
            errordlg(err.message,'Replay failed');
        end
    end

    function graphs(~,~)
        if ~validResult(result), return; end
        png=fullfile(result.outputDir,'navigation_graphs.png');
        if ~isfile(png), errordlg('The graph image for this run was not found.','Graphs unavailable'); return; end
        f=figure('Name','Navigation graphs','NumberTitle','off','Color','w');
        ax=axes('Parent',f); image(ax,imread(png)); axis(ax,'image','off');
    end

    function compare(~,~)
        try
            compare_navigation_runs();
        catch err
            errordlg(err.message,'Comparison unavailable');
        end
    end

    function sensorDemo(~,~)
        set(buttons,'Enable','off'); set(status,'String','Running simulated sensor and replanning; please wait.'); drawnow;
        cleanup=onCleanup(@enableButtons); %#ok<NASGU>
        try
            replanning_demo();
            if isgraphics(status), set(status,'String','Sensor demo finished. See the Command Window for its stop reason.'); end
        catch err
            if isgraphics(status), set(status,'String',['Sensor demo stopped: ' err.message]); end
            errordlg(err.message,'Sensor demo stopped');
        end
    end

    function refresh()
        if ~validResult(result), return; end
        m=result.metrics;
        set(summary,'String',{sprintf('Goal: [%.2f, %.2f] m     Stop: %s',result.goal,m.stopReason), ...
            sprintf('Goal error: %.3f m     Distance: %.2f m     Time: %.1f s', ...
            m.finalGoalErrorM,m.distanceTravelledM,m.simTimeS), ...
            sprintf('Minimum clearance: %.3f m     Collision detected: %d',m.minClearanceM,m.collision), ...
            sprintf('Core passed: %d     Overall passed: %d',result.corePass,result.pass), ...
            'Planner: A*     Controller: pure pursuit', ...
            'Known static map; kinematic simulation', ...
            ['Saved run: ' result.outputDir]});
    end

    function enableButtons()
        if isgraphics(fig), set(buttons,'Enable','on'); end
    end
end

function ok=validResult(r)
ok=isstruct(r) && isscalar(r) && all(isfield(r, ...
    {'trajectory','path','goal','metrics','outputDir','corePass','pass'}));
end
