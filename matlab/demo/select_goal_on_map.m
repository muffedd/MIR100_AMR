function goal = select_goal_on_map(csvPath, startXY, defaultGoal, inflate, message)
%SELECT_GOAL_ON_MAP Let a desktop MATLAB user click a world-frame goal.
%   inflate (m, optional) shades each wall's robot safety zone and rejects
%   clicks inside it immediately; the planner remains the final check.
%   message (optional) is shown above the map, e.g. why the last goal failed.
if nargin < 4 || isempty(inflate), inflate = 0; end
if nargin < 5, message = ''; end
boxes = read_world_boxes(csvPath);
fig = figure('Name','MiR100: click a destination','NumberTitle','off', ...
    'Color','w');
cleanup = onCleanup(@() closeIfOpen(fig)); %#ok<NASGU>
ax = axes(fig);
hold(ax,'on');
zoneHandle = gobjects(0);
if inflate > 0
    for k = 1:size(boxes,1)
        c = boxCorners(boxes(k,:), inflate);
        h = patch(ax,c(:,1),c(:,2),[1 0.82 0.80],'EdgeColor','none', ...
            'FaceAlpha',0.7,'HandleVisibility','off','HitTest','off');
        if k == 1
            h.HandleVisibility = 'on'; h.DisplayName = 'Too close to wall';
            zoneHandle = h;
        end
    end
end
wallHandle = gobjects(1);
for k = 1:size(boxes,1)
    corners = boxCorners(boxes(k,:), 0);
    if k == 1
        wallHandle = patch(ax,corners(:,1),corners(:,2),[0.48 0.51 0.58], ...
            'EdgeColor',[0.20 0.22 0.26],'DisplayName','Walls');
    else
        patch(ax,corners(:,1),corners(:,2),[0.48 0.51 0.58], ...
            'EdgeColor',[0.20 0.22 0.26],'HandleVisibility','off');
    end
end
startHandle = plot(ax,startXY(1),startXY(2),'go','MarkerFaceColor',[0.1 0.75 0.2], ...
    'MarkerSize',9,'DisplayName','Robot start');
goalHandle = plot(ax,defaultGoal(1),defaultGoal(2),'r^','MarkerFaceColor',[0.9 0.2 0.15], ...
    'MarkerSize',9,'DisplayName','Default goal');
allX = [boxes(:,1);startXY(1);defaultGoal(1)];
allY = [boxes(:,2);startXY(2);defaultGoal(2)];
pad = max(1,0.06*max(max(allX)-min(allX),max(allY)-min(allY)));
xlim(ax,[min(allX)-pad,max(allX)+pad]);
ylim(ax,[min(allY)-pad,max(allY)+pad]);
axis(ax,'equal'); grid(ax,'on'); box(ax,'on');
xlabel(ax,'World X (m)'); ylabel(ax,'World Y (m)');
baseTitle = {'Click a destination in the white area', ...
    'Close the window or press Enter to cancel'};
if ~isempty(message), baseTitle = [{message} baseTitle]; end
title(ax,baseTitle,'Interpreter','none');
legend(ax,[wallHandle zoneHandle startHandle goalHandle],'Location','best');
goal = [];
fig.WindowButtonDownFcn = @choosePoint;
fig.WindowKeyPressFcn = @cancelKey;
drawnow;
uiwait(fig);
if isempty(goal)
    error('run_all:cancelled','Goal selection cancelled. No motion commanded.');
end
closeIfOpen(fig);

    function choosePoint(~,~)
        if ~strcmp(fig.SelectionType,'normal'), return; end
        if ~isequal(ancestor(hittest(fig),'axes'),ax), return; end
        point = ax.CurrentPoint;
        xy = point(1,1:2);
        if ~(all(isfinite(xy)) && xy(1)>=ax.XLim(1) && xy(1)<=ax.XLim(2) && ...
                xy(2)>=ax.YLim(1) && xy(2)<=ax.YLim(2))
            return;
        end
        if inflate > 0 && insideAnyZone(xy)
            title(ax,[{sprintf('[%.2f %.2f] is too close to a wall - click in the white area', xy)} ...
                baseTitle(end-1:end)],'Color',[0.75 0.1 0.1],'Interpreter','none');
            return;
        end
        goal = xy;
        uiresume(fig);
    end

    function hit = insideAnyZone(xy)
        hit = false;
        for i = 1:size(boxes,1)
            c = boxCorners(boxes(i,:), inflate);
            if inpolygon(xy(1),xy(2),c(:,1),c(:,2)), hit = true; return; end
        end
    end

    function cancelKey(~,event)
        if ismember(event.Key,{'return','escape'}), uiresume(fig); end
    end
end

function corners = boxCorners(b, grow)
corners = [-1 -1; 1 -1; 1 1; -1 1] .* (b(4:5)/2 + grow);
R = [cos(b(7)) -sin(b(7)); sin(b(7)) cos(b(7))];
corners = corners*R.' + b(1:2);
end

function closeIfOpen(fig)
if isgraphics(fig), close(fig); end
end
