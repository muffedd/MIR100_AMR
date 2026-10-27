function goal = select_goal_on_map(csvPath, startXY, defaultGoal)
%SELECT_GOAL_ON_MAP Let a desktop MATLAB user click a world-frame goal.
boxes = read_world_boxes(csvPath);
fig = figure('Name','MiR100: click a destination','NumberTitle','off', ...
    'Color','w');
cleanup = onCleanup(@() closeIfOpen(fig)); %#ok<NASGU>
ax = axes(fig);
hold(ax,'on');
wallHandle = gobjects(1);
for k = 1:size(boxes,1)
    b = boxes(k,:);
    corners = [-1 -1; 1 -1; 1 1; -1 1] .* b(4:5)/2;
    R = [cos(b(7)) -sin(b(7)); sin(b(7)) cos(b(7))];
    corners = corners*R.' + b(1:2);
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
title(ax,{'Click the destination on the map','Close the window or press Enter to cancel; blocked goals stop safely'});
legend(ax,[wallHandle startHandle goalHandle],'Location','best');
drawnow;
[x,y,button] = ginput(1);
if isempty(x) || isempty(y) || isempty(button) || button == 27 || ...
        ~isfinite(x) || ~isfinite(y)
    error('run_all:cancelled','Goal selection cancelled. No motion commanded.');
end
goal = [x y];
closeIfOpen(fig);
end

function closeIfOpen(fig)
if isgraphics(fig), close(fig); end
end
