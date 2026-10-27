function [metrics, algorithms] = navigation_diagnostics(traj,path,goal,clearance,boxes,outDir,visible)
% Actual trajectory diagnostics. Goal distance is distinct from path error.
t=traj.t(:);xy=[traj.x(:),traj.y(:)];yaw=unwrap(traj.yaw(:));
goalError=vecnorm(xy-goal(:)',2,2);
tracking=NaN(size(t));
if ~isempty(path)
    tracking=inf(size(t));
    if size(path,1)==1,tracking=vecnorm(xy-path,2,2);end
    for k=1:size(path,1)-1
        a=path(k,:);d=path(k+1,:)-a;
        u=max(0,min(1,((xy-a)*d')/max(dot(d,d),eps)));
        tracking=min(tracking,vecnorm(xy-(a+u.*d),2,2));
    end
end
speed=zeros(size(t));omega=speed;
if numel(t)>1
    speed(2:end)=vecnorm(diff(xy,1,1),2,2)./diff(t);
    omega(2:end)=diff(yaw)./diff(t);
end
algorithms=struct('planner','8-connected A*','controller','Pure pursuit', ...
    'motionModel','Kinematic differential drive','inflationM',.6052, ...
    'sensing','Known static wall map; core run has no live lidar or reactive replanning');
metrics=struct('finalGoalErrorM',goalError(end),'maxTrackingErrorM',max(tracking), ...
    'rmsTrackingErrorM',sqrt(mean(tracking.^2)), ...
    'distanceTravelledM',sum(vecnorm(diff(xy,1,1),2,2)), ...
    'simTimeS',t(end)-t(1),'minClearanceM',clearance.minOverall, ...
    'continuousClearanceLowerBoundM',clearance.continuousClearanceLowerBound, ...
    'collision',clearance.collision,'stopReason',traj.status);
data=table(t,xy(:,1),xy(:,2),yaw,speed,omega,goalError,tracking,clearance.minDist(:), ...
    'VariableNames',{'time_s','x_m','y_m','yaw_rad','speed_m_s','yaw_rate_rad_s', ...
    'distance_to_goal_m','cross_track_error_m','footprint_clearance_m'});
writetable(data,fullfile(outDir,'navigation_metrics.csv'));
visibility='off';if visible,visibility='on';end
fig=figure('Name','MiR100 navigation graphs','NumberTitle','off', ...
    'Visible',visibility,'Color','w','Position',[40 40 1350 850]);
layout=tiledlayout(fig,3,2,'TileSpacing','compact','Padding','compact');
ax=nexttile(layout);hold(ax,'on');axis(ax,'equal');grid(ax,'on');
for k=1:size(boxes,1)
    b=boxes(k,:);c=cos(b(7));s=sin(b(7));
    corners=[-1 -1;1 -1;1 1;-1 1].*b(4:5)/2;
    corners=corners*[c s;-s c]+b(1:2);
    patch(ax,corners(:,1),corners(:,2),[.8 .8 .82],'EdgeColor',[.6 .6 .6]);
end
if isempty(path),p=[NaN NaN];else,p=path;end
h1=plot(ax,p(:,1),p(:,2),'k--','LineWidth',1.3);
h2=plot(ax,xy(:,1),xy(:,2),'b-','LineWidth',1.5);
h3=plot(ax,xy(1,1),xy(1,2),'go','MarkerFaceColor','g');
h4=plot(ax,goal(1),goal(2),'rp','MarkerFaceColor','r','MarkerSize',11);
legend(ax,[h1 h2 h3 h4],{'Fresh A* plan','Executed motion','Start','Requested goal'},'Location','best');
xlabel(ax,'World X (m)');ylabel(ax,'World Y (m)');title(ax,'Planned and executed route');
ax=nexttile(layout);plot(ax,t,xy,'LineWidth',1.2);grid(ax,'on');
xlabel(ax,'Time (s)');ylabel(ax,'Position (m)');legend(ax,{'X','Y'});title(ax,'Movement versus time');
ax=nexttile(layout);plot(ax,t,[goalError tracking],'LineWidth',1.2);grid(ax,'on');
xlabel(ax,'Time (s)');ylabel(ax,'Distance (m)');
legend(ax,{'Distance to goal','Cross-track error to planned polyline'});title(ax,'Navigation errors');
ax=nexttile(layout);yyaxis(ax,'left');plot(ax,t,speed);ylabel(ax,'Linear speed (m/s)');
yyaxis(ax,'right');plot(ax,t,omega);ylabel(ax,'Angular speed (rad/s)');
xlabel(ax,'Time (s)');grid(ax,'on');title(ax,'Kinematic velocity estimates');
ax=nexttile(layout);plot(ax,t,rad2deg(yaw),'LineWidth',1.2);grid(ax,'on');
xlabel(ax,'Time (s)');ylabel(ax,'Continuous heading (degrees)');title(ax,'Robot orientation');
ax=nexttile(layout);plot(ax,t,clearance.minDist,'LineWidth',1.2);hold(ax,'on');
if ~isempty(clearance.intervalClearanceLowerBound)
    plot(ax,clearance.refined.t(1:end-1),clearance.intervalClearanceLowerBound,':');
    legend(ax,{'Footprint clearance','Between-sample lower bound'},'Location','best');
end
yline(ax,0,'r--','Contact','HandleVisibility','off');grid(ax,'on');xlabel(ax,'Time (s)');ylabel(ax,'Clearance (m)');
title(ax,'Full-footprint collision clearance');
title(layout,{sprintf('Goal [%.3f, %.3f] m | Stop: %s | Goal error %.4f m',goal,traj.status,goalError(end)), ...
    'Known wall map -> 0.6052 m inflation -> A* -> pure pursuit -> differential-drive kinematics'},'Color','k');
% Export remains readable when MATLAB's desktop uses a dark theme.
set(findall(fig,'Type','axes'),'Color','w','XColor','k','YColor','k','GridColor',[.7 .7 .7]);
set(findall(fig,'Type','text'),'Color','k');
set(findall(fig,'Type','legend'),'Color','w','TextColor','k','EdgeColor',[.3 .3 .3]);
exportgraphics(fig,fullfile(outDir,'navigation_graphs.png'),'Resolution',140);
if ~visible,close(fig);end
end
