function pptDir = make_ppt_figures(goal, opts)
%MAKE_PPT_FIGURES Export slide-ready PNGs (and an MP4) from ONE real run.
%   pptDir = MAKE_PPT_FIGURES() runs the core pipeline to the standard goal
%   [4.31 0.56], then writes every figure a presentation needs into
%   <run folder>/ppt/, numbered in slide order:
%     01-03  imported MiR100 model (iso / side / top)
%     04     occupancy map + planner keep-out zone + A* path
%     05-10  robot moving in the 3D world (6 moments, start -> goal)
%     11-16  same 6 moments in the 2D top-down view
%     17     2D planned vs executed route
%     18     all six navigation graphs on one page
%     19-24  single graphs: position, errors, error calibration,
%            speed, heading, clearance
%     25-26  results summary table, validation (T1-T6) table
%     mir100_demo_3d.mp4 / mir100_demo_topdown.mp4 (desktop MATLAB only)
%   Every number comes from this run; nothing is typed in by hand.
%
%   MAKE_PPT_FIGURES([x y]) uses your own goal (world metres).
%   MAKE_PPT_FIGURES(goal, opts): opts.frames (default 6), opts.dpi (200),
%   opts.video (true; MPEG-4 needs Windows or macOS MATLAB).
%
%   Run from the matlab/ folder:  pptDir = make_ppt_figures;

if nargin < 1 || isempty(goal), goal = [4.31 0.56]; end
if nargin < 2 || isempty(opts), opts = struct(); end
if ~isfield(opts, 'frames'), opts.frames = 6; end
if ~isfield(opts, 'dpi'),    opts.dpi    = 200; end
if ~isfield(opts, 'video'),  opts.video  = true; end

matlabDir = fileparts(fileparts(mfilename('fullpath')));
addpath(matlabDir);
csvPath  = fullfile(matlabDir, 'assets', 'hard_world_boxes.csv');
urdfPath = fullfile(matlabDir, 'assets', 'mir_description', 'urdf', 'mir100_matlab.urdf');

% One real run. The demo is rendered below with slide settings instead.
results = run_all(goal, struct('showAnimation', false, 'renderDemo', false));
if ~results.corePass
    error('make_ppt_figures:runFailed', ...
        'Core run did not pass (%s). No slides exported for a failed run.', ...
        results.plannerStatusGlobal);
end
outDir = results.outputDir;
pptDir = fullfile(outDir, 'ppt');
demoDir = fullfile(outDir, 'demo');
if ~isfolder(pptDir), mkdir(pptDir); end

traj = results.trajectory;
nS = numel(traj.t);
kf = unique(round(linspace(1, nS, opts.frames)));
route = results.path;
if size(route, 1) < 2, route = [results.start; results.goal]; end

demoOpts = struct('trajLog', traj, 'wpsWorld', route, ...
    'showAnimation', false, 'diagnosticOnly', false, ...
    'keyframeIdx', kf, 'keyframeRes', opts.dpi);
if opts.video, demoOpts.videoProfile = 'MPEG-4'; end
manifest = make_demo(csvPath, urdfPath, demoDir, demoOpts);

% ---- 01-03 robot model, 05-17 motion frames, 18 graph page -------------
copyAs(demoDir, 'robot_iso.png',  pptDir, '01_robot_model_iso.png');
copyAs(demoDir, 'robot_side.png', pptDir, '02_robot_model_side.png');
copyAs(demoDir, 'robot_top.png',  pptDir, '03_robot_model_top.png');
for i = 1:numel(kf)
    tag = sprintf('%d_of_%d_t%05.2fs.png', i, numel(kf), traj.t(kf(i)));
    copyAs(demoDir, sprintf('keyframe3d_%05d.png', kf(i)), pptDir, ...
        sprintf('%02d_3d_motion_%s', 4 + i, tag));
    copyAs(demoDir, sprintf('keyframe2d_%05d.png', kf(i)), pptDir, ...
        sprintf('%02d_2d_motion_%s', 10 + i, tag));
end
copyAs(demoDir, 'trajectory_overview.png', pptDir, '17_2d_route_planned_vs_executed.png');
copyAs(outDir, 'navigation_graphs.png', pptDir, '18_all_navigation_graphs.png');
for v = {'mir100_demo_3d', 'mir100_demo_topdown'}
    for ext = {'.mp4', '.avi', '.mj2'}
        copyAs(demoDir, [v{1} ext{1}], pptDir, [v{1} ext{1}]);
    end
end

% ---- 04 occupancy map ----------------------------------------------------
boxes = read_world_boxes(csvPath);
mapFigure(fullfile(pptDir, '04_occupancy_map_and_astar_path.png'), ...
    boxes, results, opts.dpi);

% ---- 19-24 single graphs from the run's numeric log -------------------
d = readtable(fullfile(outDir, 'navigation_metrics.csv'));
m = results.metrics;
t = d.time_s;

[f, ax] = graphFigure();
plot(ax, t, d.x_m, t, d.y_m, 'LineWidth', 2);
legend(ax, {'X', 'Y'}, 'Location', 'best');
finish(f, ax, 'Robot position versus time', 'Time (s)', 'Position (m)', ...
    fullfile(pptDir, '19_position_vs_time.png'), opts.dpi);

[f, ax] = graphFigure();
plot(ax, t, d.distance_to_goal_m, t, d.cross_track_error_m, 'LineWidth', 2);
legend(ax, {'Distance to goal', 'Cross-track error to planned path'}, 'Location', 'best');
finish(f, ax, sprintf('Navigation errors (final goal error %.3f m)', m.finalGoalErrorM), ...
    'Time (s)', 'Distance (m)', fullfile(pptDir, '20_navigation_errors.png'), opts.dpi);

errorCalibrationFigure(fullfile(pptDir, '21_error_calibration.png'), d, m, opts.dpi);

[f, ax] = graphFigure();
yyaxis(ax, 'left');  plot(ax, t, d.speed_m_s, 'LineWidth', 2);  ylabel(ax, 'Linear speed (m/s)');
yyaxis(ax, 'right'); plot(ax, t, d.yaw_rate_rad_s, 'LineWidth', 1.5); ylabel(ax, 'Angular speed (rad/s)');
finish(f, ax, 'Kinematic speed profile', 'Time (s)', '', ...
    fullfile(pptDir, '22_speed_profile.png'), opts.dpi);

[f, ax] = graphFigure();
plot(ax, t, rad2deg(d.yaw_rad), 'LineWidth', 2);
finish(f, ax, 'Robot heading', 'Time (s)', 'Heading (degrees)', ...
    fullfile(pptDir, '23_heading.png'), opts.dpi);

[f, ax] = graphFigure();
plot(ax, t, d.footprint_clearance_m, 'LineWidth', 2); hold(ax, 'on');
yline(ax, 0, 'r--', 'Contact', 'LineWidth', 1.5);
yline(ax, m.minClearanceM, 'k:', sprintf('Minimum %.3f m', m.minClearanceM), 'LineWidth', 1.5);
finish(f, ax, 'Full-footprint clearance to the nearest wall', 'Time (s)', 'Clearance (m)', ...
    fullfile(pptDir, '24_wall_clearance.png'), opts.dpi);

% ---- 25-26 tables --------------------------------------------------------
yesNo = {'No', 'Yes'};
rows = {
    'Start -> goal (world m)', sprintf('[%.2f, %.2f] -> [%.2f, %.2f]', results.start, results.goal)
    'Planner', sprintf('%s, %.4f m inflation', results.algorithms.planner, results.algorithms.inflationM)
    'Planned path length', sprintf('%.2f m (%d waypoints)', results.planInfo.lengthM, size(results.path, 1))
    'Controller / model', 'Pure pursuit / kinematic differential drive'
    'Distance travelled', sprintf('%.2f m', m.distanceTravelledM)
    'Simulated time', sprintf('%.2f s', m.simTimeS)
    'Final goal error', sprintf('%.4f m', m.finalGoalErrorM)
    'Max / RMS tracking error', sprintf('%.3f / %.3f m', m.maxTrackingErrorM, m.rmsTrackingErrorM)
    'Min wall clearance', sprintf('%.4f m', m.minClearanceM)
    'Collision', yesNo{1 + logical(m.collision)}
    'Stop reason', m.stopReason};
tableFigure(fullfile(pptDir, '25_results_summary.png'), ...
    'Navigation results (kinematic simulation)', rows, opts.dpi);

rv = results.tests;
vrows = cell(numel(rv), 3);
for i = 1:numel(rv)
    vrows(i, :) = {sprintf('%s  %s', rv(i).id, rv(i).name), rv(i).status, ...
        shorten(rv(i).details, 95)};
end
tableFigure(fullfile(pptDir, '26_validation_tests.png'), ...
    'Validation battery (independent reference goal [4.31 0.56])', vrows, opts.dpi);

writeIndex(pptDir, results, manifest);
fprintf('\nPPT figures ready: %s\n', pptDir);
if ispc, winopen(pptDir); end
end

% ======================= local functions =================================

function copyAs(srcDir, srcName, dstDir, dstName)
src = fullfile(srcDir, srcName);
if isfile(src), copyfile(src, fullfile(dstDir, dstName)); end
end

function [f, ax] = graphFigure()
f = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1280 720]);
ax = axes(f); hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
end

function finish(f, ax, ttl, xl, yl, file, dpi)
title(ax, ttl, 'FontSize', 18, 'FontWeight', 'bold', 'Color', 'k');
xlabel(ax, xl, 'FontSize', 15);
if ~isempty(yl), ylabel(ax, yl, 'FontSize', 15); end
set(ax, 'FontSize', 13, 'Color', 'w', 'XColor', 'k', 'GridColor', [.75 .75 .75]);
lg = findobj(f, 'Type', 'legend');
if ~isempty(lg), set(lg, 'FontSize', 13, 'TextColor', 'k', 'Color', 'w'); end
exportgraphics(f, file, 'Resolution', dpi);
close(f);
end

function errorCalibrationFigure(file, d, m, dpi)
% How large the tracking error is, how often, and how the goal error converges.
f = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1280 720]);
tl = tiledlayout(f, 1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');
ax = nexttile(tl); hold(ax, 'on'); grid(ax, 'on');
e = d.cross_track_error_m * 1000;
histogram(ax, e, 40, 'FaceColor', [0.2 0.45 0.85]);
xline(ax, m.rmsTrackingErrorM*1000, 'k--', sprintf('RMS %.1f mm', m.rmsTrackingErrorM*1000), 'LineWidth', 1.5);
xline(ax, m.maxTrackingErrorM*1000, 'r--', sprintf('Max %.1f mm', m.maxTrackingErrorM*1000), 'LineWidth', 1.5);
xline(ax, 200, 'Color', [0 .5 0], 'Label', 'T3 limit 200 mm', 'LineWidth', 1.5);
xlabel(ax, 'Cross-track error (mm)'); ylabel(ax, 'Samples');
title(ax, 'Tracking error distribution');
ax2 = nexttile(tl); grid(ax2, 'on');
semilogy(ax2, d.time_s, max(d.distance_to_goal_m, 1e-3), 'LineWidth', 2);
grid(ax2, 'on'); hold(ax2, 'on');
yline(ax2, m.finalGoalErrorM, 'k--', sprintf('Final %.3f m', m.finalGoalErrorM), 'LineWidth', 1.5);
xlabel(ax2, 'Time (s)'); ylabel(ax2, 'Distance to goal (m, log)');
title(ax2, 'Goal error convergence');
title(tl, 'Error calibration', 'FontSize', 18, 'FontWeight', 'bold');
set([ax ax2], 'FontSize', 13, 'Color', 'w');
exportgraphics(f, file, 'Resolution', dpi);
close(f);
end

function mapFigure(file, boxes, results, dpi)
% The grid the planner searches: walls, keep-out band, then the A* path.
res = 0.05;
map = build_map(boxes, res, []);
if isstruct(map)
    occ = logical(map.Grid); cpm = map.Resolution; org = map.Origin(:).';
else
    occ = checkOccupancy(map) ~= 0; cpm = map.Resolution;
    org = [map.XWorldLimits(1) map.YWorldLimits(1)];
end
inflateM = 0.6052;
rEff = inflateM + (sqrt(2) + 0.05)/cpm;   % same raster pad as plan_path
rc = ceil(rEff*cpm);
[dx, dy] = meshgrid(-rc:rc);
keepOut = conv2(double(occ), double(dx.^2 + dy.^2 <= (rEff*cpm)^2), 'same') > 0;
[ny, nx] = size(occ);
img = ones(ny, nx, 3);
keepCol = [1 0.85 0.6]; wallCol = [0.35 0.37 0.42];
for c = 1:3
    ch = img(:,:,c);
    ch(keepOut) = keepCol(c);
    ch(occ) = wallCol(c);
    img(:,:,c) = ch;
end
f = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1280 900]);
ax = axes(f); hold(ax, 'on');
image(ax, org(1) + ([1 nx] - 0.5)/cpm, org(2) + ([1 ny] - 0.5)/cpm, flipud(img));
set(ax, 'YDir', 'normal'); axis(ax, 'equal', 'tight');
hp = plot(ax, results.path(:,1), results.path(:,2), 'b-', 'LineWidth', 2.2);
hs = plot(ax, results.start(1), results.start(2), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 11);
hg = plot(ax, results.goal(1), results.goal(2), 'rp', 'MarkerFaceColor', 'r', 'MarkerSize', 16);
hw = patch(ax, nan, nan, [0.35 0.37 0.42]);
hk = patch(ax, nan, nan, [1 0.85 0.6], 'EdgeColor', 'none');
legend(ax, [hw hk hp hs hg], {'Walls (occupied cells)', ...
    sprintf('Keep-out: %.4f m robot radius + raster pad', inflateM), ...
    'A* path', 'Start', 'Goal'}, 'Location', 'southoutside', 'NumColumns', 3, 'FontSize', 12);
xlabel(ax, 'World X (m)'); ylabel(ax, 'World Y (m)');
title(ax, sprintf('Occupancy grid (%.0f mm cells) and A* path', 1000/cpm), 'FontSize', 18);
set(ax, 'FontSize', 13);
exportgraphics(f, file, 'Resolution', dpi);
close(f);
end

function tableFigure(file, ttl, rows, dpi)
n = size(rows, 1); nc = size(rows, 2);
f = figure('Visible', 'off', 'Color', 'w', 'Position', [100 100 1500 140 + 46*n]);
ax = axes(f, 'Position', [0 0 1 1], 'Visible', 'off');
xlim(ax, [0 1]); ylim(ax, [0 n + 1.6]); hold(ax, 'on');
text(ax, 0.03, n + 0.95, ttl, 'FontSize', 20, 'FontWeight', 'bold', 'Interpreter', 'none');
if nc == 2, xs = [0.03 0.42]; else, xs = [0.03 0.36 0.44]; end
for i = 1:n
    y = n - i + 0.5;
    if mod(i, 2) == 1
        patch(ax, [0.02 0.98 0.98 0.02], y + [-0.5 -0.5 0.5 0.5], [0.95 0.96 0.98], 'EdgeColor', 'none');
    end
    for c = 1:nc
        s = rows{i, c}; col = 'k'; fw = 'normal'; fs = 15;
        if c == 2, fw = 'bold'; end
        if nc == 3 && c == 2
            if strcmp(s, 'PASS'), col = [0 0.5 0]; else, col = [0.8 0 0]; end
        end
        if nc == 3 && c == 3, fs = 12; end
        text(ax, xs(c), y, s, 'FontSize', fs, 'FontWeight', fw, 'Color', col, 'Interpreter', 'none');
    end
end
exportgraphics(f, file, 'Resolution', dpi);
close(f);
end

function s = shorten(s, n)
s = char(s);
if numel(s) > n, s = [s(1:n-3) '...']; end
end

function writeIndex(pptDir, results, manifest)
fid = fopen(fullfile(pptDir, 'INDEX.txt'), 'w');
if fid < 0, return; end
c = onCleanup(@() fclose(fid));
fprintf(fid, 'MiR100 AMR slide figures, one real run\n');
fprintf(fid, 'run folder: %s\n', results.outputDir);
fprintf(fid, 'goal [%.3f %.3f], corePass=%d, validationPass=%d\n', ...
    results.goal, results.corePass, results.pass);
fprintf(fid, '3D video: %s\n', manifest.sceneVideo);
fprintf(fid, 'Kinematic simulation only: not physical validation.\n\n');
d = dir(fullfile(pptDir, '*.*'));
for i = 1:numel(d)
    if ~d(i).isdir, fprintf(fid, '%s\n', d(i).name); end
end
end
