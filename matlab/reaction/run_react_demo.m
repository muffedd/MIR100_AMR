%RUN_REACT_DEMO  Standalone demo of lidar_react on the hard_world boxes (no toolboxes needed).
%  Requires: ref/hard_world_boxes_from_sdf.csv (or project's read_world_boxes output).
here = fileparts(mfilename('fullpath'));
csvDemo = fullfile(here, '..', 'assets', 'hard_world_boxes.csv');
assert(isfile(csvDemo), 'run_react_demo:missingCSV', ...
    'Canonical wall CSV not found: %s (run from the integrated tree).', csvDemo);
boxes = read_world_boxes(csvDemo);   % Nx7, canonical 8-col id CSV
route = [0 0;0 1.12;0 2.81;.56 3.09;1.12 3.37;1.12 4.5;1.12 5.62;2.25 5.62;3.37 5.62; ...
         4.5 5.62;5.25 5.25;5.25 4.12;5.25 3.37;6.75 3.37;7.87 3.37;8.62 3.37] + [-4.31 -2.81];
opts = struct('boxes',boxes,'unseenBoxes',[-3.9 -1.9 .25 .5 .5 .5 .3]);
out = lidar_react([], [], route, opts);
fprintf('success=%d collided=%d replans=%d unseenDetected=%d simTime=%.1f s\n', ...
    out.success, out.collided, out.replanCount, out.unseenDetected, out.simTime);
cellfun(@disp, out.notes);
figure; hold on; axis equal;
for i = 1:size(boxes,1)
    b = boxes(i,:); c = cos(b(7)); s = sin(b(7));
    L = [-1 -1;1 -1;1 1;-1 1].*[b(4) b(5)]/2;
    fill(b(1)+c*L(:,1)-s*L(:,2), b(2)+s*L(:,1)+c*L(:,2), [.6 .6 .7]);
end
plot(route(:,1),route(:,2),'g--'); plot(out.log.x,out.log.y,'r');
title('lidar\_react demo (core, no MATLAB execution evidence by author)');
