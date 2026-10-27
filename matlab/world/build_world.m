function [scen, boxes, wallNames] = build_world(csvPath, opts)
%BUILD_WORLD Build robotScenario with the 65 static walls of hard_world.sdf.
%   [scen, boxes, wallNames] = build_world(csv)
%   Requires Robotics System Toolbox / UAV-free robotScenario (R2022a+ for addMesh).
%   Walls: static "Box" meshes with Collision="mesh" and IsBinaryOccupied=true.
%   Ground: 30x30 "Plane" mesh named "ground", NOT occupied (excluded by identity).
%   Yaw: addMesh "Box" has no orientation input (per MathWorks addMesh doc), so
%   any wall with yaw ~= 0 is built as a "Custom" mesh from rotated world-frame
%   vertices. All 65 SDF walls have yaw = 0; the custom branch is unexercised.
    if nargin < 1, csvPath = []; end
    if nargin < 2, opts = struct(); end
    if ~isfield(opts,'UpdateRate'), opts.UpdateRate = 100; end  % dt=0.01 s; matches the certified reference trajectory
    if ~isfield(opts,'StopTime'),   opts.StopTime   = 600; end  % >= follower maxSimTime
    [boxes, wallNames] = read_world_boxes(csvPath);

    scen = robotScenario(UpdateRate=opts.UpdateRate, StopTime=opts.StopTime);
    addMesh(scen, "Plane", Position=[0 0 0], Size=[30 30], ...
        Color=[0.6 0.6 0.6], Name="ground", IsBinaryOccupied=false);

    col = [0.45 0.48 0.55];
    for i = 1:size(boxes,1)
        b = boxes(i,:);
        if abs(b(7)) < 1e-9
            addMesh(scen, "Box", Position=b(1:3), Size=b(4:6), Color=col, ...
                Collision="mesh", IsBinaryOccupied=true, Name=wallNames(i));
        else
            [v, f] = rotatedBoxMesh(b);
            addMesh(scen, "Custom", Vertices=v, Faces=f, Position=[0 0 0], ...
                Color=col, Collision="mesh", IsBinaryOccupied=true, Name=wallNames(i));
        end
    end
end

function [v, f] = rotatedBoxMesh(b)
    h = b(4:6)/2;
    c = [-1 -1 -1; 1 -1 -1; 1 1 -1; -1 1 -1; -1 -1 1; 1 -1 1; 1 1 1; -1 1 1] .* h;
    R = [cos(b(7)) -sin(b(7)); sin(b(7)) cos(b(7))];
    v = [c(:,1:2)*R.' + b(1:2), c(:,3) + b(3)];
    f = [1 3 2; 1 4 3; 5 6 7; 5 7 8; 1 2 6; 1 6 5; 2 3 7; 2 7 6; 3 4 8; 3 8 7; 4 1 5; 4 5 8];
end
