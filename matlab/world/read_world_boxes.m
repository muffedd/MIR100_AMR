function [boxes, names] = read_world_boxes(csvPath)
%READ_WORLD_BOXES Read wall boxes from CSV (hard_world.sdf, 65 walls).
%   boxes = Nx7 [xc yc zc sx sy sz yaw]; metres, radians (yaw about +Z).
%   CSV columns: id|name,xc,yc,zc,sx,sy,sz,yaw (header row required).
%   names = Nx1 string (SDF model names w0..w64). Ground plane is NOT in CSV.
    if nargin < 1 || isempty(csvPath)
        csvPath = fullfile(fileparts(mfilename('fullpath')), '..', 'assets', 'hard_world_boxes.csv');
    end
    assert(isfile(csvPath), 'read_world_boxes:missing', 'CSV not found: %s', csvPath);
    T = readtable(csvPath, 'TextType', 'string');
    vn = string(T.Properties.VariableNames);
    req = ["xc","yc","zc","sx","sy","sz","yaw"];
    assert(all(ismember(req, vn)), ...
        'read_world_boxes:columns', 'CSV needs columns: id|name,%s', strjoin(req, ','));
    if ismember("id", vn), names = T.id;
    elseif ismember("name", vn), names = T.name;
    else, error('read_world_boxes:columns', 'CSV needs an id or name column.');
    end
    boxes = [T.xc T.yc T.zc T.sx T.sy T.sz T.yaw];
    assert(all(isfinite(boxes(:))) && all(all(boxes(:,4:6) > 0)), ...
        'read_world_boxes:values', 'Non-finite value or non-positive size in CSV.');
end
