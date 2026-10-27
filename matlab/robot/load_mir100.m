function [robot, importInfo] = load_mir100(urdfPath, opts)
%LOAD_MIR100 Import the real MiR100 model (URDF + STL meshes) as a rigidBodyTree.
%
%   robot = LOAD_MIR100(urdfPath) parses the MiR100 URDF at urdfPath and
%   returns a rigidBodyTree of the actual Mobile Industrial Robots MiR100
%   platform, with its STL visuals attached. The CANONICAL URDF for this
%   project is the assets module's mir100_matlab.urdf, flattened from
%   src/mir_description/urdf/mir100_ref.urdf.xacro (the ref-matched current
%   model, NOT the classic mir.urdf.xacro) from
%   https://github.com/muffedd/ROS_mir100 at commit 6450a77: stock shell +
%   cyan band visuals on base_link, massless mirref decorative links
%   (top_holes / front_ports / corner_caps), primitive (box/cylinder)
%   collisions, 20 links / 19 joints / 10 movable. Only Gazebo-only XML
%   was removed and mesh filenames made relative (meshes/visual/...).
%
%   robot = LOAD_MIR100() uses the URDF shipped next to this file.
%
%   robot = LOAD_MIR100(urdfPath, opts) accepts a struct with fields:
%     DataFormat - "row" (default) | "column" | "struct"; use "row" so
%                  configurations are numeric vectors for follow_route and
%                  move(plat,"joint",config).
%     MeshPath   - extra mesh search root(s) (string or cellstr). Default:
%                  the folder containing the URDF, which resolves the
%                  relative meshes/... references (documented importrobot
%                  search order: MeshPath, current folder, MATLAB path, the
%                  URDF folder, one level above it).
%     Gravity    - 1x3 gravity vector. Default [0 0 0]; this is a
%                  kinematic platform model, not a dynamics simulation.
%
%   The imported tree is the full 6-wheel ref model: base_link on base_footprint,
%   2 actuated drive wheels (left_wheel_joint / right_wheel_joint,
%   continuous, axis [0 1 0]) and 4 passive caster assemblies
%   (<fl|fr|bl|br>_caster_rotation_joint about z, then
%   <fl|fr|bl|br>_caster_wheel_joint about y), plus fixed IMU, SICK S300
%   laser (front_laser_link / back_laser_link) and surface frames.
%
%   Contract: robot = load_mir100(urdfPath)  
%   Verify the result with report = verify_robot(robot) before spawning.
%
%   [robot, importInfo] = LOAD_MIR100(...) also returns the mass
%   normalization record: rawImportedMassTotal, postNormalizationMassTotal,
%   normalizedZeroMassLinks, importDefaultMassBehavior (UNVERIFIED note).
%   Source links without <inertial> are set to Mass = 0 explicitly (see
%   the normalization block at the end) so the imported mass is the
%   source-declared distribution regardless of importrobot's unverified
%   default for such links.
%
%   Requires: Robotics System Toolbox (importrobot / rigidBodyTree).
%   API verified against the MathWorks importrobot reference page.

    if nargin < 1 || isempty(urdfPath)
        urdfPath = fullfile(fileparts(mfilename('fullpath')), '..', 'assets', ...
            'mir_description', 'urdf', 'mir100_matlab.urdf');
    end
    if nargin < 2 || isempty(opts)
        opts = struct();
    end
    if ~isfield(opts, 'DataFormat'), opts.DataFormat = "row"; end
    if ~isfield(opts, 'Gravity'),    opts.Gravity    = [0 0 0]; end
    if ~isfield(opts, 'MeshPath'),   opts.MeshPath   = string(fileparts(urdfPath)); end

    if ~isfile(urdfPath)
        error('load_mir100:urdfNotFound', ...
            'URDF not found: %s. Expected matlab/assets/mir_description/urdf/mir100_matlab.urdf (canonical flat asset from ROS_mir100@6450a77).', urdfPath);
    end

    % Import the actual URDF/STL model. MeshPath is passed explicitly so the
    % relative meshes/... references resolve even when the current folder is
    % elsewhere. Meshes that cannot be found are ignored by the parser (the
    % tree still imports), so verify_robot must run afterwards.
    robot = importrobot(urdfPath, "DataFormat", opts.DataFormat, "MeshPath", opts.MeshPath);
    robot.Gravity = opts.Gravity;

    if robot.NumBodies < 1
        error('load_mir100:emptyImport', 'importrobot returned a tree with no bodies from %s.', urdfPath);
    end

    % ---- source-proven zero-mass normalization --------------------------
    % The source URDF declares NO <inertial> on 7 links: base_footprint,
    % imu_link, imu_frame, surface, mirref_top_holes, mirref_front_ports,
    % mirref_corner_caps - discovered by parsing the source at runtime,
    % not from a hardcoded list. What mass importrobot assigns such links
    % is UNVERIFIED in this environment (no MATLAB here; rigidBody's
    % documented default Mass is 1 kg, but importrobot's assignment for a
    % missing <inertial> is not documented). Intent: preserve the
    % source-declared mass distribution, so every imported body whose
    % source link has no <inertial> is set to Mass = 0 explicitly, by
    % name. Raw and post-normalization totals go to importInfo so
    % verify_robot can record them as evidence.
    [~, hasInertial] = urdf_link_masses(urdfPath);
    % The URDF root (base_footprint) imports as the tree Base, not a
    % body: never getBody() it. Normalize the Base too when its source
    % link is bare (guarded by name match).
    % rigidBodyTree.Base is a reference frame; MATLAB forbids Base.Mass.
    normalizedBase = false;
    baseExcludedFromMass = isKey(hasInertial, robot.BaseName) && ...
        ~hasInertial(robot.BaseName);
    rawMassTotal = 0;
    for b = 1:numel(robot.Bodies), rawMassTotal = rawMassTotal + robot.Bodies{b}.Mass; end
    normalizedLinks = {};
    for b = 1:numel(robot.Bodies)
        bn = robot.Bodies{b}.Name;
        if isKey(hasInertial, bn) && ~hasInertial(bn)
            robot.Bodies{b}.Mass = 0;
            normalizedLinks{end+1} = bn; %#ok<AGROW>
        end
    end
    postMassTotal = 0;
    for b = 1:numel(robot.Bodies), postMassTotal = postMassTotal + robot.Bodies{b}.Mass; end
    importInfo = struct('normalizedBase', normalizedBase, ...
        'baseExcludedFromMass', baseExcludedFromMass, ...
        'rawImportedMassTotal', rawMassTotal, ...
        'postNormalizationMassTotal', postMassTotal, ...
        'normalizedZeroMassLinks', {normalizedLinks}, ...
        'importDefaultMassBehavior', ['Raw and normalized totals measured by MATLAB ' ...
            version '; source links without inertial mass normalized to zero.']);
end
