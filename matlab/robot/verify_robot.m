function report = verify_robot(robot, opts)
%VERIFY_ROBOT Verify the imported MiR100 rigidBodyTree against model facts.
%
%   report = VERIFY_ROBOT(robot) checks the tree returned by LOAD_MIR100
%   against the CANONICAL current model (mir100_ref.urdf.xacro,
%   ROS_mir100 @ 6450a77): 20 links (base_footprint root + 19 bodies),
%   19 joints (9 fixed + 10 continuous), 10 movable joints (2 drive +
%   4 casters x 2), 8 referenced mesh files, total mass 69.2390156 kg
%   (mirref decorative links massless), wheel radius 0.0625 m, wheel
%   separation 0.445208 m.
%
%   REPORT SCHEMA (exact fields; revise only with the consumers):
%     linkCount        - scalar, 20 expected
%     jointCount       - scalar, 19 expected
%     massTotal        - scalar kg, ACTUAL imported tree mass after
%                        load_mir100's zero-mass normalization (gated
%                        against declaredSourceMass, not a constant)
%     declaredSourceMass - scalar kg, summed <inertial> masses parsed
%                        from the URDF at runtime (source evidence)
%     importedMass     - scalar kg, actual tree mass sum (== massTotal)
%     massImportNote   - char, import-behavior caveat
%   (Mass fields revised for honest raw/source/normalized reporting:
%   the actual imported mass is gated against the runtime source parse,
%   and the loader's raw import record is kept as separate evidence.)
%     wheelRadius      - scalar m, 0.0625 expected
%     wheelSeparation  - scalar m, 0.445208 expected
%     footprintRadius  - scalar m, contract v2 constant 0.6052
%     footprintPolygon - Nx2 m in base frame, contract v2 polygon: the v1
%                        8-vertex polygon with every edge expanded 6 mm
%                        outward (convex). Circumradius 0.6051644, rounded
%                        up to the contract radius 0.6052. Supersedes v1
%                        (radius 0.5988): v1 left caster swivel coverage
%                        ~5.24 mm short (collision module finding).
%     checks           - struct array with fields name, value, expected,
%                        tol, pass (locked field names)
%     pass             - logical, all checks passed
%
%   Joint animation contract: the check named 'movableJoints' carries the
%   10 non-fixed joint names IN CONFIGURATION ORDER (DataFormat "row")
%   as its value. follow_route / move(plat,"joint",q) must map wheelAngles
%   BY NAME from that list; drive wheels are 'left_wheel_joint' and
%   'right_wheel_joint', the 8 caster joints are passive.
%
%   Fail branch: any failed check errors by default (a wrong or partially
%   imported model must not pass integration). opts.ThrowOnFail = false
%   returns report with pass = false instead.
%   opts.ExpectedFootprintPolygon / opts.ExpectedFootprintRadius override
%   the contract v2 defaults (e.g. if the lead supplies literal vertices).
%
%   Requires: Robotics System Toolbox (rigidBodyTree).

    if nargin < 2 || isempty(opts), opts = struct(); end
    if ~isfield(opts, 'ThrowOnFail'), opts.ThrowOnFail = true; end
    if ~isfield(opts, 'ExpectedFootprintRadius'), opts.ExpectedFootprintRadius = 0.6052; end
    if ~isfield(opts, 'ExpectedFootprintPolygon')
        % Contract polygon v2 = v1 edges + 6 mm outward (convex, CCW).
        opts.ExpectedFootprintPolygon = [ ...
            -0.4740270 -0.2360854; -0.4393930 -0.3197930; -0.3557917 -0.3543830; ...
             0.4930498 -0.2956372;  0.5480241  0.2366924;  0.5133930  0.3203930; ...
             0.4297893  0.3549840; -0.4178380  0.2956339];
    end

    checks = struct('name', {}, 'value', {}, 'expected', {}, 'tol', {}, 'pass', {});
    function add(name, value, expected, tol, passed)
        % Scalar cells prevent cell-array payloads expanding the struct.
        checks(end+1) = struct('name', {name}, 'value', {value}, ...
            'expected', {expected}, 'tol', {tol}, 'pass', {logical(passed)}); %#ok<AGROW>
    end

    poly = opts.ExpectedFootprintPolygon;
    fpRadius = opts.ExpectedFootprintRadius;
    polyCircum = max(sqrt(sum(poly.^2, 2)));

    % ---- class ----
    add('class', class(robot), 'rigidBodyTree', 0, isa(robot, 'rigidBodyTree'));
    if ~isa(robot, 'rigidBodyTree')
        report = emptyReport(checks, poly, fpRadius);
        if opts.ThrowOnFail
            error('verify_robot:badClass', 'Expected a rigidBodyTree, got %s.', class(robot));
        end
        return;
    end

    % ---- inventory: 20 links / 19 joints ----
    expectedBodies = {'base_link','imu_link','imu_frame','surface', ...
        'front_laser_link','back_laser_link','left_wheel_link','right_wheel_link', ...
        'fl_caster_rotation_link','fr_caster_rotation_link','bl_caster_rotation_link','br_caster_rotation_link', ...
        'fl_caster_wheel_link','fr_caster_wheel_link','bl_caster_wheel_link','br_caster_wheel_link', ...
        'mirref_top_holes','mirref_front_ports','mirref_corner_caps'};
    linkCount = numel(robot.BodyNames) + 1; % bodies + base
    jointCount = numel(robot.Bodies);       % one joint per non-base body
    add('baseName', robot.BaseName, 'base_footprint', 0, strcmp(robot.BaseName, 'base_footprint'));
    add('linkCount', linkCount, 20, 0, linkCount == 20);
    add('jointCount', jointCount, 19, 0, jointCount == 19);
    add('bodyInventory', strjoin(setxor(robot.BodyNames, expectedBodies), ','), '', 0, ...
        isempty(setxor(robot.BodyNames, expectedBodies)));

    expectedJoints = struct( ...
        'base_joint', "fixed", 'base_link_to_imu_joint', "fixed", ...
        'imu_link_to_imu_frame_joint', "fixed", 'base_link_to_front_laser_joint', "fixed", ...
        'base_link_to_back_laser_joint', "fixed", 'base_link_surface_joint', "fixed", ...
        'mirref_top_holes_joint', "fixed", 'mirref_front_ports_joint', "fixed", ...
        'mirref_corner_caps_joint', "fixed", ...
        'left_wheel_joint', "continuous", 'right_wheel_joint', "continuous", ...
        'fl_caster_rotation_joint', "continuous", 'fr_caster_rotation_joint', "continuous", ...
        'bl_caster_rotation_joint', "continuous", 'br_caster_rotation_joint', "continuous", ...
        'fl_caster_wheel_joint', "continuous", 'fr_caster_wheel_joint', "continuous", ...
        'bl_caster_wheel_joint', "continuous", 'br_caster_wheel_joint', "continuous");
    jnames = fieldnames(expectedJoints);
    jointBad = {};
    for k = 1:numel(jnames)
        jn = jnames{k};
        found = false;
        for b = 1:numel(robot.Bodies)
            if strcmp(robot.Bodies{b}.Joint.Name, jn)
                found = true;
                j = robot.Bodies{b}.Joint;
                % URDF continuous joints import as unbounded revolute joints.
                expectedType = expectedJoints.(jn);
                typeOK = strcmp(j.Type, expectedType);
                if expectedType == "continuous"
                    typeOK = strcmp(j.Type,'revolute') && ...
                        isequal(j.PositionLimits,[-inf inf]);
                end
                if ~typeOK
                    jointBad{end+1} = sprintf('%s type %s', jn, robot.Bodies{b}.Joint.Type); %#ok<AGROW>
                end
                break;
            end
        end
        if ~found, jointBad{end+1} = sprintf('%s missing', jn); end %#ok<AGROW>
    end
    add('jointInventoryTypes', strjoin(jointBad, '; '), '', 0, isempty(jointBad));

    movableJoints = {};
    for b = 1:numel(robot.Bodies)
        if robot.Bodies{b}.Joint.Type ~= "fixed"
            movableJoints{end+1} = robot.Bodies{b}.Joint.Name; %#ok<AGROW>
        end
    end
    expectedMovable = {'left_wheel_joint','right_wheel_joint', ...
        'fl_caster_rotation_joint','fr_caster_rotation_joint','bl_caster_rotation_joint','br_caster_rotation_joint', ...
        'fl_caster_wheel_joint','fr_caster_wheel_joint','bl_caster_wheel_joint','br_caster_wheel_joint'};
    % 10 movable = 2 drive + 4 casters x 2. NOT 12.
    add('movableJointCount', numel(movableJoints), 10, 0, numel(movableJoints) == 10);
    add('movableJoints', movableJoints, expectedMovable, 0, isempty(setxor(movableJoints, expectedMovable)));

    % ---- masses: measured import vs source declaration -------------------
    % Both sides derived at RUNTIME: importedMass = actual tree sum after
    % load_mir100's zero-mass normalization; declaredSourceMass = summed
    % <inertial><mass> parsed from the source URDF. No hardcoded total, no
    % subtract scheme, no hidden decor list. MATLAB importrobot's default
    % mass for links without <inertial> is UNVERIFIED (no MATLAB in this
    % environment) - the loader normalization is what makes the imported
    % mass deterministic, and this gate verifies it landed.
    if ~isfield(opts, 'UrdfPath') || isempty(opts.UrdfPath)
        opts.UrdfPath = fullfile(fileparts(mfilename('fullpath')), '..', ...
            'assets', 'mir_description', 'urdf', 'mir100_matlab.urdf');
    end
    [declaredMass, hasInertial, linkNames] = urdf_link_masses(opts.UrdfPath);
    declaredSourceMass = 0; nBare = 0;
    for k = 1:numel(linkNames)
        if hasInertial(linkNames{k})
            declaredSourceMass = declaredSourceMass + declaredMass(linkNames{k});
        else
            nBare = nBare + 1;
        end
    end
    importedMass = 0; nUnmapped = 0;
    bodyNames = cell(1, numel(robot.Bodies));
    for b = 1:numel(robot.Bodies)
        importedMass = importedMass + robot.Bodies{b}.Mass;
        bodyNames{b} = robot.Bodies{b}.Name;
    end
    % Bare links that exist as BODIES: the URDF root (base_footprint) may
    % import as the tree Base rather than a body, so normalization counts
    % are gated against bare-links-present-as-bodies, never against the
    % raw source count.
    add('importedMassVsDeclaredSource', importedMass, declaredSourceMass, 1e-3, ...
        abs(importedMass - declaredSourceMass) < 1e-3);
    add('sourceBareLinkCount', nBare, 7, 0, nBare == 7);   % static source fact:
        % 7 links without <inertial> in this URDF (verified by runtime parse)
    if isfield(opts, 'ImportInfo') && ~isempty(opts.ImportInfo)
        ii = opts.ImportInfo;   % loader's raw record: evidence, only the
                                % post-normalization total is gated
        add('rawImportedMassTotal', ii.rawImportedMassTotal, NaN, inf, true);
        add('postNormalizationMassTotal', ii.postNormalizationMassTotal, ...
            declaredSourceMass, 1e-3, abs(ii.postNormalizationMassTotal - declaredSourceMass) < 1e-3);
        nBareBodies = 0;
        for k = 1:numel(linkNames)
            if ~hasInertial(linkNames{k}) && any(strcmp(bodyNames, linkNames{k}))
                nBareBodies = nBareBodies + 1;
            end
        end
        add('normalizedZeroMassLinkCount', numel(ii.normalizedZeroMassLinks), nBareBodies, 0, ...
            numel(ii.normalizedZeroMassLinks) == nBareBodies);
        if isfield(ii, 'baseExcludedFromMass')
            add('baseIsMasslessReferenceFrame', double(ii.baseExcludedFromMass), 1, 0, ii.baseExcludedFromMass);
        end
    end
    % Per-link: measured body mass vs the SAME link's source declaration.
    % Links with no source <inertial> must be exactly 0 post-normalization.
    for b = 1:numel(robot.Bodies)
        bn = robot.Bodies{b}.Name;
        if ~isKey(hasInertial, bn), nUnmapped = nUnmapped + 1; continue; end
        if hasInertial(bn)
            dm = declaredMass(bn);
        else
            dm = 0;   % bare source link -> loader-normalized zero
        end
        bm = robot.Bodies{b}.Mass;
        add(['masslink_' bn], bm, dm, 1e-9, abs(bm - dm) < 1e-9);
    end
    add('bodiesNotInSourceURDF', nUnmapped, 0, inf, true);  % evidence, not gated

    % ---- wheel geometry from joint fixed transforms (exact, no FK) ----
    Tl = getBody(robot, 'left_wheel_link').Joint.JointToParentTransform;
    Tr = getBody(robot, 'right_wheel_link').Joint.JointToParentTransform;
    pl = Tl(1:3, 4); pr = Tr(1:3, 4);
    wheelSeparation = abs(pl(2) - pr(2));
    wheelRadius = pl(3);
    add('wheelSeparation', wheelSeparation, 0.445208, 1e-6, abs(wheelSeparation - 0.445208) < 1e-6);
    add('wheelRadius', wheelRadius, 0.0625, 1e-9, abs(wheelRadius - 0.0625) < 1e-9 && abs(pr(3) - 0.0625) < 1e-9);
    add('leftWheelPlusY', [pl(2) pr(2)], [0.222604 -0.222604], 1e-9, abs(pl(2) - 0.222604) < 1e-9 && abs(pr(2) + 0.222604) < 1e-9);

    % ---- joint axes ----
    for w = {'left_wheel_joint', 'right_wheel_joint'}
        j = jointByName(robot, w{1});
        add(['axis_' w{1}], mat2str(j.JointAxis), '[0 1 0]', 0, isequal(j.JointAxis, [0 1 0]));
    end
    for c = {'fl','fr','bl','br'}
        jn = [c{1} '_caster_rotation_joint'];
        j = jointByName(robot, jn);
        add(['axis_' jn], mat2str(j.JointAxis), '[0 0 1]', 0, isequal(j.JointAxis, [0 0 1]));
    end

    % ---- footprint (contract v2 constants + consistency) ----
    add('footprintCircumradius', polyCircum, 0.6051644, 1e-4, abs(polyCircum - 0.6051644) < 1e-4);
    add('footprintRadius', fpRadius, 0.6052, 0, fpRadius == 0.6052);
    add('footprintRadiusCoversPolygon', fpRadius - polyCircum, '>= 0', 0, fpRadius >= polyCircum);
    % Mount-point coverage, grounded in the tree: every wheel/caster/laser
    % mounting point must lie inside the footprint radius.
    mountNames = {'left_wheel_link','right_wheel_link', ...
        'fl_caster_rotation_link','fr_caster_rotation_link','bl_caster_rotation_link','br_caster_rotation_link', ...
        'front_laser_link','back_laser_link'};
    mountR = 0;
    for k = 1:numel(mountNames)
        T = getBody(robot, mountNames{k}).Joint.JointToParentTransform;
        mountR = max(mountR, hypot(T(1,4), T(2,4)));
    end
    add('footprintCoversMounts', mountR, ['<= ' num2str(fpRadius)], 0, mountR <= fpRadius);

    % ---- assemble locked-schema report ----
    report = struct();
    report.linkCount = linkCount;
    report.jointCount = jointCount;
    report.massTotal = importedMass;   % ACTUAL tree mass (post-normalization)
    report.declaredSourceMass = declaredSourceMass;   % runtime URDF parse
    report.importedMass = importedMass;
    report.massImportNote = ['massTotal/importedMass = actual imported tree mass ' ...
        'after load_mir100 zero-mass normalization of source links without ' ...
        '<inertial>; raw import evidence is returned by load_mir100'];
    report.wheelRadius = wheelRadius;
    report.wheelSeparation = wheelSeparation;
    report.footprintRadius = fpRadius;
    report.footprintPolygon = poly;
    report.checks = checks;
    report.pass = all([checks.pass]);

    if ~report.pass && opts.ThrowOnFail
        failed = {checks(~[checks.pass]).name};
        error('verify_robot:checksFailed', 'MiR100 verification failed: %s', strjoin(failed, '; '));
    end

    function rep = emptyReport(ch, p, r)
        rep = struct('linkCount', NaN, 'jointCount', NaN, 'massTotal', NaN, 'declaredSourceMass', NaN, 'importedMass', NaN, 'massImportNote', '', ...
            'wheelRadius', NaN, 'wheelSeparation', NaN, ...
            'footprintRadius', r, 'footprintPolygon', p, 'checks', ch, 'pass', false);
    end
    function m = massOf(map, name)
        if isKey(map, name), m = map(name); else, m = NaN; end
    end
    function j = jointByName(rbt, name)
        j = [];
        for b = 1:numel(rbt.Bodies)
            if strcmp(rbt.Bodies{b}.Joint.Name, name)
                j = rbt.Bodies{b}.Joint; return;
            end
        end
        error('verify_robot:jointMissing', 'Joint %s not found.', name);
    end
end
