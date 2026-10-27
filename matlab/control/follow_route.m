function trajLog = follow_route(plat, wpsWorld, opts)
%FOLLOW_ROUTE Kinematic differential-drive pure-pursuit route follower.
%   trajLog = FOLLOW_ROUTE(plat, wpsWorld, opts) drives a rigidBodyTree-based
%   robotPlatform (the spawned MiR100) along 2-D world-frame waypoints using
%   a kinematic differential-drive model with pure-pursuit steering. The
%   platform base and its left/right actuated wheel joints are moved through
%   the verified robotPlatform APIs:
%       move(plat,"base",motion16)   % [x y z vx vy vz ax ay az qw qx qy qz wx wy wz]
%       move(plat,"joint",config)    % full rigidBodyTree joint vector
%       motion = read(plat,"base")   % 16-element base motion (same ordering)
%   (MathWorks robotPlatform.move / robotPlatform.read, R2022a+.)
%
%   Inputs
%     plat      - robotPlatform object for the spawned MiR100.
%     wpsWorld  - Nx2 or Nx3 waypoints [x y (z)] in the scenario world frame
%                 (metres). N >= 2. Every waypoint must be reached within
%                 opts.wpRadius in sequence; completion is NOT defined by the
%                 final pose alone.
%     opts      - (optional) struct of tuning fields; any missing field takes
%                 the default in DEFAULTS below. Recognised fields:
%                   dt            integration/log step [s]           (0.01)
%                   lookahead     pure-pursuit lookahead [m]         (0.6)
%                   maxLinVel     linear speed limit [m/s]           (1.0)
%                   maxAngVel     angular speed limit [rad/s]        (2.0)
%                   wpRadius      waypoint acceptance radius [m]     (0.15)
%                   goalRadius    final-goal acceptance radius [m]   (0.10)
%                   maxSimTime    simulated-time budget before timeout [s] (600)
%                   maxTime       accepted alias for maxSimTime
%                   maxSteps      hard step cap                   (ceil(1.5*maxSimTime/dt))
%                   wheelRadius   drive wheel radius [m]             (0.0625)
%                   wheelSep      drive wheel separation [m]         (0.4452)
%                   logStride     log every k-th step                (1)
%                   scenario      optional robotScenario handle; if given it
%                                 is advance()d each step and (when dt was
%                                 not set explicitly) dt = 1/scenario.UpdateRate
%
%   Output trajLog (struct) - locked project schema fields:
%     .t           Mx1 time stamps [s]
%     .x           Mx1 base x [m] (world frame)
%     .y           Mx1 base y [m] (world frame)
%     .yaw         Mx1 base heading [rad]
%     .wheelAngles Mx2 [left right] integrated wheel angles [rad]
%     .waypoints   the commanded route (as passed in, Nx2 or Nx3)
%     .reachedIdx  indices of reached waypoints, in order
%     .diagnosticOnly  true (kinematic demonstration, not physical validation)
%     .plannerStatus   'success' | 'timeout' | 'maxsteps'
%   Extra diagnostic fields retained:
%     .visited        Nx1 logical, per-waypoint completion flags
%     .allWaypointsReached  scalar logical (true only if every waypoint hit)
%     .status         alias of plannerStatus
%     .reason         human-readable termination reason
%     .finalError     distance to final waypoint at termination [m]
%     .wheelsCommanded / .wheelJointNames  wheel-joint resolution record
%     .opts           the effective option set used
%
%   NOTE: kinematic demonstration only. This is not a physical validation of
%   the MiR100 and implies no real-time guarantee.

    narginchk(2, 3);
    if nargin < 3 || isempty(opts)
        opts = struct();
    end
    validateattributes(plat, {'robotPlatform'}, {'scalar'}, mfilename, 'plat', 1);
    if isempty(plat.RigidBodyTree)
        error('follow_route:noRBT', ...
            'plat must be a rigidBodyTree-based robotPlatform (spawn_mir100 output).');
    end
    validateattributes(wpsWorld, {'numeric'}, {'real','finite','2d','nonempty'}, ...
        mfilename, 'wpsWorld', 2);
    if size(wpsWorld,1) < 2 || size(wpsWorld,2) < 2 || size(wpsWorld,2) > 3
        error('follow_route:badWps', 'wpsWorld must be Nx2 or Nx3, got %dx%d.', ...
            size(wpsWorld,1), size(wpsWorld,2));
    end
    opts = fill_opts(opts);
    wps = wpsWorld(:,1:2);
    nWps = size(wps,1);

    % --- Initial state from the spawned platform (do not assume spawn pose) ---
    m0 = read(plat,"base");
    x   = m0(1);  y = m0(2);  z = m0(3);
    yaw = quat_to_yaw(m0(10:13));

    % --- Locate actuated wheel joints (exact MiR100 names, then fallback) ---
    rbt  = plat.RigidBodyTree;
    % DataFormat is 'row': homeConfiguration is NUMERIC, so collect the
    % non-fixed joint names in body order (the 'row' config order) instead.
    names = {};
    for bi = 1:numel(rbt.Bodies)
        jb = rbt.Bodies{bi}.Joint;
        if ~strcmp(jb.Type, 'fixed'), names{end+1} = jb.Name; end %#ok<AGROW>
    end
    jcfg = zeros(1, numel(names));   % row vector: DataFormat 'row'
    iL = find(strcmp(names,'left_wheel_joint'), 1);
    iR = find(strcmp(names,'right_wheel_joint'), 1);
    if isempty(iL)
        iL = find(~cellfun(@isempty, regexp(names,'(^|_)left_wheel_joint$','once')), 1);
    end
    if isempty(iR)
        iR = find(~cellfun(@isempty, regexp(names,'(^|_)right_wheel_joint$','once')), 1);
    end
    moveWheels = ~isempty(iL) && ~isempty(iR);

    wheelAngles = [0 0];            % [left right], integrated
    visited = false(nWps,1);
    iw = 1;                         % active waypoint index

    % --- Preallocate log ---
    cap = ceil(opts.maxSteps/opts.logStride) + 1;
    LT = zeros(cap,1); LX = zeros(cap,1); LY = zeros(cap,1);
    LH = zeros(cap,1); LW = zeros(cap,2);
    nlog = 0;

    status = 'timeout';
    reason = 'maxSimTime budget exceeded before reaching final waypoint';

    t = 0;
    k = 0;
    logstep(); % Include the spawn pose in collision and tracking evidence.
    for k = 1:opts.maxSteps
        % --- Waypoint gating: every waypoint must come within wpRadius,
        %     in sequence, before the next one becomes active. ---
        while iw <= nWps && norm([x y] - wps(iw,:)) < eff_radius(opts, iw, nWps)
            visited(iw) = true;
            iw = iw + 1;
        end
        if iw > nWps
            status = 'success';
            reason = sprintf('all %d waypoints reached within acceptance radii', nWps);
            t = t + opts.dt;   % final log entry below uses updated t
            logstep();
            break
        end

        % --- Pure-pursuit lookahead on the remaining polyline, clamped so
        %     the active waypoint cannot be skipped past its radius. ---
        dActive = norm(wps(iw,:) - [x y]);
        rAcc = eff_radius(opts, iw, nWps);
        if iw == 1
            dirv = wps(2,:) - wps(1,:);
        else
            dirv = wps(iw,:) - wps(iw-1,:);
        end
        nd = norm(dirv);
        if nd < eps, sPast = 0; else, sPast = dot([x y] - wps(iw,:), dirv)/nd; end
        if sPast > rAcc
            % Unvisited active waypoint already lies behind us along the
            % path: home back onto it (gating forbids skipping waypoints).
            target = wps(iw,:);
            Ld = max(dActive, 0.5*opts.wpRadius);
        else
            % Lookahead clamped so the active waypoint cannot be skipped
            % past its acceptance radius.
            Ld = min(opts.lookahead, max(dActive, 0.5*opts.wpRadius));
            target = lookahead_point([x y], wps(iw:end,:), Ld);
        end

        alpha = wrap_to_pi(atan2(target(2)-y, target(1)-x) - yaw);
        if abs(alpha) > pi/2
            % Target behind: rotate in place (unicycle cannot reverse here).
            v = 0;
            w = opts.maxAngVel * sign(alpha);
        else
            kappa = 2*sin(alpha)/Ld;
            v = opts.maxLinVel * max(0.15, 1 - abs(alpha)/(pi/2));
            if iw == nWps
                v = min(v, max(0.05, 2*dActive));   % slow into the goal
            end
            w = max(-opts.maxAngVel, min(opts.maxAngVel, v*kappa));
        end

        % --- Kinematic unicycle integration ---
        yaw = yaw + w*opts.dt; % Continuous yaw is required by swept clearance.
        x = x + v*cos(yaw)*opts.dt;
        y = y + v*sin(yaw)*opts.dt;
        t = t + opts.dt;

        % --- Wheel kinematics: w_l = (v - w*L/2)/r, w_r = (v + w*L/2)/r ---
        wL = (v - w*opts.wheelSep/2) / opts.wheelRadius;
        wR = (v + w*opts.wheelSep/2) / opts.wheelRadius;
        wheelAngles = wheelAngles + [wL wR]*opts.dt;

        % --- Command the platform through verified APIs ---
        q = yaw_to_quat(yaw);
        move(plat, "base", [x y z, v*cos(yaw) v*sin(yaw) 0, 0 0 0, q, 0 0 w]);
        if moveWheels
            jcfg(iL) = wheelAngles(1);
            jcfg(iR) = wheelAngles(2);
            move(plat, "joint", jcfg);
        end
        if ~isempty(opts.scenario)
            advance(opts.scenario);
        end

        logstep();

        if t >= opts.maxSimTime
            break
        end
    end
    if strcmp(status,'timeout') && k >= opts.maxSteps && t < opts.maxSimTime
        status = 'maxsteps';
        reason = 'maxSteps exceeded before reaching final waypoint';
    end

    % --- Assemble contract log (locked project schema) ---
    trajLog = struct();
    trajLog.t   = LT(1:nlog);
    trajLog.x   = LX(1:nlog);
    trajLog.y   = LY(1:nlog);
    trajLog.yaw = LH(1:nlog);
    trajLog.wheelAngles = LW(1:nlog,:);
    trajLog.waypoints = wpsWorld;            % route as commanded (Nx2 or Nx3)
    trajLog.reachedIdx = find(visited);      % indices of waypoints reached, in order
    if isfield(opts, 'diagnosticOnly')
        trajLog.diagnosticOnly = logical(opts.diagnosticOnly);  % root owns this label
    else
        trajLog.diagnosticOnly = true;   % standalone follow = diagnostic, not planner-validated
    end
    trajLog.plannerStatus = status;          % 'success' | 'timeout' | 'maxsteps'
    % --- Extras (diagnostics retained above the locked schema) ---
    trajLog.visited = visited;
    trajLog.allWaypointsReached = all(visited);
    trajLog.status = status;
    trajLog.reason = reason;
    trajLog.finalError = norm([x y] - wps(end,:));
    trajLog.wheelsCommanded = moveWheels;   % false => wheel joints not found by name
    trajLog.wheelJointNames = '';
    if moveWheels
        trajLog.wheelJointNames = [names{iL} ',' names{iR}];
    end
    trajLog.opts = opts;

    function logstep()
        if mod(k, opts.logStride) == 0 || (exist('status','var') && strcmp(status,'success') && iw > nWps)
            nlog = nlog + 1;
            LT(nlog) = t; LX(nlog) = x; LY(nlog) = y; LH(nlog) = yaw;
            LW(nlog,:) = wheelAngles;
        end
    end
end

% -------------------------------------------------------------------------
function r = eff_radius(opts, iw, nWps)
    if iw == nWps, r = opts.goalRadius; else, r = opts.wpRadius; end
end

function target = lookahead_point(p, path, Ld)
% First point on the polyline `path` at distance >= Ld from p (interpolated
% on the segment); falls back to the farthest vertex reachable.
    target = path(1,:);   % fallback: home onto the active waypoint (never skip it)
    for i = 1:size(path,1)-1
        a = path(i,:); b = path(i+1,:); d = b - a;
        f = a - p;
        % Solve |f + u d|^2 = Ld^2 for u in [0,1].
        A = dot(d,d); B = 2*dot(f,d); C = dot(f,f) - Ld^2;
        disc = B*B - 4*A*C;
        if A < eps || disc < 0
            continue
        end
        u = (-B + sqrt(disc)) / (2*A);   % forward-most intersection on segment
        if u >= 0 && u <= 1
            target = a + u*d;
            return
        end
        u = (-B - sqrt(disc)) / (2*A);
        if u >= 0 && u <= 1
            target = a + u*d;
            return
        end
    end
end

function opts = fill_opts(opts)
    def = struct('dt',0.01, 'lookahead',0.6, 'maxLinVel',1.0, 'maxAngVel',2.0, ...
        'wpRadius',0.15, 'goalRadius',0.10, 'maxSimTime',600, 'maxTime',[], 'maxSteps',[], ...
        'wheelRadius',0.0625, 'wheelSep',0.4452, 'logStride',1, 'scenario',[]);
    dtSet = isfield(opts,'dt') && ~isempty(opts.dt);
    simSet = isfield(opts,'maxSimTime') && ~isempty(opts.maxSimTime);
    timeSet = isfield(opts,'maxTime') && ~isempty(opts.maxTime);
    f = fieldnames(def);
    for i = 1:numel(f)
        if ~isfield(opts,f{i}) || isempty(opts.(f{i}))
            opts.(f{i}) = def.(f{i});
        end
    end
    % opts.maxTime is accepted as an alias; maxSimTime is canonical.
    if ~simSet && timeSet
        opts.maxSimTime = opts.maxTime;
    end
    opts.maxTime = opts.maxSimTime;   % keep alias consistent in echoed opts
    if ~isempty(opts.scenario)
        validateattributes(opts.scenario, {'robotScenario'}, {'scalar'}, ...
            mfilename, 'opts.scenario');
        if ~dtSet
            opts.dt = 1/opts.scenario.UpdateRate;
        end
    end
    pos = {'dt','lookahead','maxLinVel','maxAngVel','wpRadius','goalRadius', ...
           'maxSimTime','wheelRadius','wheelSep','logStride'};
    for i = 1:numel(pos)
        validateattributes(opts.(pos{i}), {'numeric'}, {'scalar','real','finite','positive'}, ...
            mfilename, ['opts.' pos{i}]);
    end
    if isempty(opts.maxSteps)
        opts.maxSteps = ceil(1.5*opts.maxSimTime/opts.dt);
    end
    opts.logStride = max(1, round(opts.logStride));
end

function q = yaw_to_quat(yaw)
% [qw qx qy qz] for a pure-yaw rotation (robotPlatform quaternion ordering).
    q = [cos(yaw/2) 0 0 sin(yaw/2)];
end

function yaw = quat_to_yaw(q)
% Yaw from [qw qx qy qz].
    yaw = atan2(2*(q(1)*q(4) + q(2)*q(3)), 1 - 2*(q(3)^2 + q(4)^2));
end

function a = wrap_to_pi(a)
    a = mod(a + pi, 2*pi) - pi;
end
