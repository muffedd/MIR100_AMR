function log = roslink_goto_goal(link, goalXY, opts)
%ROSLINK_GOTO_GOAL Closed-loop Gazebo-client controller over mir100_gazebo_link.
%
%   log = ROSLINK_GOTO_GOAL(link, goalXY) drives the SIMULATED MiR100 in the
%   RUNNING Gazebo world toward goalXY using ONLY observed /odom and
%   /scan + /back_scan readings from mir100_gazebo_link. goalXY is in the
%   /odom frame (log.frameId). Every decision uses live subscriber data
%   with explicit freshness gates; on stale or missing readings the robot
%   is STOPPED and the reason recorded.
%
%   MANDATORY OPT-IN (R3): the braking model rests on an UNVERIFIED
%   deceleration assumption (opts.AssumedLinearDecel, default 0.5 m/s^2 =
%   50% derated plugin max_linear_acceleration 1.0; gz-sim decel symmetry
%   is not verifiable from the repo). This function ERRORS BEFORE ANY
%   MOTION unless opts.DecelAssumptionAccepted = true. Calibrate on the
%   live system first (procedure in README.md).
%
%   Current-speed braking check (R1): commanding a smaller v does not
%   instantly slow the robot. Each step first evaluates the CURRENT
%   measured speed from /odom twist (abs value, so a reversing robot is
%   covered): demand = |v_cur|*L + v_cur^2/(2*aDec). If demand > D the
%   robot cannot stop before its footprint reaches the nearest forward
%   hit: zero Twist, stopReason 'unsafe_current_speed'.
%
%   Rotation gating (R2): ANY nonzero angular command - including shallow
%   arcs with 0 < |angErr| <= pi/4, not just in-place turns - requires
%   BOTH scans fresh, full 360 deg ray-direction coverage (max gap <=
%   opts.MaxCoverageGapDeg) and nearest hit in ANY direction >=
%   FootprintRadius + SafetyBuffer. Otherwise stop 'blind_coverage' /
%   'turn_blocked'. Pure rotation changes no radial obstacle distance;
%   arcs additionally pass the forward braking bound.
%
%   Geometry: rays are transformed from the canonical laser mounts to the
%   BASE frame by the link (front [0.4288 0.2358] yaw +45 deg; back
%   [-0.3548 -0.2352] yaw -135 deg). Forward braking room
%   D = minFwd - FootprintRadius - SafetyBuffer over base-frame hits with
%   x_base > 0. Latency L = ControlPeriod + max(front scan age, back scan
%   age, odom age) - measured each step, conservative.
%
%   NaN returns are unknown observations -> stop 'unknown_returns'.
%   Inf = no return within the 29 m max range (model config) -> clear.
%   A PASS here is NOT a collision-free claim: discrete scan fans have
%   inter-beam gaps, a 0.05 m minimum range, sensor noise (0.01 m), and
%   the deceleration assumption above (logged: log.decelVerified = false).
%   This controller logs measurements; it does not certify safety,
%   navigation success, or real-time performance. NOT runtime-verified in
%   the authoring environment (no MATLAB / ROS 2).
%
%   opts fields (defaults):
%     DecelAssumptionAccepted false (MANDATORY: must be set true)
%     GoalTolerance 0.15 m | MaxSimTime 120 s | ControlPeriod 0.1 s
%     FootprintRadius 0.6052 m (contract polygon v2 circumradius)
%     SafetyBuffer 0.15 m | AssumedLinearDecel 0.5 m/s^2 (derated)
%     KpHeading 1.5 | SlowdownDistance 1.0 m | StaleTimeoutSec 2.0
%     RequireScans true | MaxCoverageGapDeg 20
%
%   log fields: t, x, y, yaw, v, w, vMeasured, vBrake, minBaseRange,
%   minForwardRange, stopReason ('goal_reached' | 'obstacle' |
%   'unsafe_current_speed' | 'turn_blocked' | 'blind_coverage' |
%   'unknown_returns' | 'stale_odom' | 'stale_scan' | 'no_scan_data' |
%   'timeout'), samples, goalXY, frameId, assumedLinearDecel,
%   decelVerified (always false - calibration is a live-system task).

    if nargin < 3 || isempty(opts), opts = struct(); end
    % Also attempt a stop if a subscriber callback or command throws.
    stopGuard = onCleanup(@() stopOnExit(link)); %#ok<NASGU>
    if ~isfield(opts, 'DecelAssumptionAccepted'), opts.DecelAssumptionAccepted = false; end
    if ~isfield(opts, 'GoalTolerance'),      opts.GoalTolerance = 0.15; end
    if ~isfield(opts, 'MaxSimTime'),         opts.MaxSimTime = 120; end
    if ~isfield(opts, 'ControlPeriod'),      opts.ControlPeriod = 0.1; end
    if ~isfield(opts, 'FootprintRadius'),    opts.FootprintRadius = 0.6052; end
    if ~isfield(opts, 'SafetyBuffer'),       opts.SafetyBuffer = 0.15; end
    if ~isfield(opts, 'AssumedLinearDecel'), opts.AssumedLinearDecel = 0.5; end
    if ~isfield(opts, 'KpHeading'),          opts.KpHeading = 1.5; end
    if ~isfield(opts, 'SlowdownDistance'),   opts.SlowdownDistance = 1.0; end
    if ~isfield(opts, 'StaleTimeoutSec'),    opts.StaleTimeoutSec = 2.0; end
    if ~isfield(opts, 'RequireScans'),       opts.RequireScans = true; end
    if ~isfield(opts, 'MaxCoverageGapDeg'),  opts.MaxCoverageGapDeg = 20; end

    if ~opts.DecelAssumptionAccepted
        % Zero Twist is safe to send even here; motion is not.
        try, link.stop(); catch, end
        error('roslink_goto_goal:decelAssumptionNotAccepted', ['Refusing motion: the braking model uses an UNVERIFIED ', ...
            'deceleration assumption (AssumedLinearDecel=%.2f m/s^2). Calibrate on the live system ', ...
            '(README.md procedure) and set opts.DecelAssumptionAccepted=true.'], opts.AssumedLinearDecel);
    end

    maxV = link.limits.maxLinearSpeed;
    gateR = opts.FootprintRadius + opts.SafetyBuffer;
    a = opts.AssumedLinearDecel;
    log = struct('t', [], 'x', [], 'y', [], 'yaw', [], 'v', [], 'w', [], ...
        'vMeasured', [], 'vBrake', [], 'minBaseRange', [], 'minForwardRange', [], ...
        'stopReason', 'timeout', 'samples', 0, 'goalXY', goalXY, 'frameId', 'odom', ...
        'assumedLinearDecel', a, 'decelVerified', false);
    t0 = tic;
    everScan = false;

    while toc(t0) < opts.MaxSimTime
        od = link.latestOdom();
        if ~od.fresh || od.ageSec > opts.StaleTimeoutSec
            link.stop(); log.stopReason = 'stale_odom'; break;
        end
        sf = link.latestScan('front');
        sb = link.latestScan('back');
        everScan = everScan || sf.fresh || sb.fresh;
        if opts.RequireScans && (~sf.fresh || ~sb.fresh || ...
                sf.ageSec > opts.StaleTimeoutSec || sb.ageSec > opts.StaleTimeoutSec)
            link.stop();
            if everScan, log.stopReason = 'stale_scan'; else, log.stopReason = 'no_scan_data'; end
            break;
        end

        xb = [sf.xBase, sb.xBase]; yb = [sf.yBase, sb.yBase];
        nanTotal = sf.nanCount + sb.nanCount;
        dBase = hypot(xb, yb);
        fwd = xb > 0;
        minBase = ifelse(isempty(dBase), Inf, min(dBase));
        minFwd  = ifelse(~any(fwd), Inf, min(dBase(fwd)));

        L = opts.ControlPeriod + max([sf.ageSec, sb.ageSec, od.ageSec]); % conservative latency
        D = minFwd - gateR;
        vBrake = brakeBound(D, L, a);

        % R1: current measured speed must already be stoppable within D.
        vCur = abs(od.v);
        if ~isfinite(vCur), vCur = maxV; end % unknown speed -> worst case
        if vCur * L + vCur^2 / (2 * a) > D
            link.stop(); log.stopReason = 'unsafe_current_speed';
            log = appendRow(log, toc(t0), od, vCur, 0, 0, vBrake, minBase, minFwd);
            break;
        end

        dx = goalXY(1) - od.x; dy = goalXY(2) - od.y;
        dist = hypot(dx, dy);
        if dist <= opts.GoalTolerance
            link.stop(); log.stopReason = 'goal_reached';
            log = appendRow(log, toc(t0), od, vCur, 0, 0, vBrake, minBase, minFwd);
            break;
        end

        if nanTotal > 0
            link.stop(); log.stopReason = 'unknown_returns';
            log = appendRow(log, toc(t0), od, vCur, 0, 0, vBrake, minBase, minFwd);
            break;
        end

        angErr = wrapPi(atan2(dy, dx) - od.yaw);
        w = opts.KpHeading * angErr;
        vDes = maxV * min(1, dist / opts.SlowdownDistance);
        if abs(angErr) > pi/4, vDes = 0; end % turn in place first

        % R2: ANY nonzero angular command gates coverage + radial clearance.
        if w ~= 0
            [covered, ~] = fullCoverage(sf.bearingBase, sb.bearingBase, deg2rad(opts.MaxCoverageGapDeg));
            if ~covered
                link.stop(); log.stopReason = 'blind_coverage';
                log = appendRow(log, toc(t0), od, vCur, 0, 0, vBrake, minBase, minFwd);
                break;
            end
            if minBase < gateR
                link.stop(); log.stopReason = 'turn_blocked';
                log = appendRow(log, toc(t0), od, vCur, 0, 0, vBrake, minBase, minFwd);
                break;
            end
        end

        if vDes > 0
            if D <= 0
                link.stop(); log.stopReason = 'obstacle';
                log = appendRow(log, toc(t0), od, vCur, 0, 0, vBrake, minBase, minFwd);
                break;
            end
            v = min(vDes, vBrake);
        else
            v = 0;
        end

        link.sendCmdVel(v, w);
        log = appendRow(log, toc(t0), od, vCur, v, w, vBrake, minBase, minFwd);
        pause(opts.ControlPeriod);
    end
    link.stop(); % terminal safety stop on every exit path
    log.samples = numel(log.t);

    function l = appendRow(l, t, od, vM, v, w, vB, minB, minF)
        l.t(end+1,1) = t; l.x(end+1,1) = od.x; l.y(end+1,1) = od.y;
        l.yaw(end+1,1) = od.yaw; l.vMeasured(end+1,1) = vM;
        l.v(end+1,1) = v; l.w(end+1,1) = w; l.vBrake(end+1,1) = vB;
        l.minBaseRange(end+1,1) = minB; l.minForwardRange(end+1,1) = minF;
    end
    function a2 = wrapPi(a1)
        a2 = atan2(sin(a1), cos(a1));
    end
    function y = ifelse(c, a1, b1)
        if c, y = a1; else, y = b1; end
    end
    function vB = brakeBound(D, L, a)
        if D <= 0
            vB = 0;
        else
            vB = max(0, -a * L + sqrt(a^2 * L^2 + 2 * a * D));
        end
    end
    function [ok, maxGap] = fullCoverage(bearF, bearB, maxGapRad)
        b = sort(wrapPi([bearF(:); bearB(:)]));
        if numel(b) < 2, ok = false; maxGap = Inf; return; end
        gaps = diff([b; b(1) + 2*pi]);
        maxGap = max(gaps);
        ok = maxGap <= maxGapRad;
    end
end

function stopOnExit(link)
try
    link.stop();
catch ME
    warning('roslink_goto_goal:stopFailed','Could not publish terminal stop: %s',ME.message);
end
end
