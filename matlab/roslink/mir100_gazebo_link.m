function link = mir100_gazebo_link(opts)
%MIR100_GAZEBO_LINK MATLAB client of the RUNNING ROS_mir100 Gazebo sim.
%
%   link = MIR100_GAZEBO_LINK() connects MATLAB to the existing Gazebo
%   Harmonic sim from ROS_mir100 @ 6450a77. MATLAB is a CLIENT: it
%   PUBLISHES velocity commands and SUBSCRIBES to the sim's sensor and
%   odometry topics. It publishes nothing synthetic and synthesizes no
%   sensor data; every reading returned comes from a live subscriber
%   callback and carries an explicit freshness flag.
%
%   Topic directions/types VERIFIED against the repo's own bridge nodes
%   (src/mir100_navigation/launch/slam_navigation_hardtest.launch.py,
%   nodes mir100_gz_bridge / mir100_front_scan_bridge):
%     /cmd_vel   geometry_msgs/Twist     MATLAB PUBLISHES  (]gz.msgs.Twist = ROS -> gz)
%     /odom      nav_msgs/Odometry       MATLAB subscribes ([gz.msgs.Odometry = gz -> ROS)
%     /scan      sensor_msgs/LaserScan   MATLAB subscribes (front SICK S300)
%     /back_scan sensor_msgs/LaserScan   MATLAB subscribes (back SICK S300)
%     /pose      tf2_msgs/TFMessage      OPTIONAL ground truth: the ref
%                model's gz PosePublisher emits link poses, but the repo
%                does NOT bridge them to ROS 2. Subscribing needs an extra
%                user-side bridge (see README). link.groundTruthSubscribed
%                only means the subscriber was created; actual data
%                presence is gated per-reading in latestGroundTruth().
%
%   LASER MOUNT TRANSFORMS (canonical ref model, verified from the
%   flattened mir100_ref.urdf.xacro joint origins; sensor <pose> is zero
%   so the scan frame IS the link frame):
%     front_laser_link: xyz [0.4288 0.2358 0.1914], yaw +pi/4  (45 deg)
%     back_laser_link:  xyz [-0.3548 -0.2352 0.1914], yaw -3*pi/4 (-135 deg)
%   latestScan returns ray hit points transformed into the BASE frame
%   (xBase, yBase, bearingBase), because clearance is a BASE-frame
%   quantity: a raw range is the distance from the LASER, not from the
%   robot center. Mount constants are opts-overridable.
%
%   Handles (nested-function workspace keeps the node alive while `link`
%   is alive):
%     link.sendCmdVel(v, w)   - clamped to link.limits (policy bounds;
%                               defaults 1.0 m/s, 1.5 rad/s; the sim plugin
%                               limits ACCELERATIONS 1.0 m/s^2 / 1.5 rad/s^2)
%     link.stop()             - publish zero Twist immediately
%     link.latestScan(which)  - 'front'|'back' -> struct(fresh, ageSec,
%                               count, ranges, angles, xBase, yBase,
%                               bearingBase, nanCount, frameId)
%     link.latestOdom()       -> struct(fresh, ageSec, count, x, y, yaw, v, w)
%     link.latestGroundTruth()-> struct(subscribed, fresh, found, ageSec,
%                               x, y, yaw, source)
%     link.isFresh(kind, maxAgeSec) - kind 'odom'|'front'|'back'|'groundtruth'
%     link.limits             - struct(maxLinearSpeed, maxAngularSpeed)
%     link.close()            - stop + release subscribers/publisher/node
%
%   Freshness is wall-clock time since the last subscriber callback -
%   honest and independent of sim-time/clock bridging.
%
%   PREREQUISITES: MATLAB R2022a+ (project baseline) with ROS Toolbox
%   (ros2node/ros2publisher/ros2subscriber/ros2message - Introduced R2019b
%   per MathWorks reference pages), DDS reachability to the Gazebo host
%   (same ROS_DOMAIN_ID). NOT runtime-verified: no MATLAB and no ROS 2 in
%   the authoring environment. Validate on first live run with ros2 topic list
%   and a zero-velocity command before any motion.

    if nargin < 1 || isempty(opts), opts = struct(); end
    if ~isfield(opts, 'NodeName'),        opts.NodeName = 'mir100_matlab_client'; end
    if ~isfield(opts, 'DomainID'),        opts.DomainID = []; end
    if ~isfield(opts, 'TopicCmdVel'),     opts.TopicCmdVel = '/cmd_vel'; end
    if ~isfield(opts, 'TopicOdom'),       opts.TopicOdom = '/odom'; end
    if ~isfield(opts, 'TopicScanFront'),  opts.TopicScanFront = '/scan'; end
    if ~isfield(opts, 'TopicScanBack'),   opts.TopicScanBack = '/back_scan'; end
    if ~isfield(opts, 'TopicGroundTruth'),opts.TopicGroundTruth = '/pose'; end
    if ~isfield(opts, 'SubscribeGroundTruth'), opts.SubscribeGroundTruth = true; end
    if ~isfield(opts, 'MaxLinearSpeed'),  opts.MaxLinearSpeed = 1.0; end   % m/s, controller policy bound
    if ~isfield(opts, 'MaxAngularSpeed'), opts.MaxAngularSpeed = 1.5; end  % rad/s, controller policy bound
    % Canonical ref-model laser mounts [tx ty yaw]: verified from the
    % flattened mir100_ref.urdf.xacro fixed joint origins.
    if ~isfield(opts, 'MountFront'),      opts.MountFront = [0.4288, 0.2358, pi/4]; end
    if ~isfield(opts, 'MountBack'),       opts.MountBack = [-0.3548, -0.2352, -3*pi/4]; end

    if exist('ros2node', 'file') ~= 2
        error('mir100_gazebo_link:noRosToolbox', ['ROS Toolbox not found (ros2node missing). ', ...
            'This link requires MATLAB with ROS Toolbox; the MATLAB Online core runs without it.']);
    end

    t0 = tic;
    st = struct();
    st.odom  = struct('count', 0, 't', -Inf, 'msg', []);
    st.front = struct('count', 0, 't', -Inf, 'msg', []);
    st.back  = struct('count', 0, 't', -Inf, 'msg', []);
    st.gt    = struct('count', 0, 't', -Inf, 'msg', []);

    try
        if isempty(opts.DomainID)
            node = ros2node(opts.NodeName);
        else
            node = ros2node(opts.NodeName, opts.DomainID);
        end
        pubCmd  = ros2publisher(node, opts.TopicCmdVel, 'geometry_msgs/Twist');
        subOdom = ros2subscriber(node, opts.TopicOdom, 'nav_msgs/Odometry', @(~,m) onMsg('odom', m)); %#ok<NASGU>
        subF    = ros2subscriber(node, opts.TopicScanFront, 'sensor_msgs/LaserScan', @(~,m) onMsg('front', m)); %#ok<NASGU>
        subB    = ros2subscriber(node, opts.TopicScanBack, 'sensor_msgs/LaserScan', @(~,m) onMsg('back', m)); %#ok<NASGU>
    catch ME
        error('mir100_gazebo_link:rosUnavailable', ['Could not start ROS 2 node/topics: %s\n', ...
            'Check ROS 2 + DDS discovery and ROS_DOMAIN_ID, then retry.'], ME.message);
    end

    gtSubscribed = false; subGT = [];
    if opts.SubscribeGroundTruth
        try
            subGT = ros2subscriber(node, opts.TopicGroundTruth, 'tf2_msgs/TFMessage', @(~,m) onMsg('gt', m)); %#ok<NASGU>
            gtSubscribed = true; % subscriber CREATED; data presence gated per-reading
        catch
            gtSubscribed = false; % repo does not bridge /pose by default - honest flag
        end
    end

    link = struct();
    link.groundTruthSubscribed = gtSubscribed;
    link.limits = struct('maxLinearSpeed', opts.MaxLinearSpeed, 'maxAngularSpeed', opts.MaxAngularSpeed);
    link.topics = struct('cmdVel', opts.TopicCmdVel, 'odom', opts.TopicOdom, ...
        'scanFront', opts.TopicScanFront, 'scanBack', opts.TopicScanBack, ...
        'groundTruth', opts.TopicGroundTruth);
    link.sendCmdVel = @sendCmdVel;
    link.stop = @stopRobot;
    link.latestScan = @latestScan;
    link.latestOdom = @latestOdom;
    link.latestGroundTruth = @latestGroundTruth;
    link.isFresh = @isFresh;
    link.close = @closeLink;

    function onMsg(kind, msg)
        s = st.(kind);
        s.count = s.count + 1; s.t = toc(t0); s.msg = msg;
        st.(kind) = s;
    end

    function sendCmdVel(v, w)
        v = max(-link.limits.maxLinearSpeed, min(link.limits.maxLinearSpeed, double(v)));
        w = max(-link.limits.maxAngularSpeed, min(link.limits.maxAngularSpeed, double(w)));
        msg = ros2message(pubCmd);
        msg.linear.x = v; msg.linear.y = 0; msg.linear.z = 0;
        msg.angular.x = 0; msg.angular.y = 0; msg.angular.z = w;
        send(pubCmd, msg);
    end

    function stopRobot()
        msg = ros2message(pubCmd); % default zeros
        send(pubCmd, msg);
    end

    function s = latestScan(which)
        switch lower(which)
            case 'front', r = st.front; mount = opts.MountFront;
            case 'back',  r = st.back;  mount = opts.MountBack;
            otherwise, error('mir100_gazebo_link:badScan', 'which must be ''front'' or ''back''.');
        end
        s = struct('fresh', r.count > 0, 'ageSec', age(r), 'count', r.count, ...
            'ranges', [], 'angles', [], 'xBase', [], 'yBase', [], ...
            'bearingBase', [], 'nanCount', 0, 'frameId', '');
        if r.count > 0
            m = r.msg;
            s.ranges = double(m.ranges(:)).';
            s.angles = (m.angle_min + (0:numel(s.ranges)-1) .* m.angle_increment);
            s.frameId = char(m.header.frame_id);
            s.nanCount = nnz(isnan(s.ranges));
            % Base-frame transform: hit = mount_xy + r*[cos(a+yaw_m), sin(a+yaw_m)].
            % Finite ranges only; Inf = no return within max range (kept
            % for bearing coverage, excluded from hit points).
            fin = isfinite(s.ranges);
            rayDir = s.angles(fin) + mount(3);
            s.xBase = mount(1) + s.ranges(fin) .* cos(rayDir);
            s.yBase = mount(2) + s.ranges(fin) .* sin(rayDir);
            s.bearingBase = s.angles + mount(3); % ray directions in base frame (coverage)
        end
    end

    function o = latestOdom()
        r = st.odom;
        o = struct('fresh', r.count > 0, 'ageSec', age(r), 'count', r.count, ...
            'x', NaN, 'y', NaN, 'yaw', NaN, 'v', NaN, 'w', NaN, 'frameId', 'odom');
        if r.count > 0
            m = r.msg;
            o.x = m.pose.pose.position.x;
            o.y = m.pose.pose.position.y;
            q = m.pose.pose.orientation;
            o.yaw = atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y^2 + q.z^2));
            o.v = m.twist.twist.linear.x;
            o.w = m.twist.twist.angular.z;
            o.frameId = char(m.header.frame_id);
        end
    end

    function g = latestGroundTruth()
        r = st.gt;
        g = struct('subscribed', gtSubscribed, 'fresh', r.count > 0, 'found', false, ...
            'ageSec', age(r), 'x', NaN, 'y', NaN, 'yaw', NaN, 'source', '');
        if ~gtSubscribed || r.count == 0, return; end
        tr = r.msg.transforms; % TransformStamped array (gz PosePublisher link poses)
        for k = 1:numel(tr)
            cid = char(tr(k).child_frame_id);
            if contains(cid, 'base_footprint') || contains(cid, 'base_link')
                g.x = tr(k).transform.translation.x;
                g.y = tr(k).transform.translation.y;
                q = tr(k).transform.rotation;
                g.yaw = atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y^2 + q.z^2));
                g.found = true; g.source = cid;
                return;
            end
        end
    end

    function a = age(r)
        if r.count > 0, a = toc(t0) - r.t; else, a = Inf; end
    end

    function tf = isFresh(kind, maxAgeSec)
        switch lower(kind)
            case 'odom', r = st.odom;
            case 'front', r = st.front;
            case 'back', r = st.back;
            case 'groundtruth', r = st.gt;
            otherwise, error('mir100_gazebo_link:badKind', 'unknown freshness kind %s.', kind);
        end
        tf = r.count > 0 && age(r) <= maxAgeSec;
    end

    function closeLink()
        try, stopRobot(); catch, end
        clear('pubCmd', 'subOdom', 'subF', 'subB', 'subGT', 'node');
    end
end
