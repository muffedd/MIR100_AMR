matlab/demo - demo rendering module (module author-owned)  [rev3, integration repair]
=============================================================================
Owner of this directory only. The project owns the root entry point.

FILES
  make_demo.m                     Demo renderer (this module). rev3 - MISS_HIT-linted.
  reference_render_NOT_MATLAB.py  Python reference check of world geometry
                                  and route_v2 (evidence aid only).
  demo_reference_NOT_MATLAB.png   PYTHON MATPLOTLIB RENDER - NOT MATLAB
                                  OUTPUT. Geometry check only; never
                                  present as the MATLAB demo.
  README.txt                      This file.
  REPORT_SKELETON.txt             Truthful report skeleton for the project.

WHAT make_demo.m DOES (in order)
  1. boxes   = read_world_boxes(csvPath)          (contract; world module)
  2. scen    = build_world(csvPath)               (contract; world module)
  3. robot   = load_mir100(urdfPath)              (contract; robot module)
  4. plat    = spawn_mir100(scen, robot)          (contract; robot module)
  5. trajLog = follow_route(plat, wpsWorld, simOpts)
     simOpts is FLAT (control-module canonical): dt, maxLinVel, maxAngVel,
     wpRadius, goalRadius, maxSimTime, scenario. The scenario handle is
     passed through as simOpts.scenario = scen. No nested opts struct
     reaches follow_route. trajLog fields used: t, x, y, yaw, and
     wheelAngles(:,[left right]) which drive the 3D wheel-joint
     animation (name-mapped onto the URDF joints).
  6. clearLog = check_run_clearance(trajLog, boxes)   (optional overlay)
     CANONICAL schema: minOverall, minDist, t, minAt, collision.
     Demo reads minOverall (falls back to min(minDist)), the collision
     flag, and records minAt/t in the manifest. Never invents a number.
  7. PRIMARY RENDER - 3D scene: the ACTUAL imported rigidBodyTree (visual
     meshes) animated INSIDE the actual world boxes:
       scenario = robotScenario(UpdateRate=1/dt)
       addMesh(scenario,"Plane",...) for the floor
       addMesh(scenario,"Box",Size=[sx sy sz],Position=[xc yc zc]) per wall
         (all 65 hard_world boxes have yaw=0, verified; the yaw~=0 path
          builds a "Custom" 8-vertex/12-triangle mesh, currently unexercised)
       platform = robotPlatform("mir100",scenario,RigidBodyTree=robot,
                                ReferenceFrame="ENU")
       updateMesh(platform,"RigidBodyTree",Object=robot)  (guarded)
       setup(scenario); ax = show3D(scenario)
       per executed pose k:
         motion16 = [x y baseZ, 0 0 0, 0 0 0, qw 0 0 qz, 0 0 0]
         (quaternion w-first; baseZ is GROUNDED, not assumed - see below)
         move(platform,"base",motion16)
         cfg = name-mapped joint config from trajLog.wheelAngles(k,:)
           (joints 'left_wheel_joint'/'right_wheel_joint' per the xacro,
            override via opts.wheelJointNames; struct config first,
            vector config fallback, honest manifest note if unsupported)
         move(platform,"joint",cfg)
         show3D(scenario,FastUpdate=true,Parent=ax)
     Requires Robotics System Toolbox R2022a+ (robotScenario intro R2022a).
  8. SECONDARY RENDER - top-down 2D: boxes, planned route v2, executed
     trajectory, conservative footprint + heading, clearance/collision in
     the title. Base MATLAB only.
  9. Static rigidBodyTree proof stills (iso/side/top), guarded.

RUN
  From the integrated project root (all modules on path):
    manifest = make_demo();
    manifest = make_demo(csvPath, urdfPath, outDir, opts);
  Robot asset: REQUIRED and explicit. Pass the canonical flat URDF from
  the matlab/assets tree via the urdfPath argument or opts.urdfPath
  (project supplies it). make_demo ERRORS
  ('make_demo:noRobotAsset') when none is given - it never auto-picks a
  file.

DEPENDENCIES
  Base MATLAB:                2D rendering, VideoWriter, PNG.
  Robotics System Toolbox:    rigidBodyTree stills; R2022a+ adds
                              robotScenario for the 3D scene animation.
  Sibling modules on path:      read_world_boxes, build_world, load_mir100,
                              spawn_mir100, follow_route (required);
                              check_run_clearance (optional).
  Display:                    getframe video needs a desktop session.
                              HEADLESS RUNS PRODUCE NO VIDEO - keyframe
                              PNGs only, stated bluntly in the manifest.
                              Never present keyframes as the video demo.

OUTPUT (outDir, default matlab/demo/out)
  mir100_demo_3d.avi/.mj2       PRIMARY: robot meshes in world, lossless.
  mir100_demo_topdown.avi/.mj2  2D route/footprint animation, lossless.
  keyframe3d_*.png, keyframe2d_*.png   Lossless PNG keyframes.
  trajectory_overview.png       Planned vs executed route.
  robot_iso/side/top.png        Imported rigidBodyTree stills.
  demo_manifest.txt             What ran, versions, counts, caveats.
  Video profiles: 'Uncompressed AVI' (RGB24) or 'Archival' (lossless
  MJ2). MPEG-4 never used (not available on Linux). No user media is
  recompressed anywhere.

STATUS
  NOT YET EXECUTED IN MATLAB (no runtime available to this module author;
  cloud MATLAB Online access is being checked by the lead). Statically
  reviewed + MISS_HIT-checked: mh_lint passes with only 15 low-severity
  name-value-pair notes (that Name=Value syntax is exactly what the
  MathWorks docs prescribe); mh_style reports cosmetic indentation/
  naming conventions only. No parse errors.

DEMO-SIDE DEFAULTS (project may override via opts)
  wpsWorld  = route v2 world frame (map + [-4.31 -2.81]), 16 waypoints,
              spawn [-4.31 -2.81], goal [4.31 0.56].
  dt .05, maxLinVel .9, maxAngVel 1.5, wpRadius .15, goalRadius .05,
  maxSimTime 120 - the control module owns the authoritative values.
  footprint = v2 literal 8-vertex polygon (project, caster full
              coverage), footprintRadius 0.6052. v1 polygon removed.
  baseZ     = grounded in priority order: opts.baseZ override >
              plat.InitialBasePosition(3) (actual spawn height from
              spawn_mir100, preserved) > URDF-derived (home-config lowest
              body frame minus wheel radius). Never a bare constant.

WORLD FACTS (verified from the actual SDF @6450a77, this module author)
  65 wall boxes, all yaw = 0 today (contract supports yaw).
  Outer extent from box edges: x [-6.3755, 6.3755], y [-4.8755, 4.8755].
  Walls 2.2 m tall (zc 1.1, sz 2.2).

API SOURCES (MathWorks, fetched 2026-10-27)
  videowriter:   https://www.mathworks.com/help/matlab/ref/videowriter.html
  importrobot:   https://www.mathworks.com/help/robotics/ref/importrobot.html
  robotScenario: https://www.mathworks.com/help/robotics/ref/robotscenario.html
  robotPlatform: https://www.mathworks.com/help/robotics/ref/robotplatform.html
  move:          https://www.mathworks.com/help/robotics/ref/robotplatform.move.html
  addMesh:       https://www.mathworks.com/help/robotics/ref/robotscenario.addmesh.html
  show3D:        https://www.mathworks.com/help/robotics/ref/robotscenario.show3d.html
