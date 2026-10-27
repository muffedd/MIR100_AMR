# matlab/control — follow_route.m (module author module deliverable)

## Contract
    trajLog = follow_route(plat, wpsWorld, opts)
- `plat`: rigidBodyTree-based `robotPlatform` from `spawn_mir100`.
- `wpsWorld`: Nx2 or Nx3 world-frame waypoints (metres). All N must be reached
  in sequence within acceptance radii — completion is NOT final-pose-only.
- `opts`: struct; every field optional (defaults in file header): dt(0.01),
  lookahead(0.6), maxLinVel(1.0), maxAngVel(2.0), wpRadius(0.15),
  goalRadius(0.10), maxSimTime(600, canonical; maxTime accepted as alias),
  maxSteps(1.5*maxSimTime/dt), wheelRadius(0.0625), wheelSep(0.4452),
  logStride(1), scenario([]).
- Output `trajLog` LOCKED SCHEMA fields: `t, x, y, yaw` (Mx1),
  `wheelAngles` (Mx2 [left right]), `waypoints` (route as commanded),
  `reachedIdx` (indices of reached waypoints, in order),
  `diagnosticOnly` (true), `plannerStatus` ('success'|'timeout'|'maxsteps').
  Extras retained: `visited` (Nx1 logical), `allWaypointsReached`, `status`
  (alias of plannerStatus), `reason`, `finalError`, `wheelsCommanded`,
  `wheelJointNames`, `opts`.

## Verified APIs (MathWorks docs, checked 2026-10-27)
- `move(plat,"base",motion16)`: [x y z vx vy vz ax ay az qw qx qy qz wx wy wz],
  quaternion [qw qx qy qz]. https://www.mathworks.com/help/robotics/ref/robotplatform.move.html
- `move(plat,"joint",config)`: full rigidBodyTree joint vector (same order as
  homeConfiguration). Same page.
- `read(plat,"base")` -> 16-element motion, same ordering.
  https://www.mathworks.com/help/robotics/ref/robotplatform.read.html
- robotScenario/robotPlatform introduced R2022a (Robotics System Toolbox).
- Wheel joints in the MiR100 xacro (prefix=""): `left_wheel_joint`,
  `right_wheel_joint` (continuous); resolved by exact name with regex
  fallback `(^|_)left_wheel_joint$` — casters (`*_caster_wheel_joint`) cannot
  collide with that pattern. Wheel radius 0.0625 and separation
  2*0.222604 = 0.4452 confirmed from mir_100_v1.urdf.xacro.

## Design notes for the project
- Base z is held at the spawned height from read(plat,"base"); x/y/yaw are
  integrated kinematically (unicycle) and commanded as absolute base motion
  each step — no trajectory is assigned, so scenario time only advances if
  you pass `opts.scenario` (it is advance()d per step; dt then defaults to
  1/scenario.UpdateRate). follow_route's signature has no scenario arg, so
  this is the only way to keep the scenario clock in step.
- Pure pursuit with strict sequential waypoint gating: a waypoint must come
  within wpRadius (goalRadius for the last) before the next becomes active.
  Two safeguards enforce this: lookahead clamped to distance-to-active
  (lookahead falls back to the active waypoint, never the final one), and a
  "missed waypoint" home-back when the robot's projection passes an
  unvisited waypoint beyond its radius. Rotate-in-place when the target is
  behind (|alpha| > pi/2).
- Wheel angles are integrated from (v,w) via the diff-drive model and
  applied to both wheel joints through move(plat,"joint",...); other joints
  (casters) stay at home configuration.
- KINEMATIC DEMONSTRATION ONLY: no dynamics, slip, or collision checking.
  Not physical validation of the MiR100 and no real-time guarantee.

## Verification evidence (NO MATLAB EXECUTION CLAIMED)
- MATLAB/Octave are not installed in this environment; no MATLAB run claimed.
- Static check (custom): bracket balance outside strings/comments,
  function/end accounting (8 functions, 37 end tokens = 8 + 29 blocks),
  contract fields and API call presence — PASSED (static_check logic embedded
  in follow_route_ref_check.py companion; re-runnable).
- Python reference port of the identical algorithm (follow_route_ref_check.py):
  route v2 world frame (spawn [-4.31,-2.81], goal [4.31,0.56]):
  ALL CHECKS PASSED — status success, all 16 waypoints visited within radii,
  final error 0.0997 m (< 0.10), tEnd 21.93 s (< 120), no NaN, monotone t,
  max per-waypoint approach 0.0997 m. Plot: follow_route_ref.png.
- Robustness suite (same reference port): spawn yaw pi/2 and pi, +1 m lateral
  offset, -0.5 m behind start, Nx3 input, dt=0.02/lookahead=0.4/v=1.2,
  tight radii 0.08/0.05, v=0.3 — 8/8 PASSED.
- Two real bugs were caught and fixed by the reference check before delivery
  (lookahead fallback targeting final waypoint; missed-waypoint non-return).
