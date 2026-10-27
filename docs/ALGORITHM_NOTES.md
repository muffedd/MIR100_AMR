# ALGORITHM_NOTES.md - planner/controller selection for the MiR100 AMR project
(module: matlab/roslink - docs/version grounding. All source URLs verified live 2026-10-27.
No MATLAB execution. Planner-fit statements are engineering judgment for OUR map/robot,
NOT benchmarks. Rev B: prior overclaims corrected - see change notes inline.)

OUR PARAMETERS (the selection basis)
- Robot: MiR100, differential drive - wheel radius 0.0625 m, separation 0.445208 m;
  can rotate in place (v=0, w!=0). Non-circular 8-vertex footprint polygon,
  circumradius 0.6052 m, width 0.709 m at 90 deg heading.
- World: 65 static walls, fully known; binary occupancy grid 0.05 m (216x276 cells).
  Unseen obstacles only via lidar (scan 12.5 Hz in Gazebo).
- Baseline: MATLAB R2022a desktop-portable; MATLAB Online runs the CURRENT release.
- Safety DESIGN INTENT: certified full-footprint clearance before every move and
  stop-with-zero-contact on infeasibility. HONEST STATUS: the audit found the
  current implementation checks endpoints and the lattice rotation sweeps are NOT
  certified - full swept certification is an open repair item. Zero-contact so far
  is a Python-mirror result, NOT a guarantee. Speed/physics unvalidated; MATLAB
  NOT RUN.

ROLES ARE SEPARATE: global planner, local detour planner, and path tracker are
complements, not alternatives to each other.

1. GLOBAL PLANNER (known static world) - CHOICE: grid A*
- plannerAStarGrid (Navigation Toolbox, R2020b): "performs an A* search on an
  occupancy map and finds shortest obstacle-free path... as determined by
  heuristic cost." Fit rationale: map is already an occupancy grid, world fully
  known, deterministic/reproducible, cheap at 216x276 cells.
  https://www.mathworks.com/help/nav/ref/plannerastargrid.html
- Dijkstra: same path as A* with a zero heuristic, expands more cells; no output
  difference for one start-goal pair (standard property of A* with an admissible
  heuristic; judgment, not benchmarked here).
- RRT (plannerRRT, R2019b) / RRT* (plannerRRTStar, R2019b) / mobileRobotPRM
  (R2019b): sampling-based methods designed for continuous/high-dimensional or
  multi-query settings. ENGINEERING JUDGMENT (not benchmarked): on a small,
  fully known 2-D grid we expect no benefit over grid A*, and sampling adds
  run-to-run randomness plus post-smoothing work. That is a fit argument for
  this map, not a general claim about these planners.
  https://www.mathworks.com/help/nav/ref/plannerrrt.html
  https://www.mathworks.com/help/nav/ref/plannerrrtstar.html
  https://www.mathworks.com/help/robotics/ref/mobilerobotprm.html
- plannerHybridAStar: for vehicles with a minimum turning radius (car-like); the
  MiR100 turns in place, so its machinery is unneeded here.
- D* Lite: NOT a shipped MathWorks planner (absent from the official function
  list; community implementations only) - adopting it means unverified
  hand-written code. plannerAStar is R2023a, plannerMPNET R2024a - above baseline.
  https://www.mathworks.com/help/nav/motion-planning.html (official function list)

2. LOCAL DETOUR PLANNER (unseen lidar obstacles) - CHOICE: SE(2) lattice
- Heading-aware (8 headings x 0.1 m) lattice with exact footprint-polygon masks.
  Rationale: the footprint is non-circular (0.6052 m radius vs 0.709 m width at
  90 deg), so heading-blind circular inflation over-blocks passages while
  point-robot/shrunk-radius planning is unsafe (project rule). Fail-closed
  INTENT: no certified path => stop. AUDIT CAVEAT: rotation sweeps in the
  lattice are not yet certified (endpoint checks); treat zero-contact outcomes
  as mirror evidence, not proof.
- controllerVFH (Vector Field Histogram): shipped reactive alternative, simpler,
  but provides no full-polygon clearance certification. Documented alternative,
  not chosen. https://www.mathworks.com/help/nav/motion-planning.html

3. PATH TRACKER (controller) - CHOICE UNDER REVIEW: rotate-then-drive
- Replanned paths are piecewise straight at 45 deg multiples; rotate-then-drive
  tracks those segments exactly and cannot cut corners. Matches differential-drive
  kinematics (in-place rotation).
  https://www.mathworks.com/help/robotics/ref/differentialdrivekinematics.html
- controllerPurePursuit (R2019b) REJECTED BY OUR TEST: official doc says a higher
  lookahead "can cause the vehicle to cut corners along the path"; recorded
  outcome: corner-cutting into the checked boundary made the footprint check
  refuse moves -> STUCK at the shipped demo box (v2 polygon).
  https://www.mathworks.com/help/robotics/ref/controllerpurepursuit-system-object.html
- DWA: not a shipped MathWorks controller (absent from the official
  path-following list); a hand-rolled DWA would be unverified code.
- differentialDriveMPPIController: R2025a, Simulink block (Offroad Autonomy
  Library) - above baseline, Simulink-dependent. Out.
  https://www.mathworks.com/help/robotics/ref/differentialdrivemppicontroller.html
- MPC path following: needs Model Predictive Control Toolbox (extra product).
  https://www.mathworks.com/help/robotics/ug/create-path-following-model-predictive-controller.html
- controllerTEB: see section 4 - it is the CANDIDATE for the high-speed request,
  subject to the constraints below. It is not chosen yet and not proven here.

4. HIGH-SPEED REQUEST - exact controllerTEB facts (official page, verified
2026-10-27: https://www.mathworks.com/help/nav/ref/controllerteb.html)
ACCESS:
- Object introduced R2023a; MinTurningRadius, MaxReverseVelocity, GoalTolerance
  are R2023b+ only. Above the R2022a desktop baseline.
- On MATLAB Online the CURRENT release is served, so the TEB API is
  release-eligible there IF the user's license includes Navigation Toolbox. The
  Anna University license's product list is UNCONFIRMED (no `ver` output yet);
  access is not established until the user signs in and checks.
FOOTPRINT API (material constraint):
- RobotInformation accepts Shape "Rectangle" or "Point" ONLY - it cannot ingest
  our exact 8-vertex polygon. Options: an enclosing rectangle (more conservative
  at some corners) or Point (unsafe, violates project rule). Exact-polygon TEB
  is not possible with the shipped API.
SPEED/ACCEL INPUTS:
- MaxVelocity [lin ang] default [0.8 1.6]; MaxAcceleration [lin ang] default
  [2.4 4.8]. These are DECLARED limits the caller must set from measured robot
  capability; the defaults are NOT MiR100 measurements and we have no measured
  values yet.
CONTROL RATE / RUNTIME:
- No fixed-rate property. step() computes velocity commands and an optimized
  trajectory over LookAheadTime (default 5 s); longer look-ahead = earlier
  reaction but slower controller; NumIteration (default 2) and map size also
  cost time. Actual achievable loop rate must be measured on the user's machine;
  nothing here establishes a real-time rate.
SAFETY SEMANTICS (material):
- ObstacleSafetyMargin (default 0.5 m) is "a soft constraint that the planner
  may ignore" (verbatim). TEB optimizes a cost; it does NOT certify clearance.
  The project's external full-footprint check and fail-closed stop would still
  be required around it.
WHAT WE CAN IMPLEMENT WITHOUT INVENTING SPEED:
- Keep A* (global) + rotate-then-drive with clearance-scheduled speed at the
  current conservative declared limits - shippable within R2022a, no new claims.
- Optionally wire TEB behind the same contract with placeholder limits marked
  MUST-MEASURE, keeping the external footprint check - gated OFF until (1) the
  user's license actually shows Navigation Toolbox, (2) deceleration and max
  speeds are measured (spec sheet or a Gazebo decel run), (3) first real MATLAB
  execution passes. Any quoted speed number before that route would be invented.

CLOUD-ROUTE NOTE (ties to VERSION_MATRIX F3): on MATLAB Online's current
release, controllerPurePursuit's second output is CURVATURE (R2026a rename),
not angular velocity - the project's own tracker avoids that dependency.
