# Custom navigation goals and diagnostics

Goal: make a chosen world-frame goal, a fresh plan and navigation graphs part
of the normal MATLAB entry point, preserving the robot, walls and safety gates.

1. Extend run_all compatibly: numeric first argument means goal [x y]; a
   desktop call without a goal prompts for coordinates. Keep root-folder calls.
2. Always call the planner. On no path, command no motion and report the exact
   reason. Never substitute the old diagnostic waypoints for a requested goal.
3. Keep the canonical T1-T6 battery, labelled separately from the chosen-goal run.
4. Save each run under its own directory. Add actual path/position/error/speed/
   heading/clearance graphs, numeric CSV and results. Label A* and pure pursuit
   and distinguish tracking error from distance to goal. Core has no lidar.
5. Show the chosen goal and fresh path in the existing 3D replay.
6. Verify in MATLAB: custom feasible goal end to end, blocked goal without any
   motion, repeated-run freshness, and graph values against known geometry.

No new dependencies, speed increases, footprint changes or computer-use tools.
