# plan_path (module: matlab/planning) - revised
[path,info] = plan_path(map,startXY,goalXY,inflateM). path Nx2 world m; zeros(0,2)+info.reason on failure (no error thrown). Check info.ok.

## Official docs checked (read via fetch, not executed)
- checkOccupancy: https://www.mathworks.com/help/nav/ref/binaryoccupancymap.checkoccupancy.html
  xy world frame; 0 free, 1 occupied, -1 UNKNOWN INCLUDING OUTSIDE THE MAP; grid indices start at top-left; matrix forms are top-left row 1.
  DEFECT FIXED: v1 used (==1), which would treat unknown/outside as free. Now occupied = (status ~= 0).
- binaryOccupancyMap: https://www.mathworks.com/help/nav/ref/binaryoccupancymap.html
  Resolution = cells/m (read-only), GridSize=[rows cols], XWorldLimits/YWorldLimits=[min max], GridLocationInWorld = bottom-left corner. Pages also exist under /help/robotics/ref/ (Robotics System Toolbox); exact product/release requirement NOT verified.
- Orientation: I use point queries at cell centres (ascending y => row 1 = ymin in my internal matrix), so the top-left matrix convention of getOccupancy never matters. Struct input expects grid row 1 = ymin (bottom), origin = world coord of grid(1,1) bottom-left corner.
- Not checked: occupancyMap vs binaryOccupancyMap differences beyond the above; mobileRobotPRM/plannerRRT (not used).

## Method / safety
- Disk inflation (conv2), outside map = blocked, unknown = blocked, 8-connected A*, no corner cutting.
- Raster pad = sqrt(2) cells + 0.05 cell: a point anywhere in a free cell vs anywhere in an occupied cell can be closer than their centre distance by up to 2 half-diagonals = 1.414 cell (not 0.5). With this pad every accepted cell has true clearance >= inflateM from every occupied cell, PROVIDED the map raster is conservative (build_map/check_map must guarantee boxes are fully covered).
- Line-of-sight shortcut: samples every 0.05 cell, requires every touched cell free and both corner-neighbour cells free on diagonal steps (no corner cutting). 0.05-cell sampling gap is covered by the pad. Not an exact supercover but certified by the pad. Start/goal snapped to exact points (within start/goal cell, covered by pad).
- Fails closed: never shrinks footprint; every planner (A*, PRM, RRT) should fail closed and the trajectory still needs check_run_clearance.
- Disk inflation cannot use clearance narrower than 2*r. Earlier notes claimed the shipped world's narrowest corridor is 1.125 m and would BLOCK the planner; a real Python run of build_map + plan_path on the actual 65-box world at 0.6052 inflation REFUTED that: a 15.20 m path exists (spawn to goal). The 1.125 m figure came from a synthetic probe, not the actual world. No blocking is expected in the shipped world; if a path IS empty, that is a true finding to report, not an expectation.

## Evidence
NOT executed in MATLAB/Octave (neither present). Static review only (block balance checked by hand). Python port of the same algorithm (ref_check.py): synthetic two-wall corridor probe - gap 1.125 m blocked at r=0.5988/0.60, passes at r=0.5; gap 1.30 m BLOCKED at r=0.5988/0.60 because of the raster pad (needs gap > 2*(r+0.07)=~1.34 m at 20 cells/m), passes at r=0.5. Choose a finer resolution to reduce pad (pad = 1.46/res). REAL-world check (not synthetic): build_map + plan_path replicated in Python on the shipped 65-box world at 0.6052 inflation, res 0.05, auto origin - PATH FOUND, 15.20 m, 277 waypoints. The planned path then followed with control's Python pure-pursuit ref (277/277 wps, 0.099 m final error) with continuous polygon clearance lower bound +167.5 mm - collision-free. Evidence class: Python replication, not MATLAB runtime.
Edge cases handled: invalid/NaN input, start/goal out of bounds, inside obstacle, inside inflation, unreachable, unknown cells.
