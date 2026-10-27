# Integration review checklist (per module author return)
1. Files land ONLY in the module's own matlab/<module>/ directory.
2. Function signatures match INTEGRATION_CONTRACT exactly (names,
   arg order, units: meters/radians, 0.05 m per map cell).
3. Hard facts unchanged: wheel 0.0625/0.4452, radius 0.5988, mass
   69.239, footprint polygon vertices, 65 boxes, route v2 + offset.
4. World frame math: map + (-4.31,-2.81); spawn (-4.31,-2.81),
   goal (4.31,0.56). No frame confusion in code or comments.
5. Planner inflation 0.60 NOT shrunk even if corridor blocks -
   a blocked plan is reported, not engineered away.
6. Collision checks use the actual footprint polygon, not a point
   or a bare circle.
7. Every used API has an "Introduced in" comment; anything post-
   R2022a is flagged for the version matrix.
8. No claim that any file was executed in MATLAB. Static evidence
   (python cross-checks, parse checks) is quoted with numbers.
9. No new dependencies beyond Robotics System Toolbox + Navigation
   Toolbox (except module 11 stubs, clearly marked NOT-BUILT).
10. Report-back states: complete / flagged / failed + what remains
    for real MATLAB execution.
11. Planner fail-closed: empty path = reported BLOCKED finding;
    follow_route on predeclared waypoints is labeled
    diagnosticOnly=true and never counted as a planner pass.
12. Failed checks stay visible in the handover (user instruction
    05:04 IST); nothing removed to meet the deadline.
13. Canonical CSV = assets/hard_world_boxes.csv (8 cols, id header).
    read_world_boxes accepts id|name (project edit); ALL consumers
    read CSVs only through read_world_boxes - no raw readmatrix paths.
14. build_map res argument = metres/cell (0.05), root passes 0.05.
    Map passed between modules ONLY as the occupancyMap toolbox object.
15. Planner re-spin (module author): diagonal quantisation pad sqrt(2)/2
    cells (not 0.5), supercover LOS (point sampling can miss corner
    cells), drop the "randomized planners can't fail closed" claim;
    verify occupancyMap AND binaryOccupancyMap API branches.
