# matlab/world
Source: muffedd/ROS_mir100 @ 6450a77, src/mir_description/worlds/hard_world.sdf.
Files: read_world_boxes.m, build_world.m, hard_world_boxes.csv (65 walls), check_world_static.py.
- boxes = Nx7 [xc yc zc sx sy sz yaw] (m, rad); all yaw=0, all height 2.2, zc=1.1.
- build_world returns [scen, boxes, wallNames]; ground "ground" plane has IsBinaryOccupied=false, walls true, Collision="mesh".
- NOT run in MATLAB (no MATLAB/Octave in environment). API per mathworks.com/help/robotics/ref/robotscenario.addmesh.html (R2022a+, Name/Position/Size/Color/Collision/IsBinaryOccupied; Box has no orientation).
- Unverified: Collision="mesh" on Custom meshes; collisionBox bridge was not needed (Collision="mesh" on a Box fits a collision box per doc). Wall names w0..w64 stay unique.
- Plane ground: 30x30 as in SDF. Walls are inside world frame as-is (no map offset).
