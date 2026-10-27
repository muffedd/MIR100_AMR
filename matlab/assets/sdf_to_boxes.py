#!/usr/bin/env python3
"""Convert hard_world.sdf wall boxes to hard_world_boxes.csv.
Columns: id,xc,yc,zc,sx,sy,sz,yaw  (metres, radians; world frame as in SDF).
Ground plane (a <plane> geometry) and observer_cam (no collision) are excluded by identity: only
<model> with a <collision><geometry><box> is kept. Usage: sdf_to_boxes.py [in.sdf] [out.csv]"""
import sys, xml.etree.ElementTree as ET
src = sys.argv[1] if len(sys.argv)>1 else 'mir_description/worlds/hard_world.sdf'
dst = sys.argv[2] if len(sys.argv)>2 else 'hard_world_boxes.csv'
rows=[]
for m in ET.parse(src).getroot().iter('model'):
    b = m.find('.//collision/geometry/box/size')
    if b is None: continue
    x,y,z,r,p,yw = map(float,(m.findtext('pose') or '0 0 0 0 0 0').split())
    assert abs(r)<1e-9 and abs(p)<1e-9, 'non yaw-only pose'
    sx,sy,sz = map(float,b.text.split())
    rows.append((m.get('name'),x,y,z,sx,sy,sz,yw))
with open(dst,'w') as f:
    f.write('id,xc,yc,zc,sx,sy,sz,yaw\n')
    for r in rows: f.write('%s,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.6f\n'%r)
print(len(rows),'boxes ->',dst)
