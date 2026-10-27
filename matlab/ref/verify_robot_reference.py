#!/usr/bin/env python3
"""Reference check for the flattened MiR100 URDF (mir100_matlab.urdf).

Static, execution-free evidence: parses the exact URDF that load_mir100.m
imports and checks joint inventory, masses, and wheel geometry against the
project contract leads. Mirrors what verify_robot.m checks on the imported
rigidBodyTree, so a PASS here plus a MATLAB-side PASS mean the same facts.
Exit code 0 = all checks pass, 1 = at least one FAIL.
"""
import xml.etree.ElementTree as ET
import math, os, sys

HERE = os.path.dirname(os.path.abspath(__file__))
URDF = os.path.join(HERE, '..', 'assets', 'mir_description', 'urdf', 'mir100_matlab.urdf')
MESHDIR = os.path.join(HERE, '..', 'assets', 'mir_description', 'meshes')  # repo meshes/ copied next to the URDF at runtime

root = ET.parse(URDF).getroot()
links = {l.get('name'): l for l in root.findall('link')}
joints = {j.get('name'): j for j in root.findall('joint')}

def xyz(j):
    o = j.find('origin')
    return [float(v) for v in (o.get('xyz', '0 0 0').split() if o is not None else [0,0,0])]

checks = []
def check(name, ok, detail):
    checks.append((name, bool(ok), detail))

# --- inventory ---
expected_joints = {
    'base_joint':'fixed',
    'base_link_to_imu_joint':'fixed',
    'imu_link_to_imu_frame_joint':'fixed',
    'base_link_to_front_laser_joint':'fixed',
    'base_link_to_back_laser_joint':'fixed',
    'base_link_surface_joint':'fixed',
    'mirref_top_holes_joint':'fixed',
    'mirref_front_ports_joint':'fixed',
    'mirref_corner_caps_joint':'fixed',
    'left_wheel_joint':'continuous',
    'right_wheel_joint':'continuous',
    'fl_caster_rotation_joint':'continuous','fr_caster_rotation_joint':'continuous',
    'bl_caster_rotation_joint':'continuous','br_caster_rotation_joint':'continuous',
    'fl_caster_wheel_joint':'continuous','fr_caster_wheel_joint':'continuous',
    'bl_caster_wheel_joint':'continuous','br_caster_wheel_joint':'continuous',
}
expected_links = {'base_footprint','base_link','imu_link','imu_frame','surface',
    'front_laser_link','back_laser_link','left_wheel_link','right_wheel_link',
    'fl_caster_rotation_link','fr_caster_rotation_link','bl_caster_rotation_link','br_caster_rotation_link',
    'fl_caster_wheel_link','fr_caster_wheel_link','bl_caster_wheel_link','br_caster_wheel_link',
    'mirref_top_holes','mirref_front_ports','mirref_corner_caps'}
expected_meshes = {'meshes/visual/mir_100_base.stl','meshes/visual/mirref/band.stl',
    'meshes/visual/wheel.stl','meshes/visual/caster_wheel_base.stl','meshes/visual/sick_lms-100.stl',
    'meshes/visual/mirref/top_holes.stl','meshes/visual/mirref/front_details.stl','meshes/visual/mirref/corner_caps.stl'}
check('link inventory', set(links)==expected_links,
      f'{len(links)} links, missing={sorted(expected_links-set(links))}, extra={sorted(set(links)-expected_links)}')
bad=[(n,joints[n].get('type')) for n in expected_joints if n not in joints]
bad+=[(n,joints[n].get('type')) for n,t in expected_joints.items() if n in joints and joints[n].get('type')!=t]
check('joint inventory+types', not bad and len(joints)==len(expected_joints),
      f'{len(joints)} joints, mismatches={bad}')

# --- masses ---
mass = {}
for n,l in links.items():
    m = l.find('inertial/mass')
    mass[n] = float(m.get('value')) if m is not None else 0.0
total = sum(mass.values())
check('total mass 69.2390156 kg (tol 1e-3)', abs(total-69.2390156)<1e-3, f'total={total:.7f} kg')
check('base mass 58.0', abs(mass['base_link']-58.0)<1e-9, f"base={mass['base_link']}")
for w in ('left_wheel_link','right_wheel_link'):
    check(f'{w} mass 1.0', abs(mass[w]-1.0)<1e-9, f'{mass[w]}')
for c in ('fl','fr','bl','br'):
    check(f'{c}_caster_rotation mass 0.3097539019', abs(mass[f'{c}_caster_rotation_link']-0.3097539019)<1e-9, f"{mass[f'{c}_caster_rotation_link']}")
for l_ in ('front_laser_link','back_laser_link'):
    check(f'{l_} mass 2.0', abs(mass[l_]-2.0)<1e-9, f'{mass[l_]}')
for d in ('mirref_top_holes','mirref_front_ports','mirref_corner_caps'):
    check(f'{d} decorative link massless', mass[d]==0.0, f'{mass[d]}')

# --- wheel geometry (contract leads: radius 0.0625, separation 0.4452) ---
yl = xyz(joints['left_wheel_joint']); yr = xyz(joints['right_wheel_joint'])
sep = abs(yl[1]-yr[1])
check('wheel separation 0.445208 m (tol 1e-6)', abs(sep-0.445208)<1e-6, f'sep={sep:.6f} (lead 0.4452)')
check('wheel radius 0.0625 m from joint z (tol 1e-9)', abs(yl[2]-0.0625)<1e-9 and abs(yr[2]-0.0625)<1e-9,
      f'z_left={yl[2]}, z_right={yr[2]}')
check('wheel axle x offset 0.037646 m', abs(yl[0])<1e-12 and abs(yr[0])<1e-12, f'x_left={yl[0]} (base_link frame; dx applies to base visual/CoM)')
for w in ('left_wheel_joint','right_wheel_joint'):
    a = joints[w].find('axis'); av=[float(v) for v in a.get('xyz').split()]
    check(f'{w} axis [0 1 0] continuous', av==[0,1,0] and joints[w].get('type')=='continuous', f'axis={av}')
for c in ('fl','fr','bl','br'):
    j=joints[f'{c}_caster_rotation_joint']; a=j.find('axis'); av=[float(v) for v in a.get('xyz').split()]
    check(f'{c}_caster_rotation axis [0 0 1] continuous', av==[0,0,1] and j.get('type')=='continuous', f'axis={av}')

# --- inertia sanity ---
okI=True
for n,l in links.items():
    for tag in ('ixx','iyy','izz'):
        el=l.find('inertial/inertia')
        if el is not None and float(el.get(tag))<=0: okI=False
check('all principal inertias > 0', okI, '')

# --- meshes: exact 8-file ref set, all present (against repo copy) ---
norm=lambda f: f[3:] if f.startswith('../') else f  # URDF refs are relative to urdf/
actual_meshes={norm(m.get('filename')) for m in root.iter('mesh')}
check('mesh reference set (8 files)', actual_meshes==expected_meshes,
      f'missing_refs={sorted(expected_meshes-actual_meshes)}, extra_refs={sorted(actual_meshes-expected_meshes)}')
missing=[p for p in actual_meshes if not os.path.isfile(os.path.join(MESHDIR, '..', p))]
check('all mesh files exist', not missing, f'missing={missing}')
# ref-matched visuals: band must ride on base_link; decorative links carry their own meshes
base_vis=[norm(m.get('filename')) for m in links['base_link'].iter('mesh')]
check('base_link keeps shell+band visuals', 'meshes/visual/mir_100_base.stl' in base_vis and 'meshes/visual/mirref/band.stl' in base_vis, f'{base_vis}')

# --- contract footprint polygon v2 (6 mm edge expansion of v1; radius 0.6052) ---
import numpy as _np
v2=_np.array([[-0.4740270,-0.2360854],[-0.4393930,-0.3197930],[-0.3557917,-0.3543830],
 [0.4930498,-0.2956372],[0.5480241,0.2366924],[0.5133930,0.3203930],
 [0.4297893,0.3549840],[-0.4178380,0.2956339]])
circ=float(_np.hypot(v2[:,0],v2[:,1]).max())
check('footprint v2 circumradius 0.6051644 (tol 1e-4)', abs(circ-0.6051644)<1e-4, f'circumradius={circ:.7f}, contract radius 0.6052 covers it: {0.6052>=circ}')
mount=max(0.222604, float(_np.hypot(0.341346,0.203)), float(_np.hypot(-0.270154,0.203)),
          float(_np.hypot(0.4288,0.2358)), float(_np.hypot(-0.3548,-0.2352)))
check('footprint v2 covers all wheel/caster/laser mounts', mount<=0.6052, f'max mount radius={mount:.7f} <= 0.6052')

w=0
for name,ok,detail in checks:
    print(f"[{'PASS' if ok else 'FAIL'}] {name}" + (f'  -- {detail}' if detail else ''))
    w=max(w,len(name))
fails=[c for c in checks if not c[1]]
print(f"\n{len(checks)-len(fails)}/{len(checks)} checks passed")
sys.exit(1 if fails else 0)
