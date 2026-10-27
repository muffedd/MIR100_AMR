#!/usr/bin/env python3
"""Executed Python reference, NOT execution/translation of MATLAB code.
Uses numpy/scipy for STL-envelope and independent convex SAT cross-checks.
Run: python3 reference_collision.py --repo /path/to/ROS_mir100 --output results.json
"""
import argparse, json, math, struct
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial import ConvexHull
ORIGINAL_FP = np.array([[-.4679,-.2352],[-.4348,-.3152],[-.3548,-.3483],
                        [.4876,-.29],[.5419,.2358],[.5088,.3158],[.4288,.3489],[-.4124,.29]])
FP = np.array([[-.474026970,-.236085370],[-.439392950,-.319792950],
               [-.355791680,-.354382980],[.493049750,-.295637190],
               [.548024070,.236692380],[.513392950,.320392950],
               [.429789310,.354983960],[-.417838050,.295633920]])
TOL=1e-9

def cross(a,b): return float(a[0]*b[1]-a[1]*b[0])
def project(p,a,b):
    u=b-a
    return a+u*np.clip(np.dot(p-a,u)/np.dot(u,u),0,1) if np.dot(u,u) else a

def segment_gap(a,b,c,d):
    u=b-a; v=d-c; den=cross(u,v)
    if abs(den)>16*np.finfo(float).eps*max(np.linalg.norm(u)*np.linalg.norm(v),np.finfo(float).tiny):
        s=cross(c-a,v)/den; t=cross(c-a,u)/den
        if 0<=s<=1 and 0<=t<=1: return 0.
    return min(np.linalg.norm(a-project(a,c,d)),np.linalg.norm(b-project(b,c,d)),
               np.linalg.norm(c-project(c,a,b)),np.linalg.norm(d-project(d,a,b)))

def contains(poly,p):
    inside=False
    for a,b in zip(poly,np.roll(poly,-1,axis=0)):
        if np.linalg.norm(p-project(p,a,b))<=TOL: return True
        if (a[1]>p[1])!=(b[1]>p[1]):
            x=(b[0]-a[0])*(p[1]-a[1])/(b[1]-a[1])+a[0]
            if p[0]<x: inside=not inside
    return inside

def gap(A,B):
    if contains(A,B[0]) or contains(B,A[0]): return 0.
    return min(segment_gap(a,b,c,d) for a,b in zip(A,np.roll(A,-1,axis=0))
               for c,d in zip(B,np.roll(B,-1,axis=0)))

def sat_hit(A,B):
    """Independent separating-axis predicate for convex polygons."""
    for P in (A,B):
        for a,b in zip(P,np.roll(P,-1,axis=0)):
            edge=b-a; axis=np.array([-edge[1],edge[0]])
            if np.linalg.norm(axis)==0: continue
            axis/=np.linalg.norm(axis); pa=A@axis; pb=B@axis
            if pa.max()<pb.min()-TOL or pb.max()<pa.min()-TOL: return False
    return True

def transform(P,pose):
    x,y,a=pose; c,s=np.cos(a),np.sin(a)
    return P@np.array([[c,s],[-s,c]])+[x,y]

def wall_poly(box):
    return transform(np.array([[-1,-1],[1,-1],[1,1],[-1,1]])*box[3:5]/2,
                     [box[0],box[1],box[6]])

def stl(path):
    raw=path.read_bytes(); n=struct.unpack_from('<I',raw,80)[0]
    assert len(raw)==84+50*n
    dtype=np.dtype([('normal','<f4',(3,)),('v','<f4',(3,3)),('attr','<u2')])
    return np.frombuffer(raw[84:],dtype=dtype)['v'].reshape(-1,3).astype(float)

def verify_footprint(repo):
    root=repo/'src/mir_description/meshes'
    base=stl(root/'collision/mir_100_base.stl'); base[:,0]+=.037646
    scans=stl(root/'visual/sick_lms-100.stl')
    scans[:,:2]*=[1,-1] # URDF mesh Rx(pi)
    fixed=[base[:,:2]]
    for pose in [[.4288,.2358,np.pi/4],[-.3548,-.2352,-3*np.pi/4]]:
        fixed.append(transform(scans[:,:2],pose))
    fixed=np.concatenate(fixed)
    # Each hull halfspace must contain every fixed mesh vertex.
    eq=ConvexHull(FP).equations
    violation=float((fixed@eq[:,:2].T+eq[:,2]).max())
    assert violation<=TOL
    # Wheel y-axis rotation changes xz, not y. Bound by radial max sqrt(x²+z²).
    wheel=stl(root/'visual/wheel.stl'); wheel_r=float(np.linalg.norm(wheel[:,[0,2]],axis=1).max())
    wheel_y=float(np.abs(wheel[:,1]).max())
    drive=[]
    for y in [-.222604,.222604]:
        drive.extend([[x,y+dy] for x in [-wheel_r,wheel_r] for dy in [-wheel_y,wheel_y]])
    drive=np.asarray(drive); assert (drive@eq[:,:2].T+eq[:,2]).max()<=TOL
    caster=stl(root/'collision/caster_wheel_base.stl')
    caster_hub_r=float(np.linalg.norm(caster[:,:2],axis=1).max())
    caster_wheel_r=math.hypot(.0382+wheel_r,wheel_y)
    caster_r=max(caster_hub_r,caster_wheel_r)
    attainable_caster_radius=float(np.sqrt((.0382+np.linalg.norm(wheel[:,[0,2]],axis=1))**2+wheel[:,1]**2).max())
    original_eq=ConvexHull(ORIGINAL_FP).equations
    original_margins=[]
    caster_margins=[]
    for x in [.341346,-.270154]:
        for y in [-.203,.203]:
            margin=float(-(np.array([x,y])@eq[:,:2].T+eq[:,2]).max())-caster_r
            caster_margins.append(margin); assert margin>=0
            original_margins.append(float(-(np.array([x,y])@original_eq[:,:2].T+original_eq[:,2]).max())-attainable_caster_radius)
    return dict(fixed_mesh_halfspace_max_signed_m=violation,
                fixed_mesh_max_radius_m=float(np.linalg.norm(fixed,axis=1).max()),
                envelope_radius_m=float(np.linalg.norm(FP,axis=1).max()),
                caster_all_angles_bound_radius_m=caster_r,
                caster_attainable_vertex_radius_m=attainable_caster_radius,
                caster_envelope_min_margin_m=min(caster_margins),
                caster_sweep_contained=True,drive_wheel_spin_contained=True,
                original_polygon_caster_all_angles_min_margin_m=min(original_margins),
                original_polygon_encloses_all_caster_angles=bool(min(original_margins)>=0),
                fixed_mesh_vertices_checked=len(fixed))

def load_world(repo):
    root=ET.parse(repo/'src/mir_description/worlds/hard_world.sdf').getroot()
    boxes=[]; names=[]
    for m in root.findall('./world/model'):
        if m.attrib['name']=='ground_plane': continue
        if m.find('./link/collision/geometry/box/size') is None: continue
        p=list(map(float,m.findtext('pose').split()))
        assert abs(p[3])+abs(p[4])<1e-12
        size=list(map(float,m.findtext('./link/collision/geometry/box/size').split()))
        boxes.append(p[:3]+size+[p[5]]); names.append(m.attrib['name'])
    assert len(boxes)==65
    return np.array(boxes),names

def checks(repo):
    S=np.array([[-1.,-1],[1,-1],[1,1],[-1,1]])
    H=np.array([[-2.,-.1],[2,-.1],[2,.1],[-2,.1]])
    V=H[:,::-1]
    cases=[('separated',S,S+[3,0],1.),('shared_edge',S,S+[2,0],0.),
           ('corner',S,S+[2,2],0.),('B_contained',S,S/4,0.),
           ('A_contained',S/4,S,0.),('cross_without_vertices',H,V,0.),
           ('degenerate_edge',np.vstack([S[0],S]),S+[3,0],1.)]
    for name,A,B,expect in cases: assert abs(gap(A,B)-expect)<1e-12,name
    rng=np.random.default_rng(6450); random_count=400
    for _ in range(random_count):
        A=transform(FP,[*rng.uniform(-3,3,2),rng.uniform(-np.pi,np.pi)])
        B=transform(S*rng.uniform(.1,2),[*rng.uniform(-3,3,2),rng.uniform(-np.pi,np.pi)])
        d=gap(A,B); assert (d<=TOL)==sat_hit(A,B)
        pose=[*rng.uniform(-10,10,2),rng.uniform(-np.pi,np.pi)]
        assert abs(gap(transform(A,pose),transform(B,pose))-d)<1e-10
    # Check interpolation bound on 80 random rigid motions against dense probes.
    radius=np.linalg.norm(FP,axis=1).max(); max_defect=0.
    for _ in range(80):
        p=rng.uniform(-2,2,3); q=p+rng.uniform(-.2,.2,3)
        B=transform(S*.25,[*rng.uniform(-2,2,2),rng.uniform(-3,3)])
        d0=gap(transform(FP,p),B); d1=gap(transform(FP,q),B)
        residual=np.linalg.norm(q[:2]-p[:2])/2+2*radius*np.sin(min(abs(q[2]-p[2])/4,np.pi/2))
        lb=max(0,min(d0,d1)-residual)
        dense=min(gap(transform(FP,p+(q-p)*a),B) for a in np.linspace(0,1,31))
        max_defect=max(max_defect,lb-dense); assert lb<=dense+1e-10
    out=dict(evidence='Executed Python reference only; MATLAB code not executed',
             edge_cases=[c[0] for c in cases],random_sat_and_rigid_invariance_cases=random_count,
             interpolation_bound_cases=80,interpolation_dense_probes_per_case=31,
             interpolation_max_bound_violation_m=max_defect)
    if repo:
        out['footprint_verification']=verify_footprint(repo)
        boxes,names=load_world(repo)
        start=transform(FP,[-4.31,-2.81,0])
        vals=[gap(start,wall_poly(b)) for b in boxes]; i=int(np.argmin(vals))
        out['world']=dict(wall_count=len(boxes),start_clearance_m=vals[i],nearest_wall=names[i],
                          start_pose=[-4.31,-2.81,0],world_height_range_m=[float(min(boxes[:,2]-boxes[:,5]/2)),float(max(boxes[:,2]+boxes[:,5]/2))])
    return out

if __name__=='__main__':
    parser=argparse.ArgumentParser(); parser.add_argument('--repo',type=Path); parser.add_argument('--output',type=Path)
    args=parser.parse_args(); result=checks(args.repo)
    txt=json.dumps(result,indent=2); print(txt)
    if args.output: args.output.write_text(txt+'\n')
