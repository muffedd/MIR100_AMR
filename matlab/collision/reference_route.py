#!/usr/bin/env python3
"""Checks supplied route with explicit in-place turns, not a controller run."""
import argparse,json
import numpy as np
from reference_collision import FP,load_world,wall_poly,transform,gap,TOL

def check(repo):
    boxes,names=load_world(repo); walls=[wall_poly(b) for b in boxes]
    wall_min=np.array([w.min(0) for w in walls]);wall_max=np.array([w.max(0) for w in walls])
    route=np.array([[0,0],[0,1.12],[0,2.81],[.56,3.09],[1.12,3.37],[1.12,4.5],
                    [1.12,5.62],[2.25,5.62],[3.37,5.62],[4.5,5.62],[5.25,5.25],
                    [5.25,4.12],[5.25,3.37],[6.75,3.37],[7.87,3.37],[8.62,3.37]])+[-4.31,-2.81]
    def distance(p):
        poly=transform(FP,p)
        lower=np.linalg.norm(np.maximum(0,np.maximum(wall_min-poly.max(0),poly.min(0)-wall_max)),axis=1)
        best=np.inf; wall=-1
        for j in np.argsort(lower):
            if lower[j]>best: break
            d=gap(poly,walls[j])
            if d<best:best,wall=d,int(j)
            if d<=TOL:break
        return best,wall
    poses=[];yaw=0.
    for i in range(len(route)-1):
        d=route[i+1]-route[i];target=np.arctan2(d[1],d[0]);delta=np.arctan2(np.sin(target-yaw),np.cos(target-yaw))
        for f in np.linspace(0,1,max(2,int(abs(delta)/.02)+2)):
            poses.append([*route[i],yaw+f*delta])
        yaw+=delta
        for f in np.linspace(0,1,max(2,int(np.linalg.norm(d)/.03)+2)):
            poses.append([*(route[i]+f*d),yaw])
    vals=[distance(p) for p in poses];ix=min(range(len(vals)),key=lambda i:vals[i][0])
    r=np.linalg.norm(FP,axis=1).max();p=np.array(poses)
    residual=np.linalg.norm(np.diff(p[:,:2],axis=0),axis=1)/2+2*r*np.sin(np.minimum(np.abs(np.diff(p[:,2]))/4,np.pi/2))
    lb=np.maximum(0,np.minimum(np.array(vals[:-1])[:,0],np.array(vals[1:])[:,0])-residual)
    result=dict(evidence='Executed Python polygon reference, not MATLAB execution or controller trajectory',
                trajectory_model='Straight route segments, with shortest in-place rotations at each waypoint; initial yaw 0',
                sample_count=len(poses),max_linear_sample_step_m=.03,max_yaw_sample_step_rad=.02,
                min_sampled_clearance_m=vals[ix][0],nearest_wall=names[vals[ix][1]],minimum_pose=poses[ix],
                any_sampled_collision=any(v[0]<=TOL for v in vals),continuous_clearance_lower_bound_m=float(lb.min()))
    assert not result['any_sampled_collision'] and result['continuous_clearance_lower_bound_m']>0
    return result

if __name__=='__main__':
    from pathlib import Path
    p=argparse.ArgumentParser();p.add_argument('--repo',required=True,type=Path);p.add_argument('--output',type=Path)
    a=p.parse_args();text=json.dumps(check(a.repo),indent=2);print(text)
    if a.output:a.output.write_text(text+'\n')
