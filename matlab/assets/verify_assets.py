#!/usr/bin/env python3
"""Static asset checks (no MATLAB/ROS needed): mesh paths, box count, footprint vs URDF collision geometry."""
import re, os, csv, math, numpy as np, xml.etree.ElementTree as ET
ok=True
for u in ['mir100_flat.urdf','mir100_matlab.urdf']:
    p='mir_description/urdf/'+u; t=open(p).read()
    for f in re.findall(r'<mesh filename="([^"]+)"',t):
        e=os.path.exists(os.path.normpath(os.path.join(os.path.dirname(p),f))); ok&=e
        if not e: print('MISSING',u,f)
    print(u,'mesh paths ok' if ok else 'PROBLEM')
n=len(list(csv.DictReader(open('hard_world_boxes.csv')))); print('boxes',n); ok&=(n==65)
r=ET.parse('mir_description/urdf/mir100_matlab.urdf').getroot()
def rot(rpy):
    a,b,c=rpy; Rx=np.array([[1,0,0],[0,math.cos(a),-math.sin(a)],[0,math.sin(a),math.cos(a)]])
    Ry=np.array([[math.cos(b),0,math.sin(b)],[0,1,0],[-math.sin(b),0,math.cos(b)]])
    Rz=np.array([[math.cos(c),-math.sin(c),0],[math.sin(c),math.cos(c),0],[0,0,1]]); return Rz@Ry@Rx
J={j.find('child').get('link'):j for j in r.iter('joint')}
def vec(e,k,d='0 0 0'): return np.array(list(map(float,(e.get(k) if e is not None and e.get(k) else d).split())))
def T(link):
    # pose of link in base_footprint (fixed/zero-angle joints)
    M=np.eye(4)
    while link in J:
        o=J[link].find('origin'); Mj=np.eye(4); Mj[:3,:3]=rot(vec(o,'rpy')); Mj[:3,3]=vec(o,'xyz'); M=Mj@M; link=J[link].find('parent').get('link')
    return M
pts=[]
for l in r.iter('link'):
    for c in l.findall('collision'):
        g=c.find('geometry')[0]; o=c.find('origin'); Mc=np.eye(4); Mc[:3,:3]=rot(vec(o,'rpy')); Mc[:3,3]=vec(o,'xyz'); M=T(l.get('name'))@Mc
        if g.tag=='box':
            s=vec(g,'size')/2
            for sx in(-1,1):
                for sy in(-1,1):
                    for sz in(-1,1): pts.append((M@np.append(s*[sx,sy,sz],1))[:3])
        elif g.tag=='cylinder' and 'rotation' not in l.get('name') and 'caster_wheel' not in l.get('name') and 'wheel' not in l.get('name'): pass
        elif g.tag=='cylinder':
            rad=float(g.get('radius')); ln=float(g.get('length'))
            for a in np.linspace(0,2*math.pi,24,endpoint=False):
                for h in(-ln/2,ln/2): pts.append((M@np.array([rad*math.cos(a),h,rad*math.sin(a),1]))[:3] if 'wheel' in l.get('name') else (M@np.array([rad*math.cos(a),rad*math.sin(a),h,1]))[:3])
pts=np.array(pts); xy=pts[:,:2]
# caster swivel envelope: wheel centre offset 0.0382 from pivot + radius 0.0625 -> circle of 0.1007 about each pivot (ignored by box/cyl list above if swivel=0)
P=np.array([[-.4679,-.2352],[-.4348,-.3152],[-.3548,-.3483],[.4876,-.29],[.5419,.2358],[.5088,.3158],[.4288,.3489],[-.4124,.29]])
def inside(poly,q):
    s=0
    for i in range(len(poly)):
        a,b=poly[i],poly[(i+1)%len(poly)]; s+= (b[0]-a[0])*(q[1]-a[1])-(b[1]-a[1])*(q[0]-a[0])>=-1e-3*math.hypot(b[0]-a[0],b[1]-a[1])
    return s in(0,len(poly))
out=[q for q in xy if not inside(P,q)]
print('collision pts',len(xy),'bbox x[%.4f,%.4f] y[%.4f,%.4f]'%(xy[:,0].min(),xy[:,0].max(),xy[:,1].min(),xy[:,1].max()))
print('nav2 footprint polygon contains all URDF collision points:',not out, '(outside: %d)'%len(out)); print('polygon max radius %.4f'%np.hypot(*P.T).max(),'; collision max radius %.4f'%np.hypot(*xy.T).max())
print('ALL OK' if ok else 'FAIL')
