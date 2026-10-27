"""Reference (Python) mirror of build_map/check_map. NOT MATLAB evidence."""
import numpy as np, csv, os
_P=os.path.join(os.path.dirname(os.path.abspath(__file__)),'..','assets','hard_world_boxes.csv')
B=np.array([[float(x) for x in r[-7:]] for r in list(csv.reader(open(_P)))[1:]])  # id column + 7 data columns
res=20; mg=.5; zb=(.05,1.0)
b=B[(B[:,2]-B[:,5]/2<zb[1])&(B[:,2]+B[:,5]/2>zb[0])]
ext=.5*(abs(b[:,3]*np.cos(b[:,6]))+abs(b[:,4]*np.sin(b[:,6])))
eyx=.5*(abs(b[:,3]*np.sin(b[:,6]))+abs(b[:,4]*np.cos(b[:,6])))
lo=np.array([(b[:,0]-ext).min(),(b[:,1]-eyx).min()])-mg
hi=np.array([(b[:,0]+ext).max(),(b[:,1]+eyx).max()])+mg
org=np.floor(lo*res)/res
nc=int(np.ceil((hi[0]-org[0])*res-1e-9)); nr=int(np.ceil((hi[1]-org[1])*res-1e-9))
X,Y=np.meshgrid(org[0]+(np.arange(nc)+.5)/res, org[1]+(np.arange(nr)+.5)/res)
occ=np.zeros((nr,nc),bool); h=.5/res
for r in b:
    dx=X-r[0];dy=Y-r[1];c,s=np.cos(r[6]),np.sin(r[6])
    occ|=(abs(c*dx+s*dy)<=r[3]/2+h+1e-9)&(abs(-s*dx+c*dy)<=r[4]/2+h+1e-9)
print('walls',len(b),'bbox lo/hi',lo,hi,'origin',org,'size',nr,'x',nc,'cells; occupied frac %.4f'%occ.mean())
def q(p):
    col=int(np.floor((p[0]-org[0])*res)); row=int(np.floor((p[1]-org[1])*res))
    return (0<=col<nc and 0<=row<nr) and occ[row,col]
def inb(p): return org[0]<=p[0]<org[0]+nc/res and org[1]<=p[1]<org[1]+nr/res
off=np.array([-4.31,-2.81])
S=np.array([-4.31,-2.81]);G=np.array([4.31,.56])
route=np.array([[0,0],[0,1.12],[0,2.81],[.56,3.09],[1.12,3.37],[1.12,4.5],[1.12,5.62],[2.25,5.62],[3.37,5.62],[4.5,5.62],[5.25,5.25],[5.25,4.12],[5.25,3.37],[6.75,3.37],[7.87,3.37],[8.62,3.37]])+off
print('start in/free',inb(S),not q(S),' goal in/free',inb(G),not q(G))
print('route vertices bad:',[i for i,p in enumerate(route) if not inb(p) or q(p)])
# wall probes
bad=0
for r in b:
    for f in [(0,0),(.45,0),(-.45,0),(0,.45),(0,-.45)]:
        if not q((r[0]+f[0]*r[3],r[1]+f[1]*r[4])): bad+=1
print('interior probes unoccupied:',bad)
# segment clearance: min distance from route polyline to any wall box, vs robot radius .5988
def dist_box(p,r):
    dx=max(abs(p[0]-r[0])-r[3]/2,0);dy=max(abs(p[1]-r[1])-r[4]/2,0);return np.hypot(dx,dy)
md=1e9
for a,c in zip(route[:-1],route[1:]):
    for t in np.linspace(0,1,60):
        p=a+(c-a)*t; md=min(md,min(dist_box(p,r) for r in b))
print('min distance of the HARDCODED LEGACY polyline to walls %.3f m (legacy route, old radius .5988; NOT the planner output, NOT clear of the current 0.6052 inflation)'%md)
# route segments free in grid
bads=0
for a,c in zip(route[:-1],route[1:]):
    for t in np.linspace(0,1,200):
        if q(a+(c-a)*t): bads+=1
print('route polyline samples in occupied cells:',bads)

# --- current-parameter check: grid A* (8-connected) on walls inflated by 0.6052 m, start/goal as run_all uses
from scipy.ndimage import distance_transform_edt
import heapq
R=0.6052
dt=distance_transform_edt(~occ)/res          # metres from nearest occupied cell
free=dt>=R
def cell(p): return (int(np.floor((p[1]-org[1])*res)), int(np.floor((p[0]-org[0])*res)))
s0,g0=cell(S),cell(G)
print('inflation %.4f m: start free %s goal free %s'%(R,free[s0],free[g0]))
dist={s0:0.0};prev={};pq=[(0.0,s0)];found=False
while pq:
    d,u=heapq.heappop(pq)
    if u==g0: found=True;break
    if d>dist[u]:continue
    for dr in(-1,0,1):
        for dc in(-1,0,1):
            if dr==dc==0:continue
            v=(u[0]+dr,u[1]+dc)
            if not(0<=v[0]<nr and 0<=v[1]<nc) or not free[v]:continue
            nd=d+np.hypot(dr,dc)/res
            if nd<dist.get(v,1e18):dist[v]=nd;prev[v]=u;heapq.heappush(pq,(nd+np.hypot(v[0]-g0[0],v[1]-g0[1])/res*0,v))
print('A* on 0.6052-inflated 65-wall grid: ', ('PATH FOUND length %.2f m (grid, unsmoothed)'%dist[g0]) if found else 'NO PATH')
if found:
    u=g0;pts=[u]
    while u!=s0:u=prev[u];pts.append(u)
    print('min wall distance along path %.3f m (>= %.4f required)'%(min(dt[c] for c in pts),R))
