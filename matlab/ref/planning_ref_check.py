# SYNTHETIC corridor probe only. Radii .5988/.6/.5 are conceptual inflation values (.5988 is the OLD footprint radius; current is 0.6052).
# It is NOT a validation of the current robot or the 65-wall world.
import numpy as np, heapq
from scipy.signal import convolve2d
def plan(occ,res,org,s,g,r):
    ny,nx=occ.shape; r=r+(2**.5+.05)/res;rc=int(np.ceil(r*res))
    dx,dy=np.meshgrid(range(-rc,rc+1),range(-rc,rc+1)); disk=((dx**2+dy**2)<=(r*res)**2).astype(float)
    p=np.ones((ny+2*rc,nx+2*rc)); p[rc:rc+ny,rc:rc+nx]=occ
    inf=convolve2d(p,disk,'same')>0; inf=inf[rc:rc+ny,rc:rc+nx]
    c=lambda q:(int((q[0]-org[0])*res),int((q[1]-org[1])*res))
    S,G=c(s),c(g)
    if inf[S[1],S[0]] or inf[G[1],G[0]]: return None
    d={S:0};pq=[(0,S)];cl=set()
    while pq:
        f,u=heapq.heappop(pq)
        if u in cl: continue
        cl.add(u)
        if u==G: return d[u]/res
        for a,b in [(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]:
            x,y=u[0]+a,u[1]+b
            if not(0<=x<nx and 0<=y<ny) or inf[y,x]: continue
            if a and b and (inf[u[1],x] or inf[y,u[0]]): continue
            ng=d[u]+np.hypot(a,b)
            if ng<d.get((x,y),1e9): d[(x,y)]=ng; heapq.heappush(pq,(ng+np.hypot(x-G[0],y-G[1]),(x,y)))
    return None
res=20; W,H=8,6
for gap in (1.125,1.30):
    occ=np.zeros((H*res,W*res),bool)
    wl=int((3-gap/2)*res); wr=int((3+gap/2)*res)
    occ[:wl,80:84]=True; occ[wr:,80:84]=True
    for r in (0.5988,0.60,0.5):
        print('gap',gap,'r',r,plan(occ,res,(0,0),(1,3),(7,3),r))
