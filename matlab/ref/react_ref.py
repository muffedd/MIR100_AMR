"""Python reference mirror of lidar_react.m core logic (NOT MATLAB evidence)."""
import numpy as np, csv, heapq, sys
from scipy.signal import convolve2d
def boxes(fn):
    return np.array([[float(x) for x in r[-7:]] for r in list(csv.reader(open(fn)))[1:]])
def polys(b):
    P=[]
    for x,y,z,sx,sy,sz,yaw in b:
        c,s=np.cos(yaw),np.sin(yaw);L=np.array([[-sx/2,-sy/2],[sx/2,-sy/2],[sx/2,sy/2],[-sx/2,sy/2]])
        P.append(np.c_[x+c*L[:,0]-s*L[:,1],y+s*L[:,0]+c*L[:,1]])
    return P
from matplotlib.path import Path as MP
def inpoly(pts,q): return MP(q).contains_points(pts,radius=1e-9)
FP=np.array([[-0.474027,-0.236085],[-0.439393,-0.319793],[-0.355792,-0.354383],[0.493050,-0.295637],[0.548024,0.236692],[0.513393,0.320393],[0.429789,0.354984],[-0.417838,0.295634]])  # polygon v2
def tf(F,p):
    c,s=np.cos(p[2]),np.sin(p[2]);return np.c_[p[0]+c*F[:,0]-s*F[:,1],p[1]+s*F[:,0]+c*F[:,1]]
def segs(P):
    A=np.vstack(P);B=np.vstack([np.roll(q,-1,0) for q in P]);return A,B
def scan(p,A,B,n=360,R=8):
    rel=np.deg2rad(np.linspace(-180,180,n+1)[:n]);ang=p[2]+rel
    dx,dy=np.cos(ang)[:,None],np.sin(ang)[:,None]
    ex,ey=(B[:,0]-A[:,0])[None],(B[:,1]-A[:,1])[None];wx,wy=(A[:,0]-p[0])[None],(A[:,1]-p[1])[None]
    den=dx*ey-dy*ex
    with np.errstate(all='ignore'):
        t=(wx*ey-wy*ex)/den;u=(wx*dy-wy*dx)/den
    bad=(abs(den)<1e-12)|(t<=0)|(u<0)|(u>1);t[bad]=np.inf
    r=t.min(1);r[r>R]=np.inf
    pts=np.c_[p[0]+r*np.cos(ang),p[1]+r*np.sin(ang)];has=np.isfinite(r);r2=np.where(has,r,R)
    return r2,pts,has,rel
def hits(fp,P):
    for q in P:
        if q[:,0].max()<fp[:,0].min() or q[:,0].min()>fp[:,0].max() or q[:,1].max()<fp[:,1].min() or q[:,1].min()>fp[:,1].max():continue
        if inpoly(q,fp).any() or inpoly(fp,q).any():return True
        n=len(fp);m=len(q)
        cr=lambda a,b,c:(b[0]-a[0])*(c[1]-a[1])-(b[1]-a[1])*(c[0]-a[0])
        for i in range(n):
            for j in range(m):
                a,b,c,d=fp[i],fp[(i+1)%n],q[j],q[(j+1)%m]
                if cr(a,b,c)*cr(a,b,d)<0 and cr(c,d,a)*cr(c,d,b)<0:return True
    return False
class G: pass
def mk(o,sz,res):
    g=G();g.res=res;g.o=o;nx,ny=int(np.ceil(sz[0]/res)),int(np.ceil(sz[1]/res));g.shape=(ny,nx)
    g.cx,g.cy=np.meshgrid((np.arange(nx)+.5)*res+o[0],(np.arange(ny)+.5)*res+o[1]);return g
def ij(g,xy):return np.c_[np.floor((xy[:,1]-g.o[1])/g.res).astype(int),np.floor((xy[:,0]-g.o[0])/g.res).astype(int)]
def raster(g,P):
    occ=np.zeros(g.shape,bool)
    for q in P:
        sel=(g.cx>=q[:,0].min()-g.res)&(g.cx<=q[:,0].max()+g.res)&(g.cy>=q[:,1].min()-g.res)&(g.cy<=q[:,1].max()+g.res)
        pts=np.c_[g.cx[sel],g.cy[sel]];m=np.zeros(g.shape,bool);m[sel]=inpoly(pts,q);occ|=m
    return convolve2d(occ.astype(float),np.ones((3,3)),'same')>0
def astar(bl,s,g):
    ny,nx=bl.shape
    if bl[s] or bl[g]:return None
    gs={s:0};par={};op=[(0,s)];cl=set()
    nb=[(-1,-1,1.4142),(-1,0,1),(-1,1,1.4142),(0,-1,1),(0,1,1),(1,-1,1.4142),(1,0,1),(1,1,1.4142)]
    while op:
        _,c=heapq.heappop(op)
        if c in cl:continue
        cl.add(c)
        if c==g:break
        for dr,dc,w in nb:
            r,cc=c[0]+dr,c[1]+dc
            if not(0<=r<ny and 0<=cc<nx) or bl[r,cc] or (r,cc) in cl:continue
            if w>1.1 and (bl[c[0],cc] or bl[r,c[1]]):continue
            ng=gs[c]+w
            if ng<gs.get((r,cc),1e18):gs[(r,cc)]=ng;par[(r,cc)]=c;heapq.heappush(op,(ng+np.hypot(r-g[0],cc-g[1]),(r,cc)))
    if g not in cl:return None
    p=[g]
    while p[-1]!=s:p.append(par[p[-1]])
    return p[::-1]
def replan(g,st,un,pose,gl,infl=None):
    p=plan_se2(g,st,un,pose,gl)
    return (None,None) if p is None else (densify(p),'SE2-footprint')
def densify(q):
    out=[q[0]]
    for k in range(1,len(q)):
        L=np.linalg.norm(q[k]-q[k-1]);m=max(1,int(np.ceil(L/.25)))
        for a in range(1,m+1):out.append(q[k-1]+(q[k]-q[k-1])*a/m)
    return np.array(out)

def plan_se2(g,st,un,start,goal,K=8,tol=.2):
    """Orientation-aware lattice A*: state (i,j,k) on 2*res grid, K headings, exact footprint
    cell mask per heading (polygon grown by 0.71*cell). No radius shrinking."""
    f=2;occ=(st|un);ny,nx=occ.shape;py,px=-(-ny//f),-(-nx//f)
    pad=np.zeros((py*f,px*f),bool);pad[:ny,:nx]=occ;oc=pad.reshape(py,f,px,f).any((1,3))
    rp=g.res*f;mk_=[]
    r=int(np.ceil(.75/rp))
    yy,xx=np.mgrid[-r:r+1,-r:r+1];pts=np.c_[xx.ravel()*rp,yy.ravel()*rp]
    for k in range(K):
        th=2*np.pi*k/K;G_=FP+.71*rp*FP/np.linalg.norm(FP,axis=1)[:,None]
        m=inpoly(pts,tf(G_,[0,0,th]));mk_.append((yy.ravel()[m],xx.ravel()[m]))
    def free(i,j,k):
        di,dj=mk_[k];ii=i+di;jj=j+dj
        if ii.min()<0 or jj.min()<0 or ii.max()>=py or jj.max()>=px:return False
        return not oc[ii,jj].any()
    def cell(xy):return (int(np.floor((xy[1]-g.o[1])/rp)),int(np.floor((xy[0]-g.o[0])/rp)))
    s0=cell(start[:2]);g0=cell(goal)
    open_=[];gs={};par={}
    for k in range(K):
        d=abs(wrap(start[2]-2*np.pi*k/K))
        if free(*s0,k):gs[(*s0,k)]=.4*d/(np.pi/4);heapq.heappush(open_,(gs[(*s0,k)],(*s0,k)))
    if not open_:return None
    cl=set();moves=[(np.sign(round(np.sin(2*np.pi*k/K))),np.sign(round(np.cos(2*np.pi*k/K)))) for k in range(K)]
    end=None
    while open_:
        _,c=heapq.heappop(open_)
        if c in cl:continue
        cl.add(c);i,j,k=c
        if np.hypot(i-g0[0],j-g0[1])*rp<=tol and True:end=c;break
        nb=[((i,j,(k+1)%K),.4),((i,j,(k-1)%K),.4)]
        di,dj=moves[k];nb.append(((i+di,j+dj,k),rp*np.hypot(di,dj)))
        for n,w in nb:
            if n in cl or not free(*n):continue
            ng=gs[c]+w
            if ng<gs.get(n,1e18):gs[n]=ng;par[n]=c;heapq.heappush(open_,(ng+np.hypot(n[0]-g0[0],n[1]-g0[1])*rp,n))
    if end is None:return None
    p=[end]
    while p[-1] in par:p.append(par[p[-1]])
    p=p[::-1];xy=[]
    for i,j,k in p:
        q=np.array([(j+.5)*rp+g.o[0],(i+.5)*rp+g.o[1]])
        if not xy or np.linalg.norm(q-xy[-1])>1e-9:xy.append(q)
    xy.append(np.array(goal,float))   # final approach point (safety filter still gates every command)
    return np.array(xy)
def los(a,b,bl,g):
    n=max(2,int(np.ceil(np.linalg.norm(b-a)/(g.res/2))));xy=a+(b-a)*np.linspace(0,1,n)[:,None];i=ij(g,xy)
    i[:,0]=i[:,0].clip(0,bl.shape[0]-1);i[:,1]=i[:,1].clip(0,bl.shape[1]-1);return not bl[i[:,0],i[:,1]].any()
def shortcut(p,bl,g):
    q=[p[0]];i=0;n=len(p)
    while i<n-1:
        j=n-1
        while j>i+1 and not los(p[i],p[j],bl,g):j-=1
        q.append(p[j]);i=j
    out=[q[0]]
    for k in range(1,len(q)):
        L=np.linalg.norm(q[k]-q[k-1]);m=max(1,int(np.ceil(L/.25)))
        for a in range(1,m+1):out.append(q[k-1]+(q[k]-q[k-1])*a/m)
    return np.array(out)
def corners(p):
    c=[p[0]]
    for k in range(1,len(p)-1):
        a=p[k]-p[k-1];b=p[k+1]-p[k]
        if abs(wrap(np.arctan2(b[1],b[0])-np.arctan2(a[1],a[0])))>1e-3:c.append(p[k])
    c.append(p[-1]);return np.array(c)
def rtd(C,wi,pose,vmax=.5,wmax=.8):
    """rotate-then-drive waypoint tracker"""
    while wi<len(C)-1 and np.hypot(*(C[wi]-pose[:2]))<.06:wi+=1
    d=C[wi]-pose[:2];dist=np.hypot(*d);err=wrap(np.arctan2(d[1],d[0])-pose[2])
    if abs(err)>.05:return 0.,float(np.clip(2*err,-wmax,wmax) if abs(2*err)>.15 else .15*np.sign(err)),wi
    return min(vmax,max(.1,dist)),float(np.clip(2*err,-wmax,wmax)),wi
def blocked(path,idx,pose,un,g,o):
    if not un.any():return False
    r,c=np.nonzero(un);U=np.c_[(c+.5)*g.res+g.o[0],(r+.5)*g.res+g.o[1]]
    grow=FP+.10*FP/np.linalg.norm(FP,axis=1)[:,None];s=0;k=idx;p=pose[:2]
    while s<=1.5 and k<len(path)-1:
        q=path[k+1];sg=q-p;L=np.linalg.norm(sg);h=np.arctan2(sg[1],sg[0]) if L>1e-6 else pose[2]
        fp=tf(grow,[p[0],p[1],h]);near=U[(U[:,0]>=fp[:,0].min())&(U[:,0]<=fp[:,0].max())&(U[:,1]>=fp[:,1].min())&(U[:,1]<=fp[:,1].max())]
        if len(near) and inpoly(near,fp).any():return True
        s+=max(L,.05);p=q;k+=1
    return False
wrap=lambda a:(a+np.pi)%(2*np.pi)-np.pi
def pp(path,idx,pose,vmax=.5,wmax=.8,la=.6):
    n=len(path);w_=np.arange(idx,min(n,idx+41));idx=w_[np.argmin(np.hypot(path[w_,0]-pose[0],path[w_,1]-pose[1]))]
    tgt=path[-1];s=0
    for j in range(idx,n-1):
        s+=np.linalg.norm(path[j+1]-path[j])
        if s>=la:tgt=path[j+1];break
    al=wrap(np.arctan2(tgt[1]-pose[1],tgt[0]-pose[0])-pose[2])
    if abs(al)>np.deg2rad(50):v,w=0,np.sign(al)*wmax
    else:v=vmax*max(np.cos(al),.2);w=np.clip(2*al,-wmax,wmax)
    v=min(v,max(.1,np.hypot(*(path[-1]-pose[:2]))));return v,w,idx
grow_fp=FP+.03*FP/np.linalg.norm(FP,axis=1)[:,None]
def run(csvf,pathw,unseen,dt=.1,maxT=400):
    b=boxes(csvf);kp=polys(b);up=polys(np.array(unseen).reshape(-1,7));tp=kp+up;A,B=segs(tp)
    allp=np.vstack([pathw]+[q for q in kp]);o=allp.min(0)-2;sz=allp.max(0)+2-o;g=mk(o,sz,.05)
    st=raster(g,kp);sn=convolve2d(st.astype(float),np.ones((3,3)),'same')>0;un=np.zeros(g.shape,bool)
    pose=np.array([*pathw[0],np.arctan2(*(pathw[1]-pathw[0])[::-1])]);cur=pathw;idx=0;t=0;rp=0;goal=pathw[-1];log=[];coll=False;ok=False;mn=9;last=pose.copy();stk=0;bs=None;lastRp=-9;C=None;wi=1;newpath=False
    while t<maxT:
        r,pts,has,rel=scan(pose,A,B);mn=min(mn,r.min());rngU=np.full(len(r),8.)
        ri=np.nonzero(has)[0]
        if len(ri):
            I=ij(g,pts[ri]);m=(I[:,0]>=0)&(I[:,0]<g.shape[0])&(I[:,1]>=0)&(I[:,1]<g.shape[1])
            new=m.copy();new[m]=~sn[I[m,0],I[m,1]];un[I[new,0],I[new,1]]=True;rngU[ri[new]]=r[ri[new]]
        if hits(tf(FP,pose),tp):coll=True;print('COLLISION',t,pose);break
        if np.hypot(*(pose[:2]-goal))<.15:ok=True;break
        if t-lastRp>=1.5 and blocked(cur,idx,pose,un,g,None):
            bs=bs if bs is not None else t
            np_,I=replan(g,st,un,pose,goal)
            if np_ is not None:cur=np_;idx=0;rp+=1;bs=None;lastRp=t;newpath=True;print(f't={t:.1f} replan#{rp} infl={I} n={len(np_)}')
            else:
                print(f't={t:.1f} replan FAILED')
                if t-bs>20:break
        _,_,idx=pp(cur,idx,pose)
        if C is None or newpath:C=corners(cur);wi=1;newpath=False
        v,w,wi=rtd(C,wi,pose)
        fr=rngU[np.abs(rel)<=np.deg2rad(15)].min();lim=.5 if fr>1.5 else max(0,min(.5,.6*(fr-.65)));v=min(v,lim)
        if un.any():
            r_,c_=np.nonzero(un);UC=np.c_[(c_+.5)*g.res+g.o[0],(r_+.5)*g.res+g.o[1]]
        else:UC=np.zeros((0,2))
        def safe(q):
            fp=tf(grow_fp,q)
            if hits(fp,kp):return False
            if len(UC):
                nr=UC[(UC[:,0]>=fp[:,0].min())&(UC[:,0]<=fp[:,0].max())&(UC[:,1]>=fp[:,1].min())&(UC[:,1]<=fp[:,1].max())]
                if len(nr) and inpoly(nr,fp).any():return False
            return True
        chosen=None
        for cv,cw in ((v,w),(v/2,w),(0,w),(0,-w),(-.2,0),(-.2,w),(-.2,-w),(0,0)):
            q=np.array([pose[0]+cv*np.cos(pose[2])*dt,pose[1]+cv*np.sin(pose[2])*dt,wrap(pose[2]+cw*dt)])
            if safe(q):chosen=q;break
        pose=chosen if chosen is not None else pose;t+=dt;log.append([t,*pose])
        if np.hypot(*(pose[:2]-last[:2]))>.02:last=pose.copy();stk=t
        if t-stk>30:print('STUCK',pose);break
    return dict(success=ok and not coll,collided=coll,replans=rp,unseen=int(un.sum()),t=t,minR=mn,log=np.array(log))
CASES={'none':None,
 'shipped_demo [-3.9,-1.9,0.5,yaw.3]':[-3.9,-1.9,.25,.5,.5,.5,.3],
 'corridor_mouth [-4.31,-0.91,0.5] (old default)':[-4.31,-.91,.25,.5,.5,.5,0],
 'room [-4.31,-1.7,0.5]':[-4.31,-1.7,.25,.5,.5,.5,0],
 'small [-4.0,-1.5,0.4]':[-4.0,-1.5,.25,.4,.4,.5,0]}
if __name__=='__main__':
    import contextlib,io
    ro=np.array([[0,0],[0,1.12],[0,2.81],[.56,3.09],[1.12,3.37],[1.12,4.5],[1.12,5.62],[2.25,5.62],[3.37,5.62],[4.5,5.62],[5.25,5.25],[5.25,4.12],[5.25,3.37],[6.75,3.37],[7.87,3.37],[8.62,3.37]])+[-4.31,-2.81]
    for k,u in CASES.items():
        f=io.StringIO()
        with contextlib.redirect_stdout(f):r=run(__import__('os').path.join(__import__('os').path.dirname(__import__('os').path.abspath(__file__)),'..','assets','hard_world_boxes.csv'),ro,[u] if u else [])
        o=f.getvalue();print(k,'| success',r['success'],'collided',r['collided'],'replans',r['replans'],'t',round(r['t'],1),'replanFAILED x',o.count('FAILED'),'| minR',round(float(r['minR']),2),[l for l in o.splitlines() if 'STUCK' in l or 'COLL' in l])
    print('footprint v2 y-extent (width at heading 90deg):',round(FP[:,1].max()-FP[:,1].min(),3),'m ; corridor mouth walls at y=-0.94: free x gap [-5.25,-3.75]=1.50 m; 0.5 m box centred x=-4.31 leaves left gap 0.69 m, right gap 0.31 m')
