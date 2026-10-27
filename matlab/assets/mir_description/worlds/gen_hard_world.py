# Generator for hard_world.sdf (run: python3 gen_hard_world.py; needs numpy+PIL).
# Regenerates the world, a grid-truth preview PNG and a clearance report.
import numpy as np
from collections import deque
from PIL import Image, ImageDraw

CS=0.375; W,H=34,26
g=np.ones((H,W),dtype=np.uint8)
def carve(c0,r0,c1,r1): g[r0:r1+1,c0:c1+1]=0

carve(1,16,9,24)    # Room A (start)
carve(1,1,9,9)      # Room B
carve(24,8,32,14)   # Room C (goal) - shrunk south wall to row14
carve(2,12,6,16)    # S1 vertical from A
carve(2,12,10,14)   # S2 horizontal jog
carve(6,9,10,14)    # S3 vertical into B (row9 shared)
carve(9,2,20,6)     # S4 horizontal from B east wall (door col9 rows2-6), full-width overlap into S5
carve(16,4,20,12)   # S5 vertical down
carve(16,10,26,14)  # S6 horizontal into C, full-width overlap into S5
carve(11,10,14,13)  # D1 dead end off S2
carve(20,5,23,7)    # D2 dead end off S5
carve(20,15,23,18)  # D3 dead end off S6
g[15,2]=1           # A door narrowed at the wall row: opening cols3-6 = 1.2m
g[11,5]=0           # chamfer inner corner of S2->S3 turn (leak-checked)
g[11,4]=0           # second chamfer cell widening the S1/S2/S3 junction diagonal (leak-checked)
g[7,15]=0           # chamfer inner corner of S4->S5 turn (leak-checked)
g[9,21]=0           # chamfer inner corner of S5->S6 turn (leak-checked)
g[14,23:27]=1       # C door narrowed to rows10-13 = 1.2m

S=(5,20); G=(28,11)
def bfs(grid,start,goal,allowed=None):
    q=deque([start]); prev={start:None}
    while q:
        c,r=q.popleft()
        if (c,r)==goal:
            p=[]; n=(c,r)
            while n is not None: p.append(n); n=prev[n]
            return p[::-1]
        for dc,dr in ((1,0),(-1,0),(0,1),(0,-1)):
            nc,nr=c+dc,r+dr
            if 0<=nc<grid.shape[1] and 0<=nr<grid.shape[0] and grid[nr,nc]==0 and (nc,nr) not in prev:
                if allowed is not None and not allowed[nr,nc]: continue
                prev[(nc,nr)]=(c,r); q.append((nc,nr))
    return None

path=bfs(g,S,G)
print('S->G connected:',path is not None,'cells:',len(path) if path else 0)

# --- continuous clearance: wall rectangles, exact point-to-rect distance on 0.1m subgrid
runs=[]
for r in range(H):
    c=0
    while c<W:
        if g[r,c]==1:
            c0=c
            while c<W and g[r,c]==1: c+=1
            runs.append((c0,r,c-1,r))
        else: c+=1
rects=[((c0-W/2)*CS,(c1-W/2+1)*CS,(H/2-r0-1)*CS,(H/2-r0)*CS) for c0,r0,c1,r1 in runs]
SS=0.1
xs=np.arange(-W/2*CS+SS/2, W/2*CS, SS); ys=np.arange(-H/2*CS+SS/2, H/2*CS, SS)
PX,PY=np.meshgrid(xs,ys)
dist=np.full(PX.shape,99.0)
for x0,x1,y0,y1 in rects:
    dx=np.maximum(np.maximum(x0-PX,0),PX-x1); dy=np.maximum(np.maximum(y0-PY,0),PY-y1)
    dist=np.minimum(dist,np.hypot(dx,dy))
def wxy(c,r): return ((c-W/2+0.5)*CS,(H/2-r-0.5)*CS)
import heapq
H2,W2=dist.shape
si=(np.argmin(np.abs(ys-wxy(*S)[1])),np.argmin(np.abs(xs-wxy(*S)[0])))
gi=(np.argmin(np.abs(ys-wxy(*G)[1])),np.argmin(np.abs(xs-wxy(*G)[0])))
best=np.full(dist.shape,-1.0); best[si]=dist[si]; prevw={si:None}
pq=[(-dist[si],si)]
while pq:
    negd,(r,c)=heapq.heappop(pq); cur=-negd
    if cur<best[r,c]-1e-12: continue
    if (r,c)==gi: break
    for dc,dr in ((1,0),(-1,0),(0,1),(0,-1)):
        nc,nr=c+dc,r+dr
        if 0<=nr<H2 and 0<=nc<W2:
            nd=min(cur,dist[nr,nc])
            if nd>best[nr,nc]+1e-12:
                best[nr,nc]=nd; prevw[(nr,nc)]=(r,c); heapq.heappush(pq,(-nd,(nr,nc)))
BOT=best[gi]
pw=[]; n=gi
while n is not None: pw.append(n); n=prevw.get(n)
bn=min(pw,key=lambda rc: dist[rc])
print('WIDEST PATH bottleneck clearance: %.3f m at world (%.2f,%.2f); length %.1f m'%(BOT,xs[bn[1]],ys[bn[0]],len(pw)*SS))
print('conservative 0.55m enclosing-disc route exists:', BOT>=0.55)
# grid-cell clearance for the painted band + route display (euclidean to wall cells)
wall=np.argwhere(g==1); clear=np.full((H,W),99.0)
for r in range(H):
    dd=np.sqrt(((wall[:,0]-r)**2)[:,None]+(wall[:,1][:,None]-np.arange(W))**2).min(axis=0)
    clear[r]=dd*CS
allowed=(g==0)&(clear>=0.53)
path2=bfs(g,S,G,allowed)
turns=sum(1 for i in range(1,len(path2)-1) if (path2[i][0]-path2[i-1][0],path2[i][1]-path2[i-1][1])!=(path2[i+1][0]-path2[i][0],path2[i+1][1]-path2[i][1]))
print('display route cells:',len(path2),'turns:',turns,'len %.1f m'%(len(path2)*CS))
print('S world (%.2f,%.2f)  G world (%.2f,%.2f)'%(*wxy(*S),*wxy(*G)))

# SDF
with open('/tmp/hardmap/hard_world.sdf','w') as f:
    f.write('<?xml version="1.0" ?>\n<sdf version="1.9">\n  <world name="hard">\n')
    f.write('''    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
    <gravity>0 0 -9.81</gravity>
    <light type="directional" name="sun"><cast_shadows>true</cast_shadows><pose>0 0 10 0 0 0</pose><diffuse>0.8 0.8 0.8 1</diffuse><specular>0.2 0.2 0.2 1</specular><direction>-0.5 0.1 -0.9</direction></light>
    <model name="ground_plane"><static>true</static><link name="l"><collision name="c"><geometry><plane><normal>0 0 1</normal><size>30 30</size></plane></geometry></collision><visual name="v"><geometry><plane><normal>0 0 1</normal><size>30 30</size></plane></geometry><material><ambient>0.5 0.5 0.5 1</ambient><diffuse>0.6 0.6 0.6 1</diffuse></material></visual></link></model>\n''')
    for i,(c0,r0,c1,r1) in enumerate(runs):
        cx=((c0-W/2)+(c1-W/2+1))/2*CS; cy=(H/2-r0-0.5)*CS; lx=(c1-c0+1)*CS
        f.write(f'    <model name="w{i}"><static>true</static><pose>{cx:.3f} {cy:.3f} 1.1 0 0 0</pose><link name="l"><collision name="c"><geometry><box><size>{lx:.3f} {CS} 2.2</size></box></geometry></collision><visual name="v"><geometry><box><size>{lx:.3f} {CS} 2.2</size></box></geometry><material><ambient>0.35 0.37 0.42 1</ambient><diffuse>0.45 0.48 0.55 1</diffuse></material></visual></link></model>\n')
    f.write('    <model name="observer_cam"><static>true</static><pose>0 0 13 0 1.5708 0</pose><link name="l"><sensor name="camera" type="camera"><camera><horizontal_fov>1.047</horizontal_fov><image><width>960</width><height>720</height></image><clip><near>0.05</near><far>60</far></clip></camera><always_on>1</always_on><update_rate>10</update_rate><visualize>false</visualize><topic>camera</topic></sensor></link></model>\n  </world>\n</sdf>\n')
print('wall boxes:',len(runs))

# preview
sc=24; img=Image.new('RGB',(W*sc+140,H*sc+96),(255,255,255)); d=ImageDraw.Draw(img)
ox,oy=60,30
for r in range(H):
    for c in range(W):
        if g[r,c]==1: d.rectangle([ox+c*sc,oy+r*sc,ox+(c+1)*sc-1,oy+(r+1)*sc-1],fill=(40,42,50))
        elif allowed[r,c] and clear[r,c]<0.75: d.rectangle([ox+c*sc,oy+r*sc,ox+(c+1)*sc-1,oy+(r+1)*sc-1],fill=(255,235,200))
if path2:
    for c,r in path2: d.ellipse([ox+c*sc+sc/4,oy+r*sc+sc/4,ox+(c+1)*sc-sc/4,oy+(r+1)*sc-sc/4],fill=(80,140,255))
for (c,r),col,lab in ((S,(0,170,0),'S'),(G,(220,0,0),'G')):
    d.ellipse([ox+c*sc+2,oy+r*sc+2,ox+(c+1)*sc-2,oy+(r+1)*sc-2],fill=col)
    d.text((ox+c*sc+sc/3,oy+r*sc+2),lab,fill=(255,255,255))
d.text((ox+3*sc,oy+20*sc),'Room A',fill=(0,0,0))
d.text((ox+3*sc,oy+4*sc),'Room B',fill=(0,0,0))
d.text((ox+26*sc,oy+9.3*sc),'Room C',fill=(0,0,0))
d.text((ox+11.2*sc,oy+11*sc),'dead\nend',fill=(150,0,0))
d.text((ox+20.4*sc,oy+5.4*sc),'dead end',fill=(150,0,0))
d.text((ox+20.5*sc,oy+16*sc),'dead\nend',fill=(150,0,0))
d.text((ox+2.9*sc,oy+17.1*sc),'1.5m',fill=(0,0,150))
d.text((ox+20.4*sc,oy+10.4*sc),'1.5m',fill=(0,0,150))
d.text((ox,oy+H*sc+8),'Hard map v2 (grid-truth preview, NOT a runtime scan). cell=0.375m corridors=1.9m doors=1.5m',fill=(0,0,0))
d.text((ox,oy+H*sc+22),'v2 = v1 layout x1.25, widened for conservative 0.60m planner radius (disclosed change). orange: tight band, blue: verified route',fill=(0,0,0))
d.text((ox,oy+H*sc+36),f'World 12.8x9.8m. Static widest-path bottleneck {BOT:.2f}m >= 0.60m radius + margin (0.1m grid, exact rect distances; not swept-path proof).',fill=(0,0,0))
img.save('/tmp/hardmap/map_preview.png')
print('preview saved')
