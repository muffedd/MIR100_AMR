# Static reference check (NOT MATLAB execution): re-parse SDF, compare to CSV.
import re,csv,sys
import os
H=os.path.dirname(os.path.abspath(__file__))
sdf=sys.argv[1] if len(sys.argv)>1 else os.path.join(H,'..','assets','mir_description','worlds','hard_world.sdf')
c=sys.argv[2] if len(sys.argv)>2 else os.path.join(H,'..','assets','hard_world_boxes.csv')
s=open(sdf).read()
m=re.findall(r'<model name="(w\d+)">.*?<pose>([^<]+)</pose>.*?<collision.*?<size>([^<]+)</size>',s)
rows=list(csv.DictReader(open(c)))
assert len(m)==len(rows)==65
for (n,p,z),r in zip(m,rows):
    p=list(map(float,p.split()));z=list(map(float,z.split()))
    assert r['id']==n and abs(float(r['xc'])-p[0])<1e-9 and abs(float(r['yc'])-p[1])<1e-9 and abs(float(r['zc'])-p[2])<1e-9
    assert [float(r[k]) for k in('sx','sy','sz')]==z and float(r['yaw'])==p[5] and p[3]==p[4]==0
xs=[float(r['xc'])+float(r['sx'])/2 for r in rows]+[float(r['xc'])-float(r['sx'])/2 for r in rows]
ys=[float(r['yc'])+float(r['sy'])/2 for r in rows]+[float(r['yc'])-float(r['sy'])/2 for r in rows]
print('OK 65 walls match SDF; extent x[%.3f,%.3f] y[%.3f,%.3f]; ground in SDF excluded by name'%(min(xs),max(xs),min(ys),max(ys)))
