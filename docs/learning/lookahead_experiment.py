"""Change ONE number and watch what happens: the pure-pursuit lookahead.

Runs the Python mirror of follow_route.m (matlab/ref/follow_route_ref_check.py)
on the original 16-waypoint route for several lookahead distances, then measures
  * time and final goal error,
  * cross-track error (distance to the planned polyline),
  * minimum clearance between the real robot footprint and the 65 walls.

This is the PYTHON MIRROR on the older 16-waypoint route, not MATLAB and not the
A* path, so use it to learn the trend, not to quote numbers in the report.

Run from anywhere:   python docs/learning/lookahead_experiment.py
Needs: numpy, matplotlib, shapely   (pip install numpy matplotlib shapely)
"""
import csv, pathlib
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from shapely.geometry import Polygon, LineString, Point, box as sbox
from shapely.ops import unary_union

HERE = pathlib.Path(__file__).resolve().parent
REPO = HERE.parents[1]
REF = REPO / "matlab" / "ref" / "follow_route_ref_check.py"

# Reuse the mirror's function without running its built-in checks.
src = REF.read_text()
src = src[:src.index("# ---- Route v2")]
ns = {}
exec(compile(src, str(REF), "exec"), ns)
follow = ns["follow_route_ref"]

route_map = np.array([[0,0],[0,1.12],[0,2.81],[.56,3.09],[1.12,3.37],[1.12,4.5],
    [1.12,5.62],[2.25,5.62],[3.37,5.62],[4.5,5.62],[5.25,5.25],[5.25,4.12],
    [5.25,3.37],[6.75,3.37],[7.87,3.37],[8.62,3.37]])
wps = route_map + np.array([-4.31, -2.81])

FOOT = np.array([[-0.474027,-0.236085],[-0.439393,-0.319793],[-0.355792,-0.354383],
                 [0.493050,-0.295637],[0.548024,0.236692],[0.513393,0.320393],
                 [0.429789,0.354984],[-0.417838,0.295634]])
walls = []
with open(REPO / "matlab" / "assets" / "hard_world_boxes.csv") as f:
    for r in csv.DictReader(f):
        xc, yc, sx, sy = (float(r[k]) for k in ("xc", "yc", "sx", "sy"))
        walls.append(sbox(xc - sx/2, yc - sy/2, xc + sx/2, yc + sy/2))
WALLS = unary_union(walls)
PATH = LineString(wps)

def pose_poly(x, y, yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    return Polygon(FOOT @ np.array([[c, s], [-s, c]]) + [x, y])

def analyse(r, every=5):
    xy = np.c_[r["x"], r["y"]]
    ct = np.array([PATH.distance(Point(p)) for p in xy])
    idx = range(0, len(xy), every)
    clr = min(pose_poly(r["x"][i], r["y"][i], r["yaw"][i]).distance(WALLS) for i in idx)
    return ct, clr

print(f"{'lookahead':>9} {'status':>8} {'time s':>7} {'goal err m':>10} "
      f"{'max xtrack m':>12} {'rms xtrack m':>12} {'min clearance m':>15}")
fig, ax = plt.subplots(figsize=(9, 6)); ax.set_aspect("equal")
for b in walls[:0]: pass
for poly in WALLS.geoms if hasattr(WALLS, "geoms") else [WALLS]:
    ax.fill(*poly.exterior.xy, color="#e3e1df", lw=0)
ax.plot(wps[:, 0], wps[:, 1], "k--", lw=1, label="planned route")
colors = ["#0075de", "#1a8f4c", "#ffb110", "#e32d14", "#7a3fd0"]
for L, col in zip([0.2, 0.4, 0.6, 1.0, 1.5], colors):
    r = follow(wps[0], 0.0, wps, lookahead=L)
    ct, clr = analyse(r)
    print(f"{L:9.2f} {r['status']:>8} {r['t'][-1]:7.2f} {r['finalErr']:10.4f} "
          f"{ct.max():12.3f} {np.sqrt((ct**2).mean()):12.3f} {clr:15.3f}")
    ax.plot(r["x"], r["y"], color=col, lw=1.6, label=f"lookahead {L} m")
ax.legend(loc="lower right", fontsize=9); ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
ax.set_title("Pure pursuit: short lookahead hugs the path, long lookahead cuts corners")
out = HERE / "fig6_lookahead_experiment.png"
fig.savefig(out, dpi=150, bbox_inches="tight")
print("saved", out)
