import numpy as np, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# ---- Reference mirror of follow_route.m (same algorithm, same defaults) ----
def follow_route_ref(p0, yaw0, wps, dt=0.01, lookahead=0.6, maxLinVel=1.0,
                     maxAngVel=2.0, wpRadius=0.15, goalRadius=0.10,
                     maxTime=120.0, wheelRadius=0.0625, wheelSep=0.4452):
    maxSteps = int(np.ceil(1.5*maxTime/dt))
    wps = np.asarray(wps, float)[:, :2]; nWps = len(wps)
    x, y, yaw = p0[0], p0[1], yaw0
    visited = np.zeros(nWps, bool); iw = 0
    wheels = np.zeros(2)
    T=[]; X=[]; Y=[]; H=[]; W=[]; status='timeout'; t=0.0
    def wrap(a): return (a+np.pi)%(2*np.pi)-np.pi
    for k in range(1, maxSteps+1):
        while iw < nWps and np.hypot(x-wps[iw,0], y-wps[iw,1]) < (goalRadius if iw==nWps-1 else wpRadius):
            visited[iw]=True; iw+=1
        if iw >= nWps:
            status='success'; t+=dt
            T.append(t);X.append(x);Y.append(y);H.append(yaw);W.append(wheels.copy()); break
        dAct = np.hypot(wps[iw,0]-x, wps[iw,1]-y)
        rAcc = goalRadius if iw==nWps-1 else wpRadius
        dirv = wps[1]-wps[0] if iw==0 else wps[iw]-wps[iw-1]
        nd = np.hypot(*dirv)
        sPast = 0.0 if nd<1e-12 else ((np.array([x,y])-wps[iw])@dirv)/nd
        if sPast > rAcc:
            target = wps[iw].copy(); Ld = max(dAct, 0.5*wpRadius)
        else:
            Ld = min(lookahead, max(dAct, 0.5*wpRadius))
            target = wps[iw].copy()  # fallback: home onto active waypoint
            for i in range(iw, nWps-1):
                a=wps[i]; d=wps[i+1]-a; f=a-np.array([x,y])
                A=d@d; B=2*f@d; C=f@f-Ld*Ld; disc=B*B-4*A*C
                if A<1e-12 or disc<0: continue
                u=(-B+np.sqrt(disc))/(2*A)
                if 0<=u<=1: target=a+u*d; break
                u=(-B-np.sqrt(disc))/(2*A)
                if 0<=u<=1: target=a+u*d; break
        alpha = wrap(np.arctan2(target[1]-y, target[0]-x) - yaw)
        if abs(alpha) > np.pi/2:
            v=0.0; w=maxAngVel*np.sign(alpha)
        else:
            kap=2*np.sin(alpha)/Ld
            v=maxLinVel*max(0.15, 1-abs(alpha)/(np.pi/2))
            if iw==nWps-1: v=min(v, max(0.05, 2*dAct))
            w=np.clip(v*kap, -maxAngVel, maxAngVel)
        yaw=wrap(yaw+w*dt); x+=v*np.cos(yaw)*dt; y+=v*np.sin(yaw)*dt; t+=dt
        wheels += np.array([(v-w*wheelSep/2)/wheelRadius, (v+w*wheelSep/2)/wheelRadius])*dt
        T.append(t);X.append(x);Y.append(y);H.append(yaw);W.append(wheels.copy())
        if t>=maxTime: break
    return dict(t=np.array(T),x=np.array(X),y=np.array(Y),yaw=np.array(H),
                wheels=np.array(W),visited=visited,status=status,
                finalErr=np.hypot(x-wps[-1,0],y-wps[-1,1]))

# ---- Route v2 (map frame) -> world frame ----
route_map = np.array([[0,0],[0,1.12],[0,2.81],[.56,3.09],[1.12,3.37],[1.12,4.5],
    [1.12,5.62],[2.25,5.62],[3.37,5.62],[4.5,5.62],[5.25,5.25],[5.25,4.12],
    [5.25,3.37],[6.75,3.37],[7.87,3.37],[8.62,3.37]])
off = np.array([-4.31,-2.81])
wps_w = route_map + off
spawn = np.array([-4.31,-2.81]); goal = np.array([4.31,.56])
assert np.allclose(wps_w[0], spawn) and np.allclose(wps_w[-1], goal), 'route endpoints mismatch'

r = follow_route_ref(spawn, 0.0, wps_w)
checks = []
checks.append(('status success', r['status']=='success'))
checks.append(('all waypoints visited', r['visited'].all()))
checks.append(('final error < 0.10', r['finalErr'] < 0.10))
checks.append(('time < 120s', r['t'][-1] < 120))
checks.append(('no NaN in log', np.isfinite(r['x']).all() and np.isfinite(r['yaw']).all() and np.isfinite(r['wheels']).all()))
checks.append(('t strictly increasing', np.all(np.diff(r['t'])>0)))
# per-waypoint closest approach within acceptance radius
dmin = [np.min(np.hypot(r['x']-w[0], r['y']-w[1])) for w in wps_w]
checks.append(('each wp approached < wpRadius (goal<goalRadius)',
               all(d < (0.10 if i==len(dmin)-1 else 0.15) for i,d in enumerate(dmin))))
path_len = np.sum(np.hypot(np.diff(r['x']), np.diff(r['y'])))
for name, ok in checks: print(('PASS' if ok else 'FAIL'), '-', name)
print(f"steps={len(r['t'])} tEnd={r['t'][-1]:.2f}s pathLen={path_len:.2f}m finalErr={r['finalErr']:.4f}m")
print('max per-wp approach dist:', f"{max(dmin):.4f}m", '| wheelAngles end:', r['wheels'][-1].round(2))

fig,ax = plt.subplots(1,2, figsize=(12,5))
ax[0].plot(wps_w[:,0], wps_w[:,1], 'ro--', label='waypoints', ms=4)
ax[0].plot(r['x'], r['y'], 'b-', label='trajectory')
ax[0].plot(*spawn,'g^',ms=10,label='spawn'); ax[0].plot(*goal,'k*',ms=12,label='goal')
ax[0].set_aspect('equal'); ax[0].legend(); ax[0].grid(); ax[0].set_title('pure pursuit route following (reference)')
ax[1].plot(r['t'], r['wheels'][:,0], label='left'); ax[1].plot(r['t'], r['wheels'][:,1], label='right')
ax[1].set_xlabel('t [s]'); ax[1].set_ylabel('wheel angle [rad]'); ax[1].legend(); ax[1].grid()
plt.tight_layout(); plt.savefig('/tmp/follow_route_ref.png', dpi=110)
print('plot saved /tmp/follow_route_ref.png')
assert all(ok for _,ok in checks), 'REFERENCE CHECK FAILED'
print('ALL REFERENCE CHECKS PASSED')
