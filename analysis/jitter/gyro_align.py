# Fit gyro -> SLAM-body alignment: SLAM rotation over windows vs M * integrated gyro. Reports rotation angle of the
# best rotation (Kabsch), per-axis scale, and residual before/after.
import sys, numpy as np
def qmul(a,b):
    x1,y1,z1,w1=a.T; x2,y2,z2,w2=b.T
    return np.stack([w1*x2+x1*w2+y1*z2-z1*y2, w1*y2-x1*z2+y1*w2+z1*x2, w1*z2+x1*y2-y1*x2+z1*w2, w1*w2-x1*x2-y1*y2-z1*z2],1)
def qinv(q): return q*np.array([-1,-1,-1,1])
def qlog(q):
    q=q*np.sign(q[:,3:4]+1e-12); v=q[:,:3]; s=np.linalg.norm(v,axis=1); ang=2*np.arctan2(s,q[:,3])
    return v/np.maximum(s,1e-12)[:,None]*ang[:,None]
for path in sys.argv[1:]:
    I=[];S=[]
    for l in open(path):
        if l.startswith('I,'): p=l.split(','); I.append([float(x) for x in p[2:6]])
        elif l.startswith('S,'): p=l.split(','); S.append([float(x) for x in p[2:10]])
    I=np.array(I);S=np.array(S);S=S[np.argsort(S[:,0])]; _,u=np.unique(S[:,0],return_index=True);S=S[u]
    _,u=np.unique(I[:,0],return_index=True); I=I[u]
    S=S[S[:,0]>S[0,0]+15e9]
    # body-frame SLAM rotation between SLAM samples k steps apart vs integrated gyro (rate, rad/s average)
    k=6  # 100 ms
    q=S[:,4:8]; t=S[:,0]
    dq=qmul(qinv(q[:-k]),q[k:]); rot=qlog(dq)  # body frame rotation vector over window
    dtw=(t[k:]-t[:-k])*1e-9
    good=(dtw<0.13)&(np.linalg.norm(rot,axis=1)<0.6)
    # integrated gyro over the same windows (sum of rate*dt; body frame, small-angle ok for |rot|<0.6 rad)
    ci=np.concatenate([np.zeros((1,3)),np.cumsum(I[1:,1:4]*np.diff(I[:,0])[:,None]*1e-9,axis=0)])
    G=np.array([np.interp(t,I[:,0],ci[:,j]) for j in range(3)]).T
    gw=G[k:]-G[:-k]
    A=gw[good]; B=rot[good]
    # remove bias: include bias*dt term -> fit B = M A + b*dt
    X=np.hstack([A,dtw[good][:,None]])
    sol,_,_,_=np.linalg.lstsq(X,B,rcond=None); M=sol[:3].T; b=sol[3]
    U,s,Vt=np.linalg.svd(M); R=U@np.diag([1,1,np.sign(np.linalg.det(U@Vt))])@Vt
    ang=np.degrees(np.arccos(np.clip((np.trace(R)-1)/2,-1,1)))
    r0=np.sqrt(np.mean(np.sum((B-A)**2,1))); r1=np.sqrt(np.mean(np.sum((B-X@sol)**2,1)))
    print(f'{path.split("/")[-1]}: {good.sum()} windows; best gyro->SLAM rotation {ang:.2f} deg; matrix singular values (scale) {np.round(s,4)}; bias {np.round(np.degrees(b),3)} deg/s; mismatch rms {np.degrees(r0):.3f} -> {np.degrees(r1):.3f} deg per 100 ms')
    print('   M =',np.round(M,4).tolist())
