# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
# Usage: steamvr_prediction_check.py vrposelog.csv [...]  -- angular velocity frame and SteamVR prediction accuracy.
import sys, numpy as np
def qmul(a,b):
    x1,y1,z1,w1=a.T; x2,y2,z2,w2=b.T
    return np.stack([w1*x2+x1*w2+y1*z2-z1*y2, w1*y2-x1*z2+y1*w2+z1*x2, w1*z2+x1*y2-y1*x2+z1*w2, w1*w2-x1*x2-y1*y2-z1*z2],1)
def qinv(q): return q*np.array([-1,-1,-1,1])
def qlog(q):
    q=q*np.sign(q[:,3:4]+1e-12); v=q[:,:3]; s=np.linalg.norm(v,axis=1); ang=2*np.arctan2(s,q[:,3])
    return v/np.maximum(s,1e-12)[:,None]*ang[:,None]
def rotv(q,v):
    u=q[...,:3]; w=q[...,3:4]; t=2*np.cross(u,v); return v+w*t+np.cross(u,t)
for path in sys.argv[1:]:
    D=np.genfromtxt(path,delimiter=',',names=True,skip_header=1,invalid_raise=False)
    t=D['t_s']; pred=D['pred_s']
    N=np.stack([D['N_qx'],D['N_qy'],D['N_qz'],D['N_qw']],1); P=np.stack([D['P_qx'],D['P_qy'],D['P_qz'],D['P_qw']],1)
    Nw=np.stack([D['N_wx'],D['N_wy'],D['N_wz']],1)
    dt=np.diff(t)
    ok=(dt>0.008)&(dt<0.014)
    w_world=qlog(qmul(N[1:],qinv(N[:-1])))/dt[:,None]
    w_body=qlog(qmul(qinv(N[:-1]),N[1:]))/dt[:,None]
    rep=0.5*(Nw[1:]+Nw[:-1])
    sp=np.linalg.norm(w_world,axis=1); m=ok&(sp>0.3)
    def cmp(a,b): 
        cos=np.sum(a*b,1)/np.linalg.norm(a,axis=1)/np.linalg.norm(b,axis=1); return np.degrees(np.arccos(np.clip(cos,-1,1)))
    print(path.split('/')[-1], f'{m.sum()} turning frames')
    print(f'  reported angular velocity vs actual, angle between axes (median/p90): world frame {np.median(cmp(rep[m],w_world[m])):.1f}/{np.percentile(cmp(rep[m],w_world[m]),90):.1f} deg, head frame {np.median(cmp(rep[m],w_body[m])):.1f}/{np.percentile(cmp(rep[m],w_body[m]),90):.1f} deg; magnitude ratio {np.median(np.linalg.norm(rep[m],axis=1)/sp[m]):.3f}')
    # SteamVR's prediction: P vs N same row
    dP_world=qlog(qmul(P,qinv(N))); dP_body=qlog(qmul(qinv(N),P))
    exp_w=Nw*pred[:,None]
    mm=np.linalg.norm(Nw,axis=1)>0.3
    print(f'  SteamVR prediction rotation P*N^-1 vs reported w*pred: world {np.median(np.linalg.norm(dP_world[mm]-exp_w[mm],axis=1))/np.median(np.linalg.norm(exp_w[mm],axis=1))*100:.0f}% mismatch, head-frame {np.median(np.linalg.norm(dP_body[mm]-exp_w[mm],axis=1))/np.median(np.linalg.norm(exp_w[mm],axis=1))*100:.0f}% mismatch')
    # Prediction accuracy: P at row i vs N interpolated at t_i+pred_i (slerp approx via nearest+lerp normalize)
    tgt=t+pred; j=np.searchsorted(t,tgt); good=(j>0)&(j<len(t))
    jj=j[good]; a=(tgt[good]-t[jj-1])/(t[jj]-t[jj-1])
    q0=N[jj-1]; q1=N[jj]; q1=q1*np.sign(np.sum(q0*q1,1))[:,None]; qi=q0*(1-a)[:,None]+q1*a[:,None]; qi/=np.linalg.norm(qi,axis=1)[:,None]
    err=np.degrees(np.linalg.norm(qlog(qmul(P[good],qinv(qi))),axis=1))
    # compare with "no prediction" (N at i vs truth at t+pred)
    err0=np.degrees(np.linalg.norm(qlog(qmul(N[good],qinv(qi))),axis=1))
    spd=np.degrees(np.linalg.norm(Nw[good],axis=1))
    for lo,hi in [(0,10),(10,60),(60,1e9)]:
        k=(spd>=lo)&(spd<hi)
        if k.sum()>20: print(f'  {lo}-{hi} deg/s: predicted vs actual at photon time p50/p90 {np.median(err[k]):.3f}/{np.percentile(err[k],90):.3f} deg; with no prediction {np.median(err0[k]):.3f}/{np.percentile(err0[k],90):.3f} deg')
