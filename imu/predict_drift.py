#!/usr/bin/env python3
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
"""How far an accelerometer-only position prediction drifts, against the headset's SLAM position H later, for variants
of the accelerometer handling (gravity estimate, bias, lever arm). Clean SLAM stretches only.
Usage: predict_drift.py record.csv [...]"""
import sys, numpy as np
G=9.79873
BIAS=np.array([0.1515,0.1924,0.0256])  # accel_calib.py, bias model, 7 recordings
def rotv(q,v):
    u=q[...,:3]; w=q[...,3:4]; t=2*np.cross(u,v); return v+w*t+np.cross(u,t)
def qmul(a,b):
    x1,y1,z1,w1=a.T; x2,y2,z2,w2=b.T
    return np.stack([w1*x2+x1*w2+y1*z2-z1*y2, w1*y2-x1*z2+y1*w2+z1*x2, w1*z2+x1*y2-y1*x2+z1*w2, w1*w2-x1*x2-y1*y2-z1*z2],1)
def qinv(q): return q*np.array([-1,-1,-1,1])
def qlog(q):
    q=q*np.sign(q[:,3:4]+1e-12); v=q[:,:3]; s=np.linalg.norm(v,axis=1); a=2*np.arctan2(s,q[:,3])
    return v/np.maximum(s,1e-12)[:,None]*a[:,None]
VARIANTS={'current (1 s gravity average)':dict(grav='avg',bias=False,lever=None),
          'known gravity':dict(grav='known',bias=False,lever=None),
          'known gravity + bias':dict(grav='known',bias=True,lever=None)}
for lev in [(0,0.02,0),(0,-0.02,0),(0,0,0.03),(0,0,-0.03),(0.02,0,0)]:
    VARIANTS[f'known gravity + bias + lever {lev}']=dict(grav='known',bias=True,lever=np.array(lev))
res={k:{} for k in VARIANTS}
for path in sys.argv[1:]:
    I=[];S=[]
    for l in open(path):
        if l.startswith('I,'): p=l.split(','); I.append([float(x) for x in p[2:9]])
        elif l.startswith('S,'): p=l.split(','); S.append([float(x) for x in p[2:10]])
    I=np.array(I); _,u=np.unique(I[:,0],return_index=True); I=I[u]
    S=np.array(S); S=S[np.argsort(S[:,0])]; _,u=np.unique(S[:,0],return_index=True); S=S[u]
    S=S[S[:,0]>S[0,0]+20e9]
    bad=np.where((np.diff(S[:,0])>50e6)|(np.linalg.norm(np.diff(S[:,1:4],axis=0),axis=1)>0.02))[0]
    good=np.ones(len(S),bool)
    for b in bad: good[max(0,b-30):b+40]=False   # keep away from SLAM glitches (truth must be clean)
    dt=1/1000; g=np.arange(S[0,0],S[-1,0],dt*1e9)
    ip=lambda T,X: np.array([np.interp(g,T,X[:,j]) for j in range(X.shape[1])]).T
    P=ip(S[:,0],S[:,1:4]); Q=ip(S[:,0],S[:,4:8]); Q/=np.linalg.norm(Q,axis=1)[:,None]
    A=ip(I[:,0],I[:,4:7]); W=ip(I[:,0],I[:,1:4]); okg=np.interp(g,S[:,0],good.astype(float))>0.999
    Ww=rotv(Q,W); Al=np.gradient(Ww,dt,axis=0)
    # velocity at start from SLAM, smoothed over +-33 ms (the same for every variant)
    k=33; V=(np.roll(P,-k,0)-np.roll(P,k,0))/(2*k*dt)
    for name,v in VARIANTS.items():
        a=A-BIAS if v['bias'] else A
        aw=rotv(Q,a)
        if v['grav']=='avg':
            gr=np.zeros_like(aw); gr[0]=aw[0]; kk=dt/(1.0+dt)
            for i in range(1,len(aw)): gr[i]=gr[i-1]+(aw[i]-gr[i-1])*kk
            lin=aw-gr
        else:
            lin=aw-np.array([0,G,0])
        if v['lever'] is not None:
            r=rotv(Q,np.broadcast_to(v['lever'],Q[:,:3].shape))  # SLAM point rel. accelerometer, head frame
            lin=lin+np.cross(Al,r)+np.cross(Ww,np.cross(Ww,r))
        for H in (0.016,0.05,0.25):
            n=int(H/dt); starts=np.arange(2000,len(g)-n-50,40); starts=starts[okg[starts]&okg[starts+n]]
            errs=[];spd=[]
            for s in starts:
                vv=V[s].copy(); pp=P[s].copy()
                acc_seg=lin[s:s+n]
                vel=vv+np.cumsum(acc_seg,0)*dt; pos=pp+np.cumsum(vel,0)*dt
                errs.append(np.linalg.norm(pos[-1]-P[s+n])); spd.append(np.degrees(np.linalg.norm(W[s:s+n],axis=1).max()))
            res[name].setdefault(H,[]).append((np.array(errs),np.array(spd)))
for name in VARIANTS:
    line=f'{name:44s}'
    for H in (0.016,0.05,0.25):
        e=np.concatenate([x[0] for x in res[name][H]])*1000; s=np.concatenate([x[1] for x in res[name][H]])
        slow=s<30; fast=s>=60
        line+=f' | {int(H*1000)} ms: all {np.median(e):.2f}, slow {np.median(e[slow]):.2f}, fast {np.median(e[fast]) if fast.sum()>10 else float("nan"):.2f}'
    print(line+'  (median mm)')
