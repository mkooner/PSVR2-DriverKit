#!/usr/bin/env python3
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
"""Head pose quality of Basalt vs the headset's own SLAM pose, both on the headset clock (VTS ns), measured against the
IMU (EuRoC imu0, the same samples Basalt used): rotation vs gyro, per-band position wobble the accelerometer didn't see
and rotation wobble the gyro didn't see (Welch, Hann 2 s), jumps off a smooth path by turning speed, Basalt cost.
Usage: benchmark.py euroc_dir basalt_out_dir record.csv"""
import sys, numpy as np
def qmul(a,b):
    x1,y1,z1,w1=a.T; x2,y2,z2,w2=b.T
    return np.stack([w1*x2+x1*w2+y1*z2-z1*y2, w1*y2-x1*z2+y1*w2+z1*x2, w1*z2+x1*y2-y1*x2+z1*w2, w1*w2-x1*x2-y1*y2-z1*z2],1)
def qinv(q): return q*np.array([-1,-1,-1,1])
def qlog(q):
    q=q*np.sign(q[:,3:4]+1e-12); v=q[:,:3]; s=np.linalg.norm(v,axis=1); a=2*np.arctan2(s,q[:,3])
    return v/np.maximum(s,1e-12)[:,None]*a[:,None]
def rotv(q,v):
    u=q[...,:3]; w=q[...,3:4]; t=2*np.cross(u,v); return v+w*t+np.cross(u,t)
BANDS=[(1,3),(3,8),(8,15),(15,28)]
def welch(x,dt,scale,n=480):
    win=np.hanning(n); f=np.fft.rfftfreq(n,dt); acc=np.zeros(len(f)); m=0
    for s in range(0,len(x)-n,n//2):
        c=x[s:s+n]-x[s:s+n].mean(0); X=np.fft.rfft(c*win[:,None],axis=0)*scale(f)[:,None]; acc+=np.sum(np.abs(X)**2,1); m+=1
    psd=acc/max(m,1)/np.sum(win**2)/n*2
    return [np.sqrt(np.sum(psd[(f>=lo)&(f<hi)])) for lo,hi in BANDS]
eu,bo,rec=sys.argv[1:4]
I=np.genfromtxt(eu+'/mav0/imu0/data.csv',delimiter=',',skip_header=1); ti=I[:,0]; gyr=I[:,1:4]; acc=I[:,4:7]
B=np.genfromtxt(bo+'/tracking.csv',delimiter=',',skip_header=1); B=B[B[:,4]!=42]
srcs={'Basalt':(B[:,0],B[:,1:4],np.stack([B[:,5],B[:,6],B[:,7],B[:,4]],1))}
S=[]
for l in open(rec):
    if l.startswith('S,'): S.append([float(x) for x in l.split(',')[2:10]])
S=np.array(S); S=S[np.argsort(S[:,0])]; _,u=np.unique(S[:,0],return_index=True); S=S[u]
S=S[(S[:,0]>=ti[0])&(S[:,0]<=ti[-1])]
srcs['headset']=(S[:,0],S[:,1:4],S[:,4:8]/np.linalg.norm(S[:,4:8],axis=1)[:,None])
t0=max(srcs['Basalt'][0][0],srcs['headset'][0][0])+2e9; t1=min(srcs['Basalt'][0][-1],srcs['headset'][0][-1])
print(f'common span {(t1-t0)*1e-9:.1f} s (first 2 s of Basalt skipped for initialisation); Basalt {len(B)} poses, headset {len(S)}')
ci=np.vstack([np.zeros((1,3)),np.cumsum(gyr[1:]*np.diff(ti)[:,None]*1e-9,axis=0)])
for name,(t,p,q) in srcs.items():
    m=(t>=t0)&(t<=t1); t,p,q=t[m],p[m],q[m]
    print(f'== {name}: {len(t)} poses, spacing median {np.median(np.diff(t))/1e6:.1f} ms, gaps > 50 ms: {(np.diff(t)>50e6).sum()}, max gap {np.diff(t).max()/1e6:.0f} ms')
    # rotation vs gyro, 100 ms windows (small-angle sum of gyro, body frame)
    k=6; G=np.array([np.interp(t,ti,ci[:,j]) for j in range(3)]).T
    rq=qlog(qmul(qinv(q[:-k]),q[k:])); rg=G[k:]-G[:-k]; turn=np.linalg.norm(rg,axis=1)>np.radians(1)
    err=np.degrees(np.linalg.norm(rq-rg,axis=1)); ratio=np.linalg.norm(rq,axis=1)/np.maximum(np.linalg.norm(rg,axis=1),1e-9)
    print(f'   rotation vs gyro over 100 ms: error median {np.median(err[turn]):.3f} deg, p99 {np.percentile(err[turn],99):.2f} deg (turning windows {turn.sum()}); scale {np.median(ratio[turn]):.3f}')
    # per-band wobble on a 240 Hz grid
    dt=1/240; g=np.arange(t[0],t[-1],dt*1e9)
    ip=lambda T,X: np.array([np.interp(g,T,X[:,j]) for j in range(X.shape[1])]).T
    P=ip(t,p); Q=ip(t,q); Q/=np.linalg.norm(Q,axis=1)[:,None]; A=ip(ti,acc); W=ip(ti,gyr)
    a_pose=np.gradient(np.gradient(P,dt,axis=0),dt,axis=0); a_imu=rotv(Q,A)
    w_pose=np.vstack([qlog(qmul(Q[1:],qinv(Q[:-1])))/dt,np.zeros((1,3))]); w_imu=rotv(Q,W)
    dp=welch(a_pose-a_imu,dt,lambda f:1/np.maximum((2*np.pi*f)**2,1e-9)); dr=welch(w_pose-w_imu,dt,lambda f:1/np.maximum(2*np.pi*f,1e-9))
    print('   position wobble not in accelerometer, mm: '+', '.join(f'{lo}-{hi} Hz {v*1000:.3f}' for (lo,hi),v in zip(BANDS,dp)))
    print('   rotation wobble not in gyro, deg:          '+', '.join(f'{lo}-{hi} Hz {np.degrees(v):.4f}' for (lo,hi),v in zip(BANDS,dr)))
    # jumps: deviation from constant-velocity prediction, by turning speed
    v=np.diff(p,axis=0)/np.diff(t)[:,None]*1e9; pred=p[1:-1]+v[:-1]*(np.diff(t)[1:,None]*1e-9)
    dev=np.linalg.norm(p[2:]-pred,axis=1)*1000
    idx=np.searchsorted(ti,t[2:]); sp=np.degrees(np.array([np.linalg.norm(gyr[max(0,i-100):i+1],axis=1).max() if i>0 else 0 for i in idx]))
    line='   off a smooth path (mm, p99 / share > 5 mm):'
    for lo,hi in [(0,10),(10,30),(30,60),(60,120),(120,1e9)]:
        kk=(sp>=lo)&(sp<hi)
        if kk.sum()>20: line+=f'  {lo}-{hi if hi<1e8 else "max"} deg/s: {np.percentile(dev[kk],99):.1f} / {100*(dev[kk]>5).mean():.1f}%'
    print(line)
T=np.genfromtxt(bo+'/timing.csv',delimiter=',',skip_header=1)
d=np.diff(T[:,1])/1e6; d=d[len(d)//10:]
print(f'Basalt processing (max-speed playback, 1 thread): per frame median {np.median(d):.1f} ms, p90 {np.percentile(d,90):.1f} ms -> {1000/np.median(d):.0f} frames/s (camera runs at 60)')
