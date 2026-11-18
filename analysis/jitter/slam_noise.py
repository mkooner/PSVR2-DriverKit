# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
# Usage: slam_noise.py record.csv [...]  -- SLAM pose noise vs the IMU per frequency band (upper bound).
import sys, numpy as np
def rotv(q,v):
    u=q[...,:3]; w=q[...,3:4]; t=2*np.cross(u,v); return v+w*t+np.cross(u,t)
def qmul(a,b):
    x1,y1,z1,w1=a.T; x2,y2,z2,w2=b.T
    return np.stack([w1*x2+x1*w2+y1*z2-z1*y2, w1*y2-x1*z2+y1*w2+z1*x2, w1*z2+x1*y2-y1*x2+z1*w2, w1*w2-x1*x2-y1*y2-z1*z2],1)
def qinv(q): return q*np.array([-1,-1,-1,1])
def qlog(q):
    q=q*np.sign(q[:,3:4]+1e-12); v=q[:,:3]; s=np.linalg.norm(v,axis=1); ang=2*np.arctan2(s,q[:,3])
    return v/np.maximum(s,1e-12)[:,None]*ang[:,None]
BANDS=[(1,3),(3,8),(8,15),(15,28)]
def welch_bands(x, dt, scale_fn, n=480):
    # x: (N,3) signal; returns rms per band of scale_fn(f)*X(f)
    win=np.hanning(n); f=np.fft.rfftfreq(n,dt); acc=np.zeros(len(f)); m=0
    for s in range(0,len(x)-n,n//2):
        c=x[s:s+n]; tt=np.arange(n); c=c-np.polyval(np.polyfit(tt,c,1),tt[:,None]) if False else c-c.mean(0)
        X=np.fft.rfft(c*win[:,None],axis=0)*scale_fn(f)[:,None]
        acc+=np.sum(np.abs(X)**2,1); m+=1
    psd=acc/m/np.sum(win**2)/n*2  # Parseval: mean square = sum|X|^2/(n*sum w^2), one-sided x2
    return [np.sqrt(np.sum(psd[(f>=lo)&(f<hi)])) for lo,hi in BANDS]
for path in sys.argv[1:]:
    I=[];S=[]
    for l in open(path):
        if l.startswith('I,'): p=l.split(','); I.append([float(x) for x in p[2:9]])
        elif l.startswith('S,'): p=l.split(','); S.append([float(x) for x in p[2:10]])
    I=np.array(I);S=np.array(S);S=S[np.argsort(S[:,0])]; _,u=np.unique(S[:,0],return_index=True);S=S[u]
    _,u=np.unique(I[:,0],return_index=True); I=I[u]
    S=S[S[:,0]>S[0,0]+15e9]
    bad=np.where((np.diff(S[:,0])>50e6)|(np.linalg.norm(np.diff(S[:,1:4],axis=0),axis=1)>0.03))[0]
    cuts=np.concatenate([[0],bad+1,[len(S)]]); k=np.argmax(np.diff(cuts)); S=S[cuts[k]:cuts[k+1]]
    dt=1/240; g=np.arange(S[0,0]+0.2e9,S[-1,0]-0.2e9,dt*1e9)
    ip=lambda t,x: np.array([np.interp(g,t,x[:,j]) for j in range(x.shape[1])]).T
    pos=ip(S[:,0],S[:,1:4]); q=ip(S[:,0],S[:,4:8]); q/=np.linalg.norm(q,axis=1)[:,None]
    acc=ip(I[:,0],I[:,4:7]); gyr=ip(I[:,0],I[:,1:4])
    aw=rotv(q,acc)
    a_slam=np.gradient(np.gradient(pos,dt,axis=0),dt,axis=0)
    w_slam=qlog(qmul(q[1:],qinv(q[:-1])))/dt; w_slam=np.vstack([w_slam,w_slam[-1:]])
    w_imu=rotv(q,gyr)
    inv2=lambda f: 1/np.maximum((2*np.pi*f)**2,1e-9); inv1=lambda f: 1/np.maximum(2*np.pi*f,1e-9)
    dpos=welch_bands(a_slam-aw,dt,inv2); spos=welch_bands(a_slam,dt,inv2)
    drot=welch_bands(w_slam-w_imu,dt,inv1); srot=welch_bands(w_imu,dt,inv1)
    fmt=lambda v,s,k: ', '.join(f'{lo}-{hi} Hz {a*k:.3f} (motion {b*k:.2f})' for (lo,hi),a,b in zip(BANDS,v,s))
    print(f'{path.split("/")[-1]}: {len(g)*dt:.0f} s clean SLAM, head speed median {np.degrees(np.median(np.linalg.norm(gyr,axis=1))):.0f} deg/s')
    print('  SLAM position not explained by accelerometer, rms mm: '+fmt(dpos,spos,1000))
    print('  SLAM rotation not explained by gyro, rms deg:          '+fmt(drot,srot,180/np.pi))
