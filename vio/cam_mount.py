#!/usr/bin/env python3
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
"""Each camera's mount on the head as the driver computes it: CAMPOSE lines (ct_replay CT_REPLAY_DUMP_CAM_POSES,
camera world poses in OpenXR camera axes, host ns) against the record file's headset poses (S lines), scanning the
time offset for the most constant mount. Usage: cam_mount.py campose.csv record.csv"""
import sys, numpy as np
def qmul(a,b):
    x1,y1,z1,w1=np.moveaxis(a,-1,0); x2,y2,z2,w2=np.moveaxis(b,-1,0)
    return np.stack([w1*x2+x1*w2+y1*z2-z1*y2, w1*y2-x1*z2+y1*w2+z1*x2, w1*z2+x1*y2-y1*x2+z1*w2, w1*w2-x1*x2-y1*y2-z1*z2],-1)
def qinv(q): return q*np.array([-1,-1,-1,1])
def rotv(q,v):
    u=q[...,:3]; w=q[...,3:4]; t=2*np.cross(u,v); return v+w*t+np.cross(u,t)
def qavg(Q):
    Q=Q*np.sign(Q@Q[0])[:,None]; w,V=np.linalg.eigh(Q.T@Q); return V[:,-1]*np.sign(V[3,-1])
C=np.genfromtxt(sys.argv[1],delimiter=',')[:,1:]
S=[]
for l in open(sys.argv[2]):
    if l.startswith('S,'): S.append([float(x) for x in l.split(',')[1:10]])
S=np.array(S); S=S[np.argsort(S[:,0])]
# use the longest stretch without SLAM jumps (> 3 cm between samples)
bad=np.where(np.linalg.norm(np.diff(S[:,2:5],axis=0),axis=1)>0.03)[0]
cuts=np.concatenate([[0],bad+1,[len(S)]]); k=np.argmax(np.diff(cuts)); S=S[cuts[k]:cuts[k+1]]
def head_at(t):
    i=np.clip(np.searchsorted(S[:,0],t),1,len(S)-1); a=(t-S[i-1,0])/(S[i,0]-S[i-1,0])
    p=S[i-1,2:5]*(1-a)[:,None]+S[i,2:5]*a[:,None]
    q0=S[i-1,5:9]; q1=S[i,5:9]*np.sign(np.sum(S[i-1,5:9]*S[i,5:9],1))[:,None]
    q=q0*(1-a)[:,None]+q1*a[:,None]; q/=np.linalg.norm(q,axis=1)[:,None]; return p,q, (t>S[0,0])&(t<S[-1,0])
res={}
for cam in range(4):
    c=C[C[:,0]==cam]; t=c[:,1]; pc=c[:,2:5]; qc=c[:,5:9]
    best=None
    for off_ms in np.arange(-60,61,2):
        ph,qh,ok=head_at(t+off_ms*1e6)
        if ok.sum()<50: continue
        qi=qinv(qh[ok]); q_hc=qmul(qi,qc[ok]); p_hc=rotv(qi,pc[ok]-ph[ok])
        qa=qavg(q_hc); ang=np.degrees(2*np.arccos(np.clip(np.abs(q_hc@qa),0,1)))
        spread=np.median(ang)+np.median(np.linalg.norm(p_hc-np.median(p_hc,0),axis=1))*100
        if best is None or spread<best[0]: best=(spread,off_ms,qa,np.median(p_hc,0),np.median(ang),ok.sum(),np.median(np.linalg.norm(p_hc-np.median(p_hc,0),axis=1)))
    sp,off,qa,pa,angs,n,ps=best; res[cam]=(qa,pa)
    print(f'camera {cam}: {n} samples, best offset {off:+.0f} ms; mount in head frame q(xyzw) {np.round(qa,5)} p {np.round(pa*1000,1)} mm; spread {angs:.3f} deg / {ps*1000:.2f} mm')
q0,p0=res[0]; q1,p1=res[1]
qr=qmul(qinv(q0),q1); pr=rotv(qinv(q0),p1-p0)
print(f'check cam0->cam1 from driver poses: rotation {np.degrees(2*np.arccos(min(1,abs(qr[3])))):.2f} deg, distance {np.linalg.norm(pr)*1000:.1f} mm (calibration: 30.19 deg, 78.9 mm)')
np.save(sys.argv[1]+'.mount.npy',np.array([np.r_[res[c][0],res[c][1]] for c in range(4)]))
