# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
# Usage: imu_offset.py record.csv [...]  -- accelerometer position relative to the SLAM point, velocity domain.
# Velocity-domain fit, band 0.7-6 Hz: v_slam = v_imu + w x (p_slam - p_imu), p_slam - p_imu = R e  (e: SLAM point rel. to IMU, body frame)
# v_imu = integral(R a - g). Also scans a time shift of SLAM vs IMU.
import sys, numpy as np
def rotv(q,v):
    u=q[...,:3]; w=q[...,3:4]; t=2*np.cross(u,v); return v+w*t+np.cross(u,t)
def skew(v): return np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])
def qmat(q):
    x,y,z,w=q; return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
for path in sys.argv[1:]:
    I=[];S=[]
    for l in open(path):
        if l.startswith('I,'): p=l.split(','); I.append([float(x) for x in p[2:9]])
        elif l.startswith('S,'): p=l.split(','); S.append([float(x) for x in p[2:10]])
    I=np.array(I);S=np.array(S);S=S[np.argsort(S[:,0])]
    _,u=np.unique(S[:,0],return_index=True);S=S[u]
    # drop big SLAM jumps region? keep simple: use longest segment without >5cm jumps
    jumps=np.where(np.linalg.norm(np.diff(S[:,1:4],axis=0),axis=1)>0.05)[0]
    cuts=np.concatenate([[0],jumps+1,[len(S)]]); seg=np.argmax(np.diff(cuts)); S=S[cuts[seg]:cuts[seg+1]]
    dt=0.005; fs=1/dt
    def band(x,lo=0.7,hi=6.0):
        X=np.fft.rfft(x,axis=0); f=np.fft.rfftfreq(len(x),dt); X[(f<lo)|(f>hi)]=0; return np.fft.irfft(X,len(x),axis=0)
    best=None
    for shift_ms in range(-30,31,2):
        sh=shift_ms*1e6
        t0=max(I[0,0],S[0,0]+sh)+2e9; t1=min(I[-1,0],S[-1,0]+sh)-1e9
        if t1-t0<5e9: continue
        g=np.arange(t0,t1,dt*1e9)
        ip=lambda t,x: np.array([np.interp(g,t,x[:,j]) for j in range(x.shape[1])]).T
        pos=ip(S[:,0]+sh,S[:,1:4]); q=ip(S[:,0]+sh,S[:,4:8]); q/=np.linalg.norm(q,axis=1)[:,None]
        acc=ip(I[:,0],I[:,4:7]); gyr=ip(I[:,0],I[:,1:4])
        aw=rotv(q,acc); aw-=aw.mean(0)
        v_imu=band(np.cumsum(aw,0)*dt); v_slam=band(np.gradient(pos,dt,axis=0))
        w_w=rotv(q,gyr)
        # w x (R e) = skew(w) R e ; band each column of the regressor
        A=np.zeros((len(g),3,3))
        for i in range(len(g)): A[i]=skew(w_w[i])@qmat(q[i])
        A=np.stack([band(A[:,:,k]) for k in range(3)],axis=2)  # (n,3,3)
        b=v_slam-v_imu
        Af=A.reshape(-1,3); bf=b.reshape(-1)
        e,_,_,_=np.linalg.lstsq(Af,bf,rcond=None)
        r0=np.sqrt(np.mean(b**2)); r1=np.sqrt(np.mean((bf-Af@e)**2))
        if best is None or r1<best[0]: best=(r1,shift_ms,e,r0,np.sqrt(np.mean(v_slam**2)))
    r1,sh,e,r0,vs=best
    print(f'{path.split("/")[-1]}: best SLAM time shift {sh:+d} ms; SLAM point rel. to accelerometer e = ({e[0]*100:+.1f}, {e[1]*100:+.1f}, {e[2]*100:+.1f}) cm (head frame, |e| {np.linalg.norm(e)*100:.1f}); velocity mismatch rms {r0*100:.1f} -> {r1*100:.1f} cm/s (SLAM band velocity rms {vs*100:.1f} cm/s)')
