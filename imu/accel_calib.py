#!/usr/bin/env python3
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
"""Headset accelerometer bias + per-axis scale from quasi-static samples: | diag(s) (a - b) | = g at every orientation.
Usage: accel_calib.py record.csv [...]   (driver head axes, m/s^2)"""
import sys, numpy as np
G=9.79873  # local gravity stored in the headset's IMU calibration block (0x0c float 29)
A=[]
for path in sys.argv[1:]:
    I=[]
    for l in open(path):
        if l.startswith('I,'): p=l.split(','); I.append([float(x) for x in p[2:9]])
    I=np.array(I); _,u=np.unique(I[:,0],return_index=True); I=I[u]
    w=np.linalg.norm(I[:,1:4],axis=1); a=I[:,4:7]
    # 50 ms windows: still if gyro < 3 deg/s and accel std < 0.05 m/s^2
    n=100; k=0
    for s in range(0,len(I)-n,n):
        if w[s:s+n].max()<np.radians(3) and a[s:s+n].std(0).max()<0.05:
            A.append(a[s:s+n].mean(0)); k+=1
    print(f'{path.split("/")[-1]}: {k} still windows')
A=np.array(A); print(f'{len(A)} still windows; gravity direction spread: x {A[:,0].min():.1f}..{A[:,0].max():.1f}, y {A[:,1].min():.1f}..{A[:,1].max():.1f}, z {A[:,2].min():.1f}..{A[:,2].max():.1f} m/s2')
q=np.array([0,0,0,1,1,1.0])
def res(q): return np.linalg.norm((A-q[:3])*q[3:],axis=1)-G
for it in range(50):
    r=res(q); J=np.empty((len(A),6))
    for j in range(6):
        d=np.zeros(6); d[j]=1e-6; J[:,j]=(res(q+d)-r)/1e-6
    step=np.linalg.lstsq(J,-r,rcond=None)[0]; q=q+step
    if np.linalg.norm(step)<1e-10: break
r0=np.linalg.norm(A,axis=1)-G; r1=res(q)
print(f'|a|-g before: mean {r0.mean():+.3f} rms {np.sqrt(np.mean(r0**2)):.3f} m/s2; after: mean {r1.mean():+.4f} rms {np.sqrt(np.mean(r1**2)):.4f}')
print(f'bias {np.round(q[:3],4)} m/s2, scale {np.round(q[3:],5)}')
cov=np.linalg.inv(J.T@J)*np.mean(r1**2); print('1-sigma uncertainty: bias', np.round(np.sqrt(np.diag(cov))[:3],4), 'scale', np.round(np.sqrt(np.diag(cov))[3:],5))
