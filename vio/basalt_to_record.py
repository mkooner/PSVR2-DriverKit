#!/usr/bin/env python3
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
"""Record file with the headset's SLAM poses (S lines) replaced by Basalt's, aligned to the SLAM world frame, arriving
as late as they would live (image transfer latency from the camera recording + Basalt's processing time), so
steamvr_sim shows what the driver would output with Basalt as its pose source.
Usage: basalt_to_record.py record.csv camera.pv2c basalt_tracking.csv processing_ms out.csv"""
import sys, struct, numpy as np
rec, cam, trk, proc_ms, out = sys.argv[1], sys.argv[2], sys.argv[3], float(sys.argv[4]), sys.argv[5]
def qmul(a,b):
    x1,y1,z1,w1=a.T; x2,y2,z2,w2=b.T
    return np.stack([w1*x2+x1*w2+y1*z2-z1*y2, w1*y2-x1*z2+y1*w2+z1*x2, w1*z2+x1*y2-y1*x2+z1*w2, w1*w2-x1*x2-y1*y2-z1*z2],1)
def qinv(q): return q*np.array([-1,-1,-1,1])
def q2R(q):
    x,y,z,w=q; return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],[2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],[2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
B=np.genfromtxt(trk,delimiter=',',skip_header=1); B=B[B[:,4]!=42]
bt=B[:,0]; bp=B[:,1:4]; bq=np.stack([B[:,5],B[:,6],B[:,7],B[:,4]],1)
lines=open(rec).read().splitlines()
S=np.array([[float(x) for x in l.split(',')[1:10]] for l in lines if l.startswith('S,')]); S=S[np.argsort(S[:,1])]
# image transfer latency: arrival host time - header VTS (us), median
lat=[]; 
with open(cam,'rb') as f:
    f.read(8)
    for i in range(600):
        h=f.read(16)
        if len(h)<16: break
        ns,L,d=struct.unpack('<QII',h); hd=f.read(16); f.seek(L-16,1); lat.append(ns-struct.unpack('<I',hd[8:12])[0]*1000)
# host-vts offset of the SLAM stream (its own arrival), to put Basalt's arrival on the same host clock
off_s=np.median(S[:,0]-S[:,1]); off_img=np.median(lat)
# align Basalt world to SLAM world: rotation from orientation pairs (body frames match within 0.3 deg), then translation
i=np.searchsorted(S[:,1],bt); ok=(i>0)&(i<len(S))&(bt>bt[0]+3e9); i=i[ok]
qs=S[i,5:9]; qb=bq[ok]; D=qmul(qs,qinv(qb)); D=D*np.sign(D[:,3:4]); Rw=np.linalg.eigh(D.T@D)[1][:,-1]
R=q2R(Rw); t=np.median(S[i,2:5]-(bp[ok]@R.T),0)
ang=np.degrees(2*np.arccos(np.clip(np.abs(np.sum(qmul(qinv(qs),qmul(np.tile(Rw,(len(qb),1)),qb))*np.array([0,0,0,1]),1)),0,1)))
print(f'image arrival latency median {off_img/1e6:.1f} ms after exposure (headset SLAM arrives {off_s/1e6 - (off_s-off_img)/1e6:.1f}); world alignment residual median {np.median(ang):.2f} deg')
qa=qmul(np.tile(Rw,(len(bq),1)),bq); pa=bp@R.T+t
host=bt+off_img+proc_ms*1e6
outl=[l for l in lines if not l.startswith('S,')]
outl+= [f'S,{int(h)},{int(v)},{p[0]:.6f},{p[1]:.6f},{p[2]:.6f},{q[0]:.7f},{q[1]:.7f},{q[2]:.7f},{q[3]:.7f}' for h,v,p,q in zip(host,bt,pa,qa)]
open(out,'w').write('\n'.join(outl)+'\n')
print(f'wrote {len(bt)} Basalt poses as S lines, arriving {off_img/1e6+proc_ms:.1f} ms after exposure, to {out}')
