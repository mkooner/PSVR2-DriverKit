#!/usr/bin/env python3
"""Same-moment cross-camera disagreement from a tracker dataset, and the camera 1 correction that removes it.
usage (from psvr2-test): analysis/cross_camera.py logs/dataset-<date>"""
import sys; sys.path.insert(0,'analysis')
import numpy as np, ctdataset as D
def qmul(a,b):
    ax,ay,az,aw=a; bx,by,bz,bw=b
    return np.array([aw*bx+ax*bw+ay*bz-az*by, aw*by-ax*bz+ay*bw+az*bx, aw*bz+ax*by-ay*bx+az*bw, aw*bw-ax*bx-ay*by-az*bz])
QF=np.array([1.0,0,0,0])
cams,S,dev=D.load(sys.argv[1])
def world(s,pose):
    cp,cq=s['cam_pose']; p,q=pose
    return cp+D.qrot(cq,D.qrot(QF,p)), qmul(qmul(cq,QF),q)
fc=np.zeros(4,int)
by={}
for s in S:
    if s['cam_pose'] is None: continue
    for d,st in s['states'].items():
        if st['found'] is None: continue
        fc[s['cam']]+=1
        nb=sum(1 for b in s['blobs'] if b['dev']==d)
        by.setdefault(d,[]).append((s['ts'],s['cam'],world(s,st['found']),st['found'],nb,s))
print("found poses per camera", fc)
for d,L in by.items():
    L.sort(key=lambda x:x[0]); pr={}
    for i in range(len(L)):
        for j in range(i+1,len(L)):
            if L[j][0]-L[i][0]>3_000_000: break
            if L[j][1]!=L[i][1]:
                a,b=(L[i],L[j]) if L[i][1]<L[j][1] else (L[j],L[i])
                k=(a[1],b[1]); dw=b[2][0]-a[2][0]
                # along each camera's ray
                ra=a[3][0]/np.linalg.norm(a[3][0]); 
                pr.setdefault(k,[]).append((np.linalg.norm(dw),a[4],b[4],np.linalg.norm(a[3][0]),np.linalg.norm(b[3][0]),(b[0]-a[0])/1e6))
    for k,v in sorted(pr.items()):
        v=np.array(v); print(f"dev {d} cams {k}: {len(v)} same-moment pairs (dt median {np.median(v[:,5]):.2f} ms) | world disagreement median {np.median(v[:,0])*100:.1f} cm p90 {np.percentile(v[:,0],90)*100:.1f} | distance cam a {np.median(v[:,3]):.2f} m, b {np.median(v[:,4]):.2f} m")
def inv(q): return np.array([-q[0],-q[1],-q[2],q[3]])
def ang(a,b): return 2*np.degrees(np.arccos(min(1,abs(np.dot(a,b)))))
print(" -- fit: cam1 extrinsic correction (in cam0's frame)")
A=[];B=[];QA=[];QB=[]
for d,L in by.items():
    for i in range(len(L)):
        for j in range(i+1,len(L)):
            if L[j][0]-L[i][0]>500_000: break
            if {L[i][1],L[j][1]}=={0,1}:
                a,b=(L[i],L[j]) if L[i][1]==0 else (L[j],L[i])
                cp,cq=a[5]['cam_pose']   # cam0 pose at that time
                A.append(D.qrot(inv(cq),a[2][0]-cp)); B.append(D.qrot(inv(cq),b[2][0]-cp))
                QA.append(qmul(inv(cq),a[2][1])); QB.append(qmul(inv(cq),b[2][1]))
A=np.array(A);B=np.array(B)
ma,mb=A.mean(0),B.mean(0); H=(B-mb).T@(A-ma); U,s,Vt=np.linalg.svd(H); Dg=np.diag([1,1,np.sign(np.linalg.det(Vt.T@U.T))]); Rm=Vt.T@Dg@U.T; t=ma-Rm@mb
res=np.linalg.norm(A-(B@Rm.T+t),axis=1)
rang=np.degrees(np.arccos((np.trace(Rm)-1)/2))
dq=[ang(a,b) for a,b in zip(QA,QB)]
print(f"   pairs {len(A)} | before: mean offset (cam0 frame) {np.round((A-B).mean(0)*100,2)} cm, |.| median {np.median(np.linalg.norm(A-B,axis=1))*100:.2f}, orientation disagreement median {np.median(dq):.2f} deg")
print(f"   translation-only fix: residual median {np.median(np.linalg.norm(A-B-(A-B).mean(0),axis=1))*100:.2f} cm")
print(f"   rigid fix: rotation {rang:.2f} deg, translation {np.round(t*100,2)} cm -> residual median {np.median(res)*100:.2f} cm p90 {np.percentile(res,90)*100:.2f}")
# where are cams in the head frame (relative), from first sample poses of cam0 and cam1 close in time
c0=[s for s in S if s['cam']==0 and s['cam_pose'] is not None][100]; c1=min((s for s in S if s['cam']==1 and s['cam_pose'] is not None), key=lambda s:abs(s['ts']-c0['ts']))
rel=D.qrot(inv(c0['cam_pose'][1]),c1['cam_pose'][0]-c0['cam_pose'][0])
print(f"   recorded cam1 position in cam0 frame {np.round(rel*100,2)} cm (baseline {np.linalg.norm(rel)*100:.1f} cm), relative rotation {ang(c0['cam_pose'][1],c1['cam_pose'][1]):.1f} deg")
