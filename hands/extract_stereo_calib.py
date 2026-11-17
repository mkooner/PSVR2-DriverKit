#!/usr/bin/env python3
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
"""Stereo calibration of the PS VR2 bottom camera pair (cameras 0 and 1) for Mercury, from a constellation tracker
dataset (RECORD_DATASET=1): the intrinsics and T_DISTORTION_PSVR2 coefficients the tracker uses, and the camera 0 ->
camera 1 transform from the camera world poses of frames both cameras saw at the same time.

Writes JSON: views[0..1] {width, height, K (3x3), dist (20)}, and the OpenCV stereo extrinsics R (3x3), T (3) with
x_cam1 = R x_cam0 + T (OpenCV camera axes: x right, y down, z forward), plus the baseline and both cameras' poses in
camera 0 for checking.
Usage: extract_stereo_calib.py dataset out.json
"""
import json, sys
import numpy as np
sys.path.insert(0, __file__.rsplit('/', 2)[0] + '/analysis')
import ctdataset

def qmat(q):
    x, y, z, w = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])

cams, samples, devices = ctdataset.load(sys.argv[1])
by_ts = {}
for s in samples:
    if s['cam_pose'] is not None and s['cam'] in (0, 1):
        by_ts.setdefault(s['ts'], {})[s['cam']] = s['cam_pose']
Rs, Ts = [], []
F = np.diag([1.0, -1.0, -1.0])  # OpenXR camera axes <-> OpenCV camera axes
for ts, d in by_ts.items():
    if 0 in d and 1 in d:
        (p0, q0), (p1, q1) = d[0], d[1]
        R0, R1 = qmat(q0), qmat(q1)               # camera-to-world, OpenXR axes
        # cam0 point -> world -> cam1, in OpenXR axes, then into OpenCV axes on both sides
        R_xr = R1.T @ R0
        T_xr = R1.T @ (p0 - p1)
        Rs.append(F @ R_xr @ F)
        Ts.append(F @ T_xr)
if not Rs:
    sys.exit('no frame with both camera 0 and camera 1 poses')
Ts = np.array(Ts)
# All frames share one rigid relation; take the medians and report the spread as a check.
T = np.median(Ts, axis=0)
U, _, Vt = np.linalg.svd(np.mean(Rs, axis=0))
R = U @ Vt
spread_mm = np.percentile(np.linalg.norm(Ts - T, axis=1), 99) * 1000
views = []
for i in (0, 1):
    c = cams[i]
    views.append(dict(width=c['w'], height=c['h'], K=c['K'].tolist(), dist=list(c['dist']), model=c['model']))
ang = np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))
out = dict(source=sys.argv[1], views=views, R=R.tolist(), T=T.tolist(), baseline_m=float(np.linalg.norm(T)),
           relative_rotation_deg=float(ang), frames_used=len(Rs), translation_spread_p99_mm=float(spread_mm))
json.dump(out, open(sys.argv[2], 'w'), indent=1)
print(f'{len(Rs)} shared frames; baseline {np.linalg.norm(T)*1000:.1f} mm (spread p99 {spread_mm:.2f} mm), '
      f'relative rotation {ang:.1f} deg; T {np.round(T*1000,1)} mm; views {views[0]["width"]}x{views[0]["height"]}, '
      f'fx {views[0]["K"][0][0]:.1f} cx {views[0]["K"][0][2]:.1f} cy {views[0]["K"][1][2]:.1f}, model {views[0]["model"]}')
