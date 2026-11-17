#!/usr/bin/env python3
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
"""Controller optical tracking gaps from a tracker dataset (RECORD_DATASET=1) and the sense IMU recordings.

Per controller: time to first fix, gaps >100 ms (count, time share, length classes), gyro speed at each loss, and for
each gap whether the controller was in some camera's view (its pose interpolated across the gap, projected with the
camera intrinsics into 1016x1016 images, ignoring distortion beyond a rough radius check).
Usage: ct_gaps.py dataset sense-L.csv sense-R.csv
"""
import sys, collections
import numpy as np
sys.path.insert(0, __file__.rsplit('/', 2)[0])
import ctdataset

def qm(q):
    x, y, z, w = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)], [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])

cams, samples, devices = ctdataset.load(sys.argv[1])
imu = {d: np.loadtxt(p, delimiter=',', comments='#') for d, p in ((0, sys.argv[2]), (1, sys.argv[3]))}
by = collections.defaultdict(list)
for s in samples:
    by[s['ts']].append(s)
uts = np.array(sorted(by))
t0 = uts[0]
F = np.diag([1.0, -1.0, -1.0])
for dev in (0, 1):
    pos = {}  # ts -> world position of the fix
    for t in uts:
        for s in by[t]:
            f = s['states'].get(dev, {}).get('found')
            if f is not None and s['cam_pose'] is not None:
                (pc, qc) = s['cam_pose']
                p_cv, q_cv = f
                pos[t] = np.asarray(pc) + qm(qc) @ (F @ np.asarray(p_cv))
                break
    fixed = np.array([t in pos for t in uts])
    if not fixed.any():
        print(f'device {dev}: no fixes')
        continue
    idx = np.where(fixed)[0]
    it, gy = imu[dev][:, 0], np.degrees(np.linalg.norm(imu[dev][:, 2:5], axis=1))
    gaps = []
    for a, b in zip(idx[:-1], idx[1:]):
        g = (uts[b] - uts[a]) / 1e9
        if g <= 0.1:
            continue
        m = (it >= uts[a] - 50e6) & (it <= uts[a] + 50e6)
        speed = gy[m].max() if m.any() else np.nan
        # in view: midpoint of the two fixes, in any camera frame at the middle of the gap, in front and inside
        mid = (pos[uts[a]] + pos[uts[b]]) / 2
        tm = uts[(a + b) // 2]
        inview = False
        for s in by[tm]:
            if s['cam_pose'] is None:
                continue
            pc, qc = s['cam_pose']
            pcam = qm(qc).T @ (mid - np.asarray(pc))  # OpenXR camera axes: -z forward
            if pcam[2] < -0.05:
                c = cams[s['cam']]
                u = c['K'][0][0] * pcam[0] / -pcam[2]
                v = c['K'][1][1] * -pcam[1] / -pcam[2]
                if (u * u + v * v) ** 0.5 < 0.9 * c['w'] / 2 * 1.6:  # inside the fisheye's useful radius, roughly
                    inview = True
        gaps.append((g, speed, inview, (uts[a] - t0) / 1e9))
    gaps = np.array(gaps, dtype=float)
    span = (uts[idx[-1]] - uts[idx[0]]) / 1e9
    print(f'device {dev}: first fix {(uts[idx[0]] - t0) / 1e9:.1f} s; {len(gaps)} gaps >0.1 s, {gaps[:, 0].sum():.1f} s = '
          f'{100 * gaps[:, 0].sum() / span:.0f}% of {span:.0f} s; frames with a fix {100 * fixed[idx[0]:idx[-1]].mean():.0f}%')
    for lo, hi in ((0.1, 0.3), (0.3, 1), (1, 3), (3, 99)):
        m = (gaps[:, 0] >= lo) & (gaps[:, 0] < hi)
        if m.any():
            print(f'   {lo}-{hi} s: {m.sum():3d}  median gyro at loss {np.nanmedian(gaps[m, 1]):4.0f} deg/s, '
                  f'in a camera\'s view {int(gaps[m, 2].sum())}/{m.sum()}')
    fast = gaps[:, 1] >= 300
    print(f'   at loss: {int((gaps[:, 1] < 100).sum())} below 100 deg/s, {int(fast.sum())} at 300+ deg/s; '
          f'in view during the gap: {int(gaps[:, 2].sum())} of {len(gaps)}')
