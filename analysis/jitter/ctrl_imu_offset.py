#!/usr/bin/env python3
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
"""Time offset between Sense controller optical fixes and its IMU orientation (pssense "OPT" debug lines: camera
timestamp, optical pose, and the IMU orientation the driver looked up for that same time).

Compares how far the controller turned between two optical fixes ~50-120 ms apart with how far the IMU orientation
turned over the same interval shifted by d (IMU series interpolated from the OPT lines). Rotation angles only, so the
fixed LED->IMU rotation doesn't matter. Best d = how much later the IMU must be read than the driver reads it now.
Usage: ctrl_imu_offset.py combined.log
"""
import re, sys
import numpy as np

def qmul(a, b):
    x1, y1, z1, w1 = a; x2, y2, z2, w2 = b
    return np.array([w1*x2 + x1*w2 + y1*z2 - z1*y2, w1*y2 - x1*z2 + y1*w2 + z1*x2,
                     w1*z2 + x1*y2 - y1*x2 + z1*w2, w1*w2 - x1*x2 - y1*y2 - z1*z2])
inv = lambda q: np.array([-q[0], -q[1], -q[2], q[3]])
ang = lambda q: 2 * np.degrees(np.arccos(min(1.0, abs(q[3]))))

def slerp(a, b, u):
    if np.dot(a, b) < 0:
        b = -b
    d = np.clip(np.dot(a, b), -1, 1)
    th = np.arccos(d)
    if th < 1e-6:
        return a
    return (np.sin((1 - u) * th) * a + np.sin(u * th) * b) / np.sin(th)

pat = re.compile(r'OPT (\d+) ([01]) \S+ \S+ \S+ (\S+) (\S+) (\S+) (\S+) IMU 1 (\S+) (\S+) (\S+) (\S+)')
rows = {0: {}, 1: {}}
for line in open(sys.argv[1], errors='replace'):
    m = pat.search(line)
    if m:
        rows[int(m[2])][int(m[1])] = (np.array(list(map(float, m.groups()[2:6]))),
                                      np.array(list(map(float, m.groups()[6:10]))))
for h in (0, 1):
    ts = np.array(sorted(rows[h]))
    if len(ts) < 50:
        continue
    opt = [rows[h][t][0] for t in ts]; imu = [rows[h][t][1] for t in ts]
    def imu_at(t):
        k = np.searchsorted(ts, t)
        if k <= 0 or k >= len(ts) or ts[k] - ts[k-1] > 60e6:
            return None
        return slerp(imu[k-1], imu[k], (t - ts[k-1]) / (ts[k] - ts[k-1]))
    pairs = []
    for i in range(len(ts)):
        for j in range(i + 1, min(i + 12, len(ts))):
            dt = (ts[j] - ts[i]) * 1e-9
            if 0.05 <= dt <= 0.12:
                a = ang(qmul(inv(opt[i]), opt[j]))
                if a / dt > 150:  # fast
                    pairs.append((ts[i], ts[j], a))
                break
    shifts = np.arange(-120, 121, 10) * 1e6
    res = []
    for d in shifts:
        e = []
        for t1, t2, a in pairs:
            q1, q2 = imu_at(t1 + d), imu_at(t2 + d)
            if q1 is not None and q2 is not None:
                e.append(abs(ang(qmul(inv(q1), q2)) - a))
        res.append(np.median(e) if len(e) > 10 else np.nan)
    res = np.array(res)
    k = np.nanargmin(res)
    print(f'controller {h}: {len(pairs)} fast pairs (>150 deg/s); best IMU shift {shifts[k]/1e6:+.0f} ms '
          f'(median angle mismatch {res[k]:.1f} deg; at 0 ms {res[12]:.1f} deg)')
    print('   ' + ', '.join(f'{shifts[i]/1e6:+.0f}:{res[i]:.1f}' for i in range(0, len(shifts), 2)))
