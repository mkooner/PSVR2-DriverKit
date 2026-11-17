#!/usr/bin/env python3
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
"""How wrong is the Sense controller IMU orientation the tracker's gates get when the camera frame is newer than the
newest IMU sample (Bluetooth delivers IMU ~16 ms late)? From a PSSENSE_RECORD_FILE recording (400 Hz IMU, with the
3DoF orientation).

m_relation_history_get past the newest entry extrapolates with m_predict_relation, which takes the angular velocity in
world space; pssense stores the body-frame gyro there. For each sample time t and data lag L, extrapolate from the
sample L earlier: (a) as now (body gyro treated as world), (b) with the gyro rotated to world first. Error against the
recorded orientation at t, by controller speed.
Usage: ctrl_imu_extrap.py sense-*.csv
"""
import sys
import numpy as np

def qmul(a, b):
    x1, y1, z1, w1 = a.T; x2, y2, z2, w2 = b.T
    return np.stack([w1*x2 + x1*w2 + y1*z2 - z1*y2, w1*y2 - x1*z2 + y1*w2 + z1*x2,
                     w1*z2 + x1*y2 - y1*x2 + z1*w2, w1*w2 - x1*x2 - y1*y2 - z1*z2], 1)
def qrot(q, v):
    u = q[:, :3]; w = q[:, 3:4]
    return v + 2 * np.cross(u, np.cross(u, v) + w * v)
def qinv(q):
    return q * np.array([-1, -1, -1, 1])
def qexp(r):
    a = np.linalg.norm(r, axis=1, keepdims=True)
    s = np.where(a > 1e-12, np.sin(a/2) / np.maximum(a, 1e-12), 0.5)
    return np.concatenate([r * s, np.cos(a/2)], 1)
def angle(a, b):
    d = np.abs(np.sum(a * b, axis=1))
    return 2 * np.degrees(np.arccos(np.clip(d, 0, 1)))

for path in sys.argv[1:]:
    D = np.loadtxt(path, delimiter=',', comments='#')
    t = D[:, 1] * 1e-9; g = D[:, 2:5]; q = D[:, 8:12]
    q /= np.linalg.norm(q, axis=1, keepdims=True)
    speed = np.degrees(np.linalg.norm(g, axis=1))
    print(f'== {path.split("/")[-1]}')
    for lag_ms in (5, 10, 15, 25):
        k = np.searchsorted(t, t - lag_ms / 1000.0) - 1   # newest sample available at t - lag
        ok = (k >= 0) & (k < np.arange(len(t)))
        i = np.arange(len(t))[ok]; k = k[ok]
        dt = (t[i] - t[k])[:, None]
        # m_predict_relation: body = q^-1 * w_stored * q, then orientation * exp(body * dt)
        w_stored_now = g[k]                          # pssense stores the body gyro
        body_now = qrot(qinv(q[k]), w_stored_now)     # treated as world -> rotated again
        pred_now = qmul(q[k], qexp(body_now * dt))
        w_world = qrot(q[k], g[k])                    # fixed: store the world-frame rate
        body_fix = qrot(qinv(q[k]), w_world)          # == g[k]
        pred_fix = qmul(q[k], qexp(body_fix * dt))
        hold = q[k]
        e_now, e_fix, e_hold = angle(pred_now, q[i]), angle(pred_fix, q[i]), angle(hold, q[i])
        line = f'  data {lag_ms:2d} ms behind:'
        for lo, hi in [(0, 100), (100, 300), (300, 600), (600, 5000)]:
            m = (speed[i] >= lo) & (speed[i] < hi)
            if m.sum() > 50:
                line += (f' | {lo}-{hi} deg/s: now p50/p90 {np.median(e_now[m]):.1f}/{np.percentile(e_now[m],90):.1f}, '
                         f'fixed {np.median(e_fix[m]):.1f}/{np.percentile(e_fix[m],90):.1f}, '
                         f'no extrapolation {np.median(e_hold[m]):.1f}')
        print(line)
