#!/usr/bin/env python3
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
"""Temporal offset between the headset's SLAM orientation and its gyro (recorded S and I rows, both on the headset
clock vts).

For SLAM pose pairs ~100 ms apart while the head turns (> 20 deg/s), compares the SLAM relative rotation q1^-1 q2 with
the gyro integrated from t1 + d to t2 + d (body frame, as the pose filter integrates it), for shifts d. The best d is
the time error between the two streams; the residual at the best d is SLAM's own orientation noise + scale/axis
errors. Positive d: the gyro must be read later than the SLAM timestamp says (SLAM poses are stamped early).
Usage: slam_gyro_offset.py record.csv [...]
"""
import sys
import numpy as np

def qmul(a, b):
    x1, y1, z1, w1 = a; x2, y2, z2, w2 = b
    return np.array([w1*x2 + x1*w2 + y1*z2 - z1*y2, w1*y2 - x1*z2 + y1*w2 + z1*x2,
                     w1*z2 + x1*y2 - y1*x2 + z1*w2, w1*w2 - x1*x2 - y1*y2 - z1*z2])

def qinv(q):
    return np.array([-q[0], -q[1], -q[2], q[3]])

def qexp(v):
    a = np.linalg.norm(v)
    if a < 1e-12:
        return np.array([v[0]/2, v[1]/2, v[2]/2, 1.0])
    s = np.sin(a/2)/a
    return np.array([v[0]*s, v[1]*s, v[2]*s, np.cos(a/2)])

def qangle(q):
    return 2*np.degrees(np.arccos(min(1.0, abs(q[3]))))

for path in sys.argv[1:]:
    I = []; S = []
    for l in open(path):
        if l.startswith('I,'):
            I.append([float(x) for x in l.split(',')[2:6]])
        elif l.startswith('S,'):
            S.append([float(x) for x in l.split(',')[2:10]])
    I = np.array(I); S = np.array(S)
    if len(S) < 1000:
        continue
    iv, g = I[:, 0], I[:, 1:4]
    # Integrate the gyro like the filter (sample held until the next, dt from vts; equal vts -> dt 0)
    cum = [np.array([0, 0, 0, 1.0])]
    for k in range(1, len(iv)):
        dt = (iv[k] - iv[k-1]) * 1e-9
        cum.append(qmul(cum[-1], qexp(g[k-1] * dt)) if dt > 0 else cum[-1])
    cum = np.array(cum)
    def gyro_rel(t1, t2):
        a = np.searchsorted(iv, t1); b = np.searchsorted(iv, t2)
        if a <= 0 or b >= len(iv):
            return None
        return qmul(qinv(cum[a]), cum[b])
    sv, sq = S[:, 0], S[:, 4:8]
    t0 = sv[0]
    pairs = []
    for i in range(0, len(S) - 7, 3):
        j = i + 6  # ~100 ms at 60 Hz
        if (sv[i] - t0) < 15e9:
            continue
        rel = qmul(qinv(sq[i]), sq[j])
        dt = (sv[j] - sv[i]) * 1e-9
        if not (0.08 < dt < 0.13):
            continue
        ang = qangle(rel)
        if ang / dt > 20:
            pairs.append((sv[i], sv[j], rel, ang / dt))
    if len(pairs) < 50:
        print(path, 'too few moving pairs'); continue
    shifts = np.arange(-30, 31, 1) * 1e6
    res = []
    for d in shifts:
        e = []
        for t1, t2, rel, sp in pairs:
            gr = gyro_rel(t1 + d, t2 + d)
            if gr is not None:
                e.append(qangle(qmul(qinv(gr), rel)))
        res.append(np.median(e))
    res = np.array(res)
    best = shifts[np.argmin(res)] / 1e6
    name = path.split('/')[-1][7:22]
    print(f'== {name}: {len(pairs)} turning pairs (median {np.median([p[3] for p in pairs]):.0f} deg/s)')
    print(f'  best shift {best:+.0f} ms: median residual {res.min():.3f} deg per 100 ms; at 0 ms {res[30]:.3f} deg')
    print('  residual by shift (ms: deg) ' + ', '.join(f'{shifts[k]/1e6:+.0f}: {res[k]:.2f}' for k in range(0, 61, 5)))
