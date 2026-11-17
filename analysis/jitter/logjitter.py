#!/usr/bin/env python3
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
"""Jitter in what SteamVR gives an app (tools/vrposelog output), for comparing drivers with the same numbers.

Per refresh: the pose predicted to photons (P, what a game renders with) and unpredicted (N). Jitter is the part of
each frame's rotation/position step a smooth curve through +-4 frames doesn't explain (real head motion is smooth at
~45 ms). Head speed bins come from the unpredicted rotation rate. Still windows (2 s with rotation under 0.5 deg/s,
e.g. the headset on a desk) give the RMS wobble of orientation and position.
Usage: logjitter.py vrposelog.csv [...]
"""
import sys
import numpy as np

def qmul(a, b):
    x1, y1, z1, w1 = a.T; x2, y2, z2, w2 = b.T
    return np.stack([w1*x2 + x1*w2 + y1*z2 - z1*y2, w1*y2 - x1*z2 + y1*w2 + z1*x2,
                     w1*z2 + x1*y2 - y1*x2 + z1*w2, w1*w2 - x1*x2 - y1*y2 - z1*z2], 1)

def inv(q):
    return q * np.array([-1, -1, -1, 1])

def qlog(q):
    q = q * np.sign(q[:, 3:4] + 1e-15)
    vn = np.linalg.norm(q[:, :3], axis=1, keepdims=True)
    ang = 2 * np.arctan2(vn, q[:, 3:4])
    return np.where(vn > 1e-12, q[:, :3] / np.maximum(vn, 1e-12) * ang, 2 * q[:, :3])

def residual(x, half=4):
    tt = np.arange(-half, half + 1)
    c = np.linalg.pinv(np.vander(tt, 4))[-1]
    r = np.stack([x[:, k] - np.convolve(x[:, k], c[::-1], mode='same') for k in range(3)], 1)
    n = np.linalg.norm(r, axis=1)
    n[:half] = np.nan; n[-half:] = np.nan
    return n

def pct(a, ps=(50, 90, 99)):
    a = a[np.isfinite(a)]
    return ' / '.join(f'{np.percentile(a, x):.4f}' for x in ps) if len(a) > 20 else '-'

for path in sys.argv[1:]:
    head = open(path).readline().strip()
    D = np.genfromtxt(path, delimiter=',', names=True, skip_header=1 if head.startswith('#') else 0,
                      invalid_raise=False)
    ok = (D['P_valid'] == 1) & (D['N_valid'] == 1)
    D = D[ok]
    t = D['t_s']
    dt = np.diff(t)
    print(f'== {path}: {head}')
    print(f'  {len(D)} frames, {t[-1]-t[0]:.0f} s; frame interval median {np.median(dt)*1000:.2f} ms, '
          f'missed (> 1.5 frames) {int((dt > 1.5*np.median(dt)).sum())}; prediction median {np.median(D["pred_s"])*1000:.1f} ms')
    out = {}
    for tag in ('N', 'P'):
        q = np.stack([D[f'{tag}_qx'], D[f'{tag}_qy'], D[f'{tag}_qz'], D[f'{tag}_qw']], 1)
        p = np.stack([D[f'{tag}_px'], D[f'{tag}_py'], D[f'{tag}_pz']], 1)
        steps = np.degrees(qlog(qmul(q[1:], inv(q[:-1]))))
        out[tag] = (q, p, steps, residual(steps), residual(np.diff(p, axis=0) * 1000))
    speed = np.linalg.norm(out['N'][2], axis=1) / np.median(dt)
    bins = [(0, 10, 'still <10 deg/s'), (10, 60, 'turning 10-60'), (60, 1e9, 'fast >60')]
    for tag, name in (('N', 'unpredicted'), ('P', 'predicted to photons')):
        rr, pr = out[tag][3], out[tag][4]
        print(f'  {name:21s} rot jitter deg p50/p90/p99: ' +
              ' | '.join(f'{nm}: {pct(rr[(speed>=lo)&(speed<hi)])}' for lo, hi, nm in bins))
        print(f'  {"":21s} pos jitter mm  p50/p90/p99: ' +
              ' | '.join(f'{nm}: {pct(pr[(speed>=lo)&(speed<hi)])}' for lo, hi, nm in bins))
    # Still windows
    q, p = out['N'][0], out['N'][1]
    W = int(round(2.0 / np.median(dt)))
    rots, poss = [], []
    for s in range(0, len(q) - W, W):
        if speed[s:s+W-1].max() < 0.5:
            rel = np.degrees(np.linalg.norm(qlog(qmul(q[s:s+W], inv(np.repeat(q[s:s+1], W, 0)))), axis=1))
            rots.append(np.sqrt(np.mean((rel - rel.mean()) ** 2)))
            poss.append(np.sqrt(np.mean(np.sum((p[s:s+W] - p[s:s+W].mean(0)) ** 2, 1))) * 1000)
    if rots:
        print(f'  still windows (2 s, < 0.5 deg/s): {len(rots)}; orientation RMS {np.median(rots):.4f} deg, '
              f'position RMS {np.median(poss):.3f} mm (medians)')
    else:
        print('  no still windows (put the headset down for a few seconds to measure them)')
