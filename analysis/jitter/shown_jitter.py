#!/usr/bin/env python3
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
"""Jitter in what SteamVR shows, from pose-sim/steamvr_sim's per-frame dump (89.91 Hz frames).

For each frame the driver's pose "now" is extrapolated SteamVR-style over H with the angular velocity source
(raw gyro sample, or the mean of the last 10 ms) and the filter's linear velocity. Jitter is the part of each frame's
rotation/position step that a smooth curve through the neighbouring frames (cubic, +-4 frames = +-44 ms) doesn't
explain: real head motion is smooth at that scale, noise and update steps are not. Also: its spectrum (to tell noise
from the 60 Hz SLAM vs 90 Hz frame beat) and the prediction error against the driver's own pose H later.
Usage: shown_jitter.py frames.csv [t_from t_to]
"""
import sys
import numpy as np

D = np.genfromtxt(sys.argv[1], delimiter=',', names=True)
if len(sys.argv) > 3:
    D = D[(D['t_s'] >= float(sys.argv[2])) & (D['t_s'] <= float(sys.argv[3]))]
D = D[D['valid'] == 1]
dt = 1 / 89.91
q = np.stack([D['qx'], D['qy'], D['qz'], D['qw']], 1)
p = np.stack([D['px'], D['py'], D['pz']], 1)
v = np.stack([D['vx'], D['vy'], D['vz']], 1)
W = {'raw': np.stack([D['wx'], D['wy'], D['wz']], 1), 'avg10': np.stack([D['a10x'], D['a10y'], D['a10z']], 1),
     'avg20': np.stack([D['a20x'], D['a20y'], D['a20z']], 1)}
speed = np.degrees(np.linalg.norm(W['raw'], axis=1))

def qmul(a, b):
    x1, y1, z1, w1 = a.T; x2, y2, z2, w2 = b.T
    return np.stack([w1*x2 + x1*w2 + y1*z2 - z1*y2, w1*y2 - x1*z2 + y1*w2 + z1*x2,
                     w1*z2 + x1*y2 - y1*x2 + z1*w2, w1*w2 - x1*x2 - y1*y2 - z1*z2], 1)

def qexp(r):
    a = np.linalg.norm(r, axis=1, keepdims=True)
    s = np.where(a > 1e-12, np.sin(a/2) / np.maximum(a, 1e-12), 0.5)
    return np.concatenate([r * s, np.cos(a/2)], 1)

def qlog(qq):
    qq = qq * np.sign(qq[:, 3:4] + 1e-15)
    vn = np.linalg.norm(qq[:, :3], axis=1, keepdims=True)
    ang = 2 * np.arctan2(vn, qq[:, 3:4])
    return np.where(vn > 1e-12, qq[:, :3] / np.maximum(vn, 1e-12) * ang, 2 * qq[:, :3])

def inv(qq):
    return qq * np.array([-1, -1, -1, 1])

def steps_rot(qs):
    # world-frame rotation from frame k-1 to k, degrees
    return np.degrees(qlog(qmul(qs[1:], inv(qs[:-1]))))

def residual(x, half=4):
    # x: (n, 3) per-frame steps; residual against a local cubic fit (centre point excluded from nothing; plain fit)
    n = len(x); out = np.full(n, np.nan)
    tt = np.arange(-half, half + 1)
    A = np.vander(tt, 4)
    P = np.linalg.pinv(A)  # 4 x (2h+1); centre value = last row of coeffs (constant term)
    c = P[-1]
    for k in range(3):
        sm = np.convolve(x[:, k], c[::-1], mode='same')
        if k == 0:
            r = np.zeros((n, 3))
        r[:, k] = x[:, k] - sm
    r[:half] = np.nan; r[-half:] = np.nan
    return np.linalg.norm(r, axis=1)

def pct(a, ps=(50, 90, 99)):
    a = a[np.isfinite(a)]
    return ' / '.join(f'{np.percentile(a, x):.4f}' for x in ps) if len(a) else '-'

print(f'{len(D)} frames, {len(D)*dt:.0f} s; head speed p50 {np.median(speed):.1f}, p90 {np.percentile(speed,90):.1f} deg/s')
bins = [(0, 10, 'still <10 deg/s'), (10, 60, 'turning 10-60'), (60, 1e9, 'fast >60')]
Hs = [0, 1, 2, 3]  # frames
for src in ('raw', 'avg10', 'avg20'):
    for h in Hs:
        H = h * dt
        qs = qmul(qexp(W[src] * H), q)
        st = steps_rot(qs)
        res = residual(st)
        sp = speed[1:]
        line = f'  rot jitter (deg, p50/p90/p99) w={src:5s} H={H*1000:4.1f} ms: '
        line += ' | '.join(f'{name}: {pct(res[(sp>=lo)&(sp<hi)])}' for lo, hi, name in bins)
        if h > 0:
            err = np.degrees(np.linalg.norm(qlog(qmul(q[h:], inv(qs[:-h]))), axis=1))
            line += f' | pred err p50/p90 {np.percentile(err,50):.3f}/{np.percentile(err,90):.3f}'
        if src == 'raw' or h == 2:
            print(line)
for h in Hs:
    H = h * dt
    ps = p + v * H
    st = np.diff(ps, axis=0) * 1000
    res = residual(st)
    sp = speed[1:]
    line = f'  pos jitter (mm, p50/p90/p99) H={H*1000:4.1f} ms: ' + ' | '.join(
        f'{name}: {pct(res[(sp>=lo)&(sp<hi)])}' for lo, hi, name in bins)
    if h > 0:
        err = np.linalg.norm(p[h:] - ps[:-h], axis=1) * 1000
        line += f' | pred err p50/p90 {np.percentile(err,50):.2f}/{np.percentile(err,90):.2f}'
    if 'have_slam' in D.dtype.names:
        # against the headset's own SLAM position at t + H (filter lag included), all frames and fast ones
        sl = np.stack([D['sx'], D['sy'], D['sz']], 1)
        okk = D['have_slam'][h:] == 1
        e = np.linalg.norm(sl[h:] - ps[:len(ps)-h], axis=1) * 1000
        fast = speed[:len(ps)-h] >= 60
        line += f' | vs SLAM p50/p90 {np.percentile(e[okk],50):.2f}/{np.percentile(e[okk],90):.2f}'
        if (okk & fast).sum() > 20:
            line += f' (fast {np.percentile(e[okk&fast],50):.2f}/{np.percentile(e[okk&fast],90):.2f})'
    print(line)

# Spectrum of the shown rotation rate and position velocity, moving frames, H = 0 and 22 ms
def spectrum(x, label):
    n = 256
    x = x - np.nanmean(x, axis=0)
    x = np.nan_to_num(x)
    P = np.zeros(n // 2 + 1); m = 0
    for s in range(0, len(x) - n, n // 2):
        seg = x[s:s+n]
        P += (np.abs(np.fft.rfft(seg * np.hanning(n)[:, None], axis=0)) ** 2).sum(axis=1); m += 1
    f = np.fft.rfftfreq(n, dt)
    tot = P[f > 0.5].sum()
    bands = [(0.5, 5), (5, 15), (15, 25), (25, 35), (35, 46)]
    print(f'  {label}: ' + ', '.join(f'{lo}-{hi} Hz {P[(f>=lo)&(f<hi)].sum()/tot*100:.1f}%' for lo, hi in bands)
          + '; peaks ' + ', '.join(f'{f[i]:.1f}' for i in (np.argsort(P[f > 5])[-3:][::-1] + np.argmax(f > 5))))
print('  spectrum of the shown motion (share of power):')
for h in (0, 2):
    qs = qmul(qexp(W['raw'] * h * dt), q)
    spectrum(steps_rot(qs) / dt, f'rotation rate, H={h*dt*1000:.0f} ms')
    spectrum(np.diff(p + v * h * dt, axis=0) / dt, f'velocity,      H={h*dt*1000:.0f} ms')
