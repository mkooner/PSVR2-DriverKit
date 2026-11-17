#!/usr/bin/env python3
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
"""Headset IMU stream baseline from PSVR2_RECORD_FILE recordings (I rows: host_est_ns, vts_ns, gyro xyz rad/s,
accel xyz m/s^2).

Reports sample timing (vts spacing, duplicates, out of order, gaps), the host-time estimate's jitter against the
headset clock (host_est is USB arrival minus the sample's place in its packet), and gyro noise/bias in still periods
(500 ms windows with gyro std < 0.02 rad/s and |accel| within 0.3 of 9.81), with the noise spectrum.
Usage: imu_stats.py record.csv [...]
"""
import sys
import numpy as np

def load(path):
    rows = []
    for line in open(path):
        if line.startswith('I,'):
            rows.append([float(x) for x in line.split(',')[1:9]])
    return np.array(rows)

for path in sys.argv[1:]:
    I = load(path)
    if len(I) < 4000:
        continue
    host, vts, g, a = I[:, 0], I[:, 1], I[:, 2:5], I[:, 5:8]
    dv = np.diff(vts) / 1e3  # us
    name = path.split('/')[-1][7:22]
    dur = (vts[-1] - vts[0]) * 1e-9
    print(f'== {name}: {len(I)} IMU samples over {dur:.0f} s = {len(I)/dur:.0f} Hz')
    vals, counts = np.unique(np.round(dv), return_counts=True)
    top = sorted(zip(counts, vals), reverse=True)[:4]
    print(f'  vts spacing (us): ' + ', '.join(f'{v:.0f} x{c}' for c, v in top) +
          f'; duplicates {int((dv == 0).sum())}, out of order {int((dv < 0).sum())}, gaps > 1 ms {int((dv > 1000).sum())}')
    # Host estimate vs the headset clock: fit host = vts*k + b, residuals are the arrival-time jitter
    k, b = np.polyfit(vts - vts[0], host - host[0], 1)
    res = (host - host[0]) - (k * (vts - vts[0]) + b)
    print(f'  host vs headset clock: skew {(k-1)*1e6:+.1f} ppm; host-time residual std {res.std()/1e3:.0f} us, '
          f'p1/p99 {np.percentile(res,1)/1e3:.0f}/{np.percentile(res,99)/1e3:.0f} us')
    # Still windows
    W = 1000  # 0.5 s at 2 kHz
    still = []
    for s in range(0, len(I) - W, W):
        gs = g[s:s+W]; an = np.linalg.norm(a[s:s+W], axis=1)
        if gs.std(axis=0).max() < 0.02 and abs(an.mean() - 9.81) < 0.3:
            still.append(s)
    if not still:
        print('  no still periods'); continue
    G = np.concatenate([g[s:s+W] for s in still])
    bias = np.array([g[s:s+W].mean(axis=0) for s in still])
    noise = np.concatenate([g[s:s+W] - g[s:s+W].mean(axis=0) for s in still])
    print(f'  still: {len(still)*0.5:.0f} s; gyro noise std (deg/s) {np.degrees(noise.std(axis=0)).round(3)}; '
          f'bias mean (deg/s) {np.degrees(bias.mean(axis=0)).round(3)}, spread {np.degrees(bias.std(axis=0)).round(3)}')
    # Spectrum of the still gyro noise (Welch-like, averaged over windows)
    P = np.zeros(W // 2 + 1)
    for s in still:
        x = g[s:s+W] - g[s:s+W].mean(axis=0)
        P += (np.abs(np.fft.rfft(x * np.hanning(W)[:, None], axis=0)) ** 2).sum(axis=1)
    f = np.fft.rfftfreq(W, 1 / 2000)
    P /= len(still)
    bands = [(0, 5), (5, 20), (20, 60), (60, 200), (200, 1000)]
    tot = P[1:].sum()
    print('  still gyro noise power by band: ' + ', '.join(f'{lo}-{hi} Hz {P[(f>=lo)&(f<hi)].sum()/tot*100:.0f}%' for lo, hi in bands))
    pk = np.argsort(P[1:])[-3:][::-1] + 1
    print('  strongest bins: ' + ', '.join(f'{f[i]:.0f} Hz' for i in pk))
