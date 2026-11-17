#!/usr/bin/env python3
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
"""How much the head pose clock conversion wanders (recorded I rows).

The driver turns SteamVR's query time into headset time with hw2mono_vts, from m_clock_offset_a2b (an exponential
average of host_est - vts, integer alpha 993/1000 per 2 kHz sample: ~70 ms time constant). host_est is USB arrival
minus the sample's slot in its packet, so it carries USB delay and jitter. Reference: the minimum-delay line (the
lower envelope of host_est - vts, 1 s windows, linear fit through the minima), i.e. the least delayed arrivals.

Reports the EMA's error against the reference (mean = constant latency, std/p99 = wander), the orientation error it
causes at typical head speeds, and the spectrum of the wander.
Usage: clock_offset.py record.csv [...]
"""
import sys
import numpy as np

for path in sys.argv[1:]:
    I = np.array([[float(x) for x in l.split(',')[1:3]] for l in open(path) if l.startswith('I,')], dtype=np.int64)
    if len(I) < 20000:
        continue
    host, vts = I[:, 0], I[:, 1]
    d = host - vts  # ns, offset + delay
    # m_clock_offset_a2b exactly (integer arithmetic, truncation like C)
    alpha = int(1000 * (1.0 - 12.5 / 2000.0))
    ema = np.empty_like(d)
    a2b = 0
    for i, g in enumerate(d):
        if a2b == 0:
            a2b = int(g)
        else:
            x = (a2b - int(g)) * alpha
            a2b = int(x / 1000) + int(g)  # C division truncates toward zero
        ema[i] = a2b
    # Reference: minima over 1 s windows, robust linear fit
    t = (vts - vts[0]) * 1e-9
    W = 2000
    idx = [s + np.argmin(d[s:s+W]) for s in range(0, len(d) - W, W)]
    tm, dm = t[idx], d[idx].astype(float)
    k, b = np.polyfit(tm, dm, 1)
    for _ in range(3):  # drop windows whose minimum is far above the line (no low-delay arrival in them)
        r = dm - (k * tm + b)
        keep = r < np.percentile(r, 80)
        k, b = np.polyfit(tm[keep], dm[keep], 1)
    ref = k * t + b
    skip = int(5 * 2000)  # ignore the first 5 s
    err = (ema - ref)[skip:] / 1e6  # ms
    raw = (d - ref)[skip:] / 1e6
    name = path.split('/')[-1][7:22]
    print(f'== {name}: skew {k/1e3:+.1f} ppm')
    print(f'  arrival delay over the min line: median {np.median(raw):.2f} ms, p99 {np.percentile(raw,99):.1f} ms')
    print(f'  EMA offset error: mean {err.mean():.2f} ms (constant lag), std {err.std():.2f} ms, '
          f'p1/p99 {np.percentile(err,1):.2f}/{np.percentile(err,99):.2f} ms, frame-to-frame (11 ms) std '
          f'{np.diff(err[::22]).std():.3f} ms')
    for speed in (50, 150):
        e = np.radians(speed) * err.std() / 1e3
        print(f'    -> at {speed} deg/s head turn: orientation wander {np.degrees(e):.3f} deg std')
    # spectrum of the wander (decimate to 200 Hz)
    x = err[::10] - err[::10].mean()
    n = 2048
    if len(x) > n:
        P = np.zeros(n // 2 + 1)
        for s in range(0, len(x) - n, n // 2):
            P += np.abs(np.fft.rfft(x[s:s+n] * np.hanning(n))) ** 2
        f = np.fft.rfftfreq(n, 1 / 200)
        tot = P[1:].sum()
        print('  wander power: ' + ', '.join(f'{lo}-{hi} Hz {P[(f>=lo)&(f<hi)].sum()/tot*100:.0f}%'
                                            for lo, hi in [(0, 1), (1, 5), (5, 20), (20, 100)]))
