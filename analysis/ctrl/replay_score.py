#!/usr/bin/env python3
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
"""Score ct_replay outputs: losses >150 ms per device (the driver's "holding position" measure), time without a fix,
jumps within each camera's own pose sequence (>2 cm + 4 m/s between consecutive fixes of that camera, returning next
fix = spike), and how far two cameras' poses of the same device in the same frame disagree.
Usage: replay_score.py out.csv [...]   (reads out.csv and out.csv.fixes)
"""
import sys, collections
import numpy as np
for path in sys.argv[1:]:
    d = np.genfromtxt(path, delimiter=',', names=True)
    t = d['t_ns'] / 1e9
    f = np.genfromtxt(path + '.fixes', delimiter=',', names=True)
    out = []
    for dev in (0, 1):
        idx = np.where(d[f'fix{dev}'] > 0)[0]
        g = np.diff(t[idx])
        span = t[idx[-1]] - t[idx[0]]
        losses = int((g > 0.15).sum())
        nofix = g[g > 0.1].sum() / span * 100
        ff = f[f['device'] == dev]
        spikes = 0
        for cam in np.unique(ff['camera']):
            c = ff[ff['camera'] == cam]
            c = c[np.argsort(c['t_ns'])]
            p = np.c_[c['x'], c['y'], c['z']]
            tt = c['t_ns'] / 1e9
            for k in range(1, len(c) - 1):
                if np.linalg.norm(p[k] - p[k - 1]) > 0.02 + 4 * (tt[k] - tt[k - 1]) and \
                   np.linalg.norm(p[k + 1] - p[k - 1]) < 0.03 + 4 * (tt[k + 1] - tt[k - 1]):
                    spikes += 1
        by = collections.defaultdict(list)
        for r in ff:
            by[int(r['t_ns'])].append(np.array([r['x'], r['y'], r['z']]))
        dis = [np.linalg.norm(v[0] - v[1]) for v in by.values() if len(v) >= 2]
        out.append(f'dev{dev}: losses {losses:3d}, no fix {nofix:4.0f}%, same-camera spikes {spikes:2d}, two-camera '
                   f'frames {len(dis):4d} disagree p50 {np.median(dis) * 100 if dis else 0:4.1f} cm')
    print(f'{path.split("/")[-1]:24s} ' + ' | '.join(out))
