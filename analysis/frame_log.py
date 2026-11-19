#!/usr/bin/env python3
"""Direct-mode frame timing from OVRD_FRAME_LOG (FRAME_LOG=1, ovrd_frame_recorder.hpp).

    python3 -I analysis/frame_log.py logs/frames-<stamp>.csv [--window 5] [--events 40]

Per window: Monado frames, how many showed a new SteamVR frame, missed ones (no frame in time), late ones (presented
before the vsync they were shown in), replaced ones, SteamVR's frame time (vsync -> Present), GPU wait, layers and
which image sets the layers came from. Then the individual events (missed, late, replaced, long Present gaps, layer
set changes) with their time.
"""
import argparse
import sys

import numpy as np


def load(path):
    P, F, R, Q, D = [], [], [], [], 0
    for line in open(path):
        x = line.strip().split(',')
        k = x[0]
        try:
            if k == 'P':
                P.append((int(x[1]), int(x[2]), int(x[3])))
            elif k == 'F':
                F.append(tuple(int(v) for v in x[1:12]))
            elif k == 'R':
                R.append(int(x[1]))
            elif k == 'Q':
                Q.append((int(x[1]), int(x[2]), int(x[3])))
            elif k == 'D':
                D += int(x[2])
        except (ValueError, IndexError):
            continue
    return (np.array(P, dtype=np.int64).reshape(-1, 3), np.array(F, dtype=np.int64).reshape(-1, 11),
            np.array(R, dtype=np.int64), np.array(Q, dtype=np.int64).reshape(-1, 3), D)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('log')
    ap.add_argument('--window', type=float, default=5.0)
    ap.add_argument('--events', type=int, default=40)
    a = ap.parse_args()
    P, F, R, Q, dropped = load(a.log)
    if len(F) == 0:
        print('no F rows')
        return 1
    t0 = F[0, 0]
    woke, fid, disp, vs, pres, wait_us, sync, flags, layers, set0, set1 = F.T
    sub = (flags & 1) != 0
    late = (flags & 2) != 0
    missed = (flags & 4) != 0
    ft = np.where(pres > 0, (pres - vs) / 1e6, np.nan)  # SteamVR's frame time after our vsync
    tt = (woke - t0) / 1e9
    print(f"{a.log}: {len(F)} Monado frames over {tt[-1]:.0f} s, {len(P)} Presents, {len(R)} replaced, "
          f"{dropped} records dropped")
    if len(Q):
        thr = Q[:, 1] != 0
        print(f"SteamVR throttling requested in {thr.sum()} of {len(Q)} PostPresents")
    print(f"{'window s':>9} {'frames':>6} {'shown':>6} {'missed':>6} {'late':>5} {'repl':>5} "
          f"{'SteamVR frame ms p50/p99':>25} {'GPU wait ms p50/p99':>20} {'layers':>6} {'sets':>5}")
    rt = (R - t0) / 1e9 if len(R) else np.array([])
    for w0 in np.arange(0, tt[-1], a.window):
        m = (tt >= w0) & (tt < w0 + a.window)
        if not m.any():
            continue
        f = ft[m & ~np.isnan(ft)]
        g = wait_us[m & sub] / 1e3
        sets = len(set(set0[m & (layers > 0)]) | set(set1[m & (layers > 1)]))
        print(f"{w0:4.0f}-{w0 + a.window:<4.0f} {m.sum():6d} {(m & sub).sum():6d} {(m & missed).sum():6d} "
              f"{(m & late).sum():5d} {((rt >= w0) & (rt < w0 + a.window)).sum():5d} "
              f"{np.median(f) if len(f) else np.nan:12.2f} / {np.percentile(f, 99) if len(f) else np.nan:6.2f} "
              f"{np.median(g) if len(g) else np.nan:9.2f} / {np.percentile(g, 99) if len(g) else np.nan:6.2f} "
              f"{np.mean(layers[m]):6.2f} {sets:5d}")
    ev = []
    for i in np.where(missed)[0]:
        ev.append((tt[i], 'missed: no SteamVR frame in time'))
    for i in np.where(late & sub)[0]:
        ev.append((tt[i], f'late: shown frame was presented {(vs[i] - pres[i]) / 1e6:.1f} ms before this vsync'))
    for r in rt:
        ev.append((r, 'replaced: a newer Present came before it was shown'))
    if len(P) > 1:
        pg = np.diff(P[:, 0]) / 1e6
        for i in np.where(pg > 20)[0]:
            ev.append(((P[i + 1, 0] - t0) / 1e9, f'Present gap {pg[i]:.1f} ms'))
    prev = None
    for i in range(len(F)):
        if layers[i] == 0:
            continue
        cur = (layers[i], set0[i], set1[i] if layers[i] > 1 else 0)
        if prev is not None and cur != prev:
            ev.append((tt[i], f'layers {prev[0]} -> {cur[0]}, sets {prev[1]:x}/{prev[2]:x} -> {cur[1]:x}/{cur[2]:x}'))
        prev = cur
    ev.sort()
    print(f"events ({len(ev)}, first {a.events}):")
    for t, what in ev[:a.events]:
        print(f"  {t:7.2f} s  {what}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
