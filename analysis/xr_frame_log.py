#!/usr/bin/env python3
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
"""An app's OpenXR frame timing as the runtime gave it (tools/xr_frame_log layer output).

On a steady display every xrWaitFrame's predicted display time is exactly one refresh after the previous one and the
predicted period is one refresh. This counts the frames where the runtime said otherwise: the period over 1.5 refreshes
(what the game logs as "the runtime's frame smoothing"), and predicted display times 0 or 2+ refreshes after the
previous frame's (a frame slotted a refresh off), per 10 s window, plus whether the app asked for its views at the
predicted time and submitted that time.

Usage: xr_frame_log.py log.csv [--refresh-hz 89.91]
"""
import argparse

import numpy as np


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log")
    ap.add_argument("--refresh-hz", type=float, default=0, help="default: the most common period in the log")
    a = ap.parse_args()
    W, L, E = [], [], []
    for line in open(a.log):
        if line.startswith("#") or line.startswith("kind"):
            continue
        x = line.strip().split(",")
        if len(x) < 6:
            continue
        k, t0, t1, xa, xb, n = x[0], int(x[1]), int(x[2]), int(x[3]), int(x[4]), int(x[5])
        if k == "W" and xa > 0:
            W.append((t0, t1, xa, xb, n))
        elif k == "L":
            L.append((t0, xa))
        elif k == "E":
            E.append((t0, xa, xb, n))
    if len(W) < 20:
        print(f"only {len(W)} xrWaitFrame results with a display time")
        return
    W = np.array(W, dtype=np.int64)
    per = W[:, 3]
    refresh = 1e9 / a.refresh_hz if a.refresh_hz > 0 else float(np.median(per))
    pt = W[:, 2]
    step = np.diff(pt) / refresh
    ratio = per / refresh
    t = (W[:, 1] - W[0, 1]) * 1e-9
    print(f"{len(W)} frames over {t[-1]:.0f} s, refresh {refresh / 1e6:.3f} ms")
    print(f"predicted period: 1 refresh {100 * (np.abs(ratio - 1) < 0.25).mean():.1f}%, "
          f"over 1.5 refreshes {100 * (ratio > 1.5).mean():.1f}% (values {np.unique(np.round(ratio, 2))[:8]})")
    print(f"predicted display time after the previous frame's: 1 refresh {100 * (np.abs(step - 1) < 0.25).mean():.1f}%, "
          f"0 {100 * (np.abs(step) < 0.25).mean():.2f}%, 2 {100 * (np.abs(step - 2) < 0.25).mean():.2f}%, "
          f"3+ {100 * (step > 2.5).mean():.2f}%, other {100 * ((np.abs(step - np.round(step)) >= 0.25)).mean():.2f}%")
    lead = (pt - W[:, 1])  # predicted display time minus the wait's return, in the runtime's clock minus ours
    lead = lead - np.median(lead)
    print(f"predicted time vs wait return (spread around its median): p1 {np.percentile(lead, 1) / 1e6:+.2f} "
          f"p99 {np.percentile(lead, 99) / 1e6:+.2f} ms")
    if E:
        E = np.array(E, dtype=np.int64)
        submitted = set(E[:, 1].tolist())
        print(f"xrEndFrame: {len(E)}; submitted with a predicted display time: "
              f"{100 * np.mean([p in submitted for p in pt]):.1f}% of predictions")
    if L:
        asked = set(int(x[1]) for x in L)
        print(f"xrLocateViews asked for a predicted display time in {100 * np.mean([p in asked for p in pt]):.1f}% of frames")
    print("per 10 s: period > 1.5 refreshes / display time steps of 0 or 2+ refreshes")
    for s in np.arange(0, t[-1], 10):
        m = (t >= s) & (t < s + 10)
        mm = m[1:]
        if m.sum() < 10:
            continue
        odd = (np.abs(step[mm] - 1) >= 0.25).sum()
        print(f"  {s:4.0f}-{s + 10:4.0f} s: {m.sum():4d} frames, long period {int((ratio[m] > 1.5).sum()):3d}, "
              f"off-slot steps {int(odd):3d}")


if __name__ == "__main__":
    main()
