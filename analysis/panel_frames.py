#!/usr/bin/env python3
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
"""What the PS VR2 panel itself shows: its display frame counter, from the "D" lines of a head pose recording.

Every IMU sample (2 kHz) carries the headset's display frame counter and the scan line it's on; the driver records it
once per change. On a steady display the counter goes up by exactly 1 every refresh. This reports how evenly it does,
in the headset's own clock (its VTS microseconds) and in ours: frames the panel held for two refreshes or more, counter
jumps (frames it skipped), and the spread of the refresh interval, per 10 s window. The scan line at the change refines
when the new frame started (the IMU sample is up to 0.5 ms after it).

Usage: panel_frames.py record.csv
"""
import argparse

import numpy as np


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("record")
    a = ap.parse_args()
    rows = []
    for line in open(a.record):
        if line.startswith("D,"):
            x = line.split(",")
            rows.append((int(x[1]), int(x[2]), int(x[3]), int(x[4])))
    if len(rows) < 100:
        print(f"{len(rows)} panel frame records: none or too few (a recording from before 2026-11-18 has none)")
        return
    d = np.array(rows, dtype=np.int64)
    host, vts, cnt, line = d[:, 0], d[:, 1].astype(np.int64), d[:, 2], d[:, 3]
    vts = np.unwrap(vts, period=2 ** 32)  # 32-bit microsecond counter
    dc = np.diff(cnt) % 65536  # 16-bit frame counter
    dt_vts = np.diff(vts)  # us, headset clock
    period = np.median(dt_vts[dc == 1])
    lines_per_frame = 2200.0  # 2040 visible + blanking (psvr2_hmd_get_compositor_info)
    start = vts - line / lines_per_frame * period  # when this frame started, headset clock
    ds = np.diff(start)
    one = dc == 1
    t = (host - host[0]) * 1e-9
    print(f"{len(d)} panel frame changes over {t[-1]:.0f} s; refresh {period:.1f} us ({1e6 / period:.3f} Hz) by the headset's clock")
    print(f"counter steps: +1 {100 * one.mean():.3f}%, +2 (a frame skipped) {100 * (dc == 2).mean():.3f}%, "
          f"+3 or more {100 * (dc >= 3).mean():.3f}%, 0 {100 * (dc == 0).mean():.3f}%")
    held = one & (ds > 1.5 * period)
    print(f"refresh interval (+1 steps, scan-line corrected): median {np.median(ds[one]):.1f} us, p1 {np.percentile(ds[one], 1):.1f}, "
          f"p99 {np.percentile(ds[one], 99):.1f}, max {ds[one].max():.1f}; frames held 1.5+ refreshes {int(held.sum())}")
    print(f"scan line when the change was seen: median {np.median(line):.0f}, p99 {np.percentile(line, 99):.0f} "
          f"(of ~{lines_per_frame:.0f}; small = the change was caught right after the new frame began)")
    print("per 10 s: skipped / held / interval spread (p1-p99, us)")
    for s in np.arange(0, t[-1], 10):
        m = (t[1:] >= s) & (t[1:] < s + 10)
        if m.sum() < 50:
            continue
        mm = m & one
        print(f"  {s:4.0f}-{s + 10:4.0f} s: {m.sum():4d} frames, skipped {int(((dc >= 2) & m).sum()):3d}, held {int((held & m).sum()):3d}, "
              f"interval {np.percentile(ds[mm], 1):7.1f}-{np.percentile(ds[mm], 99):7.1f}")


if __name__ == "__main__":
    main()
