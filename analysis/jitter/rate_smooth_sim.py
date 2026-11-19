#!/usr/bin/env python3
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
"""Calibrate the turn-rate smoothing window of the SteamVR plugin (steamvr_psvr2 Hmd::updatePose) on recorded gyro.

SteamVR here shows frames without reprojection and predicts the head pose a game draws with from the driver's latest
pose and the angular velocity it was handed, ~15-22 ms ahead. With the latest single gyro sample as that velocity the
head's small fast wobbles are multiplied over the prediction and every frame gets a fresh kink (2026-11-18: drawn
poses' frame-to-frame turn-rate change, lag-1 autocorrelation -0.05..-0.25, p99 1.5 deg; Sony's driver +0.52 / 0.21
deg, like real head motion). The plugin can instead send the mean rotation rate over its last W of 1 ms pose samples.

Modelled as the plugin does it: 1 ms pose samples (the head's orientation, gyro integrated), rate = rotation between
the sample W ago and the latest / W, SteamVR uses the latest sample (0-1 ms old) and extrapolates it to the display
time. W 0: the latest gyro sample (the old behaviour). Per W and prediction H, on 90 Hz frames while turning
(> 40 deg/s): "kink" = frame-to-frame change of the drawn turn step (p99, deg) and its lag-1 autocorrelation
("link"; real head motion ~+0.5), and the error against the true head at the display time (p50/p90/p99, deg).

Usage: rate_smooth_sim.py record.csv [...]
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import predictor_sim as ps  # noqa: E402

P = 1 / 89.91
WINDOWS_MS = (0, 16, 24, 32, 40, 48, 56, 64, 80)
HORIZONS_MS = (15, 22)


def drawn_stats(q, truth):
    st = np.degrees(ps.qlog(ps.qmul(ps.inv(q[:-1]), q[1:])))
    d2 = st[1:] - st[:-1]
    sp = np.linalg.norm(st[1:], axis=1) / P
    turn = sp > 40
    mag = np.linalg.norm(d2[turn], axis=1)
    a, b = d2[turn][:-1], d2[turn][1:]
    link = np.mean([np.corrcoef(a[:, j], b[:, j])[0, 1] for j in range(3)])
    err = np.degrees(np.linalg.norm(ps.qlog(ps.qmul(ps.inv(truth), q)), axis=1))[2:][turn]
    return np.percentile(mag, 99), link, np.median(err), np.percentile(err, 90), np.percentile(err, 99), turn.sum()


def main():
    rng = np.random.default_rng(7)
    results = {}
    for path in sys.argv[1:]:
        t, w = ps.load(path)
        m = ps.Motion(t, w)
        fr = np.arange(t[0] + 2, t[-1] - 2, P)
        ts = fr - rng.uniform(0, 0.001, len(fr))  # the plugin's latest 1 ms sample when SteamVR is asked
        ts = np.floor(ts * 1000) / 1000           # on the plugin's 1 ms grid
        q_now = m.at(ts)
        real = drawn_stats(m.at(fr), m.at(fr))
        if real[5] < 200:
            print(f"{path}: {real[5]} turning frames, skipped (too few)")
            continue
        print(f"{path}: {real[5]} turning frames; real head motion kink p99 {real[0]:.3f} link {real[1]:+.2f}")
        for H in HORIZONS_MS:
            for W in WINDOWS_MS:
                if W == 0:
                    rate = m.w[m.idx(ts)]
                else:
                    q_old = m.at(ts - W / 1000)
                    rate = ps.qlog(ps.qmul(ps.inv(q_old), q_now)) / (W / 1000)
                q = ps.qmul(q_now, ps.qexp(rate * (fr - ts + H / 1000)[:, None]))
                s = drawn_stats(q, m.at(fr + H / 1000))
                results.setdefault((H, W), []).append((s[0] / real[0], s[1], s[2], s[3], s[4]))
                print(f"  H {H:2d} ms  W {W:2d} ms: kink p99 {s[0]:.3f} ({s[0] / real[0]:.2f}x real) link {s[1]:+.2f} | "
                      f"error p50 {s[2]:.3f} p90 {s[3]:.3f} p99 {s[4]:.3f} deg")
    print("\nSummary over recordings (worst recording for the kink ratio, median for the rest):")
    for H in HORIZONS_MS:
        for W in WINDOWS_MS:
            r = np.array(results.get((H, W), []))
            if len(r):
                print(f"  H {H:2d} ms  W {W:2d} ms: kink worst {r[:, 0].max():.2f}x real (median {np.median(r[:, 0]):.2f}x) "
                      f"link {np.median(r[:, 1]):+.2f} | error p50 {np.median(r[:, 2]):.3f} p90 {np.median(r[:, 3]):.3f} "
                      f"p99 {np.median(r[:, 4]):.3f} deg")
    return 0


if __name__ == '__main__':
    sys.exit(main())
