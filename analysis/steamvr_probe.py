#!/usr/bin/env python3
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
"""How SteamVR's own pose prediction (vrserver) really behaves, from tools/vrposeprobe and the plugin's pose log.

For each answer SteamVR gave for "the headset pose X seconds from now", find the moment of the head's real motion (the
plugin's pose for now, OVRD_POSE_LOG) it matches best. That moment minus the call time is the prediction SteamVR really
made. A correct predictor gives about X + a constant (the tracking latency) every time; jumps, a second peak or a drift
over the session show what SteamVR does with our poses, independent of any app or OpenXR.

Usage: steamvr_probe.py pose-log.csv probe.csv [--min-speed 40] [--window 15]
"""
import argparse
import importlib.util
import os

import numpy as np

here = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("tl", os.path.join(here, "steamvr_timeline.py"))
tl = importlib.util.module_from_spec(spec)
spec.loader.exec_module(tl)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("pose_log")
    ap.add_argument("probe")
    ap.add_argument("--min-speed", type=float, default=40)
    ap.add_argument("--window", type=float, default=15, help="trend window (s)")
    ap.add_argument("--max", type=int, default=6000, help="answers per prediction time to analyse")
    a = ap.parse_args()

    tp, qp, _, _, valid = tl.load_pose_log(a.pose_log)
    tp, qp = tp[valid > 0], qp[valid > 0]
    for i in range(1, len(qp)):
        if np.dot(qp[i], qp[i - 1]) < 0:
            qp[i] = -qp[i]
    pr = tl.read_csv(a.probe)
    pr = pr[pr["valid"] > 0]
    tq = pr["mono_ns"] * 1e-9

    def rot_at(t):
        q = np.stack([np.interp(t, tp, qp[:, k]) for k in range(4)], 1)
        q /= np.linalg.norm(q, axis=1, keepdims=True)
        return tl.quat_to_mat(q)

    print(f"head motion from {a.pose_log} ({len(tp)} poses), {len(pr)} SteamVR answers")
    print(f"{'asked':>7} {'answers':>7} {'real lead':>9} {'p10':>6} {'p90':>6} {'off>5ms':>8} {'1 refresh off':>13}   trend of the real lead per {a.window:.0f} s")
    for X in np.unique(np.round(pr["pred_s"], 3)):
        sel = np.where(np.abs(pr["pred_s"] - X) < 1e-4)[0]
        t = tq[sel]
        ok = (t + X > tp[0] + 0.1) & (t + X < tp[-1] - 0.1)
        sel, t = sel[ok], t[ok]
        if len(sel) < 50:
            continue
        Rh = rot_at(t + X)
        Rh2 = rot_at(t + X + 0.002)
        speed = np.degrees(np.linalg.norm(tl.rotvec(Rh2 @ np.transpose(Rh, (0, 2, 1))), axis=1)) / 0.002
        m = speed > a.min_speed
        sel, t = sel[m], t[m]
        if len(sel) > a.max:
            pick = np.linspace(0, len(sel) - 1, a.max).astype(int)
            sel, t = sel[pick], t[pick]
        if len(sel) < 20:
            continue
        qa = np.c_[pr["qx"][sel], pr["qy"][sel], pr["qz"][sel], pr["qw"][sel]]
        Ra = tl.quat_to_mat(qa)
        lags = np.arange(-0.04, 0.0601, 0.00025)
        err = np.empty((len(lags), len(sel)))
        for k, L in enumerate(lags):
            err[k] = np.degrees(np.linalg.norm(tl.rotvec(Ra @ np.transpose(rot_at(t + L), (0, 2, 1))), axis=1))
        lead = lags[np.argmin(err, 0)] * 1000
        dev = lead - np.median(lead)
        off5 = np.abs(dev) > 5
        refresh_off = np.abs(np.abs(dev) - 11.1) < 3
        trend = []
        for s in np.arange(t[0], t[-1], a.window):
            w = (t >= s) & (t < s + a.window)
            if w.sum() >= 10:
                trend.append(f"{np.median(lead[w]):+.1f}")
        print(f"{X * 1000:+6.0f}ms {len(sel):7d} {np.median(lead):+8.1f}ms {np.percentile(lead, 10):+6.1f} "
              f"{np.percentile(lead, 90):+6.1f} {100 * off5.mean():7.1f}% {100 * refresh_off.mean():12.1f}%   {' '.join(trend)}")


if __name__ == "__main__":
    main()
