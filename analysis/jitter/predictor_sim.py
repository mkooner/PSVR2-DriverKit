#!/usr/bin/env python3
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
"""Head-pose predictors on recorded head motion (the driver's record-*.csv, 2 kHz gyro "I" rows).

SteamVR on this machine shows frames without reprojection, so the predicted head pose a frame is drawn with is what
reaches the eye. This replays recorded gyro data, samples a pose once per refresh as a game would, and measures with
logjitter.py's measure (the part of each frame's rotation a smooth curve through +-4 frames doesn't explain) how
rough each way of producing that pose is. Reference, same measure, head turning 60-150 deg/s (vrposelog 2026-11-16):
Sony's driver unpredicted 0.010/0.062 deg (p50/p90), predicted 0.017/0.121; ours then 0.031-0.045/0.159-0.219 and
0.054-0.085/0.290-0.602.

Truth: the gyro integrated. What it can't see: tracking drift corrections (SLAM), and SteamVR's own behaviour.
Usage: predictor_sim.py record.csv [...]
"""
import sys

import numpy as np

PERIOD = 1 / 89.91


def qmul(a, b):
    x1, y1, z1, w1 = a.T
    x2, y2, z2, w2 = b.T
    return np.stack([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2], 1)


def inv(q):
    return q * np.array([-1, -1, -1, 1])


def qexp(v):
    a = np.linalg.norm(v, axis=1, keepdims=True)
    s = np.where(a > 1e-12, np.sin(a / 2) / np.maximum(a, 1e-12), 0.5)
    return np.concatenate([v * s, np.cos(a / 2)], 1)


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
    n[:half] = np.nan
    n[-half:] = np.nan
    return n


def load(path):
    t, w = [], []
    for line in open(path):
        if line.startswith('I,'):
            x = line.split(',')
            t.append(int(x[2]))  # the headset's own sample clock (vts): exact 0.5 ms steps
            w.append((float(x[3]), float(x[4]), float(x[5])))
    t = np.array(t) / 1e9
    w = np.array(w)
    keep = np.r_[True, np.diff(t) > 0]
    return t[keep] - t[keep][0], w[keep]


class Motion:
    def __init__(self, t, w):
        self.t, self.w = t, w
        dt = np.diff(t)
        dq = qexp(w[:-1] * dt[:, None])
        q = np.zeros((len(t), 4))
        q[0] = (0, 0, 0, 1)
        for i in range(len(dq)):  # body-frame integration
            q[i + 1] = qmul(q[i:i + 1], dq[i:i + 1])[0]
            if i % 2000 == 0:
                q[i + 1] /= np.linalg.norm(q[i + 1])
        self.q = q
        self.csum = np.vstack([np.zeros(3), np.cumsum(w[:-1] * dt[:, None], axis=0)])

    def at(self, tq):
        """True orientation at times tq (slerp between samples via the local rotation)."""
        i = np.clip(np.searchsorted(self.t, tq) - 1, 0, len(self.t) - 2)
        frac = (tq - self.t[i])
        return qmul(self.q[i], qexp(self.w[i] * frac[:, None]))

    def idx(self, tq):
        return np.clip(np.searchsorted(self.t, tq, side='right') - 1, 0, len(self.t) - 1)

    def mean_w(self, tq, win):
        """Mean gyro over the last win seconds before tq (win 0: the latest sample)."""
        i1 = self.idx(tq)
        if win <= 0:
            return self.w[i1]
        i0 = np.maximum(self.idx(tq - win), 0)
        n = np.maximum(i1 - i0, 1)
        cw = np.vstack([np.zeros(3), np.cumsum(self.w, axis=0)])
        return (cw[i1 + 1] - cw[i0 + 1]) / n[:, None]

    def slope_w(self, tq, win):
        """Angular acceleration: mean gyro of the newer half of win minus the older half, over win/2."""
        a = self.mean_w(tq, win / 2)
        b = self.mean_w(tq - win / 2, win / 2)
        return (a - b) / (win / 2)


def report(name, m, frames, pred_q, speed):
    """Jitter of the pose sequence a game would draw with, and its error against the true pose at display time."""
    ref = pred_q[len(pred_q) // 2]
    v = np.degrees(qlog(qmul(np.repeat(inv(ref[None]), len(pred_q), 0), pred_q)))
    v = np.unwrap(np.radians(v), axis=0)
    v = np.degrees(v)
    r = residual(v)
    out = [f"{name:52s}"]
    for lo, hi in ((10, 60), (60, 150), (150, 400)):
        s = (speed >= lo) & (speed < hi) & np.isfinite(r)
        out.append(f"{np.percentile(r[s], 50):.3f}/{np.percentile(r[s], 90):.3f}" if s.sum() > 30 else "   -   ")
    print("  " + "   ".join(out))


def main():
    for path in sys.argv[1:]:
        t, w = load(path)
        m = Motion(t, w)
        rng = np.random.default_rng(1)
        frames = np.arange(t[0] + 0.5, t[-1] - 0.5, PERIOD)
        speed = np.degrees(np.linalg.norm(m.mean_w(frames, 0.02), axis=1))
        print(f"{path}: {t[-1]:.0f} s, {len(frames)} frames; turning 10-60: {((speed>=10)&(speed<60)).sum()}, "
              f"60-150: {((speed>=60)&(speed<150)).sum()}, 150+: {(speed>=150).sum()}")
        print(f"  {'jitter p50/p90 deg, by head speed (deg/s)':52s}   10-60         60-150        150-400")
        report("true pose at frame time (floor: real motion + gyro)", m, frames, m.at(frames), speed)
        for H in (0.0125, 0.022):
            print(f"  -- predicted {H*1e3:.1f} ms ahead")
            report("  true pose at frame time + H (perfect prediction)", m, frames, m.at(frames + H), speed)
            q0 = m.at(frames)
            for win in (0, 0.002, 0.004, 0.008, 0.016):
                wv = m.mean_w(frames, win)
                report(f"  constant velocity, gyro mean over {win*1e3:.0f} ms", m, frames, qmul(q0, qexp(wv * H)), speed)
            for win in (0.008, 0.016, 0.032):
                wv = m.mean_w(frames, 0.004)
                acc = m.slope_w(frames, win)
                report(f"  velocity (4 ms) + acceleration over {win*1e3:.0f} ms", m, frames,
                       qmul(q0, qexp(wv * H + 0.5 * acc * H * H)), speed)
            # The new driver as built: a pose thread every 1 ms predicts now + H from the latest gyro sample with zero
            # velocity handed over; the game reads whatever was sent last (0-1 ms old).
            stale = rng.uniform(0, 0.001, len(frames))
            ts = frames - stale
            report("  as built: 1 ms pose thread, zero velocity, raw gyro", m, frames,
                   qmul(m.at(ts), qexp(m.mean_w(ts, 0) * H)), speed)
            # A timestamp that is wrong by a varying amount (clock mapping jitter): pose for t + noise.
            for jit in (0.0005, 0.001, 0.002):
                tj = frames + rng.normal(0, jit, len(frames))
                report(f"  perfect prediction, pose time wrong by {jit*1e3:.1f} ms rms", m, frames, m.at(tj + H), speed)
    return 0


if __name__ == '__main__':
    sys.exit(main())
