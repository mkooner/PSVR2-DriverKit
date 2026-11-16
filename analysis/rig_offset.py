#!/usr/bin/env python3
"""
Estimates the camera rig position relative to the head pose origin (PSVR2_CAM_RIG_OFFSET) from a session.

If the cameras actually sit d (head frame) away from where the driver puts them, a controller that is not moving
appears to move as the head turns: its solved world position is w_i = c - R_head,i * d. In windows where the
controller is still (its IMU rotation under 1 deg; rest it on a table) and the head turns, least squares over d and
one centre c per window recovers d.

Recording procedure: rest one controller on a table in view of the headset; slowly turn the head left/right and
up/down for 20-30 s, keeping the controller in view. Run with PSSENSE_LOG=debug (launch-steamvr.sh does).

usage: rig_offset.py <steamvr log> <record csv>
       rig_offset.py --selftest
"""
import re
import sys

import numpy as np


def rotm(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def qangle(a, b):
    d = abs(np.dot(a, b)) / np.linalg.norm(a) / np.linalg.norm(b)
    return 2 * np.degrees(np.arccos(min(1.0, d)))


def windows(samples, still_deg=1.0, turn_deg=8.0, span_s=0.8):
    """samples: (t_s, device, world_pos, R_head, q_imu) sorted by device and time."""
    out = []
    for dev in sorted({s[1] for s in samples}):
        P = [s for s in samples if s[1] == dev]
        i = 0
        while i < len(P):
            j = i
            # A window ends at span_s, at a gap in the solves, or at a jump in position (the controller was moved).
            while (j + 1 < len(P) and P[j + 1][0] - P[i][0] < span_s and P[j + 1][0] - P[j][0] < 0.1 and
                   np.linalg.norm(P[j + 1][2] - P[j][2]) < 0.05):
                j += 1
            W = P[i:j + 1]
            if len(W) >= 8:
                still = max(qangle(W[0][4], s[4]) for s in W) < still_deg
                turn = max(np.degrees(np.arccos(np.clip((np.trace(W[0][3].T @ s[3]) - 1) / 2, -1, 1))) for s in W)
                if still and turn > turn_deg:
                    out.append(W)
            i = j + 1
    return out


def solve(wins):
    n = len(wins)
    rows = sum(len(W) for W in wins)
    A = np.zeros((3 * rows, 3 + 3 * n))
    b = np.zeros(3 * rows)
    r = 0
    for k, W in enumerate(wins):
        for s in W:
            A[3 * r:3 * r + 3, 0:3] = -s[3]
            A[3 * r:3 * r + 3, 3 + 3 * k:6 + 3 * k] = np.eye(3)
            b[3 * r:3 * r + 3] = s[2]
            r += 1
    sol, _, _, sv = np.linalg.lstsq(A, b, rcond=None)
    res = A @ sol - b
    s2 = res @ res / max(1, len(b) - A.shape[1])
    cov = s2 * np.linalg.inv(A.T @ A)[:3, :3]
    return sol[:3], np.sqrt(np.diag(cov)), np.sqrt(s2), sv[0] / sv[-1]


def selftest():
    rng = np.random.default_rng(1)
    d_true = np.array([0.04, -0.02, -0.07])
    samples = []
    t = 0.0
    for seg in range(30):
        c = rng.normal(size=3) * 0.3 + np.array([0, -0.4, -0.5])
        qi = np.array([0, 0, 0, 1.0])
        for k in range(48):  # 0.8 s at 60 Hz
            yaw = np.radians(25 * np.sin(2 * np.pi * (t / 4.0)))
            pitch = np.radians(10 * np.sin(2 * np.pi * (t / 5.0)))
            qy = np.array([0, np.sin(yaw / 2), 0, np.cos(yaw / 2)])
            qp = np.array([np.sin(pitch / 2), 0, 0, np.cos(pitch / 2)])
            x1, y1, z1, w1 = qy
            x2, y2, z2, w2 = qp
            q = np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                          w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])
            Rh = rotm(q)
            w = c - Rh @ d_true + rng.normal(size=3) * 0.01  # 1cm solve noise
            samples.append((t, 0, w, Rh, qi))
            t += 1 / 60
        t += 1.0  # the controller is moved to the next spot between segments
    d, sig, rms, cond = solve(windows(samples))
    ok = np.all(np.abs(d - d_true) < 3 * sig + 0.005)
    print(f"selftest: true {np.round(d_true * 100, 1)} cm, solved {np.round(d * 100, 1)} cm +- {np.round(sig * 100, 1)}"
          f" (rms {rms * 100:.1f} cm, cond {cond:.0f}) -> {'PASS' if ok else 'FAIL'}")
    return ok


def load(log, rec):
    heads = {}
    for line in open(rec):
        if line.startswith('O,'):
            f = line.strip().split(',')
            heads[int(f[1])] = (np.array(list(map(float, f[2:5]))), np.array(list(map(float, f[5:9]))))
    out = []
    for line in open(log):
        if 'OPT ' not in line:
            continue
        f = line.split()
        i = f.index('OPT')
        j = f.index('IMU')
        ts = int(f[i + 1])
        if ts in heads:
            hp, hq = heads[ts]
            w = np.array(list(map(float, f[i + 3:i + 6])))
            out.append((ts / 1e9, int(f[i + 2]), w, rotm(hq), np.array(list(map(float, f[j + 2:j + 6])))))
    out.sort(key=lambda s: (s[1], s[0]))
    return out


def main():
    if len(sys.argv) > 1 and sys.argv[1] == '--selftest':
        sys.exit(0 if selftest() else 1)
    samples = load(sys.argv[1], sys.argv[2])
    wins = windows(samples)
    print(f"{len(samples)} solves, {len(wins)} windows with a still controller and a turning head")
    if len(wins) < 10:
        print("not enough windows for a reliable estimate (need >= 10): record with the controller resting in view "
              "while turning the head")
        return
    d, sig, rms, cond = solve(wins)
    print(f"rig offset {np.round(d * 100, 1)} cm +- {np.round(sig * 100, 1)} (residual rms {rms * 100:.1f} cm, "
          f"condition {cond:.0f})")
    if np.all(sig < 0.01) and np.linalg.norm(d) < 0.15:
        print(f'PSVR2_CAM_RIG_OFFSET="{d[0] * 100:.1f} {d[1] * 100:.1f} {d[2] * 100:.1f}"')
    else:
        print("estimate not well determined or implausible (> 15cm): not suggesting a value")


if __name__ == '__main__':
    main()
