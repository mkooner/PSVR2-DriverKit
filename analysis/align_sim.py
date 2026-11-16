#!/usr/bin/env python3
"""
Simulates learning each controller's 3DoF -> tracking-world alignment Y online (Y ~= optical * imu^-1) from accepted
optical solves, causally, as the driver would, and measures:
  - orientation jump when optical tracking stops (display switches to Y * imu instead of raw imu)
  - the gate reference error (up direction of optical vs Y * imu) on accepted solves, vs the current X-28.25 reference
Bootstrap: N consistent solves (all within BOOT_TOL of their mean), then slerp toward each solve within UPDATE_TOL
with rate ALPHA; re-bootstrap for the gates after STALE_S without fixes (display keeps the last Y).
usage: align_sim.py <steamvr log>
"""
import sys
import numpy as np

BOOT_N, BOOT_TOL, UPDATE_TOL, ALPHA, STALE_S = 15, 15.0, 20.0, 0.02, 1.0


def qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz])


def inv(q):
    return np.array([-q[0], -q[1], -q[2], q[3]])


def qrot(q, v):
    u, w = q[:3], q[3]
    return v + 2 * np.cross(u, np.cross(u, v) + w * v)


def ang(a, b):
    return 2 * np.degrees(np.arccos(min(1, abs(np.dot(a, b)))))


def nlerp(a, b, u):
    if np.dot(a, b) < 0:
        b = -b
    q = a * (1 - u) + b * u
    return q / np.linalg.norm(q)


UP = np.array([0, 1.0, 0])
X = np.array([np.sin(np.radians(28.25) / 2), 0, 0, np.cos(np.radians(28.25) / 2)])


def main():
    S = {0: [], 1: []}
    for line in open(sys.argv[1]):
        if 'OPT ' not in line:
            continue
        f = line.split()
        i = f.index('OPT')
        j = f.index('IMU')
        S[int(f[i + 2])].append((int(f[i + 1]) / 1e9, np.array(list(map(float, f[i + 6:i + 10]))),
                                 np.array(list(map(float, f[j + 2:j + 6])))))
    for h in (0, 1):
        Y = None
        boot = []
        gate_trusted = False
        last_t = None
        jumps_new, jumps_raw, err_new, err_old, rebased = [], [], [], [], 0
        P = S[h]
        for k, (t, qo, qi) in enumerate(P):
            if last_t is not None and t - last_t > 0.06:
                # optical stopped after the previous sample: display switches to Y * imu
                to, qoo, qio = P[k - 1]
                if Y is not None:
                    jumps_new.append(ang(qoo, qmul(Y, qio)))
                jumps_raw.append(ang(qoo, qio))
            if last_t is not None and t - last_t > STALE_S:
                gate_trusted = False
                boot = []
            last_t = t
            y = qmul(qo, inv(qi))
            if gate_trusted:
                err_new.append(np.degrees(np.arccos(np.clip(qrot(inv(qo), UP) @ qrot(inv(qmul(Y, qi)), UP), -1, 1))))
            err_old.append(np.degrees(np.arccos(np.clip(qrot(inv(qo), UP) @ qrot(inv(qmul(qi, X)), UP), -1, 1))))
            if not gate_trusted:
                boot.append(y if y[3] >= 0 else -y)
                if len(boot) >= BOOT_N:
                    m = np.mean([b if np.dot(b, boot[0]) >= 0 else -b for b in boot], 0)
                    m /= np.linalg.norm(m)
                    if all(ang(b, m) < BOOT_TOL for b in boot):
                        Y = m
                        gate_trusted = True
                        rebased += 1
                    boot = boot[1:]
            elif ang(y, Y) < UPDATE_TOL:
                Y = nlerp(Y, y, ALPHA)
        J1, J0, E1, E0 = map(np.array, (jumps_new, jumps_raw, err_new, err_old))
        print(f"hand {h}: {len(J0)} optical losses | display jump raw 3DoF median {np.median(J0):.0f} max {J0.max():.0f} deg"
              f" -> aligned median {np.median(J1):.1f} p90 {np.percentile(J1, 90):.1f} max {J1.max():.1f} deg")
        print(f"   gate reference error on accepted solves: current (X 28.25) median {np.median(E0):.1f} p99 "
              f"{np.percentile(E0, 99):.1f} max {E0.max():.1f} | aligned median {np.median(E1):.1f} p99 "
              f"{np.percentile(E1, 99):.1f} max {E1.max():.1f} | bootstraps {rebased}")


if __name__ == '__main__':
    main()
