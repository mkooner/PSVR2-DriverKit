#!/usr/bin/env python3
"""
Solve the PS VR2 camera 0 mount (PSVR2_CAM0_MOUNT) from a test session.

Pairs the pssense "OPT" log lines (optical controller pose + IMU orientation) with the recorder's "O" rows (head
pose each camera sample was placed from) by timestamp. With the right mount, the optical controller orientation
agrees with the IMU on which way is up. The optical pose under a different mount R is
    q' = q_O * R * R_used^-1 * q_O^-1 * q_world
so R is found by minimising the up-vector disagreement over all samples (numpy only, Gauss-Newton).

Usage: mount-calib.py <steamvr-*.log> <record-*.csv> [used_pitch used_yaw used_roll]
       mount-calib.py --selftest
"""
import sys
import numpy as np

UP = np.array([0.0, 1.0, 0.0])


def qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([aw * bx + ax * bw + ay * bz - az * by,
                     aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw,
                     aw * bw - ax * bx - ay * by - az * bz])


def qinv(q):
    return np.array([-q[0], -q[1], -q[2], q[3]]) / np.dot(q, q)


def qrot(q, v):
    return qmul(qmul(q, np.array([*v, 0.0])), qinv(q))[:3]


def qaxis(axis, deg):
    a = np.radians(deg) / 2
    return np.array([*(np.array(axis, float) * np.sin(a)), np.cos(a)])


def qexp(w):
    th = np.linalg.norm(w)
    if th < 1e-12:
        return np.array([*(w / 2), 1.0])
    return np.array([*(w / th * np.sin(th / 2)), np.cos(th / 2)])


CV_TO_HEAD = qaxis([1, 0, 0], 180)


def mount_from_angles(pitch, yaw, roll):
    # Same composition as psvr2_constellation_tracking.c
    return qmul(qmul(qmul(qaxis([0, 1, 0], yaw), qaxis([1, 0, 0], pitch)), qaxis([0, 0, 1], roll)), CV_TO_HEAD)


def angles_from_mount(q):
    m = qmul(q, qinv(CV_TO_HEAD))  # Ry(yaw) Rx(pitch) Rz(roll)
    x, y, z, w = m / np.linalg.norm(m)
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    pitch = np.degrees(np.arcsin(-R[1, 2]))
    yaw = np.degrees(np.arctan2(R[0, 2], R[2, 2]))
    roll = np.degrees(np.arctan2(R[1, 0], R[1, 1]))
    return pitch, yaw, roll


def residuals(R, R_used_inv, samples):
    out = []
    for qO, qW, u_dev in samples:
        q = qmul(qmul(qmul(qmul(qO, R), R_used_inv), qinv(qO)), qW)
        out.append(qrot(q, u_dev) - UP)
    return np.concatenate(out)


def solve(samples, R_used, iters=30):
    R_used_inv = qinv(R_used)
    R = R_used.copy()
    for _ in range(iters):
        r0 = residuals(R, R_used_inv, samples)
        J = np.zeros((len(r0), 3))
        for k in range(3):
            d = np.zeros(3)
            d[k] = 1e-6
            J[:, k] = (residuals(qmul(R, qexp(d)), R_used_inv, samples) - r0) / 1e-6
        step, *_ = np.linalg.lstsq(J, -r0, rcond=None)
        R = qmul(R, qexp(step))
        R /= np.linalg.norm(R)
        if np.linalg.norm(step) < 1e-9:
            break
    err = residuals(R, R_used_inv, samples).reshape(-1, 3)
    before = residuals(R_used, R_used_inv, samples).reshape(-1, 3)
    return R, np.degrees(np.arcsin(np.clip(np.linalg.norm(err, axis=1) / 2, 0, 1)) * 2), \
        np.degrees(np.arcsin(np.clip(np.linalg.norm(before, axis=1) / 2, 0, 1)) * 2)


def load(log_path, rec_path):
    heads = {}
    for line in open(rec_path):
        if line.startswith('O,'):
            f = line.strip().split(',')
            heads[int(f[1])] = np.array(list(map(float, f[2:9])))
    samples = []
    for line in open(log_path):
        if ' OPT ' not in line:
            continue
        t = line.split(' OPT ')[1].split()
        ts = int(t[0])
        pose = np.array(list(map(float, t[2:9])))
        imu_ok = t[10] == '1'
        qimu = np.array(list(map(float, t[11:15])))
        if not imu_ok or ts not in heads:
            continue
        qO = heads[ts][3:]
        qW = pose[3:]
        u_dev = qrot(qinv(qimu), UP)  # world up seen in the device frame, from the IMU
        samples.append((qO / np.linalg.norm(qO), qW / np.linalg.norm(qW), u_dev))
    return samples


def selftest():
    rng = np.random.default_rng(1)
    R_true = mount_from_angles(-27, 21, 4)
    R_used = mount_from_angles(-20, 15, 0)
    samples = []
    for _ in range(300):
        qO = qexp(rng.normal(0, 0.6, 3))            # head orientations
        qdev = qexp(rng.normal(0, 1.0, 3))          # true device orientation in world
        # What the tracker reported with the wrong mount
        qW = qmul(qmul(qmul(qmul(qO, R_used), qinv(R_true)), qinv(qO)), qdev)
        u_dev = qrot(qinv(qdev), UP) + rng.normal(0, 0.01, 3)
        samples.append((qO, qW, u_dev / np.linalg.norm(u_dev)))
    R, err, before = solve(samples, R_used)
    print("selftest: true (-27, 21, 4), solved (%.1f, %.1f, %.1f), err %.2f deg (was %.1f)" %
          (*angles_from_mount(R), np.median(err), np.median(before)))


def main():
    if len(sys.argv) > 1 and sys.argv[1] == '--selftest':
        selftest()
        return
    log_path, rec_path = sys.argv[1], sys.argv[2]
    used = tuple(map(float, sys.argv[3:6])) if len(sys.argv) >= 6 else (-20.0, 15.0, 0.0)
    samples = load(log_path, rec_path)
    print(f"paired samples: {len(samples)}")
    if len(samples) < 30:
        print("not enough paired samples (need PSSENSE_LOG=debug and PSVR2_RECORD_FILE, controllers in view)")
        return
    R, err, before = solve(samples, mount_from_angles(*used))
    p, y, r = angles_from_mount(R)
    print(f"up-vector disagreement: {np.median(before):.1f} deg with the used mount -> {np.median(err):.1f} deg")
    print(f'PSVR2_CAM0_MOUNT="{p:.1f} {y:.1f} {r:.1f}"')


if __name__ == '__main__':
    main()
