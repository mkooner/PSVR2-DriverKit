#!/usr/bin/env python3
"""
Runs the camera chain through the harness (Monado's own math, validated: the as-built variant reproduces the driver's
logged camera positions exactly) for each convention variant, re-places every logged hardware solve with it, and
reports:
  - cross-camera disagreement (same controller, different cameras, < 20 ms apart; head pose from the O rows)
  - where the controllers end up relative to the head (below / in front)
  - which way camera 0 and 1 face (yaw, pitch)
The as-built variant must reproduce what was measured on the headset directly from the world positions (validation).

usage: variants.py <steamvr log> <record csv> <harness dir>
"""
import itertools
import re
import subprocess
import sys

import numpy as np


def qrot(q, v):
    u, w = np.asarray(q[:3]), q[3]
    return v + 2 * np.cross(u, np.cross(u, v) + w * v)


def qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz])


def poses_for(hdir, mt, rt, inv, flip):
    out = subprocess.run([f'{hdir}/chain', f'{hdir}/block.bin', str(mt), str(rt), str(inv), str(flip)],
                         capture_output=True, text=True, check=True).stdout
    P = {}
    for line in out.split('\n'):
        if line.strip():
            v = line.split()
            P[int(v[0])] = (np.array(list(map(float, v[1:4]))), np.array(list(map(float, v[4:8]))))
    return P


def load(log, rec):
    heads = {}
    for line in open(rec):
        if line.startswith('O,'):
            f = line.strip().split(',')
            heads[int(f[1])] = (np.array(list(map(float, f[2:5]))), np.array(list(map(float, f[5:9]))))
    ptrs, cam, pc, out = [], None, None, []
    for line in open(log):
        m = re.search(r'Camera (0x[0-9a-f]+) RANSAC-PnP recovered pose was good enough', line)
        if m:
            cam = m.group(1)
            ptrs.append(cam) if cam not in ptrs else None
            continue
        m = re.search(r'\[pushPose\] Pose: orient \S+ \S+ \S+ \S+ pos (\S+) (\S+) (\S+)', line)
        if m:
            pc = np.array(list(map(float, m.groups())))
            continue
        if 'OPT ' in line and pc is not None:
            f = line.split()
            i = f.index('OPT')
            ts = int(f[i + 1])
            if cam is not None and ts in heads:
                out.append((ts, int(f[i + 2]), cam, pc, np.array(list(map(float, f[i + 3:i + 6]))), heads[ts]))
            pc, cam = None, None
    order = {p: k for k, p in enumerate(sorted(ptrs, key=lambda s: int(s, 16)))}
    return [(ts, d, order[c], p, w, h) for ts, d, c, p, w, h in out]


def evaluate(solves, place):
    by_pair, last = {}, {}
    below = front = 0
    for ts, dev, c, p, w, (hp, hq) in solves:
        ph = place(c, p, w, hp, hq)
        below += ph[1] < 0
        front += ph[2] < 0
        if dev in last:
            pts, pcam, pph = last[dev]
            if pcam != c and ts - pts < 20_000_000:
                by_pair.setdefault(tuple(sorted((pcam, c))), []).append(np.linalg.norm(ph - pph))
        last[dev] = (ts, c, ph)
    n = len(solves)
    return by_pair, 100 * below / n, 100 * front / n


def fmt(by_pair):
    allv = np.concatenate([np.array(v) for v in by_pair.values()])
    return np.median(allv) * 100, ' '.join(f'{a}-{b}:{np.median(v) * 100:.1f}' for (a, b), v in sorted(by_pair.items()))


def facing(q):
    fwd = qrot(q, np.array([0, 0, -1.0]))
    return np.degrees(np.arctan2(-fwd[0], -fwd[2])), np.degrees(np.arcsin(np.clip(fwd[1], -1, 1)))


def main():
    log, rec, hdir = sys.argv[1:4]
    solves = load(log, rec)

    def measured(c, p, w, hp, hq):  # the world positions the driver actually produced, into the head frame
        return qrot(np.array([-hq[0], -hq[1], -hq[2], hq[3]]), w - hp)

    bp, b, f = evaluate(solves, measured)
    med, parts = fmt(bp)
    print(f'MEASURED on hardware (driver world positions): disagreement median {med:.1f} cm [{parts}] | below {b:.0f}% front {f:.0f}%')
    rows = []
    for mt, rt, inv, flip in itertools.product((0, 1), (0, 1), (0, 1, 2), (0, 1, 2)):
        P = poses_for(hdir, mt, rt, inv, flip)

        def place(c, p, w, hp, hq, P=P):
            t, q = P[c]
            return qrot(q, p) + t

        bp, b, f = evaluate(solves, place)
        med, parts = fmt(bp)
        y0, p0 = facing(P[0][1])
        y1, p1 = facing(P[1][1])
        rows.append((med, mt, rt, inv, flip, parts, b, f, y0, p0, y1, p1))
    names = {0: 'none', 1: 'Rx180', 2: 'Ry180'}
    invn = {0: 'from>to', 1: 'never', 2: 'always'}
    for med, mt, rt, inv, flip, parts, b, f, y0, p0, y1, p1 in sorted(rows):
        built = ' <== AS BUILT' if (mt, rt, inv, flip) == (0, 1, 0, 0) else ''
        print(f'mount {"R0^T" if mt else "R0  "} rel {"R^T" if rt else "R  "} invert {invn[inv]:7s} flip {names[flip]:5s} | '
              f'median {med:5.1f} cm [{parts}] | below {b:3.0f}% front {f:3.0f}% | cam0 yaw {y0:+4.0f} pitch {p0:+4.0f} | '
              f'cam1 yaw {y1:+4.0f} pitch {p1:+4.0f}{built}')


if __name__ == '__main__':
    main()
