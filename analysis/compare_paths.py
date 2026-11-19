#!/usr/bin/env python3
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
"""
One table, same measurements, for every way the game reached the headset: what was shown per displayed frame against
where the head really was when it was shown.

Sessions (any number, any mix):
  openxr:<compframes-*-openxr.csv>   Monado OpenXR (launch-openxr.sh; Monado's compositor log)
  direct:<compframes-*.csv>          SteamVR in the plugin's direct mode (Monado's compositor inside vrserver)
  steamvr:<steamvr-frames-*.csv>     SteamVR normal mode (tools/vrframetiming; uses steamvr-vrposes-* next to it)

Per displayed frame:
  - head reference: Monado: the device's own pose every 2 ms (T rows); SteamVR: SteamVR's own head pose 1000/s
    (steamvr-vrposes, same space as the game's pose)
  - game pose: the pose the game drew the frame with (Monado: layer 0 view poses, Y rows, eye midpoint; the constant
    transform between the game's space and the device's is fitted; SteamVR: m_HmdPose)
  - shown time: Monado: the vblank after the present + the median photon offset; SteamVR: the frame's vsync (frame
    clock linked to CLOCK_MONOTONIC by vrframetiming's "# link" rows) + one refresh
  - repeated: the same game frame again (Monado: same layer timestamp; SteamVR: presents > 1 or reprojected)
  - rotation after re-warp (Monado only: SteamVR's compositor doesn't report its re-warp pose)
Errors are against the head at the shown time; frame-to-frame changes are what shows as jitter (a constant offset is
latency, not jitter, and the fitted space transform absorbs it).

Usage: compare_paths.py kind:path [kind:path ...] [--window from_s to_s]
"""
import math
import os
import sys

import numpy as np

PERIOD = 1 / 89.91


def qmul(a, b):
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])


def qconj(q):
    return np.array([-q[0], -q[1], -q[2], q[3]])


def qrot(q, v):
    return qmul(qmul(q, np.r_[v, 0.0]), qconj(q))[:3]


def rotvec_deg(q):
    if q[3] < 0:
        q = -q
    s = np.linalg.norm(q[:3])
    return np.zeros(3) if s < 1e-12 else q[:3] / s * math.degrees(2 * math.atan2(s, q[3]))


def mat_to_quat(m):
    tr = m[0][0] + m[1][1] + m[2][2]
    if tr > 0:
        s = math.sqrt(tr + 1) * 2
        return np.array([(m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s, 0.25 * s])
    if m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1 + m[0][0] - m[1][1] - m[2][2]) * 2
        return np.array([0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s, (m[2][1] - m[1][2]) / s])
    if m[1][1] > m[2][2]:
        s = math.sqrt(1 + m[1][1] - m[0][0] - m[2][2]) * 2
        return np.array([(m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s, (m[0][2] - m[2][0]) / s])
    s = math.sqrt(1 + m[2][2] - m[0][0] - m[1][1]) * 2
    return np.array([(m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s, (m[1][0] - m[0][1]) / s])


class Truth:
    """Head reference: times (s, CLOCK_MONOTONIC) and poses (px,py,pz,qx,qy,qz,qw)."""

    def __init__(self, t, poses):
        o = np.argsort(t)
        self.t = np.asarray(t, float)[o]
        self.p = np.asarray(poses, float)[o]
        ok = np.linalg.norm(self.p[:, :3], axis=1) > 1e-6  # untracked (startup) samples report position 0
        self.t, self.p = self.t[ok], self.p[ok]
        for i in range(1, len(self.p)):
            if np.dot(self.p[i, 3:], self.p[i - 1, 3:]) < 0:
                self.p[i, 3:] *= -1
        self.gap = max(3 * float(np.median(np.diff(self.t))), 0.006) if len(self.t) > 2 else 0.01

    def at(self, t):
        i = np.searchsorted(self.t, t)
        if i <= 0 or i >= len(self.t) or self.t[i] - self.t[i - 1] > self.gap:
            return None
        a = (t - self.t[i - 1]) / (self.t[i] - self.t[i - 1])
        p = self.p[i - 1] + a * (self.p[i] - self.p[i - 1])
        p[3:] /= np.linalg.norm(p[3:])
        return p


def fit_space(game, truth_at_shown):
    """Constant rigid transform game space -> head reference space (rotation then translation), robust medians."""
    rs = []
    for g, h in zip(game, truth_at_shown):
        r = qmul(h[3:], qconj(g[3:]))
        rs.append(r if r[3] >= 0 else -r)
    R = np.median(np.array(rs), axis=0)
    R /= np.linalg.norm(R)
    T = np.median(np.array([h[:3] - qrot(R, g[:3]) for g, h in zip(game, truth_at_shown)]), axis=0)
    return R, T


def load_monado(path):
    W, P, B, Y, T = [], {}, [], {}, ([], [])
    for line in open(path):
        x = line.strip().split(',')
        k = x[0]
        try:
            if k == 'W' and len(x) >= 13:
                W.append((int(x[2]), int(x[3]), int(x[5]), np.array([float(v) for v in x[6:13]])))
            elif k == 'P' and len(x) >= 4:
                P[int(x[2])] = int(x[1])
            elif k == 'B' and len(x) >= 2:
                B.append(int(x[1]))
            elif k == 'Y' and len(x) >= 17:
                e0 = np.array([float(v) for v in x[3:10]])
                e1 = np.array([float(v) for v in x[10:17]])
                Y[int(x[1])] = np.r_[(e0[:3] + e1[:3]) / 2, e0[3:]]
            elif k == 'T' and len(x) >= 9:
                T[0].append(int(x[1]) * 1e-9)
                T[1].append([float(v) for v in x[2:9]])
        except ValueError:
            pass  # a row cut off at the end of the file
    if not T[0] or not Y:
        # Sessions recorded before the compositor log had T/Y rows (2026-11-18 11:25): the SteamVR plugin's direct log
        # next to it has the head reference (T) and the game's pose keyed by its display time (R).
        d = os.path.join(os.path.dirname(path), os.path.basename(path).replace('compframes-', 'direct-'))
        if not os.path.exists(d):
            raise SystemExit(f"{path}: no T/Y rows and no {d}")
        R = {}
        for line in open(d):
            x = line.strip().split(',')
            try:
                if x[0] == 'T' and len(x) >= 9 and not T[0][:0]:
                    T[0].append(int(x[1]) * 1e-9)
                    T[1].append([float(v) for v in x[2:9]])
                elif x[0] == 'R' and len(x) >= 18:
                    R[int(x[3])] = np.array([float(v) for v in x[4:11]])
            except ValueError:
                pass
        by_ts = R
    truth = Truth(*T)
    B = np.array(sorted(B), dtype=np.int64)
    if not truth.t.size:
        raise SystemExit(f"{path}: no usable head reference")
    shown_vb = {}
    if len(B) > 10:
        for fid, pred, ts, head in W:
            if fid in P:
                i = np.searchsorted(B, P[fid])
                if i < len(B):
                    shown_vb[fid] = int(B[i])
    offs = [pred - shown_vb[fid] for fid, pred, ts, head in W if fid in shown_vb]
    photon = float(np.median(offs)) if offs else 0.0
    frames = []
    prev_ts = None
    by_ts = locals().get('by_ts')
    for fid, pred, ts, head in W:
        shown = ((shown_vb[fid] + photon) if fid in shown_vb else pred) * 1e-9
        game = by_ts.get(ts) if by_ts is not None else Y.get(fid)
        frames.append(dict(t=shown, slip=(shown - pred * 1e-9) * 1e3, repeat=(ts == prev_ts), game=game, warp=head))
        prev_ts = ts
    return frames, truth, dict(vblanks=len(B), photon_ms=photon / 1e6)


def load_steamvr(path):
    base = os.path.basename(path)
    vp = os.path.join(os.path.dirname(path), base.replace('steamvr-frames-', 'steamvr-vrposes-'))
    T = ([], [])
    for line in open(vp):
        x = line.strip().split(',')
        if not x[0].isdigit() or len(x) < 12:
            continue
        try:
            if x[2] != '1':
                continue
            T[0].append(int(x[0]) * 1e-9)
            q = [float(v) for v in x[5:9]]
            p = [float(v) for v in x[9:12]]
            T[1].append(p + q)
        except ValueError:
            pass
    truth = Truth(*T)
    links, rows, header = [], [], None
    for line in open(path):
        if line.startswith('# link '):
            x = line.split()
            links.append(float(x[2]) - float(x[3]))
        elif line.startswith('frame,'):
            header = line.strip().split(',')
        elif header and line[:1].isdigit():
            x = line.strip().split(',')
            if len(x) == len(header):
                rows.append(dict(zip(header, x)))
    if len(links) < 5:
        raise SystemExit(f"{path}: no '# link' rows to tie SteamVR's frame clock to CLOCK_MONOTONIC")
    links = np.array(links)
    mode = np.median(links[np.abs(links - np.median(links)) < 0.002])
    frames = []
    seen = set()
    for r in rows:
        f = int(r['frame'])
        if f in seen:
            continue
        seen.add(f)
        if r['pose_valid'] != '1':
            continue
        m = [[float(r[f'm{a}{b}']) for b in range(4)] for a in range(3)]
        q = mat_to_quat(m)
        game = np.r_[m[0][3], m[1][3], m[2][3], q / np.linalg.norm(q)]
        shown = float(r['t_s']) + mode + PERIOD
        presents, flags = int(r['presents']), int(r['reproj_flags'])
        frames.append(dict(t=shown, slip=0.0, repeat=presents > 1 or (flags & 0x0b) != 0, game=game, warp=None,
                           dropped=int(r['dropped']), mis=int(r['mispresented'])))
    frames.sort(key=lambda d: d['t'])
    return frames, truth, dict(link=len(links))


def measure(frames, truth, window=None):
    t0 = frames[0]['t'] if frames else 0
    if window:
        frames = [f for f in frames if window[0] <= f['t'] - t0 <= window[1]]
    use = [(f, truth.at(f['t']), truth.at(f['t'] - 0.0111)) for f in frames]
    use = [u for u in use if u[1] is not None and u[2] is not None]
    if len(use) < 20:
        return None
    games = [(f['game'], h) for f, h, _ in use if f['game'] is not None]
    R, T = fit_space([g for g, _ in games], [h for _, h in games]) if len(games) > 20 else (None, None)
    out = dict(n=len(use), repeat=[], slip=[], speed=[], lin=[], grot=[], gpos=[], wrot=[])
    for f, h, hp in use:
        out['repeat'].append(f['repeat'])
        out['slip'].append(f['slip'])
        out['speed'].append(np.linalg.norm(rotvec_deg(qmul(h[3:], qconj(hp[3:])))) / 0.0111)
        out['lin'].append(1000 * np.linalg.norm(h[:3] - hp[:3]) / 0.0111)
        if f['game'] is not None and R is not None:
            gq = qmul(R, f['game'][3:])
            gp = qrot(R, f['game'][:3]) + T
            out['grot'].append(rotvec_deg(qmul(gq, qconj(h[3:]))))
            out['gpos'].append(1000 * (gp - h[:3]))
        else:
            out['grot'].append(np.full(3, np.nan))
            out['gpos'].append(np.full(3, np.nan))
        w = f['warp']
        out['wrot'].append(rotvec_deg(qmul(w[3:] / np.linalg.norm(w[3:]), qconj(h[3:]))) if w is not None
                           else np.full(3, np.nan))
    for k in ('repeat', 'slip', 'speed', 'lin'):
        out[k] = np.array(out[k])
    for k in ('grot', 'gpos', 'wrot'):
        out[k] = np.array(out[k])
    # Frames where the head reference itself moves impossibly fast (> 1000 deg/s or 5 m/s within 11 ms: a glitch in the
    # reference, e.g. the driver's pose for "now", 2026-11-18 11:50) are measurement errors: their errors are NaN.
    bad = (out['speed'] > 1000) | (out['lin'] > 5000)
    out['excluded'] = int(bad.sum())
    for k in ('grot', 'gpos', 'wrot'):
        out[k][bad] = np.nan
    out['dropped'] = sum(f.get('dropped', 0) for f, _, _ in use)
    out['mis'] = sum(f.get('mis', 0) for f, _, _ in use)
    return out


def pc(v, p):
    v = np.asarray(v, float)
    v = v[np.isfinite(v)]
    return float(np.percentile(v, p)) if len(v) else float('nan')


def rows_for(m):
    still = (m['speed'] < 3) & (m['lin'] < 20)
    turn = (m['speed'] > 20) & (m['speed'] < 200)
    move = m['lin'] > 100
    sw = lambda a: np.r_[np.nan, np.linalg.norm(np.diff(a, axis=0), axis=1)]
    gr, gp, wr = sw(m['grot']), sw(m['gpos']), sw(m['wrot'])
    gpa = np.linalg.norm(m['gpos'], axis=1)
    r = {}
    r['displayed frames'] = f"{m['n']}"
    r['game frame repeated / reprojected'] = f"{m['repeat'].sum()} ({100 * m['repeat'].mean():.2f}%)"
    r['dropped / mis-presented (SteamVR)'] = f"{m['dropped']} / {m['mis']}"
    r['slip > half a refresh'] = f"{int((np.abs(m['slip']) > 5.6).sum())}"
    r['excluded (reference glitch)'] = f"{m['excluded']}"
    for name, sel in (('still', still), ('turning', turn), ('moving', move)):
        if sel.sum() < 5:
            continue
        r[f'{name}: frames'] = f"{sel.sum()}"
        r[f'{name}: game pos swing p50/p90/p99 mm'] = f"{pc(gp[sel], 50):.2f} / {pc(gp[sel], 90):.2f} / {pc(gp[sel], 99):.2f}"
        r[f'{name}: game rot swing p50/p90/p99 deg'] = f"{pc(gr[sel], 50):.3f} / {pc(gr[sel], 90):.3f} / {pc(gr[sel], 99):.3f}"
        r[f'{name}: game pos off p90 mm'] = f"{pc(gpa[sel], 90):.2f}"
        r[f'{name}: rot after re-warp swing p90/p99 deg'] = f"{pc(wr[sel], 90):.3f} / {pc(wr[sel], 99):.3f}"
    return r


def main():
    args = sys.argv[1:]
    window = None
    if '--window' in args:
        i = args.index('--window')
        window = (float(args[i + 1]), float(args[i + 2]))
        args = args[:i] + args[i + 3:]
    if not args:
        print(__doc__)
        return 2
    cols = {}
    for spec in args:
        kind, path = spec.split(':', 1)
        frames, truth, info = (load_steamvr(path) if kind == 'steamvr' else load_monado(path))
        m = measure(frames, truth, window)
        name = f"{kind} {os.path.basename(path).split('-', 1)[-1][:22]}"
        cols[name] = rows_for(m) if m else {'displayed frames': 'too few'}
    keys = []
    for c in cols.values():
        for k in c:
            if k not in keys:
                keys.append(k)
    w0 = max(len(k) for k in keys) + 2
    names = list(cols)
    width = max(26, *(len(n) for n in names)) + 2
    print(' ' * w0 + ''.join(n.ljust(width) for n in names))
    for k in keys:
        print(k.ljust(w0) + ''.join(cols[n].get(k, '-').ljust(width) for n in names))
    if window:
        print(f"(window {window[0]:.0f}-{window[1]:.0f} s after each session's first frame)")
    return 0


if __name__ == '__main__':
    sys.exit(main())
