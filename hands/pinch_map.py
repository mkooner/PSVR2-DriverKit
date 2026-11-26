#!/usr/bin/env python3
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
"""The pinch map: pinch vs not pinch from Hand Landmarker's output (both cameras: gesture embedding + joint measures,
as gesture_map.py), fitted on one labelled session and tested on another, with the driver's click timing replayed
(average of the last N frames' pinch probability, on above a threshold, each change held some ms; restart after a
100 ms gap). Scored per pinch type: held, slow, quick taps, bursts of three, and open hands that must not click.

Sessions: name=detection.log,labels.csv,replay_to_session_ns (MERCURY_DETECTION_LOG "E" lines from hand_sim).
usage: pinch_map.py sessionA sessionB [--write header.h]"""
import sys, csv, bisect, collections, itertools
import numpy as np

PALM = lambda P: np.linalg.norm(P[5] - P[17]) + 1e-6
def joints_feats(w):
    P = w.reshape(21, 3); palm = PALM(P); f = []
    for tip in (8, 12, 16, 20):
        f.append(np.linalg.norm(P[4] - P[tip]) / palm); f.append(np.linalg.norm(P[tip] - P[0]) / palm)
    for a, b, c in ((5, 6, 7), (6, 7, 8), (9, 10, 11), (10, 11, 12), (13, 14, 15), (14, 15, 16), (17, 18, 19), (18, 19, 20), (1, 2, 3), (2, 3, 4)):
        u, v = P[a] - P[b], P[c] - P[b]
        f.append(np.dot(u, v) / (np.linalg.norm(u) * np.linalg.norm(v) + 1e-9))
    return np.array(f, dtype=np.float32)

NOT_PINCH = ('open', 'fist', 'point', 'claw', 'position')

def load(spec):
    log, lab, off = spec.split(',')
    off = int(off)
    F = collections.defaultdict(dict)
    for line in open(log):
        if not line.startswith('E,'):
            continue
        v = line.rstrip('\n').split(',')
        if float(v[4]) < 0.5:
            continue
        nums = np.array(v[6:], dtype=np.float32)
        F[(int(v[1]) + off, int(v[2]))][int(v[3])] = (nums[126:254], joints_feats(nums[:63]))
    frames = {h: sorted((k[0], vec(F[k])) for k in F if k[1] == h) for h in (0, 1)}
    P = [r for r in csv.DictReader(open(lab)) if r['hand'] in ('left', 'right')]
    return dict(name=log.split('/')[-1], frames=frames, prompts=P)

def vec(fr):
    z128, z18 = np.zeros(128, np.float32), np.zeros(18, np.float32)
    e0 = fr[0][0] if 0 in fr else z128; e1 = fr[1][0] if 1 in fr else z128
    j0 = fr[0][1] if 0 in fr else z18; j1 = fr[1][1] if 1 in fr else z18
    f = [0 in fr, 1 in fr]
    return np.concatenate([e0, e1, f, j0, j1, f]).astype(np.float32)

def labelled(sess, hand):
    """(features, 1 pinch / 0 not) for the frames inside fit-worthy prompts of this hand."""
    T = sess['frames'][hand]; ts = [t for t, _ in T]
    X, y = [], []
    for p in sess['prompts']:
        if p['hand'] != ('left' if hand == 0 else 'right'):
            continue
        g = p['gesture']
        if g == 'pinch':
            lab = 1
        elif g in NOT_PINCH:
            lab = 0
        else:
            continue
        i, j = bisect.bisect_left(ts, int(p['start_ns'])), bisect.bisect_right(ts, int(p['end_ns']))
        for k in range(i, j):
            X.append(T[k][1]); y.append(lab)
    return np.array(X), np.array(y)

def fit(X, y, l2=1e-2, it=800, lr=0.5):
    mu, sd = X.mean(0), X.std(0) + 1e-6
    Xn = (X - mu) / sd
    w = np.zeros(X.shape[1]); b = 0.0
    cw = np.where(y == 1, len(y) / (2 * max(y.sum(), 1)), len(y) / (2 * max((1 - y).sum(), 1)))
    for _ in range(it):
        p = 1 / (1 + np.exp(-(Xn @ w + b)))
        g = (p - y) * cw / len(y)
        w -= lr * (Xn.T @ g + l2 * w); b -= lr * g.sum()
    return dict(mu=mu, sd=sd, w=w, b=b)

def prob(m, X):
    return 1 / (1 + np.exp(-(((X - m['mu']) / m['sd']) @ m['w'] + m['b'])))

def clicks(sess, hand, m, N, th, hold_ms):
    """The click timeline the driver would give: [(t, on)]."""
    T = sess['frames'][hand]
    if not T:
        return []
    p = prob(m, np.array([v for _, v in T]))
    out, hist, on, since, last = [], [], False, 0, None
    for (t, _), pi in zip(T, p):
        if last is not None and t - last > 100e6:
            hist, on, since = [], False, 0
        last = t
        hist.append(pi); hist = hist[-N:]
        want = (sum(hist) / len(hist)) >= th
        if want != on:
            if since == 0:
                since = t
            if t - since >= hold_ms * 1e6:
                on, since = want, 0
        else:
            since = 0
        out.append((t, on))
    return out

def score(sess, hand, C):
    hs = 'left' if hand == 0 else 'right'
    ts = [t for t, _ in C]
    def seg(a, b):
        i, j = bisect.bisect_left(ts, a), bisect.bisect_right(ts, b); return [C[k][1] for k in range(i, j)]
    def presses(v): return sum(1 for i in range(1, len(v)) if v[i] and not v[i - 1]) + (1 if v and v[0] else 0)
    r = collections.defaultdict(list)
    hp = [p for p in sess['prompts'] if p['hand'] == hs]
    for i, p in enumerate(hp):
        a, b, g, o = int(p['start_ns']), int(p['end_ns']), p['gesture'], p['orientation']
        v = seg(a, b)
        if not v:
            continue
        if g == 'pinch' and o == 'tap':
            r['tap clicked'].append(any(seg(a, b + int(0.35e9))))
        elif g == 'pinch' and o == 'slow_hold':
            pr = hp[i - 1] if i else None
            ce = int(pr['end_ns']) if pr is not None and pr['orientation'] == 'slow_close' else a
            cs = int(pr['start_ns']) if pr is not None and pr['orientation'] == 'slow_close' else a
            first = next((t for t, on in C if cs <= t <= b and on), None)
            r['slow caught'].append(first is not None)
            if first is not None:
                r['slow click vs touch s'].append((first - ce) / 1e9)
        elif g == 'pinch':
            r['held caught'].append(any(v)); r['held on'].append(sum(v) / len(v))
        elif g == 'burst':
            r['burst clicks'].append(presses(v))
        elif g == 'open' and o == 'tap_release':
            r['tap released'].append(not all(v))
        elif g in NOT_PINCH:
            r['not-pinch clicked'].append(any(v))
    return r

def summary(r):
    f = lambda k: (sum(r[k]), len(r[k])) if r.get(k) else (0, 0)
    held = f('held caught'); slow = f('slow caught'); tap = f('tap clicked'); rel = f('tap released'); bad = f('not-pinch clicked')
    caught = held[0] + slow[0] + tap[0]; total = held[1] + slow[1] + tap[1]
    on = 100 * np.mean(r['held on']) if r.get('held on') else float('nan')
    bursts = ','.join(map(str, r.get('burst clicks', []))) or '-'
    st = ','.join(f'{x:+.1f}' for x in r.get('slow click vs touch s', [])) or '-'
    return caught, total, bad, (f"pinch caught {caught}/{total} (held {held[0]}/{held[1]}, slow {slow[0]}/{slow[1]}, taps {tap[0]}/{tap[1]}), "
            f"held on {on:.0f}%, taps released {rel[0]}/{rel[1]}, bursts {bursts}, slow vs touch {st}; NOT-pinch clicked {bad[0]}/{bad[1]}")

if __name__ == '__main__':
    A, B = load(sys.argv[1]), load(sys.argv[2])
    grid = list(itertools.product((1, 3, 5), (0.5, 0.6, 0.7, 0.8), (0, 30, 60)))
    for hand in (0, 1):
        print(f"### {'left' if hand == 0 else 'right'} hand")
        models = {}
        for tr, te in ((A, B), (B, A)):
            X, y = labelled(tr, hand); models[tr['name']] = fit(X, y)
        # pick the timing on the sum over both directions (fit one session, test the other)
        best = None
        for N, th, hold in grid:
            tot = 0
            for tr, te in ((A, B), (B, A)):
                c, n, bad, _ = summary(score(te, hand, clicks(te, hand, models[tr['name']], N, th, hold)))
                tot += (c / max(n, 1)) - (bad[0] / max(bad[1], 1))
            if best is None or tot > best[0]:
                best = (tot, N, th, hold)
        _, N, th, hold = best
        print(f"  timing chosen: average {N} frames, threshold {th}, hold {hold} ms")
        for tr, te in ((A, B), (B, A)):
            print(f"  fit {tr['name']} -> test {te['name']}: {summary(score(te, hand, clicks(te, hand, models[tr['name']], N, th, hold)))[3]}")
