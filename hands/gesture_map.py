#!/usr/bin/env python3
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
"""Map the benchmark's labelled hand actions to Hand Landmarker's output: which gestures are clearly told apart?

Input: MERCURY_DETECTION_LOG "E" lines (hand_sim with MERCURY_MP_BOTH_VIEWS=1: per frame, hand, camera: world landmarks,
crop landmarks, gesture embedding), the gesture labels, hand_sim's REPLAY_TO_SESSION_NS. Each gesture was prompted in
two rounds per hand: fit on round 1, test on round 2 (and the other way), so a score is on prompts the fit never saw.
Two readers: (a) a softmax classifier on the gesture embedding of both cameras (Model Maker's custom-gesture idea);
(b) the same on simple joint measures (finger bends, thumb-to-fingertip distances / palm width).
usage: gesture_map.py detection.log gesture-labels.csv replay_to_session_ns"""
import csv, sys, bisect, collections
import numpy as np
log, lab, off = sys.argv[1], sys.argv[2], int(sys.argv[3])
CLASSES = ['open', 'pinch', 'fist', 'point', 'claw']

# Prompts: gesture prompts per hand with a round number (1st or 2nd time that gesture/orientation was asked).
prompts = []
seen = collections.Counter()
for r in csv.DictReader(open(lab)):
    if r['hand'] not in ('left', 'right') or r['gesture'] not in CLASSES:
        continue
    key = (r['hand'], r['gesture'], r['orientation'])
    seen[key] += 1
    rnd = 1 if r['gesture'] != 'open' else 0
    prompts.append(dict(hand=0 if r['hand'] == 'left' else 1, g=r['gesture'], o=r['orientation'],
                        a=int(r['start_ns']), b=int(r['end_ns']), rnd=seen[key] if r['gesture'] != 'open' else None))
# "open" rests: give them the round of the gesture prompt just before (so open is split the same way).
last = {0: 1, 1: 1}
for p in prompts:
    if p['g'] != 'open':
        last[p['hand']] = p['rnd']
    else:
        p['rnd'] = last[p['hand']]

def joints_feats(w):
    P = w.reshape(21, 3)
    palm = np.linalg.norm(P[5] - P[17]) + 1e-6
    f = []
    for tip in (8, 12, 16, 20):
        f.append(np.linalg.norm(P[4] - P[tip]) / palm)       # thumb tip to each fingertip
        f.append(np.linalg.norm(P[tip] - P[0]) / palm)       # fingertip to wrist (curl)
    for a, b, c in ((5, 6, 7), (6, 7, 8), (9, 10, 11), (10, 11, 12), (13, 14, 15), (14, 15, 16), (17, 18, 19), (18, 19, 20), (1, 2, 3), (2, 3, 4)):
        u, v = P[a] - P[b], P[c] - P[b]
        f.append(np.dot(u, v) / (np.linalg.norm(u) * np.linalg.norm(v) + 1e-9))  # bend at each finger joint
    return np.array(f)

# Frames: (session ns, hand) -> {view: (embedding, joint features, presence)}
F = collections.defaultdict(dict)
USED = {}
for line in open(log):
    if line.startswith('G,'):
        g = line.strip().split(',')
        USED[(int(g[1]) + off, int(g[2]))] = int(g[9])
    if not line.startswith('E,'):
        continue
    v = line.rstrip('\n').split(',')
    ts, h, view, pres = int(v[1]) + off, int(v[2]), int(v[3]), float(v[4])
    if pres < 0.5:
        continue
    nums = np.array(v[6:], dtype=np.float32)
    world, emb = nums[:63], nums[126:254]
    F[(ts, h)][view] = (emb, joints_feats(world))
ONE = __import__('os').environ.get('ONE_VIEW') == '1'
if ONE:  # only the camera the driver read for this hand's gestures (as it runs live: once per hand)
    for k in list(F):
        u = USED.get(k)
        F[k] = {0: F[k][u]} if u in F[k] else {}
        if not F[k]:
            del F[k]
keys = sorted(F)
tsl = [k[0] for k in keys]

def frames(p):
    i, j = bisect.bisect_left(tsl, p['a']), bisect.bisect_right(tsl, p['b'])
    return [F[keys[k]] for k in range(i, j) if keys[k][1] == p['hand']]

def vec(fr, kind):
    if kind == 'both':
        return np.concatenate([vec(fr, 'emb'), vec(fr, 'joint')])
    idx = 0 if kind == 'emb' else 1
    z = np.zeros_like(next(iter(fr.values()))[idx])
    a = fr[0][idx] if 0 in fr else z; b = fr[1][idx] if 1 in fr else z
    return np.concatenate([a, b, [0 in fr, 1 in fr]])

def fit(X, y, k, l2=1e-2, it=600, lr=0.5):
    mu, sd = X.mean(0), X.std(0) + 1e-6
    Xn = (X - mu) / sd
    W = np.zeros((X.shape[1], k)); bb = np.zeros(k)
    Y = np.eye(k)[y]
    cw = len(y) / (k * np.maximum(np.bincount(y, minlength=k), 1))  # balance the classes (open has most frames)
    sw = cw[y][:, None]
    for _ in range(it):
        Z = Xn @ W + bb; Z -= Z.max(1, keepdims=True); Pz = np.exp(Z); Pz /= Pz.sum(1, keepdims=True)
        G = (Pz - Y) * sw / len(y)
        W -= lr * (Xn.T @ G + l2 * W); bb -= lr * G.sum(0)
    return lambda Xt: ((Xt - mu) / sd) @ W + bb

SMOOTH = int(__import__('os').environ.get('SMOOTH', '1'))
KINDS = __import__('os').environ.get('KINDS', 'emb,joint').split(',')
NAMES = {'emb': 'gesture embedding, both cameras', 'joint': 'joint measures, both cameras', 'both': 'embedding + joints'}
for kind, name in ((k, NAMES[k]) for k in KINDS):
    print(f"\n### reader: {name}")
    for hand in (0, 1):
        res = collections.defaultdict(lambda: [0, 0, 0, 0])  # prompts, prompts right, frames, frames right
        conf = collections.Counter()
        for test_rnd in (1, 2):
            tr = [p for p in prompts if p['hand'] == hand and p['rnd'] != test_rnd]
            te = [p for p in prompts if p['hand'] == hand and p['rnd'] == test_rnd]
            X, y = [], []
            for p in tr:
                for fr in frames(p):
                    X.append(vec(fr, kind)); y.append(CLASSES.index(p['g']))
            if not X:
                continue
            f = fit(np.array(X), np.array(y), len(CLASSES))
            for p in te:
                fr = frames(p)
                if not fr:
                    continue
                sc = f(np.array([vec(x, kind) for x in fr]))
                if SMOOTH > 1:  # the mean score over the last SMOOTH frames (causal, as live)
                    cs = np.cumsum(np.vstack([np.zeros((1, sc.shape[1])), sc]), 0)
                    sc = np.array([(cs[i + 1] - cs[max(0, i + 1 - SMOOTH)]) / (i + 1 - max(0, i + 1 - SMOOTH)) for i in range(len(sc))])
                pred = sc.argmax(1)
                c = CLASSES.index(p['g'])
                vote = np.bincount(pred, minlength=len(CLASSES)).argmax()
                k = res[p['g']]; k[0] += 1; k[1] += vote == c; k[2] += len(pred); k[3] += int((pred == c).sum())
                conf[(p['g'], CLASSES[vote])] += 1
        print(f"  {'left' if hand == 0 else 'right'} hand (tested on the round not fitted):")
        for g in CLASSES:
            k = res[g]
            if k[0]:
                wrong = ', '.join(f"{CLASSES[i]} {conf[(g, CLASSES[i])]}" for i in range(len(CLASSES)) if CLASSES[i] != g and conf[(g, CLASSES[i])])
                print(f"    {g:6s} prompts {k[1]}/{k[0]} right ({100*k[1]/k[0]:.0f}%), frames {100*k[3]/max(k[2],1):.0f}% right" + (f"; read as: {wrong}" if wrong else ''))
