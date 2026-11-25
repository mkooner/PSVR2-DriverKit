#!/usr/bin/env python3
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
"""Dashboard gesture candidates on hand_sim's per-frame output (the plugin's own palm pose, MediaPipe readings) with
the session's head poses (vrhandlog), scored against the spoken prompts.
usage: dash_candidates.py sim.csv vrhandlog.csv labels.csv"""
import csv, math, sys
sim, log, lab = sys.argv[1:4]
t0 = next(int(l.split('t0_mono_ns')[1]) for l in open(log) if 't0_mono_ns' in l)
r = csv.reader(l for l in open(log) if not l.startswith('#')); h = next(r); ix = {k: i for i, k in enumerate(h)}
head = {x[ix['t_s']]: [float(x[ix['hmd_p' + k]]) for k in 'xyz'] for x in r if len(x) == len(h) and x[ix['hmd_valid']] == '1'}
L = list(csv.DictReader(open(lab)))
def rot(q, v):
    x, y, z, w = q; cx = y*v[2]-z*v[1]; cy = z*v[0]-x*v[2]; cz = x*v[1]-y*v[0]; tx, ty, tz = 2*cx, 2*cy, 2*cz
    return [v[0]+w*tx+(y*tz-z*ty), v[1]+w*ty+(z*tx-x*tz), v[2]+w*tz+(x*ty-y*tx)]
frames = {'left': [], 'right': []}
for row in csv.DictReader(open(sim)):
    hp = head.get(row['t_s'])
    for s in frames:
        if row[s + '_valid'] != '1' or hp is None:
            frames[s].append((float(row['t_s']), None)); continue
        p = [float(row[s + '_p' + k]) for k in 'xyz']; q = [float(row[s + '_q' + k]) for k in 'xyzw']
        n = rot(q, [0, -1, 0]); d = [hp[i] - p[i] for i in range(3)]; dl = math.sqrt(sum(c*c for c in d)) or 1
        facing = sum(n[i] * d[i] / dl for i in range(3)); height = p[1] - hp[1]
        opn = row[s + '_mp_valid'] == '1' and row[s + '_mp_category'] == '2' and float(row[s + '_mp_score']) >= 0.5
        frames[s].append((float(row['t_s']), (facing, height, opn)))
def where(t, side):
    m = t0 + t * 1e9
    for p in L:
        if p['hand'] == side and int(p['start_ns']) <= m <= int(p['end_ns']) + 1e9:
            return p['gesture'] + ('/' + p['orientation'] if p['gesture'] == 'system' else '')
    return 'free/between'
def run(fmin, hmin, hold):
    acts = []
    for s, F in frames.items():
        since = last = None; fired = False; oc = 0; opn = False
        for t, v in F:
            o = v is not None and v[2]
            oc = oc + 1 if o != opn else 0
            if oc >= 2: opn = not opn; oc = 0
            ok = v is not None and opn and v[0] > fmin and v[1] > hmin
            if ok:
                if since is None or t - last > 0.15: since = t; fired = False
                last = t
                if not fired and t - since >= hold: fired = True; acts.append((t, s))
            elif last is not None and t - last > 0.15:
                since = None; fired = False
    hits = set(); other = []
    for t, s in acts:
        w = where(t, s)
        if w.startswith('system'): hits.add((s, w))
        else: other.append(w)
    return len(hits), other
print("dashboard prompts caught (of 4) / presses anywhere else (which prompts)")
for fmin in (0.5, 0.65, 0.8):
    for hmin in (-0.15, -0.20, -0.25, -0.30, -0.35):
        for hold in (0.8, 1.0, 1.5):
            n, other = run(fmin, hmin, hold)
            print(f"  facing>{fmin} height>{hmin:+.2f} hold {hold}s: {n}/4, else {len(other)} {sorted(set(other))}")
