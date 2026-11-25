#!/usr/bin/env python3
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
"""Score SteamVR hand inputs against the spoken prompts (gesture-labels CSV).

usage: score_prompts.py labels.csv vrhandlog.csv [sim.csv ...]
For the real session (vrhandlog buttons) and each simulation (hand_sim output), per hand: pinch prompts with the
trigger pressed during the prompt, fist prompts with the grip pressed, and false presses (a new trigger press on open/fist/point/
claw prompts, a new grip press on open/pinch/point/claw prompts). A prompt counts from its start to 1 s after its end."""
import csv, sys

labels, log = sys.argv[1], sys.argv[2]
sims = sys.argv[3:]
t0 = 0
for l in open(log):
    if 't0_mono_ns' in l:
        t0 = int(l.split('t0_mono_ns')[1].split()[0])
        break
L = [r for r in csv.DictReader(open(labels))]

def series_real():
    r = csv.reader(l for l in open(log) if not l.startswith('#')); h = next(r); ix = {k: i for i, k in enumerate(h)}
    out = {'left': [], 'right': []}
    for x in r:
        if len(x) != len(h): continue
        t = t0 + int(float(x[ix['t_s']]) * 1e9)
        for s in out:
            b = int(x[ix[s + '_buttons']] or 0)
            out[s].append((t, (b >> 33) & 1, (b >> 2) & 1, 0, x[ix[s + '_valid']] == '1',
                           tuple(float(x[ix[s + '_p' + k]]) for k in 'xyz')))
    # The real session's dashboard toggles (SteamVR events) stand in for the system presses.
    try:
        for e in csv.DictReader(open(log + '.events.csv')):
            if e['name'] in ('VREvent_DashboardActivated', 'VREvent_DashboardDeactivated'):
                t = t0 + int(float(e['t_s']) * 1e9)
                out['dash'] = out.get('dash', []) + [t]
    except OSError:
        pass
    return out

def series_sim(p):
    out = {'left': [], 'right': []}
    for x in csv.DictReader(open(p)):
        t = t0 + int(float(x['t_s']) * 1e9)
        for s in out:
            out[s].append((t, x[s + '_trigger_click'] == '1', x[s + '_grip_click'] == '1',
                           x[s + '_system_click'] == '1', x[s + '_valid'] == '1',
                           tuple(float(x[s + '_p' + k]) for k in 'xyz')))
    # System presses (either hand) as dashboard toggles.
    for side in ('left', 'right'):
        prev = False
        for v in out[side]:
            if v[3] and not prev:
                out['dash'] = out.get('dash', []) + [v[0]]
            prev = v[3]
    return out

def score(name, S):
    res = []
    for side in ('left', 'right'):
        pin = fist = pin_n = fist_n = ft = ft_n = fg = fg_n = 0
        for r in L:
            if r['hand'] != side: continue
            a, b = int(r['start_ns']), int(r['end_ns']) + 1_000_000_000
            seg = [v for v in S[side] if a <= v[0] <= b]
            trig = any(v[1] for v in seg); grip = any(v[2] for v in seg)
            # False presses: a new press (off -> on) inside the prompt, not one held over from the previous prompt.
            prev = [v for v in S[side] if v[0] < a][-1:] or [(0, 0, 0)]
            seq = prev + seg
            new_trig = any(q[1] and not p[1] for p, q in zip(seq, seq[1:]))
            new_grip = any(q[2] and not p[2] for p, q in zip(seq, seq[1:]))
            g = r['gesture']
            if g == 'pinch': pin_n += 1; pin += trig
            if g == 'fist': fist_n += 1; fist += grip
            if g in ('open', 'fist', 'point', 'claw'): ft_n += 1; ft += new_trig
            if g in ('open', 'pinch', 'point', 'claw'): fg_n += 1; fg += new_grip
        tracked = sum(v[4] for v in S[side]) / max(len(S[side]), 1)
        steps = sorted(((a[5][0]-b[5][0])**2 + (a[5][1]-b[5][1])**2 + (a[5][2]-b[5][2])**2) ** 0.5
                       for a, b in zip(S[side][1:], S[side]) if a[4] and b[4])
        shake = steps[len(steps) // 2] * 1000 if steps else 0
        res.append(f"{side}: pinch {pin}/{pin_n}, fist {fist}/{fist_n}, false trigger {ft}/{ft_n}, false grip {fg}/{fg_n}, "
                   f"tracked {tracked:.0%}, per-frame move median {shake:.1f} mm")
    dash = S.get('dash', [])
    dp = [r for r in L if r['gesture'] == 'system']
    hit = sum(any(int(r['start_ns']) <= t <= int(r['end_ns']) + 1_000_000_000 for t in dash) for r in dp)
    outside = sum(not any(int(r['start_ns']) <= t <= int(r['end_ns']) + 1_000_000_000 for r in dp) for t in dash)
    res.append(f"dashboard prompts {hit}/{len(dp)}, toggles outside them {outside}")
    print(f"{name:28s} " + " | ".join(res))

score('real session (full model)', series_real())
for p in sims:
    score(p.split('/')[-1], series_sim(p))
