#!/usr/bin/env python3
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
"""Compare hand_sim's prediction with what SteamVR really got in the same session (vrhandlog), frame by frame.

usage: compare_sim.py sim.csv vrhandlog.csv
Reports per hand: tracked agreement, position error, and each trigger / grip press (actual vs predicted, matched
within 0.3 s), and dashboard toggles (actual from the events file vs predicted system presses)."""
import csv, math, sys, os

sim_path, log_path = sys.argv[1], sys.argv[2]
def rows(path):
    r = csv.reader(l for l in open(path) if not l.startswith('#'))
    h = next(r)
    return h, [x for x in r if len(x) == len(h)]

sh, srows = rows(sim_path); si = {k: i for i, k in enumerate(sh)}
lh, lrows = rows(log_path); li = {k: i for i, k in enumerate(lh)}
actual = {round(float(x[li['t_s']]), 6): x for x in lrows}

def edges(ts, vals):
    out, prev = [], 0
    for t, v in zip(ts, vals):
        if v and not prev: out.append(t)
        prev = v
    return out

def match(a, p, tol=0.3):
    used, hit = set(), 0
    for t in a:
        best = min((abs(t - q), j) for j, q in enumerate(p) if j not in used) if len(used) < len(p) else None
        if best and best[0] <= tol:
            used.add(best[1]); hit += 1
    return hit

ts = []; res = {}
for side in ('left', 'right'):
    both = sim_only = act_only = neither = 0; err = []
    a_trig, p_trig, a_grip, p_grip, p_sys = [], [], [], [], []
    for x in srows:
        t = round(float(x[si['t_s']]), 6)
        y = actual.get(t)
        if y is None: continue
        sv = x[si[side + '_valid']] == '1'; av = y[li[side + '_valid']] == '1'
        both += sv and av; sim_only += sv and not av; act_only += av and not sv; neither += not sv and not av
        if sv and av:
            sp = [float(x[si[side + '_p' + k]]) for k in 'xyz']; ap = [float(y[li[side + '_p' + k]]) for k in 'xyz']
            err.append(math.dist(sp, ap))
        b = int(y[li[side + '_buttons']] or 0)
        a_trig.append((t, (b >> 33) & 1)); a_grip.append((t, (b >> 2) & 1))
        p_trig.append((t, x[si[side + '_trigger_click']] == '1')); p_grip.append((t, x[si[side + '_grip_click']] == '1'))
        p_sys.append((t, x[si[side + '_system_click']] == '1'))
    n = both + sim_only + act_only + neither
    E = lambda s: edges([a for a, _ in s], [b for _, b in s])
    at, pt, ag, pg = E(a_trig), E(p_trig), E(a_grip), E(p_grip)
    err.sort()
    print(f"{side}: {n} frames. tracked: both {both/n:.0%}, only real {act_only/n:.0%}, only simulated {sim_only/n:.0%}, "
          f"neither {neither/n:.0%} -> agreement {(both+neither)/n:.0%}")
    if err:
        print(f"   position, real vs simulated: median {err[len(err)//2]*100:.1f} cm, 90% {err[int(len(err)*.9)]*100:.1f} cm")
    print(f"   trigger presses: real {len(at)}, simulated {len(pt)}, matched {match(at, pt)}; real at {[round(t,1) for t in at]}, simulated at {[round(t,1) for t in pt]}")
    print(f"   grip presses:    real {len(ag)}, simulated {len(pg)}, matched {match(ag, pg)}; real at {[round(t,1) for t in ag]}, simulated at {[round(t,1) for t in pg]}")
    res[side] = E(p_sys)
ev = log_path + '.events.csv'
if os.path.exists(ev):
    dash = [float(r['t_s']) for r in csv.DictReader(open(ev)) if r['name'] in ('VREvent_DashboardActivated', 'VREvent_DashboardDeactivated')]
    sysp = sorted(res['left'] + res['right'])
    print(f"dashboard toggles: real {len(dash)} at {[round(t,1) for t in dash]}; simulated system presses {len(sysp)} at {[round(t,1) for t in sysp]}; matched {match(dash, sysp, 0.5)}")
