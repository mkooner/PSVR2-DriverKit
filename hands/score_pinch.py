#!/usr/bin/env python3
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
"""Score a pinch focus session (GESTURE_PROMPTS_SET=pinch): held, slow, fast (taps, bursts of three) pinches and
open hands that must not click. Clicks from a vrhandlog (live) or hand_sim output (replay).
usage: score_pinch.py gesture-labels.csv vrhandlog.csv [hand_sim.csv ...]"""
import csv, sys, collections

def load(path, t0):
    out = {'left': [], 'right': []}
    rows = csv.DictReader(l for l in open(path) if not l.startswith('#'))
    for r in rows:
        t = t0 + float(r['t_s']) * 1e9
        for s in out:
            if s + '_buttons' in r:
                on = (int(r[s + '_buttons'] or 0) >> 33) & 1 == 1
            else:
                on = r[s + '_trigger_click'] == '1'
            out[s].append((t, on))
    return out

labels, log = sys.argv[1], sys.argv[2]
t0 = next(int(l.split('t0_mono_ns')[1]) for l in open(log) if 't0_mono_ns' in l)
P = [r for r in csv.DictReader(open(labels)) if r['hand'] in ('left', 'right')]
for path in sys.argv[2:]:
    C = load(path, t0)
    print(f"== {path.split('/')[-1]}")
    for hand in ('left', 'right'):
        S = C[hand]
        def seg(a, b): return [on for t, on in S if a <= t <= b]
        def presses(a, b):
            v = seg(a, b); return sum(1 for i in range(1, len(v)) if v[i] and not v[i - 1]) + (1 if v and v[0] else 0)
        res = collections.defaultdict(list)
        hp = [p for p in P if p['hand'] == hand]
        for i, p in enumerate(hp):
            a, b, g, o = int(p['start_ns']), int(p['end_ns']), p['gesture'], p['orientation']
            v = seg(a, b)
            if not v:
                continue
            if g == 'pinch' and o not in ('tap', 'slow_hold'):
                res['held: caught'].append(any(v)); res['held: on share'].append(sum(v) / len(v))
            elif g == 'pinch' and o == 'tap':
                # a tap counts if the click comes during the tap or within 0.35 s after it
                res['fast taps: clicked'].append(any(seg(a, b + int(0.35e9))))
            elif g == 'burst':
                res['fast bursts of 3: clicks'].append(presses(a, b))
            elif g == 'pinch' and o == 'slow_hold':
                prev = hp[i - 1] if i > 0 else None
                start = int(prev['start_ns']) if prev and prev['orientation'] == 'slow_close' else a
                close_end = int(prev['end_ns']) if prev and prev['orientation'] == 'slow_close' else a
                ons = [t for t, on in S if start <= t <= b and on]
                res['slow: caught'].append(bool(ons))
                if ons:
                    res['slow: click vs fingers touching (s)'].append((ons[0] - close_end) / 1e9)
            elif g == 'open' and o not in ('tap_release',):
                res['open: clicked by mistake'].append(any(v))
            elif g == 'open' and o == 'tap_release':
                res['tap release: let go'].append(not all(v))
        print(f"  {hand}:")
        for k in ('held: caught', 'held: on share', 'slow: caught', 'slow: click vs fingers touching (s)', 'fast taps: clicked',
                  'fast bursts of 3: clicks', 'tap release: let go', 'open: clicked by mistake'):
            v = res.get(k, [])
            if not v:
                continue
            if k in ('held: on share',):
                print(f"    {k:38s} mean {100*sum(v)/len(v):.0f}% of the hold")
            elif k.startswith('slow: click'):
                print(f"    {k:38s} " + ', '.join(f"{x:+.2f}" for x in v))
            elif k.startswith('fast bursts'):
                print(f"    {k:38s} " + ', '.join(str(x) for x in v) + " (3 each wanted)")
            else:
                print(f"    {k:38s} {sum(v)}/{len(v)}")
