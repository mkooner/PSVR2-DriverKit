#!/usr/bin/env python3
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
"""Does the simulator behave like a live session? The same statistics from a live session's logs and from hand_sim runs.

usage: live_vs_sim.py live_driver.log live_vrserver.txt live_vrhandlog.csv  sim1.log sim1.csv [sim2.log sim2.csv ...]
Statistics (they don't depend on what the hands did, so different sessions can be compared):
  tracker results per second while a hand is tracked (5 s windows >= 90% tracked),
  MediaPipe gesture readings valid (mean and lowest 10 s window),
  trigger and grip presses: share held under 0.15 s, share re-pressed within 0.15 s of letting go (chatter)."""
import csv, re, sys

def rate_and_mp(text):
    out = {}
    for h in ('Left', 'Right'):
        m = re.findall(h + r' hand \(5 s, (\d+) frames\): tracked (\d+)%', text)
        busy = [int(f) for f, k in m if int(k) >= 90]
        mp = [int(x) for x in re.findall(h.lower() + r' hand MediaPipe gestures \(10 s\): valid (\d+)% of tracked', text)]
        out[h] = (sum(busy) / len(busy) / 5 if busy else float('nan'), sum(mp) / len(mp) if mp else float('nan'),
                  min(mp) if mp else float('nan'))
    return out

def chatter_from_events(path):
    ev = list(csv.DictReader(open(path)))
    out = {}
    for dev, h in (('1', 'Left'), ('2', 'Right')):
        res = []
        for btn in ('33', '2'):
            down = last_up = None; durs = []; gaps = []
            for e in ev:
                if e['device'] != dev or e['button'] != btn: continue
                t = float(e['t_s'])
                if e['name'] == 'VREvent_ButtonPress':
                    if last_up is not None: gaps.append(t - last_up)
                    down = t
                elif e['name'] == 'VREvent_ButtonUnpress' and down is not None:
                    durs.append(t - down); down = None; last_up = t
            res.append((len(durs), sum(d < 0.15 for d in durs), sum(g < 0.15 for g in gaps)))
        out[h] = res
    return out

def chatter_from_sim(path):
    rows = list(csv.DictReader(open(path)))
    out = {}
    for h in ('Left', 'Right'):
        res = []
        for col in ('_trigger_click', '_grip_click'):
            c = h.lower() + col; down = last_up = None; prev = False; durs = []; gaps = []
            for r in rows:
                t = float(r['t_s']); v = r[c] == '1'
                if v and not prev:
                    if last_up is not None: gaps.append(t - last_up)
                    down = t
                if prev and not v and down is not None:
                    durs.append(t - down); last_up = t
                prev = v
            res.append((len(durs), sum(d < 0.15 for d in durs), sum(g < 0.15 for g in gaps)))
        out[h] = res
    return out

def show(name, rm, ch):
    for h in ('Left', 'Right'):
        r, mpm, mpl = rm[h]; (tn, ts, tg), (gn, gs, gg) = ch[h]
        pct = lambda a, b: f"{a / b:.0%}" if b else "-"
        print(f"  {name:22s} {h:5s}: tracker {r:5.1f}/s; MediaPipe valid mean {mpm:4.0f}% (lowest {mpl:3.0f}%); "
              f"trigger {tn:3d} presses, short {pct(ts, tn):>4s}, re-pressed {pct(tg, tn):>4s}; "
              f"grip {gn:3d}, short {pct(gs, gn):>4s}, re-pressed {pct(gg, gn):>4s}")

live_log, live_vrs, live_hl = sys.argv[1:4]
text = open(live_log, errors='replace').read() + open(live_vrs, errors='replace').read()
show('LIVE', rate_and_mp(text), chatter_from_events(live_hl + '.events.csv'))
args = sys.argv[4:]
for i in range(0, len(args), 2):
    show(args[i].split('/')[-1].replace('.log', ''), rate_and_mp(open(args[i], errors='replace').read()),
         chatter_from_sim(args[i + 1]))
