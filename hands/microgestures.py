#!/usr/bin/env python3
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
"""Thumb tap and thumb swipe state machines (the same rules as the plugin's ovrn_microgestures.hpp), replayed on the
logged landmarks of labelled recordings to count false events (the recordings have no taps or swipes: every event is
false) and how often each candidate rule fires during pinches, fists and open hands.
usage: microgestures.py [--sweep]"""
import sys, collections, itertools
import numpy as np
sys.path.insert(0, __import__('os').path.dirname(__import__('os').path.abspath(__file__)))
import gesture_fit as gf

F = gf.FEATURES
DEFAULT = dict(alpha=0.5, tap_contact=0.25, tap_release=0.45, tap_min_ms=50, tap_max_ms=300, tap_arm_ms=150,
               tap_cooldown_ms=250, double_ms=400, swipe_region=0.45, swipe_min=0.30, swipe_min_ms=100,
               swipe_max_ms=500, swipe_ratio=2.0, swipe_cooldown_ms=400, grace_ms=100)

class Micro:
    def __init__(self, p):
        self.p = p; self.reset()
    def reset(self):
        self.s = None; self.tap_state = 'idle'; self.armed_since = None; self.contact_t = None; self.cool_until = 0
        self.last_tap = None; self.sw_start = None; self.sw_cool_until = 0; self.sw_done = False; self.last = None
    def update(self, t, f, valid, blocked):
        p = self.p; ev = []
        if self.last is not None and t - self.last > p['grace_ms'] * 1e6:
            self.reset()
        if not valid or blocked:
            self.reset(); self.last = t if valid else self.last; return ev
        self.last = t
        x = np.array([f[F.index('thumb_index_side')], f[F.index('thumb_local_x')], f[F.index('thumb_local_y')]])
        self.s = x if self.s is None else p['alpha'] * x + (1 - p['alpha']) * self.s
        side, lx, ly = self.s
        # tap: armed after the thumb stays clear of the index finger; a quick dip in and out is a tap
        if self.tap_state == 'idle':
            if side > p['tap_release']:
                self.armed_since = self.armed_since or t
                if t - self.armed_since >= p['tap_arm_ms'] * 1e6 and t >= self.cool_until:
                    self.tap_state = 'armed'
            else:
                self.armed_since = None
        elif self.tap_state == 'armed':
            if side < p['tap_contact']:
                self.tap_state = 'contact'; self.contact_t = t
        elif self.tap_state == 'contact':
            dt = (t - self.contact_t) / 1e6
            if side > p['tap_release']:
                if p['tap_min_ms'] <= dt <= p['tap_max_ms']:
                    ev.append('tap')
                    if self.last_tap is not None and (t - self.last_tap) / 1e6 <= p['double_ms']:
                        ev.append('double_tap'); self.last_tap = None
                    else:
                        self.last_tap = t
                    self.cool_until = t + p['tap_cooldown_ms'] * 1e6
                self.tap_state = 'idle'; self.armed_since = None
            elif dt > p['tap_max_ms']:
                self.tap_state = 'held'
        elif self.tap_state == 'held':
            if side > p['tap_release']:
                self.tap_state = 'idle'; self.armed_since = None
        # swipe: inside the region on the index finger, a quick move along or across it
        if side < p['swipe_region'] and t >= self.sw_cool_until:
            if self.sw_start is None:
                self.sw_start = (t, lx, ly); self.sw_done = False
            elif not self.sw_done:
                t0, x0, y0 = self.sw_start; dt = (t - t0) / 1e6; dx, dy = lx - x0, ly - y0
                if dt > p['swipe_max_ms']:
                    self.sw_start = (t, lx, ly)
                elif dt >= p['swipe_min_ms'] and max(abs(dx), abs(dy)) >= p['swipe_min']:
                    big, small = (abs(dx), abs(dy)) if abs(dx) >= abs(dy) else (abs(dy), abs(dx))
                    if big >= p['swipe_ratio'] * small:
                        ev.append(('swipe_forward' if dx > 0 else 'swipe_back') if abs(dx) >= abs(dy) else
                                  ('swipe_thumbward' if dy > 0 else 'swipe_away'))
                        self.sw_done = True; self.sw_cool_until = t + p['swipe_cooldown_ms'] * 1e6
        else:
            self.sw_start = None
        return ev

def run(p, verbose=True):
    offs = dict(l.split() for l in open('../logs/landmarks/replay_to_session_ns.txt'))
    tot = collections.Counter(); minutes = 0.0
    out = []
    for st in sorted(offs):
        s = gf.load_session('../logs', '20261125-' + st, int(offs[st]))
        for h in (0, 1):
            T = s['frames'][h]
            if not T: continue
            # "blocked": a fist or pinch shape (the formula pinch / fist rules from the built-in config)
            m = Micro(p); c = collections.Counter(); where = collections.Counter()
            ts = [t for t, _, _ in T]
            labels = [None] * len(T)
            import bisect
            for pr in s['prompts']:
                if pr['hand'] != gf.HANDS[h]: continue
                i, j = bisect.bisect_left(ts, int(pr['start_ns'])), bisect.bisect_right(ts, int(pr['end_ns']))
                for k in range(i, j): labels[k] = gf.state_of(pr) or pr['gesture']
            on = (0.558 if h == 0 else 0.500); fist_reach = (1.0752 if h == 0 else None)
            for k, (t, f, pres) in enumerate(T):
                blocked = f[F.index('thumb_index')] <= on or (h == 0 and f[F.index('index_reach')] <= 1.0752) or \
                          (h == 1 and f[F.index('little_reach')] <= 0.8964)
                for e in m.update(t, f, pres >= 0.5, blocked):
                    c[e] += 1; where[(e, labels[k])] += 1
            dur = (T[-1][0] - T[0][0]) / 60e9; minutes += dur
            tot.update(c)
            out.append((st, gf.HANDS[h], dur, c, where))
    if verbose:
        for st, hs, dur, c, where in out:
            print(f"  {st} {hs:5s} ({dur:.1f} min): " + (', '.join(f"{k} {v} ({v / dur:.1f}/min)" for k, v in sorted(c.items())) or 'no events'))
            if where:
                print("       during: " + ', '.join(f"{e}@{l} {n}" for (e, l), n in where.most_common(6)))
    return tot, minutes

if __name__ == '__main__':
    tot, minutes = run(DEFAULT)
    print(f"false events over {minutes:.1f} min of both hands: " + ', '.join(f"{k} {v} ({v / minutes:.2f}/min)" for k, v in sorted(tot.items())))
