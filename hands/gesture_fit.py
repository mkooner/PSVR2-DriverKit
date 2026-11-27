#!/usr/bin/env python3
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
"""Fit finger gesture rules on Hand Landmarker's joints from labelled recordings, validate them on unseen sessions (or
the unseen round of prompts), replay them through the driver's state machine, and write the driver's gesture config.

Data (all already extracted; nothing is re-run and the recordings aren't touched):
  logs/landmarks/landmarks-<stamp>.log   "E" lines from hand_sim (MERCURY_DETECTION_LOG): frame time, hand, camera,
                                         MediaPipe presence, right-hand score, 21 world landmarks, ...
  logs/landmarks/replay_to_session_ns.txt  clock offset per recording
  logs/gesture-labels-<stamp>-retimed.csv  prompts, re-timed to when the hand changed (hands/retime_labels.py)

Features: the driver's gesture::jointFeatures (hg_gesture_geometry.hpp), from the camera with the higher presence.
Engine: a copy of ovrn_gesture_rules.hpp's state machine (idle / candidate / active / releasing, hysteresis, hold
times, grace on tracking loss, exclusive with priority, feature smoothing).

usage: gesture_fit.py [--logs DIR] [--write gestures.json] [--header ovrn_gesture_rules_default.hpp]
       gesture_fit.py --replay STAMP --config gestures.json [--hand left|right] [--export out.csv]
         (the debugging replay: every frame's features, each gesture's phase and why it changed)"""
import argparse, bisect, collections, csv, json, math, os, sys
import numpy as np

FEATURES = ['index_pip', 'index_dip', 'middle_pip', 'middle_dip', 'ring_pip', 'ring_dip', 'little_pip', 'little_dip',
            'index_reach', 'middle_reach', 'ring_reach', 'little_reach',
            'thumb_index', 'thumb_middle', 'thumb_ring', 'thumb_little', 'thumb_ip', 'thumb_out',
            'thumb_index_side', 'thumb_local_x', 'thumb_local_y', 'thumb_local_z']
HANDS = ('left', 'right')

# ---------------------------------------------------------------------------------------------------- features / data

def angle(a, b, c):
    u, v = a - b, c - b
    lu, lv = np.linalg.norm(u), np.linalg.norm(v)
    if lu < 1e-6 or lv < 1e-6:
        return math.nan
    return math.degrees(math.acos(max(-1.0, min(1.0, float(u @ v / (lu * lv))))))

def features(w):
    """gesture::jointFeatures of one camera's 21 world landmarks (63 floats), or None."""
    P = w.reshape(21, 3).astype(np.float64)
    palm = np.linalg.norm(P[5] - P[17])
    if not palm > 1e-6:
        return None
    pip, dip, reach, thumb = [], [], [], []
    for mcp in (5, 9, 13, 17):
        pip.append(angle(P[mcp], P[mcp + 1], P[mcp + 2])); dip.append(angle(P[mcp + 1], P[mcp + 2], P[mcp + 3]))
        base = np.linalg.norm(P[mcp] - P[0])
        reach.append(np.linalg.norm(P[mcp + 3] - P[0]) / base if base > 1e-6 else math.nan)
        thumb.append(np.linalg.norm(P[4] - P[mcp + 3]) / palm)
    f = [x for pd in zip(pip, dip) for x in pd] + reach + thumb + [angle(P[2], P[3], P[4]), np.linalg.norm(P[4] - P[5]) / palm]
    # thumb tip to the index's PIP-DIP segment; the thumb tip in the index finger's frame (gesture::indexFrame)
    seg = P[7] - P[6]; l2 = seg @ seg
    u = 0.0 if l2 < 1e-12 else min(1.0, max(0.0, float((P[4] - P[6]) @ seg / l2)))
    f.append(np.linalg.norm(P[4] - (P[6] + u * seg)) / palm)
    ax = P[8] - P[5]; n = np.linalg.norm(ax)
    if n < 1e-6:
        return None
    ax = ax / n
    across = P[5] - P[17]; ay = across - ax * (across @ ax); n = np.linalg.norm(ay)
    if n < 1e-6:
        return None
    ay = ay / n; az = np.cross(ax, ay); rel = P[4] - P[5]
    f += [rel @ ax / palm, rel @ ay / palm, rel @ az / palm]
    f = np.array(f)
    return f if np.all(np.isfinite(f)) else None

def load_session(logs, stamp, off):
    by = collections.defaultdict(dict)
    for line in open(os.path.join(logs, 'landmarks', f'landmarks-{stamp}.log')):
        v = line.rstrip('\n').split(',')
        pres = float(v[4])
        f = features(np.array(v[6:69], dtype=np.float32)) if pres >= 0.5 else None
        if f is not None:
            by[(int(v[1]) + off, int(v[2]))][int(v[3])] = (pres, f)
    frames = {h: [] for h in (0, 1)}
    for (t, h), views in by.items():
        pres, f = max(views.values(), key=lambda x: x[0])
        frames[h].append((t, f, pres))
    for h in frames:
        frames[h].sort(key=lambda x: x[0])
    prompts = [r for r in csv.DictReader(open(os.path.join(logs, f'gesture-labels-{stamp}-retimed.csv')))
               if r['hand'] in HANDS]
    seen = collections.Counter()
    for p in prompts:
        k = (p['hand'], p['gesture'], p['orientation']); seen[k] += 1; p['round'] = seen[k]
    return dict(stamp=stamp, frames=frames, prompts=prompts)

NOT_A_STATE = ('transition', 'burst', 'system')

def state_of(p):
    """The hand state a label shows: its gesture name ("pinch" is the index pinch; "position" prompts are open hands;
    None for transitions, bursts and the dashboard gesture, which aren't one state). A new gesture recorded under a new
    name is picked up as it is."""
    g = p['gesture']
    if g == 'pinch':
        return 'index_pinch'
    if g in ('open', 'position'):
        return 'open'
    return None if g in NOT_A_STATE else g

def labelled(sess, hand, rounds=None):
    """(features, state, prompt id) of the hand's frames inside labelled windows (unsmoothed)."""
    T = sess['frames'][hand]; ts = [t for t, _, _ in T]
    X, S, G = [], [], []
    for pid, p in enumerate(sess['prompts']):
        if p['hand'] != HANDS[hand]:
            continue
        st = state_of(p)
        if st is None or (rounds is not None and p['round'] not in rounds):
            continue
        i, j = bisect.bisect_left(ts, int(p['start_ns'])), bisect.bisect_right(ts, int(p['end_ns']))
        for k in range(i, j):
            X.append(T[k][1]); S.append(st); G.append(pid)
    return np.array(X), np.array(S), np.array(G)

# ------------------------------------------------------------------------------------------------- rules and fitting

def auc(score, y):
    o = np.argsort(score); r = np.empty(len(score)); r[o] = np.arange(1, len(score) + 1)
    n1 = y.sum(); n0 = len(y) - n1
    return (r[y].sum() - n1 * (n1 + 1) / 2) / (n1 * n0) if n1 and n0 else math.nan

def f1_of(pred, y):
    tp = (pred & y).sum(); fp = (pred & ~y).sum(); fn = (~pred & y).sum()
    return 2 * tp / max(2 * tp + fp + fn, 1)

def fit_terms(X, y):
    """The best one- or two-term AND rule (F1 on the fit frames); thresholds "on", "off" a quarter of the way toward
    the median of the other states (hysteresis)."""
    aucs = [auc(X[:, i], y) for i in range(len(FEATURES))]
    top = sorted(range(len(FEATURES)), key=lambda i: -abs(aucs[i] - 0.5))[:6]
    cand = []
    for a in top:
        sa = aucs[a] >= 0.5
        for qa in np.quantile(X[:, a], np.linspace(0.02, 0.98, 49)):
            cand.append([(a, sa, qa)])
    for a in top:
        for b in top:
            if b <= a:
                continue
            sa, sb = aucs[a] >= 0.5, aucs[b] >= 0.5
            for qa in np.quantile(X[:, a], np.linspace(0.05, 0.95, 19)):
                for qb in np.quantile(X[:, b], np.linspace(0.05, 0.95, 19)):
                    cand.append([(a, sa, qa), (b, sb, qb)])
    best, bf = None, -1
    for c in cand:
        pred = np.ones(len(y), bool)
        for fi, above, q in c:
            pred &= (X[:, fi] >= q) if above else (X[:, fi] <= q)
        f = f1_of(pred, y)
        if f > bf:
            best, bf = c, f
    terms = []
    for fi, above, q in best:
        med_other = float(np.median(X[~y, fi]))
        terms.append(dict(feature=FEATURES[fi], op='>=' if above else '<=', on=round(float(q), 4),
                          off=round(float(q + 0.25 * (med_other - q)), 4)))
    return terms, bf

# --------------------------------------------------------------------------------------- the driver's state machine

class Engine:
    """A copy of ovrn::gest::rules::Engine (same transitions)."""
    def __init__(self, gestures, smooth=1, grace_ms=100):
        self.g = gestures; self.smooth = max(1, smooth); self.grace = grace_ms * 1e6
        self.phase = ['idle'] * len(gestures); self.since = [0] * len(gestures)
        self.hist = []; self.last_valid = 0

    def active(self):
        for i, p in enumerate(self.phase):
            if p in ('active', 'releasing'):
                return i
        return -1

    def _holds(self, g, on, f):
        for t in g['terms']:
            x = f[FEATURES.index(t['feature'])]; th = t['on'] if on else t['off']
            if not ((x <= th) if t['op'] == '<=' else (x >= th)):
                return False
        return True

    def update(self, f, valid, conf, t):
        ch = []
        def note(i, to, why):
            if self.phase[i] != to:
                ch.append((i, self.phase[i], to, why))
            self.phase[i] = to; self.since[i] = t
        anyg = any(g['enabled'] and conf >= g.get('min_confidence', 0.5) for g in self.g)
        if not valid or not anyg:
            if self.last_valid and t - self.last_valid > self.grace:
                for i in range(len(self.g)):
                    note(i, 'idle', 'tracking lost')
                self.hist = []; self.last_valid = 0
            return ch
        if self.last_valid and t - self.last_valid > self.grace:
            for i in range(len(self.g)):
                note(i, 'idle', 'tracking found again')
            self.hist = []
        self.last_valid = t
        self.hist.append(f); self.hist = self.hist[-self.smooth:]
        sm = np.mean(self.hist, axis=0)
        act = self.active()
        for i, g in enumerate(self.g):
            usable = g['enabled'] and g['terms'] and conf >= g.get('min_confidence', 0.5)
            on = usable and self._holds(g, True, sm); keep = usable and self._holds(g, False, sm)
            p = self.phase[i]
            if p == 'idle':
                if on:
                    note(i, 'candidate', 'on condition holds')
            elif p == 'candidate':
                if not on:
                    note(i, 'idle', 'on condition stopped holding')
                elif t - self.since[i] >= g['activate_ms'] * 1e6:
                    if act >= 0 and act != i and self.g[act]['priority'] >= g['priority']:
                        continue
                    if act >= 0 and act != i:
                        note(act, 'idle', 'replaced by a higher-priority gesture')
                    note(i, 'active', 'held long enough'); act = i
            elif p == 'active':
                if not keep:
                    note(i, 'releasing', 'off condition failed')
            elif p == 'releasing':
                if keep:
                    self.phase[i] = 'active'
                elif t - self.since[i] >= g['release_ms'] * 1e6:
                    note(i, 'idle', 'released')
                    if act == i:
                        act = -1
        return ch

def replay(sess, hand, gestures, smooth, grace_ms=100):
    """[(t, active gesture id or None, changes)] over every frame of the hand (gaps between frames count as lost)."""
    e = Engine(gestures, smooth, grace_ms); out = []
    for t, f, pres in sess['frames'][hand]:
        ch = e.update(f, True, pres, t)
        a = e.active()
        out.append((t, gestures[a]['id'] if a >= 0 else None, ch, f))
    return out

# ------------------------------------------------------------------------------------------------------- validation

def evaluate(sess, hand, timeline, gid, rounds=None):
    """Per-gesture validation numbers on one session for gesture gid."""
    ts = [x[0] for x in timeline]; on = np.array([x[1] == gid for x in timeline])
    tp = fp = fn = 0; caught = n_g = 0; false = n_o = 0; non_time = 0.0; false_acts = 0
    lat_on, lat_off = [], []
    hp = [p for p in sess['prompts'] if p['hand'] == HANDS[hand]]
    for p in hp:
        st = state_of(p)
        if st is None or (rounds is not None and p['round'] not in rounds):
            continue
        a, b = int(p['start_ns']), int(p['end_ns'])
        i, j = bisect.bisect_left(ts, a), bisect.bisect_right(ts, b)
        seg = on[i:j]
        if not len(seg):
            continue
        if st == gid:
            tp += seg.sum(); fn += (~seg).sum(); n_g += 1; caught += seg.any()
            k = np.argmax(seg) if seg.any() else None
            if k is not None:
                lat_on.append((ts[i + k] - a) / 1e6)
            # release: from the label's end to the first frame it's off
            jj = j
            while jj < len(ts) and on[jj] and ts[jj] - b < 3e9:
                jj += 1
            if jj < len(ts) and on[j - 1]:
                lat_off.append((ts[jj] - b) / 1e6)
        else:
            fp += seg.sum(); n_o += 1; false += seg.any()
            non_time += (b - a) / 1e9
            false_acts += int(seg[0]) + int(((seg[1:].astype(int) - seg[:-1].astype(int)) == 1).sum())
    prec = tp / max(tp + fp, 1); rec = tp / max(tp + fn, 1)
    return dict(precision=round(float(prec), 2), recall=round(float(rec), 2), prompts_caught=f"{caught}/{n_g}",
                false_prompts=f"{false}/{n_o}", false_per_min=round(60 * false_acts / max(non_time, 1e-9), 1),
                activate_ms=round(float(np.median(lat_on)), 0) if lat_on else None,
                release_ms=round(float(np.median(lat_off)), 0) if lat_off else None)

def transitions(sess, hand, timeline, gid):
    """False activations of gid in the second after a different gesture ended (transitions out of other gestures)."""
    ts = [x[0] for x in timeline]; n = bad = 0
    hp = [p for p in sess['prompts'] if p['hand'] == HANDS[hand]]
    for p, q in zip(hp, hp[1:]):
        if state_of(p) in (None, gid) or state_of(q) == gid:
            continue
        b = int(p['end_ns']); i, j = bisect.bisect_left(ts, b), bisect.bisect_right(ts, b + 1e9)
        n += 1; bad += any(timeline[k][1] == gid for k in range(i, j))
    return f"{bad}/{n}"

TIMINGS = [(a, r) for a in (0, 33, 66, 100, 150) for r in (0, 33, 66, 100)]

def choose_timing(sess, hand, gesture, smooth, rounds):
    """The hold times on the fit data: most prompts caught minus false, then the shortest activation."""
    best = None
    for a, r in TIMINGS:
        g = dict(gesture, activate_ms=a, release_ms=r)
        ev = evaluate(sess, hand, replay(sess, hand, [g], smooth), g['id'], rounds)
        c, n = map(int, ev['prompts_caught'].split('/')); f, m = map(int, ev['false_prompts'].split('/'))
        score = (c / max(n, 1)) - (f / max(m, 1)) + 0.5 * ev['precision'] + 0.5 * ev['recall'] - a / 10000
        if best is None or score > best[0]:
            best = (score, a, r)
    return best[1], best[2]

# Production rules: priority (exclusive, higher wins), what they do, and the bar to be switched on.
DEFS = {
    'fist': dict(output='grip', priority=3),
    'point': dict(output='block_pinch', priority=2),
    'claw': dict(output='block_pinch', priority=1),
    'index_pinch': dict(output='none', priority=0),
    'open': dict(output='none', priority=0),
}
NO_DATA = ['middle_pinch', 'ring_pinch', 'little_pinch', 'two_finger_grip', 'thumbs_up']
BAR = dict(precision=0.85, recall=0.7)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--logs', default=os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'logs'))
    ap.add_argument('--smooth', type=int, default=3)
    ap.add_argument('--write'); ap.add_argument('--header')
    ap.add_argument('--replay'); ap.add_argument('--config'); ap.add_argument('--hand', default='both')
    ap.add_argument('--export')
    a = ap.parse_args()
    offs = dict(l.split() for l in open(os.path.join(a.logs, 'landmarks', 'replay_to_session_ns.txt')))
    if a.replay:
        return debug_replay(a, offs)
    sessions = {s: load_session(a.logs, s if '-' in s else '20261125-' + s, int(o)) for s, o in offs.items()}
    config = dict(version=1, source='psvr2-test hands/gesture_fit.py; recordings ' + ', '.join(sorted(sessions)) +
                  ' (labels re-timed)', smooth_frames=a.smooth, grace_ms=100, hands={})
    report = []
    for hand in (0, 1):
        hs = HANDS[hand]; config['hands'][hs] = {}
        print(f"\n######## {hs} hand (features averaged over {a.smooth} frames)")
        avail = {s: set(labelled(sess, hand)[1]) for s, sess in sessions.items()}
        # the built-in gestures, then any other gesture name found in the labels (a new recorded gesture)
        found = sorted(set().union(*avail.values()) - set(DEFS) - {'open'})
        for gid, d in list(DEFS.items()) + [(x, dict(output='none', priority=1)) for x in found]:
            have = [s for s in sorted(sessions) if gid in avail[s]]
            entry = dict(enabled=False, output=d['output'], priority=d['priority'], min_confidence=0.5,
                         activate_ms=66, release_ms=66, terms=[])
            if gid == 'open':
                entry['status'] = ("not a rule: the open hand is what's left when no other gesture is active (as a rule "
                                   "it also fired on every other recorded state)")
                config['hands'][hs][gid] = entry; continue
            if not have:
                entry['status'] = 'no recorded examples'; config['hands'][hs][gid] = entry; continue
            if len(have) >= 2:
                splits = [(x, None, z, None) for x in have for z in have if x != z]
            else:
                splits = [(have[0], (1,), have[0], (2,)), (have[0], (2,), have[0], (1,))]
            vals = []
            for fs, fr, ts_, tr in splits:
                Xf, Sf, _ = labelled(sessions[fs], hand, fr)
                if not len(Xf) or (Sf == gid).sum() == 0:
                    continue
                terms, _ = fit_terms(Xf, Sf == gid)
                g = dict(id=gid, enabled=True, terms=terms, min_confidence=0.5, priority=d['priority'])
                act, rel = choose_timing(sessions[fs], hand, g, a.smooth, fr)
                g.update(activate_ms=act, release_ms=rel)
                tl = replay(sessions[ts_], hand, [g], a.smooth)
                ev = evaluate(sessions[ts_], hand, tl, gid, tr)
                ev['transitions_false'] = transitions(sessions[ts_], hand, tl, gid)
                name = f"fit {fs}{'' if fr is None else ' round %d' % fr[0]} -> test {ts_}{'' if tr is None else ' round %d' % tr[0]}"
                vals.append((name, ev))
                print(f"  {gid:11s} {name}: {' AND '.join(t['feature'] + ' ' + t['op'] + ' ' + str(t['on']) for t in terms)}; "
                      f"hold {act}/{rel} ms -> {ev}")
            # the config rule: fitted on everything with examples; timing on everything
            Xall = np.vstack([labelled(sessions[s], hand)[0] for s in have])
            Sall = np.concatenate([labelled(sessions[s], hand)[1] for s in have])
            terms, f1 = fit_terms(Xall, Sall == gid)
            g = dict(id=gid, enabled=True, terms=terms, priority=d['priority'], min_confidence=0.5)
            act, rel = choose_timing(sessions[have[0]], hand, g, a.smooth, None)
            passed = vals and all(ev['precision'] >= BAR['precision'] and ev['recall'] >= BAR['recall'] for _, ev in vals)
            kind = 'unseen session' if len(have) >= 2 else 'unseen round of prompts (only one session has it)'
            entry.update(terms=terms, activate_ms=act, release_ms=rel, enabled=bool(passed and gid != 'index_pinch'),
                         validation={n: ev for n, ev in vals})
            if gid == 'index_pinch':
                entry['status'] = ('joints don\'t separate the pinch on these cameras (validated on ' + kind +
                                   '); the pinch comes from the custom mapping')
            elif passed:
                entry['status'] = f'calibrated: passed precision >= {BAR["precision"]} and recall >= {BAR["recall"]} on the ' + kind
            else:
                entry['status'] = f'unreliable: below precision {BAR["precision"]} / recall {BAR["recall"]} on the ' + kind
            config['hands'][hs][gid] = entry
            report.append((hs, gid, entry['status']))
        for gid in [x for x in NO_DATA if x not in config['hands'][hs]]:
            config['hands'][hs][gid] = dict(enabled=False, output='none', priority=0, min_confidence=0.5,
                                            activate_ms=66, release_ms=66, terms=[],
                                            status='no recorded examples: needs a recording (gesture_prompts.py)')
    print("\n######## summary")
    for hs, gid, st in report:
        print(f"  {hs:5s} {gid:11s} {st}")
    if a.write:
        json.dump(config, open(a.write, 'w'), indent=1); print("wrote", a.write)
    if a.header:
        text = json.dumps(config, indent=1)
        with open(a.header, 'w') as o:
            o.write("// Copyright 2026, Mehrab Kooner\n// SPDX-License-Identifier: BSL-1.0\n/*!\n * @file\n"
                    " * @brief  The built-in gesture config (generated by psvr2-test hands/gesture_fit.py; don't edit):\n"
                    " *         used when $XDG_CONFIG_HOME/monado/psvr2-gestures.json doesn't exist.\n"
                    " * @ingroup st_ovrn\n */\n#pragma once\n\nnamespace ovrn::gest::rules {\n\n"
                    "inline const char *const kDefaultConfig = R\"json(" + text + ")json\";\n\n} // namespace ovrn::gest::rules\n")
        print("wrote", a.header)

def debug_replay(a, offs):
    """Replays a recording through a config: per frame the features, each enabled gesture's phase and changes."""
    stamp = a.replay
    sess = load_session(a.logs, stamp if '-' in stamp else '20261125-' + stamp, int(offs[stamp]))
    cfg = json.load(open(a.config))
    rows = []
    for hand in ((0, 1) if a.hand == 'both' else (HANDS.index(a.hand),)):
        hs = HANDS[hand]
        gs = [dict(v, id=k) for k, v in cfg['hands'][hs].items() if v.get('enabled')]
        tl = replay(sess, hand, gs, cfg.get('smooth_frames', 1), cfg.get('grace_ms', 100))
        ts = [x[0] for x in tl]
        print(f"== {hs}: {len(gs)} enabled gestures ({', '.join(g['id'] for g in gs) or 'none'})")
        for g in gs:
            print(f"   {g['id']}: {evaluate(sess, hand, tl, g['id'])}")
        for t, act, ch, f in tl:
            label = next((p['gesture'] + '/' + p['orientation'] for p in sess['prompts'] if p['hand'] == hs and
                          int(p['start_ns']) <= t <= int(p['end_ns'])), '')
            rows.append([hs, t, label, act or ''] + ['; '.join(f"{gs[i]['id']} {fr}->{to} ({why})" for i, fr, to, why in ch)]
                        + [round(float(x), 3) for x in f])
            if ch and not a.export:
                print(f"   {t / 1e9:10.3f} label {label:22s} " + '; '.join(f"{gs[i]['id']} {fr}->{to} ({why})" for i, fr, to, why in ch))
    if a.export:
        with open(a.export, 'w', newline='') as o:
            w = csv.writer(o); w.writerow(['hand', 't_ns', 'label', 'active', 'changes'] + FEATURES); w.writerows(rows)
        print("exported", a.export)

if __name__ == '__main__':
    sys.exit(main())
