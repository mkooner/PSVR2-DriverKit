#!/usr/bin/env python3
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
"""Image-based pinch detector: a small CNN on the hand crops MediaPipe already gets (MERCURY_CROP_LOG from hand_sim:
112x112 grayscale, the hand upright as MediaPipe's landmark model sees it), trained on one labelled recording and
tested on another. Hand Landmarker's joints don't carry the pinch on PS VR2 camera images (2026-11-25: every joint
feature, 3D and 2D, 0.27-0.62 separation), but the pinch is plainly visible in the crops.

Labels: gesture-labels-<stamp>-retimed.csv (re-timed to when the hand changed). Pinch = 1; open hands (all kinds),
fists, points, claws, position prompts = 0; transitions, bursts and the dashboard gesture aren't used to fit. Left-hand
crops are mirrored so one model serves both hands. Per frame the hand's pinch probability is the mean over its cameras;
the click is replayed as the driver would (mean of the last N frames, on above a threshold, each change held some ms)
and scored per pinch type: held, slow, quick taps, bursts of three, open hands that must not click.

usage: pinch_image.py train.bin:labels.csv:offset test.bin:labels.csv:offset [--epochs N] [--onnx out.onnx]"""
import argparse, bisect, collections, csv, math, os, sys, time
import numpy as np
import torch, torch.nn as nn

REC = 8 + 1 + 1 + 4 + 112 * 112
NOT_PINCH = ('open', 'position', 'fist', 'point', 'claw')

def load(spec):
    path, lab, off = spec.split(':')
    raw = np.fromfile(path, dtype=np.uint8)
    n = len(raw) // REC
    raw = raw[:n * REC].reshape(n, REC)
    ts = raw[:, :8].copy().view(np.int64)[:, 0] + int(off)
    hand = raw[:, 8].astype(np.int8); view = raw[:, 9].astype(np.int8)
    pres = raw[:, 10:14].copy().view(np.float32)[:, 0]
    img = raw[:, 14:].reshape(n, 112, 112)
    prompts = [r for r in csv.DictReader(open(lab)) if r['hand'] in ('left', 'right')]
    y = np.full(n, -1, np.int8); kind = np.array([''] * n, dtype=object)
    for h, hs in ((0, 'left'), (1, 'right')):
        idx = np.where(hand == h)[0]; order = idx[np.argsort(ts[idx])]; tsh = ts[order]
        for p in prompts:
            if p['hand'] != hs:
                continue
            g = p['gesture']
            lab_v = 1 if g == 'pinch' else (0 if g in NOT_PINCH else -1)
            a0, b0 = int(p['start_ns']), int(p['end_ns'])
            # Training labels from the steady part only: frames show the hand opening before a pinch label ends
            # (people let go when the next prompt starts) and still moving at an open label's start.
            if lab_v == 1:
                a1, b1 = a0, a0 + int(0.6 * (b0 - a0))
            elif lab_v == 0:
                a1, b1 = a0 + int(0.3e9), b0 - int(0.5e9)
            else:
                a1, b1 = a0, b0
            i, j = bisect.bisect_left(tsh, a1), bisect.bisect_right(tsh, b1)
            y[order[i:j]] = lab_v
            kind[order[i:j]] = g + '/' + p['orientation']
    keep = pres >= 0.5
    img = img.copy()
    img[hand == 0] = img[hand == 0][:, :, ::-1]  # left hands mirrored to look like right hands
    return dict(name=os.path.basename(path), ts=ts, hand=hand, view=view, img=img, y=y, keep=keep, prompts=prompts)

class Net(nn.Module):
    def __init__(self):
        super().__init__()
        c = [1, 16, 32, 48, 64]
        layers = []
        for a, b in zip(c, c[1:]):
            layers += [nn.Conv2d(a, b, 3, padding=1), nn.BatchNorm2d(b), nn.ReLU(), nn.MaxPool2d(2)]
        self.f = nn.Sequential(*layers)
        self.head = nn.Sequential(nn.Dropout(0.3), nn.Linear(64, 1))

    def forward(self, x):  # x: (n, 1, 96, 96) in 0..1
        return self.head(self.f(x).mean((2, 3))).squeeze(1)

def batch(img, idx, train, rng):
    out = np.empty((len(idx), 1, 96, 96), np.float32)
    for k, i in enumerate(idx):
        if train:
            oy, ox = rng.integers(0, 17, 2)
            a = img[i, oy:oy + 96, ox:ox + 96].astype(np.float32) / 255.0
            a = (a - a.mean()) * rng.uniform(0.7, 1.4) + a.mean() + rng.uniform(-0.15, 0.15)  # contrast, brightness
            a += rng.normal(0, rng.uniform(0, 0.04), a.shape)  # sensor noise
        else:
            a = img[i, 8:104, 8:104].astype(np.float32) / 255.0
        lo, hi = np.percentile(a, 1), np.percentile(a, 99)  # the driver's per-crop contrast stretch
        out[k, 0] = np.clip((a - lo) / max(hi - lo, 1e-3), 0, 1)
    return torch.from_numpy(out)

def train(d, epochs, seed=0):
    torch.manual_seed(seed); rng = np.random.default_rng(seed)
    idx = np.where(d['keep'] & (d['y'] >= 0))[0]
    pos, neg = idx[d['y'][idx] == 1], idx[d['y'][idx] == 0]
    net = Net(); opt = torch.optim.AdamW(net.parameters(), 2e-3, weight_decay=1e-3)
    lossf = nn.BCEWithLogitsLoss()
    steps = max(len(pos), len(neg)) * 2 // 64
    for ep in range(epochs):
        net.train(); tot = 0.0; t0 = time.time()
        for _ in range(steps):
            b = np.concatenate([rng.choice(pos, 32), rng.choice(neg, 32)])  # balanced batches
            x = batch(d['img'], b, True, rng); yb = torch.from_numpy(d['y'][b].astype(np.float32))
            opt.zero_grad(); loss = lossf(net(x), yb); loss.backward(); opt.step(); tot += loss.item()
        print(f"  epoch {ep + 1}/{epochs}: loss {tot / steps:.3f} ({time.time() - t0:.0f} s)", file=sys.stderr)
    return net

@torch.no_grad()
def predict(net, d):
    net.eval(); p = np.zeros(len(d['ts']), np.float32)
    for s in range(0, len(p), 512):
        idx = np.arange(s, min(s + 512, len(p)))
        p[idx] = torch.sigmoid(net(batch(d['img'], idx, False, None))).numpy()
    return p

def auc(s, y):
    o = np.argsort(s); r = np.empty(len(s)); r[o] = np.arange(1, len(s) + 1); n1 = y.sum(); n0 = len(y) - n1
    return (r[y].sum() - n1 * (n1 + 1) / 2) / (n1 * n0)

def timeline(d, p, hand):
    """Per camera frame: the mean pinch probability over the hand's cameras."""
    m = (d['hand'] == hand) & d['keep']
    by = collections.defaultdict(list)
    for t, v in zip(d['ts'][m], p[m]):
        by[int(t)].append(float(v))
    return sorted((t, sum(v) / len(v)) for t, v in by.items())

def clicks(T, N, th_on, th_off, hold_ms):
    out, hist, on, since, last = [], [], False, 0, None
    for t, pi in T:
        if last is not None and t - last > 100e6:
            hist, on, since = [], False, 0
        last = t; hist.append(pi); hist = hist[-N:]
        m = sum(hist) / len(hist)
        want = (m >= th_on) if not on else (m >= th_off)
        if want != on:
            since = since or t
            if t - since >= hold_ms * 1e6:
                on, since = want, 0
        else:
            since = 0
        out.append((t, on))
    return out

def score(d, hand, C):
    hs = ('left', 'right')[hand]; ts = [t for t, _ in C]
    def seg(a, b):
        i, j = bisect.bisect_left(ts, a), bisect.bisect_right(ts, b); return [C[k][1] for k in range(i, j)]
    def presses(v): return sum(1 for i in range(1, len(v)) if v[i] and not v[i - 1]) + (1 if v and v[0] else 0)
    r = collections.defaultdict(list); hp = [p for p in d['prompts'] if p['hand'] == hs]
    for i, p in enumerate(hp):
        a, b, g, o = int(p['start_ns']), int(p['end_ns']), p['gesture'], p['orientation']
        v = seg(a, b)
        if not v:
            continue
        if g == 'pinch' and o == 'tap':
            r['taps'].append(any(seg(a, b + int(0.35e9))))
        elif g == 'pinch' and o == 'slow_hold':
            pr = hp[i - 1] if i else None
            cs = int(pr['start_ns']) if pr is not None and pr['orientation'] == 'slow_close' else a
            ce = int(pr['end_ns']) if pr is not None and pr['orientation'] == 'slow_close' else a
            first = next((t for t, on in C if cs <= t <= b and on), None)
            r['slow'].append(first is not None)
            if first is not None:
                r['slow_t'].append((first - ce) / 1e9)
        elif g == 'pinch':
            r['held'].append(any(v)); r['held_on'].append(sum(v) / len(v))
        elif g == 'burst':
            r['bursts'].append(presses(v))
        elif g == 'open' and o == 'tap_release':
            r['released'].append(not all(v))
        elif g in NOT_PINCH:
            r['false'].append(any(v)); r['false_on'].append(sum(v) / len(v))
    return r

def summary(r):
    f = lambda k: f"{sum(r[k])}/{len(r[k])}" if r.get(k) else '-'
    caught = sum(sum(r.get(k, [])) for k in ('held', 'slow', 'taps')); total = sum(len(r.get(k, [])) for k in ('held', 'slow', 'taps'))
    on = f"{100 * np.mean(r['held_on']):.0f}%" if r.get('held_on') else '-'
    return (f"pinch caught {caught}/{total} (held {f('held')}, slow {f('slow')}, taps {f('taps')}); held on {on}; "
            f"taps released {f('released')}; bursts {','.join(map(str, r.get('bursts', []))) or '-'}; "
            f"slow click vs touch {','.join(f'{x:+.1f}' for x in r.get('slow_t', [])) or '-'}; "
            f"NOT-pinch clicked {f('false')} (on {100 * np.mean(r['false_on']) if r.get('false_on') else 0:.0f}% of their time)")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('train'); ap.add_argument('test')
    ap.add_argument('--epochs', type=int, default=8); ap.add_argument('--onnx'); ap.add_argument('--save')
    a = ap.parse_args()
    torch.set_num_threads(8)
    tr, te = load(a.train), load(a.test)
    for d in (tr, te):
        ok = d['keep'] & (d['y'] >= 0)
        print(f"{d['name']}: {len(d['y'])} crops, labelled {ok.sum()} (pinch {(d['y'][ok] == 1).sum()}, not {(d['y'][ok] == 0).sum()})")
    net = train(tr, a.epochs)
    for d, nm in ((tr, 'fit session (in-sample)'), (te, 'UNSEEN session')):
        p = predict(net, d); ok = d['keep'] & (d['y'] >= 0)
        print(f"\n== {nm} {d['name']}: frame separation (AUC) all {auc(p[ok], d['y'][ok] == 1):.3f}; "
              + ', '.join(f"{hs} {auc(p[ok & (d['hand'] == h)], d['y'][ok & (d['hand'] == h)] == 1):.3f}" for h, hs in ((0, 'left'), (1, 'right'))))
        for h, hs in ((0, 'left'), (1, 'right')):
            T = timeline(d, p, h)
            for N, on, off, hold in ((3, 0.5, 0.35, 30), (3, 0.6, 0.4, 30), (5, 0.6, 0.4, 0)):
                print(f"   {hs:5s} avg {N}, on {on} off {off}, hold {hold} ms: {summary(score(d, h, clicks(T, N, on, off, hold)))}")
    if a.save:
        torch.save(net.state_dict(), a.save)
    if a.onnx:
        net.eval()
        torch.onnx.export(net, torch.zeros(1, 1, 96, 96), a.onnx, input_names=['crop'], output_names=['pinch_logit'],
                          dynamic_axes={'crop': {0: 'n'}, 'pinch_logit': {0: 'n'}}, opset_version=13)
        print("wrote", a.onnx)

def cross_validate(specs, epochs, folds=4):
    """Both (all) sessions together, whole prompts held out a fold at a time: frame separation and per-prompt results
    on the held-out prompts (a pinch prompt caught / another prompt clicked if a third of its frames are over 0.5)."""
    ds = [load(s) for s in specs]
    img = np.concatenate([d['img'] for d in ds]); y = np.concatenate([d['y'] for d in ds])
    keep = np.concatenate([d['keep'] for d in ds]); hand = np.concatenate([d['hand'] for d in ds])
    # prompt id per crop (session, hand, label window): recompute with the full windows
    gid = np.full(len(y), -1, np.int64); kind = np.array([''] * len(y), dtype=object); base = 0; g = 0
    for d in ds:
        for h, hs in ((0, 'left'), (1, 'right')):
            idx = np.where(d['hand'] == h)[0]; order = idx[np.argsort(d['ts'][idx])]; tsh = d['ts'][order]
            for p in d['prompts']:
                if p['hand'] != hs:
                    continue
                i, j = bisect.bisect_left(tsh, int(p['start_ns'])), bisect.bisect_right(tsh, int(p['end_ns']))
                gid[base + order[i:j]] = g; kind[base + order[i:j]] = p['gesture'] + '/' + p['orientation']; g += 1
        base += len(d['y'])
    rng = np.random.default_rng(0); fold_of = rng.integers(0, folds, g)
    P = np.full(len(y), np.nan, np.float32)
    for f in range(folds):
        test_g = fold_of[np.maximum(gid, 0)] == f
        tr = dict(img=img, y=np.where(test_g | (gid < 0), -1, y).astype(np.int8), keep=keep)
        print(f"fold {f + 1}/{folds}", file=sys.stderr)
        net = train(tr, epochs, seed=f)
        idx = np.where(test_g & keep & (gid >= 0))[0]
        net.eval()
        with torch.no_grad():
            for s in range(0, len(idx), 512):
                b = idx[s:s + 512]; P[b] = torch.sigmoid(net(batch(img, b, False, None))).numpy()
    ok = ~np.isnan(P) & (y >= 0)
    print(f"held-out prompts, frame separation (AUC): all {auc(P[ok], y[ok] == 1):.3f}; " +
          ', '.join(f"{hs} {auc(P[ok & (hand == h)], y[ok & (hand == h)] == 1):.3f}" for h, hs in ((0, 'left'), (1, 'right'))))
    res = collections.defaultdict(lambda: [0, 0])
    for gi in np.unique(gid[ok]):
        m = (gid == gi) & ~np.isnan(P)
        k = kind[m][0]; on = (P[m] > 0.5).mean() >= 1 / 3
        hs = 'left' if hand[m][0] == 0 else 'right'
        key = (hs, 'pinch ' + k.split('/')[1] if k.startswith('pinch') else 'NOT pinch (' + k.split('/')[0] + ')')
        res[key][0] += 1; res[key][1] += int(on)
    for (hs, k), (n, on) in sorted(res.items()):
        print(f"  {hs:5s} {k:28s}: {'caught' if k.startswith('pinch') else 'clicked'} {on}/{n}")


def train_final(specs, epochs, onnx_path):
    """The model for the driver: trained on every labelled crop of all the given sessions, exported as ONNX."""
    ds = [load(s) for s in specs]
    d = dict(img=np.concatenate([x['img'] for x in ds]), y=np.concatenate([x['y'] for x in ds]),
             keep=np.concatenate([x['keep'] for x in ds]))
    ok = d['keep'] & (d['y'] >= 0)
    print(f"final model: {ok.sum()} labelled crops (pinch {(d['y'][ok] == 1).sum()}, not {(d['y'][ok] == 0).sum()})", file=sys.stderr)
    net = train(d, epochs)
    net.eval()
    torch.save(net.state_dict(), onnx_path + '.pt')  # kept even if the export fails
    torch.onnx.export(net, torch.zeros(1, 1, 96, 96), onnx_path, input_names=['crop'], output_names=['pinch_logit'],
                      opset_version=17, dynamo=False)
    print("wrote", onnx_path, file=sys.stderr)


if __name__ == '__main__':
    torch.set_num_threads(8)
    if len(sys.argv) > 1 and sys.argv[1] == '--cv':
        cross_validate(sys.argv[2:], 6)
    elif len(sys.argv) > 1 and sys.argv[1] == '--final':
        train_final(sys.argv[3:], 6, sys.argv[2])
    else:
        main()
