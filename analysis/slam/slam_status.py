#!/usr/bin/env python3
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
"""Look for the headset's tracking state in raw SLAM records (PSVR2_RECORD_SLAM_RAW / RECORD_SLAM=1).

Decodes position from each 512 byte record, then lists every 1/2/4 byte field (outside the known pose fields) that
takes more than one value, with how many distinct values it has, and prints a timeline (one row per 0.5 s) of the
fields with few values (flags and states change rarely) next to the head speed, so a field that changes while the
cameras are covered stands out.
Usage: slam_status.py slamraw.pv2c [max_fields]
"""
import struct, sys
import numpy as np

recs = []
with open(sys.argv[1], 'rb') as f:
    assert f.read(4) == b'PV2C'
    f.read(4)
    while True:
        h = f.read(16)
        if len(h) < 16:
            break
        ns, n, _ = struct.unpack('<QII', h)
        b = f.read(n)
        if len(b) < n:
            break
        if n >= 64 and b[:3] == b'SLA':
            recs.append((ns, b))
if not recs:
    sys.exit('no SLAM records')
t = np.array([r[0] for r in recs], float); t = (t - t[0]) / 1e9
B = np.frombuffer(b''.join(r[1][:512].ljust(512, b'\0') for r in recs), np.uint8).reshape(-1, 512)
pos = B[:, 16:28].copy().view('<f4')
spd = np.r_[0, np.linalg.norm(np.diff(pos, axis=0), axis=1) / np.maximum(np.diff(t), 1e-3)]
known = set(range(0, 4)) | set(range(8, 12)) | set(range(16, 72))  # header, timestamp, pose, cam0 pose
print(f'{len(recs)} records over {t[-1]:.1f} s; position range {np.round(np.ptp(pos, axis=0), 2)} m')

cands = []
for size, fmt in ((1, 'u1'), (2, '<u2'), (4, '<u4')):
    for off in range(0, 512 - size + 1, size):
        if set(range(off, off + size)) & known:
            continue
        v = B[:, off:off + size].copy().view(fmt).ravel()
        k = len(np.unique(v))
        if 1 < k <= 16:
            changes = int(np.sum(np.diff(v.astype(np.int64)) != 0))
            cands.append((changes, off, size, k, v))
# Fewest changes first: a state field flips a handful of times, counters change every record.
cands.sort(key=lambda c: (c[0], c[1]))
mx = int(sys.argv[2]) if len(sys.argv) > 2 else 12
sel = cands[:mx]
print('fields with 2-16 values (offset/size: distinct values, changes):')
for ch, off, size, k, v in sel:
    print(f'  @{off}/{size}: {k} values {sorted(set(v.tolist()))[:8]}, {ch} changes')
print('\ntime  speed m/s | ' + ' '.join(f'@{c[1]:<4d}' for c in sel))
for s in np.arange(0, t[-1], 0.5):
    m = (t >= s) & (t < s + 0.5)
    if not m.any():
        continue
    row = ' '.join(f'{int(np.median(c[4][m])):<5d}' for c in sel)
    print(f'{s:5.1f} {spd[m].mean():7.2f}   | {row}')
