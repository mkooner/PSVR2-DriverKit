#!/usr/bin/env python3
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
"""Save the bottom camera pair at given times (seconds from the recording's first frame) as one PNG each.

usage: pv2c_frames.py recording.pv2c out_prefix t1 [t2 ...]
Camera mode 0x10 transfer: 256 byte header, two BC4 images of 1024 (stride) x 1016 rows (1016 wide), back to back."""
import struct, sys
import numpy as np
from PIL import Image

W, H, STRIDE, HDR = 1016, 1016, 1024, 256

def bc4(raw):
    b = np.frombuffer(raw, np.uint8).reshape(-1, 8)
    a0 = b[:, 0].astype(np.int32); a1 = b[:, 1].astype(np.int32)
    bits = np.zeros(len(b), np.uint64)
    for i in range(6):
        bits |= b[:, 2 + i].astype(np.uint64) << np.uint64(8 * i)
    idx = np.stack([((bits >> np.uint64(3 * k)) & np.uint64(7)).astype(np.int32) for k in range(16)], 1)
    pal = np.zeros((len(b), 8), np.int32); pal[:, 0] = a0; pal[:, 1] = a1
    gt = a0 > a1
    for i in range(2, 8):
        pal[:, i] = np.where(gt, ((8 - i) * a0 + (i - 1) * a1) // 7, 0)
    for i in range(2, 6):
        pal[:, i] = np.where(gt, pal[:, i], ((6 - i) * a0 + (i - 1) * a1) // 5)
    pal[:, 6] = np.where(gt, pal[:, 6], 0); pal[:, 7] = np.where(gt, pal[:, 7], 255)
    px = np.take_along_axis(pal, idx, 1).astype(np.uint8)
    bw = STRIDE // 4; bh = len(b) // bw
    return px.reshape(bh, bw, 4, 4).transpose(0, 2, 1, 3).reshape(bh * 4, bw * 4)[:H, :W]

path, prefix = sys.argv[1], sys.argv[2]
want = sorted(float(t) for t in sys.argv[3:])
img_bytes = (STRIDE // 4) * ((H + 3) // 4) * 8
f = open(path, 'rb'); assert f.read(8)[:4] == b'PV2C'
first = None
while want:
    hdr = f.read(16)
    if len(hdr) < 16: break
    ns, ln, dr = struct.unpack('<QII', hdr)
    if first is None: first = ns
    t = (ns - first) / 1e9
    if t + 1e-3 < want[0]:
        f.seek(ln, 1); continue
    data = f.read(ln)
    a = bc4(data[HDR:HDR + img_bytes]); b = bc4(data[HDR + img_bytes:HDR + 2 * img_bytes])
    lift = lambda im: (255 * (im / 255.0) ** 0.5).astype(np.uint8)  # the driver's brightening, to see dark images
    Image.fromarray(np.hstack([lift(a), lift(b)])).resize((1016, 508)).save(f"{prefix}_{want[0]:.1f}.png")
    print(f"{prefix}_{want[0]:.1f}.png (frame at {t:.2f} s)")
    want.pop(0)
