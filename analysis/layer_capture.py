#!/usr/bin/env python3
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
"""
Driver direct mode layer captures (OVRD_DIRECT_MODE_CAPTURE, logs/direct-<stamp>-layer<N>.rgba): writes a PNG of each
layer (colour, and its alpha channel as a grey image) and reports how much of each layer is visible, i.e. what
blending it over the game layer adds.

Usage: layer_capture.py <capture.rgba> [...]
"""
import struct
import sys
import zlib

import numpy as np


def png(path, img):
    h, w = img.shape[:2]
    ch = 1 if img.ndim == 2 else img.shape[2]
    raw = b''.join(b'\x00' + img[y].tobytes() for y in range(h))
    def chunk(t, d):
        return struct.pack('>I', len(d)) + t + d + struct.pack('>I', zlib.crc32(t + d) & 0xffffffff)
    ctype = {1: 0, 3: 2, 4: 6}[ch]
    open(path, 'wb').write(b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', w, h, 8, ctype, 0, 0, 0)) +
                           chunk(b'IDAT', zlib.compress(raw, 6)) + chunk(b'IEND', b''))


def load(path):
    with open(path, 'rb') as f:
        w, h, fmt = (int(v) for v in f.readline().split())
        a = np.frombuffer(f.read(), np.uint8)
    a = a[:w * h * 4].reshape(h, w, 4)
    if fmt in (44, 50):  # B8G8R8A8: to RGBA
        a = a[:, :, [2, 1, 0, 3]]
    return a, w, h, fmt


def main():
    for path in sys.argv[1:]:
        a, w, h, fmt = load(path)
        alpha = a[:, :, 3].astype(np.float32) / 255
        rgb = a[:, :, :3].astype(np.float32)
        visible = alpha > 0.02
        print(f"{path}: {w}x{h} format {fmt}")
        print(f"  alpha: mean {alpha.mean():.3f}; visible (alpha > 0.02) {100 * visible.mean():.1f}% of pixels, "
              f"opaque (alpha > 0.98) {100 * (alpha > 0.98).mean():.1f}%")
        print(f"  colour where visible: mean {rgb[visible].mean(axis=0).round(1) if visible.any() else '-'}; "
              f"where transparent: mean {rgb[~visible].mean(axis=0).round(1) if (~visible).any() else '-'}")
        step = max(1, max(w, h) // 1200)  # keep the PNGs viewable
        base = path[:-5] if path.endswith('.rgba') else path
        png(base + '.png', np.ascontiguousarray(a[::step, ::step, :3]))
        png(base + '-alpha.png', np.ascontiguousarray(a[::step, ::step, 3]))
        print(f"  wrote {base}.png and {base}-alpha.png")


if __name__ == '__main__':
    main()
