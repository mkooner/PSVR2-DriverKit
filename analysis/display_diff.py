#!/usr/bin/env python3
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
"""Every difference in what SteamVR knows about the display between two drivers (tools/vrdisplaydump output).

Properties are named from openvr.h. The lens distortion maps (33x33 per eye, as SteamVR samples them) are compared in
render-target pixels, and each map's smoothness is shown (second differences): a map with bumps or folds bends a still
image unevenly.

Usage: display_diff.py ours.txt sony.txt [--openvr-h /usr/include/openvr/openvr.h]
"""
import argparse
import re

import numpy as np


def prop_names(header):
    names = {}
    try:
        txt = open(header).read()
    except OSError:
        return names
    for m in re.finditer(r"\b(Prop_\w+)\s*=\s*(\d+)", txt):
        names[int(m.group(2))] = m.group(1)
    return names


def load(path):
    props, other, dist = {}, {}, {}
    for line in open(path):
        x = line.split()
        if not x:
            continue
        if x[0] == "prop":
            props[int(x[1])] = (x[2], " ".join(x[3:]))
        elif x[0] == "distortion":
            eye = int(x[1])
            dist.setdefault(eye, []).append([float(v) for v in x[2:]])
        else:
            other[" ".join(x[:2]) if x[0] in ("projection_raw", "eye_to_head") else " ".join(x[:4]) if x[0] == "hidden_area" else x[0]] = x
    return props, other, {k: np.array(v) for k, v in dist.items()}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("a")
    ap.add_argument("b")
    ap.add_argument("--openvr-h", default="/usr/include/openvr/openvr.h")
    a = ap.parse_args()
    names = prop_names(a.openvr_h)
    pa, oa, da = load(a.a)
    pb, ob, db = load(a.b)
    A, B = a.a.split("/")[-1], a.b.split("/")[-1]

    print(f"== properties that differ ({A} vs {B})")
    skip = ("Serial", "Model", "Manufacturer", "Icon", "Firmware", "Path", "Resource", "Name", "Tracking", "Registered")
    for pid in sorted(set(pa) | set(pb)):
        va, vb = pa.get(pid), pb.get(pid)
        if va == vb:
            continue
        n = names.get(pid, str(pid))
        mark = "   " if any(k in n for k in skip) else ">> "
        print(f"{mark}{n:55s} {A}: {va[1] if va else '(none)'}   {B}: {vb[1] if vb else '(none)'}")

    print("\n== display model")
    for k in sorted(set(oa) | set(ob)):
        va, vb = oa.get(k), ob.get(k)
        sa = " ".join(va[1:] if va else ["(none)"])
        sb = " ".join(vb[1:] if vb else ["(none)"])
        print(f"{'   ' if sa == sb else '>> '}{k:28s} {A}: {sa}\n{'':31s} {B}: {sb}")

    w = h = None
    if "render_target" in oa:
        w, h = float(oa["render_target"][1]), float(oa["render_target"][2])
    print(f"\n== lens distortion (33x33 per eye; pixels of a {w:.0f}x{h:.0f} target)" if w else "\n== lens distortion")
    for eye in sorted(set(da) | set(db)):
        for name, d in ((A, da.get(eye)), (B, db.get(eye))):
            if d is None:
                print(f"  eye {eye} {name}: none")
                continue
            g = d[:, 5:7].reshape(33, 33, 2) * ([w, h] if w else [1, 1])  # green channel, (row=v, col=u)
            d2u = np.linalg.norm(g[:, 2:] - 2 * g[:, 1:-1] + g[:, :-2], axis=2)
            d2v = np.linalg.norm(g[2:] - 2 * g[1:-1] + g[:-2], axis=2)
            rough = np.r_[d2u.ravel(), d2v.ravel()]
            inside = ((d[:, 5] >= 0) & (d[:, 5] <= 1) & (d[:, 6] >= 0) & (d[:, 6] <= 1)).mean()
            ca = np.linalg.norm((d[:, 3:5] - d[:, 7:9]) * ([w, h] if w else [1, 1]), axis=1)
            print(f"  eye {eye} {name:30s} inside target {100 * inside:5.1f}%  smoothness (2nd diff) median "
                  f"{np.median(rough):7.3f} p99 {np.percentile(rough, 99):8.3f} max {rough.max():8.3f} px; "
                  f"red-blue spread median {np.median(ca):.2f} px")
        if eye in da and eye in db:
            diff = np.linalg.norm((da[eye][:, 5:7] - db[eye][:, 5:7]) * ([w, h] if w else [1, 1]), axis=1)
            print(f"  eye {eye} difference (green): median {np.median(diff):.1f} px, p90 {np.percentile(diff, 90):.1f}, "
                  f"max {diff.max():.1f}")


if __name__ == "__main__":
    main()
