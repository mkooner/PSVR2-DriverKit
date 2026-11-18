#!/usr/bin/env python3
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
"""Did Basalt diverge? Position range and drift of tracking.csv against the headset's SLAM path over the same time.
Usage: diverge_check.py tracking.csv record.csv"""
import sys, numpy as np
B=np.genfromtxt(sys.argv[1],delimiter=',',skip_header=1); B=B[B[:,4]!=42]
t=B[:,0]; p=B[:,1:4]
S=np.array([[float(x) for x in l.split(',')[2:6]] for l in open(sys.argv[2]) if l.startswith('S,')]); S=S[np.argsort(S[:,0])]
S=S[(S[:,0]>=t[0])&(S[:,0]<=t[-1])]
span=lambda x: np.linalg.norm(x.max(0)-x.min(0))
pb=np.linalg.norm(np.diff(p,axis=0),axis=1).sum(); ps=np.linalg.norm(np.diff(S[:,1:4],axis=0),axis=1).sum()
print(f'{(t[-1]-t[0])*1e-9:.0f} s: Basalt extent {span(p):.2f} m, path {pb:.1f} m | headset extent {span(S[:,1:4]):.2f} m, path {ps:.1f} m -> {"DIVERGED" if span(p) > 3*span(S[:,1:4]) + 0.5 else "ok"}')
