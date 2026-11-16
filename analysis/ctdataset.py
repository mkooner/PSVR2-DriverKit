#!/usr/bin/env python3
"""Reader for Monado constellation tracker datasets (t_constellation_tracker_dataset.cpp format)."""
import struct
import numpy as np


class R:
    def __init__(self, b):
        self.b, self.o = b, 0

    def u(self, fmt):
        v = struct.unpack_from('<' + fmt, self.b, self.o)
        self.o += struct.calcsize('<' + fmt)
        return v

    def pose(self):
        p = self.u('7f')
        return np.array(p[:3]), np.array(p[3:])

    def opt_pose(self):
        (h,) = self.u('B')
        return self.pose() if h else None


def load(path):
    r = R(open(path, 'rb').read())
    (magic,) = r.u('I')
    assert magic == 0x504C6A06, hex(magic)
    (nm,) = r.u('I')
    cams = []
    for _ in range(nm):
        (nc,) = r.u('I')
        for _ in range(nc):
            w, h = r.u('2I')
            K = np.array(r.u('9d')).reshape(3, 3)
            dist = r.u('20d')
            (model,) = r.u('B')
            cams.append(dict(w=w, h=h, K=K, dist=dist, model=model))
    samples, devices = [], {}
    while r.o < len(r.b):
        try:
            (pt,) = r.u('B')
            if pt == 1:
                sid, seq, ts, nb = r.u('3QI')
                blobs = []
                for _ in range(nb):
                    bid, dev, led = r.u('IBB')
                    f = r.u('8f')
                    r.u('4I')
                    size = r.u('2f')
                    bright, grey = r.u('2f')
                    blobs.append(dict(id=bid, dev=dev, led=led, cd=f[0:2], cu=f[2:4], size=size, bright=bright))
                cam_pose = r.opt_pose()
                (nd,) = r.u('I')
                states = {}
                for _ in range(nd):
                    (did,) = r.u('B')
                    prior = r.opt_pose()
                    found = r.opt_pose()
                    states[did] = dict(prior=prior, found=found)
                mi, ci = r.u('2I')
                samples.append(dict(id=sid, seq=seq, ts=ts, blobs=blobs, cam_pose=cam_pose, states=states, mosaic=mi,
                                    cam=ci))
            elif pt == 2:
                (did,) = r.u('B')
                (n,) = r.u('I')
                leds = {}
                for _ in range(n):
                    v = r.u('8f')
                    (lid,) = r.u('B')
                    leds[lid] = dict(pos=np.array(v[0:3]), normal=np.array(v[3:6]), radius=v[6], vis=v[7])
                devices[did] = leds
            else:
                break
        except struct.error:
            break
    return cams, samples, devices


def qrot(q, v):
    u, w = q[:3], q[3]
    return v + 2 * np.cross(u, np.cross(u, v) + w * v)


def project(K, pose, p):
    """pose = (pos, quat) of the device in the camera (OpenCV axes); returns undistorted pixels."""
    c = qrot(pose[1], p) + pose[0]
    return np.array([K[0, 0] * c[0] / c[2] + K[0, 2], K[1, 1] * c[1] / c[2] + K[1, 2]]), c[2]
