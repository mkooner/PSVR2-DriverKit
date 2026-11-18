#!/usr/bin/env python3
# Copyright 2026, Mehrab Kooner
# SPDX-License-Identifier: BSL-1.0
"""Per-thread CPU use and CPU wait (time ready to run but not running) during a VR session.

Every 100 ms reads /proc/<pid>/task/<tid>/schedstat (time on CPU, time waiting in the run queue) and comm for the
processes that matter: the game, vrserver (our driver inside SteamVR), vrcompositor, monado-service. A frame missed
because the game's threads couldn't get a CPU shows as run-queue wait in the game's threads at that time.

Usage: cpusample.py out.csv [process names...]   (default: wwhd vrserver vrcompositor monado-service); Ctrl+C / SIGTERM
ends it and prints a per-thread summary.
CSV: t_mono_s, pid, process, tid, thread, run_ms, wait_ms (per 100 ms interval), plus cpu (last CPU it ran on).
"""
import os
import signal
import sys
import time

out = sys.argv[1] if len(sys.argv) > 1 else "cpusample.csv"
names = sys.argv[2:] or ["wwhd", "vrserver", "vrcompositor", "monado-service"]
stop = False


def on_signal(*_):
    global stop
    stop = True


signal.signal(signal.SIGINT, on_signal)
signal.signal(signal.SIGTERM, on_signal)


def procs():
    found = []
    for pid in os.listdir("/proc"):
        if not pid.isdigit():
            continue
        try:
            with open(f"/proc/{pid}/comm") as f:
                comm = f.read().strip()
        except OSError:
            continue
        if comm in names:
            found.append((int(pid), comm))
    return found


def read_tasks(pid):
    res = {}
    try:
        tids = os.listdir(f"/proc/{pid}/task")
    except OSError:
        return res
    for tid in tids:
        try:
            with open(f"/proc/{pid}/task/{tid}/schedstat") as f:
                run_ns, wait_ns, _ = f.read().split()
            with open(f"/proc/{pid}/task/{tid}/comm") as f:
                comm = f.read().strip()
            with open(f"/proc/{pid}/task/{tid}/stat") as f:
                cpu = int(f.read().rsplit(")", 1)[1].split()[36])
        except (OSError, ValueError, IndexError):
            continue
        res[int(tid)] = (comm, int(run_ns), int(wait_ns), cpu)
    return res


last = {}
tot = {}  # (process, thread name) -> [run_ns, wait_ns, samples, worst_wait_ms]
t_start = time.monotonic()
with open(out, "w") as f:
    f.write("t_mono_s,pid,process,tid,thread,run_ms,wait_ms,cpu\n")
    plist, plist_t = [], 0.0
    while not stop:
        now = time.monotonic()
        if now - plist_t > 1.0:
            plist, plist_t = procs(), now
        for pid, pname in plist:
            for tid, (comm, run_ns, wait_ns, cpu) in read_tasks(pid).items():
                key = (pid, tid)
                if key in last:
                    dr, dw = run_ns - last[key][0], wait_ns - last[key][1]
                    if dr > 0 or dw > 0:
                        f.write(f"{now:.3f},{pid},{pname},{tid},{comm},{dr / 1e6:.3f},{dw / 1e6:.3f},{cpu}\n")
                    t = tot.setdefault((pname, comm), [0, 0, 0, 0.0])
                    t[0] += dr
                    t[1] += dw
                    t[2] += 1
                    t[3] = max(t[3], dw / 1e6)
                last[key] = (run_ns, wait_ns)
        time.sleep(max(0.0, 0.1 - (time.monotonic() - now)))

span = time.monotonic() - t_start
print(f"{span:.0f} s sampled; per thread: CPU use, time waiting for a CPU, worst wait in a 100 ms interval", file=sys.stderr)
rows = sorted(tot.items(), key=lambda kv: -kv[1][0])
for (pname, comm), (r, w, n, worst) in rows[:40]:
    if r < 0.002 * span * 1e9 and w < 0.002 * span * 1e9:
        continue
    print(f"  {pname:15s} {comm:20s} CPU {100 * r / (span * 1e9):6.1f}%  waiting {100 * w / (span * 1e9):5.1f}%  worst wait {worst:6.1f} ms/100 ms",
          file=sys.stderr)
