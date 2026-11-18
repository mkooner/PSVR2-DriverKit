#!/usr/bin/env python3
# Run a command, print "wall user sys" seconds of it and its children (resource.getrusage).
import resource, subprocess, sys, time
t=time.time(); subprocess.run(sys.argv[1:], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
r=resource.getrusage(resource.RUSAGE_CHILDREN); print(f'{time.time()-t:.2f} {r.ru_utime:.2f} {r.ru_stime:.2f}')
