#!/usr/bin/env python3
# Scores sweep.sh results: detector hits on frames with hands (frame range given) vs false hits without.
import sys,re,glob,os,collections
d0,lo,hi=sys.argv[1],int(sys.argv[2]),int(sys.argv[3])
rows=[]
for t in sorted(glob.glob(d0+'/v*.txt')):
    sc=collections.defaultdict(dict)
    for l in open(t):
        f=l.split(); n=int(re.search(r'f(\d+)-',f[0]).group(1)); side=f[0][-5]
        sc[f[1]][(n,side)]=max(float(f[3]),float(f[7]))
    for var,d in sc.items():
        inr=[v for (n,s),v in d.items() if lo<=n<=hi]; out=[v for (n,s),v in d.items() if not lo<=n<=hi]
        pairs=[(d[(n,'L')]+d[(n,'R')])/2 for (n,s) in d if s=='L' and lo<=n<=hi and (n,'R') in d]
        rows.append((sum(v>0.9 for v in inr),os.path.basename(t)[:-4],var,sum(v>0.5 for v in inr),len(inr),sum(p>=0.92 for p in pairs),len(pairs),sum(v>0.5 for v in out),len(out)))
rows.sort(reverse=True)
for r in rows[:int(sys.argv[4]) if len(sys.argv)>4 else 15]:
    print(f'{r[1]:9s} {r[2]:11s} hands >0.9 {r[0]:2d}/{r[4]}  >0.5 {r[3]:2d}   both-views 0.92 {r[5]:2d}/{r[6]}   false {r[7]}/{r[8]}')
