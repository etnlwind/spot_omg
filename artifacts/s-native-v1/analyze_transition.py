import json,numpy as np
from pathlib import Path

def runs(mask):
 edges=np.diff(np.r_[False,mask,False].astype(int));return list(zip(np.where(edges==1)[0],np.where(edges==-1)[0]))
def analyze(path):
 a=np.loadtxt(path,delimiter=',');w=a[(a[:,0]>=6)&(a[:,0]<28)];out=[]
 for ia,ib,offset in [(0,3,0),(1,2,.5)]:
  ids=np.floor(w[:,1]-offset).astype(int);steps=[]
  for cid in np.unique(ids):
   v=w[ids==cid];f=v[:,1]-offset-cid;v=v[(f>=0)&(f<.5)];f=f[(f>=0)&(f<.5)]
   if len(f)<4 or f[0]>.025 or f[-1]<.475:continue
   main=[];extra=[]
   for leg in (ia,ib):
    z=v[:,2+leg];episodes=runs(z>1)
    if not episodes:main.append(None);extra.append([]);continue
    best=max(episodes,key=lambda t:np.sum(z[t[0]:t[1]]))
    main.append((float(v[best[0],0]),float(v[best[1]-1,0]+.02),float(z[best[0]:best[1]].max())))
    extra.append([dict(duration_ms=int((e-s)*20),peak_mm=float(z[s:e].max())) for s,e in episodes if (s,e)!=best])
   if any(m is None or m[2]<5 for m in main):steps.append(dict(missed=True));continue
   steps.append(dict(liftoff_ms=round(abs(main[0][0]-main[1][0])*1000),touchdown_ms=round(abs(main[0][1]-main[1][1])*1000),peaks_mm=[m[2] for m in main],extra_airborne=extra))
  valid=[s for s in steps if not s.get('missed')]
  out.append(dict(pair=['FL/RR','FR/RL'][len(out)],missed=sum(s.get('missed',False) for s in steps),max_liftoff_ms=max(s['liftoff_ms'] for s in valid),max_touchdown_ms=max(s['touchdown_ms'] for s in valid),median_touchdown_ms=float(np.median([s['touchdown_ms'] for s in valid])),steps=steps))
 paused=[(e-s)*20 for s,e in runs(np.all(w[:,2:6]<=1,axis=1)) if s>0 and e<len(w)]
 return dict(pairs=out,all_ground_median_ms=float(np.median(paused)),all_ground_max_ms=int(max(paused)),peak_tilt_deg=float(abs(a[:,-3:-1]).max()))
if __name__ == '__main__':
 import argparse
 parser=argparse.ArgumentParser(description='Analyze 20ms physics traces; primary swing is the airborne interval with the greatest clearance integral. Extra bounces remain in the report.')
 parser.add_argument('csv', nargs='+')
 args=parser.parse_args()
 print(json.dumps({path:analyze(path) for path in args.csv},indent=2))
