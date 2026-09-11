"""Measure commanded diagonal phase vs actual foot release/contact (evaluation only)."""
import json,csv
from concurrent.futures import ProcessPoolExecutor
import numpy as np
from check_body_height import OUT,ROOT
from diagnose_turn_clearance import run
from validate_support_shift import Recorder

def first_sustained(mask,count=3):
    for i in range(len(mask)-count+1):
        if all(mask[i:i+count]):return i
    return None

def evaluate(name):
    cfg=json.loads((OUT/(name+'.json')).read_text())['profile'];rec=Recorder()
    result,rows=run(1000,0,20.02,profile=name,override=cfg,
        cushion=json.loads((ROOT/'foot_cushion_10mm.json').read_text()),observer=rec)
    byleg={l:[r for r in rows if r['leg']==l] for l in ['FL','FR','RL','RR']}
    events=[]
    for a,b in [('FL','RR'),('FR','RL')]:
        aa,bb=byleg[a],byleg[b];groups=[];group=[]
        for i,r in enumerate(aa):
            if r['swing']:group.append(i)
            elif group:groups.append(group);group=[]
        for group in groups:
            if group[0]==0:continue
            e=dict(pair=a+'-'+b,start_s=aa[group[0]]['time_s'],end_s=aa[group[-1]]['time_s'])
            for l,rr in [(a,aa),(b,bb)]:
                r=[rr[i] for i in group]
                released=first_sustained([x['force_n']<=.2 for x in r])
                lifted=first_sustained([x['clearance_mm']>2 for x in r])
                # Touchdown after the first sustained release, including next stance.
                touch=None
                if released is not None:
                    start=group[released]+3
                    idx=first_sustained([x['force_n']>.2 for x in rr[start:group[-1]+16]])
                    if idx is not None:touch=rr[start+idx]['time_s']
                e[l]=dict(release_s=None if released is None else r[released]['time_s'],
                    clear_2mm_s=None if lifted is None else r[lifted]['time_s'],touchdown_s=touch,
                    peak_clearance_mm=max(x['clearance_mm'] for x in r))
            for key in ['release_s','clear_2mm_s','touchdown_s']:
                x,y=e[a][key],e[b][key]
                e[key+'_rear_minus_front_ms']=None if x is None or y is None else 1000*(y-x)
            events.append(e)
    summary=dict(name=name,dt_s=.02,sustained_samples=3,events=events,pairs={})
    for a,b in [('FL','RR'),('FR','RL')]:
        aa,bb=byleg[a],byleg[b]
        mid=[i for i,r in enumerate(aa) if r['middle_swing']]
        summary['pairs'][a+'-'+b]=dict(
            commanded_swing_flags_identical=all(x['swing']==y['swing'] for x,y in zip(aa,bb)),
            mid_swing_contact_disagreement_fraction=float(np.mean([(aa[i]['force_n']>.2)!=(bb[i]['force_n']>.2) for i in mid])),
            mid_swing_clearance_difference_rms_mm=float(np.sqrt(np.mean([(aa[i]['clearance_mm']-bb[i]['clearance_mm'])**2 for i in mid]))))
    (OUT/(name+'-sync.json')).write_text(json.dumps(summary,indent=2))
    with (OUT/(name+'-sync.csv')).open('w') as f:
        w=csv.DictWriter(f,fieldnames=rows[0]);w.writeheader();w.writerows(rows)
    (OUT/(name+'-sync-joints.json')).write_text(json.dumps(rec.frames,default=lambda x:x.item()))
    print(name,summary['pairs'],flush=True)
    return summary
if __name__=='__main__':
    with ProcessPoolExecutor(max_workers=2) as pool:
        list(pool.map(evaluate,['diagonal-preload-1','diagonal-load-attitude']))
