"""Validate saved front lift candidate; preserve every failure and cycle metric."""
import argparse,json,copy
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
from search_measured_gait import trial
from search_front_turn_lift import OUT

def main():
    parser=argparse.ArgumentParser(__doc__);parser.add_argument('--short-only',action='store_true');args=parser.parse_args()
    bundle=json.loads((OUT/'profiles.json').read_text())
    profile=bundle['profiles']['measured_front_lift'];profile['position_wbc']['turn_only']=True
    (OUT/'profiles.json').write_text(json.dumps(bundle,indent=2,ensure_ascii=False))
    tasks=[] if args.short_only else [('front_lift',profile,d,67,'final_nominal') for d in ('left','right')]
    tasks += [('front_lift',profile,d,17,scenario) for scenario in ('delay60','motor85') for d in ('left','right')]
    tasks += [('front_lift',profile,'forward',17,'final_nominal')]
    rows=[]
    with ProcessPoolExecutor(max_workers=3) as pool:
        for r in pool.map(trial,tasks):
            rows.append(r);print(r['direction'],r['scenario'],r.get('faults',r.get('error')),r.get('max_mid_contact'),r.get('yaw_rate_deg_s'),flush=True)
            (OUT/('stress-validation.json' if args.short_only else 'validation.json')).write_text(json.dumps(rows,indent=2))
    profile['validation_status']='failed'
    (OUT/'profiles.json').write_text(json.dumps(bundle,indent=2,ensure_ascii=False))
if __name__=='__main__':main()
