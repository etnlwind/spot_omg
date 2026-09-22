"""Bounded offline search. Reject foot dragging even when no fall is reported."""
import argparse
import contextlib
import io
import itertools
import json
from pathlib import Path
from types import SimpleNamespace
import numpy as np
from verify_v93_side_step import simulate


def assess(report):
    rows=report['rows']
    active=[r for r in rows if r['stage']=='walk' and r['safety']=='ok']
    def air(r,legs):
        return all(r['clearance_mm'][i]>2 and r['load_n'][i]<.5 for i in legs)
    swing=report['parameters']['v10_experiment'][2]
    # A late left lift during right recovery is a sequence failure, not success.
    la=[r for r in active if r['phase']<=swing+.01 and air(r,(0,2))]
    late_left=[r for r in active if r['phase']>swing+.01 and air(r,(0,2))]
    touchdown=next((r for r in active if la and r['t']>la[0]['t'] and all(r['load_n'][i]>1 for i in (0,2))),None)
    ra=[r for r in active if touchdown and r['t']>touchdown['t'] and air(r,(1,3))]
    recovered=next((r for r in active if ra and r['t']>ra[0]['t'] and all(f>1 for f in r['load_n'])),None)
    end=next(r for r in reversed(rows) if r['stage']=='walk')
    start=rows[99]
    slip=max(float(np.linalg.norm(np.array(r['foot_world_mm'])[i,:2]-np.array(start['foot_world_mm'])[i,:2]))
             for r in active if r['phase']<report['parameters']['v10_experiment'][2] for i in (1,3))
    error=float(max(abs(np.array(end['actual_deg'])-np.array(start['target_deg']))))
    dy=end['position_mm'][1]-start['position_mm'][1]
    dx=end['position_mm'][0]-start['position_mm'][0]
    roll=max(abs(r['roll_deg']) for r in active)
    passed=bool(len(la)>=2 and len(ra)>=2 and recovered and not report['summary']['first_fault']
                and dy>5 and abs(dx)<10 and roll<10 and slip<10 and error<2)
    return dict(passed=passed,left_air_s=len(la)*.02,right_air_s=len(ra)*.02,
                late_left_air_s=len(late_left)*.02,
                left_touchdown_s=touchdown['t'] if touchdown else None,
                right_touchdown_s=recovered['t'] if recovered else None,
                dx_mm=dx,dy_mm=dy,peak_roll_deg=roll,right_stance_slip_mm=slip,
                stand_error_deg=error,fault=report['summary']['first_fault'])


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--fold-lead',action='store_true')
    parser.add_argument('--reassess',action='store_true')
    args=parser.parse_args()
    if args.reassess:
        results=[]
        for folder in sorted(args.output.glob('case-*')):
            report=json.loads((folder/'trajectory.json').read_text(encoding='utf-8'))
            results.append(dict(case=int(folder.name.split('-')[1]),config=report['parameters']['v10_experiment'],**assess(report)))
        (args.output/'validated-results.json').write_text(json.dumps(results,indent=2),encoding='utf-8')
        print(json.dumps(dict(cases=len(results),passed=sum(r['passed'] for r in results),
              left_air_cases=sum(r['left_air_s']>0 for r in results),late_air_cases=sum(r['late_left_air_s']>0 for r in results),
              no_fault=sum(r['fault'] is None for r in results)),indent=2));return
    args.output.mkdir(parents=True,exist_ok=False)
    candidates=[[step,lift,duration,push,.10,0.] for step,lift,duration,push in
                itertools.product((.02,.04),(.02,.045),(.08,.12,.18),(.25,.75))]
    if args.fold_lead:
        candidates=[[.03,lift,duration,.5,catch,delay] for lift,duration,catch,delay in
                    itertools.product((.025,.045),(.08,.12),(.04,.10,.16),(.2,.3))]
    results=[]
    for idx,cfg in enumerate(candidates):
        folder=args.output/f'case-{idx:02d}'
        options=SimpleNamespace(output=folder,profile='attitudepd_v10',input=-1000,
            seconds=4.,lift=[10]*4,width=[0]*4,no_heading=False,transfer_mm=None,v10_config=cfg)
        try:
            with contextlib.redirect_stdout(io.StringIO()):report,*_=simulate(options)
            result=dict(case=idx,config=cfg,**assess(report))
        except Exception as exc:
            result=dict(case=idx,config=cfg,passed=False,error=str(exc))
        results.append(result)
        (args.output/'results.json').write_text(json.dumps(results,indent=2),encoding='utf-8')
        print(json.dumps(result),flush=True)


if __name__=='__main__':main()
