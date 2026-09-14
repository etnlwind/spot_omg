"""Additional CAD cushion-bottom IK candidates, with the same measured-data plant."""

# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[4]))
import copy,json,itertools
from concurrent.futures import ProcessPoolExecutor
from simulation.mujoco.scripts.tuning.search_measured_gait import trial, OUT

def main():
    base=json.loads((OUT/'profiles.json').read_text())['profiles']['measured_lift']
    candidates=[]
    for i,(lift,duty,period) in enumerate(itertools.product((.022,.032),(.5,.6),(.85,1.05))):
        p=copy.deepcopy(base);p.pop('measured_swing',None);p.pop('turn_reverse_params',None)
        p['params']=[period,duty,.08,lift,.21,-.035,.75]
        p['support_shift']={'turn_path':'arc','turn_sweep_rad':.3,'constant_body_height':True,'lateral_m':0.,'lower_m':0.,'turn_lift_ramp':.4,'lead_s':0.,'kinematic_lead_s':0.}
        candidates.append((f'cad_{i:02d}',p))
    rows=[]
    with ProcessPoolExecutor(max_workers=3) as pool:
        for r in pool.map(trial,[(n,p,'left',12,'nominal') for n,p in candidates]):
            rows.append(r);print(r['name'],round(r['score'],2),r.get('error',''),r.get('min_peak_clearance_mm'),r.get('max_mid_contact'),flush=True)
            (OUT/'cad-search.json').write_text(json.dumps(rows,indent=2))
        selected=sorted(rows,key=lambda r:r['score'])[:2]
        more=list(pool.map(trial,[(r['name'],r['profile'],d,14,'nominal') for r in selected for d in ('forward','right')]))
    (OUT/'cad-validation.json').write_text(json.dumps(more,indent=2))
    for r in more:print(r['name'],r['direction'],round(r['score'],2),flush=True)
if __name__=='__main__':main()
