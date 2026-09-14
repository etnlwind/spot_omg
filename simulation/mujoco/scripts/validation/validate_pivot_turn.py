"""Long-run torso-center audit; translation is measured, never pinned."""

# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[4]))
import json,copy
from concurrent.futures import ProcessPoolExecutor
from simulation.mujoco.scripts.analysis.audit_pivot_center import OUT, trial

def main():
    candidates=[]
    for file,name in [('refined-shift-search.json','center_refine_80'),('inside-search.json','inside_60_4')]:
        p=copy.deepcopy(next(r['profile'] for r in json.loads((OUT/file).read_text()) if r['name']==name));candidates.append((name,p))
    rows=[]
    with ProcessPoolExecutor(max_workers=2) as pool:
        for r in pool.map(trial,[(n,d,67,p) for n,p in candidates for d in ('left','right')]):
            rows.append(r);print(r['profile'],r['direction'],r['max_center_excursion_mm'],r['net_translation_mm'],r['yaw_rate_deg_s'],r['faults'],flush=True)
            (OUT/'long-validation.json').write_text(json.dumps(rows,indent=2))
    chosen=min(candidates,key=lambda c:sum(10000*bool(r['faults'])+r['max_center_excursion_mm']+r['net_translation_mm']+10*max(0,24-abs(r['yaw_rate_deg_s'])) for r in rows if r['profile']==c[0]))
    p=chosen[1];p['source_candidate']=chosen[0];p['label']='몸체 중심 · 제자리 회전 · 실험 (검증실패)';p['validation_status']='failed'
    p['validation']='artifacts/audits/pivot-center-2026-09-12/long-validation.json'
    (OUT/'profiles.json').write_text(json.dumps({'profiles':{'center_pivot':p}},indent=2,ensure_ascii=False))
    print('SELECTED',chosen[0],flush=True)
if __name__=='__main__':main()
