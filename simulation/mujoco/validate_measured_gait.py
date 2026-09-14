"""Long-run and sensitivity validation; no hardware I/O, no hidden fallback."""
import copy,json
from concurrent.futures import ProcessPoolExecutor
from search_measured_gait import trial,OUT
from gait_profiles import load_profiles

def main():
    base=copy.deepcopy(load_profiles()['cruise'])
    selected=json.loads((OUT/'profiles.json').read_text())['profiles']['measured_lift']
    tasks=[(n,p,d,67,'final_nominal') for n,p in [('baseline',base),('measured_lift',selected)] for d in ('forward','left','right')]
    tasks += [('measured_lift',selected,d,17,s) for s in ('delay60','motor85','heavy','soft','fit_candidate') for d in ('forward','left','right')]
    rows=[]
    with ProcessPoolExecutor(max_workers=3) as pool:
        for r in pool.map(trial,tasks):
            rows.append(r)
            print(r['name'],r['direction'],r['scenario'],'fault',r.get('faults',r.get('error')),'tilt',round(r.get('peak_tilt_deg',0),2),'contact',round(r.get('max_mid_contact',1),2),flush=True)
            (OUT/'robust-validation.json').write_text(json.dumps(rows,indent=2))
    passed=all(r.get('eligible',False) and r['peak_tilt_deg']<=3 and r['rms_roll_deg']<=1.5 and r['rms_pitch_deg']<=1.5 for r in rows if r['name']=='measured_lift')
    selected['validation_status']='passed' if passed else 'failed'
    selected['label']='실측 기반 · 선행 발 들기 · 실험'+('' if passed else ' (검증실패)')
    selected['validation']='artifacts/audits/measured-gait-2026-09-12/robust-validation.json'
    (OUT/'profiles.json').write_text(json.dumps({'profiles':{'measured_lift':selected}},indent=2,ensure_ascii=False))
if __name__=='__main__':main()
