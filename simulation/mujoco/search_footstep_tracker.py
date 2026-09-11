"""Compare requested stride/height/period choices with sensor-started feet.

No physical robot IO. Each requested stride has its own result; a short stride
cannot silently replace a failed longer request.
"""
import copy
import json
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path
import numpy as np
from diagnose_turn_clearance import run

ROOT=Path(__file__).resolve().parent
OUT=ROOT.parents[1]/'artifacts/upright/2026-09-11/cushion'


def trial(job):
    stride,period,height=job
    profile=copy.deepcopy(json.loads((ROOT/'upright_profiles.json').read_text())['profiles']['cushion_support_shift'])
    profile['params'][0]=period;profile['params'][2]=stride;profile['params'][4]=height
    pad=json.loads((ROOT/'foot_cushion_10mm.json').read_text())
    trace=[]
    def observe(now,r):
        state=r.plant.row()
        trace.append(dict(time_s=now,roll=state['roll_deg'],pitch=state['pitch_deg'],
            x=state['position_m'][0],safety=r.safety,moving=r.motion is not None,
            tracking=state['max_tracking_error_deg'],saturated=state['torque_limit_fraction']))
    result,rows=run(1000,0,14,profile='candidate',override=profile,cushion=pad,observer=observe)
    active=[f for f in trace if f['time_s']>=5 and f['moving']]
    faults=[f['time_s'] for f in trace if f['safety']!='ok']
    peak=max((max(abs(f['roll']),abs(f['pitch'])) for f in trace if f['time_s']>=3),default=180.)
    speed=(trace[-1]['x']-trace[250]['x'])/(trace[-1]['time_s']-trace[250]['time_s'])
    contact=max((v['middle_swing_contact_fraction'] if v['middle_swing_contact_fraction'] is not None else 1. for v in result['legs'].values()))
    clearance=min((v['peak_clearance_mm'] if v['peak_clearance_mm'] is not None else 0. for v in result['legs'].values()))
    result.update(requested_stride_mm=1000*stride,requested_speed_m_s=stride/period,
        signed_forward_speed_m_s=speed,first_fault_s=min(faults) if faults else None,
        peak_all_tilt_deg=peak,screen_seconds=14,all_feet_min_peak_clearance_mm=clearance,
        active_samples=len(active))
    # Failure and lack of sustained forward motion rank before speed. This is
    # a screening score, not the final acceptance criteria or a safety proof.
    result['screen_score']=(10000 if faults else 0)+(1000 if speed<=0 else 0)+peak*10+contact*30+max(0,15-clearance)*2+abs(speed-stride/period)*100
    return result


def main():
    OUT.mkdir(parents=True,exist_ok=True)
    jobs=[(s,t,h) for s in (.06,.10,.14) for t in (1.8,2.4,3.2,4.8) for h in (.23,.24)]
    results=[]
    with ProcessPoolExecutor(max_workers=2) as pool:
        futures={pool.submit(trial,job):job for job in jobs}
        for future in as_completed(futures):
            result=future.result();results.append(result)
            (OUT/'footstep-tracker-search.json').write_text(json.dumps(results,indent=2))
            print(len(results),futures[future],result['safety'],round(result['signed_forward_speed_m_s'],3),
                  'tilt',round(result['peak_all_tilt_deg'],1),'score',round(result['screen_score'],1),flush=True)
    best={str(mm):min((r for r in results if round(r['requested_stride_mm'])==mm),key=lambda r:r['screen_score']) for mm in (60,100,140)}
    (OUT/'footstep-tracker-best-by-stride.json').write_text(json.dumps(best,indent=2))


if __name__=='__main__':main()
