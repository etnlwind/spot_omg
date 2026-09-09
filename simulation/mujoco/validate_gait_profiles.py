"""Freeze varied profiles, then test longer runs and unseen plant conditions."""
import json
import numpy as np
from search_gait_profiles import RESULTS, physics, run
from optimize_cad_gait import NAMES, evaluate
from servo import SharedGaitPolicy
from cad_gait import KEYS


def main():
    refined=json.loads((RESULTS/'refined.json').read_text())['trials']
    search=json.loads((RESULTS/'search.json').read_text())['trials']
    choices={'crawl':('Crawl / four-beat',refined[20]),
             'cruise':('Cruise / long stride',refined[16]),
             'trot':('Trot / brisk diagonal',search[0]),
             'highstep':('High step / clearance',refined[13])}
    output={};profiles={}
    for name,(label,r) in choices.items():
        profiles[name]={'label':label,'family':r['family'],'params':[r['params'][k] for k in NAMES]}
    for scenario in ('nominal','heavy_slippery','com_offset','motor_loss'):
        p,m=physics(scenario);output[scenario]={}
        for name,profile in profiles.items():
            r=run(profile['params'],profile['family'],p,m,20.)
            output[scenario][name]=r
            print(scenario,name,'OK' if r['eligible'] else 'reject',round(r['speed_m_s'],3),round(r['peak_tilt_deg'],1),round(r['yaw_deg'],1),flush=True)
        if scenario=='nominal':
            policy=SharedGaitPolicy()
            def baseline(params,t,scale):
                values,_=policy.drive_walk_targets(t/1.8,scale,1.,0.)
                return np.array([values[k] for k in KEYS])
            output[scenario]['legacy']=evaluate(None,p,m,duration=20,target_function=baseline)
        (RESULTS/'validation.json').write_text(json.dumps(output,indent=2))
    (RESULTS/'selected.json').write_text(json.dumps({'status':'simulator-only; estimated plant; see validation.json for failures','default':'cruise','profiles':profiles},indent=2))


if __name__=='__main__':main()
