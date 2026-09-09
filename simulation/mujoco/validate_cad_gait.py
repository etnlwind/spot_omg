"""Select using longer runs and multiple estimated physical conditions."""
import json
import numpy as np
from optimize_cad_gait import RESULTS,NAMES,evaluate,make_scenario


def main():
    history=json.loads((RESULTS/'search.json').read_text())['trials']
    chosen=sorted([r for r in history if not r['fallen']],key=lambda r:r['score'],reverse=True)[:12]
    # Include lower motor-load alternatives, not just fast candidates.
    gentle=sorted([r for r in history if not r['fallen'] and r['above_rated_fraction']<.26 and r['slip_m_s']<.05],key=lambda r:r['score'],reverse=True)[:6]
    candidates={r['trial']:r for r in chosen+gentle}
    scenarios=('nominal','heavy_slippery','light_grippy','com_offset')
    reports={str(k):dict(params=v['params'],scenarios={}) for k,v in candidates.items()}
    reports['baseline']=dict(params=None,scenarios={})
    for scenario in scenarios:
        p,model=make_scenario(scenario)
        for key,item in reports.items():
            params=None if item['params'] is None else np.array([item['params'][k] for k in NAMES])
            report=evaluate(params,p,model,duration=20,baseline=key=='baseline')
            report['physics_parameters']=p
            item['scenarios'][scenario]=report
            print(scenario,key,round(report['speed_m_s'],3),'FALL' if report['fallen'] else 'ok','yaw',round(report['yaw_deg'],1),'load',round(report['above_rated_fraction'],2),flush=True)
            (RESULTS/'validation.json').write_text(json.dumps(reports,indent=2))
    ranked=[]
    for key,item in reports.items():
        scores=[r['score'] for r in item['scenarios'].values()]
        item['robust_score']=float(np.mean(scores)*.4+np.min(scores)*.6)
        item['all_upright']=all(not r['fallen'] for r in item['scenarios'].values())
        if key!='baseline':ranked.append((item['all_upright'],item['robust_score'],key))
    ranked.sort(reverse=True)
    winner=ranked[0][2]
    selected=dict(reports[winner],trial=winner,selection='All-upright first, then 60% worst-case score + 40% mean across four 20s scenarios',trajectory='C2 foot trajectory v2',search_seed=73)
    (RESULTS/'selected.json').write_text(json.dumps(selected,indent=2))
    (RESULTS/'validation.json').write_text(json.dumps(reports,indent=2))
    print('SELECTED',winner,selected['robust_score'],flush=True)
    make_scenario('nominal')

if __name__=='__main__':main()
