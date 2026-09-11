"""10mm attached cushion sensitivity; contact softness is an estimate, not foam FEM."""
import json
from pathlib import Path
from diagnose_turn_clearance import run
ROOT=Path(__file__).resolve().parent
OUT=ROOT.parents[1]/'artifacts/upright/2026-09-11/cushion'

def main():
    OUT.mkdir(parents=True,exist_ok=True)
    policy=json.loads((ROOT/'upright_profiles.json').read_text())['profiles']['upright']
    base=json.loads((ROOT/'foot_cushion_10mm.json').read_text())
    results=[]
    for name,friction,softness in [('bare',None,None),('nominal',1.,.02),('firm',.8,.008),('soft_grippy',1.4,.03)]:
        pad=None if name=='bare' else dict(base,friction=[friction,.005,.0001],contact_time_constant_s=softness)
        summary,_=run(1000,0,40,profile='upright',override=policy,cushion=pad)
        summary['case']=name;results.append(summary)
        (OUT/'validation.json').write_text(json.dumps(results,indent=2))
        print(name,summary['safety'],round(summary['speed_m_s'],3),summary['legs'],flush=True)

if __name__=='__main__':main()
