"""Replay the measured-cap reach policy; estimates only, no robot IO."""
import json
from pathlib import Path
from diagnose_turn_clearance import run
ROOT=Path(__file__).resolve().parent
OUT=ROOT.parents[1]/'artifacts/upright/2026-09-11/cushion'

def main():
    OUT.mkdir(parents=True,exist_ok=True)
    policy=json.loads((ROOT/'upright_profiles.json').read_text())['profiles']['cushion_j2lift']
    pad=json.loads((ROOT/'foot_cushion_10mm.json').read_text())
    results=[]
    for name,linear,yaw,seconds,scenario in [('forward',1000,0,60,'nominal'),('reverse',-600,0,24,'nominal'),('left',0,600,24,'nominal'),('right',0,-600,24,'nominal'),('com_offset',1000,0,30,'com_offset')]:
        summary,_=run(linear,yaw,seconds,profile='cushion_j2lift',override=policy,cushion=pad,scenario=scenario)
        summary['case']=name;results.append(summary)
        (OUT/'j2lift-validation.json').write_text(json.dumps(results,indent=2))
        print(name,summary['safety'],round(summary['speed_m_s'],3),[(k,round(v['touchdown_forward_median_mm'] or 0,1),round(v['peak_tilt_deg'] or 0,1),round(v['middle_swing_contact_fraction'] or 0,2)) for k,v in summary['legs'].items()],flush=True)

if __name__=='__main__':main()
