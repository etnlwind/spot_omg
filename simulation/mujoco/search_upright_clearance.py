"""Compare taller postures with visible swing clearance; no hardware IO."""
import json
from pathlib import Path
from diagnose_turn_clearance import run
from gait_profiles import load_profiles, foot_targets
OUT=Path(__file__).resolve().parents[2]/'artifacts/upright/2026-09-11'

def main():
 OUT.mkdir(parents=True,exist_ok=True)
 results=[]
 candidates=[('cruise',load_profiles()['cruise'])]
 for height in (.23,.24,.25):
  for lift in (.03,.04):
   name=f'upright-{round(height*1000)}-{round(lift*1000)}'
   candidates.append((name,dict(label=name,family='trot',params=[1.4,.64,.065,lift,height,-.01,.75])))
 for name,profile in candidates:
  result,_=run(1000,0,14,profile='cruise' if name=='cruise' else 'level15',override=profile)
  result['name']=name
  result['neutral_deg']=foot_targets(profile['params'],0,0).reshape(4,3)[0].tolist()
  result['eligible']=result['safety']=='ok' and all(v['middle_swing_contact_fraction'] is not None and v['middle_swing_contact_fraction']<.1 and v['peak_clearance_mm']>20 and v['peak_tilt_deg']<8 for v in result['legs'].values())
  results.append(result)
  (OUT/'search.json').write_text(json.dumps(results,indent=2))
  print(name,result['eligible'],round(result['speed_m_s'],3),[(v['peak_clearance_mm'],v['middle_swing_contact_fraction'],v['peak_tilt_deg']) for v in result['legs'].values()],flush=True)
if __name__=='__main__':main()
