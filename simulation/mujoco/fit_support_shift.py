"""Offline iterative learning of phase-indexed body compensation. No live IO."""
import copy,json
from pathlib import Path
import numpy as np
from diagnose_turn_clearance import run
ROOT=Path(__file__).resolve().parent
OUT=ROOT.parents[1]/'artifacts/upright/2026-09-11/cushion'

def main():
    profile=copy.deepcopy(json.loads((ROOT/'upright_profiles.json').read_text())['profiles']['cushion_forward'])
    pad=json.loads((ROOT/'foot_cushion_10mm.json').read_text())
    config=dict(lateral_m=.005,lower_m=.01,feedback_gain=.15,ki=0.,lead_s=.1,feedforward_coefficients=np.zeros((2,7)).tolist())
    profile['support_shift']=config;results=[]
    for iteration in range(7):
        result,rows=run(1000,0,24,profile='support_shift',override=copy.deepcopy(profile),cushion=pad)
        results.append(result);(OUT/'support-shift-fit.json').write_text(json.dumps(results,indent=2))
        leg=result['legs']['FL'];print(iteration,result['safety'],round(result['speed_m_s'],3),round(leg['peak_tilt_deg'] or 0,2),[(k,round(v['middle_swing_contact_fraction'] or 0,2)) for k,v in result['legs'].items()],flush=True)
        data=[r for r in rows if r['leg']=='FL' and r['time_s']>9 and r['moving']]
        if len(data)<20:break
        phase=np.array([r['phase'] for r in data])*2*np.pi
        x=np.array([np.ones(len(phase)),np.sin(phase),np.cos(phase),np.sin(2*phase),np.cos(2*phase),np.sin(3*phase),np.cos(3*phase)]).T
        y=np.array([[r['roll_deg'],r['pitch_deg']] for r in data])
        update=np.linalg.lstsq(x,y,rcond=None)[0].T
        config['feedforward_coefficients']=(np.asarray(config['feedforward_coefficients'])+.3*update).tolist()
    # The final coefficient update has NOT been simulated. Never publish it as
    # a verified fit. Keep it separately, and export the best actually run row.
    (OUT/'support-shift-next-unvalidated.json').write_text(json.dumps(profile,indent=2))
    tested=[r for r in results if r['safety']=='ok' and r['legs']['FL']['peak_tilt_deg'] is not None]
    if tested:
        best=min(tested,key=lambda r:r['legs']['FL']['peak_tilt_deg'])
        (OUT/'support-shift-fitted-profile.json').write_text(json.dumps(best['profile'],indent=2))

if __name__=='__main__':main()
