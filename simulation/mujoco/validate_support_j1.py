"""J1 support experiment A/B; no hardware IO or truth-state feedback."""
import json
from pathlib import Path
from diagnose_turn_clearance import run
ROOT=Path(__file__).resolve().parent
OUT=ROOT.parents[1]/'artifacts/upright/2026-09-11/cushion'

def main():
    profiles=json.loads((ROOT/'upright_profiles.json').read_text())['profiles']
    pad=json.loads((ROOT/'foot_cushion_10mm.json').read_text());results=[]
    for scenario,seconds in [('nominal',60),('com_offset',30)]:
        for name in ('cushion_j2lift','cushion_j1level'):
            summary,_=run(1000,0,seconds,profile=name,override=profiles[name],cushion=pad,scenario=scenario)
            results.append(summary);(OUT/'support-j1-validation.json').write_text(json.dumps(results,indent=2))
            l=summary['legs']['FL'];print(name,scenario,summary['safety'],round(summary['speed_m_s'],3),{k:round(l[k],4) if l[k] is not None else None for k in ['peak_tilt_deg','rms_roll_deg','rms_pitch_deg','max_j1_correction_deg','stance_center_speed_m_s','minimum_adaptive_scale']},flush=True)

if __name__=='__main__':main()
