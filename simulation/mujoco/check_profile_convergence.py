"""Repeat nominal trials at half the integration timestep."""
import json
from gait_profiles import load_profiles
from search_gait_profiles import physics, run, RESULTS


def main():
    p,model=physics(dt=.00025)
    rows={}
    for name,profile in load_profiles().items():
        rows[name]=run(profile['params'],profile['family'],p,model,20)
        print(name,round(rows[name]['speed_m_s'],4),round(rows[name]['peak_tilt_deg'],2),flush=True)
        (RESULTS/'convergence.json').write_text(json.dumps({'timestep_s':.00025,'results':rows},indent=2))


if __name__=='__main__':main()
