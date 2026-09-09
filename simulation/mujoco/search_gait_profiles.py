"""Compare gait families without changing physical parameters to gain speed."""
import argparse
import json
from pathlib import Path
import mujoco
import numpy as np
from cad_physics import CAD, build
from gait_profiles import foot_targets
from optimize_cad_gait import NAMES, evaluate

RESULTS=Path(__file__).parent/'gait_search'/'profiles'


def physics(scenario='nominal',dt=.0005):
    p=json.loads((CAD/'physics_parameters.json').read_text());p['timestep_s']=dt
    if scenario=='heavy_slippery':
        for k in p['mass_kg']:
            if k!='battery':p['mass_kg'][k]*=1.2
        p['friction'][0]=.55;p['pack_open_circuit_voltage']=10.8
    elif scenario=='com_offset':
        p['battery_center_cad_m'][0]+=.025;p['battery_center_cad_m'][1]-=.05
        p['command_delay_s']=.04
    elif scenario=='motor_loss':
        for k in ('motor_3215','motor_3250'):
            p[k]['stall_nm']*=.85;p[k]['speed_rad_s']*=.85
    xml,p=build(p,write_scene=False)
    return p,mujoco.MjModel.from_xml_string(xml)


def run(params, family, p, model, duration):
    fn=lambda values,t,scale:foot_targets(values,t,scale,family)
    r=evaluate(np.array(params),p,model,duration=duration,target_function=fn)
    r['family']=family
    r['eligible']=not r['fallen'] and r['peak_tilt_deg']<10 and abs(r['yaw_deg'])<12 and r['slip_m_s']<.06 and r['saturation_fraction']<.2 and r['above_rated_fraction']<.4
    return r


def main():
    parser=argparse.ArgumentParser(__doc__)
    parser.add_argument('--count',type=int,default=60)
    parser.add_argument('--refine',action='store_true',help='repeat the fixed 30-candidate local grid')
    args=parser.parse_args();RESULTS.mkdir(exist_ok=True)
    p,model=physics();rng=np.random.default_rng(20260909);rows=[]
    seeds=[]
    for family in ('trot','crawl','amble','pace','bound'):
        for period in (.5,.65,.8):
            seeds.append((family,[period,.78 if family=='crawl' else .62,.075,.025,.225,-.025,2.]))
    previous=json.loads((RESULTS.parent/'selected.json').read_text())['params']
    seeds.insert(0,('trot',[previous[k] for k in NAMES]))
    if args.refine:
        seeds=[]
        for period in (.7,.85,1.05):
            for stride in (.045,.062,.08):
                for lift in (.012,.02):
                    seeds.append(('trot',[period,.60,stride,lift,.20175,-.035,.75]))
        for family in ('crawl','amble','pace','bound'):
            for period in (.85,1.1,1.4):
                seeds.append((family,[period,.8 if family=='crawl' else .65,.05,.012,.20175,-.035,1.5]))
        args.count=len(seeds)
    for i in range(args.count):
        if i<len(seeds):family,params=seeds[i]
        else:
            family=('trot','trot','trot','amble','crawl')[i%5]
            params=[rng.uniform(.45,.85),rng.uniform(.75,.84) if family=='crawl' else rng.uniform(.54,.7),rng.uniform(.05,.115),rng.uniform(.015,.035),rng.uniform(.205,.24),rng.uniform(-.04,-.005),rng.uniform(0,5)]
        r=run(params,family,p,model,10. if args.refine else 8.);rows.append(r)
        print(i,family,'OK' if r['eligible'] else 'reject','speed',round(r['speed_m_s'],3),'tilt',round(r['peak_tilt_deg'],1),'yaw',round(r['yaw_deg'],1),'slip',round(r['slip_m_s'],3),flush=True)
        (RESULTS/('refined.json' if args.refine else 'search.json')).write_text(json.dumps({'timestep_s':.0005,'seed':20260909,'trials':rows},indent=2))


if __name__=='__main__':main()
