"""Full-input 60-second regression after the 2026-09-09 tilt incident."""
import json
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
from search_gait_profiles import physics
from virtual_robot import RobotController
from cad_physics import Simulation


def run_case(case, params=None, turn_reverse_params=None):
    profile,scenario,motion,delay=case
    p,m=physics(scenario);p['bno055']={'fusion_delay_s':delay}
    r=RobotController(Simulation(p,m));r.select_profile(profile)
    if params is not None:r.profiles[profile]['params']=params
    if turn_reverse_params is not None:r.profiles[profile]['turn_reverse_params']=turn_reverse_params
    peak=0.;bad=False;first_fault=None;seq=0;responses=[]
    for i in range(3500):
        t=i*.02
        if i==100:
            seq+=1;r.command(f'drive 0 0 {seq}',t)
        if 100<=i<3100 and i%10==0:
            elapsed=t-2;linear,yaw=1000,0
            if motion=='turn_forward' and int(elapsed/6)%2==0:linear,yaw=0,-500
            elif motion=='reverse' and int(elapsed/10)%2==0:linear=-1000
            elif motion=='arc':yaw=500 if int(elapsed/10)%2==0 else -500
            elif motion=='sweep':linear=(0,250,500,750,1000)[min(4,int(elapsed/12))]
            seq+=1;r.command(f'@D {seq} {linear} {yaw}',t)
        if i==3100:seq+=1;r.command(f'@S {seq}',t)
        r.tick(t);row=r.plant.row()
        peak=max(peak,abs(row['roll_deg']),abs(row['pitch_deg']))
        bad=bad or any(not c.endswith('_foot') for c in row['contacts'])
        if r.safety!='ok' and first_fault is None:first_fault=t
        out=r.drain().decode()
        if 'stopped' in out:responses.append(out.strip())
    return dict(profile=profile,scenario=scenario,motion=motion,delay=delay,reverse_limit=600 if profile in ('trot','highstep') else 1000,peak_tilt_deg=peak,
                nonfoot_contact=bad,first_fault_s=first_fault,safety=r.safety,
                stopped=r.motion is None and r.transition is None,responses=responses)


if __name__=='__main__':
    cases=[(p,'nominal',motion,.02) for p in ('cruise','crawl','legacy','trot','highstep')
           for motion in ('straight','turn_forward','reverse','sweep')]
    cases += [('cruise',s,m,d) for s,d in (('heavy_slippery',.02),('com_offset',.02),('nominal',.06))
              for m in ('straight','turn_forward','arc')]
    rows=[]
    with ProcessPoolExecutor(max_workers=4) as pool:
        for r in pool.map(run_case,cases):
            rows.append(r);print(json.dumps(r),flush=True)
            Path(__file__).with_name('high_speed_balance_validation.json').write_text(json.dumps(rows,indent=2)+'\n')
    assert all(r['safety']=='ok' and r['stopped'] and not r['nonfoot_contact'] for r in rows)
