"""Search clearance AND chassis stability with unchanged estimated plant."""
import json,math
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
import numpy as np
from search_gait_profiles import physics
from cad_physics import Simulation
from virtual_robot import RobotController


def run(case, profile=None, scenario="nominal", seed=55):
    height,period,stride,duty,duration,*stance=case
    down,offset=stance[:2] if stance else (.20175,-.035)
    family=stance[2] if len(stance)>2 else "trot"
    p,m=physics(scenario if scenario in ('heavy_slippery','com_offset') else 'nominal')
    p['bno055']={**p.get('bno055',{}),'seed':seed}
    if scenario=='delay60':p['bno055']['fusion_delay_s']=.06
    r=RobotController(Simulation(p,m));r.select_profile(profile or 'level')
    params=[period,duty,stride,height,down,offset,.75]
    if profile is None:
        r.profiles['level']['params']=params
        r.profiles['level']['family']=family
    else:
        params=r.profiles[profile]['params']
        period,duty,stride,height,down,offset,_=params
        family=r.profiles[profile]['family']
    body=m.body('cad_base').id;feet=[m.geom(n+'_foot').id for n in ('fl','fr','rl','rr')]
    stop=100+round(duration/.02);angles=[];xyz=[];peak=0.;bad=False;first=None;last=None
    torques=[];powers=[];rated=[];heading=[]
    previous=[None]*4;cycle_peak=[0.]*4;peaks=[[] for _ in range(4)];near=mid=0
    for i in range(stop+150):
        t=i*.02
        if i==100:r.command('drive 0 0 1',t)
        if 100<=i<stop and i%10==0:r.command(f'@D {i} 1000 0',t)
        if i==stop:r.command('@S 99999',t)
        r.tick(t);row=r.plant.row();peak=max(peak,abs(row['roll_deg']),abs(row['pitch_deg']));bad |= any(not n.endswith('_foot') for n in row['contacts'])
        if 250<=i<stop:
            if first is None:first=r.plant.data.qpos[:2].copy()
            rotation=r.plant.data.xmat[m.body('robot').id].reshape(3,3)
            heading.append(math.atan2(rotation[1,0],rotation[0,0]))
            torques.append(r.plant.data.ctrl.copy());powers.append(float(np.sum(abs(r.plant.data.ctrl*r.plant.data.qvel[r.plant.v]))));rated.append(row['above_rated_fraction'])
            last=r.plant.data.qpos[:2].copy();angles.append([row['roll_deg'],row['pitch_deg']]);xyz.append(r.plant.data.xipos[body].copy())
            for leg,offset in enumerate((0,.5,.75,.25) if family=="crawl" else (0,.5,.5,0)):
                q=(r.phase-.02/period+offset)%1
                height_actual=(r.plant.data.geom_xpos[feet[leg],2]-m.geom_size[feet[leg],0])*1000
                if previous[leg] is not None and q<previous[leg]:
                    peaks[leg].append(cycle_peak[leg]);cycle_peak[leg]=0.
                cycle_peak[leg]=max(cycle_peak[leg],height_actual);previous[leg]=q
                if duty+.25*(1-duty)<q<duty+.75*(1-duty):
                    mid+=1;near+=height_actual<1.
        r.drain()
        if r.safety!='ok':break
    a=np.array(angles);xyz=np.array(xyz)
    if len(a)<4:return dict(params=params,passed=False,safety=r.safety)
    acc=np.diff(xyz,n=2,axis=0)/.02**2
    # Drop the first partially observed cycle on each foot.
    per_leg=[v[1:] for v in peaks];all_peaks=[x for v in per_leg for x in v]
    stopped=r.motion is None and r.transition is None
    return dict(params=params,family=family,safety=r.safety,nonfoot_contact=bad,stopped=stopped,
                passed=r.safety=='ok' and not bad and stopped,rms_tilt_deg=float(np.sqrt(np.mean(np.sum(a*a,axis=1)))),peak_tilt_deg=peak,
                angular_rate_rms=float(np.sqrt(np.mean(np.sum((np.diff(a,axis=0)/.02)**2,axis=1)))),
                bounce_mm=float(np.std(xyz[:,2])*1000),vertical_acceleration_rms=float(np.sqrt(np.mean(acc[:,2]**2))),
                speed_m_s=float(np.linalg.norm(last-first)/((len(a)-1)*.02)),
                forward_speed_m_s=float(np.dot(last-first,[math.cos(heading[0]),math.sin(heading[0])])/((len(a)-1)*.02)),
                heading_drift_deg=float(np.degrees(np.unwrap(heading)[-1]-heading[0])),
                mean_abs_mechanical_power_w=float(np.mean(powers)),above_rated_joint_fraction=float(np.mean(rated)),
                peak_joint_torque_nm=np.max(np.abs(torques),axis=0).tolist(),
                median_peak_clearance_mm=float(np.median(all_peaks)) if all_peaks else 0.,
                p10_peak_clearance_mm=float(np.percentile(all_peaks,10)) if all_peaks else 0.,
                per_leg_median_clearance_mm=[float(np.median(v)) if v else 0 for v in per_leg],
                mid_swing_near_ground_fraction=near/max(1,mid))

if __name__=='__main__':
    cases=[(.007,1.,.065,.64,18.)]
    cases += [(h,p,s,.64,18.) for h in (.012,.015,.020) for p in (1.,1.2,1.4) for s in (.05,.065)]
    rows=[]
    with ProcessPoolExecutor(max_workers=4) as pool:
        for row in pool.map(run,cases):
            rows.append(row);print(json.dumps(row),flush=True)
            Path(__file__).with_name('raised_level_candidates.json').write_text(json.dumps(rows,indent=2)+'\n')
