"""Identical-plant OFF/ON experiment; truth is recorded only for evaluation."""

# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[4]))

from simulation.mujoco.paths import REPO_ROOT, SIM_ROOT, RESULTS_ROOT
from pathlib import Path
import argparse
import csv
import json
import time
import numpy as np
import mujoco
from simulation.mujoco.runtime.cad_physics import Simulation, foot_clearance
from simulation.mujoco.runtime.virtual_robot import RobotController
from simulation.mujoco.scripts.visualization.run_calculated_placement import parameters

ROOT=REPO_ROOT

def metrics(rows,start,end):
    selected=[r for r in rows if start<=r['t']<end]
    if not selected:return dict(samples=0)
    a=lambda key:np.asarray([r[key] for r in selected])
    angle=a('roll_pitch');gyro=a('gyro_truth')[:,:2]
    return dict(samples=len(selected),start_s=start,end_s=end,
        roll_pitch_pp_deg=np.ptp(angle,axis=0).tolist(),
        roll_pitch_rms_deg=np.sqrt(np.mean(angle**2,axis=0)).tolist(),
        roll_pitch_max_deg=np.abs(angle).max(axis=0).tolist(),
        gyro_xy_rms_deg_s=np.degrees(np.sqrt(np.mean(gyro**2,axis=0))).tolist(),
        forward_m=float(a('com')[-1,0]-a('com')[0,0]),
        tracking_max_deg=float(np.max(abs(a('command')-a('actual')))),
        correction_peak_mm=float(max(max(abs(x) for x in r['placement'].get('applied_dz',[0]*4)) for r in selected)*1000))

def run(enabled,destination,config=None,duration=30,linear=1.,yaw=0.):
    p=parameters()
    if config is not None:p['body_stabilizer']=config
    plant=Simulation(p);robot=RobotController(plant);robot.select_profile('attitudepd')
    robot.balance.enabled=False;robot.heading.enabled=False
    robot.body_stabilizer.enabled=enabled
    m,d=plant.model,plant.data;feet=[m.geom(l+'_foot').id for l in ('fl','fr','rl','rr')]
    floor=m.geom('floor').id;rows=[];fault=None;latencies=[]
    for i in range(round(duration/.02)):
        t=i*.02
        if i==500:robot.command(f'drive {round(linear*1000)} {round(yaw*1000)} 1',t)
        elif i>500 and i%10==0 and fault is None:robot.command(f'@D {i} {round(linear*1000)} {round(yaw*1000)}',t)
        before=time.perf_counter();robot.tick(t);latencies.append(time.perf_counter()-before)
        if fault is None and robot.safety!='ok':fault=dict(t=t,reason=robot.safety)
        force=np.zeros(4)
        for j,c in enumerate(d.contact):
            if floor not in (c.geom1,c.geom2):continue
            other=c.geom2 if c.geom1==floor else c.geom1
            if other in feet:
                value=np.zeros(6);mujoco.mj_contactForce(m,d,j,value);force[feet.index(other)]+=max(0,value[0])
        velocity=np.zeros(6);mujoco.mj_objectVelocity(m,d,mujoco.mjtObj.mjOBJ_XBODY,m.body('robot').id,velocity,1)
        state=plant.row();messages=robot.drain().decode(errors='replace')
        rows.append(dict(t=t,physics_time_s=float(d.time),phase=getattr(robot,'nominal_phase',robot.phase),
            qpos=d.qpos.tolist(),qvel=d.qvel.tolist(),ctrl=d.ctrl.tolist(),
            roll=state['roll_deg'],pitch=state['pitch_deg'],roll_pitch=[state['roll_deg'],state['pitch_deg']],
            gyro_truth=velocity[:3].tolist(),com=state['com_m'],force=force.tolist(),
            clearance=[foot_clearance(m,d,g)*1000 for g in feet],command=robot.command_target.tolist(),
            nominal=robot.target.tolist(),actual=np.degrees(d.qpos[plant.q]).tolist(),
            torque=robot.torque,safety=robot.safety,pose=robot.pose,imu=robot.imu_reading,
            placement=robot.body_stabilizer.diagnostic.copy(),messages=messages))
    observed_end=fault['t'] if fault else duration
    summary=dict(mode='body-pd-'+('on' if enabled else 'off'),fault=fault,duration_s=duration,
        requested_forward_s=duration-10,mass_kg=float(m.body_mass.sum()),
        observed=metrics(rows,10,observed_end),host_simulation_step_p95_ms=float(np.percentile(latencies,95)*1000),
        validation_passed=False,validation_note='OFF/ON comparison; physical robot not tested')
    result=dict(summary=summary,parameters=p,records=rows,
        config=json.loads((ROOT/'config/body_stabilization.json').read_text()) if config is None else config,
        gait=dict(period_s=1.35,duty=.52,stride_m=.08,lift_m=.024,profile='attitudepd',base='centerpivot',linear=linear,yaw=yaw),
        feedback='Only delayed/quantized Euler + separately delayed body gyro; contact forces/truth are evaluation only',
        recording='Physical qpos/qvel 50Hz; no target substitution')
    destination.parent.mkdir(parents=True,exist_ok=True);destination.write_text(json.dumps(result,indent=2))
    fields=['time','sample_time_ms','status','roll','pitch','gx','gy','filtered_roll','filtered_pitch','filtered_gx','filtered_gy',
            'roll_error','pitch_error','roll_u','pitch_u','fl_z','fr_z','rl_z','rr_z','fl_weight','fr_weight','rl_weight','rr_weight',
            'axis_clamp','foot_clamp','slew_limited','ik_failed']
    with destination.with_suffix('.csv').open('w',newline='') as f:
        writer=csv.writer(f);writer.writerow(fields)
        for row in rows:
            diag=row['placement'];writer.writerow([row['t'],diag.get('sample_timestamp_ms'),diag.get('status'),
                *diag.get('raw',[0]*4),*diag.get('filtered',[0]*4),*diag.get('error',[0]*2),*diag.get('u_applied',[0]*2),
                *diag.get('applied_dz',[0]*4),*diag.get('stance_weights',[0]*4),diag.get('axis_clamp_mask'),
                diag.get('foot_clamp_mask'),diag.get('slew_limited'),diag.get('ik_failed')])
    print(json.dumps(summary),flush=True)
    return result

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('mode',choices=['on','off'])
    parser.add_argument('--output',type=Path,required=True);parser.add_argument('--config',type=Path)
    parser.add_argument('--duration',type=float,default=30);parser.add_argument('--linear',type=float,default=1)
    parser.add_argument('--yaw',type=float,default=0);args=parser.parse_args()
    if not 10<args.duration<=120:parser.error('duration must be >10 and <=120 s')
    run(args.mode=='on',args.output,json.loads(args.config.read_text()) if args.config else None,args.duration,args.linear,args.yaw)
