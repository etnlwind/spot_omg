"""Locate loaded forward foot slip in the recorded post-push experiment.

Re-run the same deterministic physics and check joint states against the video
log before using contact-point velocities. Rolling contact with zero material
slip is not counted as dragging. This is offline analysis, not robot control.
"""
import argparse
import csv
import gzip
import json
import sys
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[4]))
import mujoco
import numpy as np
from simulation.mujoco.runtime.cad_physics import Simulation
from simulation.mujoco.runtime.virtual_robot import RobotController,load_parameters,parse_args
from simulation.mujoco.scripts.analysis.analyze_knee_liftoff import tuck_experiment
from simulation.mujoco.scripts.analysis.analyze_j1_steady_hold import j1_experiment


def analyze(directory,config,source_log=None,walk_seconds=8.,j1_mode=None):
    if source_log:
        with gzip.open(source_log,'rt',encoding='utf-8') as f:source=json.load(f)
        recorded=[dict(time_s=r['time_s'],phase=r['phase'],
            j1_actual_deg=r['actual'][::3],j2_actual_deg=r['actual'][1::3],j3_actual_deg=r['actual'][2::3],
            body_relative_x_from_s_mm=(np.array(r['actual_feet_m'])[:,0]*1000).tolist()) for r in source['rows']]
    else:recorded=json.loads((directory/'capture/trajectory.json').read_text())
    config=json.loads(config.read_text())
    parameters=load_parameters(parse_args([]))
    parameters['pack_open_circuit_voltage']=config['pack_open_circuit_voltage']
    plant=Simulation(parameters);robot=RobotController(plant)
    robot.select_profile('s_native_v6_2_6')
    model,data=plant.model,plant.data
    legs=('FL','FR','RL','RR')
    feet=[model.geom(leg.lower()+'_foot').id for leg in legs]
    floor=model.geom('floor').id
    rows=[];max_replay_error=0.
    mode=j1_mode if j1_mode is not None else config.get('j1_mode','baseline')
    with tuck_experiment(**config['trajectory']),j1_experiment(mode):
        for index,reference in enumerate(recorded):
            t=reference['time_s']
            if t>=2+walk_seconds:break
            if index==100:robot.command('drive 1000 0 1',t)
            elif index>100 and index%10==0:robot.command(f'@D {index} 1000 0',t)
            robot.tick(t)
            actual=np.degrees(data.qpos[plant.q])
            expected=np.array([reference[f'j{joint}_actual_deg'] for joint in (1,2,3)]).T.reshape(12)
            max_replay_error=max(max_replay_error,float(np.max(abs(actual-expected))))
            rotation=data.xmat[model.body('robot').id].reshape(3,3)
            forward=rotation[:2,0];forward=forward/np.linalg.norm(forward)
            load=np.zeros(4);weighted_forward=np.zeros(4);weighted_speed=np.zeros(4)
            for n,contact in enumerate(data.contact):
                if floor not in (contact.geom1,contact.geom2):continue
                other=contact.geom2 if contact.geom1==floor else contact.geom1
                if other not in feet:continue
                leg=feet.index(other)
                force=np.zeros(6);mujoco.mj_contactForce(model,data,n,force)
                weight=max(0.,force[0])
                jp=np.zeros((3,model.nv));jr=np.zeros_like(jp)
                mujoco.mj_jac(model,data,jp,jr,contact.pos,int(model.geom_bodyid[other]))
                velocity=(jp@data.qvel)[:2]*1000
                load[leg]+=weight
                weighted_forward[leg]+=weight*float(velocity@forward)
                weighted_speed[leg]+=weight*float(np.linalg.norm(velocity))
            speed=np.divide(weighted_speed,load,out=np.zeros(4),where=load>0)
            forward_speed=np.divide(weighted_forward,load,out=np.zeros(4),where=load>0)
            body_x_speed=(np.array(reference['body_relative_x_from_s_mm'])-
                np.array(recorded[max(0,index-1)]['body_relative_x_from_s_mm']))/.02
            q=(reference['phase']+np.array([.5,0,0,.5]))%1
            mask=(q>.5)&(load>1)&(forward_speed>20)&(body_x_speed>20)&(t>=2)
            rows.append(dict(time_s=t,leg_phase=q.tolist(),normal_force_n=load.tolist(),
                world_forward_slip_mm_s=forward_speed.tolist(),world_slip_mm_s=speed.tolist(),
                body_forward_speed_mm_s=body_x_speed.tolist(),drag=mask.tolist()))
    assert max_replay_error<1e-9,max_replay_error
    events=[]
    for leg,name in enumerate(legs):
        indices=np.array([i for i,r in enumerate(rows) if r['drag'][leg]],dtype=int)
        groups=np.split(indices,np.where(np.diff(indices)>1)[0]+1)
        for group in groups:
            if not len(group):continue
            part=[rows[i] for i in group]
            events.append(dict(leg=name,start_s=round(part[0]['time_s'],2),
                end_s=round(part[-1]['time_s']+.02,2),duration_s=round(len(group)*.02,2),
                peak_force_n=max(r['normal_force_n'][leg] for r in part),
                peak_forward_slip_mm_s=max(r['world_forward_slip_mm_s'][leg] for r in part),
                forward_slip_mm=sum(r['world_forward_slip_mm_s'][leg]*.02 for r in part),
                swing_fraction_start=(part[0]['leg_phase'][leg]-.5)*2,
                swing_fraction_end=(part[-1]['leg_phase'][leg]-.5)*2))
    events.sort(key=lambda r:(r['start_s'],r['leg']))
    report=dict(time_basis='Simulation time including 2 seconds before drive. Half-speed player times are doubled.',
        walk_seconds=walk_seconds,j1_mode=mode,source_log=str(source_log) if source_log else None,
        sample_period_s=.02,video_frame_period_s=.04,max_replay_joint_error_deg=max_replay_error,
        definition='Scheduled recovery, normal load > 1 N, forward material contact slip > 20 mm/s, body-relative forward foot motion > 20 mm/s.',
        events=events,major_events=[r for r in events if r['duration_s']>=.08],
        total_seconds_per_leg={leg:round(sum(e['duration_s'] for e in events if e['leg']==leg),2) for leg in legs})
    output=directory/'drag-analysis';output.mkdir(parents=True,exist_ok=True)
    (output/'events.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    (output/'contact-velocity.json').write_text(json.dumps(rows)+'\n',encoding='utf-8')
    with (output/'events.csv').open('w',encoding='utf-8',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=['leg','start_s','end_s','duration_s',
            'peak_force_n','peak_forward_slip_mm_s','forward_slip_mm',
            'swing_fraction_start','swing_fraction_end'])
        writer.writeheader();writer.writerows(events)
    print(json.dumps({k:v for k,v in report.items() if k!='events'},indent=2))


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory',type=Path,required=True)
    parser.add_argument('--config',type=Path,default=Path('config/experiments/post_push_j2_j3_recovery.json'))
    parser.add_argument('--source-log',type=Path,help='Replay an analysis JSON.GZ instead of a capture log')
    parser.add_argument('--walk-seconds',type=float,default=8.)
    parser.add_argument('--j1-mode',choices=['baseline','without_lateral_transfer','fixed_j1'])
    args=parser.parse_args();analyze(args.directory,args.config,args.source_log,args.walk_seconds,args.j1_mode)
