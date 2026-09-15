"""V6.2 rear-reach geometry and protected physical walk/STOP observations."""
if __package__ in (None, ''):
    import sys
    from pathlib import Path
    sys.path.insert(0,str(Path(__file__).resolve().parents[4]))

import argparse
import json
from pathlib import Path
import numpy as np
import mujoco
from simulation.mujoco.runtime.cad_physics import Simulation,foot_clearance
from simulation.mujoco.runtime.virtual_robot import RobotController,load_parameters,parse_args
from simulation.mujoco.runtime.s_native_gait import SNativeGait,PROFILES,NAME
from simulation.mujoco.runtime.standing_pose import SoleKinematics


def geometry(plant,profile):
    gait=SNativeGait(plant.model,plant.stand_target,PROFILES[profile])
    rows=[]
    for phase in np.linspace(.5,3.5,151):
        q=gait.targets(phase%1,1.,1.,0.)
        if phase<2.5:continue
        for i,leg in enumerate(('fl','fr','rl','rr')):
            hip=gait.kin.data.xanchor[plant.model.joint(leg+'_j2').id]
            knee=gait.kin.data.xanchor[plant.model.joint(leg+'_j3').id]
            foot=gait.kin.foot(i)
            upper=hip-knee;lower=foot-knee
            rows.append(dict(phase=float(phase%1),leg=leg,
                x_mm=float((foot[0]-gait.origin[i,0])*1000),
                z_mm=float((foot[2]-gait.origin[i,2])*1000),
                j2_deg=float(q[3*i+1]),j3_deg=float(q[3*i+2]),
                lower_from_ground_deg=float(np.degrees(np.arctan2(abs(lower[2]),abs(lower[0])))),
                knee_inner_deg=float(np.degrees(np.arccos(np.clip(np.dot(upper,lower)/np.linalg.norm(upper)/np.linalg.norm(lower),-1,1))))))
    return dict(profile=profile,parameters=PROFILES[profile],
        rear_endpoints={leg:min((r for r in rows if r['leg']==leg),key=lambda r:r['x_mm']) for leg in ('rl','rr')},
        front_endpoints={leg:max((r for r in rows if r['leg']==leg),key=lambda r:r['x_mm']) for leg in ('rl','rr')}),rows


def trial(command=(1000,0),walk_seconds=8.,profile='s_native_v6_2',observe_recovery=False):
    plant=Simulation(load_parameters(parse_args([])))
    robot=RobotController(plant);robot.select_profile(profile)
    stop_tick=100+round(walk_seconds/.02)
    rows=[];floor=plant.model.geom('floor').id
    kin=SoleKinematics(plant.model,plant.stand_target) if observe_recovery else None
    for i in range(stop_tick+200):
        t=i*.02
        if i==100:robot.command(f'drive {command[0]} {command[1]} 1',t)
        elif i==stop_tick:robot.command('@S 1000',t)
        elif 100<i<stop_tick and i%10==0:robot.command(f'@D {i} {command[0]} {command[1]}',t)
        robot.tick(t);state=plant.row()
        rotation=plant.data.xmat[plant.model.body('robot').id].reshape(3,3)
        contacts=[]
        for c in plant.data.contact:
            if floor not in (c.geom1,c.geom2):
                contacts.append([plant.model.geom(c.geom1).name,plant.model.geom(c.geom2).name])
        rows.append(dict(time_s=t,roll_deg=state['roll_deg'],pitch_deg=state['pitch_deg'],
            yaw_deg=float(np.degrees(np.arctan2(rotation[1,0],rotation[0,0]))),
            position_m=plant.data.qpos[:3].tolist(),
            target=robot.command_target.tolist(),actual=np.degrees(plant.data.qpos[plant.q]).tolist(),
            clearance_mm=[foot_clearance(plant.model,plant.data,plant.model.geom(l+'_foot').id)*1000 for l in ('fl','fr','rl','rr')],
            non_floor_contacts=contacts,safety=robot.safety,reply=robot.drain().decode()))
        if observe_recovery:
            kin.set_angles(np.degrees(plant.data.qpos[plant.q]))
            load=np.zeros(4)
            for index,c in enumerate(plant.data.contact):
                if floor not in (c.geom1,c.geom2):continue
                other=c.geom2 if c.geom1==floor else c.geom1
                for leg,foot in enumerate(kin.feet):
                    if other==foot:
                        force=np.zeros(6);mujoco.mj_contactForce(plant.model,plant.data,index,force)
                        load[leg]+=max(0.,force[0])
            rows[-1].update(phase=float(getattr(robot,'nominal_phase',0)),
                force_n=load.tolist(),foot_body_m=[kin.foot(j).tolist() for j in range(4)])
    yaw=np.degrees(np.unwrap(np.radians([r['yaw_deg'] for r in rows])))
    faults=[r for r in rows if r['safety']!='ok']
    summary=dict(profile=profile,command=list(command),walk_seconds=walk_seconds,
        completed=not robot.motion and not robot.transition,
        safety=robot.safety,first_fault_s=faults[0]['time_s'] if faults else None,
        max_walk_tilt_deg=max(max(abs(r['roll_deg']),abs(r['pitch_deg'])) for r in rows[100:stop_tick]),
        max_stop_tilt_deg=max(max(abs(r['roll_deg']),abs(r['pitch_deg'])) for r in rows[stop_tick:]),
        walk_yaw_deg=float(yaw[stop_tick-1]-yaw[99]),
        walk_displacement_m=(np.array(rows[stop_tick-1]['position_m'])-rows[99]['position_m']).tolist(),
        final_s_target_error_deg=float(np.max(abs(robot.command_target-robot.stand_target))),
        final_s_actual_error_deg=float(np.max(abs(np.degrees(plant.data.qpos[plant.q])-robot.stand_target))),
        non_floor_contact_frames=sum(bool(r['non_floor_contacts']) for r in rows),
        stopped_s=next((r['time_s'] for r in rows if '$SPOTDRIVE stopped' in r['reply']),None))
    return summary,rows


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--profile',choices=tuple(PROFILES),default=NAME)
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    plant=Simulation(load_parameters(parse_args([])))
    for profile in dict.fromkeys(('s_native_v6_1','s_native_v6_2',args.profile)):
        summary,rows=geometry(plant,profile)
        (args.output/f'{profile}-geometry.json').write_text(json.dumps(dict(summary=summary,rows=rows),indent=2))
        print(json.dumps(summary),flush=True)
    results=[]
    for command,duration in [((473,0),8.),((1000,0),8.),((1000,0),.3),((1000,0),1.2),
                             ((-600,0),8.),((0,500),8.),((0,-500),8.),((600,250),8.)]:
        summary,rows=trial(command,duration,profile=args.profile,observe_recovery=True);results.append(summary)
        (args.output/f'trial-{command[0]}-{command[1]}-{duration}.json').write_text(json.dumps(dict(summary=summary,rows=rows)))
        print(json.dumps(summary),flush=True)
    (args.output/'summary.json').write_text(json.dumps(results,indent=2))
    for s in results:
        assert s['safety']=='ok' and s['completed'] and s['first_fault_s'] is None,s
        assert s['final_s_target_error_deg']<.01 and s['final_s_actual_error_deg']<1.1,s


if __name__=='__main__':main()
