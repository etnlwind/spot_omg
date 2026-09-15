"""Measure two-step STOP placement with physical cushion contacts enabled."""
if __package__ in (None, ''):
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[4]))

import argparse
import json
from pathlib import Path
import mujoco
import numpy as np
from simulation.mujoco.runtime.cad_physics import Simulation, foot_clearance
from simulation.mujoco.runtime.virtual_robot import RobotController, load_parameters, parse_args


def replay(command=473, walk_seconds=8., profile='s_native_v6_1'):
    plant=Simulation(load_parameters(parse_args([])))
    robot=RobotController(plant)
    robot.select_profile(profile)
    stop_tick=100+round(walk_seconds/.02)
    floor=plant.model.geom('floor').id
    feet=[]
    rows=[]
    previous=None
    for tick in range(stop_tick+150):
        now=tick*.02
        if tick==100:
            robot.command(f'drive {command} 0 1',now)
            feet=robot.s_native_gait.kin.feet
        elif tick==stop_tick:
            robot.command('@S 1000',now)
        elif 100<tick<stop_tick and tick%10==0:
            robot.command(f'@D {tick} {command} 0',now)
        robot.tick(now)
        reply=robot.drain().decode()
        target=robot.command_target[::3].copy()
        if tick>=stop_tick:
            load=np.zeros(4)
            for contact_index in range(plant.data.ncon):
                c=plant.data.contact[contact_index]
                if floor not in (c.geom1,c.geom2):continue
                other=c.geom2 if c.geom1==floor else c.geom1
                for leg,foot in enumerate(feet):
                    if other==foot:
                        force=np.zeros(6)
                        mujoco.mj_contactForce(plant.model,plant.data,contact_index,force)
                        load[leg]+=max(0.,force[0])
            state=plant.row()
            rows.append(dict(stop_s=(tick-stop_tick)*.02,
                position_m=plant.data.qpos[:3].tolist(),
                roll_deg=state['roll_deg'],pitch_deg=state['pitch_deg'],
                target_j1_deg=target.tolist(),
                actual_j1_deg=np.degrees(plant.data.qpos[plant.q])[::3].tolist(),
                changing_j1=(abs(target-previous)>.05).tolist(),
                force_n=load.tolist(),
                clearance_mm=[foot_clearance(plant.model,plant.data,f)*1000 for f in feet],
                safety=robot.safety,reply=reply))
        previous=target
    moving=[(r,i) for r in rows for i in range(4) if r['changing_j1'][i]]
    summary=dict(command=command,walk_seconds=walk_seconds,
        max_force_during_j1_move_n=max((r['force_n'][i] for r,i in moving),default=0.),
        min_clearance_during_j1_move_mm=min((r['clearance_mm'][i] for r,i in moving),default=0.),
        max_tilt_deg=max(max(abs(r['roll_deg']),abs(r['pitch_deg'])) for r in rows),
        displacement_m=(np.array(rows[-1]['position_m'])-rows[0]['position_m']).tolist(),
        actual_s_error_deg=float(np.max(abs(np.degrees(plant.data.qpos[plant.q])-robot.stand_target))),
        target_s_error_deg=float(np.max(abs(robot.command_target-robot.stand_target))),
        complete=not robot.motion and not robot.transition,
        safety=robot.safety)
    # Report the stricter contact criterion separately from target completion.
    # No force sensor is fed to this open-loop foot-placement controller.
    summary['unloaded_during_j1_command']=summary['max_force_during_j1_move_n']<.5
    return summary,rows


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    summaries=[]
    for command in (473,1000):
        for duration in (.04,.3,1.2,8.):
            summary,rows=replay(command,duration)
            summaries.append(summary)
            (args.output/f'physical-{command}-{duration}.json').write_text(
                json.dumps(dict(summary=summary,rows=rows),indent=2),encoding='utf-8')
            print(json.dumps(summary),flush=True)
    (args.output/'summary.json').write_text(json.dumps(summaries,indent=2),encoding='utf-8')
    for s in summaries:
        assert s['safety']=='ok' and s['complete'],s
        assert s['target_s_error_deg']<.01 and s['actual_s_error_deg']<1.1,s


if __name__=='__main__':main()
