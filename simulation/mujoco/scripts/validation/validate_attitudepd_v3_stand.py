"""Estimated MuJoCo Stand -> gait -> Stand validation. No hardware transport."""
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[4]))
import argparse
import json
import numpy as np
from simulation.mujoco.runtime.cad_physics import Simulation
from simulation.mujoco.runtime.virtual_robot import RobotController
from simulation.mujoco.scripts.visualization.run_calculated_placement import parameters


def run(output):
    results=[]
    for name,linear,yaw in [('forward',1000,0),('reverse',-1000,0),('left',0,500),('right',0,-500)]:
        robot=RobotController(Simulation(parameters()))
        robot.select_profile("attitudepd_v3")
        robot.body_stabilizer.enabled=False  # Same axis-unverified PD state as V2 hardware trial.
        robot.heading.enabled=False
        records=[];drive_at=None;stop_at=None;fault=None
        robot.command('stand',0.)
        for i in range(1100):
            t=i*.02
            if drive_at is None and t>=7 and robot.transition is None:
                robot.command(f'drive {linear} {yaw} 1',t);drive_at=t
            elif drive_at is not None and stop_at is None:
                if t-drive_at>=4.:
                    robot.command('@S 100000',t);stop_at=t
                elif i%10==0:
                    robot.command(f'@D {i+2} {linear} {yaw}',t)
            robot.tick(t)
            row=robot.plant.row()
            records.append(dict(t=t,command=robot.command_target.tolist(),
                actual=np.degrees(robot.plant.data.qpos[robot.plant.q]).tolist(),
                roll=row['roll_deg'],pitch=row['pitch_deg'],pose=robot.pose,
                motion=robot.motion is not None,transition=robot.transition is not None,
                safety=robot.safety,pd=robot.body_stabilizer.diagnostic.copy(),messages=robot.drain().decode(errors='replace')))
            if robot.safety!='ok':fault=dict(t=t,reason=robot.safety);break
            if stop_at is not None and t-stop_at>=2 and robot.motion is None and robot.transition is None:break
        final_error=float(np.max(abs(robot.command_target-robot.stand_target)))
        summary=dict(case=name,profile=robot.profile,backend='MuJoCo estimated physics',
            physical_robot_test=False,PD='off',drive_at=drive_at,stop_at=stop_at,fault=fault,
            final_pose=robot.pose,final_target_b_error_deg=final_error,
            final_actual_b_error_deg=float(np.max(abs(np.degrees(robot.plant.data.qpos[robot.plant.q])-robot.stand_target))),
            max_roll_deg=max(abs(r['roll']) for r in records),max_pitch_deg=max(abs(r['pitch']) for r in records),
            command_flow_pass=fault is None and stop_at is not None and robot.pose=='stand' and final_error<1.e-5,
            standby_b_deg=robot.stand_target.tolist())
        results.append(dict(summary=summary,records=records))
        print(json.dumps(summary),flush=True)
        robot.body_stabilizer.close()
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(results,indent=2)+'\n')

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    run(p.parse_args().output)
