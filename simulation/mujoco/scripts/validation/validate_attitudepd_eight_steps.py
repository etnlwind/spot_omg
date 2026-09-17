"""Eight planned diagonal placements in MuJoCo; never accesses physical hardware."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[4]))
import argparse
import json
import math
import numpy as np
import mujoco
from simulation.mujoco.runtime.cad_physics import Simulation, foot_clearance
from simulation.mujoco.runtime.virtual_robot import RobotController
from simulation.mujoco.scripts.visualization.run_calculated_placement import parameters


def run(output):
    plant = Simulation(parameters())
    robot = RobotController(plant)
    robot.select_profile('attitudepd_v2')
    robot.body_stabilizer.enabled = False  # match current hardware axis-unverified PD
    robot.heading.enabled = False
    feet = [plant.model.geom(leg+'_foot').id for leg in ('fl','fr','rl','rr')]
    floor = plant.model.geom('floor').id
    records, placements = [], []
    previous_phase = None
    accumulated = 0.
    stop_sent = False
    fault = None
    for i in range(1500):
        t = i*.02
        if i == 500:
            robot.command('drive 1000 0 1', t)
        elif i > 500 and i % 10 == 0 and not stop_sent and fault is None:
            robot.command(f'@D {i} 1000 0', t)
        robot.tick(t)
        if i >= 500 and robot.motion is not None and robot.transition is None and not stop_sent:
            phase = robot.phase
            if previous_phase is not None:
                delta = (phase-previous_phase) % 1.
                if delta > .1:
                    raise RuntimeError('Unexpected phase jump; refuse to invent step count')
                accumulated += delta
            else:
                accumulated = phase
            previous_phase = phase
            count = min(8, int(math.floor(accumulated*2+1.e-6)))
            while len(placements) < count:
                placements.append(dict(step=len(placements)+1, time_s=t,
                    pair='FR/RL' if len(placements)%2 == 0 else 'FL/RR'))
            if count == 8:
                robot.command('@S 100000', t)
                stop_sent = True
        force = [0.]*4
        for contact_index, contact in enumerate(plant.data.contact):
            if floor not in (contact.geom1, contact.geom2):
                continue
            other = contact.geom2 if contact.geom1 == floor else contact.geom1
            if other in feet:
                v = np.zeros(6)
                mujoco.mj_contactForce(plant.model, plant.data, contact_index, v)
                force[feet.index(other)] += max(0., float(v[0]))
        row=plant.row()
        records.append(dict(time_s=t, planned_steps=len(placements),phase=robot.phase,
            clearance_mm=[foot_clearance(plant.model,plant.data,g)*1000 for g in feet],
            roll_deg=row['roll_deg'],pitch_deg=row['pitch_deg'],force_n=force,
            safety=robot.safety,pose=robot.pose,messages=robot.drain().decode(errors='replace')))
        if robot.safety != 'ok':
            fault=dict(time_s=t,reason=robot.safety)
            break
        if stop_sent and robot.motion is None and robot.transition is None:
            break
    summary=dict(profile='attitudepd_v2',backend='MuJoCo estimated physics',
        requested_planned_steps=8,planned_steps=len(placements),stop_requested=stop_sent,
        stopped=stop_sent and robot.motion is None and robot.transition is None,
        fault=fault,final_pose=robot.pose,torque=robot.torque,PD='off',
        step_definition='One diagonal-pair nominal swing completion; two per cycle. Actual contacts recorded separately.',
        physical_robot_test=False)
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(dict(summary=summary,placements=placements,records=records),indent=2)+'\n')
    print(json.dumps(summary,indent=2))

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    run(parser.parse_args().output)
