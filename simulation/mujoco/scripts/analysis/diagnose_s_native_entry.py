"""Offline V5 entry: separate joint tracking from body tilt at touchdown.

Counterfactual heights are diagnostics, never fed back into the controller.
Support-off is an isolation experiment, not a deployable gait.
"""
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


def run(support=True, isolate_j1=False):
    plant = Simulation(load_parameters(parse_args(['--allow-fall'])))
    robot = RobotController(plant)
    robot.select_profile('s_native_v5')
    robot.profiles['s_native_v5']['support_transfer'] = support
    model, data = plant.model, plant.data
    probe = mujoco.MjData(model)
    feet = [model.geom(leg+'_foot').id for leg in ('fl', 'fr', 'rl', 'rr')]
    floor = model.geom('floor').id
    rows = []

    def heights(qpos):
        probe.qpos[:] = qpos
        mujoco.mj_forward(model, probe)
        return np.array([foot_clearance(model, probe, f)*1000 for f in feet])

    for tick in range(300):
        now = tick*.02
        if tick == 100:
            robot.command('drive 1000 0 1', now)
            if isolate_j1:
                # Deliberately violates the entry specification to isolate its
                # physical effect. Never register this as an accepted profile.
                robot.s_native_gait.normal_adduction[:] = 0
                robot.s_native_gait.walking_lateral_offset[:] = 0
        elif tick > 100 and tick % 10 == 0:
            robot.command(f'@D {tick} 1000 0', now)
        robot.tick(now)
        actual = heights(data.qpos)
        flat = data.qpos.copy()
        flat[3:7] = [1, 0, 0, 0]
        flat_actual = heights(flat)
        flat[plant.q] = np.radians(robot.command_target)
        flat_command = heights(flat)
        force = np.zeros(4)
        # Contact index is its position in data.contact, not a geom/body ID.
        for j in range(data.ncon):
            c = data.contact[j]
            if floor in (c.geom1, c.geom2):
                other = c.geom2 if c.geom1 == floor else c.geom1
                if other in feet:
                    wrench = np.zeros(6)
                    mujoco.mj_contactForce(model, data, j, wrench)
                    force[feet.index(other)] += max(0., wrench[0])
        state = plant.row()
        rows.append(dict(t=now, roll=state['roll_deg'], pitch=state['pitch_deg'],
            actual_height_mm=actual.tolist(), flat_actual_mm=flat_actual.tolist(),
            flat_command_mm=flat_command.tolist(), load_n=force.tolist(),
            target_deg=robot.command_target.tolist(), actual_deg=state['actual_deg']))
        robot.drain()
    touchdown = {}
    for leg, i in [('FR', 1), ('RL', 2)]:
        lifted = False
        for row in rows[100:150]:
            if row['actual_height_mm'][i] > 2 and row['load_n'][i] < .1:
                lifted = True
            if lifted and row['load_n'][i] > .5:
                touchdown[leg] = row['t']
                break
    return dict(support_transfer=support, isolate_j1=isolate_j1, touchdown_s=touchdown,
        toppled_s=next((r['t'] for r in rows[100:] if max(abs(r['roll']), abs(r['pitch'])) >= 90), None),
        rows=rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    for name, support, isolate in [('v5', True, False), ('support-off', False, False), ('j1-fixed-isolation', True, True)]:
        result = run(support, isolate)
        (args.output/(name+'.json')).write_text(json.dumps(result, indent=2), encoding='utf-8')
        print(name, {k: v for k, v in result.items() if k != 'rows'}, flush=True)


if __name__ == '__main__':
    main()
