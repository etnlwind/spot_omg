"""V5/V6 support-exchange comparison using unmodified estimated physics."""
if __package__ in (None, ''):
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[4]))

import argparse
import json
from pathlib import Path
import numpy as np
from simulation.mujoco.runtime.cad_physics import Simulation, foot_clearance
from simulation.mujoco.runtime.virtual_robot import RobotController, parse_args, load_parameters
from simulation.mujoco.runtime.s_native_gait import PROFILES


def trial(name, command, stop_after=None):
    plant = Simulation(load_parameters(parse_args(['--allow-fall'])))
    robot = RobotController(plant)
    robot.select_profile(name)
    feet = [plant.model.geom(l+'_foot').id for l in ('fl','fr','rl','rr')]
    rows = []
    for i in range(500):
        t = i*.02
        if i == 100:
            start = plant.data.qpos[:3].copy()
            robot.command(f'drive {command} 0 1', t)
        elif i > 100 and i % 10 == 0 and (stop_after is None or t < 2+stop_after):
            robot.command(f'@D {i} {command} 0', t)
        if stop_after is not None and i == 100+round(stop_after/.02):
            robot.command('@S 1000', t)
        robot.tick(t)
        state = plant.row()
        rows.append(dict(time_s=t, roll=state['roll_deg'], pitch=state['pitch_deg'],
            height_mm=[foot_clearance(plant.model,plant.data,f)*1000 for f in feet],
            target=robot.command_target.tolist(), actual=state['actual_deg'],
            safety=robot.safety, moving=bool(robot.motion), transition=bool(robot.transition)))
        robot.drain()
    active = rows[100:]
    report = dict(profile=name, parameters=robot.profiles[name], command=command, stop_after_s=stop_after,
        max_tilt_deg=max(max(abs(r['roll']),abs(r['pitch'])) for r in active),
        toppled_s=next((r['time_s'] for r in active if max(abs(r['roll']),abs(r['pitch']))>=90),None),
        displacement_m=(plant.data.qpos[:3]-start).tolist(),
        air_time_above_2mm_s=[sum(r['height_mm'][i]>2 for r in active)*.02 for i in range(4)],
        stopped=not robot.motion and not robot.transition,
        final_target_error_deg=float(np.max(abs(robot.command_target-robot.stand_target))),
        final_actual_error_deg=float(np.max(abs(np.degrees(plant.data.qpos[plant.q])-robot.stand_target))))
    return report, rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--profiles', nargs='+', choices=list(PROFILES), default=['s_native_v5', 's_native_v6'])
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    reports = []
    cases = [(name, command, stop) for name in args.profiles
             for command, stop in ([(1000,None)] if name == 's_native_v5' else
                 [(300,None),(600,None),(1000,None),(1000,.3),(1000,1.2)])]
    for name, command, stop in cases:
        report, rows = trial(name,command,stop)
        reports.append(report)
        label=f'{name}-{command}-stop-{stop}'
        (args.output/(label+'.json')).write_text(json.dumps(dict(summary=report,rows=rows),indent=2),encoding='utf-8')
        print(json.dumps(report),flush=True)
    (args.output/'summary.json').write_text(json.dumps(reports,indent=2),encoding='utf-8')


if __name__ == '__main__':
    main()
