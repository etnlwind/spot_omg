"""Offline V6.2.1 swing contact audit; frozen-pose geometry is not a dynamics ablation."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[4]))
import argparse
import json
import mujoco
import numpy as np
from simulation.mujoco.runtime.cad_physics import Simulation, foot_clearance
from simulation.mujoco.runtime.virtual_robot import RobotController, load_parameters, parse_args


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    plant = Simulation(load_parameters(parse_args([])))
    robot = RobotController(plant)
    robot.profile = 's_native_v6_2_1'
    m, d = plant.model, plant.data
    scratch = mujoco.MjData(m)
    legs = ('FL', 'FR', 'RL', 'RR')
    feet = [m.geom(l.lower()+'_foot').id for l in legs]
    floor = m.geom('floor').id
    records = []
    def heights(qpos):
        scratch.qpos[:] = qpos
        mujoco.mj_forward(m, scratch)
        return np.array([foot_clearance(m, scratch, f)*1000 for f in feet])
    for tick in range(700):
        t = tick*.02
        if tick == 100:
            robot.command('drive 1000 0 1', t)
        elif tick == 500:
            robot.command('@S 500', t)
        elif 100 < tick < 500 and tick % 10 == 0:
            robot.command(f'@D {tick} 1000 0', t)
        robot.tick(t)
        state = plant.row()
        actual = heights(d.qpos)
        ideal_q = d.qpos.copy()
        ideal_q[plant.q] = np.radians(robot.command_target)
        ideal = heights(ideal_q)
        level_q = ideal_q.copy()
        level_q[3:7] = [1, 0, 0, 0]
        level = heights(level_q)
        load = np.zeros(4)
        for c in range(d.ncon):
            con = d.contact[c]
            if floor not in (con.geom1, con.geom2):
                continue
            other = con.geom2 if con.geom1 == floor else con.geom1
            if other in feet:
                force = np.zeros(6)
                mujoco.mj_contactForce(m, d, c, force)
                load[feet.index(other)] += max(0., force[0])
        phase = float(getattr(robot, 'nominal_phase', 0.))
        q = (phase+np.array([.5, 0., 0., .5])) % 1
        progress = (q-.5)*2
        records.append(dict(time_s=t, phase=phase, swing_progress=progress.tolist(),
            clearance_mm=actual.tolist(), normal_force_n=load.tolist(),
            ideal_tracking_at_actual_body_mm=ideal.tolist(),
            ideal_tracking_at_level_body_same_height_mm=level.tolist(),
            tracking_height_delta_mm=(actual-ideal).tolist(),
            attitude_height_delta_mm=(ideal-level).tolist(),
            target_deg=robot.command_target.tolist(), actual_deg=state['actual_deg'],
            roll_deg=state['roll_deg'], pitch_deg=state['pitch_deg'],
            body_z_m=float(d.qpos[2]), safety=robot.safety))
        if tick % 100 == 0:
            print(f'replay {t:.1f}s', flush=True)
    summary = {'scope':'Offline Python dynamics, estimated physics; no robot connection. Frozen-pose substitutions do not simulate corrected dynamics.',
               'mujoco_version':mujoco.__version__, 'window':'4 <= video time <= 10; 0.2 <= swing progress <= 0.9; force > 0.5 N',
               'legs':{}}
    for i, leg in enumerate(legs):
        selected=[r for r in records if 4 <= r['time_s'] <= 10 and .2 <= r['swing_progress'][i] <= .9]
        loaded=[r for r in selected if r['normal_force_n'][i] > .5]
        worst=min(selected,key=lambda r:r['clearance_mm'][i])
        summary['legs'][leg]=dict(samples=len(selected),loaded_samples=len(loaded),
            minimum_clearance_mm=worst['clearance_mm'][i],
            worst_sample=worst,
            loaded_median_tracking_height_delta_mm=float(np.median([r['tracking_height_delta_mm'][i] for r in loaded])) if loaded else None,
            loaded_median_attitude_height_delta_mm=float(np.median([r['attitude_height_delta_mm'][i] for r in loaded])) if loaded else None,
            loaded_with_ideal_tracking_below_ground=sum(r['ideal_tracking_at_actual_body_mm'][i]<=0 for r in loaded))
    args.output.mkdir(parents=True,exist_ok=True)
    (args.output/'trajectory.json').write_text(json.dumps(records,indent=2)+'\n')
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    print(json.dumps({leg:{k:v for k,v in s.items() if k!='worst_sample'} for leg,s in summary['legs'].items()},indent=2))

if __name__ == '__main__':
    main()
