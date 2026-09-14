"""Reproduce V5 stop cases and optional first-step CAD triangle clearance.

Offline estimated physics only. Requires requirements-stow.txt for --clearance.
No live connection or hardware transport is opened.
"""
if __package__ in (None, ""):
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[4]))

import argparse
import copy
import json
from pathlib import Path

import numpy as np
from simulation.mujoco.runtime.cad_physics import Simulation
from simulation.mujoco.runtime.s_native_gait import PROFILES, SNativeGait
from simulation.mujoco.runtime.virtual_robot import RobotController, load_parameters, parse_args


def stop_cases():
    reports = []
    for command, walk in ((1000, .3), (300, 1.), (600, 1.), (1000, 1.)):
        plant = Simulation(load_parameters(parse_args(['--allow-fall'])))
        robot = RobotController(plant)
        robot.profile = 's_native_v5'
        stop_tick = 100 + round(walk / .02)
        rows = []
        for tick in range(300):
            time = tick * .02
            if tick == 100:
                robot.command(f'drive {command} 0 1', time)
            elif 100 < tick < stop_tick and tick % 10 == 0:
                robot.command(f'@D {tick} {command} 0', time)
            if tick == stop_tick:
                robot.command('@S 1000', time)
            robot.tick(time)
            state = plant.row()
            rows.append(dict(t=time, target=robot.command_target.tolist(),
                actual=state['actual_deg'], roll=state['roll_deg'], pitch=state['pitch_deg'],
                moving=robot.motion is not None, transition=robot.transition is not None,
                safety=robot.safety))
        reports.append(dict(command=command, stop_at=stop_tick*.02,
            stopped=robot.motion is None and robot.transition is None,
            target_error_deg=float(np.max(abs(robot.command_target-robot.stand_target))),
            actual_error_deg=float(np.max(abs(np.degrees(plant.data.qpos[plant.q])-robot.stand_target))),
            final_tilt_deg=max(abs(rows[-1]['roll']), abs(rows[-1]['pitch'])), rows=rows))
    return reports


def clearance_cases():
    import fcl
    plant = Simulation(load_parameters(parse_args([])))
    model = plant.model
    objects = {}
    for name in ('fr_j2', 'fr_j3', 'body_0', 'body_1', 'body_2', 'body_3'):
        geom = model.geom(name).id
        mesh = model.geom_dataid[geom]
        va, fa = model.mesh_vertadr[mesh], model.mesh_faceadr[mesh]
        vertices = model.mesh_vert[va:va+model.mesh_vertnum[mesh]].astype(float)
        faces = model.mesh_face[fa:fa+model.mesh_facenum[mesh]]
        bvh = fcl.BVHModel()
        bvh.beginModel(len(vertices), len(faces))
        bvh.addSubModel(vertices, faces)
        bvh.endModel()
        objects[name] = (geom, fcl.CollisionObject(bvh))
    reports = []
    for limit in (None, 10., 9.):
        profile = copy.deepcopy(PROFILES['s_native_v5'])
        if limit is None:
            profile.pop('normal_adduction_limit_deg')
        else:
            profile['normal_adduction_limit_deg'] = limit
        gait = SNativeGait(model, plant.stand_target, profile)
        rows = []
        for phase in np.linspace(.5, 1., 51):
            angles = gait.targets(phase % 1, 1., 1., 0.)
            gait.kin.set_angles(angles)
            data = gait.kin.data
            for geom, obj in objects.values():
                obj.setTransform(fcl.Transform(data.geom_xmat[geom].reshape(3, 3), data.geom_xpos[geom]))
            pairs = []
            for link in ('fr_j2', 'fr_j3'):
                for body in ('body_0', 'body_1', 'body_2', 'body_3'):
                    a, b = objects[link][1], objects[body][1]
                    distance = fcl.distance(a, b, fcl.DistanceRequest(), fcl.DistanceResult())
                    collisions = fcl.collide(a, b, fcl.CollisionRequest(), fcl.CollisionResult())
                    pairs.append(dict(pair=[link, body], distance_mm=distance*1000, collision=collisions > 0))
            rows.append(dict(phase=float(phase), angles=angles.tolist(), pairs=pairs))
        reports.append(dict(normal_limit_deg=limit, normal_adduction_deg=gait.normal_adduction.tolist(),
            min_distance_mm=min(p['distance_mm'] for row in rows for p in row['pairs']),
            collision=any(p['collision'] for row in rows for p in row['pairs']), rows=rows))
    return dict(method='FCL triangle BVH; first swing at full stride amplitude, 51 poses, FR J2/J3 vs four chassis parts. Other pairs, tolerances and full dynamic clearance are not assessed.', cases=reports)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--clearance', action='store_true')
    parser.add_argument('--output', type=Path, default=Path('artifacts/s-native-v5/recheck-entry'))
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    stops = stop_cases()
    (args.output/'stop-validation.json').write_text(json.dumps(stops, indent=2))
    print(json.dumps([{k: v for k, v in row.items() if k != 'rows'} for row in stops], indent=2), flush=True)
    if args.clearance:
        clearance = clearance_cases()
        (args.output/'triangle-clearance-comparison.json').write_text(json.dumps(clearance, indent=2))
        print(json.dumps([{k: v for k, v in row.items() if k != 'rows'} for row in clearance['cases']], indent=2))


if __name__ == '__main__':
    main()
