"""Read-only recorded-state CAD audit of diagonal ground-height differences.

Rotate every cushion vertex, then select the lowest vertex separately. A
shared root translation and yaw cancel from diagonal bottom-height difference.
Flat-pose and measured-attitude decompositions are counterfactual geometry,
not a dynamic simulation or a proof of the initiating physical cause.
"""
from pathlib import Path
import json
import math
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'simulation/mujoco'))
from cad_physics import Simulation
from support_shift import SupportShift


def rotation(roll, pitch):
    r, p = math.radians(roll), math.radians(pitch)
    rx = np.array([[1., 0., 0.], [0., math.cos(r), -math.sin(r)], [0., math.sin(r), math.cos(r)]])
    ry = np.array([[math.cos(p), 0., math.sin(p)], [0., 1., 0.], [-math.sin(p), 0., math.cos(p)]])
    return ry @ rx


def geometry(kin, angles, roll, pitch):
    kin.set_angles(angles)
    flat, tilted = [], []
    rot = rotation(roll, pitch)
    for i, f in enumerate(kin.feet):
        vertices = kin.vertices[i] @ kin.data.geom_xmat[f].reshape(3, 3).T + kin.data.geom_xpos[f]
        flat.append(float(vertices[:, 2].min() * 1000))
        tilted.append(float((vertices @ rot.T)[:, 2].min() * 1000))
    flat, tilted = np.array(flat), np.array(tilted)
    out = {}
    for front, rear, name in ((0, 3, 'FL/RR'), (1, 2, 'FR/RL')):
        flat_delta = flat[front] - flat[rear]
        tilted_delta = tilted[front] - tilted[rear]
        out[name] = dict(flat_front_minus_rear_bottom_mm=float(flat_delta),
            measured_attitude_front_minus_rear_bottom_mm=float(tilted_delta),
            rotation_increment_mm=float(tilted_delta - flat_delta))
    return out


def support_geometry(kin, angles, roll, pitch):
    kin.set_angles(angles)
    rot = rotation(roll, pitch)
    # Model COM and foot centers at recorded joints, transformed together.
    center = np.average(kin.data.xipos, axis=0, weights=kin.model.body_mass) @ rot.T
    feet = kin.data.geom_xpos[kin.feet] @ rot.T
    lines = {}
    for front, rear, name in ((0, 3, 'FL/RR'), (1, 2, 'FR/RL')):
        a, b = feet[[front, rear], :2]
        v = b - a
        fraction = np.dot(center[:2] - a, v) / np.dot(v, v)
        delta = center[:2] - a
        signed = (v[0]*delta[1] - v[1]*delta[0]) / np.linalg.norm(v)
        lines[name] = dict(com_perpendicular_distance_mm=float(abs(signed)*1000),
            signed_com_distance_mm=float(signed*1000),
            com_projection_fraction=float(fraction),
            approximate_gravity_moment_nm=float(abs(signed)*kin.model.body_mass.sum()*9.81))
    return dict(com_body_rotated_m=center.tolist(), feet_body_rotated_m=feet.tolist(),
        lines=lines, limitation='Center-line proxy includes cushion rolling. Actual finite contact patches, COP, inertia and dynamic contact force distribution are not solved by this geometric distance.')


def main():
    params = json.loads((ROOT / 'simulation/mujoco/cad_300mm/physics_parameters_measured_total_2754g.json').read_text())
    params.update(timestep_s=.0005, experimental_stow=True)
    params['foot_cushion'] = json.loads((ROOT / 'simulation/mujoco/foot_cushion_10mm.json').read_text())
    plant = Simulation(params)
    kin = SupportShift(plant.model)
    cases = {
        'causal-original.json': [10.2, 10.38, 10.5, 10.8, 11., 11.44, 11.54, 11.84, 14.],
        'causal-nocorrection.json': [10.2, 10.38, 10.5, *np.arange(10.6, 11.041, .04), 11.44, 11.54, 11.84, 14.],
        'ground-ff-ablation.json': [15.6, 16.14, 16.36],
        'ground-ff-final60.json': [15.6, 16.24, 16.4],
    }
    results = []
    for name, epochs in cases.items():
        source = Path(__file__).with_name(name)
        rows = json.loads(source.read_text())['records']
        for epoch in epochs:
            row = min(rows, key=lambda r:abs(r['t'] - epoch))
            record = dict(source=str(source), time_s=row['t'], roll_deg=row['roll'], pitch_deg=row['pitch'],
                force_n=row['force'], clearance_mm=row['clearance'],
                geometry={field:geometry(kin, row[field], row['roll'], row['pitch'])
                          for field in ('nominal', 'command', 'filtered', 'actual') if field in row},
                support_geometry=support_geometry(kin, row['actual'], row['roll'], row['pitch']))
            results.append(record)
    output = dict(definition='All deltas are front minus diagonal rear. Flat pose keeps recorded joint angles with root roll/pitch zero. Rotation increment is tilted minus flat geometry; root translation cancels. Lowest cushion vertex is reselected in each orientation.',
        mass_kg=float(plant.model.body_mass.sum()), records=results)
    Path(__file__).with_name('contact-geometry-decomposition.json').write_text(json.dumps(output, indent=2))
    for r in results:
        print(Path(r['source']).name, r['time_s'], 'RP', np.round([r['roll_deg'], r['pitch_deg']], 2),
              'actual',r['geometry']['actual'])


if __name__ == '__main__':
    main()
