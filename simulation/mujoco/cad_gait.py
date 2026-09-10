"""Replay the shared C drive policy on the new STEP's provisional 12-joint rig.

Kinematic preview: fixed body, prescribed angles, no force/contact prediction.
"""
import argparse
import copy
import json
import math
import time
from pathlib import Path
from xml.etree import ElementTree as ET

import mujoco
import numpy as np
from servo import SharedGaitPolicy

CAD = Path(__file__).parent / 'cad_300mm'
KEYS = [(leg, j) for leg in ('FL', 'FR', 'RL', 'RR') for j in (1, 2, 3)]


def vec(values):
    return ' '.join(f'{v:.12g}' for v in values)


def build(write_files=True):
    cylinders = json.loads((CAD / 'cylinder_candidates.json').read_text())
    source = ET.parse(CAD / 'scene.xml').getroot()
    root = ET.Element('mujoco', model='New STEP / shared drive / KINEMATIC ONLY')
    ET.SubElement(root, 'compiler', angle='radian', meshdir=str(CAD.resolve()))
    ET.SubElement(root, 'option', gravity='0 0 0')
    ET.SubElement(root, 'statistic', center='0 0 0.22', extent='0.65')
    root.append(copy.deepcopy(source.find('asset')))
    # Camera fill and directional studio lights keep the moving robot readable.
    visual = ET.SubElement(root, 'visual')
    ET.SubElement(visual, 'headlight', ambient='.35 .35 .35',
                  diffuse='.5 .5 .5', specular='.15 .15 .15')
    ET.SubElement(root.find('asset'), 'texture', name='studio_sky', type='skybox',
                  builtin='gradient', rgb1='.28 .34 .42', rgb2='.55 .62 .70',
                  width='512', height='3072')
    world = ET.SubElement(root, 'worldbody')
    ET.SubElement(world, 'light', name='studio_key', pos='0 -1 2',
                  directional='true', dir='.3 .4 -1', diffuse='.6 .6 .6')
    ET.SubElement(world, 'light', name='studio_fill', pos='0 1 2',
                  directional='true', dir='-.4 -.3 -1', diffuse='.3 .3 .3',
                  castshadow='false')
    ET.SubElement(world, 'geom', type='plane', size='1 1 .01', pos='0 0 0', rgba='.42 .46 .50 1', contype='0', conaffinity='0')
    # CAD +X is robot left, CAD -Y is robot front, CAD +Z is up.
    base = ET.SubElement(world, 'body', name='cad_base', pos='.1495 -.05225 .253306637937', quat='.707106781187 0 0 .707106781187')
    geoms = {g.get('name'): g for g in source.find('worldbody').findall('geom')}
    for name, geom in geoms.items():
        if name.startswith('body'):
            base.append(copy.deepcopy(geom))
    mapping = {}
    for leg in ('FL', 'FR', 'RL', 'RR'):
        parent, previous = base, np.zeros(3)
        for j in (1, 2, 3):
            name = f'{leg.lower()}_j{j}'
            # Large hip bearing bore; upper/knee motor shaft bores. Restrict
            # by group, radius, direction and height to exclude screw holes.
            candidates = [e for e in cylinders[name]
                          if abs(e['radius_mm'] - (11 if j == 1 else 3)) < .01
                          and abs(e['axis'][1 if j == 1 else 0]) > .99
                          and (j == 1 or (e['origin_mm'][2] > 40 if j == 2 else -95 < e['origin_mm'][2] < -70))]
            unique = {tuple(e['origin_mm']): e for e in candidates}
            if len(unique) != 1:
                raise ValueError(f'{name}: ambiguous shaft candidates: {len(unique)}')
            candidate = next(iter(unique.values()))
            pivot = np.array(candidate['origin_mm']) / 1000
            axis = np.array(candidate['axis'])
            expected = np.array([0, -1 if leg.endswith('L') else 1, 0]) if j == 1 else np.array([1 if j == 2 else -1, 0, 0])
            if axis @ expected < 0:
                axis = -axis
            body = ET.SubElement(parent, 'body', name=name+'_link', pos=vec(pivot-previous))
            ET.SubElement(body, 'inertial', pos='0 0 0', mass='.1', diaginertia='.001 .001 .001')
            ET.SubElement(body, 'joint', name=name, type='hinge', axis=vec(axis), limited='false')
            geom = copy.deepcopy(geoms[name]); geom.set('pos', vec(-pivot)); body.append(geom)
            ET.SubElement(body, 'site', name=name+'_axis', type='sphere', size='.003', rgba='1 .2 .1 1')
            mapping[name] = dict(pivot_cad_m=pivot.tolist(), axis_cad=axis.tolist(), source_radius_mm=candidate['radius_mm'])
            parent, previous = body, pivot
    if not write_files:
        return root, mapping
    ET.indent(root)
    ET.ElementTree(root).write(CAD / 'gait_scene.xml', encoding='unicode')
    (CAD / 'joint_mapping.json').write_text(json.dumps(dict(
        status='Provisional geometry-derived axes; not calibrated on hardware',
        mode='kinematic only; body fixed; inertials are compiler placeholders',
        zero='Exported near-straight CAD pose provisionally treated as logical 0/0/0',
        coordinates='robot X=-CAD Y, robot Y=CAD X, robot Z=CAD Z',
        caveats=['CAD groups treated as serial J1 -> J2 -> J3 links; motor housings may need reassignment.',
                 'Shaft directions retain CAD misalignments; logical signs follow existing URDF convention.',
                 'No contact, motor torque, battery, mass, balance or locomotion prediction.'],
        joints=mapping), indent=2))
    return CAD / 'gait_scene.xml'


def main():
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument('--check', action='store_true')
    parser.add_argument('--duration', type=float, default=12)
    parser.add_argument('--linear', type=float, default=.6)
    parser.add_argument('--yaw', type=float, default=0)
    parser.add_argument('--output', type=Path, default=Path('/private/tmp/spot-cad-gait'))
    args = parser.parse_args()
    if not all(math.isfinite(x) for x in (args.duration, args.linear, args.yaw)) or args.duration <= 0 or max(abs(args.linear), abs(args.yaw)) > 1:
        parser.error('finite duration > 0 and linear/yaw in [-1,1] required')
    model = mujoco.MjModel.from_xml_path(str(build()))
    data = mujoco.MjData(model)
    policy = SharedGaitPolicy()
    addresses = [model.jnt_qposadr[model.joint(f'{leg.lower()}_j{j}').id] for leg, j in KEYS]
    requested = [args.linear, args.yaw]
    def keypress(key):
        values = {87:(1,0),83:(-1,0),65:(0,-1),68:(0,1),32:(0,0)}
        if key in values:
            requested[:] = values[key]
    viewer = None
    if not args.check:
        import mujoco.viewer as mj_viewer
        viewer = mj_viewer.launch_passive(model, data, key_callback=keypress)
        viewer.cam.lookat[:] = [0, 0, .22]
        viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = .95, 135, -20
    print('NEW STEP: shared C drive, fixed-body kinematic replay. W/S/A/D/Space. No torque/contact simulation.', flush=True)
    phase = linear = yaw = 0.
    frames = []
    started = time.monotonic()
    try:
        for index in range(math.ceil(args.duration / .02)):
            if viewer and not viewer.is_running():
                break
            elapsed = index * .02
            linear += np.clip(requested[0]-linear, -.04, .04)
            yaw += np.clip(requested[1]-yaw, -.04, .04)
            targets, support = policy.drive_targets(phase, policy.smootherstep(min(1, elapsed/.7)), float(linear), float(yaw))
            angles = np.array([targets[k] for k in KEYS])
            data.qpos[addresses] = np.radians(angles)
            data.time = elapsed
            mujoco.mj_forward(model, data)
            if not np.isfinite(data.xpos).all():
                raise RuntimeError('Nonfinite kinematics')
            frames.append(dict(time_s=elapsed, angles_deg=angles.tolist(), scheduled_support=sorted(support)))
            phase = (phase + .02/(2.4-.6*min(1, abs(linear)+abs(yaw)))) % 1
            if viewer:
                viewer.sync()
                time.sleep(max(0, started+(index+1)*.02-time.monotonic()))
        args.output.mkdir(parents=True, exist_ok=True)
        all_angles = np.array([f['angles_deg'] for f in frames])
        report = dict(mode='kinematic policy replay, not dynamic walking', samples=len(frames), joints=[f'{l}_J{j}' for l,j in KEYS],
                      ranges_deg=np.ptp(all_angles, axis=0).tolist() if len(frames) else [],
                      note='Support is commanded policy state, not measured ground contact. Body translation is fixed.')
        (args.output/'summary.json').write_text(json.dumps(report, indent=2))
        (args.output/'frames.json').write_text(json.dumps(frames))
        print(json.dumps(report), flush=True)
        if viewer:
            print('Replay finished; close window to exit.', flush=True)
            while viewer.is_running():
                viewer.sync(); time.sleep(.05)
    finally:
        if viewer:
            viewer.close()


if __name__ == '__main__':
    main()
