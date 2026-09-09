"""Check CAD placement survives articulation and shared policy reaches all joints."""
import json
from pathlib import Path
import subprocess
import sys

import mujoco
import numpy as np

CAD = Path(__file__).parent / 'cad_300mm'


def test_zero_pose_preserves_every_imported_mesh_transform():
    source = mujoco.MjModel.from_xml_path(str(CAD / 'scene.xml'))
    rig = mujoco.MjModel.from_xml_path(str(CAD / 'gait_scene.xml'))
    sd, rd = mujoco.MjData(source), mujoco.MjData(rig)
    mujoco.mj_forward(source, sd)
    mujoco.mj_forward(rig, rd)
    assert rig.njnt == 12
    base = rig.body('cad_base').id
    rotation = rd.xmat[base].reshape(3, 3)
    for index in range(source.ngeom):
        name = mujoco.mj_id2name(source, mujoco.mjtObj.mjOBJ_GEOM, index)
        target = rig.geom(name).id
        np.testing.assert_allclose(rd.geom_xpos[target], rotation @ sd.geom_xpos[index] + rd.xpos[base], atol=1e-9)
        np.testing.assert_allclose(rd.geom_xmat[target].reshape(3,3), rotation @ sd.geom_xmat[index].reshape(3,3), atol=1e-9)


def test_shared_policy_replay_moves_all_twelve_cad_joints(tmp_path):
    subprocess.run([sys.executable, str(Path(__file__).with_name('cad_gait.py')),
                    '--check', '--duration', '5', '--output', str(tmp_path)],
                   capture_output=True, text=True, check=True, timeout=30)
    report = json.loads((tmp_path/'summary.json').read_text())
    assert report['samples'] == 250
    assert len(report['ranges_deg']) == 12
    assert all(np.isfinite(report['ranges_deg']))
    assert min(report['ranges_deg']) > 0
    frames = json.loads((tmp_path/'frames.json').read_text())
    pairs = {tuple(f['scheduled_support']) for f in frames}
    assert ('FL', 'RR') in pairs
    assert ('FR', 'RL') in pairs
