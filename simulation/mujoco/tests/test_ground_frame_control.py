
# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[3]))

from simulation.mujoco.paths import REPO_ROOT, SIM_ROOT, RESULTS_ROOT
import json
from pathlib import Path
from types import SimpleNamespace
import mujoco
import numpy as np
import pytest
from simulation.mujoco.runtime.cad_physics import build
from simulation.mujoco.scripts.tuning.search_gait_profiles import physics
from simulation.mujoco.runtime.ground_frame_control import GroundFrameControl
from simulation.mujoco.runtime.gait_profiles import foot_targets


@pytest.fixture(scope='module')
def model():
    parameters, _ = physics()
    parameters['foot_cushion'] = json.loads((SIM_ROOT / 'config/foot_cushion_d37p3_l27mm.json').read_text())
    xml, _ = build(parameters, write_scene=False)
    return mujoco.MjModel.from_xml_string(xml)


def test_level_target_is_unchanged(model):
    controller = GroundFrameControl(model)
    angles = foot_targets([1.35, .52, .08, .032, .22, -.035, .75], 0, 0).reshape(12)
    sensor = SimpleNamespace(filtered=[0, 0], rate=[0, 0], failures=0)
    np.testing.assert_array_equal(controller.apply(angles, angles, sensor, .25, .52), angles)


def test_world_swing_height_uses_rotated_cushion(model):
    controller = GroundFrameControl(model)
    params = [1.35, .52, .08, .032, .22, -.035, .75]
    angles = foot_targets(params, .75 * params[0], 1).reshape(12)
    sensor = SimpleNamespace(filtered=[20, -10], rate=[0, 0], failures=0)
    for _ in range(15):
        output = controller.apply(angles, angles, sensor, .75, .52,
            {'max_correction_deg': 25., 'max_step_deg': 5.})
    assert max(controller.diagnostic['applied_task_residual_m']) < .0001
    assert max(abs(output[::3] - angles[::3])) > .01
    assert not controller.diagnostic['correction_clipped']


def test_invalid_sensor_resets_correction(model):
    controller = GroundFrameControl(model)
    angles = np.tile([0., 45., 90.], 4)
    controller.correction[:] = 4.
    sensor = SimpleNamespace(filtered=[float('nan'), 0], rate=[0, 0], failures=0)
    np.testing.assert_array_equal(controller.apply(angles, angles, sensor, .25, .52), angles)
    np.testing.assert_array_equal(controller.correction, np.zeros(12))
    assert not controller.diagnostic['enabled']


def test_unreachable_correction_is_reported(model):
    controller = GroundFrameControl(model)
    angles = np.tile([0., 45., 90.], 4)
    sensor = SimpleNamespace(filtered=[80, 0], rate=[0, 0], failures=0)
    output = controller.apply(angles, angles, sensor, .75, .52,
        {'max_correction_deg': .1, 'max_step_deg': .05})
    assert controller.diagnostic['correction_clipped']
    assert max(controller.diagnostic['applied_task_residual_m']) > .001
    assert max(abs(output - angles)) <= .050001
