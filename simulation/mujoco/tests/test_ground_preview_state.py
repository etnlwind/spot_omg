
# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[3]))

from simulation.mujoco.paths import REPO_ROOT, SIM_ROOT, RESULTS_ROOT
import json
from pathlib import Path
import numpy as np
import pytest
import simulation.mujoco.runtime.virtual_robot as virtual_robot
import simulation.mujoco.runtime.drive_controller as drive_controller
from simulation.mujoco.runtime.cad_physics import Simulation
from simulation.mujoco.runtime.virtual_robot import RobotController
from simulation.mujoco.runtime.ground_frame_nominal import configure


@pytest.fixture
def preview(monkeypatch):
    monkeypatch.setattr(virtual_robot, 'shared_drive_step', drive_controller.step)
    path = SIM_ROOT
    p = json.loads((path / 'cad_300mm/physics_parameters_measured_total_2754g.json').read_text())
    p['foot_cushion'] = json.loads((path / 'config/foot_cushion_d37p3_l27mm.json').read_text())
    robot = RobotController(Simulation(p)); robot.select_profile('centerpivot')
    info = configure(robot, dict(period_s=3.2, duty=.7, cartesian_stride_m=.096,
        foothold_transfer=True, touchdown_x_offset_m=.02, max_foothold_y_m=.01,
        anticipate_full_amplitude=True, startup_s=4))
    return robot, info


def test_stop_uses_corrected_start_and_aligned_neutral_without_state_overwrite(preview):
    robot, info = preview
    robot.begin(('drive',), 0.)
    robot.command_target = robot.target + np.tile([.2, .3, .4], 4)
    corrected = robot.command_target.copy()
    qpos = robot.plant.data.qpos.copy(); qvel = robot.plant.data.qvel.copy()
    robot.finish_stop('requested')
    np.testing.assert_array_equal(robot.transition[0], corrected)
    np.testing.assert_array_equal(robot.transition[1], np.array(info['neutral_deg']).reshape(12))
    np.testing.assert_array_equal(robot.plant.data.qpos, qpos)
    np.testing.assert_array_equal(robot.plant.data.qvel, qvel)
    robot.target = robot.transition[1].copy(); robot.transition = None
    robot.reply('$SPOTDRIVE stopped reason=requested\r\nOK')
    assert robot.pose == 'stand'


def test_repeat_begin_resets_custom_phase_and_foothold_history(preview):
    robot, info = preview
    robot.begin(('drive',), 0.); robot.request = (1., 0.)
    first=[]
    for _ in range(10):
        robot.target = virtual_robot.shared_drive_step(robot)
        first.append(robot.target.copy())
    for _ in range(140):
        robot.target = virtual_robot.shared_drive_step(robot)
    robot.command_target = robot.target.copy()
    robot.finish_stop('requested')
    robot.begin(('drive',), 5.); robot.request = (1., 0.)
    second=[]
    for _ in range(10):
        robot.target = virtual_robot.shared_drive_step(robot)
        second.append(robot.target.copy())
    np.testing.assert_allclose(second, first, atol=1e-10)


def test_completed_shared_motion_cannot_be_overwritten(preview, monkeypatch):
    robot, _ = preview
    held = robot.target + .2
    def ending_step(r):
        r.motion = None; r.target = held.copy()
        return held.copy()
    monkeypatch.setattr(virtual_robot, 'shared_drive_step', ending_step)
    configure(robot, dict(period_s=3.2, duty=.7, cartesian_stride_m=.096))
    robot.begin(('drive',), 0.)
    np.testing.assert_array_equal(virtual_robot.shared_drive_step(robot), held)
