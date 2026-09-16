"""Stand speed follows the command and measured Landing, never target identity."""
import json

import numpy as np
import pytest

from simulation.mujoco.runtime.cad_gait import CAD
from simulation.mujoco.runtime.cad_physics import Simulation
from simulation.mujoco.runtime.stow_policy import LANDING, pose_frame
from simulation.mujoco.runtime.virtual_robot import RobotController


@pytest.mark.parametrize('target', [[0., 45., 90.] * 4,
                                   [0., 46.011386, 90.016018] * 4,
                                   [2., 50., 100.] * 4])
def test_landing_to_stand_does_not_depend_on_a_registered_target(target):
    output, duration = pose_frame(LANDING, target, stand_requested=True)
    assert duration == 0
    np.testing.assert_allclose(output, target, atol=.1)
    assert pose_frame(LANDING, target, stand_requested=False)[1] > 0


def test_stand_outside_recognized_landing_keeps_supervised_interpolation():
    target = [0., 46., 90.] * 4
    start = LANDING.copy()
    start[0] = 8.  # Outside the shared 80-tick (~7 degree) recognition gate.
    assert pose_frame(start, target, stand_requested=True)[1] > 0


def test_simulator_forwards_stand_intent_and_preserves_physical_state():
    parameters = json.loads((CAD / 'physics_parameters.json').read_text())
    robot = RobotController(Simulation(parameters))
    robot.plant.data.qpos[robot.plant.q] = np.radians(LANDING)
    robot.plant.data.qvel[:] = 0
    robot.stand_target = np.array([0., 46.011386, 90.016018] * 4)
    before = robot.plant.data.qpos.copy()
    robot.command('stand', 0)
    assert robot.transition[3] == .02
    assert robot.transition[4] == 'OK stand'
    np.testing.assert_array_equal(robot.plant.data.qpos, before)

    # A different command going to the same numbers must not inherit fast Stand.
    robot.blend_pose(robot.stand_target, 'OK stand11')
    assert robot.transition[3] > 1
