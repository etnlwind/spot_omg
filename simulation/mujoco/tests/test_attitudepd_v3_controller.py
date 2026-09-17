"""Stand/start/stop controller contracts; physics validation is recorded separately."""
import json
import numpy as np
import pytest
from simulation.mujoco.paths import SIM_ROOT
from simulation.mujoco.runtime.cad_physics import Simulation
from simulation.mujoco.runtime.virtual_robot import RobotController


@pytest.fixture
def robot():
    parameters=json.loads((SIM_ROOT/'cad_300mm/physics_parameters_measured_total_2754g.json').read_text())
    parameters['foot_cushion']=json.loads((SIM_ROOT/'config/foot_cushion_d37p3_l27mm.json').read_text())
    controller=RobotController(Simulation(parameters))
    yield controller
    controller.body_stabilizer.close()


def test_default_stand_command_and_stop_share_b(robot):
    assert robot.profile=='attitudepd_v3'
    robot.command('syncstate',0.)
    assert 'attitudepd_v3' in robot.drain().decode().split(' caps=')[1].split(' ')[0]
    b=robot.stand_target.copy()
    assert b[1]>50 and not np.allclose(b,robot.base_stand_target)
    robot.command('stand',0.)
    np.testing.assert_array_equal(robot.transition[1],b)
    # A corrected last gait command must fade directly toward B on stop.
    robot.motion=('drive',);robot.transition=None
    corrected=b+np.tile([.2,.3,.4],4)
    robot.command_target=corrected.copy()
    robot.finish_stop('requested')
    np.testing.assert_array_equal(robot.transition[0],corrected)
    np.testing.assert_array_equal(robot.transition[1],b)
    assert robot.torque


def test_already_at_b_starts_without_preparation_and_selection_never_teleports(robot):
    b=robot.stand_target.copy()
    # Fixture-only encoder input: explicitly test the measured-at-B branch.
    robot.plant.data.qpos[robot.plant.q]=np.radians(b)
    robot.target=b.copy();robot.command_target=b.copy();robot.pose='stand'
    robot.begin(('drive',),0.)
    assert robot.transition is None
    np.testing.assert_array_equal(robot.target,b)
    robot.motion=None
    qpos=robot.plant.data.qpos.copy();qvel=robot.plant.data.qvel.copy()
    robot.select_profile('attitudepd_v2')
    np.testing.assert_array_equal(robot.stand_target,robot.base_stand_target)
    np.testing.assert_array_equal(robot.plant.data.qpos,qpos)
    np.testing.assert_array_equal(robot.plant.data.qvel,qvel)
    robot.select_profile('attitudepd_v3')
    np.testing.assert_array_equal(robot.stand_target,b)
    np.testing.assert_array_equal(robot.plant.data.qpos,qpos)


def test_start_from_a_prepares_b_without_an_intermediate_a_command(robot):
    robot.begin(('drive',),0.)
    assert robot.transition is not None
    np.testing.assert_array_equal(robot.transition[1],robot.stand_target)
    np.testing.assert_allclose(robot.transition[0],np.degrees(robot.plant.data.qpos[robot.plant.q]))
