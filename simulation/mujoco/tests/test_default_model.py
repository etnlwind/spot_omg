
# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[3]))
import pytest
import numpy as np
from simulation.mujoco.runtime.cad_physics import Simulation
from simulation.mujoco.runtime.virtual_robot import parse_args, load_parameters, model_description


def test_default_launch_builds_measured_robot_with_cushions():
    parameters = load_parameters(parse_args([]))
    cushion = parameters['foot_cushion']
    assert cushion['sole_diameter_m'] == pytest.approx(.0373)
    assert cushion['total_length_m'] == pytest.approx(.027)
    plant = Simulation(parameters)
    assert np.isfinite(plant.model.geom_pos).all()
    assert np.isfinite(plant.model.geom_size).all()
    assert plant.model.body_mass.sum() == pytest.approx(2.754, abs=.001)
    assert '2.754 kg / cushion D37.3 x 27 mm' in model_description(plant, parameters)


def test_missing_cushion_fails_instead_of_silently_removing_it(tmp_path):
    args = parse_args(['--foot-cushion', str(tmp_path/'missing.json')])
    with pytest.raises(FileNotFoundError):
        load_parameters(args)


def test_stand_command_and_stop_share_aligned_start_pose():
    from simulation.mujoco.runtime.virtual_robot import RobotController
    from simulation.mujoco.runtime.standing_pose import ContactKinematics, LEGS
    plant=Simulation(load_parameters(parse_args([])))
    robot=RobotController(plant)
    kin=ContactKinematics(plant.model);kin.set_angles(robot.stand_target)
    for i,leg in enumerate(LEGS):
        assert abs(kin.foot(i)[0]-kin.data.xanchor[plant.model.joint(leg+'_j2').id,0]) < .0002
    initial=robot.target.copy()
    robot.command('stand',0)
    np.testing.assert_allclose(robot.transition[1],initial)
    robot.transition=None;robot.motion=('drive',)
    robot.finish_stop('requested')
    np.testing.assert_allclose(robot.transition[1],initial)


def test_s_drive_starts_without_preparation_and_preserves_physical_state():
    from simulation.mujoco.runtime.virtual_robot import RobotController
    plant=Simulation(load_parameters(parse_args([])));robot=RobotController(plant)
    robot.select_profile('centerpivot')
    qpos=plant.data.qpos.copy();qvel=plant.data.qvel.copy()
    robot.command('drive 600 0 1',0.)
    assert robot.transition is None
    np.testing.assert_array_equal(plant.data.qpos,qpos)
    np.testing.assert_array_equal(plant.data.qvel,qvel)
    robot.tick(.02)
    assert robot.elapsed > 0
    assert max(abs(robot.target-robot.stand_target)) < .1


def test_different_policy_neutrals_all_map_to_s():
    from simulation.mujoco.runtime.standing_pose import StandingGaitFrame
    plant=Simulation(load_parameters(parse_args([])))
    frame=StandingGaitFrame(plant.model,plant.stand_target)
    for neutral in (np.tile([0,45,90],4),np.tile([0,40,80],4)):
        np.testing.assert_allclose(frame.targets(neutral,neutral),plant.stand_target,atol=.01)
