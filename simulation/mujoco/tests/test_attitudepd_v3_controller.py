"""Stand/start/stop controller contracts; physics validation is recorded separately."""
import json
import numpy as np
import pytest
from simulation.mujoco.paths import SIM_ROOT
from simulation.mujoco.runtime.cad_physics import Simulation
from simulation.mujoco.runtime.virtual_robot import RobotController


@pytest.fixture(params=['attitudepd_v3','attitudepd_v4','attitudepd_v5','attitudepd_v6'])
def robot(request):
    parameters=json.loads((SIM_ROOT/'cad_300mm/physics_parameters_measured_total_2754g.json').read_text())
    parameters['foot_cushion']=json.loads((SIM_ROOT/'config/foot_cushion_d37p3_l27mm.json').read_text())
    controller=RobotController(Simulation(parameters))
    assert controller.profile=='attitudepd_v6'
    controller.select_profile(request.param)
    yield controller
    controller.body_stabilizer.close()


def test_default_stand_command_and_stop_share_b(robot):
    assert robot.profile in ('attitudepd_v3','attitudepd_v4','attitudepd_v5','attitudepd_v6')
    robot.command('syncstate',0.)
    assert robot.profile in robot.drain().decode().split(' caps=')[1].split(' ')[0]
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
    profile=robot.profile
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
    robot.select_profile(profile)
    np.testing.assert_array_equal(robot.stand_target,b)
    np.testing.assert_array_equal(robot.plant.data.qpos,qpos)


def test_start_from_a_prepares_b_without_an_intermediate_a_command(robot):
    robot.begin(('drive',),0.)
    assert robot.transition is not None
    np.testing.assert_array_equal(robot.transition[1],robot.stand_target)
    np.testing.assert_allclose(robot.transition[0],np.degrees(robot.plant.data.qpos[robot.plant.q]))


def test_simwalk_phase_and_pd_use_the_same_deployed_period(robot):
    import mujoco
    b=robot.stand_target.copy()
    robot.plant.data.qpos[robot.plant.q]=np.radians(b)
    mujoco.mj_forward(robot.plant.model,robot.plant.data)
    robot.target=b.copy();robot.command_target=b.copy();robot.pose='stand'
    robot.command('simwalk 5',0.)
    assert robot.motion[0]=='profile' and robot.transition is None
    robot.elapsed=2.;robot.linear=1.;robot.yaw=0.;robot.phase=0.
    expected=robot.active_profile_period()
    if robot.profile=='attitudepd_v5':assert expected==pytest.approx(1.05/1.3)
    if robot.profile=='attitudepd_v6':assert expected==pytest.approx(1.05/1.3*1.25)
    frames=[]
    def capture(target,frame,*args,**kwargs):
        frames.append(frame.copy())
        return target.copy()
    robot.body_stabilizer.apply=capture
    robot.tick(.02)
    assert robot.phase==pytest.approx(.02/expected)
    assert frames[-1]['period_s']==pytest.approx(expected)


def test_shared_start_scale_matches_simwalk_drive_and_foot_lift_adapter(robot,monkeypatch):
    import ctypes as ct
    from simulation.mujoco.runtime.drive_controller import NAMES

    # Inspect the production command pipeline without stepping estimated
    # contact physics or adding body feedback to its nominal targets.
    monkeypatch.setattr(robot.plant,'step',lambda **kwargs: None)
    robot.body_stabilizer.apply=lambda target,*args,**kwargs: target.copy()
    robot.heading.enabled=False
    robot.tracking_enabled=False
    robot.foot_lift_mm=[30,30,0,0]
    fp=ct.POINTER(ct.c_float)
    nominal_fn=robot.plant.policy._library.spot_locomotion_targets
    nominal_fn.argtypes=(ct.c_int,*([ct.c_float]*4),fp)
    nominal_fn.restype=ct.c_int
    lift_fn=robot.plant.policy._library.spot_foot_lift
    lift_fn.argtypes=(ct.POINTER(ct.c_uint32),ct.c_float,ct.c_float,ct.c_float,fp,fp)
    lift_fn.restype=ct.c_int

    for linear,yaw in ((1.,0.),(-1.,0.),(0.,.5)):
        for elapsed in (.5,1.,1.5,2.):
            outputs=[]
            for kind in ('drive','profile'):
                robot.motion=('drive',) if kind=='drive' else ('profile',10.,1.)
                robot.transition=None
                robot.stopping_reason=None
                robot.pose='stand';robot.safety='ok';robot.torque=True
                robot.last_packet=0.
                robot.phase=.75
                robot.elapsed=elapsed-.02
                robot.linear=linear;robot.yaw=yaw;robot.request=(linear,yaw)
                robot.tick(.02)
                assert robot.motion is not None,(kind,linear,yaw,elapsed,robot.safety,robot.drain().decode())
                assert robot.elapsed==pytest.approx(elapsed,abs=2e-6)
                duration=2. if robot.profile=='attitudepd_v6' and linear>0 else 1.
                expected_scale=robot.plant.policy.smootherstep(min(1.,elapsed/duration))
                assert robot.gait_start_scale()==pytest.approx(expected_scale,abs=2e-6)

                # Explicitly give the C foot adapter the expected entry
                # scale: a stale one-second adapter cannot pass by merely
                # agreeing with an equally stale simulator gait wrapper.
                expected=(ct.c_float*12)()
                assert nominal_fn(NAMES.index(robot.profile),.75,expected_scale,linear,yaw,expected)
                nominal=np.array(expected)
                activity=expected_scale*min(1.,(abs(linear)+abs(yaw))/.15)
                assert lift_fn((ct.c_uint32*4)(30,30,0,0),.75,robot.active_profile_params()[1],
                               activity,(ct.c_float*4)(0,.5,.5,0),expected)
                np.testing.assert_allclose(robot.target,expected,rtol=0,atol=2e-4)
                assert np.max(abs(np.array(expected)-nominal))>.01
                outputs.append(robot.target.copy())
            np.testing.assert_allclose(outputs[0],outputs[1],rtol=0,atol=2e-4)
