"""Real shared C PD→CAD IK contracts; no plant or hardware outcome claim.

Fixed nominal targets isolate the adapter from walking/contact dynamics. The
last tests exercise the simulator's Stop/IK-failure dispatch with physical
state unchanged, rather than asserting source text or mock motor motion.
"""
import ctypes as C
import json
from pathlib import Path

import numpy as np
import pytest

from body_stabilizer import BodyStabilizer, CONFIG


NOMINAL = np.tile([0., 48., 90.], 4)
OFFSETS = np.array([0., .5, .5, 0.])


@pytest.fixture
def pd():
    controller = BodyStabilizer()
    yield controller
    controller.close()


def sample(t, sequence, roll=0., pitch=0., gx=0., gy=0.):
    return dict(roll_tenths=roll*10, pitch_tenths=pitch*10,
                gyro_body_rad_s=[gx, gy, 0.], sample_time_s=t,
                gyro_sample_time_s=t, gyro_sequence=sequence,
                gyro_axis_verified=True)


def frame(phase=.25, moving=True):
    return dict(phase=phase, period_s=1.35, duty=.52, moving=moving)


def settle(controller, phase=.25, **imu):
    result = NOMINAL.copy()
    for i in range(25):
        t = i*.02
        result = controller.apply(NOMINAL, frame(phase), sample(t, i+1, **imu), t)
    return result


def test_every_named_c_default_matches_json(pd):
    expected = json.loads(CONFIG.read_text())
    default = pd.Config()
    pd.lib.spot_pd_default(C.byref(default))
    assert set(expected)-{'schema_version'} == {name for name, _ in pd.Config._fields_}
    for name, _ in pd.Config._fields_:
        assert getattr(default, name) == pytest.approx(expected[name], rel=1e-6, abs=1e-9)
        assert getattr(pd.config, name) == pytest.approx(getattr(default, name))


@pytest.mark.parametrize('phase', [.25, .75])
@pytest.mark.parametrize('axis', ['roll', 'pitch'])
@pytest.mark.parametrize('sign', [-1, 1])
def test_cad_p_signs_and_exact_swing_preservation(pd, phase, axis, sign):
    before = pd.feet(NOMINAL)
    result = settle(pd, phase, **{axis: sign*2.})
    delta = pd.feet(result)-before
    stance = (phase+OFFSETS) % 1 < .52
    expected = np.array([1, -1, 1, -1]) if axis == 'roll' else np.array([-1, -1, 1, 1])
    assert np.all(delta[stance, 2]*expected[stance]*sign > 0)
    np.testing.assert_array_equal(result.reshape(4, 3)[~stance], NOMINAL.reshape(4, 3)[~stance])
    np.testing.assert_array_equal(delta[~stance], 0.)
    np.testing.assert_allclose(delta[:, :2], 0., atol=1e-6)
    np.testing.assert_allclose(delta[:, 2], pd.diagnostic['applied_dz'], atol=1e-6)


@pytest.mark.parametrize('axis', ['gx', 'gy'])
def test_real_gyro_damps_with_no_angle_change(pd, axis):
    result = settle(pd, **{axis: .2})
    delta = pd.feet(result)-pd.feet(NOMINAL)
    signs = np.array([1, -1]) if axis == 'gx' else np.array([-1, 1])
    assert np.all(delta[[0, 3], 2]*signs > 0)
    np.testing.assert_array_equal(pd.diagnostic['error'], [0., 0.])
    assert pd.diagnostic['filtered'][2 if axis == 'gx' else 3] == pytest.approx(.2)


def test_limit_applies_to_actual_cad_displacement_not_only_requested_dz():
    pd = BodyStabilizer(dict(kp_roll=2., kp_pitch=2., max_error_rad=.6,
                            angle_alpha=0., gyro_alpha=0.))
    try:
        result = settle(pd, roll=10., pitch=-10.)
        delta = pd.feet(result)-pd.feet(NOMINAL)
        assert np.linalg.norm(delta, axis=1).max() <= .005+1e-8
        np.testing.assert_allclose(delta[:, :2], 0., atol=1e-6)
        np.testing.assert_allclose(delta[:, 2], pd.diagnostic['applied_dz'], atol=1e-6)
    finally:
        pd.close()


def test_off_initially_is_bit_identical_to_float32_nominal(pd):
    pd.enabled = False
    nominal = np.tile([.1234567, 47.987654, 89.765432], 4)
    result = pd.apply(nominal, frame(), sample(0, 1, roll=4., gx=.1), 0)
    np.testing.assert_array_equal(result, nominal.astype(np.float32))


@pytest.mark.parametrize('reason', ['off', 'stale', 'unverified'])
def test_disable_paths_fade_latent_correction_at_fixed_stance(pd, reason):
    result = settle(pd, roll=4.)
    old_u = np.asarray(pd.diagnostic['u_applied'])
    assert abs(old_u[0]) > .005
    if reason == 'off':
        pd.enabled = False
    for i in range(1, 20):
        now = .48+i*.02
        packet = sample(now, 25+i, roll=4.)
        if reason == 'stale':
            packet = sample(.1, 1, roll=4.)
        if reason == 'unverified':
            packet['gyro_axis_verified'] = False
        result = pd.apply(NOMINAL, frame(), packet, now)
        new_u = np.asarray(pd.diagnostic['u_applied'])
        assert np.max(abs(new_u-old_u)) <= pd.config.slew_rad_s*.02+1e-8
        np.testing.assert_array_equal(result.reshape(4, 3)[[1, 2]], NOMINAL.reshape(4, 3)[[1, 2]])
        old_u = new_u
    np.testing.assert_array_equal(result, NOMINAL)


@pytest.mark.parametrize('boundary', [.52, 1.])
def test_stance_boundary_correction_tends_continuously_to_zero(pd, boundary):
    settle(pd, roll=4.)
    for i, phase in enumerate([boundary-1e-5, boundary+1e-5]):
        t=.5+i*.02
        result=pd.apply(NOMINAL, frame(phase), sample(t, 26+i, roll=4.), t)
        assert abs(pd.diagnostic['applied_dz'][0]) < 1e-8
        np.testing.assert_allclose(result[:3], NOMINAL[:3], atol=1e-6)


@pytest.fixture
def robot():
    from cad_physics import Simulation
    from virtual_robot import RobotController
    root = Path(__file__).parent
    parameters=json.loads((root/'cad_300mm/physics_parameters_measured_total_2754g.json').read_text())
    parameters['foot_cushion']=json.loads((root/'foot_cushion_10mm.json').read_text())
    instance=RobotController(Simulation(parameters))
    instance.select_profile('attitudepd')
    yield instance
    instance.body_stabilizer.close()


def test_simulator_stop_starts_from_full_corrected_command_without_qpos_mutation(robot):
    robot.begin(('drive',), 0.)
    robot.command_target=robot.target+np.tile([.2, .3, .4], 4)
    corrected=robot.command_target.copy()
    qpos=robot.plant.data.qpos.copy();qvel=robot.plant.data.qvel.copy()
    robot.finish_stop('requested')
    np.testing.assert_array_equal(robot.transition[0], corrected)
    np.testing.assert_array_equal(robot.transition[1], np.tile([0, 45, 90], 4))
    np.testing.assert_array_equal(robot.plant.data.qpos, qpos)
    np.testing.assert_array_equal(robot.plant.data.qvel, qvel)
    assert robot.torque


def test_invalid_runtime_toggle_does_not_change_enabled_state(robot):
    for command in ('@B 3', '@B -1', '@B 1 junk', 'stabilize maybe'):
        robot.command(command, 0.)
        assert robot.body_stabilizer.enabled
        assert b'ERROR:' in robot.drain()
    robot.command('@B 0', 0.)
    assert not robot.body_stabilizer.enabled
    robot.command('@B 2', 0.)
    assert not robot.body_stabilizer.enabled
    robot.command('@B 1', 0.)
    assert robot.body_stabilizer.enabled


def drive_frame(robot):
    robot.motion=('drive',)
    robot.transition=None
    robot.phase=.25
    robot.elapsed=1.
    robot.linear=1.
    robot.request=(1., 0.)
    robot.last_packet=0.


def advance_nominal(robot):
    robot.phase=(robot.phase+.02/1.35) % 1
    robot.elapsed+=.02
    return NOMINAL.copy()


def test_simulator_corrects_target_phase_and_bypasses_legacy_balance(robot, monkeypatch):
    import virtual_robot
    drive_frame(robot)
    calls=[]
    def correction(target, control_frame, reading, now, **kwargs):
        calls.append((target.copy(), control_frame.copy()))
        return target.copy()
    def unexpected_balance(*args, **kwargs):
        pytest.fail('The PD policy must bypass legacy joint correction completely')
    monkeypatch.setattr(virtual_robot, 'shared_drive_step', advance_nominal)
    monkeypatch.setattr(robot.body_stabilizer, 'apply', correction)
    monkeypatch.setattr(robot.balance, 'apply', unexpected_balance)
    robot.balance.integral[:]=1
    robot.balance.correction[:]=1
    robot.tick(0.)
    assert len(calls)==1 and calls[0][1]['phase']==.25
    assert robot.phase != calls[0][1]['phase']
    np.testing.assert_array_equal(calls[0][0], NOMINAL)
    np.testing.assert_array_equal(robot.balance.integral, 0.)
    np.testing.assert_array_equal(robot.balance.correction, 0.)


def test_simulator_ik_failure_holds_full_previous_command_and_torque(robot, monkeypatch):
    import virtual_robot
    drive_frame(robot)
    previous=NOMINAL+np.tile([.1, .2, -.1], 4)
    robot.command_target=previous.copy()
    def failed(*args, **kwargs):
        raise ValueError('Injected IK infeasibility')
    monkeypatch.setattr(virtual_robot, 'shared_drive_step', advance_nominal)
    monkeypatch.setattr(robot.body_stabilizer, 'apply', failed)
    robot.tick(0.)
    np.testing.assert_array_equal(robot.command_target, previous)
    np.testing.assert_array_equal(robot.target, previous)
    assert robot.motion is None and robot.transition is None
    assert robot.safety=='planner' and robot.torque
    assert b'command held, torque preserved' in robot.drain()


@pytest.mark.parametrize('bad', [True, 100., -1, 0, 1001, 2**32+100])
def test_host_does_not_accept_timeout_wrap_or_wrong_type(bad):
    with pytest.raises(ValueError):
        BodyStabilizer({'timeout_ms': bad})


@pytest.mark.parametrize('bad', [True, 1., 0, 2, '1'])
def test_host_schema_validation_matches_generator(bad):
    with pytest.raises(ValueError):
        BodyStabilizer({'schema_version': bad})
