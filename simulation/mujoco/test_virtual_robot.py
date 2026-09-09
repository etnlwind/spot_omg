import json
import numpy as np
import pytest
from cad_gait import CAD
from cad_physics import Simulation
from virtual_robot import RobotController


@pytest.fixture
def robot():
    p=json.loads((CAD/'physics_parameters.json').read_text())
    p['timestep_s']=.0005
    return RobotController(Simulation(p))


def tick(robot, count, start=0):
    for n in range(count): robot.tick(start+n*.02)


def test_identity_and_unsupported_are_honest(robot):
    robot.command('identity',0)
    assert b'backend=sim' in robot.drain()
    robot.command('balance on',0)
    assert b'ERROR:' in robot.drain()
    robot.command('syncstate',0)
    assert b'balance=suspended' in robot.drain()


def test_motion_uses_torque_not_teleport(robot):
    before=robot.plant.data.qpos.copy()
    robot.command('landing',0)
    np.testing.assert_array_equal(before, robot.plant.data.qpos)
    assert b'OK' not in robot.drain()
    tick(robot,51)
    assert robot.plant.data.time == pytest.approx(1.02)
    assert not np.allclose(before,robot.plant.data.qpos)
    assert np.any(robot.plant.data.ctrl)
    assert b'OK landing' in robot.drain()


def test_relax_turns_off_actuator_force(robot):
    robot.command('relax',0); robot.tick(0)
    np.testing.assert_array_equal(robot.plant.data.ctrl, np.zeros(12))


def test_watchdog_stops_and_completes_after_transition(robot):
    robot.command('drive 600 0 1',0)
    assert b'started' in robot.drain()
    robot.tick(.81)
    assert robot.motion is None
    assert b'stopped' not in robot.drain()
    tick(robot,51, .83)
    assert b'reason=watchdog' in robot.drain()


def test_packet_sequence_and_input_validation(robot):
    robot.command('drive 600 0 10',0); robot.drain()
    robot.command('@D 9 -600 0',.1)
    assert robot.request == (.6,0)
    robot.command('@D 11 900 900',.2)
    assert robot.request == (.9,.5)
    robot.command('@D 12 1001 0',.3)
    assert robot.request == (.9,.5)
    assert b'ERROR:' in robot.drain()
    robot.command('@S 10',.4)
    assert robot.motion is not None
    robot.command('@S 12',.4)
    assert robot.motion is None


def test_disconnect_stops_without_waiting_for_network(robot):
    robot.command('drive 600 0 1',0)
    robot.disconnected()
    assert robot.motion is None and robot.transition is not None


def test_shared_drive_shaping_matches_embedded_integer_rules(robot):
    p=robot.plant.policy
    assert p.drive_slew(0,1000)==40
    assert p.drive_slew(-50,-60)==-60
    assert p.drive_period_ms(0,0)==2400
    assert p.drive_period_ms(333,0)==2201
    assert p.drive_period_ms(1000,500)==1800


def test_profile_selection_is_idle_only_and_reported(robot):
    robot.command('simprofile highstep',0)
    assert robot.profile == 'highstep'
    robot.command('syncstate',0)
    assert b'profile=highstep' in robot.drain()
    robot.command('drive 600 0 1',0)
    robot.command('simprofile crawl',0)
    assert robot.profile == 'highstep'
    assert b'ERROR: busy' in robot.drain()


def test_unknown_profile_does_not_replace_current(robot):
    profile=robot.profile
    robot.command('simprofile missing',0)
    assert robot.profile == profile
    assert b'ERROR:' in robot.drain()


@pytest.mark.parametrize('profile', ['legacy','cruise','trot'])
def test_timed_gui_demo_completes_without_drive_heartbeat(robot, profile):
    tick(robot,60)
    robot.command('simprofile '+profile,1.2)
    robot.drain()
    robot.command('simwalk 1',1.2)
    tick(robot,250,1.2)
    assert robot.motion is None and robot.transition is None
    assert b'OK' in robot.drain()


def test_sensor_fault_stops_after_three_failed_polls(robot):
    tick(robot,60)
    robot.command('drive 600 0 1',1.2)
    robot.imu.online=False
    robot.tick(1.20);robot.tick(1.22)
    assert robot.safety=='ok'
    robot.tick(1.24)
    assert robot.safety=='imu' and robot.motion is None


def test_sensor_bias_drives_tilt_safety_without_ground_truth_tilt(robot):
    from bno055_emulator import BNO055Emulator,BNO055Config
    robot.imu=BNO055Emulator(BNO055Config(roll_bias_deg=15,noise_std_deg=0))
    tick(robot,60)
    assert robot.safety=='tilt'  # standing balance also monitors tilt
    robot.command('drive 600 0 1',1.2)
    assert robot.motion is None
    assert abs(robot.plant.row()['roll_deg'])<5
    robot.command('imudiag',1.24)
    assert b'BNO055' in robot.drain()


def test_balance_enabled_by_default_and_suspended_for_landing(robot):
    tick(robot,60)
    assert robot.balance.applied
    robot.command('syncstate',1.2)
    assert b'balance=active' in robot.drain()
    robot.command('landing',1.2)
    tick(robot,60,1.2)
    assert not robot.balance.applied
    robot.command('stand',2.4)
    tick(robot,60,2.4)
    assert robot.balance.applied
    robot.command('simbalance off',3.6)
    tick(robot,30,3.6)
    assert not robot.balance.applied
    robot.command('syncstate',4.2)
    assert b'balance=off' in robot.drain()


def test_stop_transition_preserves_level_control(robot):
    tick(robot,60)
    robot.command('simwalk 1',1.2)
    tick(robot,100,1.2)
    robot.finish_stop('requested') if robot.motion else None
    robot.tick(3.2)
    assert robot.balance.applied
    tick(robot,80,3.22)
    assert robot.pose=='stand' and robot.balance.applied


def test_idle_stop_is_acknowledged_after_rejected_drive(robot):
    robot.safety='tilt'
    robot.command('drive 998 0 1',0)
    assert b'ERROR' in robot.drain()
    robot.command('@S 2',0)
    assert b'reason=already-stopped' in robot.drain()
    assert robot.safety=='tilt'


def test_fault_saves_sensor_and_control_history(robot,tmp_path):
    import json
    robot.incident_directory=tmp_path
    tick(robot,60)
    robot.imu.online=False
    tick(robot,3,1.2)
    assert robot.safety=='imu'
    files=list(tmp_path.glob('incident-*.json'))
    assert len(files)==1
    report=json.loads(files[0].read_text())
    assert report['reason']=='imu'
    assert len(report['frames'])==63
    assert 'balance' in report['frames'][-1]
    tick(robot,5,1.26)
    assert len(list(tmp_path.glob('incident-*.json')))==1


def test_motion_uses_pi_and_standing_retains_d(robot):
    tick(robot,60)
    assert robot.balance.kd>0
    robot.command('drive 998 0 1',1.2)
    robot.tick(1.2)
    assert robot.balance.kd==0
    assert robot.balance.kp==.1


@pytest.mark.parametrize('profile,limit', [('trot',600),('highstep',600),('cruise',1000)])
def test_reverse_envelope_is_reported_and_applies_to_drive_and_packets(robot,profile,limit):
    robot.select_profile(profile)
    robot.command('syncstate',0)
    assert f'reverse_limit={limit}'.encode() in robot.drain()
    robot.command('drive -1000 0 1',0)
    assert robot.request[0]==-limit/1000
    robot.command('@D 2 -1000 0',.2)
    assert robot.request[0]==-limit/1000
    robot.command('@D 3 1000 0',.4)
    assert robot.request[0]==1


def test_directional_cruise_blends_without_neutral_discontinuity(robot):
    forward=[.8,.64,.065,.007,.20175,-.035,.75]
    base=[1.05,.6,.08,.012,.20175,-.035,.75]
    robot.profiles['cruise']['params']=forward
    robot.profiles['cruise']['turn_reverse_params']=base
    robot.linear=-1
    np.testing.assert_allclose(robot.active_profile_params(),base)
    robot.linear=1
    np.testing.assert_allclose(robot.active_profile_params(),forward)
    robot.linear=.25
    np.testing.assert_allclose(robot.active_profile_params(),(np.array(base)+forward)/2)
    robot.linear=0
    before=robot.active_profile_params()
    robot.linear=1e-5
    np.testing.assert_allclose(robot.active_profile_params(),before,atol=1e-10)
