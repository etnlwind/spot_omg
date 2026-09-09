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
    assert b'balance=monitor' in robot.drain()


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
