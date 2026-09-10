import json
import numpy as np
from cad_gait import CAD
from cad_physics import Simulation
from virtual_robot import RobotController
from stow_policy import LANDING, FOLDED


def controller(enabled=True):
    p=json.loads((CAD/'physics_parameters.json').read_text())
    p.update(timestep_s=.0005,experimental_stow=enabled)
    return RobotController(Simulation(p))


def tick(r,count):
    for _ in range(count):r.tick(float(r.plant.data.time))


def test_stow_roundtrip_uses_physics_and_restores_encoding():
    r=controller();before=r.plant.data.qpos.copy()
    r.command('stow',0)
    np.testing.assert_array_equal(r.plant.data.qpos,before)
    assert r.stow_path and r.plant.stow_active
    for _ in range(705):
        tick(r,1)
        if r.pose=='stow':break
    assert r.pose=='stow' and r.transition is None
    assert b'OK stow' in r.drain()
    np.testing.assert_allclose(r.command_target,FOLDED,atol=.01)
    assert np.max(abs(np.degrees(r.plant.data.qpos[r.plant.q])-FOLDED))<8
    assert not r.torque
    tick(r,150)
    np.testing.assert_array_equal(r.plant.data.ctrl,np.zeros(12))
    assert not r.balance.applied
    r.command('drive 500 0 1',15)
    assert b'ERROR' in r.drain() and r.motion is None
    r.command('landing',15);tick(r,705)
    assert r.pose=='landing' and not r.stow_path and not r.plant.stow_active
    assert np.max(abs(np.degrees(r.plant.data.qpos[r.plant.q])-LANDING))<3


def test_stow_disconnect_and_hold_stop_transition_and_allow_unfold():
    r=controller();r.command('stow',0);tick(r,10)
    r.disconnected()
    assert r.transition is None and not r.stow_queue and r.pose=='stow-paused'
    r.command('landing',1)
    assert r.transition[3]==12
    r.command('hold',1)
    assert r.transition is None and r.pose=='stow-paused'
    r.command('stand',1)
    assert b'ERROR' in r.drain()


def test_normal_simulator_cannot_advertise_or_execute_stow():
    r=controller(False);r.command('syncstate',0)
    assert b'simstow' not in r.drain()
    r.command('stow',0)
    assert b'ERROR' in r.drain() and not r.stow_path


def test_emergency_interrupt_cancels_midfold_without_pose_teleport_or_resume():
    r=controller();r.command('stow',0);tick(r,400)
    before=r.plant.data.qpos.copy();velocity=r.plant.data.qvel.copy()
    r.command('\x03',float(r.plant.data.time))
    np.testing.assert_array_equal(r.plant.data.qpos,before)
    np.testing.assert_array_equal(r.plant.data.qvel,velocity)
    held=r.target.copy()
    assert not r.stow_queue and r.transition is None
    assert r.pose=='stow-paused' and r.torque
    assert b'STOPPED:' in r.drain()
    for target in r.plant.delay:np.testing.assert_allclose(np.degrees(target),held)
    tick(r,100)
    np.testing.assert_array_equal(r.target,held)
    assert r.transition is None and r.motion is None
    assert b'OK stow' not in r.drain()


def test_stow_can_resume_either_direction_from_interrupted_position():
    for next_command, expected in [('stow',FOLDED),('landing',LANDING)]:
        r=controller();r.command('stow',0);tick(r,400)
        r.command('\x03',float(r.plant.data.time));tick(r,25)
        before=r.plant.data.qpos.copy()
        held=np.degrees(before[r.plant.q]) if next_command=='landing' else r.target.copy()
        r.command(next_command,float(r.plant.data.time))
        np.testing.assert_array_equal(r.plant.data.qpos,before)
        np.testing.assert_array_equal(r.transition[0],held)
        assert r.transition[3]==12
        for _ in range(705):
            tick(r,1)
            if r.pose==next_command:break
        assert r.pose==next_command
        assert np.max(abs(np.degrees(r.plant.data.qpos[r.plant.q])-expected))<8


def test_unfold_restarts_from_gravity_settled_pose_and_reenables_torque():
    r=controller();r.command('stow',0);tick(r,850)
    assert r.pose=='stow' and not r.torque
    before=r.plant.data.qpos.copy()
    measured=np.degrees(before[r.plant.q])
    assert np.max(abs(measured-FOLDED))>8  # gravity moved the unpowered joints
    r.command('landing',float(r.plant.data.time))
    np.testing.assert_array_equal(r.plant.data.qpos,before)
    np.testing.assert_allclose(r.transition[0],measured)
    assert r.torque
    for target in r.plant.delay:np.testing.assert_allclose(np.degrees(target),measured)
    tick(r,705)
    assert r.pose=='landing' and not r.stow_path


def test_failed_stow_completion_does_not_keep_driving_unreached_goal():
    r=controller();r.command('stow',0)
    r.stow_queue.clear()
    # Simulate a blocked actuator at completion; the measured pose is far away.
    r.transition=(r.target.copy(),np.array(FOLDED),11.99,12.,'OK stow')
    tick(r,1)
    assert r.pose=='stow-paused' and r.transition is None and not r.torque
    assert b'ERROR:' in r.drain()
    np.testing.assert_array_equal(r.plant.data.ctrl,np.zeros(12))
