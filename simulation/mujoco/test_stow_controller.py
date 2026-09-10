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
    tick(r,705)
    assert r.pose=='stow' and r.transition is None
    assert b'OK stow' in r.drain()
    np.testing.assert_allclose(r.command_target,FOLDED,atol=.01)
    assert np.max(abs(np.degrees(r.plant.data.qpos[r.plant.q])-FOLDED))<8
    assert np.all(np.degrees(r.plant.data.qpos[r.plant.q])[[1,4]] < -250)
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
