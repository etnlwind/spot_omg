import json
from pathlib import Path
from types import SimpleNamespace
import mujoco
import numpy as np
import pytest
from cad_physics import build
from search_gait_profiles import physics
from footstep_tracker import FootstepTracker

PARAMS=[4.8,.7,.14,.04,.24,-.01,.75]

@pytest.fixture
def tracker():
    p,_=physics();p['foot_cushion']=json.loads(Path(__file__).with_name('foot_cushion_10mm.json').read_text())
    xml,_=build(p,write_scene=False)
    t=FootstepTracker(mujoco.MjModel.from_xml_string(xml))
    q=t.plan(PARAMS,0,0,0,0,{})
    t.observe(q,SimpleNamespace(filtered=[0,0],failures=0),reset_contacts=True)
    return t


def test_requested_velocity_determines_half_stance_reach(tracker):
    tracker.plan(PARAMS,.69,1,1,0,{})
    tracker.plan(PARAMS,.701,1,1,0,{})
    assert tracker.diagnostic['requested_speed_m_s']==pytest.approx(.14/4.8)
    for leg in (0,3):
        assert tracker.ends[leg,0]-tracker.base[0]-tracker.reference[leg,0]==pytest.approx(.07)


def test_liftoff_is_latched_from_measured_foot_position(tracker):
    measured=tracker.feet.copy();measured[0,0]+=.013
    tracker.feet=measured
    tracker.plan(PARAMS,.701,1,1,0,{})
    np.testing.assert_array_equal(tracker.starts[0],measured[0])
    start=tracker.starts.copy();tracker.feet+=.01
    tracker.plan(PARAMS,.75,1,1,0,{})
    np.testing.assert_array_equal(tracker.starts,start)


def test_front_and_rear_have_common_world_swing_height(tracker):
    tracker.plan(PARAMS,.701,1,1,0,{})
    tracker.plan(PARAMS,.85,1,1,0,{})
    feet=np.array(tracker.diagnostic['target_feet_m'])
    assert feet[0,2]==pytest.approx(.04,abs=.001)
    assert feet[3,2]==pytest.approx(.04,abs=.001)
    np.testing.assert_array_equal(feet[[1,2]],tracker.anchors[[1,2]])
    assert max(abs(np.array(tracker.diagnostic['body_shift_m'])[1:]))<=.0100001


def test_sensor_loss_holds_last_target_and_exposes_hold(tracker):
    q=tracker.plan(PARAMS,.8,1,1,0,{})
    tracker.observe(None,None)
    np.testing.assert_array_equal(tracker.plan(PARAMS,.9,1,1,0,{}),q)
    assert tracker.diagnostic['sensor_hold']


def test_j1_lock_is_enforced_inside_ik(tracker):
    fixed=tracker.last_target[::3].copy()
    previous=None
    changed=False
    for phase in np.linspace(.701,.95,12):
        q=tracker.plan(PARAMS,float(phase),1,1,0,{'lock_j1':True})
        np.testing.assert_array_equal(q[::3],fixed)
        if previous is not None:changed |= bool(np.max(abs(q[1::3]-previous[1::3]))>.1)
        previous=q
    assert changed
    assert tracker.diagnostic['j1_locked']
    assert np.isfinite(tracker.diagnostic['planned_residual_m'])
