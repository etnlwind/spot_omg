"""Trajectory continuity and repeatability of the interactive reset path."""
import numpy as np
import json
from optimize_cad_gait import targets, INITIAL, make_scenario, evaluate, RESULTS, NAMES
from cad_physics import Simulation
from gait_lab import reset_simulation


def test_foot_transition_has_continuous_joint_position_and_velocity():
    p=INITIAL.copy();period,duty=p[:2];h=1e-5
    for boundary in (0.,period*duty,period*.5,period*((.5+duty)%1)):
        before=targets(p,boundary-h);at=targets(p,boundary);after=targets(p,boundary+h)
        assert np.max(abs(after-before))<.02
        np.testing.assert_allclose((at-before)/h,(after-at)/h,atol=.1)


def test_replay_reset_reuses_data_and_resets_actuator_history():
    p,model=make_scenario();sim=Simulation(p,model);data=sim.data
    initial=data.qpos.copy()
    for _ in range(30):sim.step(.6,balance=False)
    first=data.qpos.copy()
    sim=reset_simulation(p,model,data)
    assert sim.data is data
    np.testing.assert_array_equal(data.qpos,initial)
    assert data.time==0 and sim.phase==0
    assert np.all(sim.target_velocity==0)
    for _ in range(30):sim.step(.6,balance=False)
    np.testing.assert_array_equal(data.qpos,first)


def test_selected_gait_improves_speed_with_bounded_tilt_and_motor_load():
    selected=json.loads((RESULTS/'selected.json').read_text())
    params=np.array([selected['params'][k] for k in NAMES])
    p,model=make_scenario()
    baseline=evaluate(None,p,model,duration=20,baseline=True)
    result=evaluate(params,p,model,duration=20)
    assert not result['fallen']
    assert result['speed_m_s']>baseline['speed_m_s']*3
    assert result['cost_of_transport']<baseline['cost_of_transport']
    assert result['peak_tilt_deg']<10
    assert abs(result['yaw_deg'])<10
    assert result['above_rated_fraction']<.35


def test_scenario_models_do_not_overwrite_each_others_physics():
    p1,m1=make_scenario('nominal')
    expected_mass=m1.body_mass.sum()
    p2,m2=make_scenario('heavy_slippery')
    assert m1.body_mass.sum()==expected_mass
    assert m2.body_mass.sum()>expected_mass
    assert p1['pack_open_circuit_voltage']==11.1
    assert p2['pack_open_circuit_voltage']==10.8
