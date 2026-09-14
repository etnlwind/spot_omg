"""Planner-contract checks; these do not prove actual contact or no-slip motion."""
import json
from pathlib import Path
from types import SimpleNamespace

import mujoco
import numpy as np
import pytest

from cad_physics import build
from calculated_placement_preview import (
    CalculatedPlacement, DUTY, PERIOD, STATIC_SHIFT, STRIDE, tilted_pose,
)
from support_shift import LEGS, SupportShift
from footstep_tracker import rotation


@pytest.fixture(scope='module')
def model():
    root = Path(__file__).parent
    params = json.loads((root/'cad_300mm/physics_parameters_measured_total_2754g.json').read_text())
    params['foot_cushion'] = json.loads((root/'foot_cushion_d37p3_l27mm.json').read_text())
    xml, _ = build(params, write_scene=False)
    return mujoco.MjModel.from_xml_string(xml)


@pytest.fixture
def goals():
    return np.array([[.2, .1, -.2], [.2, -.1, -.2],
                     [-.2, .1, -.2], [-.2, -.1, -.2]])


def good_sensor():
    return dict(orientation=np.eye(3), com=np.zeros(3), height=.2,
                velocity_valid=True, raw_velocity=np.zeros(3), velocity_age_s=0.,
                encoders=np.tile([0.,45.,90.],4), ground=-.2)


def update(control, goals, phase, elapsed=2.):
    robot=SimpleNamespace(elapsed=elapsed,
        attitude_filter=SimpleNamespace(filtered=[0.,0.],rate=[0.,0.]))
    return control.update(goals, goals, robot,
                          phase % 1., DUTY, PERIOD)


@pytest.fixture
def cad_pose(model):
    k=SupportShift(model)
    q=np.tile([0.,45.,90.],4)
    k.set_angles(q)
    goals=np.array([k.foot(i) for i in range(4)])
    goals[:,2]=goals[:,2].mean()
    for _ in range(4):
        k.set_angles(q)
        goals[:,0]=[k.data.xanchor[model.joint(leg+'_j2').id][0] for leg in LEGS]
        q,error=k.solve(goals,q,iterations=30)
        assert error<.0001
    k.set_angles(q)
    com=np.average(k.data.xipos,axis=0,weights=model.body_mass)
    sensor=good_sensor()
    sensor.update(com=com,height=float(com[2]-goals[:,2].mean()),
                  encoders=q,ground=float(goals[:,2].mean()))
    return goals,sensor


def test_baseline_and_static_diagonal_offsets_no_input_mutation(model, goals, monkeypatch):
    original = goals.copy()
    for mode in ('baseline', 'static', 'inward_prepared'):
        control = CalculatedPlacement(model, mode)
        monkeypatch.setattr(control, 'observe', lambda *args: None)
        for phase in np.arange(0., 2.01, .02/PERIOD):
            result = update(control, goals, phase)
            delta = result - goals
            np.testing.assert_allclose(delta[0], delta[3], atol=1e-15)
            np.testing.assert_allclose(delta[1], delta[2], atol=1e-15)
            np.testing.assert_array_equal(delta[:, 1:], 0.)
            if mode in ('baseline', 'inward_prepared'):
                np.testing.assert_array_equal(result, goals)
            else:
                assert np.all(delta[:, 0] >= STATIC_SHIFT[0]-1e-15)
                assert np.all(delta[:, 0] <= 1e-15)
        if mode == 'static':
            np.testing.assert_allclose(control.current, np.tile(STATIC_SHIFT, (4, 1)))
        np.testing.assert_array_equal(goals, original)


def test_stance_holds_additive_offset_until_next_swing(model, goals, monkeypatch):
    control = CalculatedPlacement(model, 'static')
    monkeypatch.setattr(control, 'observe', lambda *args: None)
    update(control, goals, .51)
    update(control, goals, .8)
    update(control, goals, .999999)
    update(control, goals, 0.)
    landed = control.current[[0, 3]].copy()
    for phase in (.1, .3, .519999):
        update(control, goals, phase)
        np.testing.assert_array_equal(control.current[[0, 3]], landed)
    np.testing.assert_allclose(landed, np.tile(STATIC_SHIFT, (2, 1)))


def test_swing_boundary_position_and_offset_rate_are_continuous(model, goals, monkeypatch):
    control = CalculatedPlacement(model, 'static')
    monkeypatch.setattr(control, 'observe', lambda *args: None)
    epsilon = 1e-5
    update(control, goals, DUTY-epsilon)
    before = control.current[[0, 3]].copy()
    update(control, goals, DUTY+epsilon)
    after = control.current[[0, 3]].copy()
    assert np.max(abs(after-before)) < 1e-10
    update(control, goals, 1.-epsilon)
    before = control.current[[0, 3]].copy()
    update(control, goals, epsilon)
    after = control.current[[0, 3]].copy()
    assert np.max(abs(after-before)) < 1e-10
    # Maximum derivative of the quintic blend is 1.875. This checks the
    # Cartesian offset step only, not resulting joint velocity/acceleration.
    control.reset()
    old = control.current.copy()
    bound = abs(STATIC_SHIFT[0])*1.875*.02/(PERIOD*(1.-DUTY))
    for phase in np.arange(0., 2., .02/PERIOD):
        update(control, goals, phase)
        assert np.max(abs(control.current-old)) <= bound+1e-12
        old = control.current.copy()


def test_dynamic_x_only_target_uses_common_acceleration_and_force_solution(model, cad_pose, monkeypatch):
    goals,sensor=cad_pose
    control = CalculatedPlacement(model, 'acceleration')
    monkeypatch.setattr(control, 'observe', lambda *args: sensor)
    control.velocity[:] = [.05, 0., 0.]
    update(control, goals, .51)
    update(control, goals, .52)
    event = control.events[-1]
    acceleration = (STRIDE/(PERIOD*DUTY)-.05)/(PERIOD*DUTY)
    solution = event['solution']
    assert solution['feasible']
    assert solution['mode'] == 'x-only'
    np.testing.assert_allclose(event['planned_acceleration_m_s2'], [acceleration, 0.])
    np.testing.assert_allclose(control.end[0],control.end[3],atol=1e-5)
    assert event['planned_touchdown_height_difference_m']<.0001
    assert not event['acceleration_limited']
    np.testing.assert_allclose(solution['moment_about_com_nm'], 0., atol=1e-12)
    # Holding a planned offset during stance is not a claim of true ground anchoring.
    update(control, goals, .99999)
    update(control, goals, 0.)
    previous = control.current[[0, 3]].copy()
    control.velocity[:] = [.3, .1, 0.]
    update(control, goals, .2)
    np.testing.assert_array_equal(control.current[[0, 3]], previous)


def test_sensor_loss_preserves_previous_endpoint_and_reset_clears_history(model, cad_pose, monkeypatch):
    goals,sensor=cad_pose
    control = CalculatedPlacement(model, 'acceleration')
    monkeypatch.setattr(control, 'observe', lambda *args: sensor)
    update(control, goals, .51)
    update(control, goals, .7)
    endpoint = control.end[[0, 3]].copy()
    before = control.current.copy()
    monkeypatch.setattr(control, 'observe', lambda *args: None)
    update(control, goals, .7)
    np.testing.assert_array_equal(control.current, before)
    update(control, goals, .99999)
    update(control, goals, 0.)
    update(control, goals, .51)
    update(control, goals, .7)
    np.testing.assert_array_equal(control.end[[0, 3]], endpoint)
    assert control.events[-1]['sensor_hold_previous_endpoint']
    assert not control.diagnostic['sensor_valid']
    control.reset()
    np.testing.assert_array_equal(control.current, 0.)
    np.testing.assert_array_equal(control.end, 0.)
    np.testing.assert_array_equal(control.velocity, 0.)
    assert control.events == [] and control.previous_sensor is None
    assert control.fault is None


@pytest.mark.parametrize('age', [.120001, float('inf')])
def test_stale_velocity_blocks_new_target_without_removing_old_offset(model, goals, monkeypatch, age):
    control = CalculatedPlacement(model, 'acceleration')
    sensor = good_sensor()
    sensor.update(velocity_valid=False, velocity_age_s=age)
    monkeypatch.setattr(control, 'observe', lambda *args: sensor)
    previous = np.tile([-.025, 0., 0.], (4, 1))
    control.current[:] = previous
    control.start[:] = previous
    control.end[:] = previous
    update(control, goals, .51)
    update(control, goals, .7)
    np.testing.assert_array_equal(control.current, previous)
    assert control.events[-1]['sensor_hold_previous_endpoint']
    assert 'solution' not in control.events[-1]


def test_acceleration_clipping_is_visible_and_infeasible_plan_is_not_applied(model, cad_pose, monkeypatch):
    goals,sensor=cad_pose
    control = CalculatedPlacement(model, 'acceleration')
    monkeypatch.setattr(control, 'observe', lambda *args: sensor)
    control.velocity[:] = [-1., 0., 0.]
    update(control, goals, .51)
    update(control, goals, .52)
    event = control.events[-1]
    assert event['acceleration_limited']
    assert event['raw_requested_acceleration_m_s2'][0] > 1.
    assert event['planned_acceleration_m_s2'][0] == 1.
    control.reset()
    # A target below the CAD leg workspace must fail via IK, not a guessed
    # 80mm Cartesian offset gate.
    invalid = goals.copy()
    invalid[:, 2] = -1.
    update(control, invalid, .51)
    update(control, invalid, .52)
    assert control.fault
    assert control.events[-1]['planning_failure']=='unreachable nominal touchdown'
    np.testing.assert_array_equal(control.current, 0.)


def test_sensor_boundary_is_quantized_and_delayed_and_invalid_imu_is_rejected(model):
    control = CalculatedPlacement(model, 'acceleration')
    qpos = np.zeros(model.nq)
    robot = SimpleNamespace(
        elapsed=1.,
        plant=SimpleNamespace(data=SimpleNamespace(qpos=qpos,time=10.), q=control.kin.q),
        imu_reading={'age_ms': 0},
        attitude_filter=SimpleNamespace(filtered=[0., 0.], failures=0))
    first = np.tile([0., 45.021, 90.021], 4)
    for index in range(3):
        qpos[control.kin.q] = np.radians(first + [0., .1, .2][index])
        packet = control.observe(robot, .2, DUTY, PERIOD)
        if index < 2:
            assert packet is None
    assert packet is not None
    expected = np.round(first*4096/360)*360/4096
    np.testing.assert_allclose(np.degrees(control.kin.data.qpos[control.kin.q]), expected)
    assert not packet['velocity_valid']  # First delayed packet has no derivative.
    robot.imu_reading['age_ms'] = 101
    assert control.observe(robot, .2, DUTY, PERIOD) is None
    assert control.previous_sensor is None
    robot.imu_reading['age_ms'] = 0
    robot.attitude_filter.failures = 1
    assert control.observe(robot, .2, DUTY, PERIOD) is None


def test_motion_reset_keeps_idle_sensor_history_and_monotonic_age(model):
    control=CalculatedPlacement(model,'acceleration')
    qpos=np.zeros(model.nq)
    qpos[control.kin.q]=np.radians(np.tile([0.,45.,90.],4))
    data=SimpleNamespace(qpos=qpos,time=10.)
    robot=SimpleNamespace(elapsed=10.,plant=SimpleNamespace(data=data,q=control.kin.q),
        imu_reading={'age_ms':0},attitude_filter=SimpleNamespace(filtered=[0.,0.],failures=0))
    for _ in range(4):
        packet=control.observe(robot,.2,DUTY,PERIOD)
    assert packet['velocity_valid'] and packet['velocity_age_s']==0.
    channel=control.channel;history=control.previous_sensor
    control.current[:]=.01
    control.reset_motion()
    assert control.channel is channel and control.previous_sensor is history
    np.testing.assert_array_equal(control.current,0.)
    assert control.velocity_updated_s==10.
    # A motion elapsed-time reset cannot make sensor freshness negative.
    robot.elapsed=0.;data.time=10.14
    control.previous_sensor=None
    packet=control.observe(robot,.2,DUTY,PERIOD)
    assert packet['velocity_age_s']==pytest.approx(.14)


@pytest.mark.parametrize('angles_deg',[(0.,0.),(3.,-2.)])
def test_dynamic_landing_geometry_is_common_world_plane_with_tilt(model,cad_pose,angles_deg):
    goals,sensor=cad_pose
    control=CalculatedPlacement(model,'acceleration')
    ids=np.array([0,3])
    R=rotation(*np.radians(angles_deg))
    sensor['orientation']=R
    robot=SimpleNamespace(elapsed=2.,attitude_filter=SimpleNamespace(
        filtered=np.array(angles_deg)*10.,rate=[0.,0.]))
    prediction=goals.copy()
    prediction[ids,0]+=.5*STRIDE
    prediction[[1,2],0]+=STRIDE*(.5-.5/DUTY)
    seed,error=control.kin.solve(prediction,sensor['encoders'],iterations=30)
    assert error<.001
    tilted_pose(control.kin,seed,R)
    before=np.array([control.kin.foot(i) for i in ids])
    offset,entry=control.dynamic_endpoint(goals,ids,robot,sensor,2.,PERIOD,DUTY)
    assert offset is not None,entry
    after=np.array(entry['planned_world_touchdown_m'])
    np.testing.assert_allclose(after[:,2],sensor['ground'],atol=.0001)
    np.testing.assert_allclose(after[:,1],before[:,1],atol=.0001)
    np.testing.assert_allclose((after-before)[0,0],(after-before)[1,0],atol=.0001)
    np.testing.assert_allclose(entry['solution']['moment_balance_error_nm'],0.,atol=1e-12)
    assert entry['touchdown_ik_residual_m']<.0001
    if any(angles_deg):
        # Equal gravity-frame landing height generally requires different
        # body-Z offsets and hence different joint angles for opposite legs.
        assert abs(offset[0,2]-offset[1,2])>.01


def test_tilted_floor_reference_uses_rotated_cushion_vertices(model):
    control = CalculatedPlacement(model, 'acceleration')
    q = np.tile([0., 45., 90.], 4)
    q = np.round(q*4096/360)*360/4096
    k = control.kin
    k.set_angles(q)
    R = rotation(np.radians(10.), np.radians(-5.))
    com = np.average(k.data.xipos, axis=0, weights=model.body_mass) @ R.T
    support = [0, 3]  # At phase .2 the delayed FL/RR pair is scheduled stance.
    bottoms = []
    old_hybrid = []
    for leg in support:
        geom = k.feet[leg]
        vertices = (k.vertices[leg] @ k.data.geom_xmat[geom].reshape(3, 3).T
                    + k.data.geom_xpos[geom]) @ R.T
        bottoms.append(vertices[:, 2].min())
        old_hybrid.append((k.foot(leg) @ R.T)[2])
    expected_height = com[2] - np.median(bottoms)
    old_height = com[2] - np.median(old_hybrid)
    assert abs(expected_height-old_height) > .0001  # Old hybrid rotation is detectably wrong.
    qpos = np.zeros(model.nq)
    qpos[k.q] = np.radians(q)
    robot = SimpleNamespace(elapsed=1.,
        plant=SimpleNamespace(data=SimpleNamespace(qpos=qpos), q=k.q),
        imu_reading={'age_ms': 0},
        attitude_filter=SimpleNamespace(filtered=[100., -50.], failures=0))
    for _ in range(3):
        packet = control.observe(robot, .2, DUTY, PERIOD)
    assert packet['height'] == pytest.approx(expected_height, abs=1e-12)


def test_reference_acceleration_is_derivative_of_requested_velocity():
    h = 1e-6
    for elapsed in (.1, .3, .5, .8, 1.1):
        speed, acceleration = CalculatedPlacement.reference(elapsed)
        numeric = (CalculatedPlacement.reference(elapsed+h)[0]
                   - CalculatedPlacement.reference(elapsed-h)[0])/(2*h)
        assert acceleration == pytest.approx(numeric, abs=1e-9)
        assert speed >= 0.
    assert CalculatedPlacement.reference(-1.) == (0., 0.)
    assert CalculatedPlacement.reference(2.) == pytest.approx((STRIDE/(PERIOD*DUTY), 0.))


def test_inward_entry_preserves_stance_and_xz_and_mirrors_diagonal_pair(model,goals,monkeypatch):
    control=CalculatedPlacement(model,'inward')
    monkeypatch.setattr(control,'observe',lambda *args:None)
    previous=np.zeros((4,3));previous_stance=np.ones(4,dtype=bool)
    max_step=.010*1.875*.02/(PERIOD*(1-DUTY))
    for phase in np.arange(0.,2.,.02/PERIOD):
        output=update(control,goals,phase)
        delta=output-goals
        np.testing.assert_array_equal(delta[:,[0,2]],0.)
        np.testing.assert_allclose(delta[0,1],-delta[3,1],atol=1e-15)
        np.testing.assert_allclose(delta[1,1],-delta[2,1],atol=1e-15)
        assert np.all(delta[:,1]*goals[:,1]<=1e-15)
        assert np.max(abs(delta))<=.010+1e-15
        assert np.max(abs(delta-previous))<=max_step+1e-12
        # The new width is not applied abruptly to already supporting legs.
        both_stance=previous_stance&control.stance
        np.testing.assert_allclose(delta[both_stance],previous[both_stance],atol=1e-15)
        previous=delta.copy();previous_stance=control.stance.copy()
    np.testing.assert_allclose(control.current[:,1],[-.010,.010,-.010,.010])
    control.reset_motion()
    np.testing.assert_array_equal(control.current,0.)


def test_inward_cad_j1_adduction_keeps_foot_height_and_foreaft_goals(model,cad_pose):
    goals,sensor=cad_pose
    target=goals.copy();target[:,1]-=np.sign(target[:,1])*.010
    kin=SupportShift(model)
    q,error=kin.solve(target,sensor['encoders'],iterations=30)
    assert error<.0001
    kin.set_angles(q)
    actual=np.array([kin.foot(i) for i in range(4)])
    np.testing.assert_allclose(actual,target,atol=.0001)
    delta=q.reshape(4,3)-sensor['encoders'].reshape(4,3)
    # Actual CAD sign: a negative canonical J1 increment brings each foot inward.
    assert np.all(delta[:,0]<-2.) and np.all(delta[:,0]>-4.)
    assert np.max(abs(delta[:,1:]))<3.
