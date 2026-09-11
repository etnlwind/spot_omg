import json
from pathlib import Path
import numpy as np
import mujoco
import pytest
from cad_physics import build
from search_gait_profiles import physics
from support_shift import SupportShift,transfer_gate,swing_path

@pytest.fixture(scope='module')
def controller():
    p,_=physics();p['foot_cushion']=json.loads(Path(__file__).with_name('foot_cushion_10mm.json').read_text())
    xml,p=build(p,write_scene=False)
    return SupportShift(mujoco.MjModel.from_xml_string(xml))

PARAMS=[4.8,.7,.14,.04,.24,-.01,.75]
CONFIG=dict(lateral_m=.01,lower_m=.01,feedback_gain=.15,ki=0)

def test_static_preload_uses_gravity_sign_and_resets(controller):
    from types import SimpleNamespace
    cfg=dict(CONFIG,lock_j1=True,load_preload_gain=1.)
    q=controller.plan(PARAMS,.8,1,1,0,cfg)
    controller.reset()
    delta=controller.load_preload(q,PARAMS,cfg)
    assert np.max(abs(delta))<=.25
    np.testing.assert_array_equal(delta[::3],0.)
    # Swing leg has no assigned support force: its torque is the derivative
    # of gravitational potential. Verify independently by finite differences.
    torque=controller.diagnostic['load_torque_estimate_nm']
    for j in (1,2):
        energies=[]
        for sign in (-1,1):
            test=q.copy();test[j]+=sign*.001;controller.set_angles(test)
            energies.append(-np.sum(controller.model.body_mass[:,None]*
                controller.data.xipos*controller.model.opt.gravity))
        derivative=(energies[1]-energies[0])/np.radians(.002)
        assert torque[j]==pytest.approx(derivative,abs=1e-6)
    controller.feedback(q,None,SimpleNamespace(filtered=[0,0],failures=0),PARAMS,cfg,True)
    np.testing.assert_array_equal(controller.load_correction,0.)

def test_diagonal_schedule_has_no_four_foot_wait(controller):
    params=[1.44,.5,.06,.02,.20175,-.01,.75]
    cfg=dict(CONFIG,lock_j1=True,constant_body_height=True,
             lateral_m=0,kinematic_lead_s=.04)
    for phase in np.linspace(0,1,31,endpoint=False):
        controller.plan(params,phase,1,1,0,cfg)
        stance=controller.diagnostic['scheduled_stance']
        assert sum(stance)==2
        assert stance[0]==stance[3] and stance[1]==stance[2]
        assert controller.last_phase==phase
        assert controller.diagnostic['body_shift_m'][2]==0.

def test_preparation_and_release_are_continuous():
    window=.2/4.8
    assert transfer_gate(.7-window,.7,window)==pytest.approx(0)
    assert transfer_gate(.7,.7,window)==pytest.approx(1)
    assert transfer_gate(window,.7,window)==pytest.approx(0)
    for boundary in [0,window,.7-window,.7,1]:
        a=transfer_gate((boundary-1e-6)%1,.7,window)
        b=transfer_gate((boundary+1e-6)%1,.7,window)
        assert abs(a-b)<1e-7

def test_full_stride_and_common_cushion_clearance(controller):
    x0,_=swing_path(0,.7);x1,_=swing_path(.7,.7)
    assert .14*(x0-x1)==pytest.approx(.14)
    config=dict(CONFIG,lateral_m=0,lower_m=0)
    for phase in [0,.3,.69,.7,.8,.9,.99]:
        q=controller.plan(PARAMS,phase,1,1,0,config)
        assert controller.diagnostic['planned_residual_m']<.0002
        positions=np.array([controller.foot(i) for i in range(4)])
        for pair in [(0,3),(1,2)]:
            assert positions[pair[0],2]==pytest.approx(positions[pair[1],2],abs=.0002)
        assert np.all(q.reshape(4,3)>=[-30,-45,0])
        assert np.all(q.reshape(4,3)<=[30,100,150])

def test_body_shift_bounds_and_phase_wrap(controller):
    for phase in np.linspace(0,1,25):
        controller.plan(PARAMS,phase%1,1,1,0,CONFIG)
        assert max(abs(np.array(controller.diagnostic['body_shift_m'])))<=.01000001
    a=controller.plan(PARAMS,1-1e-6,1,1,0,CONFIG)
    b=controller.plan(PARAMS,1e-6,1,1,0,CONFIG)
    assert max(abs(a-b))<.02

def test_disabled_feedback_and_missing_encoders(controller):
    q=controller.plan(PARAMS,.8,1,1,0,CONFIG)
    np.testing.assert_array_equal(controller.feedback(q,None,None,PARAMS,CONFIG,True),q)
    np.testing.assert_array_equal(controller.feedback(q,None,None,PARAMS,CONFIG,False),q)


def test_cad_jacobians_match_all_twelve_joint_directions(controller):
    q=controller.plan(PARAMS,.82,1,1,0,CONFIG)
    for leg in range(4):
        controller.set_angles(q);_,jac=controller.foot(leg,True)
        for joint in range(3):
            plus=q.copy();minus=q.copy();plus[leg*3+joint]+=.001;minus[leg*3+joint]-=.001
            controller.set_angles(plus);a=controller.foot(leg)
            controller.set_angles(minus);b=controller.foot(leg)
            np.testing.assert_allclose(jac[:,joint],(a-b)/np.radians(.002),atol=2e-5)


def test_sensor_fault_removes_residual_correction(controller):
    from types import SimpleNamespace
    q=controller.plan(PARAMS,.82,1,1,0,CONFIG)
    attitude=SimpleNamespace(filtered=[30,-20],failures=0)
    corrected=controller.feedback(q,q,attitude,PARAMS,CONFIG,True)
    assert np.max(abs(corrected-q))<=.250001
    attitude.failures=1
    np.testing.assert_array_equal(controller.feedback(q,q,attitude,PARAMS,CONFIG,True),q)
    assert not np.any(controller.correction)


def test_delayed_quantized_encoders_are_not_current_state():
    from position_wbc import EncoderChannel
    channel=EncoderChannel()
    assert channel.read(np.zeros(12)) is None
    assert channel.read(np.ones(12)) is None
    np.testing.assert_array_equal(channel.read(np.ones(12)*2),np.zeros(12))


def test_final_policy_boundaries_and_stance_constraints(controller):
    profile=json.loads(Path(__file__).with_name('upright_profiles.json').read_text())['profiles']['cushion_rear_copy_phase']
    cfg=profile['support_shift'];window=.2/PARAMS[0]
    for boundary in [0,window,.2-window,.2,.5,.5+window,.7-window,.7]:
        a=controller.plan(PARAMS,(boundary-1e-6)%1,1,1,0,cfg)
        b=controller.plan(PARAMS,(boundary+1e-6)%1,1,1,0,cfg)
        assert max(abs(a-b))<.02
        assert controller.diagnostic['stance_residual_m']<.001


def test_rear_swing_copies_front_after_attitude_compensation(controller):
    cfg=json.loads(Path(__file__).with_name('upright_profiles.json').read_text())['profiles']['cushion_rear_copy_phase']['support_shift']
    for phase,front,rear,stance in [(.82,0,3,(1,2)),(.32,1,2,(0,3))]:
        controller.plan(PARAMS,phase,1,1,0,dict(cfg,rear_swing_from_front=False))
        before=np.array([controller.foot(i) for i in range(4)])
        controller.plan(PARAMS,phase,1,1,0,cfg)
        after=np.array([controller.foot(i) for i in range(4)])
        relative=after-controller.shoulders
        np.testing.assert_allclose(relative[front,[0,2]],relative[rear,[0,2]],atol=.0002)
        np.testing.assert_allclose(after[list(stance)],before[list(stance)],atol=.0002)


def test_stop_profile_switch_and_imu_loss_preserve_safety(controller):
    from cad_physics import Simulation
    from virtual_robot import RobotController
    p,_=physics();p['foot_cushion']=json.loads(Path(__file__).with_name('foot_cushion_10mm.json').read_text())
    robot=RobotController(Simulation(p,controller.model))
    robot.load_experimental_profiles(Path(__file__).with_name('upright_profiles.json'))
    robot.command('simprofile cushion_support_shift',0)
    robot.command('drive 0 0 1',0)
    robot.command('@S 2',.02)
    assert robot.motion is None
    for frame in range(300):robot.tick(frame*.02)
    assert robot.transition is None
    robot.command('simprofile cushion_forward',6)
    assert robot.profile=='cushion_forward'
    robot.command('simprofile cushion_support_shift',6)
    robot.command('drive 300 0 3',6)
    robot.imu.read=lambda now:None
    for frame in range(5):robot.tick(6+frame*.02)
    assert robot.safety=='imu'
    assert robot.motion is None


def test_v1_j1_lock_survives_imu_feedback(controller):
    from types import SimpleNamespace
    config=dict(CONFIG,lock_j1=True)
    controller.reset()
    for phase in np.linspace(.05,.95,10):
        q=controller.plan(PARAMS,float(phase),1,1,0,config)
        corrected=controller.feedback(q,q,SimpleNamespace(filtered=[30,-20],failures=0),PARAMS,config,True)
        np.testing.assert_array_equal(q[::3],controller.neutral_j1)
        np.testing.assert_array_equal(corrected[::3],controller.neutral_j1)
    controller.reset()


def test_push_timing_keeps_stride_monotonic_and_boundaries():
    duty=.85
    for bias in (0,6,12):
        q=np.linspace(0,duty,1001)
        x=np.array([swing_path(float(t),duty,bias)[0] for t in q])
        assert x[0]==pytest.approx(.5)
        assert x[-1]==pytest.approx(-.5)
        assert np.all(np.diff(x)<0)
        for boundary in (0,duty):
            for delta in (1e-5,2e-5):
                sample=boundary+delta if boundary==0 else boundary-delta
                assert abs(swing_path(sample,duty,bias)[0]-swing_path(sample,duty,0)[0])<1e-10
    with pytest.raises(ValueError):swing_path(.2,duty,13)


def test_common_overlap_push_preserves_stance_endpoints(controller):
    params=list(PARAMS);params[1]=.85
    baseline=dict(CONFIG,lock_j1=True,lateral_m=0,lower_m=0)
    pulse=dict(baseline,forward_pulse_m=.01,forward_pulse_shape='overlap')
    for phase in (0,.35,.5,.85):
        a=controller.plan(params,phase,1,1,0,baseline)
        b=controller.plan(params,phase,1,1,0,pulse)
        np.testing.assert_allclose(a,b,atol=1e-10)
    controller.plan(params,.175,1,1,0,pulse)
    assert controller.diagnostic['body_shift_m'][0]==pytest.approx(-.01)


def test_constant_height_removes_gait_height_command(controller):
    config=dict(CONFIG,constant_body_height=True,lock_j1=True)
    for phase in np.linspace(0,1,31):
        controller.plan(PARAMS,float(phase%1),1,1,0,config)
        assert controller.diagnostic['body_shift_m'][2]==0


def test_height_feedback_corrects_high_body_and_resets_on_sensor_loss(controller):
    controller.reset()
    config=dict(CONFIG,constant_body_height=True,lock_j1=True,height_feedback_gain=.6)
    nominal=controller.plan(PARAMS,0,0,0,0,config)
    controller.set_angles(nominal)
    feet=np.array([controller.foot(i) for i in range(4)])
    feet[:,2]-=.005
    extended,_=controller.solve(feet,nominal,iterations=16,locked_j1=nominal[::3])
    delta=controller.height_feedback(extended,np.zeros(2),PARAMS,config)
    assert 0<delta<=.001
    assert controller.diagnostic['height_estimate_valid']
    controller.feedback(nominal,None,None,PARAMS,config,True)
    assert controller.height_correction==0
    assert controller.height_error_filtered is None

def test_separated_tasks_hold_j1_and_solve_swing_world_height(controller):
    from types import SimpleNamespace
    params=[1.44,.5,.06,.02,.20175,-.01,.75]
    cfg=dict(CONFIG,lock_j1=True,constant_body_height=True,lateral_m=0,
             separate_support_swing=True,height_feedback_gain=0,feedback_gain=.15)
    q=controller.plan(params,.75,1,1,0,cfg)
    controller.reset()
    for _ in range(25):
        out=controller.feedback(q,q,SimpleNamespace(filtered=[20,-10],failures=0),params,cfg,True)
    np.testing.assert_array_equal(out[::3],q[::3])
    assert max(abs(out-q))<=6.000001
    assert controller.diagnostic['feedback_residual_m']<.0002
    # The old full-XYZ rotation overconstrained a fixed-J1 leg. The new X/Z
    # tasks remain solvable while tilt produces opposite swing-leg corrections.
    assert max(abs(out-q))>.1
    controller.feedback(q,None,None,params,cfg,True)
    np.testing.assert_array_equal(controller.correction,0.)

def test_separated_tasks_can_use_j1_when_unlocked(controller):
    from types import SimpleNamespace
    params=[1.44,.5,.06,.02,.20175,-.01,.75]
    cfg=dict(CONFIG,lock_j1=False,constant_body_height=True,lateral_m=0,
             separate_support_swing=True,height_feedback_gain=0,feedback_gain=.15)
    q=controller.plan(params,.75,1,1,0,cfg);controller.reset()
    for _ in range(25):
        out=controller.feedback(q,q,SimpleNamespace(filtered=[20,-10],failures=0),params,cfg,True)
    assert max(abs((out-q)[::3]))>.1
    assert controller.diagnostic['applied_task_residual_m']<.0002

def test_split_residual_reduces_ground_height_gap_for_same_tilt(controller):
    from types import SimpleNamespace
    params=[1.44,.5,.06,.02,.20175,-.01,.75]
    cfg=dict(CONFIG,lock_j1=False,constant_body_height=True,lateral_m=0,
             height_feedback_gain=0,load_preload_gain=0)
    q=controller.plan(params,.75,1,1,0,cfg)
    cr,sr=np.cos(np.radians(2)/2),np.sin(np.radians(2)/2)
    cp,sp=np.cos(np.radians(-1)/2),np.sin(np.radians(-1)/2)
    def world_gap(q):
        controller.set_angles(q);controller.data.qpos[3:7]=[cp*cr,cp*sr,sp*cr,-sp*sr]
        mujoco.mj_kinematics(controller.model,controller.data);mujoco.mj_comPos(controller.model,controller.data)
        return abs(controller.foot(0)[2]-controller.foot(3)[2])
    before=world_gap(q)
    controller.reset()
    for _ in range(30):
        out=controller.feedback(q,q,SimpleNamespace(filtered=[20,-10],failures=0),params,
            dict(cfg,split_residual_tasks=True),True)
    assert world_gap(out)<before*.95
