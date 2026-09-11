"""Independent physics/sign and constraint tests for the acceleration-force QP."""
import numpy as np
import pytest
from dynamics_wbc import constrained_dynamics


def particle(acceleration,mu=.5):
    # Three free translational DOFs, one internal actuated coordinate.
    M=np.diag([2.,2.,2.,1.]);bias=np.array([0.,0.,19.62,0.])
    J=np.c_[np.eye(3),np.zeros(3)]
    task=(np.eye(4),np.r_[acceleration,0.],np.ones(4)*100)
    return constrained_dynamics(M,bias,J,[task],np.array([3]),np.array([1.]),
        np.full(4,-100.),np.full(4,100.),mu)


def test_static_gravity_and_unilateral_force_sign():
    solved,report=particle([0,0,0]);assert solved is not None
    a,f,tau=solved
    np.testing.assert_allclose(f,[0,0,19.62],atol=.001)
    assert report['dynamics_residual']<1e-4
    np.testing.assert_allclose(2*a[:3]+[0,0,19.62],f,atol=1e-4)


def test_unreachable_horizontal_acceleration_is_reported_as_task_residual():
    solved,_=particle([30,30,0]);a,f,_=solved
    assert abs(f[0])+abs(f[1])<=.5*f[2]+.001
    assert np.linalg.norm(a[:2]-[30,30])>10
    assert f[2]>=0


def test_torque_limit_is_hard_even_with_large_posture_demand():
    M=np.eye(2);bias=np.array([9.81,0.])
    # Use a physically oriented z support force on a one-dimensional base.
    J=np.array([[0.,0.],[0.,0.],[1.,0.]])
    solved,report=constrained_dynamics(M,bias,J,[(np.eye(2),[0,100],100.)],
        np.array([1]),np.array([.2]),np.full(2,-100.),np.full(2,100.))
    assert solved is not None
    assert abs(solved[2][0])<=.2001
    assert report['constraint_violation']<.001


def test_impossible_force_balance_is_not_silently_relaxed():
    # Floating mass required to accelerate upwards without any contacts.
    result,report=constrained_dynamics(np.eye(2),np.array([9.81,0.]),np.zeros((0,2)),[],
        np.array([1]),np.ones(1),np.zeros(2),np.ones(2))
    assert result is None
    assert 'infeasible' in report['status']


def test_invalid_joint_envelope_rejected():
    result,report=constrained_dynamics(np.eye(2),np.zeros(2),np.zeros((0,2)),[],
        np.array([1]),np.ones(1),np.ones(2),np.zeros(2))
    assert result is None and report['status']=='infeasible-joint-envelope'

@pytest.fixture(scope='module')
def geometry():
    import json,mujoco
    from pathlib import Path
    from cad_physics import build
    from search_gait_profiles import physics
    from support_shift import SupportShift
    p,_=physics();p['foot_cushion']=json.loads(Path(__file__).with_name('foot_cushion_10mm.json').read_text())
    xml,_=build(p,write_scene=False)
    return SupportShift(mujoco.MjModel.from_xml_string(xml))


def test_full_cad_equation_and_isolated_data(geometry):
    import mujoco
    from types import SimpleNamespace
    g=geometry;g.reset()
    params=[1.44,.5,.08,.02,.20175,-.01,.75]
    cfg={'lateral_m':0,'lower_m':0,'constant_body_height':True,
         'dynamics_wbc':{'rated_factor':3.,'position_adapter':'bounded_trajectory','contact_patch_depth_m':.001}}
    nominal=g.plan(params,.2,0.,0.,0.,cfg)
    out=g.feedback(nominal,nominal,SimpleNamespace(filtered=[0,0],previous=[0,0],rate=[0,0],failures=0),params,cfg,True)
    w=g.dynamics_controller;report=w.diagnostic
    assert report['feasible']
    assert w.data is not g.data
    M=np.zeros((g.model.nv,g.model.nv));mujoco.mj_fullM(g.model,w.data,M)
    residual=M@np.array(report['qdd'])+w.data.qfrc_bias-w.data.qfrc_passive
    for point,force,leg in zip(report['force_points_m'],report['contact_force_n'],report['force_point_legs']):
        jp=np.zeros((3,g.model.nv));jr=np.zeros_like(jp)
        mujoco.mj_jac(g.model,w.data,jp,jr,np.array(point),int(g.model.geom_bodyid[g.feet[leg]]))
        residual-=jp.T@force
    residual[g.v]-=report['torque_nm']
    np.testing.assert_allclose(residual,0.,atol=.002)
    assert np.isfinite(out).all()
    assert report['support']==[0,1,2,3]  # Zero-amplitude startup is four contacts.


def test_missing_packet_clears_position_and_load_memory(geometry):
    from types import SimpleNamespace
    from dynamics_wbc import DynamicsWBC
    g=geometry
    if g.dynamics_controller is None:g.dynamics_controller=DynamicsWBC(g)
    g.dynamics_controller.adapter_delta[:]=4;g.dynamics_controller.load_delta[:]=4
    nominal=np.array([0,45,90]*4,dtype=float)
    output=g.feedback(nominal,None,SimpleNamespace(filtered=[0,0],failures=0),
        [1.44,.5,.08,.02,.20175,-.01,.75],{'dynamics_wbc':{'rated_factor':3}},True)
    np.testing.assert_array_equal(output,nominal)
    np.testing.assert_array_equal(g.dynamics_controller.load_delta,0)
    np.testing.assert_array_equal(g.dynamics_controller.adapter_delta,0)
    assert g.dynamics_controller.previous_q is None


def test_preview_gravity_contact_schedule_and_pose_bounds():
    from centroidal_preview import solve_preview
    feet=np.array([[.2,.1,-.2],[.2,-.1,-.2],[-.2,.1,-.2],[-.2,-.1,-.2]])
    params=[1.44,.5,0.,.02,.20175,-.01,.75]
    result,report=solve_preview(np.zeros(12),4.,np.diag([.1,.2,.2]),feet,np.zeros(3),.1,params,0.)
    assert result is not None, report
    states,forces=result
    assert report['lateral_peak_m']<=.0101 and report['height_error_peak_m']<=.0101
    assert np.max(abs(states[:,2]))<.001
    for k,f in enumerate(forces):
        stance=((.1+(k+.5)*.08/params[0]+np.array([0,.5,.5,0]))%1)<.5
        np.testing.assert_allclose(f[~stance],0,atol=.0002)
        assert np.all(f[:,2]>=-.0002)
        assert np.all(abs(f[:,0])+abs(f[:,1])<=.5*f[:,2]+.0002)
        acceleration=(states[k+1,8]-states[k,8])/.08
        assert f[:,2].sum()==pytest.approx(4*(9.81+acceleration),abs=.002)
