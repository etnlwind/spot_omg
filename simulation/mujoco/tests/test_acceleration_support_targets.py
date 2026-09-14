
# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[3]))
import numpy as np
import pytest
from simulation.mujoco.runtime.acceleration_support_targets import acceleration_support_targets


CENTER=np.array([-.004244537634403818,.0009649250363246598])
FEET=np.array([[.2417461733241898,-.09340841407038014],[-.15692231052749694,.09523064994677075]])


def calculate(acceleration=(0.,0.),**kwargs):
    return acceleration_support_targets(CENTER,.2,acceleration,FEET,**kwargs)


def test_static_snapshot_and_minimum_translation():
    result=calculate(mode='minimumXY')
    assert result['feasible']
    np.testing.assert_allclose(result['translation_xy_m'],[-.00851428675,-.0179940343],atol=1e-8)
    assert result['translation_norm_m']==pytest.approx(.0199067413,abs=1e-8)
    direction=FEET[1]-FEET[0]
    assert np.dot(result['translation_xy_m'],direction)==pytest.approx(0,abs=1e-12)
    np.testing.assert_allclose(result['normal_forces_n'],[10.9235722,16.0931678],atol=1e-6)
    xonly=calculate(mode='x-only')
    assert xonly['translation_xy_m'][0]==pytest.approx(-.04654275349,abs=1e-8)
    assert xonly['translation_xy_m'][1]==0.


def test_acceleration_sign_additional_15mm_and_mass_independence():
    # h=.2m, a=0.73575m/s², az=0 -> desired CoP 15mm behind COM.
    static=calculate(mode='x-only')
    dynamic=calculate((.73575,0.),mode='x-only')
    assert dynamic['desired_cop_xy_m'][0]==pytest.approx(CENTER[0]-.015)
    assert dynamic['translation_xy_m'][0]==pytest.approx(static['translation_xy_m'][0]-.015)
    heavier=calculate((.73575,0.),mode='x-only',mass_kg=5.508)
    np.testing.assert_array_equal(heavier['targets_xy_m'],dynamic['targets_xy_m'])
    np.testing.assert_allclose(heavier['normal_forces_n'],2*np.array(dynamic['normal_forces_n']))
    braking=calculate((-.73575,0.),mode='x-only')
    assert braking['translation_xy_m'][0]==pytest.approx(static['translation_xy_m'][0]+.015)


@pytest.mark.parametrize('mode',['minimumXY','x-only'])
def test_dynamic_force_and_all_moment_components(mode):
    acceleration=np.array([.7,-.3]);az=1.2
    result=calculate(acceleration,mode=mode,acceleration_z_m_s2=az,friction_coefficient=.5)
    assert result['feasible']
    np.testing.assert_allclose(result['force_balance_error_n'],0.,atol=1e-12)
    np.testing.assert_allclose(result['moment_about_com_nm'],0.,atol=1e-12)
    forces=np.array(result['contact_forces_xyz_n'])
    assert forces[:,2].sum()==pytest.approx(2.754*(9.81+az))
    assert result['required_friction_coefficient']==pytest.approx(np.linalg.norm(acceleration)/(9.81+az))
    arms=np.c_[np.array(result['targets_xy_m'])-CENTER,np.full(2,-.2)]
    np.testing.assert_allclose(np.cross(arms,forces).sum(axis=0),0.,atol=1e-12)


def test_vertical_acceleration_and_friction_gate():
    nominal=calculate((1.,0.))
    upward=calculate((1.,0.),acceleration_z_m_s2=9.81)
    np.testing.assert_allclose(CENTER-upward['desired_cop_xy_m'],.5*(CENTER-nominal['desired_cop_xy_m']))
    assert upward['total_normal_force_n']==pytest.approx(2*nominal['total_normal_force_n'])
    rejected=calculate((1.,0.),friction_coefficient=.05)
    assert rejected['geometry_feasible'] and not rejected['feasible']
    assert 'insufficient-friction' in rejected['flags']
    assert nominal['friction_feasible'] is None


@pytest.mark.parametrize(('torque','cop_delta'), [
    ((.2701674,0.),(0.,.010)),
    ((0.,.5403348),(-.020,0.)),
])
def test_centroidal_roll_pitch_moment_sign_and_force_balance(torque,cop_delta):
    # Fz=27.01674N. +tau_x moves CoP left (+Y); +tau_y moves it rearward (-X).
    result=calculate(mode='x-only',centroidal_moment_xy_nm=torque,friction_coefficient=.6)
    assert result['feasible']
    np.testing.assert_allclose(np.array(result['desired_cop_xy_m'])-CENTER,cop_delta,atol=1e-12)
    expected=np.r_[torque,0.]
    np.testing.assert_allclose(result['requested_moment_about_com_nm'],expected,atol=1e-12)
    np.testing.assert_allclose(result['moment_about_com_nm'],expected,atol=1e-12)
    np.testing.assert_allclose(result['moment_balance_error_nm'],0.,atol=1e-12)
    np.testing.assert_allclose(result['force_balance_error_n'],0.,atol=1e-12)
    assert result['required_friction_coefficient']==0.
    # The same absolute moment on twice the mass requires half the COP offset.
    heavier=calculate(mode='x-only',centroidal_moment_xy_nm=torque,mass_kg=5.508)
    np.testing.assert_allclose(np.array(heavier['desired_cop_xy_m'])-CENTER,np.array(cop_delta)/2,atol=1e-12)


def test_centroidal_moment_and_acceleration_cancel_yaw_check_each_friction_cone():
    acceleration=np.array([.7,-.3]);torque=np.array([.2,.3])
    result=calculate(acceleration,mode='x-only',centroidal_moment_xy_nm=torque)
    assert result['feasible'] and result['force_allocation_feasible']
    expected_yaw=-float(torque@acceleration)/9.81
    assert result['proportional_allocation_moment_nm'][2]==pytest.approx(expected_yaw)
    assert abs(expected_yaw)>.001
    forces=np.array(result['contact_forces_xyz_n'])
    arms=np.c_[np.array(result['targets_xy_m'])-CENTER,np.full(2,-.2)]
    np.testing.assert_allclose(np.cross(arms,forces).sum(axis=0),np.r_[torque,0.],atol=1e-12)
    np.testing.assert_allclose(result['force_balance_error_n'],0.,atol=1e-12)
    np.testing.assert_allclose(result['moment_balance_error_nm'],0.,atol=1e-12)
    per_foot=np.linalg.norm(forces[:,:2],axis=1)/forces[:,2]
    np.testing.assert_allclose(result['per_foot_required_friction_coefficient'],per_foot)
    lower=result['net_force_required_friction_coefficient']
    required=result['required_friction_coefficient']
    assert required>lower
    rejected=calculate(acceleration,mode='x-only',centroidal_moment_xy_nm=torque,
                       friction_coefficient=(lower+required)/2)
    assert not rejected['feasible'] and 'insufficient-friction' in rejected['flags']


def test_segment_endpoints_and_infeasible_axis_constraint():
    # Common X translation cannot put a Y=2m CoP within a Y=±.1m segment.
    bad=acceleration_support_targets([0,2],.2,[0,0],[[-.2,-.1],[.2,.1]],mode='x-only')
    assert not bad['feasible']
    assert min(bad['normal_forces_n'])<0
    endpoint=acceleration_support_targets([0,2],.2,[0,0],[[-.2,-.1],[.2,.1]],mode='minimumXY')
    assert endpoint['feasible'] and 'one-contact-unloaded' in endpoint['flags']
    assert endpoint['lambda_on_segment']==1.
    np.testing.assert_allclose(endpoint['moment_about_com_nm'],0.,atol=1e-12)
    horizontal=acceleration_support_targets([0,0],.2,[0,0],[[-.2,0],[.2,0]],mode='x-only')
    np.testing.assert_allclose(horizontal['translation_xy_m'],[0,0])
    bad_horizontal=acceleration_support_targets([0,.1],.2,[0,0],[[-.2,0],[.2,0]],mode='x-only')
    assert not bad_horizontal['feasible']
    assert 'x-only-cannot-change-support-y' in bad_horizontal['flags']


@pytest.mark.parametrize('az',[-9.81,-10.])
def test_no_positive_normal_support(az):
    result=calculate(acceleration_z_m_s2=az)
    assert not result['feasible']
    assert 'nonpositive-normal-acceleration' in result['flags']


def test_invalid_inputs_and_unchanged_arrays():
    before=FEET.copy();center=CENTER.copy()
    result=acceleration_support_targets(center,.2,[0,0],FEET)
    assert result['feasible']
    np.testing.assert_array_equal(FEET,before);np.testing.assert_array_equal(center,CENTER)
    for kwargs in [dict(height_m=-1),dict(mass_kg=0),dict(acceleration_xy_m_s2=[np.nan,0]),
                   dict(feet_xy_m=[[0,0],[0,0]]),dict(feet_xy_m=[0,0]),dict(friction_coefficient=np.inf),
                   dict(centroidal_moment_xy_nm=[0,np.nan]),dict(centroidal_moment_xy_nm=[0,0,0])]:
        args=dict(com_xy_m=CENTER,height_m=.2,acceleration_xy_m_s2=[0,0],feet_xy_m=FEET)
        args.update(kwargs)
        assert not acceleration_support_targets(**args)['feasible']
