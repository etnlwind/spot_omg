"""Real C native kernel: CAD parity, calibrated servo limits and physical STOP."""
import ctypes as ct
import json
import numpy as np
import pytest
from simulation.mujoco.runtime.cad_physics import Simulation
from simulation.mujoco.runtime.virtual_robot import load_parameters, parse_args
from simulation.mujoco.scripts.validation.validate_s_native_firmware import ROOT,load_binding,compare,physical_replay
from simulation.mujoco.runtime.s_native_gait import SNativeGait

# Motor tick -> CAD angle signs from the physical mounting, independent of
# the implementation under test. FR ID4 increasing is measured; FL mirrors
# FR, and the user observed the existing rear adduction working on Sep 15.
PHYSICAL_CAD_DIRECTIONS=np.array([1,-1,1,-1,1,-1,-1,-1,1,1,1,-1])

def physical_angles(ticks):
    joints=json.loads((ROOT/'tools/servo_tool/config/joints.json').read_text())['joints']
    return (np.array(ticks)-[j['center'] for j in joints])*PHYSICAL_CAD_DIRECTIONS*360/4096

def encoded(kernel,q):
    ticks=(ct.c_uint16*12)()
    assert kernel.encode((ct.c_float*12)(*q),ticks)
    return np.array(ticks)

@pytest.fixture(scope='module')
def kernel(tmp_path_factory):
    return load_binding(tmp_path_factory.mktemp('native-c'))

@pytest.fixture(scope='module')
def plant():
    return Simulation(load_parameters(parse_args([])))

@pytest.mark.parametrize('command,stop', [((1.,0.),8.),((1.,0.),0.),((1.,0.),.04),((1.,0.),.3),((.3,0.),2.),
    ((.6,0.),2.),((-1.,0.),2.),((0.,.5),2.),((0.,-.5),2.),((.6,-.3),2.),((.43,.17),2.)])
def test_native_targets_and_servo_limits(kernel,plant,command,stop):
    assert compare(kernel,plant,command,stop)['max_target_error_deg'] < .02

def test_c_kernel_walks_and_returns_to_s_in_mujoco(kernel):
    summary,_=physical_replay(kernel)
    assert summary['max_tilt_deg'] < 5
    assert summary['final_actual_s_error_deg'] < 1.1
    assert abs(summary['walk_yaw_change_deg']) < 2


@pytest.mark.parametrize('backend',['python','c'])
@pytest.mark.parametrize('yaw',[500,-500])
def test_native_protocol_turn_matches_actual_body_direction(kernel,backend,yaw):
    report,_=physical_replay(kernel if backend=='c' else None,(0,yaw),False)
    assert report['walk_yaw_change_deg']*yaw<0
    assert abs(report['walk_yaw_change_deg'])>15


@pytest.mark.parametrize('backend',['python','c'])
def test_native_heading_reduces_measured_drift_at_user_input(kernel,backend):
    lib=kernel if backend=='c' else None
    on,_=physical_replay(lib,(473,0),True)
    off,_=physical_replay(lib,(473,0),False)
    assert abs(on['walk_yaw_change_deg'])<2
    assert abs(on['walk_yaw_change_deg'])<abs(off['walk_yaw_change_deg'])*.5

def test_native_adduction_matches_physical_motor_directions(kernel,plant):
    q=plant.stand_target.copy();q[::3]=-9
    ticks=encoded(kernel,q)
    assert ticks[::3].tolist()==[1827,2191,2206,1849]
    assert np.max(abs(physical_angles(ticks)-q))<.095
    # The measured FR inward trial commanded 2203 from approximately 2093.
    q[3]=-10
    assert encoded(kernel,q)[3]==2203

def test_old_conversion_reproduces_front_outward_rear_inward(kernel,plant):
    gait=SNativeGait(plant.model,plant.stand_target)
    points=gait.origin.copy();points[:,1]+=gait.walking_lateral_offset
    q,error=gait.kin.solve_xz(points,gait.standing,gait.standing[::3]+gait.normal_adduction)
    assert error<.0002
    legacy=plant.policy._library.spot_servo_encode
    legacy.argtypes=[ct.POINTER(ct.c_float),ct.POINTER(ct.c_uint16),ct.POINTER(ct.c_float)]
    old=(ct.c_uint16*12)();decoded=(ct.c_float*12)()
    assert legacy((ct.c_float*12)(*q),old,decoded)
    def inward(ticks):
        gait.kin.set_angles(physical_angles(ticks))
        y=np.array([gait.kin.foot(i)[1] for i in range(4)])
        return abs(gait.origin[:,1])-abs(y)
    before=inward(old);after=inward(encoded(kernel,q))
    assert np.all(before[:2]<-.025) and np.all(before[2:]>.025)
    assert np.all(after>.025)

def test_only_front_j1_motor_encoding_changes(kernel,plant):
    legacy=plant.policy._library.spot_servo_encode
    legacy.argtypes=[ct.POINTER(ct.c_float),ct.POINTER(ct.c_uint16),ct.POINTER(ct.c_float)]
    for angle in (-18.,-9.,0.,9.,18.):
        q=plant.stand_target.copy();q[::3]=angle
        old=(ct.c_uint16*12)();decoded=(ct.c_float*12)()
        assert legacy((ct.c_float*12)(*q),old,decoded)
        new=encoded(kernel,q)
        assert np.array_equal(new[[1,2,4,5,6,7,8,9,10,11]],np.array(old)[[1,2,4,5,6,7,8,9,10,11]])
        assert new[0]+old[0]==2*1929 and new[3]+old[3]==2*2089
    assert encoded(kernel,plant.stand_target)[::3].tolist()==[1929,2089,2104,1951]

@pytest.mark.parametrize('field,value',[(0,float('nan')),(1,1.1),(2,1.1),(3,.6),(4,float('inf')),(6,.1)])
def test_bad_native_inputs_are_rejected(kernel,field,value):
    kernel.reset();args=[.5,.1,1,0,1,0,.02];args[field]=value
    assert not kernel.step(*args,0,(ct.c_float*12)())
