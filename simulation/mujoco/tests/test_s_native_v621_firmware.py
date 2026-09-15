"""Real-robot V6.2.1 port: C/Python parity, physical replay, profile preservation."""
import ctypes as ct
import json
import numpy as np
import pytest
from simulation.mujoco.scripts.validation.validate_s_native_firmware import ROOT,load_binding,compare,physical_replay
from simulation.mujoco.runtime.cad_physics import Simulation
from simulation.mujoco.runtime.virtual_robot import load_parameters,parse_args

@pytest.fixture(scope='module')
def kernel(tmp_path_factory):return load_binding(tmp_path_factory.mktemp('v621-c'))

@pytest.fixture(scope='module')
def plant():return Simulation(load_parameters(parse_args([])))

@pytest.mark.parametrize('command,stop', [((1.,0.),0.),((1.,0.),.04),((1.,0.),.3),
    ((1.,0.),1.2),((1.,0.),8.),((.473,0.),2.),((-.6,0.),2.),
    ((0.,.5),2.),((0.,-.5),2.),((.6,.25),2.),((.43,.17),2.)])
def test_v621_c_targets_match_python_and_fit_calibrated_servos(kernel,plant,command,stop):
    result=compare(kernel,plant,command,stop,profile='s_native_v6_2_1')
    assert result['max_target_error_deg']<.06

@pytest.mark.parametrize('command',[(1000,0),(473,0),(-600,0),(0,500),(0,-500),(600,250)])
def test_v621_actual_c_targets_complete_protected_physics_and_stop(kernel,command):
    report,_=physical_replay(kernel,command,profile='s_native_v6_2_1')
    assert report['max_tilt_deg']<10
    assert report['final_actual_s_error_deg']<1.1
    assert 12.5<=report['stopped_video_s']<=12.7
    if command[0]>0 and command[1]==0:assert abs(report['walk_yaw_change_deg'])<2

def test_v621_keeps_v61_reset_and_profile_indices(kernel,plant):
    assert compare(kernel,plant,(1.,0.),2.)['max_target_error_deg']<.02
    manifest=json.loads((ROOT/'config/locomotion_profiles.json').read_text())
    names=['legacy',*manifest['profiles']]
    assert names[16:18]==['s_native_v6_1','s_native_v6_2_1']
    assert manifest['default']=='s_native_v6_2_5'
    assert manifest['profiles']['s_native_v6_1']['params'][:4]==[1.2,.5,.085,.012]

def test_v621_uses_existing_front_j1_physical_transform(kernel,plant):
    q=plant.stand_target.copy();q[::3]=-9
    ticks=(ct.c_uint16*12)()
    assert kernel.encode((ct.c_float*12)(*q),ticks)
    assert np.array(ticks)[::3].tolist()==[1827,2191,2206,1849]
