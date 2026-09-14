"""Linear-model/constraint tests only; MuJoCo and physical validation are separate."""

# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[3]))

from simulation.mujoco.paths import REPO_ROOT, SIM_ROOT, RESULTS_ROOT
import ctypes
from pathlib import Path
import subprocess
import numpy as np
import pytest
from scipy.linalg import solve_continuous_are,expm

FP=ctypes.POINTER(ctypes.c_float)
def arr(x):return (ctypes.c_float*len(x))(*x)

@pytest.fixture(scope='module')
def lib(tmp_path_factory):
    directory=tmp_path_factory.mktemp('arc-dynamic');root=REPO_ROOT
    source=directory/'host.c';source.write_text('''#include "arc_dynamic_balance.h"
void gains(int i,float*out){for(int k=0;k<4;k++)out[k]=arc_dynamic_gain[i][k];}
int accel(int i,const float*x,float*out){ArcDynamicAxisState s={x[2],x[3]};return arc_dynamic_acceleration(i,x[0],x[1],&s,true,out);}
int advance(int i,float*s,float t,float r,int valid,float dt,float*out){ArcDynamicAxisState state={s[0],s[1]};if(!arc_dynamic_axis_step(i,&state,t,r,valid,dt,.01f,.1f,1.f,out))return 0;s[0]=state.delta;s[1]=state.velocity;return 1;}
''')
    output=directory/'host.dylib'
    subprocess.run(['clang','-shared','-fPIC','-O2','-I'+str(root/'firmware/stm32-learning/Inc'),str(source),'-o',str(output)],check=True)
    value=ctypes.CDLL(str(output));f=ctypes.c_float
    value.gains.argtypes=(ctypes.c_int,FP);value.accel.argtypes=(ctypes.c_int,FP,FP)
    value.advance.argtypes=(ctypes.c_int,FP,f,f,ctypes.c_int,f,FP)
    return value


def model(pair):
    mass=4.418;height=.198298803;inertia=[.227410649,.227380853][pair]
    A=np.array([[0,1,0,0],[mass*9.81*height/inertia,0,-mass*9.81/inertia,0],[0,0,0,1],[0,0,0,0]])
    B=np.array([[0],[mass*height/inertia],[0],[1]])
    return A,B


@pytest.mark.parametrize('pair',[0,1])
def test_gain_matches_care_with_positive_acceleration_reaction(lib,pair):
    A,B=model(pair);P=solve_continuous_are(A,B,np.diag([1,.01,100,1]),[[1.]])
    expected=B.T@P;gains=arr([0]*4);lib.gains(pair,gains)
    np.testing.assert_allclose(gains,expected.ravel(),rtol=2e-6)
    assert np.max(np.real(np.linalg.eigvals(A-B@np.array(gains)[None,:])))<0
    result=arr([0]);assert lib.accel(pair,arr([.001,0,0,0]),result)
    assert result[0]>0  # Required nonminimum-phase initial response, not a sign flip.


def test_axis_integration_converges_in_undelayed_small_signal_model(lib):
    A,B=model(0);dt=.02;augmented=np.zeros((5,5));augmented[:4,:4]=A;augmented[:4,4:]=B
    transition=expm(augmented*dt);x=np.array([1e-3,0,0,0.]);state=arr([0,0]);acceleration=arr([0])
    for _ in range(400):
        assert lib.advance(0,state,x[0],x[1],1,dt,acceleration)
        x=transition[:4,:4]@x+transition[:4,4]*acceleration[0]
        np.testing.assert_allclose(state,x[2:],atol=5e-8)
        # This test explicitly assumes ideal undelayed translation tracking;
        # avoid accumulating an unrelated float/double integrator discrepancy.
        x[2:]=state
    assert np.linalg.norm(x)<1e-7


def test_position_velocity_acceleration_bounds_use_braking_not_teleport(lib):
    state=arr([0,0]);acceleration=arr([0]);dt=.02
    for step in range(500):
        before=np.array(state);theta=.04*np.sin(step*.031);rate=.062*np.cos(step*.031)
        assert lib.advance(0,state,theta,rate,1,dt,acceleration)
        assert abs(state[0])<=.0100001 and abs(state[1])<=.1000001 and abs(acceleration[0])<=1.0000001
        np.testing.assert_allclose(state[0],before[0]+dt*before[1]+.5*dt*dt*acceleration[0],atol=2e-9)
        np.testing.assert_allclose(state[1],before[1]+dt*acceleration[0],atol=2e-8)


def test_unbrakeable_or_bad_sensor_input_rejected_atomically(lib):
    for state,theta in ((arr([.009,.09]),0.),(arr([0,0]),float('nan'))):
        before=bytes(state);output=arr([123])
        assert not lib.advance(0,state,theta,0,1,.02,output)
        assert bytes(state)==before and output[0]==123
