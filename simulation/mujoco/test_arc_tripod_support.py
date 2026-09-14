"""Independent geometry/transition checks, not physical gait validation."""
import ctypes
from pathlib import Path
import subprocess
import numpy as np
import pytest

FP=ctypes.POINTER(ctypes.c_float)
def arr(x):return (ctypes.c_float*len(x))(*x)

@pytest.fixture(scope='module')
def lib(tmp_path_factory):
    directory=tmp_path_factory.mktemp('tripod-support');root=Path(__file__).resolve().parents[2]
    source=directory/'check.c'
    source.write_text('''#include "arc_tripod_support.h"
int edge(const float *f,const float *c,int i,float m,float *o){return arc_tripod_edge_shift((const float(*)[3])f,c,i,m,o);}
int goal(const float *f,const float *c,float p,float t,float d,float m,float w,float *o){return arc_tripod_shift_goal((const float(*)[3])f,c,p,t,d,m,w,o);}
int bary(const float *f,const float *c,int i,float *o){return arc_tripod_barycentric((const float(*)[3])f,c,i,o);}
void nominal(float *o){GaitPolicyLegTarget p[4];arc_turn_targets(0,0,0,0,p);for(int i=0;i<4;i++){o[3*i]=p[i].j1_deg;o[3*i+1]=p[i].j2_deg;o[3*i+2]=p[i].j3_deg;}}
void feet(const float *q,float *o){for(int i=0;i<4;i++)arc_foot(i,q+3*i,o+3*i,NULL);}
int move(float *s,const float *q,float p,float t,float d,float m,float w,float *o){GaitPolicyLegTarget pose[4];for(int i=0;i<4;i++)pose[i]=(GaitPolicyLegTarget){q[3*i],q[3*i+1],q[3*i+2],true};if(!arc_tripod_support(s,pose,p,t,d,m,w))return 0;for(int i=0;i<4;i++){o[3*i]=pose[i].j1_deg;o[3*i+1]=pose[i].j2_deg;o[3*i+2]=pose[i].j3_deg;}return 1;}
''')
    binary=directory/'check.dylib';subprocess.run(['clang','-O2','-shared','-fPIC','-I'+str(root/'firmware/stm32-learning/Inc'),str(source),'-o',str(binary)],check=True)
    value=ctypes.CDLL(str(binary));f=ctypes.c_float
    value.edge.argtypes=(FP,FP,ctypes.c_int,f,FP)
    value.goal.argtypes=(FP,FP,f,f,f,f,f,FP)
    value.bary.argtypes=(FP,FP,ctypes.c_int,FP)
    value.nominal.argtypes=(FP,);value.feet.argtypes=(FP,FP)
    value.move.argtypes=(FP,FP,f,f,f,f,f,FP)
    return value


RECT=np.array([[.2,.1,0],[.2,-.1,0],[-.2,.1,0],[-.2,-.1,0]])

@pytest.mark.parametrize('missing',range(4))
def test_edge_margin_and_vertical_force_balance(lib,missing):
    f=arr(RECT.ravel());com=arr([0,0,.2]);shift=arr([0,0])
    assert lib.edge(f,com,missing,.006,shift)
    opposite=RECT[missing^3,:2]
    assert np.dot(shift,opposite)>0
    assert abs(np.linalg.norm(shift)-.006)<1e-7
    # Shift is much smaller than the 74.5 mm rectangle triangle centroid.
    shifted=arr([*shift,.2]);share=arr([0]*4)
    assert lib.bary(f,shifted,missing,share)
    assert share[missing]==0
    assert min(share)>=0 and abs(sum(share)-1)<1e-6
    np.testing.assert_allclose(np.array(share)@RECT[:,:2],shift,atol=1e-7)


def test_upcoming_lift_blend_continuity_and_lead(lib):
    f=arr(RECT.ravel());com=arr([0,0,.2])
    def goal(phase,duty=.75):
        output=arr([0,0]);assert lib.goal(f,com,phase,1.2,duty,.006,.12,output)
        return np.array(output)
    for phase in (0,.25,.5,.75,1):
        np.testing.assert_allclose(goal(phase-1e-5),goal(phase+1e-5),atol=1e-8)
    np.testing.assert_allclose(goal(.14),goal(.15),atol=1e-8)
    assert np.linalg.norm(goal(.20)-goal(.15))>.001
    np.testing.assert_allclose(goal(.10,.80),goal(.05,.75),atol=1e-8)


def test_full_cad_shift_keeps_z_and_changes_xy_by_opposite_body_shift(lib):
    nominal=arr([0]*12);lib.nominal(nominal);original=arr([0]*12);lib.feet(nominal,original)
    state=arr([0,0]);result=arr([0]*12)
    for _ in range(20):
        before=np.array(state)
        assert lib.move(state,nominal,.05,1.2,.75,.008,.12,result)
        assert np.linalg.norm(np.array(state)-before)<=.001+1e-8
        assert np.linalg.norm(state)<=.01+1e-8
    actual=arr([0]*12);lib.feet(result,actual)
    delta=np.array(actual).reshape(4,3)-np.array(original).reshape(4,3)
    np.testing.assert_allclose(delta[:,:2],np.broadcast_to(-np.array(state),(4,2)),atol=.00015)
    np.testing.assert_allclose(delta[:,2],0,atol=.00015)


def test_outside_triangle_and_invalid_call_do_not_modify_output(lib):
    share=arr([123]*4)
    assert not lib.bary(arr(RECT.ravel()),arr([.2,.1,.2]),0,share)
    assert list(share)==[123]*4
    nominal=arr([0]*12);lib.nominal(nominal)
    state=arr([.001,0]);before=bytes(state);result=arr([123]*12)
    assert not lib.move(state,nominal,.1,1.2,.5,.008,.12,result)
    assert bytes(state)==before and list(result)==[123]*12
