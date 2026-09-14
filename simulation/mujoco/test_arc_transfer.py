"""Independent host checks for the experimental CAD support-transfer estimate."""
import ctypes
import subprocess
from pathlib import Path
import numpy as np
import pytest

ROOT=Path(__file__).resolve().parents[2]
@pytest.fixture(scope='module')
def lib(tmp_path_factory):
    root=tmp_path_factory.mktemp('arc-transfer');src=root/'test.c';dll=root/'test.dylib'
    src.write_text('''#include "arc_transfer.h"
int apply(float *state,float *q,const float *p){GaitPolicyLegTarget pose[4];for(int i=0;i<4;i++){pose[i].j1_deg=q[3*i];pose[i].j2_deg=q[3*i+1];pose[i].j3_deg=q[3*i+2];}if(!arc_transfer_apply(state,pose,p[0],p[1],p[2],p[3],p[4],p[5],p[6],p[7]))return 0;for(int i=0;i<4;i++){q[3*i]=pose[i].j1_deg;q[3*i+1]=pose[i].j2_deg;q[3*i+2]=pose[i].j3_deg;}return 1;}
float support(float phase,float duty,float width){return arc_transfer_support(phase,duty,width);}
void geometry(const float *q,float *bias,float *feet,float *jz,float *com){GaitPolicyLegTarget p[4];for(int i=0;i<4;i++){p[i].j1_deg=q[3*i];p[i].j2_deg=q[3*i+1];p[i].j3_deg=q[3*i+2];}arc_gravity_model(p,(float(*)[3])bias,(float(*)[3])feet,(float(*)[3])jz,com);for(int i=0;i<4;i++){float point[3];arc_foot_geometry(i,q+3*i,point,NULL,feet+3*i);}}
''')
    subprocess.run(['clang','-O2','-shared','-fPIC','-I'+str(ROOT/'firmware/stm32-learning/Inc'),str(src),'-o',str(dll)],check=True)
    lib=ctypes.CDLL(str(dll));fp=ctypes.POINTER(ctypes.c_float)
    lib.apply.argtypes=(fp,fp,fp);lib.apply.restype=ctypes.c_int
    lib.support.argtypes=(ctypes.c_float,ctypes.c_float,ctypes.c_float);lib.support.restype=ctypes.c_float
    lib.geometry.argtypes=(fp,fp,fp,fp,fp)
    return lib

def fa(xs):return (ctypes.c_float*len(xs))(*xs)
def smooth(x):
    x=np.clip(x,0,1);return x*x*x*(10+x*(-15+6*x))
def support(phase,duty,width):
    if width==0:return float(phase%1<duty)
    u=(phase+width/2)%1
    if u<width:return smooth(u/width)
    if u<duty:return 1.
    if u<duty+width:return 1-smooth((u-duty)/width)
    return 0.

def test_periodic_c2_support_gate(lib):
    for duty in (.5,.55,.65):
      for width in (.04,.1,.2):
        for phase in np.linspace(-1.2,2.2,301):
            actual=lib.support(phase,duty,width)
            assert actual==pytest.approx(support(phase,duty,width),abs=5e-6)
            assert 0<=actual<=1.000001
            assert actual+lib.support(phase+.5,duty,width)>.99
        for boundary in (-width/2,width/2,duty-width/2,duty+width/2):
            eps=1e-4
            mid=lib.support(boundary,duty,width)
            left=lib.support(boundary-eps,duty,width);right=lib.support(boundary+eps,duty,width)
            assert abs((right-mid)/eps-(mid-left)/eps)<.1

def test_matches_contact_point_reference_and_dt_slew(lib):
    from servo import SharedGaitPolicy
    policy=SharedGaitPolicy();target=policy._library.spot_arc_configured
    fp=ctypes.POINTER(ctypes.c_float);target.argtypes=(*([ctypes.c_float]*4),fp,fp);target.restype=ctypes.c_int
    state=fa([0.]*12);expected_state=np.zeros(12)
    for n,phase in enumerate(np.linspace(0,2,181)):
        q=fa([0.]*12);assert target(phase,1,0,-1,fa([.021,.5,.04,0]),q)
        nominal=np.array(q);b=fa([0.]*12);feet=fa([0.]*12);jz=fa([0.]*12);com=fa([0.]*3)
        lib.geometry(q,b,feet,jz,com);f=np.array(feet).reshape(4,3);center=np.array(com)
        dt=(.01,.02,.04)[n%3];period=1.2;duty=.5;width=.12;lead=.02;gain=.3
        wa=support(phase+lead/period,duty,width/period);wb=support(phase+.5+lead/period,duty,width/period)
        shares=np.zeros(4)
        for pair,w in (((0,3),wa),((1,2),wb)):
            a,c=pair;v=f[c,:2]-f[a,:2]
            fraction=np.clip(np.dot(center[:2]-f[a,:2],v)/np.dot(v,v),.1,.9)
            shares[a]=(1-fraction)*w/(wa+wb);shares[c]=fraction*w/(wa+wb)
        torque=np.array(b).reshape(4,3)-np.array(jz).reshape(4,3)*4.418*9.81*shares[:,None]
        requested=np.clip(np.degrees(gain*torque/35).ravel(),-4,4)
        expected_state+=np.clip(requested-expected_state,-12.5*dt,12.5*dt)
        before=np.array(state)
        assert lib.apply(state,q,fa([phase,period,duty,width,lead,gain,35,dt]))
        np.testing.assert_allclose(q,nominal+expected_state,atol=2e-5)
        assert np.max(abs(np.array(state)-before))<=12.5*dt+1e-6
        assert np.max(abs(np.array(state)))<=4

def test_rejection_atomic_and_nonfinite(lib):
    nominal=[0.,40.,90.]*4;valid=[.2,1.2,.5,.12,.02,.3,35,.02]
    cases=[]
    for i in range(8):
        p=valid.copy();p[i]=float('nan');cases.append((nominal,[.1]*12,p))
    for i,v in ((1,0),(2,.4),(3,-.01),(3,1),(4,1),(5,1.1),(6,0),(7,0),(7,.061)):
        p=valid.copy();p[i]=v;cases.append((nominal,[.1]*12,p))
    badq=nominal.copy();badq[-1]=200;cases.append((badq,[.1]*12,valid))
    badq=nominal.copy();badq[2]=float('inf');cases.append((badq,[.1]*12,valid))
    badstate=[.1]*12;badstate[-1]=float('nan');cases.append((nominal,badstate,valid))
    for values,initial,p in cases:
        q=fa(values);state=fa(initial);before_q=bytes(q);before_s=bytes(state)
        assert not lib.apply(state,q,fa(p))
        assert bytes(q)==before_q and bytes(state)==before_s
