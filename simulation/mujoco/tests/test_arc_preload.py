
# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[3]))

from simulation.mujoco.paths import REPO_ROOT, SIM_ROOT, RESULTS_ROOT
import ctypes,json
from pathlib import Path
import numpy as np
import mujoco
from servo import SharedGaitPolicy
from simulation.mujoco.scripts.tuning.search_gait_profiles import physics
from simulation.mujoco.runtime.cad_physics import build
from simulation.mujoco.runtime.support_shift import SupportShift

def test_c_gravity_and_preload_match_private_cad_model():
    p,_=physics();p['foot_cushion']=json.loads((SIM_ROOT / 'config/foot_cushion_d37p3_l27mm.json').read_text())
    xml,_=build(p,write_scene=False);m=mujoco.MjModel.from_xml_string(xml);control=SupportShift(m)
    lib=SharedGaitPolicy()._library;fp=ctypes.POINTER(ctypes.c_float)
    gravity=lib.spot_arc_gravity;gravity.argtypes=(fp,fp,fp)
    preload=lib.spot_arc_preload;preload.argtypes=(fp,fp,*([ctypes.c_float]*4),fp);preload.restype=ctypes.c_int
    target=lib.spot_arc_configured;target.argtypes=(*([ctypes.c_float]*4),fp,fp);target.restype=ctypes.c_int
    state=(ctypes.c_float*12)();params=[1.2,.5,.08,.02,.20175,-.01,.75]
    for phase in np.linspace(0,3,150):
        q=(ctypes.c_float*12)();assert target(phase,1,0,-1,(ctypes.c_float*4)(.02,.5,.04,0),q)
        control.set_angles(np.array(q));mujoco.mj_comVel(m,control.data)
        bias=np.zeros(m.nv);mujoco.mj_rne(m,control.data,0,bias)
        b=(ctypes.c_float*12)();com=(ctypes.c_float*3)();gravity(q,b,com)
        np.testing.assert_allclose(b,bias[control.v],atol=2e-5)
        np.testing.assert_allclose(com,np.average(control.data.xipos,axis=0,weights=m.body_mass),atol=1e-6)
        control.last_phase=float(phase)
        expected=np.array(q)+control.load_preload(np.array(q),params,dict(load_preload_gain=.3,estimated_servo_kp=35.,load_share='com_projection'))
        out=(ctypes.c_float*12)();assert preload(state,q,phase,.5,.3,35,out)
        np.testing.assert_allclose(out,expected,atol=2e-4)

def test_preload_rejection_is_atomic_and_checks_nonfinite():
    lib=SharedGaitPolicy()._library;fp=ctypes.POINTER(ctypes.c_float)
    fn=lib.spot_arc_preload;fn.argtypes=(fp,fp,*([ctypes.c_float]*4),fp);fn.restype=ctypes.c_int
    valid=np.tile([0.,40.,90.],4)
    cases=[(valid,.2,float('nan'),.3,35.),(valid,.2,.5,.3,0.),
           (valid,float('inf'),.5,.3,35.),(valid,.2,.5,2.,35.)]
    bad=valid.copy();bad[-1]=float('nan');cases.append((bad,.2,.5,.3,35.))
    bad=valid.copy();bad[-1]=200.;cases.append((bad,.2,.5,.3,35.))
    for q,phase,duty,gain,kp in cases:
        state=(ctypes.c_float*12)(*([.1]*12));before=bytes(state)
        out=(ctypes.c_float*12)(*([123.]*12))
        assert not fn(state,(ctypes.c_float*12)(*q),phase,duty,gain,kp,out)
        assert bytes(state)==before
        assert list(out)==[123.]*12
