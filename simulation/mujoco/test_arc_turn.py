import ctypes
import json
import numpy as np
from pathlib import Path
from servo import SharedGaitPolicy
from support_shift import SupportShift
from cad_physics import build
from search_gait_profiles import physics
import mujoco

def test_cad_targets_match_python_arc_reference():
    p,_=physics();p['foot_cushion']=json.loads(Path(__file__).with_name('foot_cushion_10mm.json').read_text())
    xml,p=build(p,write_scene=False);c=SupportShift(mujoco.MjModel.from_xml_string(xml))
    policy=SharedGaitPolicy();fn=policy._library.spot_locomotion_targets
    fn.argtypes=(ctypes.c_int,ctypes.c_float,ctypes.c_float,ctypes.c_float,ctypes.c_float,ctypes.POINTER(ctypes.c_float));fn.restype=ctypes.c_int
    from drive_controller import NAMES
    for linear,yaw in ((0,-1),(0,1),(1,0),(-1,0),(.5,.8),(-.8,-.5),(0,0)):
      params=[1.44-.24*abs(yaw)/max(abs(linear)+abs(yaw),1e-9),.5,.08,.02,.20175,-.01,.75]
      for scale in (0,.4,1.):
       for phase in np.linspace(0,1,16,endpoint=False):
        out=(ctypes.c_float*12)();assert fn(NAMES.index('arcturn'),phase,scale,linear,yaw,out)
        reference=c.plan(params,phase,scale,linear,yaw,dict(lateral_m=0,lower_m=0,kinematic_lead_s=.04,turn_path='arc',turn_sweep_rad=.2,turn_lift_m=.02))
        c.set_angles(reference);expected=np.array([c.foot(i) for i in range(4)])
        c.set_angles(np.array(out));actual=np.array([c.foot(i) for i in range(4)])
        np.testing.assert_allclose(actual,expected,atol=.00025)


def test_convex_search_matches_full_cushion_mesh(tmp_path):
    import subprocess
    root=Path(__file__).resolve().parents[2]
    source=tmp_path/'search.c'
    source.write_text('''#include "arc_turn.h"
void fast(int n,const float *d,unsigned *out){for(int i=0;i<n;i++)out[i]=arc_lowest(d+3*i);}
void full(int n,const float *d,unsigned *out){for(int i=0;i<n;i++){unsigned best=0;float value=arc_projection(0,d+3*i);for(unsigned j=1;j<ARC_VERTEX_COUNT;j++){float v=arc_projection(j,d+3*i);if(v<value){value=v;best=j;}}out[i]=best;}}
float value(unsigned i,const float *d){return arc_projection(i,d);}
''')
    library=tmp_path/'search.dylib'
    subprocess.run(['clang','-O2','-shared','-fPIC','-I'+str(root/'firmware/stm32-learning/Inc'),str(source),'-o',str(library)],check=True)
    lib=ctypes.CDLL(str(library));fp=ctypes.POINTER(ctypes.c_float);ip=ctypes.POINTER(ctypes.c_uint)
    for name in ('fast','full'):getattr(lib,name).argtypes=(ctypes.c_int,fp,ip)
    lib.value.argtypes=(ctypes.c_uint,fp);lib.value.restype=ctypes.c_float
    directions=np.concatenate([np.random.default_rng(42).normal(size=(5000,3)),np.eye(3),-np.eye(3)]).astype('float32')
    fast=(ctypes.c_uint*len(directions))();full=(ctypes.c_uint*len(directions))()
    import time
    times={}
    for name,out in (('fast',fast),('full',full)):
        start=time.perf_counter();getattr(lib,name)(len(directions),directions.ctypes.data_as(fp),out);times[name]=time.perf_counter()-start
    for i,d in enumerate(directions):
        a=lib.value(fast[i],d.ctypes.data_as(fp));b=lib.value(full[i],d.ctypes.data_as(fp))
        assert abs(a-b)<1e-8,(i,d,fast[i],full[i],a,b)
    print('support search host timings',times,'speedup',times['full']/times['fast'])


def test_arc_headers_invalidate_host_library_cache(monkeypatch):
    original=Path.read_bytes
    baseline=SharedGaitPolicy._build_library()
    for name in ('arc_turn.h','arc_geometry.h'):
        def changed(path, selected=name):
            contents=original(path)
            return contents+b'\n/* dependency test */\n' if path.name==selected else contents
        with monkeypatch.context() as context:
            context.setattr(Path,'read_bytes',changed)
            assert SharedGaitPolicy._build_library()!=baseline
