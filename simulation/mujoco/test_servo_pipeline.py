"""Audit the deployed degree -> tick -> CAD-foot path, without hardware motion."""
import ctypes
import json
from pathlib import Path
import numpy as np
from servo import SharedGaitPolicy

ROOT=Path(__file__).resolve().parents[2]

def encoder():
    lib=SharedGaitPolicy()._library
    fn=lib.spot_servo_encode
    fp=ctypes.POINTER(ctypes.c_float)
    fn.argtypes=(fp,ctypes.POINTER(ctypes.c_uint16),fp);fn.restype=ctypes.c_int
    return lib,fn

def test_full_range_quantization_and_direction():
    lib,fn=encoder()
    joints=json.loads((ROOT/'tools/servo_tool/config/joints.json').read_text())['joints']
    peak=0.;extra=0;previous=None
    # All 12 axes, both signs, including all integer tenths and midpoints.
    for degree in np.linspace(-120,120,48001):
        values=(ctypes.c_float*12)(*([degree]*12));ticks=(ctypes.c_uint16*12)();decoded=(ctypes.c_float*12)()
        valid=fn(values,ticks,decoded)
        f=float(values[0]);scaled=np.float32(np.float32(f)*np.float32(10))
        tenths=int(np.float32(scaled+np.float32(.5 if scaled>=0 else -.5)))
        step=int(np.copysign((abs(tenths*4096)+1800)//3600,tenths))
        expected=[j['center']+j['direction']*step for j in joints]
        if not all(0<=v<=4095 for v in expected):
            assert not valid;continue
        assert valid and list(ticks)==expected
        peak=max(peak,max(abs(float(v)-f) for v in decoded))
        direct=int(np.copysign(np.floor(abs(f*4096/360)+.5),f))
        extra=max(extra,abs(step-direct))
        if previous is not None:
            for i,j in enumerate(joints):assert (ticks[i]-previous[i])*j['direction']>=0
        previous=list(ticks)
    assert peak<=.05+180/4096+1e-5
    assert extra<=1
    print('max angular loss deg',peak,'max extra ticks vs direct',extra)

def test_invalid_values_rejected():
    lib,fn=encoder()
    for i in range(12):
        for bad in (float('nan'),float('inf'),float('-inf'),4000,-4000):
            q=[0.]*12;q[i]=bad
            assert not fn((ctypes.c_float*12)(*q),(ctypes.c_uint16*12)(),(ctypes.c_float*12)())

def test_arc_foot_error_after_actual_encoder():
    import mujoco
    from support_shift import SupportShift
    from cad_physics import build
    from search_gait_profiles import physics
    from drive_controller import NAMES
    p,_=physics();p['foot_cushion']=json.loads(Path(__file__).with_name('foot_cushion_d37p3_l27mm.json').read_text())
    xml,_=build(p,write_scene=False);cad=SupportShift(mujoco.MjModel.from_xml_string(xml))
    lib,encode=encoder();fn=lib.spot_locomotion_targets
    fn.argtypes=(ctypes.c_int,*([ctypes.c_float]*4),ctypes.POINTER(ctypes.c_float));fn.restype=ctypes.c_int
    peak=0.;peak_z=0.
    for linear,yaw in ((0,-1),(0,1),(1,0),(-1,0),(.7,.7),(-.7,-.7)):
        for phase in np.linspace(0,1,128,endpoint=False):
            q=(ctypes.c_float*12)();ticks=(ctypes.c_uint16*12)();decoded=(ctypes.c_float*12)()
            assert fn(NAMES.index('arcturn'),phase,1,linear,yaw,q)
            assert encode(q,ticks,decoded)
            cad.set_angles(np.array(q));before=np.array([cad.foot(i) for i in range(4)])
            cad.set_angles(np.array(decoded));after=np.array([cad.foot(i) for i in range(4)])
            peak=max(peak,float(np.max(np.linalg.norm(after-before,axis=1))))
            peak_z=max(peak_z,float(np.max(abs(after[:,2]-before[:,2]))))
    assert peak<.001
    print('max CAD foot quantization loss mm',peak*1000,'vertical mm',peak_z*1000)
