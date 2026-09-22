"""Shared C PD + CAD adapter. Only a delayed sensor sample enters feedback."""

# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[3]))

from simulation.mujoco.paths import REPO_ROOT, SIM_ROOT, RESULTS_ROOT
import ctypes as C
import hashlib
import json
from pathlib import Path
import re
import tempfile
import importlib.util
import numpy as np
from servo.host_build import build_shared, library_suffix

ROOT=REPO_ROOT
INC=ROOT/'firmware/stm32-learning/Inc'
CONFIG=ROOT/'config/body_stabilization.json'
_spec=importlib.util.spec_from_file_location('spot_pd_config',ROOT/'tools/generate_body_stabilization_config.py')
_config_module=importlib.util.module_from_spec(_spec);_spec.loader.exec_module(_config_module)

class ImuState(C.Structure):
    _fields_=[(n,C.c_float) for n in ('roll','pitch','gx','gy')]+[
        ('timestamp_ms',C.c_uint32),('sequence',C.c_uint32),('valid',C.c_bool),('axis_verified',C.c_bool)]

def config_type():
    # Read the declared named layout, checking ABI size below. No parallel
    # hand-maintained config order or implicit JSON key order.
    source=(INC/'body_stabilizer.h').read_text()
    body=source[:source.index('} BodyStabilizerConfig;')].rsplit('typedef struct {',1)[1]
    fields=[]
    for kind,names in re.findall(r'(float|uint32_t)\s+([^;]+);',body):
        fields.extend((n.strip(),C.c_float if kind=='float' else C.c_uint32) for n in names.split(','))
    return type('BodyStabilizerConfig',(C.Structure,),{'_fields_':fields})

def library():
    source=(SIM_ROOT / 'runtime/body_stabilizer_host.c')
    headers=sorted(INC.glob('*.h'))
    digest=hashlib.sha256(source.read_bytes()+b''.join(p.read_bytes() for p in headers)).hexdigest()[:16]
    dest=Path(tempfile.gettempdir())/f'spot-body-pd-{digest}{library_suffix()}'
    if not dest.exists():
        build_shared([source], INC, dest, extra=['-ffp-contract=off'])
    return C.CDLL(str(dest))

class BodyStabilizer:
    def __init__(self,config=None):
        self.lib=library();self.Config=config_type();self.config=self.Config()
        self.lib.spot_pd_config_size.restype=C.c_size_t
        if self.lib.spot_pd_config_size()!=C.sizeof(self.Config):raise RuntimeError('PD config ABI mismatch')
        self.lib.spot_pd_default.argtypes=[C.POINTER(self.Config)]
        self.lib.spot_pd_default(C.byref(self.config))
        values=_config_module.load_config(CONFIG)
        if config is not None:
            values={**values,**config}
        _config_module.validate_config(values)  # validate BEFORE ctypes can wrap integers
        for name,value in values.items():
            if name=='schema_version':continue
            if name not in dict(self.Config._fields_) or not np.isfinite(value):raise ValueError('Invalid PD config '+name)
            if name=='timeout_ms' and value!=int(value):raise ValueError('timeout_ms must be an integer')
            setattr(self.config,name,value)
        self.lib.spot_pd_config_valid.argtypes=[C.POINTER(self.Config)]
        if not self.lib.spot_pd_config_valid(C.byref(self.config)):raise ValueError('Invalid PD configuration limits')
        self.lib.spot_pd_create.restype=C.c_void_p
        self.state=self.lib.spot_pd_create()
        self.lib.spot_pd_destroy.argtypes=[C.c_void_p]
        self.lib.spot_pd_reset.argtypes=[C.c_void_p]
        self.lib.spot_pd_diagnostic.argtypes=[C.c_void_p];self.lib.spot_pd_diagnostic.restype=C.c_char_p
        f=C.c_float;fp=C.POINTER(f)
        self.lib.spot_pd_apply.argtypes=[C.c_void_p,C.POINTER(self.Config),C.POINTER(ImuState),C.c_uint32,f,C.c_int,f,f,f,C.c_int,fp,fp]
        self.lib.spot_pd_apply.restype=C.c_int
        self.lib.spot_pd_apply_pace.argtypes=self.lib.spot_pd_apply.argtypes
        self.lib.spot_pd_apply_pace.restype=C.c_int
        self.lib.spot_pd_apply_side.argtypes=[*self.lib.spot_pd_apply.argtypes,C.c_int]
        self.lib.spot_pd_apply_side.restype=C.c_int
        self.lib.spot_pd_feet.argtypes=[fp,fp]
        self.enabled=True
        self.diagnostic={}

    def close(self):
        if getattr(self,'state',None):self.lib.spot_pd_destroy(self.state);self.state=None
    def __del__(self):self.close()
    def reset(self):self.lib.spot_pd_reset(self.state);self.diagnostic={}
    def feet(self,target):
        out=(C.c_float*12)();self.lib.spot_pd_feet((C.c_float*12)(*target),out)
        return np.array(out).reshape(4,3)
    def apply(self,target,frame,reading,now,permitted=True,dt=.02):
        sample=None
        if reading is not None:
            gyro=reading.get('gyro_body_rad_s')
            timestamp=min(reading['sample_time_s'],reading.get('gyro_sample_time_s',reading['sample_time_s']))
            sample=ImuState(np.radians(reading['roll_tenths']/10),np.radians(reading['pitch_tenths']/10),
                gyro[0] if gyro else 0.,gyro[1] if gyro else 0.,round(timestamp*1000)&0xffffffff,
                reading.get('gyro_sequence',0),gyro is not None,reading.get('gyro_axis_verified',False))
        out=(C.c_float*12)()
        apply=self.lib.spot_pd_apply_side if (frame.get('sideways') and not frame.get('paired_side')) else self.lib.spot_pd_apply
        if frame.get('ipsilateral_side'):apply=self.lib.spot_pd_apply_pace
        ok=apply(self.state,C.byref(self.config),C.byref(sample) if sample else None,
            round(now*1000)&0xffffffff,dt,self.enabled and permitted,(frame['phase']+frame['pair_offset'])%1 if frame.get('paired_side') else frame['phase'],frame['period_s'],frame['duty'],
            frame.get('moving',True),(C.c_float*12)(*target),out,
            *([int(frame.get('lateral',0)<0)] if (frame.get('sideways') and not frame.get('paired_side')) else []))
        self.diagnostic=json.loads(self.lib.spot_pd_diagnostic(self.state))
        self.diagnostic.update(control_time_s=now,euler_sample_time_s=reading.get('sample_time_s') if reading else None,
            gyro_sample_time_s=reading.get('gyro_sample_time_s') if reading else None)
        if not ok:raise ValueError('PD Cartesian correction/IK infeasible')
        return np.array(out)
