"""Offline Stow preflight using the actual shared C firmware encoder. No transport."""
import ctypes
import json
from pathlib import Path
import numpy as np
from servo import SharedGaitPolicy
from stow_policy import LANDING,FOLDED,FOLD_SECONDS


def validate():
    policy=SharedGaitPolicy()
    fn=policy._library.spot_servo_encode
    fn.argtypes=(ctypes.POINTER(ctypes.c_float),ctypes.POINTER(ctypes.c_uint16),ctypes.POINTER(ctypes.c_float))
    fn.restype=ctypes.c_int
    joints=json.loads((Path(__file__).parents[2]/'tools/servo_tool/config/joints.json').read_text())['joints']
    failures=[]
    for i,j in enumerate(joints):
        lo,hi=sorted([(j['min']-j['center'])*j['direction']*360/4096,
                      (j['max']-j['center'])*j['direction']*360/4096])
        for step in range(round(FOLD_SECONDS/.02)+1):
            elapsed=step*.02;t=policy.smootherstep(elapsed/FOLD_SECONDS)
            angle=LANDING[i]+(FOLDED[i]-LANDING[i])*t
            values=[0.]*12;values[i]=angle
            if not fn((ctypes.c_float*12)(*values),(ctypes.c_uint16*12)(),(ctypes.c_float*12)()):
                failures.append(dict(servo_id=j['id'],leg=j['leg'],joint=j['joint'],
                                     first_failed_fold_time_s=elapsed,first_failed_angle_deg=angle,
                                     configured_encoder_range_deg=[lo,hi],stow_target_deg=FOLDED[i]))
                break
    return dict(hardware_ready=False,encoder_pass=not failures,encoder_failures=failures,
                reason='Current Stow path fails existing firmware position encoding' if failures else 'Further physical validation required',
                additional_unverified=['exact mesh/shaft/housing/wire clearances','measured joint hard stops and encoder seam behavior',
                                       'PLA/PETG mass/inertia, backlash and friction uncertainty','loaded actuator response and electrical/thermal envelope'])


if __name__=='__main__':
    result=validate()
    path=Path(__file__).parent/'diagnostics/stow/hardware-preflight.json'
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))
    raise SystemExit(0 if result['hardware_ready'] else 2)
