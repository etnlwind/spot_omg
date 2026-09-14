"""Recorded Python/C parity across command, phase and startup amplitude."""
import sys,json,ctypes
from pathlib import Path
import numpy as np

from cad_physics import Simulation
from servo import SharedGaitPolicy
from drive_controller import NAMES
from measured_swing import targets
from pivot_turn import PivotTurn
from gait_profiles import smooth
def test_recorded_python_candidate_matches_shared_c():
    p=json.loads(Path('artifacts/audits/pivot-center-2026-09-12/preview-profiles.json').read_text())['profiles']['center_pivot']
    k=PivotTurn(Simulation(json.loads(Path('simulation/mujoco/measured_response_plant.json').read_text())).model)
    f=SharedGaitPolicy()._library.spot_locomotion_targets
    f.argtypes=(ctypes.c_int,*([ctypes.c_float]*4),ctypes.POINTER(ctypes.c_float));f.restype=ctypes.c_int
    maximum=0;worst=None
    for linear,yaw in [(0,-.5),(0,.5),(1,0),(-1,0),(.2,.3),(-.2,-.3),(0,0)]:
     params=np.array(p['turn_reverse_params'])+smooth(np.clip(linear/.5,0,1))*(np.array(p['params'])-p['turn_reverse_params'])
     for scale in (0,.2,1):
      for phase in np.linspace(0,1,101):
       ref=targets(params,phase,scale,linear,yaw,p['measured_swing'])
       weight=smooth(np.clip(1-abs(linear)/.6,0,1))*smooth(min(1,abs(yaw)/.25))
       if weight:ref+=weight*(k.plan(params,phase,scale,yaw,p['pivot_turn'],nominal=ref)-ref)
       out=(ctypes.c_float*12)()
       assert f(NAMES.index('centerpivot'),phase,scale,linear,yaw,out),(linear,yaw,scale,phase)
       error=float(abs(ref-np.array(out)).max())
       if error>maximum:maximum=error;worst=(linear,yaw,scale,phase)
    print('maximum_deg',maximum,'worst',worst)
    assert maximum < .002, (maximum,worst)
