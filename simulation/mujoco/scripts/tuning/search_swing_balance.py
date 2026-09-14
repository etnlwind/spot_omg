"""Test whether length feedback on airborne legs cancels their clearance."""

# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[4]))

from simulation.mujoco.paths import REPO_ROOT, SIM_ROOT, RESULTS_ROOT
import ctypes,json,subprocess,tempfile
from pathlib import Path
from types import SimpleNamespace
from concurrent.futures import ProcessPoolExecutor
import simulation.mujoco.scripts.tuning.search_raised_level as search_raised_level
from simulation.mujoco.runtime.virtual_robot import RobotController
ROOT=REPO_ROOT

def run(case):
    weight,path,period=case
    class CandidateRobot(RobotController):
        def __init__(self,plant):
            super().__init__(plant);self.balance.policy=SimpleNamespace(_library=ctypes.CDLL(path))
    search_raised_level.RobotController=CandidateRobot
    row=search_raised_level.run((.02,period,.05,.64,18.));row['swing_balance_weight']=weight;return row

if __name__=='__main__':
    inc=ROOT/'firmware/stm32-learning/Inc';base=(inc/'balance_control.h').read_text();folder=Path(tempfile.mkdtemp(prefix='spot-swing-balance-'));paths={}
    for weight in (0.,.2,.5):
        source=base.replace('float dz=preview.down_correction[i];',f'float dz=preview.down_correction[i]*({weight}f+(1.f-{weight}f)*support[i]);')
        d=folder/str(weight);d.mkdir();(d/'balance_control.h').write_text(source);lib=d/'policy.dylib'
        subprocess.run(['cc','-std=c11','-O2','-fPIC','-dynamiclib',str(ROOT/'tools/servo_tool/servo/gait_policy_host.c'),str(inc.parent/'Src/robot_config.c'),'-I',str(d),'-I',str(inc),'-lm','-o',str(lib)],check=True);paths[weight]=str(lib)
    rows=[]
    with ProcessPoolExecutor(max_workers=4) as pool:
        for row in pool.map(run,[(w,p,t) for w,p in paths.items() for t in (1.2,1.4)]):
            rows.append(row);print(json.dumps(row),flush=True);(RESULTS_ROOT / 'swing_balance_candidates.json').write_text(json.dumps(rows,indent=2)+'\n')
