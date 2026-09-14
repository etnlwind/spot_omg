"""Temporary gait kernels: separate front/rear lift and smooth top dwell."""

# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[4]))

from simulation.mujoco.paths import REPO_ROOT, SIM_ROOT, RESULTS_ROOT
import ctypes,json,subprocess,tempfile
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
from servo import SharedGaitPolicy
import simulation.mujoco.runtime.gait_profiles as gait_profiles
from simulation.mujoco.scripts.tuning.search_raised_level import run as evaluate
ROOT=REPO_ROOT


def run(case):
    name,path,period=case
    class CandidatePolicy(SharedGaitPolicy):
        @classmethod
        def _build_library(cls):return Path(path)
    policy=CandidatePolicy();fn=policy._library.spot_foot_targets;fp=ctypes.POINTER(ctypes.c_float)
    fn.argtypes=(fp,ctypes.c_float,ctypes.c_float,ctypes.c_int,ctypes.c_float,ctypes.c_float,fp);fn.restype=ctypes.c_int
    gait_profiles._shared=lambda:(policy,fn)
    row=evaluate((.02,period,.05,.64,18.));row['shape']=name;return row

if __name__=='__main__':
    inc=ROOT/'firmware/stm32-learning/Inc';base=(inc/'locomotion.h').read_text();folder=Path(tempfile.mkdtemp(prefix='spot-raised-shape-'));paths={}
    specs=[('front14_rear08',1.4,.8,False),('front18_rear065',1.8,.65,False),('front14_rear10',1.4,1.,False),('dwell_front12_rear08',1.2,.8,True),('dwell_front10_rear07',1.,.7,True)]
    for name,front,rear,dwell in specs:
        envelope='gait_policy_smootherstep(fminf(1.f,fminf(u,1.f-u)/.3f))' if dwell else '64*u*u*u*(1-u)*(1-u)*(1-u)'
        source=base.replace('p[3]*64*u*u*u*(1-u)*(1-u)*(1-u)',f'p[3]*(i<2?{front}f:{rear}f)*({envelope})')
        d=folder/name;d.mkdir();(d/'locomotion.h').write_text(source);lib=d/'policy.dylib'
        subprocess.run(['cc','-std=c11','-O2','-fPIC','-dynamiclib',str(ROOT/'tools/servo_tool/servo/gait_policy_host.c'),str(inc.parent/'Src/robot_config.c'),'-I',str(d),'-I',str(inc),'-lm','-o',str(lib)],check=True);paths[name]=str(lib)
    (ROOT/'artifacts/simulation/mujoco/raised_shape_libraries.json').write_text(json.dumps(paths,indent=2)+'\n')
    rows=[]
    with ProcessPoolExecutor(max_workers=4) as pool:
        for row in pool.map(run,[(n,p,t) for n,p in paths.items() for t in (1.2,1.4)]):
            rows.append(row);print(json.dumps(row),flush=True);(RESULTS_ROOT / 'raised_shape_candidates.json').write_text(json.dumps(rows,indent=2)+'\n')
