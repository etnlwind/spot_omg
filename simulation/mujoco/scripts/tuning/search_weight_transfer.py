"""Scheduled chassis translation via foot IK before a crawl foot lifts."""

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
    row=evaluate((.02,period,.05,.80,18.,.20175,-.035,'crawl'));row['transfer']=name;return row

if __name__=='__main__':
    inc=ROOT/'firmware/stm32-learning/Inc';base=(inc/'locomotion.h').read_text();folder=Path(tempfile.mkdtemp(prefix='spot-weight-transfer-'));paths={}
    for x,y in ((0.,.02),(.015,.02),(-.015,.02),(.025,.035)):
        name=f'x{x}_y{y}'
        shift=f'''float bx=0,by=0;
    if(family==1)for(int leg=0;leg<4;leg++){{
        float q=phase+phases[family][leg];q-=floorf(q);
        float distance=fabsf(remainderf(q-(p[1]+1)*.5f,1.f));
        float w=gait_policy_smootherstep(gait_policy_clampf(((1-p[1])*.5f+.06f-distance)/.06f,0,1));
        bx-={x}f*(leg<2?1.f:-1.f)*w;by-={y}f*(leg%2==0?1.f:-1.f)*w;
    }}
    '''
        source=base.replace('for(int i=0;i<4;i++) {',shift+'for(int i=0;i<4;i++) {',1).replace('float lateral=-scale*yaw*x*(i<2?1.6f:-1.6f)*pivot;','float lateral=-scale*yaw*x*(i<2?1.6f:-1.6f)*pivot-scale*by;').replace('x=p[5]+scale*command*x;','x=p[5]+scale*command*x-scale*bx;')
        d=folder/name;d.mkdir();(d/'locomotion.h').write_text(source);lib=d/'policy.dylib'
        subprocess.run(['cc','-std=c11','-O2','-fPIC','-dynamiclib',str(ROOT/'tools/servo_tool/servo/gait_policy_host.c'),str(inc.parent/'Src/robot_config.c'),'-I',str(d),'-I',str(inc),'-lm','-o',str(lib)],check=True);paths[name]=str(lib)
    rows=[]
    with ProcessPoolExecutor(max_workers=4) as pool:
        for row in pool.map(run,[(n,p,t) for n,p in paths.items() for t in (1.8,2.2)]):
            rows.append(row);print(json.dumps(row),flush=True);(RESULTS_ROOT / 'weight_transfer_candidates.json').write_text(json.dumps(rows,indent=2)+'\n')
