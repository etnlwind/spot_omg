"""High-speed conservative feedback candidates against the archived policy."""

# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[4]))

from simulation.mujoco.paths import REPO_ROOT, SIM_ROOT, RESULTS_ROOT
import ctypes,json,subprocess,tempfile
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
from simulation.mujoco.scripts.validation.validate_imu_full_speed import run
ROOT=REPO_ROOT

if __name__=='__main__':
    inc=ROOT/'firmware/stm32-learning/Inc';base=(ROOT/'artifacts/simulation/mujoco/diagnostics/imu-balance-v2-candidate.h').read_text();folder=Path(tempfile.mkdtemp(prefix='spot-imu-highspeed-'));paths={}
    variants={
      'steady_j1':base.replace('(.95f+.05f*support[i])','1.f').replace('.01f*controlled.pitch','0.f*controlled.pitch').replace('2.f*controlled.roll*side*support[i]','1.f*input->roll*side'),
      'gentle_j1':base.replace('(.95f+.05f*support[i])','1.f').replace('.01f*controlled.pitch','0.f*controlled.pitch').replace('2.f*controlled.roll*side*support[i]','.5f*input->roll*side*(.9f+.1f*support[i])'),
    }
    for name,source in variants.items():
        d=folder/name;d.mkdir();(d/'balance_control.h').write_text(source);lib=d/'policy.dylib'
        subprocess.run(['cc','-std=c11','-O2','-fPIC','-dynamiclib',str(ROOT/'tools/servo_tool/servo/gait_policy_host.c'),str(inc.parent/'Src/robot_config.c'),'-I',str(d),'-I',str(inc),'-lm','-o',str(lib)],check=True);paths[name]=str(lib)
    rows=[]
    with ProcessPoolExecutor(max_workers=4) as pool:
        for row in pool.map(run,[(n,s,'nominal',p) for n,p in paths.items() for s in (55,77,101)]):
            rows.append(row);print(json.dumps(row),flush=True)
            (RESULTS_ROOT / 'imu_highspeed_candidates.json').write_text(json.dumps(rows,indent=2)+'\n')
