"""High-speed conservative feedback candidates against the archived policy."""
import ctypes,json,subprocess,tempfile
from pathlib import Path
from concurrent.futures import ProcessPoolExecutor
from validate_imu_full_speed import run
ROOT=Path(__file__).resolve().parents[2]

if __name__=='__main__':
    inc=ROOT/'firmware/stm32-learning/Inc';base=(ROOT/'simulation/mujoco/diagnostics/imu-balance-v2-candidate.h').read_text();folder=Path(tempfile.mkdtemp(prefix='spot-imu-highspeed-'));paths={}
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
            Path(__file__).with_name('imu_highspeed_candidates.json').write_text(json.dumps(rows,indent=2)+'\n')
