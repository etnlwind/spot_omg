"""Rate audit for time-compressed V6.2.1 shape; no hardware I/O."""
import sys,json,ctypes as ct,re
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[4]))
import numpy as np
from simulation.mujoco.scripts.validation.validate_s_native_firmware import load_binding
p=Path('artifacts/s-native-v6-2-2');p.mkdir(exist_ok=True)
limits=(Path('firmware/stm32-learning/Inc/motor_capability.h').read_text())
v3215=float(re.search(r'MOTOR_STS3215_NOMINAL_MAX_VELOCITY_DEG_S ([0-9.]+)f',limits).group(1))
v3250=float(re.search(r'MOTOR_STS3250_NOMINAL_MAX_VELOCITY_DEG_S ([0-9.]+)f',limits).group(1))
nominal=np.tile([v3215,v3250,v3215],4)
k=load_binding(p/'candidate-binding');out=(ct.c_float*12)();rows=[]
for period in [2.,1.6,1.2,1.,.8,.6,.5,.4]:
 k.reset_v621();phase=.5;linear=0.;qs=[]
 for i in range(round(8/.02)):
  linear=min(1.,linear+.04);u=min(1,(i+1)*.02);amp=u**3*(10+u*(-15+6*u))
  if not k.step(phase,amp,linear,0,1,0,.02,0,out):raise RuntimeError((period,i))
  qs.append(list(out));phase=(phase+.02/(period*(1.35-.35*linear)))%1
 v=np.abs(np.diff(qs,axis=0))/.02
 row={'period_s':period,'max_joint_deg_s':v.max(axis=0).tolist(),'max_joint_tick_s':float(v.max()*4096/360),'within_nominal_motor_velocity':bool(np.all(v.max(axis=0)<=nominal)), 'nominal_velocity_deg_s':nominal.tolist()}
 rows.append(row);print(json.dumps(row),flush=True)
(p/'rate-candidates.json').write_text(json.dumps(rows,indent=2)+'\n')
