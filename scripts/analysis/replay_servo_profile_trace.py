"""Replay measured targets with persistent profile registers and compare joint feedback."""
import sys,csv,json,argparse,xml.etree.ElementTree as ET
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
import mujoco,numpy as np
from simulation.mujoco.runtime.cad_physics import Simulation,build
from simulation.mujoco.runtime.virtual_robot import load_parameters,parse_args
ap=argparse.ArgumentParser(description=__doc__)
ap.add_argument('--source',type=Path,default=Path('artifacts/s-native-v6-2-3/hardware-pilot-5s/joint-analysis'))
ap.add_argument('--output',type=Path,default=Path('artifacts/servo-profile-restore-v67'))
a=ap.parse_args();root=a.output;root.mkdir(parents=True,exist_ok=True);source=a.source
commands=list(csv.DictReader((source/'commands.csv').open()));samples=list(csv.DictReader((source/'samples.csv').open()))
keys=[k for k in commands[0] if k.endswith('_deg')]
times=np.array([float(r['send_begin_ms'])/1000 for r in commands])
angles=np.array([[float(r[k]) for k in keys] for r in commands]);angles[:,[0,3]]*=-1
results=[]
for speed,acc in [(3400,254),(300,30)]:
 p=load_parameters(parse_args([]));p['pack_open_circuit_voltage']=11.4
 # Keep the explicitly requested counterfactual separate from installed caps.
 p['servo_acceleration_cap_register']=254
 xml,p=build(p,write_scene=False);tree=ET.fromstring(xml);body=tree.find(".//body[@name='robot']")
 pos=list(map(float,body.get('pos','0 0 0').split()));pos[2]+=.3;body.set('pos',' '.join(map(str,pos)))
 equality=tree.find('equality')
 if equality is None:equality=ET.SubElement(tree,'equality')
 ET.SubElement(equality,'weld',body1='robot',solref='.002 1')
 model=mujoco.MjModel.from_xml_string(ET.tostring(tree,encoding='unicode'));plant=Simulation(p,model)
 plant.data.qpos[:7]=model.qpos0[:7];mujoco.mj_forward(model,plant.data)
 plant.servo_profile.set(speed,acc);sim_times=[0.];actual=[np.degrees(plant.data.qpos[plant.q]).copy()]
 for tick in range(256):
  t=tick*.02;i=max(0,np.searchsorted(times,t,side='right')-1)
  plant.step(targets_deg=angles[i],native_servo=True)
  sim_times.append(float(plant.data.time));actual.append(np.degrees(plant.data.qpos[plant.q]).copy())
 actual=np.array(actual);per_joint=[]
 for i in range(12):
  observed=[r for r in samples if int(r['joint'])==i and int(r['status'])==0 and 1000<=float(r['time_ms'])<=4900]
  t=np.array([float(r['time_ms'])/1000 for r in observed]);q=np.array([float(r['actual_deg']) for r in observed])
  if i in (0,3):q*=-1
  predicted=np.interp(t,sim_times,actual[:,i]);per_joint.append(dict(joint=keys[i],rms_error_deg=float(np.sqrt(np.mean((q-predicted)**2))),observed_range_deg=float(np.ptp(q)),simulated_range_deg=float(np.ptp(predicted))))
 row=dict(goal_speed=speed,acceleration=acc,mean_joint_rms_error_deg=float(np.mean([r['rms_error_deg'] for r in per_joint])),joints=per_joint)
 results.append(row);print(json.dumps(row),flush=True)
(root/'profile-trace-replay.json').write_text(json.dumps(dict(source=str(source),limitations=['Body weld and level frame are assumptions.','PD gains, friction, electrical and contact parameters remain estimates.','Reported configuration counterfactual is not an independent hardware validation.'],results=results),indent=2)+'\n')
