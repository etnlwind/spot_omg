"""Physically weld the body above the floor; retain actuator dynamics and collisions."""
import sys,json,argparse,xml.etree.ElementTree as ET
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[4]))
import mujoco,numpy as np
from simulation.mujoco.runtime.cad_physics import Simulation,build,foot_clearance
from simulation.mujoco.runtime.virtual_robot import RobotController,load_parameters,parse_args
ap=argparse.ArgumentParser(description=__doc__)
ap.add_argument('--profile',choices=['s_native_v6_2_2','s_native_v6_2_3','s_native_v6_2_4','s_native_v6_2_5'],default='s_native_v6_2_2')
ap.add_argument('--floor',action='store_true')
ap.add_argument('--period',type=float,help='Analysis-only cycle-time override')
ap.add_argument('--output',type=Path)
a=ap.parse_args()
p=a.output or Path('artifacts/'+a.profile.replace('_','-'));p.mkdir(parents=True,exist_ok=True)
parameters=load_parameters(parse_args([]));xml,parameters=build(parameters,write_scene=False)
root=ET.fromstring(xml);body=root.find(".//body[@name='robot']")
position=list(map(float,body.get('pos','0 0 0').split()));position[2]+=0 if a.floor else .3;body.set('pos',' '.join(map(str,position)))
eq=root.find('equality')
if eq is None:eq=ET.SubElement(root,'equality')
if not a.floor:ET.SubElement(eq,'weld',name='test_body_fixture',body1='robot',solref='.002 1')
model=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'))
plant=Simulation(parameters,model);plant.data.qpos[:7]=model.qpos0[:7];mujoco.mj_forward(model,plant.data)
robot=RobotController(plant)
if a.period is not None:
 if not .4<=a.period<=2.:raise ValueError('Period must be .4..2 seconds')
 robot.profiles[a.profile]['params'][0]=a.period
robot.select_profile(a.profile);rows=[];floor=model.geom('floor').id
for i in range(500):
 t=i*.02
 if i==100:robot.command('drive 1000 0 1',t)
 elif i==300:robot.command('@S 1000',t)
 elif 100<i<300 and i%10==0:robot.command(f'@D {i} 1000 0',t)
 robot.tick(t);state=plant.row()
 contacts=[[model.geom(c.geom1).name,model.geom(c.geom2).name] for c in plant.data.contact if floor not in (c.geom1,c.geom2)]
 rows.append(dict(time_s=t,target=robot.command_target.tolist(),actual=state['actual_deg'],safety=robot.safety,roll_deg=state['roll_deg'],pitch_deg=state['pitch_deg'],internal_contacts=contacts,reply=robot.drain().decode()))
summary={'support':'floor' if a.floor else 'body weld +0.3m, all contact/actuator models retained; estimated physics',
 'first_fault_s':next((r['time_s'] for r in rows if r['safety']!='ok'),None),
 'max_tracking_error_deg':float(np.max(np.abs(np.array([r['target'] for r in rows])-np.array([r['actual'] for r in rows])))),
 'internal_contact_frames':sum(bool(r['internal_contacts']) for r in rows),
 'final_error_deg':float(np.max(abs(np.array(rows[-1]['actual'])-robot.stand_target))),
 'stop_response':next((r['reply'] for r in rows if '$SPOTDRIVE stopped' in r['reply']),None)}
healthy=[r for r in rows if summary['first_fault_s'] is None or r['time_s']<summary['first_fault_s']]
summary['pre_fault_max_tracking_error_deg']=max(max(abs(a-b) for a,b in zip(r['target'],r['actual'])) for r in healthy)
summary['pre_fault_internal_contact_frames']=sum(bool(r['internal_contacts']) for r in healthy)
steady=np.array([r['actual'] for r in rows[150:300] if summary['first_fault_s'] is None or r['time_s']<summary['first_fault_s']])
summary['steady_window_complete']=summary['first_fault_s'] is None or summary['first_fault_s']>=6.
summary['steady_observed_excursion_deg']=np.ptp(steady,axis=0).tolist() if len(steady)>1 else None
summary['steady_observed_mean_abs_speed_deg_s']=(np.abs(np.diff(steady,axis=0))/.02).mean(axis=0).tolist() if len(steady)>1 else None
summary['period_s']=robot.profiles[a.profile]['params'][0]
filename='floor-physics' if a.floor else 'supported-physics'
if a.period is not None:filename+=f'-period-{a.period:g}'
(p/(filename+'.json')).write_text(json.dumps({'summary':summary,'rows':rows},indent=2));print(json.dumps(summary))
