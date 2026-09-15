"""Body-level CAD estimate from nearby sequential feedback; not a camera measurement."""
import sys,argparse,csv,json
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[2]))
import numpy as np
from simulation.mujoco.runtime.cad_physics import Simulation
from simulation.mujoco.runtime.virtual_robot import load_parameters,parse_args
from simulation.mujoco.runtime.standing_pose import SoleKinematics

def main():
 ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('samples',type=Path);ap.add_argument('--output',type=Path,required=True);a=ap.parse_args()
 plant=Simulation(load_parameters(parse_args([])));kin=SoleKinematics(plant.model,plant.stand_target)
 samples=list(csv.DictReader(a.samples.open()));rows=[]
 for i,leg in enumerate(['FL','FR','RL','RR']):
  groups=[[r for r in samples if r['joint_name']==f'{leg}-J{j}' and int(r['status'])==0] for j in (1,2,3)]
  for j3 in groups[2]:
   t=float(j3['time_ms'])
   if not 1000<=t<=4900:continue
   nearby=[min(group,key=lambda r:abs(float(r['time_ms'])-t)) for group in groups[:2]]+[j3]
   times=[float(r['time_ms']) for r in nearby]
   if max(times)-min(times)>60:continue
   angles=[float(r['actual_deg']) for r in nearby]
   if i<2:angles[0]*=-1 # physical front-J1 encoder convention -> CAD
   q=plant.stand_target.copy();q[3*i:3*i+3]=angles;kin.set_angles(q)
   knee=kin.data.xanchor[plant.model.joint(leg.lower()+'_j3').id].copy();lower=kin.foot(i)-knee
   rows.append(dict(leg=leg,time_ms=t,sample_times_ms=times,angles_cad_deg=angles,
     lower_from_level_body_plane_deg=float(np.degrees(np.arctan2(abs(lower[2]),abs(lower[0]))))))
 report={'limitations':['Level body orientation assumed; this is not a measured ground angle.','Three joints are sequential reads up to 60ms apart, not synchronized.','Sequential per-joint feedback may miss extrema between samples.'],
  'closest_to_vertical_deg':{leg:max((r['lower_from_level_body_plane_deg'] for r in rows if r['leg']==leg),default=None) for leg in ['FL','FR','RL','RR']},'rows':rows}
 a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report['closest_to_vertical_deg']))
if __name__=='__main__':main()
