"""Compare coupled hip/knee standing angles on the STEP dynamics model.
No hardware I/O. Uses V15 stride 1.6, original period, feedback OFF.
Angles use firmware canonical coordinates, not angles measured from a photo.
"""
import argparse
import json
import math
from pathlib import Path
import mujoco
import numpy as np
from cad_physics import Simulation, KEYS, build
from optimize_cad_gait import make_scenario


def shift_height(values, hip_deg):
    """Preserve sagittal foot X and lift; change nominal leg extension only.
    Equal normalized links match the firmware IK; CAD links are approximate.
    """
    if not math.isfinite(hip_deg) or not 0<=hip_deg<=60:raise ValueError("hip angle must be in 0..60 degrees")
    a=np.array(values,dtype=float).reshape(4,3).copy()
    hip=np.radians(a[:,1]);lower=hip-np.radians(a[:,2])
    x=np.sin(hip)+np.sin(lower)
    z=np.cos(hip)+np.cos(lower)+2*(math.cos(math.radians(hip_deg))-math.cos(math.pi/4))
    cosine=(x*x+z*z-2)/2
    if np.any(abs(cosine)>1+1e-6):raise ValueError('IK reach limit')
    knee=np.arccos(np.clip(cosine,-1,1))
    upper=np.arctan2(x,z)+knee/2
    a[:,1]=np.degrees(upper);a[:,2]=np.degrees(knee)
    if np.any(a[:,1]<-45) or np.any(a[:,1]>100) or np.any(a[:,2]>150):
        raise ValueError('joint limit')
    return a.ravel()


def evaluate(hip,p,m,duration=20):
    sim=Simulation(p,m);d=sim.data;body=m.body('robot').id;floor=m.geom('floor').id
    for _ in range(100):sim.step(balance=False)
    old=np.degrees(sim.desired);neutral=shift_height(old,hip)
    for i in range(50):sim.step(targets_deg=old+(neutral-old)*sim.policy.smootherstep((i+1)/50))
    for _ in range(50):sim.step(targets_deg=neutral)
    static=dict(com_height_m=float(d.subtree_com[body,2]),mean_abs_torque_nm=float(np.mean(abs(d.ctrl))),max_abs_torque_nm=float(max(abs(d.ctrl))),current_model_a=float(sim.current))
    work=slip=over=0.;slip_count=count=0;peak_tilt=peak_error=0.;electrical=0.;start=None;rows=[];status='UPRIGHT'
    for i in range(round(duration/.02)):
        t=i*.02
        target,_=sim.policy.drive_stride_targets(sim.phase,sim.policy.smootherstep(min(1,t/.7)),float(min(1,sim.linear+.04)),0,1.6)
        try: values=shift_height([target[k] for k in KEYS],hip)
        except ValueError:status='IK_LIMIT';break
        sim.step(1,balance=False,targets_deg=values)
        rotation=d.xmat[body].reshape(3,3);tilt=math.degrees(math.acos(np.clip(rotation[2,2],-1,1)))
        feet=[];bad=False
        for contact in d.contact:
            if floor in (contact.geom1,contact.geom2):
                geom=contact.geom2 if contact.geom1==floor else contact.geom1
                if not m.geom(geom).name.endswith('_foot'):bad=True
                else:feet.append((geom,contact.pos.copy()))
        if tilt>40 or bad or d.subtree_com[body,2]<.12:status='FALLEN';break
        if t>=2:
            if start is None:start=d.subtree_com[body].copy()
            count+=1;peak_tilt=max(peak_tilt,tilt)
            peak_error=max(peak_error,float(max(abs(values-np.degrees(d.qpos[sim.q])))))
            work+=float(np.sum(abs(d.ctrl*d.qvel[sim.v])))*.02
            electrical+=sim.current*sim.voltage*.02
            over+=float(np.mean(abs(d.ctrl)>sim.rated))
            for geom,pos in feet:
                v=np.zeros(6);mujoco.mj_objectVelocity(m,d,mujoco.mjtObj.mjOBJ_GEOM,geom,v,0)
                slip+=float(np.linalg.norm((v[3:]+np.cross(v[:3],pos-d.geom_xpos[geom]))[:2]));slip_count+=1
    delta=d.subtree_com[body]-(start if start is not None else d.subtree_com[body]);elapsed=max(0,(count-1)*.02)
    yaw=math.degrees(math.atan2(d.xmat[body].reshape(3,3)[1,0],d.xmat[body].reshape(3,3)[0,0]))
    return dict(timestep_s=float(m.opt.timestep),hip_deg=hip,knee_deg=2*hip,status=status,elapsed_s=(i+1)*.02,static=static,
      speed_m_s=float(delta[0]/elapsed) if elapsed else 0,yaw_deg=yaw,lateral_m=float(delta[1]),
      peak_tilt_deg=peak_tilt,peak_tracking_error_deg=peak_error,slip_m_s=slip/max(1,slip_count),
      above_rated_fraction=over/max(1,count),mechanical_cot=work/max(.001,float(m.body_mass.sum()*9.81*delta[0])),
      estimated_electrical_j_m=electrical/max(.001,float(delta[0])))


def main():
    ap=argparse.ArgumentParser(__doc__);ap.add_argument('--validate',action='store_true');ap.add_argument('--output',type=Path,default=Path('/private/tmp/spot-stance'))
    ap.add_argument('--timestep',type=float,default=.0005,help='physics step in seconds; 0.0005 validated against 0.00025')
    args=ap.parse_args();results=[]
    if not math.isfinite(args.timestep) or args.timestep<=0 or abs(.02/args.timestep-round(.02/args.timestep))>1e-6:ap.error('timestep must divide 20 ms')
    for scene in (['nominal','heavy_slippery','light_grippy','com_offset','low_friction'] if args.validate else ['nominal']):
        p,m=make_scenario(scene)
        p["timestep_s"]=args.timestep;xml,_=build(p,write_scene=False);m=mujoco.MjModel.from_xml_string(xml)
        for hip in ([35,40,45] if args.validate else [25,30,35,40,45,50,55]):
            r=evaluate(hip,p,m);r['scenario']=scene;results.append(r);print(json.dumps(r),flush=True)
    args.output.mkdir(parents=True,exist_ok=True);(args.output/'summary.json').write_text(json.dumps(results,indent=2)+'\n')

if __name__=='__main__':main()
