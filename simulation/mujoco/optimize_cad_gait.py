"""Deterministic gait search against the unchanged estimated CAD physics.

Search is a simulator-only experiment; no robot IO or firmware modification.
"""
import argparse
import copy
import json
import math
import time
from pathlib import Path
import mujoco
import numpy as np
from cad_physics import Simulation, CAD, build
from servo import SharedGaitPolicy

NAMES = ('period','duty','stride','lift','height','forward_offset','spread')
LOW = np.array([.40,.52,.025,.012,.195,-.035,0.])
HIGH = np.array([1.25,.78,.125,.060,.255,.025,9.])
INITIAL = np.array([.80,.65,.070,.025,.230,-.005,3.])
RESULTS = Path(__file__).parent / 'gait_search'
_runtime_policy = None


def shared_targets(params, t, scale=1.):
    """Deployed profile uses the exact firmware C policy; search stays Python."""
    global _runtime_policy
    if _runtime_policy is None:
        _runtime_policy = SharedGaitPolicy()
    values, _ = _runtime_policy.trot5_targets(t / params[0], scale)
    return np.array([values[(leg,j)] for leg in ('FL','FR','RL','RR') for j in (1,2,3)])


def smooth(x):
    x=np.clip(x,0.,1.)
    return x*x*x*(10+x*(-15+6*x))


def targets(params,t,scale=1.):
    period,duty,stride,lift,height,offset,spread=params
    out=[]
    for phase in (0.,.5,.5,0.):
        p=(t/period+phase)%1
        if p<duty:
            x=stride*(.5-p/duty)
            z=height
        else:
            s=(p-duty)/(1-duty)
            # Match stance velocity and zero acceleration at both ends.
            k=(1-duty)/duty
            x=stride*(-.5+(1+k)*float(smooth(s))-k*s)
            z=height-lift*64*s**3*(1-s)**3
        x=offset+scale*x
        z=height+scale*(z-height)
        # Robot-forward X; positive J2 moves the upper link backward.
        l2,l3=.141,.150
        c=(x*x+z*z-l2*l2-l3*l3)/(2*l2*l3)
        if not -1<=c<=1:raise ValueError('Unreachable foot target')
        knee=math.acos(c)
        hip=math.atan2(-x,z)+math.atan2(l3*math.sin(knee),l2+l3*math.cos(knee))
        out.extend((spread,math.degrees(hip),math.degrees(knee)))
    values=np.array(out)
    if np.any(values.reshape(4,3)<np.array([-30,-45,0])) or np.any(values.reshape(4,3)>np.array([30,100,150])):
        raise ValueError('Target outside physical joint limits')
    return values


def make_scenario(name='nominal'):
    p=json.loads((CAD/'physics_parameters.json').read_text())
    if name=='heavy_slippery':
        for k in p['mass_kg']:
            if k!='battery':p['mass_kg'][k]*=1.2
        p['friction'][0]=.55
        p['pack_open_circuit_voltage']=10.8
    elif name=='light_grippy':
        for k in p['mass_kg']:
            if k!='battery':p['mass_kg'][k]*=.85
        p['friction'][0]=1.05
        p['pack_open_circuit_voltage']=12.6
    elif name=='com_offset':
        p['battery_center_cad_m'][0]+=.025
        p['battery_center_cad_m'][1]-=.05
        p['command_delay_s']=.04
    elif name=='fine_step':p['timestep_s']=.001
    elif name=='low_friction':p['friction'][0]=.4
    elif name!='nominal':raise ValueError(name)
    xml,p=build(p,write_scene=False)
    return p,mujoco.MjModel.from_xml_string(xml)


def evaluate(params,p,model,duration=10.,baseline=False,keep_frames=False,push=False,shared=False,drive_stride=1.0,drive_period=None,target_function=None):
    sim=Simulation(p,model);m,d=sim.model,sim.data
    target_function=target_function or (shared_targets if shared else targets)
    robot=m.body('robot').id;floor=m.geom('floor').id
    # Same initial standing state and physical startup for every policy.
    for _ in range(100):sim.step(balance=False)
    if not baseline:
        start_angles=sim.desired.copy()*180/math.pi
        neutral=target_function(params,0,0)
        for i in range(50):sim.step(balance=False,targets_deg=start_angles+(neutral-start_angles)*smooth((i+1)/50))
    else:
        for _ in range(50):sim.step(balance=False)
    start=d.subtree_com[robot].copy();initial_height=start[2]
    initial_time=d.time;rows=[];work=0.;slip=0.;slip_samples=0;over=0.;saturation=0.;samples=0;tilt_sq=0.;peak_tilt=0.;bad_contact=False;fallen=False;max_penetration=0.;max_air=0;air=0;peak_tracking=0.
    measured_start=None;measured_time=None
    for i in range(round(duration/.02)):
        t=i*.02
        if push and 4<=t<4.15:d.xfrc_applied[m.body('cad_base').id,1]=8.
        else:d.xfrc_applied[:]=0
        try:
            if baseline:sim.step(1.,balance=False,startup=t,stride_scale=drive_stride,period_s=drive_period)
            else:sim.step(balance=False,targets_deg=target_function(params,t,float(smooth(t))))
        except (RuntimeError,ValueError):fallen=True;break
        peak_tracking=max(peak_tracking,float(np.degrees(abs(sim.desired-d.qpos[sim.q])).max()))
        rotation=d.xmat[robot].reshape(3,3)
        tilt=math.acos(np.clip(rotation[2,2],-1,1));yaw=math.atan2(rotation[1,0],rotation[0,0])
        z=d.subtree_com[robot,2]
        feet=[]
        for c in d.contact:
            max_penetration=max(max_penetration,-c.dist)
            if floor in (c.geom1,c.geom2):
                other=c.geom2 if c.geom1==floor else c.geom1
                name=mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,other)
                if not name.endswith('_foot'):bad_contact=True
                else:feet.append((other,c.pos.copy()))
        if not feet:air+=1;max_air=max(max_air,air)
        else:air=0
        fallen=tilt>math.radians(40) or z<.12 or z<initial_height*.55 or bad_contact
        if fallen:break
        if t>=2:
            if measured_start is None:measured_start=d.subtree_com[robot].copy();measured_time=t
            samples+=1;peak_tilt=max(peak_tilt,tilt);tilt_sq+=tilt*tilt
            work+=float(np.sum(abs(d.ctrl*d.qvel[sim.v])))*.02
            over+=float(np.mean(abs(d.ctrl)>sim.rated));saturation+=sim.saturated
            for geom,pos in feet:
                velocity=np.zeros(6);mujoco.mj_objectVelocity(m,d,mujoco.mjtObj.mjOBJ_GEOM,geom,velocity,0)
                contact_velocity=velocity[3:]+np.cross(velocity[:3],pos-d.geom_xpos[geom])
                slip+=float(np.linalg.norm(contact_velocity[:2]));slip_samples+=1
        if keep_frames:
            row=sim.row();row['policy_time_s']=t;rows.append(row)
    elapsed=(samples-1)*.02 if samples>1 else 0.
    delta=d.subtree_com[robot]-(measured_start if measured_start is not None else start)
    yaw=math.atan2(d.xmat[robot].reshape(3,3)[1,0],d.xmat[robot].reshape(3,3)[0,0])
    speed=float(delta[0]/elapsed) if elapsed>0 else 0.
    side=float(abs(delta[1])/elapsed) if elapsed>0 else 0.
    rms=math.sqrt(tilt_sq/max(1,samples));mean_slip=slip/max(1,slip_samples)
    cot=work/max(.001,m.body_mass.sum()*9.81*max(0,delta[0]))
    score=speed-.8*side-.15*abs(yaw)-.15*rms-.15*over/max(1,samples)-.10*saturation/max(1,samples)-.5*mean_slip-.02*cot
    if fallen:score=-1.-(duration-(i+1)*.02)/duration
    if abs(yaw)>math.radians(20) or peak_tilt>math.radians(20) or max_air*.02>.2 or over/max(1,samples)>.35:score-=.3
    report=dict(score=score,fallen=fallen,peak_tracking_error_deg=peak_tracking,drive_stride=drive_stride,drive_period=drive_period,elapsed_s=(i+1)*.02,speed_m_s=speed,lateral_speed_m_s=side,yaw_deg=math.degrees(yaw),tilt_rms_deg=math.degrees(rms),peak_tilt_deg=math.degrees(peak_tilt),slip_m_s=mean_slip,above_rated_fraction=over/max(1,samples),saturation_fraction=saturation/max(1,samples),mechanical_work_j=work,cost_of_transport=work/max(.001,m.body_mass.sum()*9.81*max(0,delta[0])),max_penetration_m=float(max_penetration),max_air_s=max_air*.02,displacement_m=(d.subtree_com[robot]-start).tolist(),params=dict(zip(NAMES,params)) if params is not None else None)
    if keep_frames:report['frames']=rows
    return report


def search(count,seed,source_voltage=None):
    RESULTS.mkdir(exist_ok=True)
    p,model=make_scenario()
    if source_voltage is not None:
        p['pack_open_circuit_voltage']=source_voltage
        xml,p=build(p,write_scene=False)
        model=mujoco.MjModel.from_xml_string(xml)
    baseline=evaluate(None,p,model,baseline=True)
    print('BASELINE',json.dumps(baseline),flush=True)
    rng=np.random.default_rng(seed);history=[];best=None
    old=json.loads((RESULTS/'search_v1.json').read_text())['trials'] if (RESULTS/'search_v1.json').exists() else []
    seeds=[np.array(list(r['params'].values())) for r in sorted(old,key=lambda r:r['speed_m_s'],reverse=True) if not r['fallen'] and r['slip_m_s']<.05 and r['above_rated_fraction']<.30 and r['peak_tilt_deg']<12][:10]
    for i in range(count):
        if i<len(seeds):candidate=seeds[i]
        elif i==len(seeds):candidate=INITIAL.copy()
        elif i<count//2:
            candidate=LOW+rng.random(len(LOW))*(HIGH-LOW)
        else:
            elite=sorted(history,key=lambda x:x['score'],reverse=True)[:max(3,min(12,len(history)//5))]
            parent=np.array(list(rng.choice(elite)['params'].values()))
            scale=.15*(1-(i-count/2)/(count/2))+.025
            candidate=np.clip(parent+rng.normal(0,scale,len(LOW))*(HIGH-LOW),LOW,HIGH)
        report=evaluate(candidate,p,model,duration=8.)
        report['trial']=i;history.append(report)
        if best is None or report['score']>best['score']:
            best=report;print('BEST',i,json.dumps(best),flush=True)
        if (i+1)%10==0:print('PROGRESS',i+1,'/',count,flush=True)
        (RESULTS/'search.json').write_text(json.dumps(dict(seed=seed,trajectory='C2 foot trajectory v2',physics_parameters=p,baseline=baseline,trials=history),indent=2))
    (RESULTS/'candidate.json').write_text(json.dumps(best,indent=2))
    print('DONE',json.dumps(best),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(__doc__);parser.add_argument('--trials',type=int,default=120);parser.add_argument('--seed',type=int,default=42)
    parser.add_argument('--source-voltage',type=float,default=None,help='override source voltage for reproducing earlier searches')
    args=parser.parse_args()
    if args.trials<1 or (args.source_voltage is not None and (not math.isfinite(args.source_voltage) or not 8<=args.source_voltage<=13)):
        parser.error('Positive trials and source voltage between 8 and 13 required')
    search(args.trials,args.seed,args.source_voltage)
