"""Free-body dynamics of the new STEP assembly with estimated physical parameters."""
import argparse
import collections
import json
import math
import time
from pathlib import Path
from xml.etree import ElementTree as ET
import mujoco
import numpy as np
from servo import SharedGaitPolicy, ImuSample
from cad_gait import CAD, KEYS, build as build_rig, vec


def vertices(name):
    return np.fromfile(CAD / (name+'.stl'), dtype=np.dtype([('n','<f4',3),('v','<f4',(3,3)),('a','<u2')]), offset=84)['v'].reshape(-1,3).astype(float)/1000


def inertia(body, mass, center, size):
    old = body.find('inertial')
    if old is not None:
        body.remove(old)
    s = np.maximum(size, .005)
    diagonal = mass/12 * np.array([s[1]**2+s[2]**2,s[0]**2+s[2]**2,s[0]**2+s[1]**2])
    ET.SubElement(body, 'inertial', pos=vec(center), mass=str(mass), diaginertia=vec(diagonal))


def build(parameters=None,write_scene=True):
    p = parameters or json.loads((CAD/'physics_parameters.json').read_text())
    root,mapping = build_rig(write_files=False)
    root.set('model', 'New STEP / free-body dynamics / ESTIMATED PHYSICS')
    option = root.find('option'); option.set('gravity', f"0 0 {-p['gravity_m_s2']}")
    option.set('timestep', str(p['timestep_s'])); option.set('integrator','implicitfast')
    option.set('solver','Newton'); option.set('iterations','100'); option.set('cone','elliptic')
    default = ET.SubElement(root,'default')
    ET.SubElement(default,'geom',friction=vec(p['friction']),condim='4',solref=vec([p['contact_time_constant_s'],p['contact_damping_ratio']]))
    world = root.find('worldbody'); floor = world.find("geom[@type='plane']")
    floor.set('name','floor'); floor.set('contype','1');floor.set('conaffinity','1');floor.set('size','20 20 .01')
    floor_roll=math.radians(p.get('floor_roll_deg',0.0))
    floor_pitch=math.radians(p.get('floor_pitch_deg',0.0))
    cr,sr=math.cos(floor_roll/2),math.sin(floor_roll/2)
    cp,sp=math.cos(floor_pitch/2),math.sin(floor_pitch/2)
    floor.set('quat',vec([cr*cp,sr*cp,cr*sp,-sr*sp]))
    cadbase=world.find("body[@name='cad_base']"); world.remove(cadbase)
    robot=ET.SubElement(world,'body',name='robot');ET.SubElement(robot,'freejoint',name='root');robot.append(cadbase)
    points=np.concatenate([vertices(f'body_{i}') for i in range(4)])
    lo,hi=points.min(0),points.max(0)
    inertia(cadbase,p['mass_kg']['chassis_without_battery_or_leg_groups'],(lo+hi)/2,hi-lo)
    ET.SubElement(cadbase,'geom',name='chassis_collision',type='box',pos=vec((lo+hi)/2),size=vec((hi-lo)/2),contype='1',conaffinity='1',group='3',rgba='.5 .5 .5 .15')
    battery=ET.SubElement(cadbase,'body',name='battery',pos=vec(p['battery_center_cad_m']))
    inertia(battery,p['mass_kg']['battery'],np.zeros(3),np.array(p['battery_size_m']))
    act=ET.SubElement(root,'actuator')
    for leg,j in KEYS:
        name=f'{leg.lower()}_j{j}';body=root.find(f".//body[@name='{name}_link']")
        pivot=np.array(mapping[name]['pivot_cad_m']);points=vertices(name)
        lo,hi=points.min(0),points.max(0)
        inertia(body,p['mass_kg'][f'j{j}_group'],(lo+hi)/2-pivot,hi-lo)
        joint=body.find('joint');joint.set('limited','true');joint.set('range',vec(np.radians({1:[-30,30],2:[-45,100],3:[0,150]}[j])))
        for field,key in [('damping','joint_damping'),('frictionloss','joint_friction_nm'),('armature','rotor_armature_kg_m2')]:joint.set(field,str(p[key]))
        attrs=dict(contype='1',conaffinity='1',group='3',rgba='.8 .4 .1 .15')
        if j==1:
            ET.SubElement(body,'geom',name=name+'_collision',type='box',pos=vec((lo+hi)/2-pivot),size=vec((hi-lo)/2),**attrs)
        else:
            if j==2:
                endpoint=np.array(mapping[f'{leg.lower()}_j3']['pivot_cad_m'])
            else:
                tip=points[points[:,2]<lo[2]+.008].mean(0)
                tip[2]=lo[2]+p['foot_radius_m']
                endpoint=tip
                ET.SubElement(body,'geom',name=leg.lower()+'_foot',type='sphere',pos=vec(tip-pivot),size=str(p['foot_radius_m']),**attrs)
            # Slim link contact proxy; avoid spanning the full motor-case AABB.
            midpoint=(lo+hi)/2; start=pivot.copy();start[0]=midpoint[0];end=endpoint.copy();end[0]=midpoint[0]
            ET.SubElement(body,'geom',name=name+'_collision',type='capsule',fromto=vec(np.r_[start-pivot,end-pivot]),size='.012',**attrs)
        ET.SubElement(act,'motor',name=name+'_motor',joint=name,gear='1')
    # Show detailed CAD; collision proxies remain active but hidden in GUI group 3.
    if not write_scene:
        return ET.tostring(root,encoding='unicode'),p
    ET.indent(root);ET.ElementTree(root).write(CAD/'physics_scene.xml',encoding='unicode')
    return CAD/'physics_scene.xml',p


def imu(model,data):
    r=data.xmat[model.body('robot').id].reshape(3,3)
    return ImuSample(math.atan2(r[2,1],r[2,2]),math.asin(np.clip(-r[2,0],-1,1)),float(data.qvel[3]),float(data.qvel[4]))


class Simulation:
    def __init__(self,p=None,model=None):
        if model is None:
            xml,self.p=build(p,write_scene=False)
            self.model=mujoco.MjModel.from_xml_string(xml)
        else:
            self.p=p
            self.model=model
        self.joint_zero_error=np.asarray(self.p.get('joint_zero_error_deg',[0.]*12),dtype=float)
        if self.joint_zero_error.shape!=(12,) or not np.isfinite(self.joint_zero_error).all():
            raise ValueError('joint_zero_error_deg must contain twelve finite values')
        self.data=mujoco.MjData(self.model)
        self.sensor_observer = None
        self.policy=SharedGaitPolicy();self.phase=self.linear=self.yaw=0.
        self.q=np.array([self.model.jnt_qposadr[self.model.joint(f'{l.lower()}_j{j}').id] for l,j in KEYS])
        self.v=np.array([self.model.jnt_dofadr[self.model.joint(f'{l.lower()}_j{j}').id] for l,j in KEYS])
        self.motors=[self.p['motor_3250' if j==2 else 'motor_3215'] for l,j in KEYS]
        self.stall=np.array([m['stall_nm'] for m in self.motors]);self.rated=np.array([m['rated_nm'] for m in self.motors]);self.speed=np.array([m['speed_rad_s'] for m in self.motors])
        target,_=self.policy.drive_targets(0,0,0,0);self.desired=np.radians([target[k] for k in KEYS])
        self.data.qpos[self.q]=self.desired
        mujoco.mj_forward(self.model,self.data)
        feet=[self.model.geom(l.lower()+'_foot').id for l in ('FL','FR','RL','RR')]
        floor=min(self.data.geom_xpos[i,2]-self.model.geom_size[i,0] for i in feet)
        self.data.qpos[2] += .001-floor
        mujoco.mj_forward(self.model,self.data)
        self.filtered=self.desired.copy();self.target_velocity=np.zeros(12)
        self.delay=collections.deque([self.desired.copy() for _ in range(max(0,round(self.p['command_delay_s']/.02)))])
        self.voltage=self.p['pack_open_circuit_voltage'];self.current=0.;self.limits=self.stall.copy();self.saturated=0.

    def step(self,linear=0,yaw=0,balance=True,startup=1,targets_deg=None,limit_yaw=True,stride_scale=None,period_s=None,torque_enabled=True):
        if period_s is not None and (not math.isfinite(period_s) or period_s <= 0):
            raise ValueError("period_s must be positive and finite")
        p=self.p;d=self.data;m=self.model;dt=m.opt.timestep
        if limit_yaw:
            yaw = self.policy.drive_yaw_limit(round(yaw*1000))/1000
        self.linear+=np.clip(linear-self.linear,-.04,.04);self.yaw+=np.clip(yaw-self.yaw,-.04,.04)
        if targets_deg is None:
            args=(self.phase,self.policy.smootherstep(min(1,startup)),float(self.linear),float(self.yaw))
            target,support=(self.policy.drive_targets(*args) if stride_scale is None else
                            self.policy.drive_stride_targets(*args,stride_scale))
            if balance:
                target=self.policy.balance_targets(target,sample=imu(m,d),support_legs=support,kp=1,kd=.04,leg_length_limit=.08,mode='contact-aware',j1_gain=15,j1_limit=5,foot_placement_gain=0,foot_placement_limit=0)
            targets_deg=[target[k] for k in KEYS]
        values=np.asarray(targets_deg,dtype=float)
        if values.shape != (12,) or not np.isfinite(values).all():
            raise ValueError('Expected twelve finite joint targets')
        # Same calibration, 0.1-degree rounding and 4096-tick encoding as STM32.
        if self.p.get('embedded_servo_quantization', True):
            import ctypes
            fn=self.policy._library.spot_servo_encode
            fp=ctypes.POINTER(ctypes.c_float)
            fn.argtypes=(fp,ctypes.POINTER(ctypes.c_uint16),fp);fn.restype=ctypes.c_int
            ticks=(ctypes.c_uint16*12)();decoded=(ctypes.c_float*12)()
            if not fn((ctypes.c_float*12)(*values),ticks,decoded):raise ValueError('Servo target outside calibrated tick range')
            self.servo_ticks=list(ticks);values=np.array(decoded)
        values=values+self.joint_zero_error
        self.desired=np.radians(values);self.delay.append(self.desired.copy());delayed=self.delay.popleft()
        self.phase=(self.phase+.02/(period_s if period_s is not None else (2.4-.6*min(1,abs(self.linear)+abs(self.yaw)))))%1
        for _ in range(round(.02/dt)):
            wanted=np.clip((delayed-self.filtered)/dt,-self.speed,self.speed)
            self.target_velocity+=np.clip(wanted-self.target_velocity,-p['target_acceleration_rad_s2']*dt,p['target_acceleration_rad_s2']*dt)
            increment=self.target_velocity*dt
            increment=np.where(abs(increment)>abs(delayed-self.filtered),delayed-self.filtered,increment)
            self.filtered+=increment
            raw=p['servo_kp']*(self.filtered-d.qpos[self.q])-p['servo_kd']*d.qvel[self.v]
            scale=max(.05,self.voltage/12)
            # Linear DC motor torque-speed envelope in motoring direction;
            # braking is bounded by stall torque, without regenerative charging.
            opposing=np.maximum(0,np.sign(raw)*d.qvel[self.v])
            self.limits=self.stall*scale*np.maximum(0,1-opposing/(self.speed*scale))
            torque=np.clip(raw,-self.limits,self.limits) if torque_enabled else np.zeros(12)
            d.ctrl[:]=torque
            self.current=p['electronics_current_a']+sum(motor['idle_current_a']+abs(t)/stall*(motor['stall_current_a']-motor['idle_current_a']) for motor,t,stall in zip(self.motors,torque,self.stall))
            self.voltage=max(0,p['pack_open_circuit_voltage']-self.current*p['pack_and_wiring_resistance_ohm'])
            self.saturated=float(np.mean(abs(raw)>self.limits))
            if self.sensor_observer is not None:
                self.sensor_observer(m, d)
            mujoco.mj_step(m,d)
        if not np.isfinite(d.qpos).all() or not np.isfinite(d.qvel).all() or any(w.number for w in d.warning):raise RuntimeError('Nonfinite state or MuJoCo numerical warning')

    def row(self):
        m,d=self.model,self.data;sample=imu(m,d);contacts=set();normal=0.;penetration=0.
        for c in d.contact:
            penetration=max(penetration,-float(c.dist))
            if m.geom('floor').id in (c.geom1,c.geom2):
                other=c.geom2 if c.geom1==m.geom('floor').id else c.geom1
                name=mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,other)
                contacts.add(name)
        # Contact list indices, rather than geom IDs, index contact forces.
        for i in range(d.ncon):
            c=d.contact[i]
            if m.geom('floor').id in (c.geom1,c.geom2):
                force=np.zeros(6);mujoco.mj_contactForce(m,d,i,force);normal+=force[0]
        return dict(time_s=float(d.time),position_m=d.qpos[:3].tolist(),com_m=d.subtree_com[m.body('robot').id].tolist(),roll_deg=math.degrees(sample.roll),pitch_deg=math.degrees(sample.pitch),contacts=sorted(contacts),normal_force_n=normal,max_penetration_m=penetration,voltage_v=self.voltage,current_estimate_a=self.current,torque_nm=d.ctrl.tolist(),actual_deg=np.degrees(d.qpos[self.q]).tolist(),target_deg=np.degrees(self.desired).tolist(),max_tracking_error_deg=float(np.degrees(abs(self.desired-d.qpos[self.q])).max()),torque_limit_fraction=self.saturated,above_rated_fraction=float(np.mean(abs(d.ctrl)>self.rated)))


def main():
    parser=argparse.ArgumentParser(__doc__)
    parser.add_argument('--check',action='store_true');parser.add_argument('--duration',type=float,default=12)
    parser.add_argument('--stride-scale',type=float,default=None,help='override forward excursion 1..2; omitted uses firmware default')
    parser.add_argument('--linear',type=float,default=.6);parser.add_argument('--yaw',type=float,default=0)
    balance_flags=parser.add_mutually_exclusive_group()
    balance_flags.add_argument('--balance',dest='no_balance',action='store_false',help='enable experimental shared IMU feedback (currently unstable on this rig)')
    balance_flags.add_argument('--no-balance',dest='no_balance',action='store_true')
    parser.set_defaults(no_balance=True)
    parser.add_argument('--output',type=Path,default=Path('/private/tmp/spot-cad-physics'))
    args=parser.parse_args()
    if not all(math.isfinite(v) for v in (args.duration,args.linear,args.yaw)) or args.duration<=0 or max(abs(args.linear),abs(args.yaw))>1:parser.error('Invalid duration/input')
    if args.stride_scale is not None and (not math.isfinite(args.stride_scale) or not 1<=args.stride_scale<=2):parser.error('stride-scale must be within 1..2')
    sim=Simulation();m,d=sim.model,sim.data
    # Physically settle on the floor; initialization is the only qpos assignment.
    for _ in range(100):sim.step(balance=not args.no_balance)
    start=d.qpos[:3].copy();start_height=d.subtree_com[m.body('robot').id,2];rows=[];viewer=None;requested=[args.linear,args.yaw]
    def keypress(key):
        values={87:(1,0),83:(-1,0),65:(0,-1),68:(0,1),32:(0,0)}
        if key in values:requested[:]=values[key]
    if not args.check:
        import mujoco.viewer as mv
        viewer=mv.launch_passive(m,d,key_callback=keypress);viewer.opt.geomgroup[3]=0
        viewer.cam.distance=1.1;viewer.cam.azimuth=135;viewer.cam.elevation=-20
    print(f'NEW STEP DYNAMICS: gravity, contacts, free body, 12 torque motors; estimated mass {m.body_mass.sum():.3f} kg. W/S/A/D/Space.',flush=True)
    started=time.monotonic();fallen=False
    try:
        for i in range(math.ceil(args.duration/.02)):
            if viewer and not viewer.is_running():break
            sim.step(*requested,balance=not args.no_balance,startup=i*.02/.7,stride_scale=args.stride_scale);row=sim.row();rows.append(row)
            r=d.xmat[m.body('robot').id].reshape(3,3)
            fallen=r[2,2]<.5 or row['com_m'][2]<start_height*.55
            if viewer:
                viewer.cam.lookat[:]=row['com_m'];viewer.sync();time.sleep(max(0,started+(i+1)*.02-time.monotonic()))
            if fallen:break
        args.output.mkdir(parents=True,exist_ok=True)
        report=dict(state='FALLEN' if fallen else 'UPRIGHT',mode='estimated rigid-body dynamics',samples=len(rows),mass_kg=float(m.body_mass.sum()),displacement_m=(d.qpos[:3]-start).tolist(),max_tracking_error_deg=max((r['max_tracking_error_deg'] for r in rows),default=0),min_voltage_v=min((r['voltage_v'] for r in rows),default=sim.voltage),max_penetration_m=max((r['max_penetration_m'] for r in rows),default=0),mean_normal_force_n=float(np.mean([r['normal_force_n'] for r in rows])) if rows else 0,mean_above_rated_fraction=float(np.mean([r['above_rated_fraction'] for r in rows])) if rows else 0,parameters=sim.p)
        report['experiment']=dict(initial_linear=args.linear,initial_yaw=args.yaw,stride_scale=args.stride_scale,balance=not args.no_balance,requested_duration_s=args.duration,elapsed_s=len(rows)*.02,settle_s=2.0)
        (args.output/'summary.json').write_text(json.dumps(report,indent=2));(args.output/'frames.json').write_text(json.dumps(rows))
        print(json.dumps({k:v for k,v in report.items() if k!='parameters'}),flush=True)
        if viewer:
            print('Experiment ended; close window to exit.',flush=True)
            while viewer.is_running():viewer.sync();time.sleep(.05)
    finally:
        if viewer:viewer.close()

if __name__=='__main__':main()
