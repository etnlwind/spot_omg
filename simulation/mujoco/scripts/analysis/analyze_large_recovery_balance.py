"""Floor-only controlled experiments for the large-recovery failure.

Plant truth is recorded for evaluation only. Candidate control uses nominal
geometry/phase and, if requested, the controller's delayed IMU observation.
No registered model or firmware changes.
"""
import sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[4]
sys.path[:0]=[str(ROOT),str(ROOT/'tools/servo_tool')]
import argparse
import json
import numpy as np
from simulation.mujoco.runtime.cad_physics import Simulation,foot_clearance
from simulation.mujoco.runtime.virtual_robot import RobotController,load_parameters,parse_args
from simulation.mujoco.runtime.gait_evidence import foot_loads,swing_quality
from simulation.mujoco.scripts.analysis.capture_j2_85_candidate import smooth

OFFSETS=np.array([.5,0,0,.5])

def natural_stance_x(local,duty,front,span,k):
    """Inverted-pendulum stance and C2 return, in phase coordinates."""
    rear=front-span;mid=(front+rear)/2;half=span/2
    if local<duty:
        return mid-half*np.sinh(k*(local/duty-.5))/np.sinh(k/2)
    u=(local-duty)/(1-duty)
    velocity=-half*k/duty/np.tanh(k/2)*(1-duty)
    acceleration=half*(k/duty)**2*(1-duty)**2
    a0=rear;a1=velocity;a2=-acceleration/2
    a3,a4,a5=np.linalg.solve([[1,1,1],[3,4,5],[6,12,20]],
        [front-a0-a1-a2,velocity-a1-2*a2,acceleration-2*a2])
    return a0+u*(a1+u*(a2+u*(a3+u*(a4+u*a5))))

class Recovery:
    def __init__(self,config):
        if config.get('fixed_frame'):
            forbidden=('crouch','j1','support','roll_gain','pitch_gain','centering',
                       'shift_gain','capture_gain','orientation_gain','low_stop')
            if any(config.get(key) for key in forbidden) or config.get('j1') is not None:
                raise ValueError('Fixed frame experiment forbids body lowering and J1/body-pose corrections')
        self.c=config;self.previous_attitude=None;self.rate=np.zeros(2)
        self.shift=np.zeros(3)
        self.observed=np.full(12,np.nan);self.support_shift=np.zeros(2)
        self.duties=np.full(4,config['steady_duty'] if config.get('coordinated_entry') else .5)
        self.previous_local=None;self.installed=False
        self.entry_clock=0.;self.entry_previous=None
        self.balance_x=0.

    def setup(self,gait):
        c=self.c
        if c.get('coordinated_entry'):
            def update_entry(phase,amplitude):
                if amplitude<=1e-8:
                    self.entry_clock=0.;self.entry_previous=phase
                    gait.entry_phase=0.;gait.placement_fraction[:]=0.;return
                if self.entry_previous is not None:self.entry_clock+=(phase-self.entry_previous)%1
                self.entry_previous=phase;gait.entry_previous_phase=phase
                duration=1-c['steady_duty'];e=self.entry_clock
                first=smooth(e/duration);second=smooth((e-.5)/duration)
                gait.placement_fraction[:]=second;gait.placement_fraction[1]=2*first-second
                if e<=duration:gait.entry_phase=.5*e/duration
                elif e<=.5:gait.entry_phase=.5
                elif e<=.5+duration:gait.entry_phase=.5+.5*(e-.5)/duration
                else:gait.entry_phase=1+e-(.5+duration)
            gait.update_fr_entry=update_entry
        if c.get('steady_duty'):
            base_points=gait.points
            def points(phase,amp,linear,yaw):
                local=(phase+OFFSETS)%1
                mapped=np.where(local<self.duties,local/self.duties*.5,.5+(local-self.duties)/(1-self.duties)*.5)
                result=np.array([base_points((mapped[i]-OFFSETS[i])%1,amp,linear,yaw)[i] for i in range(4)])
                if c.get('constant_stance') or c.get('natural_stance'):
                    span=gait.profile['params'][2]
                    front=(span-gait.profile.get('rear_extension_m',0))/2
                    rear=front-span
                    for i in range(4):
                        if self.duties[i]<=.5:continue
                        d=self.duties[i];s=np.clip((local[i]-d)/(1-d),0,1);k=(1-d)/d
                        x=front-span*local[i]/d if local[i]<d else rear+span*((1+k)*smooth(s)-k*s)
                        if c.get('natural_stance'):
                            period=gait.profile['params'][0]*(1.35-.35*min(1,abs(linear)+abs(yaw)))
                            exponent=np.sqrt(9.81/gait.height)*period*d*c.get('natural_scale',1)
                            x=natural_stance_x(local[i],d,front,span,exponent)
                        reference=base_points((mapped[i]-OFFSETS[i])%1,amp,linear,0)[i,0]
                        result[i,0]+=gait.origin[i,0]+amp*linear*x-reference
                return result
            gait.points=points
        if c.get('constant_stance') or c.get('natural_stance'):
            body_transfer=gait.body_transfer
            gait.body_transfer=lambda phase,linear: body_transfer(phase,linear) if gait.entry_phase<1 else 0.
        if c.get('low_stop'):
            begin=gait.begin_stop
            def begin_stop():
                begin()
                kin=gait.kin;points=gait.origin.copy();points[:,2]+=c.get('crouch',0)*smooth(gait.entry_phase)
                low,error=kin.solve_xz(points,gait.standing,gait.standing[::3])
                if error>.0002:raise ValueError('Low stop S unreachable')
                gait.standing=low;gait.origin=points
                if self.previous_local is not None:
                    swinging=self.previous_local>=self.duties
                    if swinging[0]:gait.stop_first=np.array([0,3])
                    elif swinging[1]:gait.stop_first=np.array([1,2])
            gait.begin_stop=begin_stop
        self.installed=True

    def prepare(self,gait,phase):
        if not self.installed:self.setup(gait)
        local=(phase+OFFSETS)%1
        if self.c.get('steady_duty') and self.previous_local is not None and gait.stop_progress is None:
            landed=(local<self.previous_local-.5)
            if gait.entry_phase>=1.-1e-6:self.duties[landed]=self.c['steady_duty']
        self.previous_local=local

    def __call__(self,robot,gait,phase,nominal):
        if gait.stop_progress is not None:return nominal
        c=self.c;kin=gait.kin;q=nominal.copy()
        local=(phase+OFFSETS)%1
        if c.get('steady_duty'):
            local=np.where(local<self.duties,local/self.duties*.5,.5+(local-self.duties)/(1-self.duties)*.5)
        if c.get('mapped_duty'):
            duty=c['mapped_duty']
            local=np.where(local<duty,local/duty*.5,.5+(local-duty)/(1-duty)*.5)
        gait.experimental_leg_phase=local.copy()
        swing=np.clip((local-.5)*2,0,1)
        weight=smooth(swing/c.get('rise',.25))*(1-smooth((swing-c.get('rise',.25))/c.get('fall',.65)))
        weight*=gait.entry_phase>=c.get('start',1)
        if c.get('steady_duty'):weight*=self.duties>.5
        if c.get('ramp',0):weight*=smooth((gait.entry_phase-c.get('start',1))/c['ramp'])
        if c.get('crouch',0):
            kin.set_angles(q);points=np.array([kin.foot(i) for i in range(4)])
            points[:,2]+=c['crouch']*smooth(gait.entry_phase)
            q,error=kin.solve_xz(points,q,q[::3])
            if error>.0002:raise ValueError('Walking body height unreachable')
        if c.get('toe_lift',0) and gait.entry_phase>=1:
            kin.set_angles(q);points=np.array([kin.foot(i) for i in range(4)])
            shape=smooth(swing/.2)*(1-smooth((swing-.65)/.35))
            points[:,2]+=c['toe_lift']*shape
            q,error=kin.solve_xz(points,q,q[::3])
            if error>.0002:raise ValueError('Toe clearance unreachable')
        if c.get('late_lift',0):
            kin.set_angles(q);points=np.array([kin.foot(i) for i in range(4)])
            shape=smooth((swing-.35)/.3)*(1-smooth((swing-.8)/.2))
            points[:,2]+=c['late_lift']*shape
            q,error=kin.solve_xz(points,q,q[::3])
            if error>.0002:raise ValueError('Late clearance unreachable')
        if c.get('j1') is not None:
            kin.set_angles(q);points=np.array([kin.foot(i) for i in range(4)])
            locked=q[::3]+smooth((gait.entry_phase-1)/.5)*(c['j1']-q[::3])
            q,error=kin.solve_xz(points,q,locked)
            if error>.0002:raise ValueError('Width adjustment unreachable')
        if c.get('mode','joint')=='joint':
            q[1::3]+=weight*np.maximum(0,c.get('j2',85)-q[1::3])
            q[2::3]+=weight*np.maximum(0,c.get('j3',104)-q[2::3])
        elif c['mode']=='cartesian':
            kin.set_angles(q);points=np.array([kin.foot(i) for i in range(4)])
            points[:,2]+=weight*c.get('lift',.04)
            q,error=kin.solve_xz(points,q,q[::3])
            if error>.0002:raise ValueError('Cartesian recovery unreachable')
        if c.get('diagonal_x_balance') and gait.entry_phase>=1 and robot.imu_reading is not None:
            reading=robot.imu_reading
            if reading['age_ms']<=100 and not robot.attitude_filter.failures:
                kin.set_angles(q);points=np.array([kin.foot(i) for i in range(4)])
                center=np.average(kin.data.xipos,axis=0,weights=kin.model.body_mass)
                attitude=np.radians(np.array(robot.attitude_filter.filtered)/10.)
                if reading.get('gyro_axis_verified') and reading.get('gyro_age_ms',1000)<=100:
                    attitude+=c.get('balance_lead',.08)*np.array(reading['gyro_body_rad_s'][:2])
                rr,pp=attitude
                rx=np.array([[1,0,0],[0,np.cos(rr),-np.sin(rr)],[0,np.sin(rr),np.cos(rr)]])
                ry=np.array([[np.cos(pp),0,np.sin(pp)],[0,1,0],[-np.sin(pp),0,np.cos(pp)]])
                rotation=ry@rx;feet=points@rotation.T;com=rotation@center
                shifts=[]
                for ids in ([1,2],[0,3]):
                    a,b=feet[ids];v=(b-a)[:2];normal=np.array([-v[1],v[0]])
                    denominator=normal@rotation[:2,0]
                    shifts.append(float(normal@(com[:2]-(a+b)[:2]/2)/denominator)
                                  if abs(denominator)>.005 else 0.)
                pair=0 if phase<.5 else 1
                overlap=max(.001,float(np.min(self.duties))-.5)
                blend=smooth((phase%.5)/overlap)
                demand=(shifts[pair]*blend+shifts[1-pair]*(1-blend))*c['diagonal_x_balance']
                demand=np.clip(demand,-c.get('balance_cap',.04),c.get('balance_cap',.04))
                self.balance_x+=np.clip(demand-self.balance_x,-c.get('balance_step',.0015),c.get('balance_step',.0015))
                points[:,0]+=self.balance_x*smooth((gait.entry_phase-1)/.5)
                q,error=kin.solve_xz(points,q,nominal[::3])
                if error>.0002:raise ValueError('Fixed-height diagonal X balance unreachable')
        if c.get('fore_aft_shift'):
            kin.set_angles(q);points=np.array([kin.foot(i) for i in range(4)])
            points[:,0]+=c['fore_aft_shift']*smooth((gait.entry_phase-.5)/.5)
            q,error=kin.solve_xz(points,q,nominal[::3])
            if error>.0002:raise ValueError('Fore-aft balance target unreachable')
        if c.get('preload') and np.all(self.duties>.5):
            kin.set_angles(q);points=np.array([kin.foot(i) for i in range(4)])
            raw=(phase+OFFSETS)%1;window=self.duties-.5
            early=np.clip(raw/window,0,1);late=np.clip((raw-.5)/window,0,1)
            pulse=lambda u:64*u**3*(1-u)**3
            dz=c['preload']*(pulse(late)-pulse(early))
            points[:,2]+=dz
            q,error=kin.solve_xz(points,q,nominal[::3])
            if error>.0002:raise ValueError('Load-transfer preload unreachable')
        if c.get('xz_balance_gain') and robot.imu_reading is not None and robot.imu_reading['age_ms']<=100 and not robot.attitude_filter.failures:
            kin.set_angles(q);points=np.array([kin.foot(i) for i in range(4)])
            stance=local<.5
            center=points[stance].mean(axis=0) if np.any(stance) else points.mean(axis=0)
            roll,pitch=np.radians(np.array(robot.attitude_filter.filtered)/10.)
            dz=c['xz_balance_gain']*(roll*(points[:,1]-center[1])-pitch*(points[:,0]-center[0]))
            w=smooth(local/.05)*(1-smooth((local-.45)/.05))*stance
            dz=np.clip(dz,-.015,.015)*w*smooth((gait.entry_phase-.5)/.5)
            # Zero mean on the supporting feet preserves commanded body
            # height; only J2/J3 distribute extension to control attitude.
            if np.any(stance):dz[stance]-=dz[stance].mean()
            points[:,2]+=dz
            q,error=kin.solve_xz(points,q,nominal[::3])
            if error>.0002:raise ValueError('Fixed-J1 attitude target unreachable')
        if c.get('support',0) or c.get('roll_gain',0) or c.get('pitch_gain',0) or c.get('centering',0) or c.get('shift_gain',0) or c.get('capture_gain',0):
            kin.set_angles(q);points=np.array([kin.foot(i) for i in range(4)])
            gate=smooth((gait.entry_phase-.5)/.5)
            # Feedforward reference is deliberately tested independently:
            # the preserved fixed-J1 path computes it but discards its Y.
            points[:,1]+=gate*c.get('support',0)*gait.support_displacement(phase,robot.linear,robot.yaw)
            if c.get('capture_gain',0) and np.isfinite(self.observed).all() and robot.imu_reading is not None:
                kin.set_angles(self.observed)
                measured_feet=np.array([kin.foot(i) for i in range(4)])
                center=np.average(kin.data.xipos,axis=0,weights=kin.model.body_mass)
                reading=robot.imu_reading
                rr,pp=np.radians([reading['roll_tenths']/10,reading['pitch_tenths']/10])
                rx=np.array([[1,0,0],[0,np.cos(rr),-np.sin(rr)],[0,np.sin(rr),np.cos(rr)]])
                ry=np.array([[np.cos(pp),0,np.sin(pp)],[0,1,0],[-np.sin(pp),0,np.cos(pp)]])
                rotation=ry@rx
                feet_world=measured_feet@rotation.T;center_world=rotation@center
                a,b=feet_world[[1,2] if phase<.5 else [0,3]]
                direction=(b-a)[:2];normal=np.array([-direction[1],direction[0]])
                normal/=np.linalg.norm(normal)
                if normal[1]<0:normal=-normal
                relative=center_world-(a+b)/2
                error=normal@relative[:2]
                gyro=np.array(reading.get('gyro_body_rad_s',[0,0,0]))
                speed=normal@np.cross(rotation@gyro,relative)[:2]
                demand=(error+c.get('capture_lead',.1)*speed)*normal*c['capture_gain']
                body_demand=(rotation.T@np.r_[demand,0])[:2]
                desired=self.support_shift+body_demand if c.get('capture_integral') else body_demand
                self.support_shift+=np.clip(desired-self.support_shift,-.0015,.0015)
                self.support_shift=np.clip(self.support_shift,-.045,.045)
                points[:,:2]+=gate*self.support_shift
            if c.get('centering',0):
                future=(phase+c.get('lead',0))%1
                reference=gait.command_points(future,1.,robot.linear,robot.yaw)
                predicted,error=kin.solve_xz(reference,nominal,gait.standing[::3]+gait.normal_adduction)
                if error>.0002:raise ValueError('Support preview unreachable')
                p=np.array([kin.foot(i) for i in range(4)])
                center=np.average(kin.data.xipos,axis=0,weights=kin.model.body_mass)
                a,b=p[[1,2] if future<.5 else [0,3],:2]
                line_y=a[1]+(center[0]-a[0])*(b[1]-a[1])/(b[0]-a[0])
                desired=c['centering']*(center[1]-line_y)
                self.shift[0]+=np.clip(desired-self.shift[0],-.001,.001)
                points[:,1]+=gate*np.clip(self.shift[0],-.045,.045)
            if robot.imu_reading is not None and robot.imu_reading['age_ms']<=100 and not robot.attitude_filter.failures:
                attitude=np.radians(np.array(robot.attitude_filter.filtered)/10.)
                if self.previous_attitude is not None:
                    self.rate=.8*self.rate+.2*(attitude-self.previous_attitude)/.02
                self.previous_attitude=attitude
                if c.get('gyro') and robot.imu_reading.get('gyro_axis_verified') and robot.imu_reading.get('gyro_age_ms',1000)<=100:
                    self.rate=np.array(robot.imu_reading['gyro_body_rad_s'][:2])
                roll,pitch=attitude+c.get('damping',0)*self.rate
                # Deliberate stance rotation correction, with full CAD IK
                # so J1 can transfer support instead of dropping Y goals.
                dz=c.get('roll_gain',0)*roll*points[:,1]-c.get('pitch_gain',0)*pitch*points[:,0]
                points[:,2]+=gate*np.clip(dz,-.025,.025)*(1 if c.get('all_feet') else local<.5)
                desired=-c.get('shift_gain',0)*roll*gait.height
                speed=c.get('shift_step',.0005)
                self.shift[1]+=np.clip(desired-self.shift[1],-speed,speed)
                points[:,1]+=gate*np.clip(self.shift[1],-c.get('shift_cap',.02),c.get('shift_cap',.02))
            q,error=kin.solve(points,q,iterations=60)
            if error>.0002:raise ValueError('Support correction unreachable')
        if c.get('orientation_gain') and robot.imu_reading is not None:
            reading=robot.imu_reading
            if reading['age_ms']<=100 and not robot.attitude_filter.failures:
                kin.set_angles(q);points=np.array([kin.foot(i) for i in range(4)])
                center=np.average(kin.data.xipos,axis=0,weights=kin.model.body_mass)
                angles=np.radians(np.array(robot.attitude_filter.filtered)/10)
                if reading.get('gyro_axis_verified'):
                    angles+=c.get('orientation_lead',.08)*np.array(reading['gyro_body_rad_s'][:2])
                for leg in range(4):
                    gain=c['orientation_gain']
                    if c.get('swing_level_gain') is not None:
                        blend=smooth(swing[leg]/.2)*(1-smooth((swing[leg]-.8)/.2))
                        gain=gain*(1-blend)-c['swing_level_gain']*blend
                    rr,pp=np.clip(angles*gain,-.15,.15)*smooth((gait.entry_phase-.5)/.5)
                    rx=np.array([[1,0,0],[0,np.cos(rr),-np.sin(rr)],[0,np.sin(rr),np.cos(rr)]])
                    ry=np.array([[np.cos(pp),0,np.sin(pp)],[0,1,0],[-np.sin(pp),0,np.cos(pp)]])
                    points[leg]=(ry@rx)@(points[leg]-center)+center
                q,error=kin.solve(points,q,iterations=60)
                if error>.0002:raise ValueError('Orientation target unreachable')
        # Stop must begin from the command actually sent, not the pre-bump
        # nominal pose. This only changes the solver seed during walking.
        gait.previous=q.copy()
        return q

def install_recovery(robot,config):
    """Install the same offline candidate in the evidence/video runner."""
    correction=Recovery(config);gait=robot.s_native_gait;original=gait.targets
    if config.get('period'):gait.profile['params'][0]=config['period']
    if config.get('coordinated_entry'):robot.phase=config['steady_duty']
    def adjusted(phase,amp,linear,yaw):
        correction.prepare(gait,phase)
        return correction(robot,gait,phase,original(phase,amp,linear,yaw))
    gait.targets=adjusted
    return correction

def run(config,output,seconds=8,command=344,voltage=11.1):
    params=load_parameters(parse_args([]));params['pack_open_circuit_voltage']=voltage
    plant=Simulation(params);robot=RobotController(plant);robot.select_profile('s_native_v6_2_7')
    if config.get('period'):robot.profiles[robot.profile]['params'][0]=config['period']
    if config.get('no_heading'):robot.heading.enabled=False
    correction=Recovery(config);rows=[];stop_tick=100+round(seconds/.02)
    tracking_sample=robot.tracking.sample
    def observed_sample(now,target,actual,drop=False):
        index=robot.tracking.index
        tracking_sample(now,target,actual,drop)
        if not drop:correction.observed[index]=round(float(actual[index])*4096/360)*360/4096
    robot.tracking.sample=observed_sample
    if config.get('duty'):
        # Change swing/stance allocation, not cycle period, and keep motion
        # continuous. Map each leg through its unchanged spatial path.
        robot.profiles[robot.profile]['experimental_duty']=config['duty']
    feet=[plant.model.geom(l+'_foot').id for l in ('fl','fr','rl','rr')]
    rid=plant.model.body('robot').id
    for i in range(stop_tick+200):
        t=i*.02
        if i==100:
            robot.command(f'drive {command} 0 1',t)
            if config.get('coordinated_entry'):robot.phase=config['steady_duty']
            gait=robot.s_native_gait;original=gait.targets
            if config.get('duty'):
                old_points=gait.points
                def points(phase,amp,linear,yaw):
                    local=(phase+OFFSETS)%1
                    duty=config['duty']
                    mapped=np.where(local<duty,local/duty*.5,.5+(local-duty)/(1-duty)*.5)
                    result=[]
                    for leg in range(4):
                        result.append(old_points((mapped[leg]-OFFSETS[leg])%1,amp,linear,yaw)[leg])
                    return np.array(result)
                gait.points=points
            def adjusted(phase,amp,linear,yaw):
                correction.prepare(gait,phase)
                nominal=original(phase,amp,linear,yaw)
                if config.get('duty'):
                    # A common phase transform cannot encode duty>0.5 for
                    # both pairs; Recovery computes per-leg local phases.
                    correction.c['mapped_duty']=config['duty']
                return correction(robot,gait,phase,nominal)
            gait.targets=adjusted
        elif i==stop_tick:robot.command('@S 1000',t)
        elif 100<i<stop_tick and i%10==0:robot.command(f'@D {i} {command} 0',t)
        robot.tick(t);state=plant.row();d=plant.data
        phase=float(getattr(robot,'nominal_phase',0));gait=getattr(robot,'s_native_gait',None)
        local=(phase+OFFSETS)%1
        duties=correction.duties if config.get('steady_duty') else config.get('duty',.5)
        local=np.where(local<duties,local/duties*.5,.5+(local-duties)/(1-duties)*.5)
        footpos=d.geom_xpos[feet].copy();com=d.subtree_com[rid].copy()
        pair=[1,2] if phase<.5 else [0,3]
        a,b=footpos[pair,:2];v=b-a
        offset=com[:2]-a
        distance=float((v[0]*offset[1]-v[1]*offset[0])/np.linalg.norm(v))
        rows.append(dict(time_s=t,phase=phase,leg_phase=local.tolist(),entry_phase=float(getattr(gait,'entry_phase',0)),
            roll_deg=state['roll_deg'],pitch_deg=state['pitch_deg'],safety=robot.safety,
            walking=bool(robot.motion is not None and not robot.stopping_reason),
            com_m=com.tolist(),body_position_m=d.qpos[:3].tolist(),body_quaternion=d.qpos[3:7].tolist(),
            body_velocity=d.qvel[:6].tolist(),foot_centers_m=footpos.tolist(),
            torso_height_m=float(d.xipos[plant.model.body('cad_base').id,2]),
            imu=robot.imu_reading,filtered_attitude=robot.attitude_filter.filtered,
            support_line_distance_mm=distance*1000,
            clearance_mm=[foot_clearance(plant.model,d,f)*1000 for f in feet],loads_n=foot_loads(plant.model,d),
            target=robot.command_target.tolist(),actual=state['actual_deg'],torque_nm=state['torque_nm'],
            rate=float(robot.tracking.rate),reply=robot.drain().decode()))
    drive=rows[100:stop_tick]
    evidence=[r for r in drive if r['walking'] and r['safety']=='ok']
    steady=[r for r in evidence if r['entry_phase']>=1]
    safe=all(r['safety']=='ok' for r in rows)
    summary=dict(config=config,command=command,voltage=voltage,walk_seconds=seconds,
        safety_ok=safe,first_fault_s=next((r['time_s'] for r in rows if r['safety']!='ok'),None),
        max_tilt_deg=max(max(abs(r['roll_deg']),abs(r['pitch_deg'])) for r in drive),
        gait_quality=swing_quality(evidence),steady_quality=swing_quality(steady),
        final_error_deg=float(np.max(np.abs(np.array(rows[-1]['actual'])-robot.stand_target))),
        stop_ok=bool(safe and not robot.motion and not robot.transition and np.max(np.abs(robot.command_target-robot.stand_target))<.01),
        displacement_m=(np.array(drive[-1]['body_position_m'])-rows[99]['body_position_m']).tolist())
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(dict(summary=summary,rows=rows)),encoding='utf-8')
    return summary

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--configs',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True);parser.add_argument('--seconds',type=float,default=8)
    parser.add_argument('--command',type=int,default=344);parser.add_argument('--voltage',type=float,default=11.1)
    args=parser.parse_args();configs=json.loads(args.configs.read_text())
    summaries=[]
    for name,config in configs.items():
        try:
            s=run(config,args.output/(name+'.json'),args.seconds,args.command,args.voltage)
            summaries.append(dict(name=name,**s))
            print(name,'fault',s['first_fault_s'],'tilt',round(s['max_tilt_deg'],2),'drag',
                [v['loaded_fraction'] for v in s['steady_quality']['legs'].values()],flush=True)
        except Exception as e:
            summaries.append(dict(name=name,error=repr(e)));print(name,repr(e),flush=True)
    args.output.mkdir(parents=True,exist_ok=True)
    (args.output/'summary.json').write_text(json.dumps(summaries,indent=2),encoding='utf-8')

if __name__=='__main__':main()
