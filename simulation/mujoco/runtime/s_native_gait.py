"""S-native CAD diagonal trot; V6.1 targets are mirrored by the STM32 kernel."""

# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[3]))
import numpy as np
from simulation.mujoco.runtime.standing_pose import SoleKinematics

NAME = 's_native_v6_2_7'
# Preserve the 0.4s swing, halve the hold at each diagonal exchange from
# 0.6s to 0.3s (at full input). Duty describes each leg's actual stance time.
PERIOD = 1.4
TRANSFER_FRACTION = 3/28
V1_PROFILE = dict(family='trot', params=[PERIOD, .5+2*TRANSFER_FRACTION, .04, .025],
               balance_base='cruise', s_native=True, experimental=True, transfer_fraction=TRANSFER_FRACTION, support_transfer=True, lift_exponent=.5, body_transfer_x_m=.006)

# Remove both 0.3s exchange holds, retaining the same 0.4s swing at full input.
V2_PROFILE = dict(V1_PROFILE, params=[.8, .5, .04, .025], transfer_fraction=0.)
# Retain +20mm placement and extend the rearward push to -60mm.
# The extra reach begins behind S; the starting pose and forward placement stay unchanged.
V3_PROFILE = dict(V2_PROFILE, params=[.8, .5, .08, .025], rear_extension_m=.04)
# Walking footprint below the J1 hip axes; S itself remains unchanged.
V4_PROFILE = dict(V3_PROFILE, params=list(V3_PROFILE['params']), walking_foot_y='j1', placement_entry_amplitude=.5)
# Start with FR/RL together. Only FR doubles its normal J1 angular change
# on that first step; the next half-cycle brings all four to normal adduction.
V5_PROFILE = dict(V4_PROFILE, params=list(V4_PROFILE['params']),
               entry_sequence='fr_double', start_phase=.5, entry_adduction_end=1.,
               normal_adduction_limit_deg=9.)
# Bounded lift and slower support exchange reduce the vertical impulse around
# FL/RR takeoff. These are position-trajectory limits, not force feedback.
# Keep shared pair phase/X/Z, rear reach, and the original FR entry sequence.
V6_PROFILE = dict(V5_PROFILE, params=[1.2, .5, .08, .012])
# V6.1 extends only the rear endpoint by 5mm: +20/-65mm about S.
V61_PROFILE = dict(V6_PROFILE, params=[1.2, .5, .085, .012], rear_extension_m=.045,
               stop_on_next_placement=True, stop_lift_m=.012, stop_period_s=1.2,
               stop_timing=(.2,.4,.45,.7))
# Independent V6.2 copy: the rear endpoint approaches a vertical lower leg
# in the CAD side view. Forward placement, S and the two-step STOP are inherited.
V62_PROFILE = dict(V61_PROFILE, params=[2.0, .5, .145, .012], rear_extension_m=.105,
               reverse_input_limit=.6)
V621_PROFILE = dict(V62_PROFILE, params=list(V62_PROFILE['params']),
               continuous_recovery=True, lift_exponent=.25, stop_period_s=1.6)
# Preserve the entire V6.2.1 phase geometry, including its support reference.
# This time-compressed experiment is not a retuned floor-balance policy.
V622_PROFILE = dict(V621_PROFILE, params=[.5, .5, .145, .012], support_reference_period_s=2.)
# Longer rear stroke with finite-acceleration lift at both swing boundaries.
# Allow travel time; a short clock is not a substitute for actual excursion.
V623_PROFILE = dict(V622_PROFILE, params=[1., .5, .155, .012], rear_extension_m=.115,
               smooth_lift_fraction=.35)
# Retain V623 lateral support reference for the supported-body reach trial.
V624_PROFILE = dict(V623_PROFILE, params=[1., .5, .145, .012], rear_extension_m=.105,
               support_reference_profile='s_native_v6_2_3', feedback_tracking=True)
V625_PROFILE = dict(V624_PROFILE, params=list(V624_PROFILE['params']),
               placement_swing=(.2,.8),smooth_lift_fraction=.3,
               tracking_responsive=True,tracking_samples_per_frame=2)
V626_PROFILE = dict(V625_PROFILE, params=list(V625_PROFILE['params']),
               uniform_recovery=True, tracking_deceleration=6.)
V626_PROFILE.pop('placement_swing')
# Advance recovery travel while lifting/folding, then extend continuously on
# approach. The old push endpoints and tracking protections are unchanged.
# 16 mm / 1.8 s survived the 30 s floor screen; larger folds tipped or dragged.
PROFILE = dict(V626_PROFILE, params=[1.8, .5, .145, .016],
               recovery_frontload=True, smooth_lift_fraction=.25,
               steady_j1_hold=True)
PROFILES = {'s_native_v1': V1_PROFILE, 's_native_v2': V2_PROFILE,
            's_native_v3': V3_PROFILE, 's_native_v4': V4_PROFILE,
            's_native_v5': V5_PROFILE, 's_native_v6': V6_PROFILE,
            's_native_v6_1': V61_PROFILE, 's_native_v6_2': V62_PROFILE, 's_native_v6_2_1': V621_PROFILE, 's_native_v6_2_2': V622_PROFILE, 's_native_v6_2_3': V623_PROFILE, 's_native_v6_2_4':V624_PROFILE, 's_native_v6_2_5':V625_PROFILE, 's_native_v6_2_6':V626_PROFILE, NAME: PROFILE}

class SNativeGait:
    def __init__(self, model, standing, profile=None):
        self.profile = PROFILE if profile is None else profile
        self.kin = SoleKinematics(model, standing)
        self.standing = np.asarray(standing).copy()
        self.kin.set_angles(standing)
        self.origin = np.array([self.kin.foot(i) for i in range(4)])
        self.previous = self.standing.copy()
        self.center=np.average(self.kin.data.xipos,axis=0,weights=model.body_mass)
        self.height=self.center[2]-self.origin[:,2].mean()
        self.walking_lateral_offset=np.zeros(4)
        if self.profile.get('walking_foot_y') == 'j1':
            hips=np.array([self.kin.data.xanchor[model.joint(leg+'_j1').id,1]
                           for leg in ('fl','fr','rl','rr')])
            self.walking_lateral_offset=hips-self.origin[:,1]
        self.placement_fraction=np.zeros(4)
        self.placement_active=np.zeros(4,dtype=bool)
        self.previous_leg_phase=None
        self.entry_phase=0.
        self.entry_previous_phase=None
        self.normal_previous=self.standing.copy()
        self.stop_progress=None
        if self.profile.get('entry_sequence') == 'fr_double':
            normal_points=self.origin.copy()
            normal_points[:,1]+=self.walking_lateral_offset
            normal,error=self.kin.solve(normal_points,self.standing,iterations=60)
            if error>.0002:raise ValueError('Normal J1 reference unreachable')
            self.normal_adduction=normal[::3]-self.standing[::3]
            if 'normal_adduction_limit_deg' in self.profile:
                limit=float(self.profile['normal_adduction_limit_deg'])
                if not 0<limit<=15:raise ValueError('Normal adduction limit must be within 0..15 degrees')
                self.normal_adduction=np.clip(self.normal_adduction,-limit,limit)
                normal,error=self.kin.solve_xz(normal_points,self.standing,
                    self.standing[::3]+self.normal_adduction)
                if error>.0002:raise ValueError('Limited normal adduction reference unreachable')
                self.walking_lateral_offset=np.array([self.kin.foot(i)[1] for i in range(4)])-self.origin[:,1]
            self.kin.set_angles(self.standing)

    def points(self, phase, amplitude, linear, yaw):
        # FL/RR share one oscillator; FR/RL are exactly half a cycle apart.
        # At phase zero RL/FR are forward and RR/FL backward, relative to S.
        q = (phase + np.array([.5, 0., 0., .5])) % 1.
        margin=self.profile["transfer_fraction"]
        u=np.clip((q%.5-margin)/(.5-2*margin),0.,1.)
        smooth=u*u*u*(10+u*(-15+6*u))
        c=np.where(q<.5,1-2*smooth,-1+2*smooth)
        if 'placement_swing' in self.profile:
            a,b=self.profile['placement_swing']
            u=np.clip(((q-.5)/.5-a)/(b-a),0.,1.)
            c=np.where(q>=.5,-1+2*u**3*(10+u*(-15+6*u)),c)
        extra = self.profile.get("rear_extension_m", 0.)
        stride = (self.profile["params"][2]-extra)/2 * linear
        delta = np.zeros((4, 3))
        # Cubic extension is C2 at S: extra push only behind the shoulder.
        delta[:, 0] = stride*c - extra*linear*np.maximum(0.,-c)**3
        if self.profile.get('continuous_recovery'):
            # Spread rear reach across the complete stroke. Starting the
            # extra displacement only behind S caused a crossing-speed dip.
            # Both X and knee flexion continue throughout recovery.
            progress=(1-c)/2
            delta[:,0]=stride*c-extra*linear*progress**3
        if self.profile.get('uniform_recovery'):
            # Same +20/-125mm endpoints; distribute travel across the complete
            # quintic stroke instead of concentrating the rear reach in time.
            delta[:,0]=-extra*linear/2+(stride+extra*linear/2)*c
        if self.profile.get('recovery_frontload'):
            # Beta(3,5) CDF: no X reversal/dwell, zero velocity and
            # acceleration at endpoints. Yaw retains its existing oscillator.
            u=np.clip((q-.5)*2,0.,1.)
            progress=1-(1-u)**5*(1+5*u+15*u*u)
            old=u**3*(10+u*(-15+6*u))
            delta[:,0]+=linear*self.profile['params'][2]*(progress-old)*(q>=.5)
        # Protocol yaw is right-positive. During stance c decreases, and
        # planted feet move opposite the body's rotation: use clockwise
        # placement here so positive yaw turns the body clockwise too.
        delta[:, 0] += .10*yaw*self.origin[:, 1]*c
        delta[:, 1] = -.10*yaw*self.origin[:, 0]*c
        activity = min(1., (abs(linear)+abs(yaw))/.15)
        delta[:,1] += activity*self.walking_lateral_offset
        # First half is stance front->rear; second half swings rear->front.
        margin=self.profile["transfer_fraction"]
        swing=np.clip((q-.5-margin)/(.5-2*margin),0.,1.)
        shape=np.where((swing>0)&(swing<1),np.maximum(0.,np.sin(np.pi*swing))**self.profile.get("lift_exponent",2.),0.)
        if "smooth_lift_fraction" in self.profile:
            rise=self.profile["smooth_lift_fraction"]
            u=np.clip(np.minimum(swing,1-swing)/rise,0.,1.)
            shape=u**3*(10+u*(-15+6*u))
        delta[:, 2] = self.profile["params"][3]*activity*shape
        return self.origin + amplitude*delta

    def extra_lift(self,points,phase,amplitude,linear,yaw):
        mm=np.asarray(getattr(self,'foot_lift_mm',[0,0,0,0]),dtype=float)
        if mm.shape!=(4,) or not np.isfinite(mm).all() or (mm<0).any() or (mm>2147483647).any():raise ValueError('foot lift must be nonnegative integer mm')
        margin=self.profile['transfer_fraction']
        u=np.clip((((phase+np.array([.5,0,0,.5]))%1)-.5-margin)/(.5-2*margin),0,1)
        result=points.copy()
        result[:,2]+=mm*.001*amplitude*min(1,(abs(linear)+abs(yaw))/.15)*64*u**3*(1-u)**3
        return result

    def apply_width(self,result,amplitude,linear,yaw):
        mm=np.asarray(getattr(self,'foot_width_mm',[0]*4),dtype=float)
        if not np.any(mm):return result
        if mm.shape!=(4,) or not np.isfinite(mm).all() or (np.abs(mm)>2147483647).any():raise ValueError('Invalid foot width')
        self.kin.set_angles(result)
        points=np.array([self.kin.foot(i) for i in range(4)])
        points[:,1]+=np.array([1,-1,1,-1])*mm*.001*amplitude*min(1,(abs(linear)+abs(yaw))/.15)
        result,error=self.kin.solve(points,result,iterations=60)
        if error>.0002 or not np.isfinite(result).all():raise ValueError('Foot width unreachable')
        return result

    def targets(self, phase, amplitude, linear, yaw):
        if self.profile.get('entry_sequence') == 'fr_double':
            return self.fr_entry_targets(phase, amplitude, linear, yaw)
        points = self.command_points(phase, amplitude, linear, yaw)
        if self.profile.get('walking_foot_y'):
            self.update_placement(phase,amplitude)
            activity=min(1.,(abs(linear)+abs(yaw))/.15)
            # Entry only changes a lifted pair's footprint. The first complete
            # swing after startup moves it from S to the hip line; stance feet
            # retain their lateral placement until their own swing arrives.
            points[:,1]+=(self.placement_fraction-amplitude)*activity*self.walking_lateral_offset
        points=self.extra_lift(points,phase,amplitude,linear,yaw)
        result, error = self.kin.solve(points, self.previous, iterations=60)
        if error > .001 or not np.isfinite(result).all():
            raise ValueError(f'S-native target unreachable: {error:.6f} m')
        if self.profile.get('support_transfer'):
            points[:,1]+=amplitude*self.support_displacement(phase,linear,yaw)
            result,error=self.kin.solve(points,result,iterations=60)
            if error>.001:raise ValueError('S-native support transfer unreachable')
        result=self.apply_width(result,amplitude,linear,yaw)
        self.previous = result.copy()
        self.last_target_points=points.copy()
        return result

    def update_fr_entry(self, phase, amplitude):
        """Continuous phase clock: first FR/RL swing, then FL/RR swing.

        Coefficients multiply J1's change FROM S, not Cartesian foot distance.
        The second half-cycle also relaxes the planted FR smoothly; there is
        no phase hold or extra all-feet preparation posture.
        """
        if amplitude <= 1e-8:
            self.entry_phase=0.
            self.placement_fraction[:]=0.
        elif self.entry_previous_phase is not None:
            advance=(phase-self.entry_previous_phase)%1.
            self.entry_phase+=advance
        self.entry_previous_phase=phase
        if amplitude <= 1e-8:return
        if self.entry_phase <= .5:
            u=np.clip(self.entry_phase/(.5*self.profile.get('entry_adduction_end',1.)),0.,1.)
            self.placement_fraction[:]=0.
            self.placement_fraction[1]=2*u**3*(10+u*(-15+6*u))
        else:
            u=np.clip((self.entry_phase-.5)/.5,0.,1.)
            blend=u**3*(10+u*(-15+6*u))
            self.placement_fraction[:]=blend
            self.placement_fraction[1]=2-blend

    def fr_entry_targets(self, phase, amplitude, linear, yaw):
        if self.stop_progress is not None:
            return self.fr_stop_targets(phase,amplitude,linear,yaw)
        self.update_fr_entry(phase,amplitude)
        activity=min(1.,(abs(linear)+abs(yaw))/.15)
        if amplitude <= 1e-8 or activity <= 1e-8:
            self.previous=self.standing.copy()
            self.last_target_points=self.origin.copy()
            self.normal_j1=self.standing[::3].copy()
            return self.previous.copy()
        points=self.command_points(phase,amplitude,linear,yaw)
        # Compute the normal walking angle for this X/Z pose. The first
        # adduction completes with the first landing, independently of the
        # existing one-second fore/aft stride amplitude ramp.
        points[:,1]+=(1-amplitude)*activity*self.walking_lateral_offset
        if self.profile.get('support_transfer'):
            points[:,1]+=amplitude*self.support_displacement(phase,linear,yaw)
        normal,error=self.kin.solve(points,self.normal_previous,iterations=60)
        if error>.001 or not np.isfinite(normal).all():
            raise ValueError('S-native normal adduction unreachable')
        self.normal_previous=normal.copy()
        self.normal_j1=normal[::3].copy()
        # Double the geometric S-to-normal adduction, not the extra angle
        # needed for a lifted foot, yaw or common support displacement.
        u=np.clip((self.entry_phase-.5)/.5,0.,1.)
        correction_blend=u**3*(10+u*(-15+6*u))
        locked=(self.standing[::3]+self.placement_fraction*self.normal_adduction
                +correction_blend*(self.normal_j1-self.standing[::3]-self.normal_adduction))
        if self.profile.get('steady_j1_hold'):
            locked=self.standing[::3]+self.placement_fraction*self.normal_adduction
        # Re-solve J2/J3 with J1 fixed so extra FR adduction cannot shorten
        # its stride or lift it relative to its diagonal partner RL.
        points=self.extra_lift(points,phase,amplitude,linear,yaw)
        result,error=self.kin.solve_xz(points,self.previous,locked,iterations=60)
        if error>.0002 or not np.isfinite(result).all():
            raise ValueError(f'S-native FR entry X/Z unreachable: {error:.6f} m')
        result=self.apply_width(result,amplitude,linear,yaw)
        self.previous=result.copy()
        self.last_target_points=np.array([self.kin.foot(i) for i in range(4)])
        return result

    def begin_stop(self):
        if self.stop_progress is None:
            self.stop_progress=0.
            self.stop_j1=self.previous[::3].copy()
            self.stop_placement=self.placement_fraction.copy()
            if self.profile.get('stop_on_next_placement'):
                self.stop_pose=self.previous.copy()
                self.kin.set_angles(self.stop_pose)
                self.stop_feet=np.array([self.kin.foot(i) for i in range(4)])
                # Complete the lifted diagonal first, then place its partner.
                phase=self.entry_previous_phase if self.entry_previous_phase is not None else .5
                self.stop_first=np.array([0,3] if phase<.5 else [1,2])

    @property
    def stop_ready(self):
        return self.stop_progress is not None and self.stop_progress>=1.

    def fr_stop_targets(self,phase,amplitude,linear,yaw):
        if self.profile.get('stop_on_next_placement'):
            return self.placement_stop_targets()
        # Continue phase while decelerating. Reverse the current angular
        # adduction to S over one nominal cycle, including an interrupted start.
        self.stop_progress=min(1.,self.stop_progress+.02/self.profile['params'][0])
        u=self.stop_progress;remaining=1-u**3*(10+u*(-15+6*u))
        points=self.command_points(phase,amplitude,linear,yaw)
        points[:,2]=self.origin[:,2]+remaining*(points[:,2]-self.origin[:,2])
        locked=self.standing[::3]+remaining*(self.stop_j1-self.standing[::3])
        self.placement_fraction=remaining*self.stop_placement
        result,error=self.kin.solve_xz(points,self.previous,locked,iterations=60)
        if error>.0002:raise ValueError('S-native stop X/Z unreachable')
        self.previous=result.copy()
        self.last_target_points=np.array([self.kin.foot(i) for i in range(4)])
        return result

    def placement_stop_targets(self):
        """Two in-place diagonal placements with J1 held during lift/lowering."""
        self.stop_progress=min(1.,self.stop_progress+.02/self.profile['stop_period_s'])
        first=np.isin(np.arange(4),self.stop_first)
        u=np.clip(2*self.stop_progress-np.where(first,0.,1.),0.,1.)
        def smooth(x):return x*x*x*(10+x*(-15+6*x))
        # Lift, make one placement, lower, then allow contact to settle before
        # the other pair lifts. These are body-relative goals, not sensed contact.
        lift_end,move_end,lower_start,lower_end=self.profile['stop_timing']
        move=smooth(np.clip((u-lift_end)/(move_end-lift_end),0.,1.))
        up=smooth(np.clip(u/lift_end,0.,1.))
        down=smooth(np.clip((u-lower_start)/(lower_end-lower_start),0.,1.))
        points=self.stop_feet.copy()
        points[:,0]+=(self.origin[:,0]-points[:,0])*move
        apex=np.maximum(self.stop_feet[:,2],self.origin[:,2]+self.profile['stop_lift_m'])
        points[:,2]=np.where(u<lower_start,self.stop_feet[:,2]+(apex-self.stop_feet[:,2])*up,
                            apex+(self.origin[:,2]-apex)*down)
        locked=self.stop_j1+(self.standing[::3]-self.stop_j1)*move
        result,error=self.kin.solve_xz(points,self.previous,locked,iterations=60)
        if error>.0002:raise ValueError('S-native stop placement unreachable')
        legs=result.reshape(4,3)
        legs[u<=0]=self.stop_pose.reshape(4,3)[u<=0]
        legs[u>=lower_end]=self.standing.reshape(4,3)[u>=lower_end]
        self.placement_fraction=(1-move)*self.stop_placement
        self.previous=result.copy()
        self.kin.set_angles(result)
        self.last_target_points=np.array([self.kin.foot(i) for i in range(4)])
        return result

    def update_placement(self,phase,amplitude):
        q=(phase+np.array([.5,0.,0.,.5]))%1.
        if amplitude <= 1e-8:
            self.placement_fraction[:]=0.
            self.placement_active[:]=False
        elif self.previous_leg_phase is not None:
            eligible=amplitude>=self.profile.get('placement_entry_amplitude',.5)
            liftoff=(self.previous_leg_phase<.5)&(q>=.5)
            self.placement_active|=eligible&liftoff
            u=np.clip((q-.5)/.5,0.,1.)
            fraction=u**3*(10+u*(-15+6*u))
            landed=self.placement_active&(q<.5)
            self.placement_fraction[landed]=1.
            swinging=self.placement_active&(q>=.5)
            self.placement_fraction[swinging]=np.maximum(self.placement_fraction[swinging],fraction[swinging])
        self.previous_leg_phase=q

    def support_displacement(self,phase,linear,yaw):
        # Periodic linear-inverted-pendulum solution: shift - h/g*shift'' =
        # COM-to-support-line offset evaluated at the fore/aft ZMP.
        if hasattr(self,'support_table'):
            n=len(self.support_table);p=phase%1*n;j=int(p)
            value=self.support_table[j]+(p-j)*(self.support_table[(j+1)%n]-self.support_table[j])
            return min(1.,(abs(linear)+abs(yaw))/.15)*float(np.clip(value,-.015,.015))
        n=256;t=np.arange(n)/n;margin=self.profile['transfer_fraction']
        duration=.5-2*margin;u=np.clip((t%.5-margin)/duration,0,1)
        smooth=u**3*(10+u*(-15+6*u));c=1-2*smooth
        cdd=-120*u*(1-u)*(1-2*u)/duration**2
        extra=self.profile.get('rear_extension_m',0.)*linear
        stride=(self.profile['params'][2]-self.profile.get('rear_extension_m',0.))/2*linear
        cd=-60*u*u*(1-u)**2/duration
        x=stride*c-extra*np.maximum(0.,-c)**3
        xdd=stride*cdd+np.where(c<0,extra*(6*c*cd*cd+3*c*c*cdd),0.)
        if self.profile.get('continuous_recovery'):
            progress=(1-c)/2
            x=stride*c-extra*progress**3
            xdd=(stride+1.5*extra*progress**2)*cdd-1.5*extra*progress*cd**2
        period=self.profile.get('support_reference_period_s',self.profile['params'][0])*(1.35-.35*min(1.,abs(linear)+abs(yaw)))
        h_g=self.height/9.81
        zmp_x=self.center[0]+h_g*xdd/period**2
        slope=np.where(t<.5,-1.,1.)*(self.origin[0,1]-self.origin[1,1])/(self.origin[0,0]-self.origin[2,0])
        line_y=self.origin[:,1].mean()+slope*(zmp_x-self.origin[:,0].mean()-x)
        rhs=self.center[1]-line_y
        spectrum=np.fft.rfft(rhs);frequency=2*np.pi*np.arange(len(spectrum))/period
        shift=np.fft.irfft(spectrum/(1+h_g*frequency**2),n=n)
        p=phase%1*n;j=int(p);value=shift[j]+(p-j)*(shift[(j+1)%n]-shift[j])
        return min(1.,(abs(linear)+abs(yaw))/.15)*float(np.clip(value,-.015,.015))

    def prepare_support(self,linear,yaw):
        """Periodic inverse-dynamics support reference, computed from CAD only."""
        import mujoco
        if self.profile.get('support_reference_profile'):
            if not hasattr(self,'support_reference'):
                self.support_reference=SNativeGait(self.kin.model,self.standing,
                    PROFILES[self.profile['support_reference_profile']])
            self.support_reference.prepare_support(linear,yaw)
            if hasattr(self.support_reference,'support_table'):
                self.support_table=self.support_reference.support_table
            return
        if abs(linear)+abs(yaw)<.1:return
        key=(round(linear,1),round(yaw,1))
        if getattr(self,'support_key',None)==key:return
        self.support_key=key
        n=64;phases=np.arange(n)/n;k=self.kin;m=k.model
        poses=[];feet=[];seed=self.standing.copy()
        for phase in phases:
            points=self.command_points(phase,1.,linear,yaw)
            seed,error=k.solve(points,seed,iterations=60)
            if error>.001:raise ValueError('S-native dynamics reference unreachable')
            k.set_angles(seed)
            poses.append(np.radians(seed));feet.append(np.array([k.contact(i) for i in range(4)]))
        poses=np.array(poses);feet=np.array(feet)
        period=self.profile.get('support_reference_period_s',self.profile['params'][0])*(1.35-.35*min(1.,abs(linear)+abs(yaw)))
        dt=period/n
        velocity=(np.roll(poses,-1,axis=0)-np.roll(poses,1,axis=0))/(2*dt)
        acceleration=(np.roll(poses,-1,axis=0)-2*poses+np.roll(poses,1,axis=0))/dt**2
        acceleration=np.clip(acceleration,-25.,25.)
        # Use velocity of the current contact material point, not the
        # derivative of the moving lowest-vertex locus on a rolling cushion.
        base_velocity=[]
        for j,phase in enumerate(phases):
            k.set_angles(np.degrees(poses[j]));d=k.data
            ids=[1,2] if phase<.5 else [0,3];velocities=[]
            for i in ids:
                point=k.contact(i);jp=np.zeros((3,m.nv));jr=np.zeros_like(jp)
                mujoco.mj_jac(m,d,jp,jr,point,int(m.geom_bodyid[k.feet[i]]))
                velocities.append(jp[:,k.v]@velocity[j])
            base_velocity.append(-np.mean(velocities,axis=0))
        base_velocity=np.array(base_velocity)
        base_acceleration=np.clip((np.roll(base_velocity,-1,axis=0)-np.roll(base_velocity,1,axis=0))/(2*dt),-5,5)
        rhs=[];load=float(m.body_mass.sum())*9.81;ground=self.origin[:,2].mean()
        for j,phase in enumerate(phases):
            k.set_angles(np.degrees(poses[j]));d=k.data
            d.qvel[:]=0;d.qacc[:]=0
            d.qvel[k.v]=velocity[j];d.qacc[k.v]=acceleration[j]
            d.qvel[:3]=base_velocity[j];d.qacc[:3]=base_acceleration[j]
            mujoco.mj_comVel(m,d);force=np.zeros(m.nv);mujoco.mj_rne(m,d,1,force)
            zmp_x=(ground*force[0]-force[4])/load
            zmp_y=(force[3]+ground*force[1])/load
            ids=[1,2] if phase<.5 else [0,3]
            a,b=feet[j,ids,:2]
            line_y=a[1]+(zmp_x-a[0])/(b[0]-a[0])*(b[1]-a[1])
            rhs.append(zmp_y-line_y)
        spectrum=np.fft.rfft(rhs);frequency=2*np.pi*np.arange(len(spectrum))/period
        self.support_table=np.fft.irfft(spectrum/(1+self.height/9.81*frequency**2),n=n)
        k.set_angles(self.previous)

    def body_transfer(self,phase,linear):
        margin=self.profile['transfer_fraction'];u=np.clip((phase%.5-margin)/(.5-2*margin),0,1)
        c=1-2*u**3*(10+u*(-15+6*u))
        # Bound preserves each leg's extrema at the common phase boundaries.
        bound=.225*(self.profile['params'][2]-self.profile.get('rear_extension_m',0.))*abs(linear)
        amount=np.clip(self.profile.get('body_transfer_x_m',0.),-bound,bound)
        return -amount*(1-c*c)

    def command_points(self,phase,amplitude,linear,yaw):
        points=self.points(phase,amplitude,linear,yaw)
        if self.profile.get('entry_sequence') == 'fr_double' and amplitude>1e-8:
            # The first pair must actually clear the ground before adducting.
            # The old stride ramp made the entire first swing only 0..4mm high.
            points[:,2]=self.points(phase,1.,linear,yaw)[:,2]
        points[:,0]+=amplitude*self.body_transfer(phase,linear)
        return points
