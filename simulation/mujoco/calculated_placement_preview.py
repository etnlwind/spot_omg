"""Isolated snapshot-static and acceleration-aware touchdown experiments.

No physical contact/force or floating-body state enters the planner. Encoder
acquisition is explicitly quantized/delayed. The velocity estimate assumes
planned stance feet do not slip; this assumption is assessed offline, not hidden.
"""
import numpy as np
import mujoco
from position_wbc import EncoderChannel
from support_shift import SupportShift, OFFSETS, smooth
from footstep_tracker import rotation

PERIOD = 1.35
DUTY = .52
STRIDE = .096
STARTUP = 1.
STATIC_SHIFT = np.array([-.0465427534863789, 0., 0.])
INWARD_DISTANCE = .010


def tilted_pose(kin, angles, orientation):
    kin.set_angles(angles)
    quat=np.zeros(4);mujoco.mju_mat2Quat(quat,orientation.ravel())
    kin.data.qpos[3:7]=quat
    mujoco.mj_kinematics(kin.model,kin.data);mujoco.mj_comPos(kin.model,kin.data)


def solve_tilted(kin, points, seed, orientation):
    """CAD IK of geom-center XY / rotated lowest-cushion-vertex Z."""
    q=np.asarray(seed,dtype=float).copy()
    for _ in range(30):
        tilted_pose(kin,q,orientation)
        error=[];dq=np.zeros(12)
        for i in range(4):
            actual,jac=kin.foot(i,True);e=points[i]-actual;error.append(np.linalg.norm(e))
            delta=np.linalg.solve(jac.T@jac+np.eye(3)*1e-7,jac.T@e)
            dq[i*3:i*3+3]=np.clip(np.degrees(delta),-4.,4.)
        if max(error)<.0001:break
        q=np.clip(q+dq,np.tile([-30,-45,0],4),np.tile([30,100,150],4))
    tilted_pose(kin,q,orientation)
    error=max(np.linalg.norm(points[i]-kin.foot(i)) for i in range(4))
    return q,float(error)


def composite_inertia(kin):
    m,d=kin.model,kin.data
    c=np.average(d.xipos,axis=0,weights=m.body_mass);tensor=np.zeros((3,3))
    for i,mass in enumerate(m.body_mass):
        r=d.xipos[i]-c;R=d.ximat[i].reshape(3,3)
        tensor+=R@np.diag(m.body_inertia[i])@R.T+mass*(np.dot(r,r)*np.eye(3)-np.outer(r,r))
    return tensor


class CalculatedPlacement:
    def __init__(self, model, mode):
        if mode not in ('baseline', 'static', 'acceleration', 'inward', 'inward_prepared', 'underbody'):
            raise ValueError('Unknown calculated placement mode')
        self.model = model
        self.kin = SupportShift(model)
        self.mode = mode
        self.reset()

    def reset(self):
        self.channel = EncoderChannel(delay_frames=2)
        self.stance = np.ones(4, dtype=bool)
        self.current = np.zeros((4, 3))
        self.start = np.zeros((4, 3))
        self.end = np.zeros((4, 3))
        self.previous_sensor = None
        self.velocity = np.zeros(3)
        self.velocity_updated_s = None
        self.events = []
        self.diagnostic = {}
        self.fault = None

    def reset_motion(self):
        # Preserve packets collected during the 10s idle period so the first
        # diagonal swing is not planned with an artificial cold-start sensor.
        channel, previous = self.channel, self.previous_sensor
        velocity = self.velocity.copy()
        updated = self.velocity_updated_s
        self.reset()
        self.channel, self.previous_sensor = channel, previous
        self.velocity = velocity
        self.velocity_updated_s = updated

    @staticmethod
    def reference(elapsed):
        """Steady stance speed plus a declared smooth startup speed reference.

        This is the desired body speed, not exact A(t)*foot_path differentiation.
        Actual body acceleration is evaluated separately after simulation.
        """
        u = np.clip(elapsed / STARTUP, 0., 1.)
        maximum = STRIDE / (PERIOD * DUTY)
        speed = maximum * smooth(u)
        acceleration = maximum * 30. * u*u*(1.-u)**2 / STARTUP if 0. < u < 1. else 0.
        return float(speed), float(acceleration)

    def observe(self, robot, phase, duty, period):
        # Raw joint truth is confined to this acquisition boundary. No root
        # pose, root velocity, contact force, or actual acceleration is read.
        sample = self.channel.read(np.degrees(robot.plant.data.qpos[robot.plant.q]))
        imu = robot.imu_reading
        healthy = (sample is not None and imu is not None and imu['age_ms'] <= 100
                   and not robot.attitude_filter.failures)
        if not healthy:
            self.previous_sensor = None
            return None
        k = self.kin
        orientation = rotation(*np.radians(np.asarray(robot.attitude_filter.filtered) / 10.))
        tilted_pose(k,sample,orientation)
        feet = np.array([k.foot(i) for i in range(4)])
        com = np.average(k.data.xipos, axis=0, weights=k.model.body_mass)
        phase_delayed = (phase - .04/period + OFFSETS) % 1.
        support = phase_delayed < duty
        relative = com - feet
        velocity_valid = False
        raw_velocity = np.zeros(3)
        if self.previous_sensor is not None:
            old_relative, old_support = self.previous_sensor
            common = support & old_support & (((phase+OFFSETS)%1.) < duty)
            if common.sum() >= 2:
                candidates = (relative[common] - old_relative[common]) / .02
                raw_velocity = np.median(candidates, axis=0)
                velocity_valid = bool(np.max(abs(raw_velocity)) <= .5
                                      and np.max(np.ptp(candidates, axis=0)) <= .25)
                if velocity_valid:
                    self.velocity += (.02 / (.10 + .02)) * (raw_velocity - self.velocity)
                    self.velocity_updated_s=float(getattr(robot.plant.data,'time',robot.elapsed))
        self.previous_sensor = (relative.copy(), support.copy())
        ground = np.median(feet[support, 2]) if support.sum() >= 2 else np.nan
        height = float(com[2] - ground)
        if not np.isfinite(height) or not .08 <= height <= .35:
            return None
        return dict(orientation=orientation, com=com, height=height,
                    ground=ground, encoders=sample.copy(),
                    velocity_valid=velocity_valid, raw_velocity=raw_velocity,
                    velocity_age_s=(float('inf') if self.velocity_updated_s is None
                                    else float(getattr(robot.plant.data,'time',robot.elapsed))-self.velocity_updated_s))

    def dynamic_endpoint(self, goals, ids, robot, sensor, future, period, duty):
        from acceleration_support_targets import acceleration_support_targets
        wanted_velocity,feedforward_a=self.reference(future)
        requested_a=np.array([feedforward_a,0.])+(np.array([wanted_velocity,0.])-self.velocity[:2])/(period*duty)
        acceleration=np.clip(requested_a,-1.,1.)
        # Construct the two-diagonal posture at the planned touchdown epoch.
        # Its torso orientation/ground estimate is frozen from delivered IMU.
        prediction=goals.copy();amp=smooth(min(1.,future/STARTUP))
        other=np.array([i for i in range(4) if i not in ids])
        prediction[ids,0]+=.5*STRIDE*amp
        prediction[other,0]+=STRIDE*amp*(.5-.5/duty)
        prediction[other]+=self.current[other]
        k=self.kin;R=sensor['orientation']
        q,error=k.solve(prediction,sensor['encoders'],iterations=30)
        if error>.001:
            return None,dict(planning_failure='unreachable nominal touchdown',ik_residual_m=error)
        tilted_pose(k,q,R)
        world=np.array([k.foot(i) for i in range(4)])
        angle=np.radians(np.asarray(robot.attitude_filter.filtered)/10.)
        rate=np.radians(np.asarray(robot.attitude_filter.rate)/10.)
        omega=2*np.pi/period
        alpha=-omega**2*angle-2*omega*rate
        torque=composite_inertia(k)[:2,:2]@alpha
        solution=None
        for _ in range(3):
            tilted_pose(k,q,R)
            com=np.average(k.data.xipos,axis=0,weights=k.model.body_mass)
            points=np.array([k.foot(i) for i in range(4)])
            solution=acceleration_support_targets(com[:2],float(com[2]-sensor['ground']),
                acceleration,points[ids,:2],mass_kg=float(self.model.body_mass.sum()),
                mode='x-only',friction_coefficient=.6,centroidal_moment_xy_nm=torque)
            if not solution['feasible']:
                return None,dict(planning_failure='dynamic point-support solution infeasible',solution=solution)
            world[ids,:2]=solution['targets_xy_m']
            world[ids,2]=sensor['ground']
            q,error=solve_tilted(k,world,q,R)
            if error>.001:
                return None,dict(planning_failure='calculated ground-plane touchdown unreachable',
                    ik_residual_m=error,solution=solution,target_world_m=world[ids].tolist())
        tilted_pose(k,q,R)
        planned_world=np.array([k.foot(i) for i in ids])
        k.set_angles(q)
        body=np.array([k.foot(i) for i in range(4)])
        offset=body[ids]-prediction[ids]
        entry=dict(solution=solution,reference_speed_m_s=wanted_velocity,
            feedforward_acceleration_m_s2=feedforward_a,
            velocity_estimate_m_s=self.velocity.tolist(),velocity_estimate_age_s=sensor['velocity_age_s'],
            velocity_sample_accepted=sensor['velocity_valid'],
            raw_requested_acceleration_m_s2=requested_a.tolist(),planned_acceleration_m_s2=acceleration.tolist(),
            acceleration_limited=bool(np.any(abs(requested_a)>1.)),
            desired_roll_pitch_acceleration_rad_s2=alpha.tolist(),centroidal_moment_estimate_nm=torque.tolist(),
            height_estimate_m=sensor['height'],planned_ground_z_m=float(sensor['ground']),
            planned_world_touchdown_m=planned_world.tolist(),
            planned_touchdown_height_difference_m=float(np.ptp(planned_world[:,2])),
            touchdown_ik_residual_m=error,
            prediction_assumption='Current delivered orientation/estimated floor frozen at touchdown; planned joint COM; scheduled no-slip velocity',
            moment_model='Composite CAD inertia times desired angular acceleration; swing momentum not separately predicted')
        return offset,entry

    def update(self, targets, goals, robot, phase, duty, period):
        sensor = self.observe(robot, phase, duty, period)
        local = (phase + OFFSETS) % 1.
        stance = local < duty
        events = []
        for pair in ((0, 3), (1, 2)):
            ids = np.asarray(pair)
            if not stance[pair[0]] and self.stance[pair[0]]:
                shift = self.end[ids].copy()
                entry = dict(elapsed_s=float(robot.elapsed), pair=list(pair), mode=self.mode)
                if self.mode in ('baseline', 'inward_prepared', 'underbody'):
                    shift[:] = 0.
                elif self.mode == 'static':
                    shift = np.tile(STATIC_SHIFT,(2,1))
                    entry['static_source'] = 'original no-correction snapshot 10.80s'
                elif self.mode == 'inward':
                    # Move each foot toward the torso centerline only during
                    # its scheduled swing. CAD IK below coordinates J1/J2/J3
                    # while preserving the original commanded X/Z trajectory.
                    shift = np.zeros((2,3))
                    shift[:,1] = -np.sign(goals[ids,1])*INWARD_DISTANCE
                    entry['inward_distance_m'] = INWARD_DISTANCE
                elif sensor is not None and sensor['velocity_age_s'] <= .12:
                    until_touchdown = (1.-local[pair[0]])*period
                    future = robot.elapsed + until_touchdown
                    proposed,detail=self.dynamic_endpoint(goals,ids,robot,sensor,future,period,duty)
                    entry.update(detail)
                    if proposed is None:self.fault=detail['planning_failure']
                    else:shift=proposed
                else:
                    entry['sensor_hold_previous_endpoint'] = True
                if not np.isfinite(shift).all():
                    self.fault='nonfinite calculated placement'
                if self.fault:
                    entry['planning_failure'] = self.fault
                    shift = self.current[ids].copy()
                self.start[ids] = self.current[ids]
                self.end[ids] = shift
                entry['applied_endpoint_offset_body_m'] = shift.tolist()
                self.events.append(entry); events.append(entry)
            if not stance[pair[0]]:
                u = (local[pair[0]]-duty)/(1.-duty)
                self.current[ids] = self.start[ids]+smooth(u)*(self.end[ids]-self.start[ids])
            elif not self.stance[pair[0]]:
                self.current[ids] = self.end[ids]
        self.stance = stance
        self.diagnostic = dict(mode=self.mode, offset_body_m=self.current.tolist(),
            velocity_estimate_m_s=self.velocity.tolist(), sensor_valid=sensor is not None,
            velocity_sample_accepted=bool(sensor is not None and sensor['velocity_valid']),
            events=events, planning_failure=self.fault,
            estimator='40ms-quantized-encoders + delivered-IMU; scheduled no-slip stance')
        return targets + self.current


def configure(robot, mode):
    from ground_frame_nominal import configure as nominal
    placement = CalculatedPlacement(robot.plant.model, mode)
    config = dict(period_s=PERIOD, duty=DUTY, cartesian_stride_m=STRIDE, startup_s=STARTUP)
    if mode == 'underbody':
        config.update(touchdown_x_m=.020,liftoff_x_m=.020-STRIDE,
            trajectory_note='Contact near J2; backward stance sweep; original Y/Z and 96mm travel. No inward correction.')
    if mode == 'inward':
        config['experimental_inward_distance_m'] = INWARD_DISTANCE
    elif mode == 'inward_prepared':
        placement.kin.set_angles(np.tile([0.,45.,90.],4))
        width=np.mean([abs(placement.kin.foot(i)[1]) for i in range(4)])
        config['stance_half_width_m']=float(width-INWARD_DISTANCE)
        config['experimental_inward_distance_m']=INWARD_DISTANCE
        config['initialization_note']='Already narrowed before idle; walking-entry transition is not exercised.'
    info = nominal(robot, config, placement=placement)
    robot.heading.enabled = False
    robot.balance.enabled = False
    robot.calculated_placement = placement
    old_apply=robot.balance.apply
    def idle_sample(*args,**kwargs):
        if robot.motion is None:
            placement.observe(robot,0.,DUTY,PERIOD)
        return old_apply(*args,**kwargs)
    robot.balance.apply=idle_sample
    return info, config
