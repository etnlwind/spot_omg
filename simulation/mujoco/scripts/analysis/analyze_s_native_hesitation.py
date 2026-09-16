"""Separate trajectory dwell from tracking holds; no hardware connection.

Candidate overrides live only in a copied simulation profile. Installed servo
caps, voltage, collision geometry and tracking/fault guards remain unchanged.
"""
import argparse
import ctypes as ct
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[4]))
import numpy as np
import mujoco
from simulation.mujoco.runtime.s_native_gait import SNativeGait
from simulation.mujoco.runtime.standing_pose import SoleKinematics
from simulation.mujoco.runtime.supported_demo import supported_plant
from simulation.mujoco.runtime.virtual_robot import RobotController, load_parameters, parse_args
from simulation.mujoco.runtime.cad_physics import Simulation, foot_clearance

ORIGINAL_POINTS = SNativeGait.points


def candidate_points(self, phase, amplitude, linear, yaw):
    result = ORIGINAL_POINTS(self, phase, amplitude, linear, yaw)
    candidate = self.profile.get('_hesitation_candidate', 'baseline')
    if candidate not in ('cosine', 'linear_cosine', 'linear_quintic'):
        return result
    q = (phase + np.array([.5, 0., 0., .5])) % 1
    u = q % .5 / .5
    c = np.cos(2*np.pi*q)
    if candidate == 'linear_quintic':
        s = u**3*(10+u*(-15+6*u))
        c = np.where(q < .5, 1-2*s, -1+2*s)
    extra = self.profile['rear_extension_m']
    stride = (self.profile['params'][2]-extra)/2
    x = stride*c-extra*((1-c)/2)**3
    if candidate.startswith('linear_'):
        x = -extra/2 + (stride+extra/2)*c
    result[:, 0] = self.origin[:, 0] + amplitude*(linear*x+.1*yaw*self.origin[:, 1]*c)
    activity = min(1., (abs(linear)+abs(yaw))/.15)
    result[:, 1] = self.origin[:, 1] + amplitude*(-.1*yaw*self.origin[:, 0]*c + activity*self.walking_lateral_offset)
    return result


def trial(candidate='baseline', voltage=10.9, seconds=16., floor=False, profile='s_native_v6_2_5', lift=None, deceleration=None, max_rate=None):
    if floor:
        parameters = load_parameters(parse_args([]))
        parameters['pack_open_circuit_voltage'] = voltage
        plant = Simulation(parameters)
    else:
        plant = supported_plant(voltage)
    robot = RobotController(plant)
    robot.select_profile(profile)
    if deceleration is not None:
        tracker=robot.tracking
        fn=tracker.lib.spot_tracking_step_rates
        fn.argtypes=[ct.c_void_p,ct.c_uint32,ct.c_float,ct.c_int,ct.c_float,ct.c_float,ct.POINTER(ct.c_float)]
        fn.restype=ct.c_float
        def step(now,dt,stopping,responsive=False,**kwargs):
            diag=(ct.c_float*4)()
            tracker.rate=float(fn(tracker.state,round(now*1000),dt,int(stopping),deceleration,2.,diag))
            tracker.diagnostic=dict(rate=tracker.rate,peak_error_deg=diag[0],oldest_ms=diag[1],fault=int(diag[2]),blocked_ms=diag[3])
            return tracker.rate
        tracker.step=step
    if max_rate is not None:
        previous_step=robot.tracking.step
        def capped_step(now,dt,stopping,responsive=False,**kwargs):
            rate=previous_step(now,dt,stopping,responsive,**kwargs)
            robot.tracking.rate=rate if stopping else min(rate,max_rate)
            return robot.tracking.rate
        robot.tracking.step=capped_step
    cfg = robot.profiles[profile]
    cfg['_hesitation_candidate'] = candidate
    if candidate != 'baseline':
        cfg.pop('placement_swing', None)
    if lift is not None:
        cfg['params'][3] = lift
    kin = SoleKinematics(plant.model, plant.stand_target)
    floor_id = plant.model.geom('floor').id
    rows = []
    stop_tick = 100+round(seconds/.02)
    SNativeGait.points = candidate_points
    try:
        for i in range(stop_tick+200):
            t = i*.02
            if i == 100:
                robot.command('drive 1000 0 1', t)
            elif i == stop_tick:
                robot.command('@S 10000', t)
            elif 100 < i < stop_tick and i % 10 == 0:
                robot.command(f'@D {i} 1000 0', t)
            robot.tick(t)
            q = np.degrees(plant.data.qpos[plant.q])
            kin.set_angles(robot.command_target)
            target_feet = [kin.foot(j).tolist() for j in range(4)]
            kin.set_angles(q)
            actual_feet = [kin.foot(j).tolist() for j in range(4)]
            gait = getattr(robot, 's_native_gait', None)
            load = np.zeros(4)
            for index, contact in enumerate(plant.data.contact):
                if floor_id not in (contact.geom1, contact.geom2):
                    continue
                other = contact.geom2 if contact.geom1==floor_id else contact.geom1
                if other in kin.feet:
                    force = np.zeros(6)
                    mujoco.mj_contactForce(plant.model, plant.data, index, force)
                    load[list(kin.feet).index(other)] += max(0., force[0])
            state = plant.row()
            rows.append(dict(time_s=t, phase=float(getattr(robot, 'nominal_phase', robot.phase)),
                phase_next=float(robot.phase), entry_phase=float(getattr(gait, 'entry_phase', 0)),
                rate=robot.tracking.rate, tracking=robot.tracking.diagnostic.copy(),
                target=robot.command_target.tolist(), actual=q.tolist(),
                target_feet_m=target_feet, actual_feet_m=actual_feet,
                force_n=load.tolist(),roll_deg=state['roll_deg'],pitch_deg=state['pitch_deg'],
                clearance_mm=[foot_clearance(plant.model, plant.data, f)*1000 for f in kin.feet],
                internal_contacts=sum(floor_id not in (c.geom1,c.geom2) for c in plant.data.contact),
                voltage_v=plant.voltage, safety=robot.safety,
                motion=bool(robot.motion), stopping=bool(robot.stopping_reason),
                reply=robot.drain().decode()))
    finally:
        SNativeGait.points = ORIGINAL_POINTS
    selected = [r for r in rows if 4 <= r['time_s'] < 2+seconds and r['safety']=='ok' and r['motion'] and not r['stopping']]
    summary = dict(candidate=candidate, profile=profile, voltage=voltage, floor=floor,lift=lift,deceleration=deceleration,max_rate=max_rate,
        first_fault_s=next((r['time_s'] for r in rows if r['safety']!='ok'), None),
        completed=not robot.motion and not robot.transition and robot.safety=='ok',
        selected_seconds=len(selected)*.02,
        final_s_error_deg=float(np.max(abs(np.degrees(plant.data.qpos[plant.q])-robot.stand_target))))
    if selected:
        target=np.array([r['target'] for r in selected]);actual=np.array([r['actual'] for r in selected])
        summary.update(mean_phase_rate=float(np.mean([r['rate'] for r in selected])),
            phase_hold_seconds=sum(r['rate']<.001 for r in selected)*.02,
            max_joint_error_deg=float(np.max(abs(target-actual))),
            internal_contact_frames=sum(r['internal_contacts']>0 for r in selected),
            target_excursion_deg=np.ptp(target,axis=0).tolist(),actual_excursion_deg=np.ptp(actual,axis=0).tolist())
        phase=(np.array([r['phase'] for r in selected])[:,None]+[.5,0,0,.5])%1
        middle=(phase>=.6)&(phase<=.9)
        loaded=np.array([r['force_n'] for r in selected])>1
        counts=middle.sum(axis=0)
        summary['loaded_mid_swing_fraction']=np.divide((middle&loaded).sum(axis=0),counts,
            out=np.zeros(4),where=counts>0).tolist()
        summary['max_selected_tilt_deg']=max(max(abs(r['roll_deg']),abs(r['pitch_deg'])) for r in selected)
        for key in ('target','actual'):
            feet=np.array([r[key+'_feet_m'] for r in selected])
            speed=np.linalg.norm(np.diff(feet,axis=0),axis=2)/.02
            summary[key+'_low_foot_speed_fraction']=float(np.mean(speed<.02))
            summary[key+'_peak_foot_speed_mm_s']=float(np.max(speed)*1000)
    return summary, rows


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--candidate',choices=['baseline','no_hold','cosine','linear_cosine','linear_quintic'],default='baseline')
    p.add_argument('--profile',default='s_native_v6_2_5')
    p.add_argument('--voltage',type=float,default=10.9)
    p.add_argument('--seconds',type=float,default=16)
    p.add_argument('--floor',action='store_true')
    p.add_argument('--lift',type=float)
    p.add_argument('--deceleration',type=float)
    p.add_argument('--max-rate',type=float)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    summary,rows=trial(a.candidate,a.voltage,a.seconds,a.floor,a.profile,a.lift,a.deceleration,a.max_rate)
    a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(dict(summary=summary,rows=rows),indent=2),encoding='utf-8')
    print(json.dumps(summary),flush=True)
