"""Run and record physical MuJoCo states for calculated-placement previews."""

# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[4]))

from simulation.mujoco.paths import REPO_ROOT, SIM_ROOT, RESULTS_ROOT
from pathlib import Path
import argparse
import json
import numpy as np
import mujoco
from simulation.mujoco.runtime.cad_physics import Simulation, foot_clearance
from simulation.mujoco.runtime.virtual_robot import RobotController
from simulation.mujoco.runtime.calculated_placement_preview import configure, DUTY

ROOT = REPO_ROOT


def parameters():
    p = json.loads((SIM_ROOT/'cad_300mm/physics_parameters_measured_total_2754g.json').read_text())
    p.update(timestep_s=.0005, experimental_stow=True)
    p['foot_cushion'] = json.loads((SIM_ROOT/'config/foot_cushion_d37p3_l27mm.json').read_text())
    return p


def run(mode, destination, duration=30.):
    p = parameters(); plant = Simulation(p); robot = RobotController(plant)
    robot.select_profile('centerpivot')
    geometry, config = configure(robot, mode)
    m, d = plant.model, plant.data
    feet = [m.geom(l+'_foot').id for l in ('fl','fr','rl','rr')]
    floor = m.geom('floor').id
    rows, fault = [], None
    for i in range(round(duration/.02)):
        t = i*.02
        if i == 500:
            robot.command('drive 1000 0 1', t)
        elif i > 500 and i % 10 == 0 and fault is None:
            robot.command(f'@D {i} 1000 0', t)
        try:
            robot.tick(t)
        except RuntimeError as e:
            if 'target unreachable' not in str(e):
                raise
            fault = dict(t=t, reason=str(e), type='kinematic-unreachable')
            robot.safety = 'planner'
            robot.finish_stop('unreachable calculated foot target')
            robot.tick(t)
        if fault is None and robot.calculated_placement.fault:
            fault = dict(t=t, reason=robot.calculated_placement.fault, type='placement-infeasible')
            robot.safety='planner'; robot.finish_stop(fault['reason'])
        if fault is None and robot.safety != 'ok':
            fault = dict(t=t, reason=robot.safety, type='existing-safety-stop')
        messages = robot.drain()
        if isinstance(messages,bytes):messages=messages.decode('utf-8',errors='replace')
        force = np.zeros(4)
        for j,c in enumerate(d.contact):
            if floor not in (c.geom1,c.geom2): continue
            other = c.geom2 if c.geom1 == floor else c.geom1
            if other in feet:
                value=np.zeros(6); mujoco.mj_contactForce(m,d,j,value)
                force[feet.index(other)] += max(0.,value[0])
        state = plant.row()
        matrix = d.xmat[m.body('robot').id].reshape(3,3)
        rows.append(dict(t=t, physics_time_s=float(d.time),
            phase=getattr(robot,'ground_evaluated_phase',robot.phase), linear=robot.linear,
            qpos=d.qpos.tolist(), qvel=d.qvel.tolist(), ctrl=d.ctrl.tolist(),
            roll=state['roll_deg'],pitch=state['pitch_deg'],
            yaw=float(np.degrees(np.arctan2(matrix[1,0],matrix[0,0]))),com=state['com_m'],
            force=force.tolist(),clearance=[foot_clearance(m,d,f)*1000 for f in feet],
            command=robot.command_target.tolist(),actual=np.degrees(d.qpos[plant.q]).tolist(),
            torque=robot.torque,safety=robot.safety,pose=robot.pose,
            moving=robot.motion is not None,sat=plant.saturated,voltage=plant.voltage,
            placement=robot.calculated_placement.diagnostic.copy(),messages=messages))
    com=np.array([r['com'] for r in rows])
    # Central differences below are evaluation only. Filtered acceleration is
    # additionally reported because differentiating contact impacts is noisy.
    from scipy.signal import savgol_filter
    for i,(v,a) in enumerate(zip(savgol_filter(com,21,3,deriv=1,delta=.02,axis=0),
                                 savgol_filter(com,21,3,deriv=2,delta=.02,axis=0))):
        rows[i]['actual_com_velocity_m_s']=v.tolist()
        rows[i]['actual_com_acceleration_m_s2']=a.tolist()
    active=[r for r in rows if r['t']>=10 and (fault is None or r['t']<=fault['t'])]
    a=lambda k:np.asarray([r[k] for r in active])
    phase=(a('phase')[:,None]+[0,.5,.5,0])%1
    u=(phase-DUTY)/(1-DUTY); middle=(u>1/6)&(u<5/6)
    events=robot.calculated_placement.events
    summary=dict(mode=mode,fault=fault,duration_s=duration,requested_forward_s=duration-10,
        observed_forward_until_s=(fault['t'] if fault else duration),
        mass_kg=float(m.body_mass.sum()),roll_max_deg=float(abs(a('roll')).max()),
        pitch_max_deg=float(abs(a('pitch')).max()),roll_rms_deg=float(np.sqrt(np.mean(a('roll')**2))),
        pitch_rms_deg=float(np.sqrt(np.mean(a('pitch')**2))),
        forward_displacement_m=float(a('com')[-1,0]-a('com')[0,0]),
        height_peak_to_peak_mm=float(np.ptp(a('com')[:,2])*1000),
        mid_swing_contact_fraction=[float(np.mean(a('force')[middle[:,j],j]>1.)) if middle[:,j].any() else None for j in range(4)],
        max_tracking_error_deg=float(abs(a('command')-a('actual')).max()),
        max_target_speed_rad_s=float(np.max(abs(np.diff(a('command'),axis=0)))*np.pi/180/.02) if len(active)>1 else 0.,
        final_pose=robot.pose,final_torque=robot.torque,
        planned_event_count=len(events),actuator_speed_and_torque_limits_simulated=True,
        dynamic_feasibility='experimental; point-support prediction is not a guarantee of physical tracking')
    out=dict(summary=summary,config=config,parameters=p,geometry=geometry,events=events,records=rows,
        recording='Actual physical qpos/qvel at 50Hz; render every second stored frame at25fps. No target-pose substitution.',
        comparison='Original-like 1.35s/.52/96mm/32mm reference; heading and legacy balance OFF in every case.',
        sensor_and_model='Measured total2.754kg; estimated distribution, actuators and pads; delayed BNO +40ms quantized encoders.',
        acceleration_evaluation='Offline COM Savitzky-Golay21 samples/3rd order at50Hz; never fed into planner.')
    destination.parent.mkdir(parents=True,exist_ok=True)
    destination.write_text(json.dumps(out,indent=2))
    print(json.dumps(summary),flush=True)


if __name__ == '__main__':
    parser=argparse.ArgumentParser();parser.add_argument('mode',choices=['baseline','static','acceleration','inward','inward_prepared','underbody'])
    parser.add_argument('--output',type=Path);args=parser.parse_args()
    run(args.mode,args.output or ROOT/'artifacts/audits/sway-2026-09-14'/f'calculated-{args.mode}.json')
