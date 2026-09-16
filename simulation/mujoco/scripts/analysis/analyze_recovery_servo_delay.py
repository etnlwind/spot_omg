"""Separate command delay, acceleration-profile lag and loaded joint tracking.

The supported replay receives the exact quantized commands/register values from
the floor run. Its phase is not recomputed from different tracking errors. Motor
limits and leg inertia remain unchanged. This is a simulator diagnosis, not a
measurement of physical servo firmware.
"""
import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[4]))
import mujoco
import numpy as np
from simulation.mujoco.runtime.cad_physics import Simulation, foot_clearance
from simulation.mujoco.runtime.supported_demo import supported_plant
from simulation.mujoco.runtime.virtual_robot import RobotController, load_parameters, parse_args
from simulation.mujoco.scripts.analysis.analyze_knee_liftoff import tuck_experiment
from simulation.mujoco.scripts.analysis.analyze_j1_steady_hold import j1_experiment


def snapshot(plant, t, delayed, command):
    m,d=plant.model,plant.data
    feet=[m.geom(leg+'_foot').id for leg in ('fl','fr','rl','rr')]
    load=np.zeros(4)
    for n,c in enumerate(d.contact):
        if m.geom('floor').id not in (c.geom1,c.geom2):continue
        other=c.geom2 if c.geom1==m.geom('floor').id else c.geom1
        if other in feet:
            force=np.zeros(6);mujoco.mj_contactForce(m,d,n,force)
            load[feet.index(other)]+=max(0.,force[0])
    return dict(time_s=t,command_deg=command.tolist(),
        encoded_deg=np.degrees(plant.desired).tolist(),
        delayed_deg=np.degrees(delayed).tolist(),
        reference_deg=np.degrees(plant.filtered).tolist(),
        reference_velocity_deg_s=np.degrees(plant.target_velocity).tolist(),
        actual_deg=np.degrees(d.qpos[plant.q]).tolist(),
        actual_velocity_deg_s=np.degrees(d.qvel[plant.v]).tolist(),
        torque_nm=d.ctrl.tolist(),torque_limit_nm=plant.limits.tolist(),
        contact_generalized_torque_nm=d.qfrc_constraint[plant.v].tolist(),
        clearance_mm=[foot_clearance(m,d,f)*1000 for f in feet],
        load_n=load.tolist(),registers=plant.servo_profile.snapshot())


def run(config, output, seconds=8.):
    cfg=json.loads(config.read_text(encoding='utf-8'))
    parameters=load_parameters(parse_args([]))
    voltage=cfg.get('pack_open_circuit_voltage',11.1)
    parameters['pack_open_circuit_voltage']=voltage
    plant=Simulation(parameters);robot=RobotController(plant)
    rows=[]
    with tuck_experiment(**cfg['trajectory']),j1_experiment(cfg.get('j1_mode','fixed_j1')):
        robot.select_profile('s_native_v6_2_6')
        for i in range(round((2+seconds)/.02)):
            t=i*.02
            if i==100:robot.command('drive 1000 0 1',t)
            elif i>100 and i%10==0:robot.command(f'@D {i} 1000 0',t)
            delayed=plant.delay[0].copy()
            robot.tick(t)
            row=snapshot(plant,t,delayed,robot.command_target)
            row.update(phase=float(getattr(robot,'nominal_phase',0)),safety=robot.safety)
            rows.append(row)
    supported=supported_plant(voltage)
    replay=[]
    for row in rows:
        registers=row['registers']
        supported.servo_profile.set(registers['goal_speed'],registers['acceleration'])
        delayed=supported.delay[0].copy()
        command=np.array(row['command_deg'])
        supported.step(targets_deg=command,balance=False,native_servo=True)
        replay.append(snapshot(supported,row['time_s'],delayed,command))
    # Profiles depend on commands/registers, never on ground contact.
    reference_error=float(np.max(np.abs(np.array([r['reference_deg'] for r in rows])-
                                          np.array([r['reference_deg'] for r in replay]))))
    assert reference_error<1e-9, reference_error
    summary=dict(config=str(config),walk_seconds=seconds,
        explanation='Supported fixed-command replay isolates ground/body load. References identical; actual joints may differ.',
        identical_reference_max_error_deg=reference_error,
        modeled_command_delay_s=parameters['command_delay_s'],
        acceleration_limit_deg_s2=np.degrees(plant.servo_profile.acceleration_limit()).tolist(),
        velocity_limit_deg_s=np.degrees(plant.servo_profile.velocity_limit(plant.speed)).tolist(),
        parameters={key:parameters[key] for key in ('servo_kp','servo_kd','motor_3215','motor_3250')})
    output.mkdir(parents=True,exist_ok=True)
    (output/'floor.json').write_text(json.dumps(rows),encoding='utf-8')
    (output/'supported-replay.json').write_text(json.dumps(replay),encoding='utf-8')
    (output/'summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
    print(json.dumps(summary,indent=2),flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--seconds',type=float,default=8.)
    a=p.parse_args();run(a.config,a.output,a.seconds)
