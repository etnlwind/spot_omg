"""Replay the same recorded v70 targets through old/new estimated servo references.

This compares a held-body recording, not floor stability or servo identification.
Installed 50/254/50 acceleration caps and the physical motor model are retained.
"""
import argparse
import csv
import hashlib
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
import numpy as np
from simulation.mujoco.runtime.supported_demo import supported_plant


def legacy_reference(profile, position, velocity, goal, dt, nominal_speed):
    # Exact pre-V626 calculation, retained only for this diagnostic replay.
    error = goal-position
    speed = profile.velocity_limit(nominal_speed)
    wanted = np.clip(error/dt, -speed, speed)
    velocity = velocity+np.clip(wanted-velocity,
        -profile.acceleration_limit()*dt, profile.acceleration_limit()*dt)
    increment = velocity*dt
    increment = np.where(np.abs(increment)>np.abs(error), error, increment)
    return position+increment, velocity


def replay(source, voltage, legacy):
    with (source/'commands.csv').open() as f:
        commands = list(csv.DictReader(f))
    with (source/'samples.csv').open() as f:
        samples = list(csv.DictReader(f))
    keys = [k for k in commands[0] if k.endswith('_deg')]
    times = np.array([float(r['send_begin_ms'])/1000 for r in commands])
    angles = np.array([[float(r[k]) for k in keys] for r in commands])
    angles[:, [0, 3]] *= -1  # Hardware front J1 -> CAD, as in native_servo.
    plant = supported_plant(voltage)
    if legacy:
        profile = plant.servo_profile
        profile.advance_reference = lambda *args: legacy_reference(profile, *args)
    actual = [np.degrees(plant.data.qpos[plant.q]).copy()]
    sim_times = [0.]
    for tick in range(int(np.ceil(times[-1]/.02))+1):
        index = max(0, np.searchsorted(times, tick*.02, side='right')-1)
        plant.step(targets_deg=angles[index], native_servo=True)
        actual.append(np.degrees(plant.data.qpos[plant.q]).copy())
        sim_times.append(float(plant.data.time))
    actual = np.array(actual)
    joints = []
    for j, key in enumerate(keys):
        observed = [r for r in samples if int(r['joint'])==j and int(r['status'])==0
                    and 2 <= float(r['time_ms'])/1000 <= times[-1]]
        t = np.array([float(r['time_ms'])/1000 for r in observed])
        q = np.array([float(r['actual_deg']) for r in observed])
        if j in (0, 3):
            q *= -1
        predicted = np.interp(t, sim_times, actual[:, j])
        joints.append(dict(joint=key, samples=len(t),
            rms_error_deg=float(np.sqrt(np.mean((q-predicted)**2))),
            observed_range_deg=float(np.ptp(q)), predicted_range_deg=float(np.ptp(predicted))))
    return dict(reference='legacy' if legacy else 'braking',
        command_count=len(commands), recorded_target_span_s=[float(times[0]),float(times[-1])],
        compared_sample_window_s=[2.,float(times[-1])],
        mean_joint_rms_error_deg=float(np.mean([r['rms_error_deg'] for r in joints])),
        joints=joints, registers=plant.servo_profile.snapshot())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=ROOT/'artifacts/s-native-v6-2-5/hardware-30s/joint-analysis')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--voltage', type=float, default=10.9)
    args = parser.parse_args()
    results = [replay(args.source, args.voltage, legacy) for legacy in (True, False)]
    report = dict(source=str(args.source), source_sha256={
        name:hashlib.sha256((args.source/name).read_bytes()).hexdigest()
        for name in ('commands.csv','samples.csv')}, voltage_open_circuit_v=args.voltage,
        limitations=['Recorded targets sampled on a 20 ms simulation grid.',
                     'Rigid level body fixture approximates the held-body recording.',
                     'Motor gains, friction, voltage droop and command delay remain estimates.',
                     'No parameter fitting and no new hardware operation.'], results=results)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    print(json.dumps({r['reference']:r['mean_joint_rms_error_deg'] for r in results}))


if __name__=='__main__':
    main()
