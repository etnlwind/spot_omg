"""Interactive body-supported preview using the deployed gait and servo physics.

Run with mjpython on macOS. W starts/repeats, Space stops. No hardware transport.
"""
import argparse
import collections
import json
import sys
import time
from pathlib import Path
from xml.etree import ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
import mujoco
import mujoco.viewer
import numpy as np
from simulation.mujoco.runtime.cad_physics import Simulation, build
from simulation.mujoco.runtime.virtual_robot import RobotController, load_parameters, parse_args


def supported_plant(voltage=10.9):
    parameters = load_parameters(parse_args([]))
    parameters['pack_open_circuit_voltage'] = voltage
    xml, parameters = build(parameters, write_scene=False)
    root = ET.fromstring(xml)
    body = root.find(".//body[@name='robot']")
    position = list(map(float, body.get('pos', '0 0 0').split()))
    position[2] += .3
    body.set('pos', ' '.join(map(str, position)))
    equality = root.find('equality')
    if equality is None:
        equality = ET.SubElement(root, 'equality')
    ET.SubElement(equality, 'weld', name='preview_body_fixture', body1='robot', solref='.002 1')
    model = mujoco.MjModel.from_xml_string(ET.tostring(root, encoding='unicode'))
    plant = Simulation(parameters, model)
    # Simulation's normal constructor settles free feet at floor height. Restore
    # the fixture's reference base position; leave all joint physics untouched.
    plant.data.qpos[:7] = model.qpos0[:7]
    mujoco.mj_forward(model, plant.data)
    return plant


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    from simulation.mujoco.runtime.s_native_gait import NAME
    parser.add_argument('--profile', default=NAME)
    parser.add_argument('--voltage', type=float, default=10.9)
    parser.add_argument('--seconds', type=float, default=8)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not 10 <= args.voltage <= 12.6 or not 2 <= args.seconds <= 60:
        parser.error('voltage 10..12.6V and seconds 2..60 required')
    args.output.mkdir(parents=True, exist_ok=False)
    plant = supported_plant(args.voltage)
    robot = RobotController(plant)
    robot.select_profile(args.profile)
    keys = collections.deque()
    repeating = False
    next_start = 2.
    run_started = None
    last_packet = -1.
    sequence = 1
    frame = 0
    next_frame = time.monotonic()
    rate = 1.
    interval_start = time.monotonic()
    interval_sim = 0.
    (args.output/'setup.json').write_text(json.dumps(dict(profile=args.profile,
        input_per_mille=1000, support='body weld +0.3m, feet airborne',
        physics='estimated; collisions, inertia, servo caps and tracking guards retained',
        voltage_open_circuit_v=args.voltage, camera=dict(azimuth=90,elevation=0,distance=1.05)),indent=2)+'\n')
    print('READY: W = repeat 100% / Space = Stop / mouse = camera. No hardware connection.',flush=True)
    with (args.output/'trace.jsonl').open('w',buffering=1) as trace, mujoco.viewer.launch_passive(plant.model,plant.data,key_callback=keys.append) as viewer:
        viewer.cam.azimuth=90; viewer.cam.elevation=0; viewer.cam.distance=1.05
        viewer.cam.lookat[:]=plant.data.subtree_com[plant.model.body('robot').id]-[0,0,.1]
        while viewer.is_running():
            began = time.monotonic()
            t = float(plant.data.time)
            while keys:
                key=keys.popleft()
                if key==87 and robot.safety=='ok':
                    repeating=True;next_start=t
                elif key==32:
                    repeating=False;robot.stop('keyboard-stop');run_started=None
            if repeating and not robot.motion and not robot.transition and t>=next_start:
                robot.command(f'drive 1000 0 {sequence}',t);sequence+=1
                run_started=t;last_packet=t
                print('PLAY: '+args.profile+' / 100% / body supported',flush=True)
            if run_started is not None:
                if t-run_started>=args.seconds:
                    robot.command(f'@S {sequence}',t);sequence+=1
                    run_started=None;next_start=t+6
                elif t-last_packet>=.18:
                    robot.command(f'@D {sequence} 1000 0',t);sequence+=1;last_packet=t
            robot.tick(t)
            reply=robot.drain().decode()
            if reply:print(reply.strip(),flush=True)
            if robot.safety!='ok':repeating=False;run_started=None
            state=plant.row()
            if frame%5==0 and frame<6000:
                trace.write(json.dumps(dict(**state,profile=robot.profile,safety=robot.safety,
                    motion=robot.motion[0] if robot.motion else None,command_target=robot.command_target.tolist()))+'\n')
            if began-interval_start>=1:
                rate=(float(plant.data.time)-interval_sim)/(began-interval_start)
                interval_start=began;interval_sim=float(plant.data.time)
            viewer.cam.lookat[:]=plant.data.subtree_com[plant.model.body('robot').id]-[0,0,.1]
            viewer.opt.geomgroup[3]=0
            viewer.set_texts((mujoco.mjtFontScale.mjFONTSCALE_150,mujoco.mjtGridPos.mjGRID_TOPLEFT,
                'Model\nCondition\nMotion\nServo supply\nTracking / safety\nPlayback\nControls',
                f'{robot.profile.upper()} / 100% input\nBODY SUPPORTED / feet airborne / estimated physics\n'
                f'{robot.motion[0] if robot.motion else "S - ready"}\n{plant.voltage:.2f} V simulated / {args.voltage:.1f} V open circuit\n'
                f'{state["max_tracking_error_deg"]:.1f} deg / {robot.safety}\n{rate:.2f}x real time / {"repeat" if repeating else "stopped"}\nW: play / Space: stop / mouse: camera'))
            # Drawing at 25Hz leaves the physics controller its full 50Hz budget.
            if frame%2==0: viewer.sync()
            frame+=1;next_frame+=.02
            if time.monotonic()-next_frame>.5:next_frame=time.monotonic()
            time.sleep(max(0,next_frame-time.monotonic()))


if __name__=='__main__':
    main()
