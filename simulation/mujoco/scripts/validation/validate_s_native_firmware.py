"""Compare STM32 V6.1 C against preserved Python CAD gait and physical replay."""
if __package__ in (None, ''):
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[4]))
import argparse
import ctypes as ct
import json
from pathlib import Path
import numpy as np
from simulation.mujoco.runtime.cad_physics import Simulation
from simulation.mujoco.runtime.virtual_robot import RobotController, load_parameters, parse_args
from simulation.mujoco.runtime.s_native_gait import SNativeGait, V61_PROFILE as PROFILE,PROFILES
from servo.host_build import build_shared, library_suffix

ROOT=Path(__file__).resolve().parents[4]

def load_binding(directory):
    firmware=ROOT/'firmware/stm32-learning'
    path=build_shared([firmware/'tests/s_native_binding.c',firmware/'Src/robot_config.c'],
                      firmware/'Inc',directory/('s-native'+library_suffix()))
    lib=ct.CDLL(str(path));lib.reset.argtypes=[];lib.reset.restype=None
    lib.reset_v625.argtypes=[];lib.reset_v625.restype=None
    lib.reset_v624.argtypes=[];lib.reset_v624.restype=None
    lib.reset_v623.argtypes=[];lib.reset_v623.restype=None
    lib.reset_v621.argtypes=[];lib.reset_v621.restype=None
    lib.step.argtypes=[ct.c_float]*7+[ct.c_int,ct.POINTER(ct.c_float)];lib.step.restype=ct.c_int
    lib.encode.argtypes=[ct.POINTER(ct.c_float),ct.POINTER(ct.c_uint16)];lib.encode.restype=ct.c_int
    return lib


def compare(lib,plant,request,stop_after,profile='s_native_v6_1'):
    parameters=PROFILES[profile]
    gait=SNativeGait(plant.model,plant.stand_target,parameters)
    (lib.reset_v625 if profile=='s_native_v6_2_5' else lib.reset_v624 if profile=='s_native_v6_2_4' else lib.reset_v623 if profile=='s_native_v6_2_3' else lib.reset_v621 if profile in ('s_native_v6_2_1','s_native_v6_2_2') else lib.reset)()
    phase=.5;linear=yaw=0.;previous=None;error=0.;stop_tick=round(stop_after/.02)
    output=(ct.c_float*12)()
    for i in range(stop_tick+round(parameters['stop_period_s']/.02)+5):
        stopping=i>=stop_tick
        if i==stop_tick:gait.begin_stop()
        rl,ry=(0.,0.) if stopping else request
        linear+=float(np.clip(rl-linear,-.04,.04));yaw+=float(np.clip(ry-yaw,-.04,.04))
        gait.prepare_support(rl,ry)
        u=min(1,(i+1)*.02);amp=u**3*(10+u*(-15+6*u))
        reference=gait.targets(phase,amp,linear,yaw)
        # A Python phase just below 1 can round to float32 1.0. Firmware
        # wraps its float clock before calling the kernel; mirror that boundary.
        c_phase=ct.c_float(phase).value % 1.
        if not lib.step(c_phase,amp,linear,yaw,rl,ry,.02,int(stopping),output):
            raise AssertionError(f'C IK failed at {request=} {i=} {phase=}')
        actual=np.array(output);error=max(error,float(np.max(abs(actual-reference))))
        if error>.25:raise AssertionError(f'C/Python mismatch {error:.4f} deg at {request=} {i=} {actual=} {reference=}')
        phase=(phase+.02/(parameters['params'][0]*(1.35-.35*min(1,abs(linear)+abs(yaw)))))%1
    assert lib.stopped()
    return dict(request=request,stop_after_s=stop_after,max_target_error_deg=error)


def physical_replay(lib, command=(1000,0), heading=True,profile='s_native_v6_1'):
    """Replay protocol input with tilt protection; None selects Python targets."""
    plant=Simulation(load_parameters(parse_args([])))
    robot=RobotController(plant);robot.select_profile(profile)
    robot.heading.enabled=heading
    if lib is not None:(lib.reset_v625 if profile=='s_native_v6_2_5' else lib.reset_v624 if profile=='s_native_v6_2_4' else lib.reset_v623 if profile=='s_native_v6_2_3' else lib.reset_v621 if profile in ('s_native_v6_2_1','s_native_v6_2_2') else lib.reset)()
    output=(ct.c_float*12)();rows=[]
    joints=json.loads((ROOT/'tools/servo_tool/config/joints.json').read_text())['joints']
    # Independent physical convention: left/front ID1 decreases to adduct;
    # FR ID4 increases (measured), rear ID7 increases and ID10 decreases.
    physical_directions=np.array([1,-1,1,-1,1,-1,-1,-1,1,1,1,-1])
    def targets(phase,amp,linear,yaw):
        gait=robot.s_native_gait
        stopping=gait.stop_progress is not None
        assert lib.step(phase,amp,linear,yaw,*robot.request,.02,int(stopping),output)
        ticks=(ct.c_uint16*12)();assert lib.encode(output,ticks)
        decoded=(np.array(ticks)-[j['center'] for j in joints])*physical_directions*360/4096
        assert np.max(abs(decoded-np.array(output)))<.095
        if stopping and lib.stopped():gait.stop_progress=1.
        gait.previous=np.array(output)
        return gait.previous.copy()
    for i in range(700):
        t=i*.02
        if i==100:
            robot.command(f'drive {command[0]} {command[1]} 1',t)
            if lib is not None:
                robot.s_native_gait.targets=targets
                robot.s_native_gait.prepare_support=lambda *args:None
        elif i==500:robot.command('@S 1000',t)
        elif 100<i<500 and i%10==0:robot.command(f'@D {i} {command[0]} {command[1]}',t)
        robot.tick(t);state=plant.row()
        rotation=plant.data.xmat[plant.model.body('robot').id].reshape(3,3)
        rows.append(dict(time_s=t,roll_deg=state['roll_deg'],pitch_deg=state['pitch_deg'],
                         body_yaw_deg=float(np.degrees(np.arctan2(rotation[1,0],rotation[0,0]))),
                         position_m=plant.data.qpos[:3].tolist(),applied_yaw=robot.yaw,
                         heading=robot.heading.diagnostic(),safety=robot.safety,
                         reply=robot.drain().decode(),target=robot.command_target.tolist()))
    actual_error=float(np.max(abs(np.degrees(plant.data.qpos[plant.q])-robot.stand_target)))
    assert not robot.motion and not robot.transition and robot.safety=='ok'
    assert np.max(abs(robot.command_target-robot.stand_target))<.01 and actual_error<1.1
    yaw=np.degrees(np.unwrap(np.radians([r['body_yaw_deg'] for r in rows])))
    return dict(command=list(command),heading_enabled=heading,backend='c' if lib is not None else 'python',
                walk_yaw_change_deg=float(yaw[499]-yaw[99]),
                final_yaw_change_deg=float(yaw[-1]-yaw[99]),
                walk_displacement_m=(np.array(rows[499]['position_m'])-rows[99]['position_m']).tolist(),
                max_tilt_deg=max(max(abs(r['roll_deg']),abs(r['pitch_deg'])) for r in rows[100:]),
                final_actual_s_error_deg=actual_error,
                stopped_video_s=next(r['time_s'] for r in rows if '$SPOTDRIVE stopped' in r['reply'])),rows


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    lib=load_binding(args.output);plant=Simulation(load_parameters(parse_args([])))
    reports=[]
    for request,stop in [((1.,0.),8.),((1.,0.),.3),((.3,0.),2.),((.6,0.),2.),
                         ((-1.,0.),2.),((0.,.5),2.),((.6,-.3),2.),((.43,.17),2.)]:
        report=compare(lib,plant,request,stop);reports.append(report);print(json.dumps(report),flush=True)
    (args.output/'parity.json').write_text(json.dumps(reports,indent=2),encoding='utf-8')
    report,rows=physical_replay(lib)
    (args.output/'physical-replay.json').write_text(json.dumps(dict(summary=report,rows=rows),indent=2),encoding='utf-8')
    print(json.dumps(report),flush=True)

if __name__=='__main__':main()
