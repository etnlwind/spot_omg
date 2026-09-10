"""Cross-build parity and deployed manifest/servo representation checks."""
import ctypes
import importlib.util
import subprocess
from pathlib import Path
import numpy as np
import pytest
from servo import SharedGaitPolicy
from gait_profiles import foot_targets,load_profiles

ROOT=Path(__file__).resolve().parents[2]


def test_profile_manifest_matches_embedded_generated_constants():
    subprocess.run(['python3',str(ROOT/'tools/generate_locomotion_profiles.py'),'--check'],check=True)


def test_cruise_uses_the_reverse_stride_and_lift_for_forward():
    cruise=load_profiles()['cruise']
    # Measured/user-preferred reverse gait is the reference for both directions.
    assert cruise['params']==cruise['turn_reverse_params']==[1.05,.6,.08,.012,.20175,-.035,.75]
    policy=SharedGaitPolicy();fp=ctypes.POINTER(ctypes.c_float)
    target=policy._library.spot_locomotion_targets
    target.argtypes=(ctypes.c_int,ctypes.c_float,ctypes.c_float,ctypes.c_float,ctypes.c_float,fp)
    target.restype=ctypes.c_int
    from drive_controller import NAMES
    for phase in np.linspace(0,1,101):
        trajectories=[]
        for direction in (-1,1):
            out=(ctypes.c_float*12)()
            assert target(NAMES.index('cruise'),phase,1,direction,0,out)
            a=np.radians(np.array(out).reshape(4,3))
            forward=.141*np.sin(a[:,1])+.150*np.sin(a[:,1]-a[:,2])
            down=.141*np.cos(a[:,1])+.150*np.cos(a[:,1]-a[:,2])
            trajectories.append((forward,down))
        np.testing.assert_allclose(trajectories[0][1],trajectories[1][1],atol=1e-7)
        np.testing.assert_allclose(trajectories[0][0]+trajectories[1][0],.070,atol=1e-7)


def test_turn_posture_slews_and_does_not_activate_on_fast_arc_start():
    policy=SharedGaitPolicy();fp=ctypes.POINTER(ctypes.c_float)
    drive=policy._library.spot_drive_step
    drive.argtypes=(fp,ctypes.c_int,ctypes.c_float,ctypes.c_float,ctypes.c_float,ctypes.c_int,ctypes.c_int,ctypes.c_int,fp)
    drive.restype=ctypes.c_int
    from drive_controller import NAMES
    profile=NAMES.index('jointsport')
    state=(ctypes.c_float*11)();out=(ctypes.c_float*12)()
    # A requested fast arc must not enter the taller posture during speed slew.
    for _ in range(60):
        assert drive(state,profile,.7,.7,0,True,False,False,out)
        assert state[10]==0
    # Enter pivot, then accelerate out. The posture may move at most 2%/frame.
    for linear,yaw in ((0,.5),(1,0),(0,-.5),(0,0)):
        for frame in range(60):
            previous=state[10]
            assert drive(state,profile,linear,yaw,0,True,False,False,out)
            assert abs(state[10]-previous)<=.020001
            assert np.isfinite(list(out)).all()
        assert state[10]==pytest.approx(1 if yaw else 0,abs=1e-6)


def test_unoptimized_firmware_kernel_matches_host_and_tick_commands(tmp_path):
    inc=ROOT/'firmware/stm32-learning/Inc'
    source=tmp_path/'kernel.c'
    source.write_text('''
#include "drive_control.h"
#include "locomotion_servo.h"
#include <stdio.h>
int main(void) {
 for(int profile=0;profile<LOCOMOTION_PROFILE_COUNT;profile++) {
  DriveControl s={0};
  for(int frame=0;frame<500;frame++) {
   float linear=frame<200?.8f:frame<300?.4f:frame<400?-.6f:0;
   float yaw=frame>=100 && frame<300?.4f:0;
   GaitPolicyLegTarget out[4];uint16_t ticks[12];
   if(!drive_control_step(&s,profile,linear,yaw,frame*.013f,true,true,frame>=400,out) || !locomotion_servo_targets(out,ticks)) return 1;
   for(int j=0;j<12;j++)printf("%u ",ticks[j]);puts("");
  }
 }
}
''')
    binary=tmp_path/'kernel'
    subprocess.run(['cc','-std=c11','-O0','-Wall','-Werror','-I',str(inc),str(source),str(inc.parent/'Src/robot_config.c'),'-lm','-o',str(binary)],check=True)
    expected=np.array([list(map(int,line.split())) for line in subprocess.check_output([str(binary)],text=True).splitlines()])
    policy=SharedGaitPolicy();fp=ctypes.POINTER(ctypes.c_float)
    drive=policy._library.spot_drive_step
    drive.argtypes=(fp,ctypes.c_int,ctypes.c_float,ctypes.c_float,ctypes.c_float,ctypes.c_int,ctypes.c_int,ctypes.c_int,fp)
    drive.restype=ctypes.c_int
    encode=policy._library.spot_servo_encode
    encode.argtypes=(fp,ctypes.POINTER(ctypes.c_uint16),fp);encode.restype=ctypes.c_int
    rows=[]
    for profile in range(1+len(load_profiles())):
        state=(ctypes.c_float*11)()
        for frame in range(500):
            linear=.8 if frame<200 else .4 if frame<300 else -.6 if frame<400 else 0
            yaw=.4 if 100<=frame<300 else 0
            out=(ctypes.c_float*12)();ticks=(ctypes.c_uint16*12)();decoded=(ctypes.c_float*12)()
            assert drive(state,profile,linear,yaw,ctypes.c_float(frame*ctypes.c_float(.013).value).value,True,True,frame>=400,out)
            assert encode(out,ticks,decoded)
            rows.append(list(ticks))
    # O0 vs O2 float math must not change the transmitted hardware commands.
    np.testing.assert_array_equal(rows,expected)


@pytest.mark.parametrize('profile',list(load_profiles()))
def test_all_profile_targets_fit_physical_servo_calibration(profile):
    policy=SharedGaitPolicy();fn=policy._library.spot_servo_encode;fp=ctypes.POINTER(ctypes.c_float)
    fn.argtypes=(fp,ctypes.POINTER(ctypes.c_uint16),fp);fn.restype=ctypes.c_int
    from drive_controller import NAMES
    target=policy._library.spot_locomotion_targets
    target.argtypes=(ctypes.c_int,ctypes.c_float,ctypes.c_float,ctypes.c_float,ctypes.c_float,fp)
    target.restype=ctypes.c_int
    for linear,yaw in ((1,0),(0,.5),(0,-.5),(.3,.5),(-.6,-.5)):
        for phase in np.linspace(0,1,51):
            values=(ctypes.c_float*12)()
            assert target(NAMES.index(profile),phase,1,linear,yaw,values)
            angles=np.array(values)
            ticks=(ctypes.c_uint16*12)();decoded=(ctypes.c_float*12)()
            assert fn((ctypes.c_float*12)(*angles),ticks,decoded)
            assert max(abs(angles-np.array(decoded)))<.10
