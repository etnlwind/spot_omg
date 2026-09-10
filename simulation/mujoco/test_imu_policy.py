"""Invariants of the phase-aware policy, independent of MuJoCo truth state."""
import ctypes
import math
import numpy as np
import pytest
from servo import SharedGaitPolicy
from gait_profiles import foot_targets,load_profiles
from balance_controller import BalanceController
from bno055_emulator import FirmwareAttitudeFilter


def test_phase_keeps_j1_sign_consistent_and_off_clears_correction():
    policy=SharedGaitPolicy();p=load_profiles()['imu']['params']
    attitude=FirmwareAttitudeFilter();attitude.filtered=[40,30];attitude.rate=[0,0]
    responses=[]
    for phase in (.2,.75):
        c=BalanceController(policy);c.profile='imu';c.moving=True;c.phase=phase;c.linear=.8
        c.kp=.1;c.kd=0;c.ki=.03
        nominal=foot_targets(p,phase*p[0],linear=.8)
        for _ in range(12):out=c.apply(nominal,attitude,True,True)
        responses.append(c.correction[::3].copy())
        assert c.applied and np.all(np.isfinite(out))
        assert max(abs(c.correction[::3]))<=1.5
        c.enabled=False
        for _ in range(40):out=c.apply(nominal,attitude,True,True)
        np.testing.assert_allclose(out,nominal,atol=1e-5)
        assert not c.applied and not any(c.integral)
    assert responses[0][0]>0 and responses[1][0]>0
    assert abs(responses[0][0]-responses[1][0])<.01


def test_unavailable_imu_does_not_generate_feedback():
    p=load_profiles()['imu']['params'];c=BalanceController(SharedGaitPolicy());c.profile='imu';c.moving=True
    attitude=FirmwareAttitudeFilter();attitude.filtered=[100,100];attitude.rate=[1200,-1200]
    nominal=foot_targets(p,.4)
    np.testing.assert_allclose(c.apply(nominal,attitude,False,True),nominal,atol=1e-5)
    assert not c.applied and not any(c.integral)


def test_active_policy_o0_vs_host_servo_commands(tmp_path):
    import subprocess
    from pathlib import Path
    root=Path(__file__).resolve().parents[2];inc=root/'firmware/stm32-learning/Inc'
    source=tmp_path/'imu.c'
    source.write_text('''
#include "balance_control.h"
#include "locomotion_servo.h"
#include <stdio.h>
int main(void) {
 BalanceControl s={0};
 for(int frame=0;frame<400;frame++) {
  GaitPolicyLegTarget out[4];uint16_t ticks[12];
  float phase=(frame%100)*.01f;
  GaitPolicyImuSample imu={.04f,-.03f,.12f,-.08f};
  int p=locomotion_profile_id("imu");
  if(!locomotion_targets(p,phase,1,.8f,.2f,out) ||
     !balance_control_apply_policy(&s,out,&imu,frame<350,false,.1f,0,.03f,p,phase,true,.8f,.2f) ||
     !locomotion_servo_targets(out,ticks))return 1;
  for(int j=0;j<12;j++)printf("%u ",ticks[j]);puts("");
 }
}
''')
    binary=tmp_path/'imu'
    subprocess.run(['cc','-std=c11','-O0','-Wall','-Werror','-I',str(inc),str(source),str(inc.parent/'Src/robot_config.c'),'-lm','-o',str(binary)],check=True)
    expected=np.array([list(map(int,line.split())) for line in subprocess.check_output([str(binary)],text=True).splitlines()])
    lib=SharedGaitPolicy()._library;fp=ctypes.POINTER(ctypes.c_float)
    target=lib.spot_locomotion_targets;target.argtypes=(ctypes.c_int,ctypes.c_float,ctypes.c_float,ctypes.c_float,ctypes.c_float,fp)
    balance=lib.spot_balance_control_policy;balance.argtypes=(fp,fp,fp,ctypes.c_int,ctypes.c_int,ctypes.c_float,ctypes.c_float,ctypes.c_float,ctypes.c_int,ctypes.c_float,ctypes.c_int,ctypes.c_float,ctypes.c_float)
    encode=lib.spot_servo_encode;encode.argtypes=(fp,ctypes.POINTER(ctypes.c_uint16),fp)
    state=(ctypes.c_float*16)();imu=(ctypes.c_float*4)(.04,-.03,.12,-.08);rows=[]
    from drive_controller import NAMES
    p=NAMES.index('imu')
    for frame in range(400):
        phase=ctypes.c_float((frame%100)*ctypes.c_float(.01).value).value
        out=(ctypes.c_float*12)();ticks=(ctypes.c_uint16*12)();decoded=(ctypes.c_float*12)()
        assert target(p,phase,1,.8,.2,out)
        assert balance(state,out,imu,frame<350,False,.1,0,.03,p,phase,True,.8,.2)
        assert encode(out,ticks,decoded)
        rows.append(list(ticks))
    np.testing.assert_array_equal(rows,expected)


@pytest.mark.parametrize("profile", ["level", "level15", "joint", "jointfast", "jointsport"])
def test_level_j1_feedback_is_above_servo_resolution(profile):
    from cad_gait import KEYS
    policy=SharedGaitPolicy();p=load_profiles()[profile]['params']
    attitude=FirmwareAttitudeFilter();attitude.filtered=[5,0]
    c=BalanceController(policy);c.profile=profile;c.moving=True;c.phase=.2;c.linear=1
    nominal=foot_targets(p,.2*p[0],linear=1)
    for _ in range(5):out=c.apply(nominal,attitude,True,True)
    assert .4 < max(abs(c.correction[::3])) < .6
    fn=policy._library.spot_servo_encode;fp=ctypes.POINTER(ctypes.c_float)
    fn.argtypes=(fp,ctypes.POINTER(ctypes.c_uint16),fp)
    before=(ctypes.c_uint16*12)();after=(ctypes.c_uint16*12)();decoded=(ctypes.c_float*12)()
    assert fn((ctypes.c_float*12)(*nominal),before,decoded)
    assert fn((ctypes.c_float*12)(*out),after,decoded)
    assert all(abs(int(after[i])-int(before[i]))>=4 for i in (0,3,6,9))
