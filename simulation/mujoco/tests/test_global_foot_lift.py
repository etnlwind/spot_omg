import ctypes as ct
import numpy as np
import pytest
from simulation.mujoco.tests.test_s_native_v623 import plant,kernel
from simulation.mujoco.runtime.s_native_gait import SNativeGait,PROFILES
from simulation.mujoco.runtime.virtual_robot import RobotController

@pytest.mark.parametrize('name,reset',[
    ('s_native_v6_1','reset'),('s_native_v6_2_1','reset_v621'),
    ('s_native_v6_2_3','reset_v623'),('s_native_v6_2_4','reset_v624'),
    ('s_native_v6_2_5','reset_v625'),('s_native_v6_2_6','reset_v626'),('s_native_v6_2_7','reset_v627')])
def test_native_lift_entry_and_stop_match_c(plant,kernel,name,reset):
    getattr(kernel,reset)();kernel.set_foot_lift(10,3,0,7)
    gait=SNativeGait(plant.model,plant.stand_target,PROFILES[name]);gait.foot_lift_mm=[10,3,0,7]
    gait.prepare_support(.6,0)
    out=(ct.c_float*12)()
    for frame in range(150):
        phase=(frame*.01)%1
        expected=gait.targets(phase,1,.6,0)
        assert kernel.step(phase,1,.6,0,.6,0,.02,0,out)
        np.testing.assert_allclose(out,expected,atol=.16)
    gait.begin_stop()
    for _ in range(100):
        expected=gait.targets(phase,1,0,0)
        assert kernel.step(phase,1,0,0,0,0,.02,1,out)
        np.testing.assert_allclose(out,expected,atol=.16)

def test_simulator_atomic_configuration_and_readback(plant):
    robot=RobotController(plant)
    robot.command('footlift set 30 300 0 4',0)
    assert robot.foot_lift_mm==[30,300,0,4]
    robot.command('footlift set 1 2 3 2147483648',0)
    assert robot.foot_lift_mm==[30,300,0,4]
    robot.command('gaitprofile attitudepd_v4',0)
    assert robot.foot_lift_mm==[30,300,0,4]
    robot.motion=('drive',)
    robot.command('footlift set 0 0 0 0',0)
    assert robot.foot_lift_mm==[30,300,0,4]

def test_v4_extra_lift_in_actual_simulator_drive_pipeline(plant):
    robot=RobotController(plant)
    robot.command('footlift set 10 10 0 0',0)
    robot.command('gaitprofile attitudepd_v4',0)
    robot.command('drive -1000 0 1',0)
    for frame in range(120):
        now=frame*.02
        robot.command(f'@D {frame+2} -1000 0',now)
        robot.tick(now)
    assert robot.motion is not None
    assert np.isfinite(robot.command_target).all()
    assert robot.foot_lift_mm==[10,10,0,0]
    robot.command('@S 9999',2.4)
    for frame in range(150):robot.tick(2.4+frame*.02)
    assert robot.motion is None
