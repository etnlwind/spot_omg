import ctypes as ct
import numpy as np
import pytest
from simulation.mujoco.tests.test_s_native_v623 import plant,kernel
from simulation.mujoco.runtime.s_native_gait import SNativeGait,PROFILES
from simulation.mujoco.runtime.virtual_robot import RobotController

def test_motor_width_projects_to_correct_physical_cad_feet(plant):
    """Evaluate actual motor convention, independent of the IK's own FK.

    Raw legacy encoder output is motor angles, not CAD. Front J1 must be
    negated before placing those commands in MuJoCo's CAD geometry.
    """
    from simulation.mujoco.runtime.support_shift import SupportShift
    kin=SupportShift(plant.model);lib=plant.policy._library
    fp=ct.POINTER(ct.c_float)
    width=lib.spot_foot_width
    width.argtypes=[ct.POINTER(ct.c_int32),ct.c_float,fp];width.restype=ct.c_int
    encode=lib.spot_servo_encode
    encode.argtypes=[fp,ct.POINTER(ct.c_uint16),fp];encode.restype=ct.c_int
    def physical_feet(q):
        decoded=(ct.c_float*12)();ticks=(ct.c_uint16*12)()
        assert encode((ct.c_float*12)(*q),ticks,decoded)
        cad=np.array(decoded);cad[[0,3]]*=-1
        kin.set_angles(cad)
        return np.array([kin.foot(i) for i in range(4)])
    for j1 in (-9,0,9):
        base=plant.stand_target.copy();base[::3]=j1
        before=physical_feet(base)
        for mm in (-5,5):
            out=(ct.c_float*12)(*base)
            assert width((ct.c_int32*4)(mm,mm,mm,mm),1,out)
            delta=physical_feet(out)-before
            np.testing.assert_allclose(delta[:,1]*[1,-1,1,-1],mm*.001,atol=.0006)
            np.testing.assert_allclose(delta[:,[0,2]],0,atol=.0006)

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
    robot.command('footlift save 30 300 0 4',0)
    assert robot.foot_lift_mm==[30,300,0,4]
    robot.command('footlift save 1 2 3 2147483648',0)
    assert robot.foot_lift_mm==[30,300,0,4]
    robot.command('gaitprofile attitudepd_v4',0)
    assert robot.foot_lift_mm==[30,300,0,4]
    robot.motion=('drive',)
    robot.command('footlift save 0 0 0 0',0)
    assert robot.foot_lift_mm==[30,300,0,4]

def test_v4_extra_lift_in_actual_simulator_drive_pipeline(plant):
    robot=RobotController(plant)
    robot.command('footlift save 10 10 0 0',0)
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


def test_settings_survive_controller_restart_and_failed_write(plant, tmp_path, monkeypatch):
    path=tmp_path/'robot.json'
    first=RobotController(plant, path)
    first.command('footlift save 20 30 4 5',0)
    second=RobotController(plant, path)
    assert second.foot_lift_mm == [20,30,4,5]
    second.command('footlift set 0 0 0 0',0) # Legacy apps cannot overwrite it.
    assert second.foot_lift_mm == [20,30,4,5]
    def fail(*args): raise OSError('write failure')
    monkeypatch.setattr('simulation.mujoco.runtime.foot_lift_store.os.replace',fail)
    second.command('footlift save 7 8 9 10',0)
    assert second.foot_lift_mm == [20,30,4,5]
    assert RobotController(plant,path).foot_lift_mm == [20,30,4,5]


@pytest.mark.parametrize('name,reset',[
    ('s_native_v6_1','reset'),('s_native_v6_2_1','reset_v621'),
    ('s_native_v6_2_3','reset_v623'),('s_native_v6_2_4','reset_v624'),
    ('s_native_v6_2_5','reset_v625'),('s_native_v6_2_6','reset_v626'),('s_native_v6_2_7','reset_v627')])
def test_native_width_entry_and_stop_match_c(plant,kernel,name,reset):
    getattr(kernel,reset)();kernel.set_foot_width(-3,4,-2,3)
    gait=SNativeGait(plant.model,plant.stand_target,PROFILES[name]);gait.foot_width_mm=[-3,4,-2,3]
    gait.prepare_support(.6,0);out=(ct.c_float*12)()
    for frame in range(100):
        phase=(frame*.01)%1
        expected=gait.targets(phase,1,.6,0)
        assert kernel.step(phase,1,.6,0,.6,0,.02,0,out)
        np.testing.assert_allclose(out,expected,atol=.16)
    gait.begin_stop()
    for _ in range(100):
        expected=gait.targets(phase,1,0,0)
        assert kernel.step(phase,1,0,0,0,0,.02,1,out)
        np.testing.assert_allclose(out,expected,atol=.16)
    np.testing.assert_allclose(expected,plant.stand_target,atol=.16)


def test_width_persistence_old_clients_and_failed_save(plant,tmp_path,monkeypatch):
    path=tmp_path/'settings.json'
    robot=RobotController(plant,path)
    robot.command('footlift save 1 2 3 4 -5 6 -7 8',0)
    reboot=RobotController(plant,path)
    assert reboot.foot_width_mm==[-5,6,-7,8]
    reboot.command('footlift save 2 3 4 5',0)
    assert RobotController(plant,path).foot_width_mm==[-5,6,-7,8]
    def fail(*args):raise OSError('disk failure')
    monkeypatch.setattr('simulation.mujoco.runtime.foot_lift_store.os.replace',fail)
    reboot.command('footlift save 9 9 9 9 0 0 0 0',0)
    assert reboot.foot_width_mm==[-5,6,-7,8] and reboot.foot_lift_mm==[2,3,4,5]
