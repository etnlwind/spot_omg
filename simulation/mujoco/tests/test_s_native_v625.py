"""Preserve geometry and hard fault limits while reducing recovery delay."""
import json
import numpy as np
import pytest
from servo import SharedGaitPolicy
from simulation.mujoco.tests.test_s_native_v623 import plant,kernel
from simulation.mujoco.runtime.s_native_gait import SNativeGait,PROFILES,NAME
from simulation.mujoco.runtime.gait_tracking import GaitTracking
from simulation.mujoco.runtime.virtual_robot import RobotController
from simulation.mujoco.scripts.validation.validate_s_native_firmware import compare,ROOT

@pytest.mark.parametrize('command,stop',[
    ((1.,0.),0),((1.,0.),.04),((1.,0.),.3),((1.,0.),1.2),((1.,0.),4),
    ((-.6,0.),2),((0.,.5),2),((0.,-.5),2),((.6,.25),2),((.43,.17),2)])
def test_c_python_entry_swing_and_stop(kernel,plant,command,stop):
    assert compare(kernel,plant,command,stop,profile=NAME)['max_target_error_deg']<.15

def test_lift_precedes_horizontal_recovery_and_pair_geometry_is_shared(plant):
    gait=SNativeGait(plant.model,plant.stand_target,PROFILES[NAME])
    for phase in np.linspace(0,1,101):
        d=gait.points(phase,1.,1.,0.)-gait.origin
        np.testing.assert_allclose(d[0,[0,2]],d[3,[0,2]],atol=1e-10)
        np.testing.assert_allclose(d[1,[0,2]],d[2,[0,2]],atol=1e-10)
    d=gait.points(.6,1.,1.,0.)-gait.origin
    assert abs(d[1,0]+.125)<1e-8 and d[1,2]>.009
    for phase,expected in [(0.,[-.125,.02,.02,-.125]),(.5,[.02,-.125,-.125,.02])]:
        np.testing.assert_allclose((gait.points(phase,1.,1.,0.)-gait.origin)[:,0],expected,atol=1e-10)

def feed(t,now,error):
    for i in range(12):t.lib.spot_tracking_sample(t.state,i,error,now)

def test_faster_recovery_retains_stop_error_and_stale_faults():
    old,new=[GaitTracking(SharedGaitPolicy()) for _ in range(2)]
    for t in [old,new]:feed(t,0,14);assert t.step(0,.02,False)==0
    for now in range(20,521,20):
        for t,responsive in [(old,False),(new,True)]:
            feed(t,now,2);t.step(now/1000,.02,False,responsive=responsive)
    assert old.rate<.11 and new.rate>.99
    for now in range(540,1181,20):
        feed(new,now,14);assert new.step(now/1000,.02,False,True)==0
    assert new.diagnostic['fault']==2
    new.reset(2);new.step(2.62,.02,False,True);assert new.diagnostic['fault']==1
    new.reset(3);feed(new,3000,14);assert new.step(3,.02,True,True)==1

def test_default_and_old_profile_selection_preserve_capability(plant):
    robot=RobotController(plant);assert robot.profile==NAME=='s_native_v6_2_5'
    assert robot.profiles[NAME]['tracking_samples_per_frame']==2
    robot.select_profile('s_native_v6_2_4');assert robot.tracking_enabled
    assert robot.profiles[robot.profile].get('tracking_samples_per_frame',1)==1
    assert 'placement_swing' not in PROFILES['s_native_v6_2_4']
    robot.select_profile('s_native_v6_2_3');assert not robot.tracking_enabled
    robot.select_profile(NAME);assert robot.tracking_enabled
    manifest=json.loads((ROOT/'config/locomotion_profiles.json').read_text())
    assert list(manifest['profiles'])[-2:]==['s_native_v6_2_4',NAME]
