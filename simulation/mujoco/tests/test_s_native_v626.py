"""Full-stroke recovery, preserved profiles, and unchanged hard tracking guards."""
import json
import numpy as np
import pytest
from simulation.mujoco.tests.test_s_native_v623 import plant, kernel
from simulation.mujoco.runtime.s_native_gait import SNativeGait, PROFILES
NAME = "s_native_v6_2_6"
from simulation.mujoco.runtime.virtual_robot import RobotController
from simulation.mujoco.runtime.gait_tracking import GaitTracking
from simulation.mujoco.scripts.validation.validate_s_native_firmware import compare, ROOT
from simulation.mujoco.scripts.analysis.analyze_s_native_hesitation import trial
from servo import SharedGaitPolicy


@pytest.mark.parametrize('command,stop',[
    ((1.,0.),0),((1.,0.),.04),((1.,0.),.3),((1.,0.),1.2),((1.,0.),4),
    ((-.6,0.),2),((0.,.5),2),((0.,-.5),2),((.6,.25),2),((.43,.17),2)])
def test_c_python_match_including_entry_and_stop(kernel,plant,command,stop):
    assert compare(kernel,plant,command,stop,profile=NAME)['max_target_error_deg']<.15


def test_recovery_moves_through_whole_swing_and_keeps_endpoints(plant):
    gait=SNativeGait(plant.model,plant.stand_target,PROFILES[NAME])
    rows=np.array([gait.points(p,1.,1.,0.)-gait.origin for p in np.linspace(.5,.999,150)])
    assert np.all(np.diff(rows[:,1,0])>0)
    np.testing.assert_allclose(rows[:,0,[0,2]],rows[:,3,[0,2]],atol=1e-10)
    np.testing.assert_allclose(rows[:,1,[0,2]],rows[:,2,[0,2]],atol=1e-10)
    for phase,expected in [(0.,[-.125,.02,.02,-.125]),(.5,[.02,-.125,-.125,.02])]:
        np.testing.assert_allclose((gait.points(phase,1.,1.,0.)-gait.origin)[:,0],expected,atol=1e-10)
    # Fold while moving, then unfold while moving; no stationary X plateau.
    assert rows[20,1,2]>rows[0,1,2] and rows[-20,1,2]>rows[-1,1,2]


def test_early_deceleration_does_not_relax_hard_guards():
    tracking=GaitTracking(SharedGaitPolicy())
    def feed(now,error):
        for j in range(12):tracking.lib.spot_tracking_sample(tracking.state,j,error,now)
    feed(0,10)
    assert tracking.step(0,.02,False,True,6)==pytest.approx(.88)
    feed(20,14)
    assert tracking.step(.02,.02,False,True,6)==0
    for now in range(40,621,20):
        feed(now,14);tracking.step(now/1000,.02,False,True,6)
    assert tracking.diagnostic['fault']==2
    tracking.reset(1);tracking.step(1.62,.02,False,True,6)
    assert tracking.diagnostic['fault']==1
    tracking.reset(2);feed(2000,14)
    assert tracking.step(2,.02,True,True,6)==1


def test_body_supported_recovery_reduces_holds_without_disabling_limits():
    report,_=trial(profile=NAME,voltage=10.9,seconds=16)
    assert report['completed'] and report['first_fault_s'] is None
    assert report['phase_hold_seconds']<.5
    assert report['mean_phase_rate']>.55
    assert report['actual_low_foot_speed_fraction']<.02
    assert report['internal_contact_frames']==0 and report['final_s_error_deg']<1.1


def test_profile_indices_old_recovery_and_tracking_selection(plant):
    manifest=json.loads((ROOT/'config/locomotion_profiles.json').read_text())
    assert ['legacy',*manifest['profiles']][21:23]==['s_native_v6_2_5',NAME]
    assert NAME=='s_native_v6_2_6'
    assert PROFILES['s_native_v6_2_5']['placement_swing']==(.2,.8)
    assert 'uniform_recovery' not in PROFILES['s_native_v6_2_5']
    robot=RobotController(plant)
    robot.select_profile(NAME)
    assert robot.profile==NAME and robot.tracking_enabled
    robot.select_profile('s_native_v6_2_3');assert not robot.tracking_enabled
    robot.select_profile(NAME);assert robot.tracking_enabled
