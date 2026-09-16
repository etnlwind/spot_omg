"""Independent early-fold profile: entry, continuous paired recovery, encoding."""
import json
import numpy as np
import pytest
from simulation.mujoco.tests.test_s_native_v623 import plant, kernel
from simulation.mujoco.runtime.s_native_gait import SNativeGait, PROFILES, NAME
from simulation.mujoco.runtime.virtual_robot import RobotController
from simulation.mujoco.scripts.validation.validate_s_native_firmware import compare, ROOT


@pytest.mark.parametrize('command,stop',[
    ((1.,0.),0),((1.,0.),.04),((1.,0.),.3),((1.,0.),1.2),((1.,0.),4),
    ((-.6,0.),2),((0.,.5),2),((0.,-.5),2),((.6,.25),2),((.43,.17),2)])
def test_firmware_matches_python_entry_walk_stop(kernel,plant,command,stop):
    assert compare(kernel,plant,command,stop,profile=NAME)['max_target_error_deg']<.15


def test_keeps_push_then_continuously_recovers_paired_feet(plant):
    new=SNativeGait(plant.model,plant.stand_target,PROFILES[NAME])
    old=SNativeGait(plant.model,plant.stand_target,PROFILES['s_native_v6_2_6'])
    # Entire propulsion stroke retained, not just endpoints.
    for p in np.linspace(0,.5,31):
        np.testing.assert_allclose(new.points(p,1,1,0)[1,0],old.points(p,1,1,0)[1,0],atol=1e-10)
    rows=np.array([new.points(p,1,1,0)-new.origin for p in np.linspace(.5,.999,300)])
    assert np.all(np.diff(rows[:,1,0])>0)
    np.testing.assert_allclose(rows[:,0,[0,2]],rows[:,3,[0,2]],atol=1e-10)
    np.testing.assert_allclose(rows[:,1,[0,2]],rows[:,2,[0,2]],atol=1e-10)
    assert new.points(.60,1,1,0)[1,0]>old.points(.60,1,1,0)[1,0]
    assert rows[:,1,2].max()==pytest.approx(.016)


def test_j1_entry_then_hold_and_no_late_widening(plant):
    gait=SNativeGait(plant.model,plant.stand_target,PROFILES[NAME])
    gait.prepare_support(1,0)
    for p in np.arange(0,2.01,.01):
        q=gait.targets((.5+p)%1,1,1,0)
        if p==.5:
            np.testing.assert_allclose(q[::3]-gait.standing[::3],[0,-18,0,0],atol=.001)
        if p>=1:
            np.testing.assert_allclose(q[::3]-gait.standing[::3],[-9]*4,atol=.001)
    gait.begin_stop()
    for _ in range(85):gait.targets(.5,1,0,0)
    np.testing.assert_allclose(gait.previous,gait.standing,atol=.001)


def test_profile_indices_defaults_and_preservation(plant):
    manifest=json.loads((ROOT/'config/locomotion_profiles.json').read_text(encoding='utf-8'))
    assert ['legacy',*manifest['profiles']][21:]==['s_native_v6_2_5','s_native_v6_2_6',NAME]
    assert manifest['default']==NAME=='s_native_v6_2_7'
    old=PROFILES['s_native_v6_2_6']
    assert old['params']==[1.,.5,.145,.012]
    assert not old.get('steady_j1_hold') and not old.get('recovery_frontload')
    robot=RobotController(plant)
    assert robot.profile==NAME and robot.tracking_enabled
    robot.select_profile('s_native_v6_2_3');assert not robot.tracking_enabled
    robot.select_profile(NAME);assert robot.tracking_enabled
