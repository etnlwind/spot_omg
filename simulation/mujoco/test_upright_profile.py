"""Experimental profile separation and physical clearance regression."""
import json
from pathlib import Path
import numpy as np
import pytest
from gait_profiles import foot_targets
from test_virtual_robot import robot
PROFILE=Path(__file__).with_name('upright_profiles.json')

def test_experiment_preserves_existing_profiles(robot):
    deployed=json.loads(json.dumps(robot.profiles))
    robot.load_experimental_profiles(PROFILE)
    assert all(robot.profiles[n]==p for n,p in deployed.items())
    robot.select_profile('upright')
    assert robot.profiles['upright']['balance_base']=='level'
    with pytest.raises(ValueError):robot.load_experimental_profiles(PROFILE)

def test_upright_is_taller_with_30mm_nominal_swing():
    p=json.loads(PROFILE.read_text())['profiles']['upright']['params']
    q=foot_targets(p,0,0).reshape(4,3)[0]
    assert q[1]<45 and q[2]<90
    down=[]
    for phase in np.linspace(p[1],1,101):
        angles=np.radians(foot_targets(p,phase*p[0]).reshape(4,3)[0])
        down.append(.141*np.cos(angles[1])+.150*np.cos(angles[1]-angles[2]))
    assert max(down)==pytest.approx(.24,abs=1e-6)
    assert min(down)==pytest.approx(.21,abs=1e-6)

def test_upright_actually_lifts_front_feet():
    from diagnose_turn_clearance import run
    profile=json.loads(PROFILE.read_text())['profiles']['upright']
    result,_=run(1000,0,18,profile='upright',override=profile)
    assert result['safety']=='ok'
    for leg in ('FL','FR','RL','RR'):
        values=result['legs'][leg]
        assert values['peak_clearance_mm']>20
        assert values['middle_swing_contact_fraction']<.10
        assert values['peak_tilt_deg']<6


def test_cushion_j2lift_increases_swing_j2_without_moving_stance_targets():
    profiles=json.loads(PROFILE.read_text())['profiles']
    old=profiles['cushion_reach']['params']
    new=profiles['cushion_j2lift']['params']
    before=[];after=[]
    for phase in np.linspace(0,1,101):
        a=foot_targets(old,phase*old[0]).reshape(4,3)
        b=foot_targets(new,phase*new[0]).reshape(4,3)
        if phase < old[1]:
            np.testing.assert_allclose(a[0],b[0],atol=1e-4)
        else:
            before.append(a[0,1]);after.append(b[0,1])
    assert max(after)-max(before)>2.5
