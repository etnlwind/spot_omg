import json
from pathlib import Path
import numpy as np
import pytest
from cad_physics import Simulation
from pivot_turn import PivotTurn

@pytest.fixture(scope='module')
def planner():
    p=json.loads(Path(__file__).with_name('measured_response_plant.json').read_text())
    return PivotTurn(Simulation(p).model)

P=[1.05,.52,.07,.022,.20175,-.035,.75]
C={'sweep_rad':.24,'lift_m':.024}

def test_stance_feet_share_zero_translation_rotation(planner):
    phase=.51;eps=1e-6
    before=planner.goals(P,phase-eps,1,-1,C);after=planner.goals(P,phase+eps,1,-1,C)
    points=planner.goals(P,phase,1,-1,C)
    velocity=(after-before)/(2*eps*P[0])
    omega=-C['sweep_rad']/(P[1]*P[0])
    for leg,offset in enumerate((0,.5,.5,0)):
        if (phase+offset)%1<P[1]:
            radius=points[leg,:2]-planner.center[:2]
            np.testing.assert_allclose(velocity[leg,:2],omega*np.array([-radius[1],radius[0]]),atol=1e-7)
            assert abs(velocity[leg,2])<1e-8

def test_cad_ik_preserves_planned_cushion_points(planner):
    for yaw in (-1,1):
        for phase in (.1,.51,.7,.9):
            goal=planner.goals(P,phase,1,yaw,C)
            q=planner.plan(P,phase,1,yaw,C)
            planner.kin.set_angles(q)
            actual=np.array([planner.kin.foot(i) for i in range(4)])
            assert np.max(np.linalg.norm(actual-goal,axis=1))<.00015

def test_pivot_path_continuity_and_limits(planner):
    for phase in (P[1],1):
        a=planner.goals(P,phase-1e-7,1,1,C);b=planner.goals(P,phase+1e-7,1,1,C)
        assert abs(a-b).max()<1e-6
    for invalid in ({'sweep_rad':.8},{'lift_m':.1}):
        with pytest.raises(ValueError):planner.plan(P,.6,1,1,invalid)

def test_center_correction_preserves_foot_heights(planner):
    from measured_swing import targets
    q=targets(P,.8,1,0,-1,{'forward_only':True})
    planner.kin.set_angles(q);before=np.array([planner.kin.foot(i) for i in range(4)])
    config={'mode':'center_correction','sweep_rad':.24,'center_shift_m':[.06,0.]}
    result=planner.plan(P,.8,1,-1,config,nominal=q)
    planner.kin.set_angles(result);after=np.array([planner.kin.foot(i) for i in range(4)])
    np.testing.assert_allclose(after[:,2],before[:,2],atol=1e-4)
    assert np.max(np.linalg.norm(after[:,:2]-before[:,:2],axis=1))<.012
    with pytest.raises(ValueError):planner.plan(P,.8,1,1,{**config,'center_shift_m':[.1,0.]},nominal=q)

def test_entry_transition_preserves_existing_foot_xy(planner):
    from gait_profiles import foot_targets
    entry=np.tile([0.,45.,90.],4)
    planner.set_entry(entry);expected=planner.entry_feet.copy()
    neutral=foot_targets(P,0,0)
    result=planner.plan(P,0,0,0,{'mode':'center_correction','center_shift_m':[0.,0.],'retain_entry_foot_xy':True},nominal=neutral)
    planner.kin.set_angles(result)
    actual=np.array([planner.kin.foot(i) for i in range(4)])
    np.testing.assert_allclose(actual[:,:2],expected[:,:2],atol=1e-4)
