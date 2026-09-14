
# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[3]))

from simulation.mujoco.paths import REPO_ROOT, SIM_ROOT, RESULTS_ROOT
import numpy as np
import pytest
from simulation.mujoco.runtime.gait_profiles import foot_targets
from simulation.mujoco.runtime.measured_swing import targets
P=[1.05,.52,.08,.024,.20175,-.035,.75]
CFG={'rise_fraction':.25,'fall_fraction':.3}

def test_stance_and_zero_command_match_shared_c():
    for linear,yaw in ((1,0),(0,1),(0,-1),(0,0)):
        for phase in np.linspace(0,1,81):
            q=targets(P,phase,1,linear,yaw,CFG);baseline=foot_targets(P,phase*P[0],1,'trot',linear,yaw)
            for i,offset in enumerate((0,.5,.5,0)):
                if (phase+offset)%1<P[1] or not (linear or yaw):
                    np.testing.assert_allclose(q[3*i:3*i+3],baseline[3*i:3*i+3],atol=2e-5)

def test_liftoff_touchdown_and_plateau_continuity():
    for yaw in (-1,0,1):
        for phase in (P[1],P[1]+(1-P[1])*.25,P[1]+(1-P[1])*.7,1):
            before=targets(P,phase-1e-7,1,0 if yaw else 1,yaw,CFG)
            after=targets(P,phase+1e-7,1,0 if yaw else 1,yaw,CFG)
            assert max(abs(after-before))<.001

def test_horizontal_contact_path_unchanged_and_lift_is_broader():
    for u in (.15,.3,.5,.7,.85):
        phase=P[1]+(1-P[1])*u
        a=targets(P,phase,1,1,0,CFG);b=foot_targets(P,phase*P[0],1,'trot',1,0)
        def fk(q):
            h,k=np.radians(q[1:3]);return np.array([-.141*np.sin(h)-.150*np.sin(h-k),.141*np.cos(h)+.150*np.cos(h-k)])
        fa,fb=fk(a),fk(b)
        assert abs(fa[0]-fb[0])<1e-6
        assert fa[1]<=fb[1]+1e-6

def test_invalid_shape_is_rejected():
    with pytest.raises(ValueError):targets(P,.8,1,1,0,{'rise_fraction':.6})

def test_forward_only_turn_uses_original_arch():
    config={**CFG,'forward_only':True}
    for yaw in (-1,1):
        for phase in np.linspace(0,1,101):
            np.testing.assert_allclose(targets(P,phase,1,0,yaw,config),foot_targets(P,phase*P[0],1,'trot',0,yaw),atol=5e-5)

def test_front_turn_lift_preserves_forward_rear_and_stance():
    old={**CFG,'forward_only':True}
    new={**old,'front_turn_extra_lift_m':.02,'front_turn_hold':.7}
    for phase in np.linspace(0,1,101):
        np.testing.assert_array_equal(targets(P,phase,1,1,0,new),targets(P,phase,1,1,0,old))
        for yaw in (-1,1):
            a=targets(P,phase,1,0,yaw,new);b=targets(P,phase,1,0,yaw,old)
            np.testing.assert_array_equal(a[6:],b[6:])
            for i,offset in enumerate((0,.5)):
                if (phase+offset)%1<P[1]:np.testing.assert_array_equal(a[3*i:3*i+3],b[3*i:3*i+3])

def test_front_turn_lift_adds_requested_vertical_clearance():
    old={**CFG,'forward_only':True}
    new={**old,'front_turn_extra_lift_m':.02}
    phase=P[1]+.5*(1-P[1])
    a=targets(P,phase,1,0,-1,new);b=targets(P,phase,1,0,-1,old)
    def fk(q):
        h,k=np.radians(q[1:3]);return np.array([-.141*np.sin(h)-.150*np.sin(h-k),.141*np.cos(h)+.150*np.cos(h-k)])
    delta=fk(a)-fk(b)
    np.testing.assert_allclose(delta,[0,-.02],atol=1e-7)
    for boundary in (P[1],1):
        assert max(abs(targets(P,boundary+1e-7,1,0,1,new)-targets(P,boundary-1e-7,1,0,1,new)))<.001


def test_front_turn_extra_limits():
    for extra in (-.001,.041,float('nan')):
        with pytest.raises(ValueError):targets(P,.8,1,0,1,{'front_turn_extra_lift_m':extra})

def test_diagonal_lift_redistribution_keeps_total_planned_lift():
    old={**CFG,'forward_only':True}
    new={**old,'front_turn_extra_lift_m':.007,'rear_turn_extra_lift_m':-.007}
    phase=P[1]+.5*(1-P[1])
    a=targets(P,phase,1,0,-1,new);b=targets(P,phase,1,0,-1,old)
    def z(q):
        h,k=np.radians(q[1:3]);return .141*np.cos(h)+.150*np.cos(h-k)
    delta=[z(a[i:i+3])-z(b[i:i+3]) for i in (0,9)]
    np.testing.assert_allclose(delta,[-.007,.007],atol=1e-7)
    for extra in (-.019,.021,float('nan')):
        with pytest.raises(ValueError):targets(P,.8,1,0,1,{'rear_turn_extra_lift_m':extra})

def test_cad_front_lift_moves_measured_cushion_bottom_vertically():
    import json
    from pathlib import Path
    from simulation.mujoco.runtime.cad_physics import Simulation
    from simulation.mujoco.runtime.support_shift import SupportShift
    config=json.loads((SIM_ROOT / 'config/measured_response_plant.json').read_text())
    kin=SupportShift(Simulation(config).model)
    old={**CFG,'forward_only':True};new={**old,'cad_turn_lift_m':[.007,.007,0,0]}
    phase=P[1]+.5*(1-P[1])
    a=targets(P,phase,1,0,-1,new);b=targets(P,phase,1,0,-1,old)
    kin.set_angles(b);before=np.array([kin.foot(i) for i in range(4)])
    kin.set_angles(a);after=np.array([kin.foot(i) for i in range(4)])
    np.testing.assert_allclose(after-before,[[0,0,.007],[0,0,0],[0,0,0],[0,0,0]],atol=3e-5)
    np.testing.assert_array_equal(targets(P,phase,1,1,0,new),targets(P,phase,1,1,0,old))

def test_inside_front_turn_lift_selects_low_front_leg_and_rejects_invalid():
    old={**CFG,'forward_only':True}
    new={**old,'cad_turn_inside_extra_m':.004}
    for yaw,phase,leg in [(-1,.8,0),(1,.3,1)]:
        a=targets(P,phase,1,0,yaw,new);b=targets(P,phase,1,0,yaw,old)
        untouched=[j for j in range(12) if j//3!=leg]
        np.testing.assert_array_equal(a[untouched],b[untouched])
        assert np.linalg.norm(a[leg*3:leg*3+3]-b[leg*3:leg*3+3])>.1
    for extra in (-.001,.011,float('nan')):
        with pytest.raises(ValueError):targets(P,.8,1,0,-1,{'cad_turn_inside_extra_m':extra})
