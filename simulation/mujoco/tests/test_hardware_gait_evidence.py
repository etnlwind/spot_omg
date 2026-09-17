import numpy as np
from simulation.mujoco.runtime.gait_evidence import swing_quality
from simulation.mujoco.scripts.visualization.replay_joint_trace import decode_targets

def test_empty_or_dragging_evidence_cannot_pass():
    assert not swing_quality([])['passed']
    rows=[dict(phase=p,clearance_mm=[10]*4,loads_n=[0]*4) for p in (.25,.75)]
    assert swing_quality(rows)['passed']
    rows[1]['loads_n'][2]=2
    assert not swing_quality(rows)['passed']
    rows[1]['loads_n'][2]=0
    rows[1]['clearance_mm'][2]=1
    assert not swing_quality(rows)['passed']


def test_leg_phase_override_audits_actual_swing_window():
    rows=[dict(phase=.1,leg_phase=[.75]*4,clearance_mm=[10]*4,loads_n=[0]*4)]
    assert swing_quality(rows)['passed']
    rows[0]['loads_n'][1]=1
    assert not swing_quality(rows)['passed']

def test_native_trace_front_j1_decoded_once_and_j2_unwrapped():
    mapping={j:dict(center=2048,direction=1) for j in range(12)}
    commands=[dict(target=[2150]*12)]
    native,is_native=decode_targets({'revision':'s-native-v6-2-7-v77'},mapping,commands)
    legacy,_=decode_targets({'revision':'legacy'},mapping,commands)
    assert is_native
    np.testing.assert_allclose(native[0,[0,3]],-legacy[0,[0,3]])
    np.testing.assert_allclose(native[0,[1,2,4,5,6,7,8,9,10,11]],legacy[0,[1,2,4,5,6,7,8,9,10,11]])
    commands[0]['target'][1]=0x8000|937
    decoded,_=decode_targets({'revision':'s-native-test'},mapping,commands)
    assert decoded[0,1]==(-937-2048)*360/4096
