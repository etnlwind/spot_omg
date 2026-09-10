"""Physical invariants of the estimated CAD model, independent of gait success."""
import numpy as np
from cad_physics import Simulation
import mujoco


def test_free_body_falls_under_gravity_with_motors_off():
    s=Simulation();m,d=s.model,s.data
    d.qpos[2]+=.5
    mujoco.mj_forward(m,d)
    before=d.subtree_com[m.body('robot').id,2]
    for _ in range(50):mujoco.mj_step(m,d)
    after=d.subtree_com[m.body('robot').id,2]
    assert abs((before-after)-.5*9.81*.1**2)<.003
    assert m.nv==18 and m.nu==12
    assert np.all(m.body_inertia[m.body_mass>0]>0)


def test_standing_weight_is_supported_by_real_contacts():
    s=Simulation()
    for _ in range(250):s.step(balance=False)
    rows=[]
    for _ in range(50):
        s.step(balance=False);rows.append(s.row())
    weight=s.model.body_mass.sum()*9.81
    assert abs(np.mean([r['normal_force_n'] for r in rows])-weight)<weight*.03
    assert set(rows[-1]['contacts'])=={'fl_foot','fr_foot','rl_foot','rr_foot'}
    assert max(r['max_penetration_m'] for r in rows)<.002
    assert abs(rows[-1]['roll_deg'])<5
    assert abs(rows[-1]['pitch_deg'])<5
    assert np.max(abs(s.data.ctrl)/(s.stall*s.voltage/12))<1.01
    assert s.voltage<s.p['pack_open_circuit_voltage']


def test_forward_policy_advances_free_body_without_pose_overwrites():
    s=Simulation()
    for _ in range(100):s.step(balance=False)
    start=s.data.qpos[:3].copy()
    for i in range(300):s.step(.6,balance=False,startup=i*.02/.7)
    assert s.data.qpos[0]-start[0]>.02
    assert abs(s.row()['roll_deg'])<15
    assert abs(s.row()['pitch_deg'])<15
    assert not any(w.number for w in s.data.warning)


def test_zero_command_delay_applies_new_target_in_first_control_tick():
    import json
    from pathlib import Path
    base=json.loads((Path(__file__).parent/'cad_300mm/physics_parameters.json').read_text())
    changes=[]
    for delay in (0.,.02):
        s=Simulation({**base,'command_delay_s':delay,'embedded_servo_quantization':False})
        initial=s.filtered.copy()
        target=np.degrees(initial);target[2]+=5
        s.step(targets_deg=target,balance=False)
        changes.append(abs(s.filtered[2]-initial[2]))
    assert changes[0]>1e-4
    assert changes[1]<1e-10
