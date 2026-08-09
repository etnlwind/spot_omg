"""Dynamic and kinematic checks for the shared roll-balance controller."""

from __future__ import annotations

import math
from pathlib import Path
import sys

import mujoco
import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools/servo_tool"))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from servo import ImuSample, SharedGaitPolicy, SpotConfig  # noqa: E402
from walk import (  # noqa: E402
    CONFIG,
    LEGS,
    SCENE,
    SIM_PRESETS,
    canonical_targets_to_positions,
    initialize_on_ground,
    parse_args,
    read_imu,
    resolve_gait,
    set_controls,
    stance_targets,
)


def test_trot3_simulator_uses_the_same_rebalanced_full_mode_defaults(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(sys, "argv", ["walk.py", "--preset", "trot3"])
    args = parse_args()
    resolve_gait(args)
    assert args.balance_kp == pytest.approx(1.0)
    assert args.balance_kd == pytest.approx(0.04)
    assert args.balance_limit == pytest.approx(0.08)
    assert args.j1_balance_gain == pytest.approx(15.0)


def _balanced_targets(
    roll_deg: float,
    *,
    phase: float = 0.0,
    amplitude: float = 0.0,
    support_legs: set[str] | None = None,
):
    policy = SharedGaitPolicy()
    nominal, scheduled = policy.trot3_targets(
        phase, amplitude, 78.0, 108.0
    )
    corrected = policy.balance_targets(
        nominal,
        sample=ImuSample(math.radians(roll_deg), 0.0, 0.0, 0.0),
        support_legs=support_legs or scheduled,
        kp=1.0,
        kd=0.04,
        leg_length_limit=0.15,
        mode="contact-aware",
        j1_gain=5.0,
        j1_limit=5.0,
        foot_placement_gain=0.0,
        foot_placement_limit=0.08,
    )
    return nominal, corrected


def test_roll_allocation_explains_small_j1_and_large_knee_angle() -> None:
    nominal, corrected = _balanced_targets(
        8.3,
        phase=0.65,
        amplitude=1.0,
        support_legs={"FR", "RL"},
    )
    j1 = max(
        abs(corrected[(leg, 1)] - nominal[(leg, 1)]) for leg in LEGS
    )
    knee = max(
        abs(corrected[(leg, 3)] - nominal[(leg, 3)]) for leg in LEGS
    )

    # 8.3deg = 0.145rad. J1 is explicitly 5deg/rad * 0.145rad, while the
    # same normalized leg-length correction passes through nonlinear IK.
    assert j1 == pytest.approx(0.724, abs=0.01)
    assert knee == pytest.approx(13.82, abs=0.05)


def test_rebalanced_full_mode_moves_authority_from_knee_to_j1() -> None:
    policy = SharedGaitPolicy()
    nominal, _ = policy.trot3_targets(0.65, 1.0, 78.0, 108.0)
    corrected = policy.balance_targets(
        nominal,
        sample=ImuSample(math.radians(8.3), 0.0, 0.0, 0.0),
        support_legs={"FR", "RL"},
        kp=1.0,
        kd=0.04,
        leg_length_limit=0.08,
        mode="contact-aware",
        j1_gain=15.0,
        j1_limit=5.0,
        foot_placement_gain=0.0,
        foot_placement_limit=0.08,
    )
    j1 = max(
        abs(corrected[(leg, 1)] - nominal[(leg, 1)]) for leg in LEGS
    )
    knee = max(
        abs(corrected[(leg, 3)] - nominal[(leg, 3)]) for leg in LEGS
    )
    assert j1 == pytest.approx(2.173, abs=0.01)
    assert knee == pytest.approx(7.257, abs=0.05)


def _roll_after_j1_impulse(direction: int) -> float:
    config = SpotConfig.load(CONFIG)
    base = stance_targets(config, SIM_PRESETS["trot3"])
    nominal, corrected = _balanced_targets(5.0)

    j1_only = dict(nominal)
    for leg in LEGS:
        correction = corrected[(leg, 1)] - nominal[(leg, 1)]
        j1_only[(leg, 1)] += direction * correction
    target = canonical_targets_to_positions(config, j1_only)

    model = mujoco.MjModel.from_xml_path(str(SCENE))
    data = mujoco.MjData(model)
    initialize_on_ground(model, data, config, base)
    for _ in range(round(1.0 / model.opt.timestep)):
        set_controls(model, data, config, base)
        mujoco.mj_step(model, data)

    root = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "root")
    qpos = model.jnt_qposadr[root]
    half_roll = math.radians(5.0) / 2.0
    data.qpos[qpos + 3 : qpos + 7] = (
        math.cos(half_roll),
        math.sin(half_roll),
        0.0,
        0.0,
    )
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)
    for _ in range(round(0.010 / model.opt.timestep)):
        set_controls(model, data, config, target)
        mujoco.mj_step(model, data)
    return math.degrees(read_imu(model, data).roll)


def test_current_j1_side_sign_produces_restoring_torque() -> None:
    unchanged = _roll_after_j1_impulse(0)
    current_sign = _roll_after_j1_impulse(1)
    reversed_sign = _roll_after_j1_impulse(-1)

    # With an imposed +5deg body roll, the current sign reduces positive roll
    # faster than no J1 correction. Reversing it does the opposite. This test
    # exercises URDF axes, calibration directions, contacts and actuators.
    assert current_sign < unchanged < reversed_sign
