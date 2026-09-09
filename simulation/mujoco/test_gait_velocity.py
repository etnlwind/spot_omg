"""Canonical-policy velocity and STS3215 feasibility regression tests."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools/servo_tool"))

from servo.gait_analysis import analyze_gait_velocity
from servo.shared_gait import SharedGaitPolicy
from servo.spot import SpotConfig


LEGS = ("FL", "FR", "RL", "RR")
JOINTS = (1, 2, 3)
CANONICAL_DIGESTS = {
    "trot": "c218ba417eea133f4e87cac38d7f12674d896b03ca2742fec2b9a6b0b9c9af9c",
    "trot2": "7b5a857efbbb0ea389892023d5e9cab7f6e0972c6e0890ffe59fd7684e85b945",
}


def canonical_digest(gait: str) -> str:
    policy = SharedGaitPolicy()
    values: list[float] = []
    for frame in range(40):
        phase = frame / 40.0
        if gait == "trot":
            targets, _ = policy.trot_targets(phase, 1.0, 1.0)
        else:
            targets, _ = policy.trot2_targets(phase, 1.0, 78.0, 108.0)
        values.extend(
            round(targets[(leg, joint)], 5)
            for leg in LEGS
            for joint in JOINTS
        )
    payload = json.dumps(values, separators=(",", ":")).encode()
    return hashlib.sha256(payload).hexdigest()


@pytest.mark.parametrize("gait", ("trot", "trot2"))
def test_canonical_cycle_is_unchanged(gait: str) -> None:
    assert canonical_digest(gait) == CANONICAL_DIGESTS[gait]


def test_default_trot_is_reported_as_physically_infeasible() -> None:
    report = analyze_gait_velocity("trot")
    bottleneck = report.bottleneck
    assert report.status == "infeasible"
    assert (bottleneck.joint, bottleneck.leg) == (3, "FL")
    assert bottleneck.maximum_velocity_deg_s == pytest.approx(585.741, abs=0.01)
    # Guard against silently making an already-infeasible policy still faster.
    assert bottleneck.maximum_velocity_deg_s <= 590.0


def test_default_trot2_is_only_inside_documented_transient_margin() -> None:
    report = analyze_gait_velocity("trot2")
    bottleneck = report.bottleneck
    assert report.status == "transient-margin"
    assert (bottleneck.joint, bottleneck.leg) == (3, "FL")
    assert bottleneck.maximum_velocity_deg_s == pytest.approx(278.227, abs=0.01)
    assert bottleneck.maximum_velocity_deg_s > report.capability.nominal_velocity_deg_s
    assert bottleneck.maximum_velocity_deg_s <= (
        report.capability.transient_velocity_deg_s
    )


def test_trot3_starts_with_four_feet_and_alternates_with_overlap() -> None:
    policy = SharedGaitPolicy()
    _, support_at_start = policy.trot3_targets(0.0, 1.0, 78.0, 108.0)
    _, support_first_swing = policy.trot3_targets(0.20, 1.0, 78.0, 108.0)
    _, support_mid_overlap = policy.trot3_targets(0.50, 1.0, 78.0, 108.0)
    _, support_second_swing = policy.trot3_targets(0.70, 1.0, 78.0, 108.0)
    assert support_at_start == set(LEGS)
    assert support_first_swing == {"FL", "RR"}
    assert support_mid_overlap == set(LEGS)
    assert support_second_swing == {"FR", "RL"}


def test_trot3_preloads_the_scheduled_support_diagonal() -> None:
    policy = SharedGaitPolicy()
    first, _ = policy.trot3_targets(0.25, 1.0, 78.0, 108.0)
    second, _ = policy.trot3_targets(0.75, 1.0, 78.0, 108.0)
    assert first[("FL", 1)] > first[("FR", 1)]
    assert first[("RR", 1)] > first[("RL", 1)]
    assert second[("FR", 1)] > second[("FL", 1)]
    assert second[("RL", 1)] > second[("RR", 1)]


def test_trot4_has_reduced_path_and_four_foot_overlap() -> None:
    policy = SharedGaitPolicy()
    start, support_at_start = policy.trot4_targets(0.0, 1.0)
    swing, support_during_swing = policy.trot4_targets(0.20, 1.0)
    overlap, support_at_overlap = policy.trot4_targets(0.50, 1.0)
    trot3_swing, _ = policy.trot3_targets(0.20, 1.0, 78.0, 100.0)

    assert support_at_start == set(LEGS)
    assert support_during_swing == {"FL", "RR"}
    assert support_at_overlap == set(LEGS)
    assert abs(swing[("FR", 3)] - 90.0) < abs(trot3_swing[("FR", 3)] - 90.0)
    assert overlap[("FL", 1)] != overlap[("FR", 1)]


def test_trot4_zero_amplitude_is_the_calibrated_stand_geometry() -> None:
    policy = SharedGaitPolicy()

    for phase in (0.0, 0.25, 0.5, 0.75, 1.0):
        stopped, _ = policy.trot4_targets(phase, 0.0)
        for leg in LEGS:
            assert stopped[(leg, 1)] == pytest.approx(0.0)
            assert stopped[(leg, 2)] == pytest.approx(45.0)
            assert stopped[(leg, 3)] == pytest.approx(90.0)


def test_trot4_biases_only_fr_j1_outward_by_two_degrees() -> None:
    policy = SharedGaitPolicy()
    targets, _ = policy.trot4_targets(0.25, 1.0)

    # Physical linkage testing established that negative FR J1 is outward.
    # FR and RL otherwise share the same diagonal-transfer target here.
    assert targets[("FR", 1)] == pytest.approx(targets[("RL", 1)] - 2.0)
    assert targets[("FL", 1)] == pytest.approx(targets[("RR", 1)])


def test_backward_trot4_mirrors_fore_aft_path_and_preserves_support() -> None:
    policy = SharedGaitPolicy()
    forward, forward_support = policy.trot4_direction_targets(0.25, 1.0, 1)
    backward, backward_support = policy.trot4_direction_targets(0.25, 1.0, -1)

    assert backward_support == forward_support
    for leg in LEGS:
        forward_x, forward_down = policy_leg_fk(
            forward[(leg, 2)], forward[(leg, 3)])
        backward_x, backward_down = policy_leg_fk(
            backward[(leg, 2)], backward[(leg, 3)])
        assert backward_x == pytest.approx(-forward_x, abs=1.0e-5)
        assert backward_down == pytest.approx(forward_down, abs=1.0e-5)
        assert backward[(leg, 1)] == pytest.approx(forward[(leg, 1)])


def test_backward_trot4_stays_inside_calibrated_servo_limits() -> None:
    policy = SharedGaitPolicy()
    config = SpotConfig.load(ROOT / "tools/servo_tool/config/joints.json")

    for frame in range(101):
        targets, _ = policy.trot4_direction_targets(frame / 100.0, 1.0, -1)
        config.angles_to_targets(targets)


def test_crab_has_overlap_vertical_clearance_and_mirrored_directions() -> None:
    policy = SharedGaitPolicy()
    left, support = policy.crab_targets(0.35, 1.0, 1)
    right, _ = policy.crab_targets(0.35, 1.0, -1)
    stopped, stopped_support = policy.crab_targets(0.35, 0.0, 1)

    assert support == {"FL", "FR", "RL"}
    assert stopped_support == support
    for leg in LEGS:
        assert right[(leg, 1)] == pytest.approx(-left[(leg, 1)])
        assert right[(leg, 2)] == pytest.approx(left[(leg, 2)])
        assert right[(leg, 3)] == pytest.approx(left[(leg, 3)])
        assert stopped[(leg, 1)] == pytest.approx(0.0)
        assert stopped[(leg, 2)] == pytest.approx(45.0)
        assert stopped[(leg, 3)] == pytest.approx(90.0)

    # RR alone is in swing at this phase and must clear every stance foot.
    assert left[("RR", 3)] > left[("FL", 3)]
    assert left[("RR", 3)] > left[("FR", 3)]
    assert left[("RR", 3)] > left[("RL", 3)]


def test_turn_reverses_fore_aft_path_between_robot_sides() -> None:
    policy = SharedGaitPolicy()
    left_turn, support = policy.turn_targets(0.25, 1.0, 1)
    right_turn, right_support = policy.turn_targets(0.25, 1.0, -1)
    stopped, _ = policy.turn_targets(0.25, 0.0, 1)

    assert support == right_support
    for leg in LEGS:
        assert stopped[(leg, 2)] == pytest.approx(45.0)
        assert stopped[(leg, 3)] == pytest.approx(90.0)

    # Reversing turn direction swaps each side's longitudinal foot path.
    for leg in LEGS:
        forward_left, down_left = policy_leg_fk(
            left_turn[(leg, 2)], left_turn[(leg, 3)])
        forward_right, down_right = policy_leg_fk(
            right_turn[(leg, 2)], right_turn[(leg, 3)])
        assert forward_left == pytest.approx(-forward_right, abs=1.0e-5)
        assert down_left == pytest.approx(down_right, abs=1.0e-5)


def test_unit_stride_drive_and_turn_match_the_established_gaits() -> None:
    policy = SharedGaitPolicy()
    for phase in (0.0, 0.17, 0.49, 0.73):
        forward, support = policy.drive_stride_targets(phase, 1.0, 1.0, 0.0, 1.0)
        trot4, trot_support = policy.trot4_direction_targets(phase, 1.0, 1)
        left, left_support = policy.drive_targets(phase, 1.0, 0.0, -1.0)
        turn, turn_support = policy.turn_targets(phase, 1.0, 1)
        assert support == trot_support
        assert left_support == turn_support
        for key in forward:
            assert forward[key] == pytest.approx(trot4[key], abs=1.0e-5)
            assert left[key] == pytest.approx(turn[key], abs=1.0e-5)


def test_continuous_drive_blends_without_changing_support_schedule() -> None:
    policy = SharedGaitPolicy()
    config = SpotConfig.load(ROOT / "tools/servo_tool/config/joints.json")
    for frame in range(101):
        targets, support = policy.drive_targets(
            frame / 100.0, 1.0, 0.55, -0.45)
        base, base_support = policy.trot4_targets(frame / 100.0, 0.0)
        assert support == base_support
        assert support
        config.angles_to_targets(targets)
        assert all(abs(targets[key] - base[key]) < 90.0 for key in targets)


def policy_leg_fk(upper_degrees: float, knee_degrees: float) -> tuple[float, float]:
    import math

    upper = math.radians(upper_degrees)
    lower = math.radians(upper_degrees - knee_degrees)
    return math.sin(upper) + math.sin(lower), math.cos(upper) + math.cos(lower)


def test_trot4_phase_boundaries_have_small_acceleration_jump() -> None:
    policy = SharedGaitPolicy()
    step = 1.0e-3
    before, _ = policy.trot4_targets(0.60 - step, 1.0)
    boundary, _ = policy.trot4_targets(0.60, 1.0)
    after, _ = policy.trot4_targets(0.60 + step, 1.0)
    for joint in (2, 3):
        left_velocity = (boundary[("FL", joint)] - before[("FL", joint)]) / step
        right_velocity = (after[("FL", joint)] - boundary[("FL", joint)]) / step
        assert abs(left_velocity) < 0.1
        assert abs(right_velocity) < 0.1


def test_trot3_velocity_analysis_uses_the_motor_capability() -> None:
    report = analyze_gait_velocity("trot3")
    assert len(report.joints) == 12
    assert report.period_ms == 1400
    assert report.status == "within-nominal"
    assert (report.bottleneck.joint, report.bottleneck.leg) == (3, "FL")
    assert report.bottleneck.maximum_velocity_deg_s == pytest.approx(
        227.42, abs=0.02
    )
    assert report.bottleneck.maximum_velocity_deg_s < (
        report.capability.command_velocity_deg_s
    )


def test_trot4_reduces_the_j3_velocity_demand() -> None:
    trot3 = analyze_gait_velocity("trot3")
    trot4 = analyze_gait_velocity("trot4")
    assert trot4.period_ms == 1600
    assert trot4.status == "within-nominal"
    assert trot4.bottleneck.maximum_velocity_deg_s < (
        trot3.bottleneck.maximum_velocity_deg_s
    )


@pytest.mark.parametrize("gait", ("trot", "trot2", "trot3", "trot4"))
def test_velocity_report_contains_every_leg_and_joint(gait: str) -> None:
    report = analyze_gait_velocity(gait)
    assert {(item.leg, item.joint) for item in report.joints} == {
        (leg, joint) for leg in LEGS for joint in JOINTS
    }
    for item in report.joints:
        assert item.maximum_velocity_deg_s >= 0.0
        assert item.rms_velocity_deg_s >= 0.0
        assert item.maximum_delta_deg >= 0.0
        assert 0.0 <= item.maximum_phase < 1.0
