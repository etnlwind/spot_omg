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


@pytest.mark.parametrize("gait", ("trot", "trot2", "trot3"))
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
