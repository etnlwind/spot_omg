"""Motor-feasibility analysis for the shared canonical gait policy.

This module intentionally observes gait_policy.h from the host side; it never
feeds actuator facts back into that policy.  The STM32 motor capability header
is the single source for both the regression thresholds and the firmware-side
trot3 limiter.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import re

from .shared_gait import SharedGaitPolicy


LEGS = ("FL", "FR", "RL", "RR")
JOINTS = (1, 2, 3)
CONTROL_PERIOD_SECONDS = 0.020


@dataclass(frozen=True)
class MotorCapability:
    nominal_velocity_deg_s: float
    regression_transient_margin: float
    command_velocity_deg_s: float
    command_acceleration_deg_s2: float

    @property
    def transient_velocity_deg_s(self) -> float:
        return self.nominal_velocity_deg_s * self.regression_transient_margin


@dataclass(frozen=True)
class JointVelocityStats:
    leg: str
    joint: int
    maximum_velocity_deg_s: float
    rms_velocity_deg_s: float
    maximum_delta_deg: float
    maximum_phase: float


@dataclass(frozen=True)
class GaitVelocityReport:
    gait: str
    period_ms: int
    joints: tuple[JointVelocityStats, ...]
    capability: MotorCapability

    @property
    def bottleneck(self) -> JointVelocityStats:
        return max(self.joints, key=lambda item: item.maximum_velocity_deg_s)

    @property
    def status(self) -> str:
        peak = self.bottleneck.maximum_velocity_deg_s
        if peak <= self.capability.nominal_velocity_deg_s:
            return "within-nominal"
        if peak <= self.capability.transient_velocity_deg_s:
            return "transient-margin"
        return "infeasible"


def _macro_float(text: str, name: str) -> float:
    match = re.search(
        rf"^#define\s+{re.escape(name)}\s+([0-9.]+)f?\s*$",
        text,
        re.MULTILINE,
    )
    if match is None:
        raise RuntimeError(f"motor capability macro is missing: {name}")
    return float(match.group(1))


def load_motor_capability() -> MotorCapability:
    root = Path(__file__).resolve().parents[3]
    header = root / "firmware/stm32-learning/Inc/motor_capability.h"
    text = header.read_text(encoding="utf-8")
    return MotorCapability(
        nominal_velocity_deg_s=_macro_float(
            text, "MOTOR_STS3215_NOMINAL_MAX_VELOCITY_DEG_S"
        ),
        regression_transient_margin=_macro_float(
            text, "MOTOR_STS3215_REGRESSION_TRANSIENT_MARGIN"
        ),
        command_velocity_deg_s=_macro_float(
            text, "MOTOR_STS3215_COMMAND_VELOCITY_LIMIT_DEG_S"
        ),
        command_acceleration_deg_s2=_macro_float(
            text, "MOTOR_STS3215_COMMAND_ACCELERATION_LIMIT_DEG_S2"
        ),
    )


def analyze_gait_velocity(
    gait: str,
    *,
    period_ms: int | None = None,
    policy: SharedGaitPolicy | None = None,
) -> GaitVelocityReport:
    """Finite-difference one full 50 Hz cycle, including its wrap frame."""
    if gait not in {"trot", "trot2", "trot3"}:
        raise ValueError("gait must be 'trot', 'trot2' or 'trot3'")
    if period_ms is None:
        period_ms = 1400 if gait == "trot3" else 800
    frames_per_cycle = round((period_ms / 1000.0) / CONTROL_PERIOD_SECONDS)
    if frames_per_cycle < 2:
        raise ValueError("gait period is too short for 50 Hz analysis")

    shared = policy or SharedGaitPolicy()

    def targets(phase: float):
        if gait == "trot":
            return shared.trot_targets(phase, 1.0, 1.0)[0]
        if gait == "trot3":
            return shared.trot3_targets(phase, 1.0, 78.0, 108.0)[0]
        return shared.trot2_targets(phase, 1.0, 78.0, 108.0)[0]

    frames = [targets(frame / frames_per_cycle)
              for frame in range(frames_per_cycle)]
    statistics: list[JointVelocityStats] = []
    for leg in LEGS:
        for joint in JOINTS:
            key = (leg, joint)
            deltas = [
                frames[(frame + 1) % frames_per_cycle][key]
                - frames[frame][key]
                for frame in range(frames_per_cycle)
            ]
            velocities = [delta / CONTROL_PERIOD_SECONDS for delta in deltas]
            maximum_index = max(
                range(frames_per_cycle),
                key=lambda frame: abs(velocities[frame]),
            )
            statistics.append(
                JointVelocityStats(
                    leg=leg,
                    joint=joint,
                    maximum_velocity_deg_s=abs(velocities[maximum_index]),
                    rms_velocity_deg_s=math.sqrt(
                        sum(value * value for value in velocities)
                        / frames_per_cycle
                    ),
                    maximum_delta_deg=abs(deltas[maximum_index]),
                    maximum_phase=maximum_index / frames_per_cycle,
                )
            )

    return GaitVelocityReport(
        gait=gait,
        period_ms=period_ms,
        joints=tuple(statistics),
        capability=load_motor_capability(),
    )


def format_velocity_report(report: GaitVelocityReport) -> str:
    lines = [
        f"{report.gait} period={report.period_ms}ms status={report.status}",
        "joint  max_deg_s  rms_deg_s  max_delta_deg  phase",
    ]
    for item in report.joints:
        lines.append(
            f"{item.leg}-J{item.joint}  {item.maximum_velocity_deg_s:9.2f}  "
            f"{item.rms_velocity_deg_s:9.2f}  {item.maximum_delta_deg:13.3f}  "
            f"{item.maximum_phase:5.3f}"
        )
    capability = report.capability
    lines.append(
        "limits: nominal="
        f"{capability.nominal_velocity_deg_s:.1f}deg/s, "
        f"transient={capability.transient_velocity_deg_s:.1f}deg/s, "
        f"trot3-command={capability.command_velocity_deg_s:.1f}deg/s"
    )
    return "\n".join(lines)


def main() -> int:
    for gait in ("trot", "trot2", "trot3"):
        print(format_velocity_report(analyze_gait_velocity(gait)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
