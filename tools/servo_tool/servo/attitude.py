"""Body-attitude feedback shared by simulation and hardware controllers."""

from __future__ import annotations

import math
from dataclasses import dataclass


LEG_SIDE = {"FL": 1.0, "FR": -1.0, "RL": 1.0, "RR": -1.0}
LEG_END = {"FL": 1.0, "FR": 1.0, "RL": -1.0, "RR": -1.0}


@dataclass(frozen=True)
class ImuSample:
    """One fused IMU sample in the robot body coordinate system.

    X points forward, Y points left, and Z points upward. Angles and angular
    rates use radians and radians per second respectively.
    """

    roll: float
    pitch: float
    roll_rate: float
    pitch_rate: float

    def validate(self) -> None:
        if not all(
            math.isfinite(value)
            for value in (
                self.roll,
                self.pitch,
                self.roll_rate,
                self.pitch_rate,
            )
        ):
            raise ValueError("IMU sample values must be finite")


@dataclass(frozen=True)
class AttitudeController:
    """PD body-level controller that requests differential leg extension."""

    kp: float
    kd: float
    correction_limit: float

    def __post_init__(self) -> None:
        if not 0.0 <= self.kp <= 5.0:
            raise ValueError("attitude kp must be between 0 and 5")
        if not 0.0 <= self.kd <= 1.0:
            raise ValueError("attitude kd must be between 0 and 1")
        if not 0.0 < self.correction_limit <= 0.30:
            raise ValueError(
                "attitude correction limit must be above 0 and at most 0.30"
            )

    def leg_length_corrections(self, sample: ImuSample) -> dict[str, float]:
        """Return normalized downward-foot corrections for all four legs.

        Positive output extends a leg downward. Positive roll means the left
        side is high, so left legs shorten and right legs extend. Positive
        pitch means the front is low, so front legs extend and rear legs
        shorten.
        """
        roll_control, pitch_control = self.axis_controls(sample)
        corrections = {}
        for leg in LEG_SIDE:
            correction = (
                -LEG_SIDE[leg] * roll_control
                + LEG_END[leg] * pitch_control
            )
            corrections[leg] = max(
                -self.correction_limit,
                min(self.correction_limit, correction),
            )
        return corrections

    def axis_controls(self, sample: ImuSample) -> tuple[float, float]:
        """Return unclamped roll and pitch PD control efforts."""
        sample.validate()
        return (
            self.kp * sample.roll + self.kd * sample.roll_rate,
            self.kp * sample.pitch + self.kd * sample.pitch_rate,
        )
