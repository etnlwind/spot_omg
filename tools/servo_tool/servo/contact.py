"""Per-leg foot-contact estimation from STS3215 load feedback."""

from __future__ import annotations

import math
from dataclasses import dataclass


LEGS = ("FL", "FR", "RL", "RR")
LOAD_JOINTS = (2, 3)


@dataclass(frozen=True)
class ContactEstimate:
    contact: bool
    score: int
    j2_delta: int
    j3_delta: int
    changed: bool


class LoadContactEstimator:
    """Hysteretic, debounced contact detector for four legs."""

    def __init__(
        self,
        baseline: dict[tuple[str, int], int],
        *,
        engage_threshold: int = 24,
        release_threshold: int = 8,
        engage_samples: int = 2,
        release_samples: int = 3,
    ) -> None:
        expected = {
            (leg, joint_number)
            for leg in LEGS
            for joint_number in LOAD_JOINTS
        }
        if set(baseline) != expected:
            raise ValueError("load baseline requires J2 and J3 for all four legs")
        if any(not -1023 <= value <= 1023 for value in baseline.values()):
            raise ValueError("load baseline values must be within -1023..1023")
        if not 1 <= release_threshold < engage_threshold <= 1023:
            raise ValueError(
                "load thresholds must satisfy 1 <= release < engage <= 1023"
            )
        if not 1 <= engage_samples <= 100 or not 1 <= release_samples <= 100:
            raise ValueError("contact debounce samples must be between 1 and 100")
        self.baseline = dict(baseline)
        self.engage_threshold = engage_threshold
        self.release_threshold = release_threshold
        self.engage_samples = engage_samples
        self.release_samples = release_samples
        self._contacts = {leg: False for leg in LEGS}
        self._engage_counts = {leg: 0 for leg in LEGS}
        self._release_counts = {leg: 0 for leg in LEGS}

    def update(
        self, loads: dict[tuple[str, int], int]
    ) -> dict[str, ContactEstimate]:
        if set(loads) != set(self.baseline):
            raise ValueError("load sample requires J2 and J3 for all four legs")
        if any(not math.isfinite(value) for value in loads.values()):
            raise ValueError("load sample values must be finite")

        estimates = {}
        for leg in LEGS:
            j2_delta = loads[(leg, 2)] - self.baseline[(leg, 2)]
            j3_delta = loads[(leg, 3)] - self.baseline[(leg, 3)]
            score = max(abs(j2_delta), abs(j3_delta))
            previous = self._contacts[leg]

            if previous:
                if score <= self.release_threshold:
                    self._release_counts[leg] += 1
                else:
                    self._release_counts[leg] = 0
                if self._release_counts[leg] >= self.release_samples:
                    self._contacts[leg] = False
                    self._release_counts[leg] = 0
            else:
                if score >= self.engage_threshold:
                    self._engage_counts[leg] += 1
                else:
                    self._engage_counts[leg] = 0
                if self._engage_counts[leg] >= self.engage_samples:
                    self._contacts[leg] = True
                    self._engage_counts[leg] = 0

            estimates[leg] = ContactEstimate(
                contact=self._contacts[leg],
                score=score,
                j2_delta=j2_delta,
                j3_delta=j3_delta,
                changed=self._contacts[leg] != previous,
            )
        return estimates
