"""Build phase-aligned no-contact load baselines from gait CSV logs."""

from __future__ import annotations

import csv
import json
import math
import statistics
from collections import defaultdict
from pathlib import Path


def _percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    if not ordered:
        raise ValueError("cannot calculate a percentile of no values")
    index = min(len(ordered) - 1, round((len(ordered) - 1) * fraction))
    return ordered[index]


def _ceil_multiple(value: float, multiple: int = 4) -> int:
    return int(math.ceil(value / multiple) * multiple)


class DynamicLoadBaseline:
    """Median motor load indexed by servo and local gait phase."""

    VERSION = 2

    def __init__(self, models: dict[int, dict], *, source: str = "") -> None:
        if not models:
            raise ValueError("dynamic load baseline cannot be empty")
        self.models = models
        self.source = source

    @classmethod
    def from_csv(
        cls, path: str | Path, *, minimum_amplitude: float = 0.99
    ) -> "DynamicLoadBaseline":
        source = Path(path)
        rows = []
        with source.open(newline="", encoding="utf-8") as handle:
            for raw in csv.DictReader(handle):
                try:
                    amplitude = float(raw["amplitude"])
                    row = {
                        "leg": raw["leg"],
                        "joint": int(raw["joint"]),
                        "servo_id": int(raw["servo_id"]),
                        "elapsed": float(raw["elapsed_s"]),
                        "global_phase": float(raw["phase"]),
                        "phase": round(float(raw["leg_phase"]), 2),
                        "load": abs(int(raw["load"])),
                    }
                except (KeyError, TypeError, ValueError) as exc:
                    raise ValueError(f"invalid load-profile row: {raw}") from exc
                if amplitude >= minimum_amplitude:
                    rows.append(row)
        if not rows:
            raise ValueError("load profile has no full-amplitude samples")

        # Discard acceleration/ramp behavior and retain only complete cycles.
        # The first phase wrap starts the first fully settled cycle; the last
        # wrap ends the last complete cycle before the shutdown ramp.
        ordered = sorted(rows, key=lambda row: row["elapsed"])
        wraps = [
            current["elapsed"]
            for previous, current in zip(ordered, ordered[1:])
            if current["global_phase"] < previous["global_phase"] - 0.5
        ]
        if len(wraps) >= 2:
            window_start, window_end = wraps[0], wraps[-1]
            rows = [
                row
                for row in ordered
                if window_start <= row["elapsed"] < window_end
            ]
        if not rows:
            raise ValueError("load profile has no complete steady gait cycles")

        grouped: dict[tuple[int, float], list[int]] = defaultdict(list)
        identity = {}
        for row in rows:
            servo_id = row["servo_id"]
            identity.setdefault(servo_id, (row["leg"], row["joint"]))
            if identity[servo_id] != (row["leg"], row["joint"]):
                raise ValueError(f"servo {servo_id} changes identity in profile")
            grouped[(servo_id, row["phase"])].append(row["load"])

        expected_joints = {
            (leg, joint)
            for leg in ("FL", "FR", "RL", "RR")
            for joint in (2, 3)
        }
        if len(identity) != 8 or set(identity.values()) != expected_joints:
            raise ValueError("load profile requires J2/J3 for all four legs")

        models = {}
        for servo_id, (leg, joint) in sorted(identity.items()):
            phase_groups = {
                phase: values
                for (candidate, phase), values in grouped.items()
                if candidate == servo_id
            }
            if len(phase_groups) < 5:
                raise ValueError(
                    f"servo {servo_id} has too few distinct phase samples"
                )
            points = []
            residuals = []
            for phase, values in sorted(phase_groups.items()):
                median_load = statistics.median(values)
                residuals.extend(abs(value - median_load) for value in values)
                points.append(
                    {
                        "phase": phase,
                        "load": median_load,
                        "samples": len(values),
                    }
                )
            noise_p90 = _percentile(residuals, 0.90)
            noise_p95 = _percentile(residuals, 0.95)
            engage = max(24, _ceil_multiple(noise_p95 + 16))
            release = max(8, _ceil_multiple(noise_p90 + 4))
            release = min(release, engage - 4)
            models[servo_id] = {
                "leg": leg,
                "joint": joint,
                "noise_p90": noise_p90,
                "noise_p95": noise_p95,
                "noise_max": max(residuals),
                "engage_threshold": engage,
                "release_threshold": release,
                "points": points,
            }
        return cls(models, source=str(source))

    def expected_load(self, servo_id: int, leg_phase: float) -> float:
        try:
            points = self.models[servo_id]["points"]
        except KeyError as exc:
            raise KeyError(f"servo {servo_id} is absent from load baseline") from exc
        phase = leg_phase % 1.0

        def circular_distance(point: dict) -> float:
            delta = abs(phase - point["phase"])
            return min(delta, 1.0 - delta)

        return float(min(points, key=circular_distance)["load"])

    def residual(self, servo_id: int, leg_phase: float, load: int) -> float:
        return load - self.expected_load(servo_id, leg_phase)

    def save(self, path: str | Path) -> None:
        destination = Path(path)
        destination.parent.mkdir(parents=True, exist_ok=True)
        payload = {
            "version": self.VERSION,
            "source": self.source,
            "load_mode": "absolute",
            "models": {str(key): value for key, value in self.models.items()},
        }
        destination.write_text(
            json.dumps(payload, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )

    @classmethod
    def load(cls, path: str | Path) -> "DynamicLoadBaseline":
        payload = json.loads(Path(path).read_text(encoding="utf-8"))
        if payload.get("version") != cls.VERSION:
            raise ValueError("unsupported dynamic load baseline version")
        if payload.get("load_mode") != "absolute":
            raise ValueError("dynamic load baseline must use absolute load")
        return cls(
            {int(key): value for key, value in payload["models"].items()},
            source=str(payload.get("source", "")),
        )
