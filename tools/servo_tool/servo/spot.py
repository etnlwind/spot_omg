"""Twelve-joint Spot Micro configuration and synchronized motion helpers."""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

from .bus import ServoBus
from .protocol import encode_u16
from .sts3215 import STS3215, ServoState

LEG_ORDER = ("FL", "FR", "RL", "RR")
JOINTS_PER_LEG = 3
ADDR_ACCELERATION = STS3215.ADDR_ACCELERATION
TICKS_PER_REVOLUTION = STS3215.STEPS_PER_REVOLUTION


@dataclass
class JointConfig:
    name: str
    leg: str
    joint: int
    servo_id: int
    center: int
    offset: int
    minimum: int
    maximum: int
    direction: int = 1


@dataclass(frozen=True)
class GaitParameters:
    pattern: str = "trot"
    period: float = 2.4
    hip_amplitude: float = 19.3
    lift_amplitude: float = 29.9
    crouch_amplitude: float = 6.2
    speed: int = 60
    acceleration: int = 25
    duty_factor: float = 0.75
    control_rate: float = 30.0

    def validate(self) -> None:
        if self.pattern not in {"trot", "crawl"}:
            raise ValueError("gait pattern must be 'trot' or 'crawl'")
        if not 0.6 <= self.period <= 10.0:
            raise ValueError("period must be between 0.6 and 10.0 seconds")
        if not 0.1 <= self.hip_amplitude <= 30.0:
            raise ValueError("hip amplitude must be between 0.1 and 30 degrees")
        if not 0.1 <= self.lift_amplitude <= 40.0:
            raise ValueError("lift amplitude must be between 0.1 and 40 degrees")
        if not 0 <= self.crouch_amplitude <= 15.0:
            raise ValueError("crouch amplitude must be between 0 and 15 degrees")
        if not 1 <= self.speed <= 3400:
            raise ValueError("speed must be between 1 and 3400")
        if not 0 <= self.acceleration <= 254:
            raise ValueError("acceleration must be between 0 and 254")
        if not 0.5 <= self.duty_factor < 1.0:
            raise ValueError("duty factor must be at least 0.5 and below 1.0")
        if not 10.0 <= self.control_rate <= 100.0:
            raise ValueError("control rate must be between 10 and 100 Hz")


class SpotConfig:
    """Validated robot configuration loaded from ``joints.json``."""

    def __init__(
        self,
        joints: list[JointConfig],
        poses: dict[str, dict[int, int]],
        *,
        canonical_poses: dict[str, dict[int, float]] | None = None,
        gait_forward_signs: dict[str, int] | None = None,
        path: Path | None = None,
        reference_center: int = 2048,
        pose_saved_at: dict[str, str] | None = None,
        source: str = "",
        calibration_saved_at: str = "",
        gait_saved_at: str = "",
    ) -> None:
        self.joints = joints
        self.poses = poses
        self.canonical_poses = canonical_poses or {
            "landing": {2: 40.0, 3: 130.0}
        }
        self.gait_forward_signs = gait_forward_signs or {
            "FL": -1,
            "FR": -1,
            "RL": 1,
            "RR": 1,
        }
        self.path = path
        self.reference_center = reference_center
        self.pose_saved_at = pose_saved_at or {}
        self.source = source
        self.calibration_saved_at = calibration_saved_at
        self.gait_saved_at = gait_saved_at
        self._reindex()
        self._validate()

    def _reindex(self) -> None:
        self._by_id = {joint.servo_id: joint for joint in self.joints}
        self._by_leg_joint = {
            (joint.leg, joint.joint): joint for joint in self.joints
        }

    @classmethod
    def load(cls, path: str | Path) -> "SpotConfig":
        source = Path(path)
        data = json.loads(source.read_text(encoding="utf-8"))
        reference_center = int(data.get("reference_center", 2048))
        legacy_stand = data.get("stand45_directions", {})
        legacy_gait = data.get("gait_directions", {})

        def legacy_direction(item: dict) -> int:
            joint_number = int(item["joint"])
            if joint_number == 1:
                return 1
            leg = item["leg"]
            key = "upper" if joint_number == 2 else "lower"
            if leg in legacy_stand:
                return int(legacy_stand[leg][key])
            gait_key = "hip_forward" if joint_number == 2 else "knee_lift"
            return int(legacy_gait.get(leg, {}).get(gait_key, 1))

        joints = [
            JointConfig(
                name=item["name"],
                leg=item["leg"],
                joint=int(item["joint"]),
                servo_id=int(item["id"]),
                center=int(item["center"]),
                offset=int(
                    item.get("offset", int(item["center"]) - reference_center)
                ),
                minimum=int(item["min"]),
                maximum=int(item["max"]),
                direction=int(item.get("direction", legacy_direction(item))),
            )
            for item in data["joints"]
        ]
        poses = {
            name: {int(servo_id): int(position) for servo_id, position in pose.items()}
            for name, pose in data["poses"].items()
        }
        canonical_poses = {
            name: {int(joint): float(angle) for joint, angle in values.items()}
            for name, values in data.get("canonical_poses", {}).items()
        }
        gait_forward_signs = {
            leg: int(sign)
            for leg, sign in data.get("gait_forward_signs", {}).items()
        }
        return cls(
            joints,
            poses,
            canonical_poses=canonical_poses or None,
            gait_forward_signs=gait_forward_signs or None,
            path=source,
            reference_center=reference_center,
            pose_saved_at=dict(data.get("pose_saved_at", {})),
            source=str(data.get("source", "")),
            calibration_saved_at=str(data.get("calibration_saved_at", "")),
            gait_saved_at=str(data.get("gait_saved_at", "")),
        )

    @property
    def servo_ids(self) -> tuple[int, ...]:
        return tuple(joint.servo_id for joint in self.joints)

    def joint(self, leg: str, joint_number: int) -> JointConfig:
        return self._by_leg_joint[(leg, joint_number)]

    def pose(self, name: str) -> dict[int, int]:
        try:
            return dict(self.poses[name])
        except KeyError as exc:
            available = ", ".join(sorted(self.poses))
            raise KeyError(f"unknown pose {name!r}; available: {available}") from exc

    @property
    def directions(self) -> dict[str, tuple[int, int]]:
        """Compatibility view of canonical J2/J3 motor directions."""
        return {
            leg: (self.joint(leg, 2).direction, self.joint(leg, 3).direction)
            for leg in LEG_ORDER
        }

    @property
    def stand45_directions(self) -> dict[str, tuple[int, int]]:
        """Compatibility alias; stance and gait now share one direction."""
        return self.directions

    @staticmethod
    def degrees_to_ticks(angle_degrees: float) -> int:
        return round(angle_degrees * TICKS_PER_REVOLUTION / 360.0)

    @staticmethod
    def ticks_to_degrees(ticks: int) -> float:
        return ticks * 360.0 / TICKS_PER_REVOLUTION

    def angle_to_position(
        self, leg: str, joint_number: int, angle_degrees: float
    ) -> int:
        """Convert one canonical joint angle to a hardware raw position."""
        if not math.isfinite(angle_degrees):
            raise ValueError("joint angle must be finite")
        joint = self.joint(leg, joint_number)
        position = joint.center + joint.direction * self.degrees_to_ticks(
            angle_degrees
        )
        if not joint.minimum <= position <= joint.maximum:
            raise ValueError(
                f"{joint.name} angle {angle_degrees:g} degrees maps outside "
                f"{joint.minimum}..{joint.maximum}"
            )
        return position

    def position_to_angle(
        self, leg: str, joint_number: int, position: int
    ) -> float:
        """Convert a hardware raw position to a canonical joint angle."""
        joint = self.joint(leg, joint_number)
        if not joint.minimum <= position <= joint.maximum:
            raise ValueError(f"{joint.name} position is outside its limits")
        ticks = (position - joint.center) * joint.direction
        return self.ticks_to_degrees(ticks)

    def angles_to_targets(
        self, angles: dict[tuple[str, int], float]
    ) -> dict[int, int]:
        """Build absolute raw targets from canonical per-joint angles."""
        expected = {
            (leg, joint_number)
            for leg in LEG_ORDER
            for joint_number in range(1, JOINTS_PER_LEG + 1)
        }
        unknown = set(angles) - expected
        if unknown:
            raise ValueError(f"unknown joint angle keys: {sorted(unknown)}")
        targets = self.pose("neutral")
        for (leg, joint_number), angle in angles.items():
            joint = self.joint(leg, joint_number)
            targets[joint.servo_id] = self.angle_to_position(
                leg, joint_number, angle
            )
        self.validate_targets(targets)
        return targets

    def stand45_targets(self) -> dict[int, int]:
        # From a straight neutral leg, J2 tilts the upper link by 45 degrees.
        # J3 is relative to that upper link, so a 90-degree knee bend places
        # the lower link 45 degrees on the other side, forming a ">" shape.
        angles = {
            (leg, joint_number): angle
            for leg in LEG_ORDER
            for joint_number, angle in ((2, 45.0), (3, 90.0))
        }
        return self.angles_to_targets(angles)

    def landing_targets(self) -> dict[int, int]:
        """Build the calibrated landing pose from canonical joint angles."""
        # Keep the lower link horizontal: J3 is relative to J2, so their
        # canonical angle difference is 90 degrees.
        landing = self.canonical_poses["landing"]
        angles = {
            (leg, joint_number): angle
            for leg in LEG_ORDER
            for joint_number, angle in landing.items()
        }
        return self.angles_to_targets(angles)

    def set_joint_center(self, servo_id: int, center: int) -> None:
        joint = self._by_id[servo_id]
        if not joint.minimum <= center <= joint.maximum:
            raise ValueError(
                f"{joint.name} center must be within "
                f"{joint.minimum}..{joint.maximum}"
            )
        joint.center = center
        joint.offset = center - self.reference_center
        self.poses["neutral"][servo_id] = center
        self.calibration_saved_at = self._now()

    def set_directions(self, directions: dict[str, tuple[int, int]]) -> None:
        """Compatibility setter for J2/J3 directions."""
        if set(directions) != set(LEG_ORDER):
            raise ValueError("directions are required for all four legs")
        for leg, values in directions.items():
            self.joint(leg, 2).direction = values[0]
            self.joint(leg, 3).direction = values[1]
        self._validate()
        self.gait_saved_at = self._now()

    def set_joint_directions(
        self, directions: dict[str, tuple[int, int, int]]
    ) -> None:
        """Set the one canonical motor sign used by every motion path."""
        if set(directions) != set(LEG_ORDER):
            raise ValueError("directions are required for all four legs")
        for leg, values in directions.items():
            if len(values) != JOINTS_PER_LEG:
                raise ValueError("each leg requires J1, J2, J3 directions")
            for joint_number, direction in enumerate(values, start=1):
                self.joint(leg, joint_number).direction = direction
        self._validate()
        self.gait_saved_at = self._now()

    def set_pose(self, name: str, targets: dict[int, int]) -> None:
        clean_name = name.strip()
        if not clean_name or any(character in clean_name for character in "\r\n"):
            raise ValueError("pose name must be non-empty and fit on one line")
        if clean_name.lower() in {"neutral", "stand", "stand45", "landing"}:
            raise ValueError(
                f"pose name {clean_name!r} is reserved for calibrated stances"
            )
        self.validate_targets(targets)
        self.poses[clean_name] = dict(targets)
        self.pose_saved_at[clean_name] = self._now()

    def remap_ids(self, mapping: dict[tuple[str, int], int]) -> None:
        expected = {(leg, joint) for leg in LEG_ORDER for joint in range(1, 4)}
        if set(mapping) != expected or len(set(mapping.values())) != 12:
            raise ValueError("mapping requires twelve unique IDs for all joints")
        if any(not 1 <= servo_id <= 253 for servo_id in mapping.values()):
            raise ValueError("servo IDs must be between 1 and 253")
        id_changes = {
            joint.servo_id: mapping[(joint.leg, joint.joint)]
            for joint in self.joints
        }
        for joint in self.joints:
            joint.servo_id = mapping[(joint.leg, joint.joint)]
        self.poses = {
            name: {id_changes[old_id]: value for old_id, value in pose.items()}
            for name, pose in self.poses.items()
        }
        self._reindex()
        self._validate()
        self.calibration_saved_at = self._now()

    @staticmethod
    def _now() -> str:
        return datetime.now().astimezone().isoformat(timespec="seconds")

    def save(self, path: str | Path | None = None) -> None:
        destination = Path(path) if path is not None else self.path
        if destination is None:
            raise ValueError("configuration has no destination path")
        data = {
            "version": 3,
            "source": self.source,
            "reference_center": self.reference_center,
            "calibration_saved_at": self.calibration_saved_at,
            "gait_saved_at": self.gait_saved_at,
            "joints": [
                {
                    "name": joint.name,
                    "leg": joint.leg,
                    "joint": joint.joint,
                    "id": joint.servo_id,
                    "center": joint.center,
                    "offset": joint.offset,
                    "min": joint.minimum,
                    "max": joint.maximum,
                    "direction": joint.direction,
                }
                for joint in self.joints
            ],
            "pose_saved_at": self.pose_saved_at,
            "canonical_poses": {
                name: {str(joint): angle for joint, angle in values.items()}
                for name, values in self.canonical_poses.items()
            },
            "gait_forward_signs": self.gait_forward_signs,
            "poses": {
                name: {str(servo_id): value for servo_id, value in pose.items()}
                for name, pose in self.poses.items()
            },
        }
        temporary = destination.with_suffix(destination.suffix + ".tmp")
        temporary.write_text(
            json.dumps(data, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        temporary.replace(destination)
        self.path = destination
        self._write_reports(destination.parent)

    def _write_reports(self, directory: Path) -> None:
        calibration = [
            "# Spot Micro 서보 보정값",
            "",
            "> `config/joints.json`에서 자동 생성됩니다. 직접 수정하지 마세요.",
            "",
            f"- 기준 위치: `{self.reference_center}`",
            f"- 마지막 저장: `{self.calibration_saved_at or 'unknown'}`",
            "- 목표 위치 계산: `기준 위치 + Offset`",
            "",
            "| Leg | Joint | Servo ID | Offset | Target |",
            "|---|---:|---:|---:|---:|",
        ]
        for joint in self.joints:
            calibration.append(
                f"| {joint.leg} | {joint.joint} | {joint.servo_id} | "
                f"{joint.offset:+d} | {joint.center} |"
            )
        calibration.extend(
            [
                "",
                "## 확인된 관절 방향",
                "",
                "| Leg | J1 | J2 | J3 |",
                "|---|---:|---:|---:|",
            ]
        )
        for leg in LEG_ORDER:
            values = tuple(self.joint(leg, number).direction for number in (1, 2, 3))
            calibration.append(
                f"| {leg} | {values[0]:+d} | {values[1]:+d} | "
                f"{values[2]:+d} |"
            )
        calibration.append("")

        gait = [
            "# Spot Micro 보행 방향 설정",
            "",
            "> `config/joints.json`에서 자동 생성됩니다. 직접 수정하지 마세요.",
            "",
            f"- 마지막 저장: `{self.gait_saved_at or 'unknown'}`",
            "- 모든 포즈와 보행은 동일한 관절별 `Direction`을 사용합니다.",
            "- 애플리케이션 각도는 degree, 모터 출력만 raw tick입니다.",
            "",
            "| Leg | J1 Direction | J2 Direction | J3 Direction |",
            "|---|---:|---:|---:|",
        ]
        for leg in LEG_ORDER:
            values = tuple(self.joint(leg, number).direction for number in (1, 2, 3))
            gait.append(
                f"| {leg} | {values[0]:+d} | {values[1]:+d} | {values[2]:+d} |"
            )
        gait.append("")

        poses = [
            "# Spot Micro 저장 포즈",
            "",
            "> `config/joints.json`에서 자동 생성됩니다. 직접 수정하지 마세요.",
            "",
            "| Pose | Saved At | Leg | Joint | Servo ID | Position |",
            "|---|---|---|---:|---:|---:|",
        ]
        for name, targets in self.poses.items():
            if name == "neutral":
                continue
            saved_at = self.pose_saved_at.get(name, "unknown")
            for joint in self.joints:
                poses.append(
                    f"| {name} | {saved_at} | {joint.leg} | {joint.joint} | "
                    f"{joint.servo_id} | {targets[joint.servo_id]} |"
                )
        poses.append("")

        reports = {
            "servo_calibration.md": calibration,
            "gait_config.md": gait,
            "servo_poses.md": poses,
        }
        for filename, lines in reports.items():
            path = directory / filename
            temporary = path.with_suffix(path.suffix + ".tmp")
            temporary.write_text("\n".join(lines), encoding="utf-8")
            temporary.replace(path)

    def validate_targets(self, targets: dict[int, int]) -> None:
        if set(targets) != set(self.servo_ids):
            raise ValueError("targets must contain every configured servo exactly once")
        for servo_id, position in targets.items():
            joint = self._by_id[servo_id]
            if not joint.minimum <= position <= joint.maximum:
                raise ValueError(
                    f"{joint.name} (ID {servo_id}) target {position} is outside "
                    f"{joint.minimum}..{joint.maximum}"
                )

    def _validate(self) -> None:
        expected_pairs = {
            (leg, joint) for leg in LEG_ORDER for joint in range(1, 4)
        }
        pairs = {(joint.leg, joint.joint) for joint in self.joints}
        ids = [joint.servo_id for joint in self.joints]
        if pairs != expected_pairs or len(self.joints) != 12:
            raise ValueError("configuration requires joints 1..3 for all four legs")
        if len(set(ids)) != len(ids):
            raise ValueError("servo IDs must be unique")
        for joint in self.joints:
            if joint.center != self.reference_center + joint.offset:
                raise ValueError(f"{joint.name} center and offset do not match")
            if not joint.minimum <= joint.center <= joint.maximum:
                raise ValueError(f"{joint.name} center is outside its limits")
            if joint.direction not in (-1, 1):
                raise ValueError(f"{joint.name} direction must be +1 or -1")
        expected_ids = set(ids)
        for name, angles in self.canonical_poses.items():
            if not name or not angles:
                raise ValueError("canonical pose names and angles cannot be empty")
            if any(joint not in (1, 2, 3) for joint in angles):
                raise ValueError(f"canonical pose {name!r} has an unknown joint")
            if any(not math.isfinite(angle) for angle in angles.values()):
                raise ValueError(f"canonical pose {name!r} has a non-finite angle")
        if set(self.gait_forward_signs) != set(LEG_ORDER):
            raise ValueError("gait forward signs are required for all four legs")
        if any(sign not in (-1, 1) for sign in self.gait_forward_signs.values()):
            raise ValueError("gait forward signs must be +1 or -1")
        for name, targets in self.poses.items():
            if set(targets) != expected_ids:
                raise ValueError(f"pose {name!r} does not contain all servo IDs")
            self.validate_targets(targets)


class SpotRobot:
    """High-level synchronized control for the twelve-servo robot."""

    PHASE_OFFSETS = {
        # Diagonal pairs: FL+RR and FR+RL, separated by half a cycle.
        "trot": {"FL": 0.00, "RR": 0.00, "FR": 0.50, "RL": 0.50},
        # Original single-leg crawl: FR -> RR -> FL -> RL.
        "crawl": {"FR": 0.00, "RR": 0.25, "FL": 0.50, "RL": 0.75},
    }

    def __init__(self, bus: ServoBus, config: SpotConfig) -> None:
        self.bus = bus
        self.config = config

    @staticmethod
    def _motion_data(
        position: int, speed: int, acceleration: int, time_ms: int = 0
    ) -> bytes:
        if not 0 <= acceleration <= 0xFF:
            raise ValueError("acceleration must be between 0 and 255")
        return (
            bytes((acceleration,))
            + encode_u16(position)
            + encode_u16(time_ms)
            + encode_u16(speed)
        )

    def require_all(self) -> None:
        missing = [servo_id for servo_id in self.config.servo_ids if not self.bus.ping(servo_id)]
        if missing:
            raise RuntimeError(f"servos did not respond: {', '.join(map(str, missing))}")

    def read_positions(self) -> dict[int, int]:
        return {
            servo_id: STS3215(self.bus, servo_id).position
            for servo_id in self.config.servo_ids
        }

    def read_states(self) -> dict[int, ServoState]:
        return {
            servo_id: STS3215(self.bus, servo_id).read_state()
            for servo_id in self.config.servo_ids
        }

    def set_torque(self, enabled: bool) -> None:
        for servo_id in self.config.servo_ids:
            STS3215(self.bus, servo_id).enable_torque(enabled)

    def hold_current(
        self,
        servo_ids: set[int] | None = None,
        *,
        speed: int = 60,
        acceleration: int = 30,
    ) -> dict[int, int]:
        """Enable selected servos without jumping to stale goal positions."""
        if not 1 <= speed <= 3400:
            raise ValueError("speed must be between 1 and 3400")
        if not 0 <= acceleration <= 254:
            raise ValueError("acceleration must be between 0 and 254")
        selected = set(self.config.servo_ids) if servo_ids is None else set(servo_ids)
        if not selected or not selected <= set(self.config.servo_ids):
            raise ValueError("hold servo IDs must be configured")
        self.require_all()
        positions = self.read_positions()
        for servo_id in sorted(selected):
            servo = STS3215(self.bus, servo_id)
            servo.move(
                positions[servo_id], speed=speed, acceleration=acceleration
            )
            servo.enable_torque(True)
        return {servo_id: positions[servo_id] for servo_id in sorted(selected)}

    def sync_move(
        self,
        targets: dict[int, int],
        *,
        speed: int | dict[int, int] = 1000,
        acceleration: int = 80,
    ) -> None:
        self.config.validate_targets(targets)
        speeds = (
            {servo_id: speed for servo_id in targets}
            if isinstance(speed, int)
            else speed
        )
        if set(speeds) != set(targets):
            raise ValueError("speeds must contain every target servo exactly once")
        values = {
            servo_id: self._motion_data(
                position, speeds[servo_id], acceleration
            )
            for servo_id, position in targets.items()
        }
        self.bus.sync_write(ADDR_ACCELERATION, 7, values)

    def sync_positions(self, targets: dict[int, int]) -> None:
        """Stream only goal positions without resetting the STS motion profile."""
        self.config.validate_targets(targets)
        values = {
            servo_id: encode_u16(position)
            for servo_id, position in targets.items()
        }
        self.bus.sync_write(STS3215.ADDR_GOAL_POSITION, 2, values)

    def move_subset_and_wait(
        self,
        targets: dict[int, int],
        *,
        speed: int = 60,
        acceleration: int = 30,
        timeout: float = 10.0,
        tolerance: int = 50,
        poll_interval: float = 0.1,
    ) -> dict[int, ServoState]:
        """Move selected raw joint targets without changing other servo goals."""
        if not 1 <= speed <= 3400:
            raise ValueError("speed must be between 1 and 3400")
        if not 0 <= acceleration <= 254:
            raise ValueError("acceleration must be between 0 and 254")
        if not targets or not set(targets) <= set(self.config.servo_ids):
            raise ValueError("subset targets must use configured servo IDs")
        for servo_id, position in targets.items():
            joint = self.config._by_id[servo_id]
            if not joint.minimum <= position <= joint.maximum:
                raise ValueError(f"{joint.name} target is outside its limits")
        self.require_all()
        current = self.read_positions()
        current_values = {
            servo_id: self._motion_data(
                current[servo_id], speed, acceleration
            )
            for servo_id in targets
        }
        self.bus.sync_write(ADDR_ACCELERATION, 7, current_values)
        for servo_id in sorted(targets):
            STS3215(self.bus, servo_id).enable_torque(True)

        target_values = {
            servo_id: self._motion_data(position, speed, acceleration)
            for servo_id, position in targets.items()
        }
        self.bus.sync_write(ADDR_ACCELERATION, 7, target_values)
        deadline = time.monotonic() + timeout
        time.sleep(poll_interval)
        while True:
            states = {
                servo_id: STS3215(self.bus, servo_id).read_state()
                for servo_id in targets
            }
            if all(not state.moving for state in states.values()):
                break
            if time.monotonic() >= deadline:
                raise TimeoutError("selected joints did not stop before timeout")
            time.sleep(poll_interval)
        errors = {
            servo_id: state.position - targets[servo_id]
            for servo_id, state in states.items()
            if abs(state.position - targets[servo_id]) > tolerance
        }
        if errors:
            detail = ", ".join(
                f"{servo_id}={error:+d}" for servo_id, error in errors.items()
            )
            raise RuntimeError(f"target position verification failed: {detail}")
        return states

    def wait_until_stopped(
        self,
        targets: dict[int, int],
        *,
        timeout: float = 10.0,
        tolerance: int = 30,
        poll_interval: float = 0.1,
    ) -> dict[int, ServoState]:
        """Wait for all servos, then verify their final target error."""
        if timeout <= 0 or poll_interval <= 0:
            raise ValueError("timeout and poll interval must be positive")
        if tolerance < 0:
            raise ValueError("tolerance cannot be negative")
        self.config.validate_targets(targets)
        deadline = time.monotonic() + timeout
        # Give the servo control loop time to expose its moving flag.
        time.sleep(poll_interval)
        while True:
            states = self.read_states()
            if all(not state.moving for state in states.values()):
                break
            if time.monotonic() >= deadline:
                moving = [
                    str(servo_id)
                    for servo_id, state in states.items()
                    if state.moving
                ]
                raise TimeoutError(
                    "servos did not stop before timeout: " + ", ".join(moving)
                )
            time.sleep(poll_interval)

        errors = {
            servo_id: state.position - targets[servo_id]
            for servo_id, state in states.items()
            if abs(state.position - targets[servo_id]) > tolerance
        }
        if errors:
            detail = ", ".join(
                f"{servo_id}={error:+d}" for servo_id, error in errors.items()
            )
            raise RuntimeError(f"target position verification failed: {detail}")
        return states

    def move_and_wait(
        self,
        targets: dict[int, int],
        *,
        speed: int = 1000,
        acceleration: int = 80,
        timeout: float = 10.0,
        tolerance: int = 30,
    ) -> dict[int, ServoState]:
        self.sync_move(targets, speed=speed, acceleration=acceleration)
        return self.wait_until_stopped(
            targets, timeout=timeout, tolerance=tolerance
        )

    def prepare_for_motion(self, *, speed: int, acceleration: int) -> None:
        """Enable torque without jumping to a stale goal position."""
        self.require_all()
        current = self.read_positions()
        self.sync_move(current, speed=speed, acceleration=acceleration)
        self.set_torque(True)

    @staticmethod
    def _smoothstep(value: float) -> float:
        return value**3 * (value * (value * 6.0 - 15.0) + 10.0)

    @staticmethod
    def _cosine_ease(value: float) -> float:
        """Periodic joint interpolation with less endpoint dwell than smootherstep."""
        return 0.5 - 0.5 * math.cos(math.pi * value)

    @staticmethod
    def leg_forward_kinematics(
        upper_angle: float,
        knee_angle: float,
        *,
        upper_length: float = 1.0,
        lower_length: float = 1.0,
    ) -> tuple[float, float]:
        """Return canonical foot (forward, down) for the planar J2/J3 chain."""
        upper = math.radians(upper_angle)
        lower = math.radians(upper_angle - knee_angle)
        forward = upper_length * math.sin(upper) + lower_length * math.sin(lower)
        down = upper_length * math.cos(upper) + lower_length * math.cos(lower)
        return forward, down

    @staticmethod
    def leg_inverse_kinematics(
        forward: float,
        down: float,
        *,
        upper_length: float = 1.0,
        lower_length: float = 1.0,
    ) -> tuple[float, float]:
        """Solve canonical J2/J3 angles for a planar foot coordinate."""
        radius_squared = forward * forward + down * down
        cosine_knee = (
            radius_squared - upper_length**2 - lower_length**2
        ) / (2.0 * upper_length * lower_length)
        if not -1.0 <= cosine_knee <= 1.0:
            raise ValueError(
                f"foot target ({forward:.3f}, {down:.3f}) is outside IK workspace"
            )
        knee = math.acos(max(-1.0, min(1.0, cosine_knee)))
        upper = math.atan2(forward, down) + math.atan2(
            lower_length * math.sin(knee),
            upper_length + lower_length * math.cos(knee),
        )
        return math.degrees(upper), math.degrees(knee)

    def gait_targets(
        self,
        phase: float,
        base: dict[int, int],
        parameters: GaitParameters,
        *,
        amplitude_scale: float = 1.0,
    ) -> dict[int, int]:
        parameters.validate()
        if not 0.0 <= amplitude_scale <= 1.0:
            raise ValueError("amplitude scale must be between 0 and 1")
        if amplitude_scale == 0.0:
            return dict(base)
        targets = dict(base)
        phase_offsets = self.PHASE_OFFSETS[parameters.pattern]
        for leg in LEG_ORDER:
            leg_phase = (phase + phase_offsets[leg]) % 1.0
            if leg_phase < parameters.duty_factor:
                progress = leg_phase / parameters.duty_factor
                hip_wave = 1.0 - 2.0 * self._cosine_ease(progress)
                lift_wave = 0.0
            else:
                progress = (leg_phase - parameters.duty_factor) / (
                    1.0 - parameters.duty_factor
                )
                hip_wave = -1.0 + 2.0 * self._cosine_ease(progress)
                lift_wave = math.sin(math.pi * progress) ** 2

            hip_joint = self.config.joint(leg, 2)
            knee_joint = self.config.joint(leg, 3)
            hip_base = self.config.position_to_angle(
                leg, 2, base[hip_joint.servo_id]
            )
            knee_base = self.config.position_to_angle(
                leg, 3, base[knee_joint.servo_id]
            )
            base_forward, base_down = self.leg_forward_kinematics(
                hip_base, knee_base
            )

            # CLI amplitudes remain intuitive degree values, but are converted
            # into a Cartesian foot path before solving both joints together.
            stride = base_down * math.sin(math.radians(parameters.hip_amplitude))
            nominal_knee = knee_base + parameters.crouch_amplitude * amplitude_scale
            _, nominal_down = self.leg_forward_kinematics(hip_base, nominal_knee)
            lifted_knee = nominal_knee + parameters.lift_amplitude * amplitude_scale
            _, lifted_down = self.leg_forward_kinematics(hip_base, lifted_knee)
            lift_height = max(0.0, nominal_down - lifted_down)

            foot_forward = (
                base_forward
                + self.config.gait_forward_signs[leg]
                * stride
                * hip_wave
                * amplitude_scale
            )
            foot_down = nominal_down - lift_height * lift_wave
            hip_angle, knee_angle = self.leg_inverse_kinematics(
                foot_forward, foot_down
            )
            targets[hip_joint.servo_id] = self.config.angle_to_position(
                leg, 2, hip_angle
            )
            targets[knee_joint.servo_id] = self.config.angle_to_position(
                leg, 3, knee_angle
            )

        self.config.validate_targets(targets)
        return targets
