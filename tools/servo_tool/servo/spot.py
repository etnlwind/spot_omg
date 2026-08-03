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
    direction: int


@dataclass(frozen=True)
class GaitParameters:
    pattern: str = "trot"
    period: float = 2.4
    hip_amplitude: int = 220
    lift_amplitude: int = 340
    crouch_amplitude: int = 70
    speed: int = 60
    acceleration: int = 25
    duty_factor: float = 0.75
    control_rate: float = 30.0

    def validate(self) -> None:
        if self.pattern not in {"trot", "crawl"}:
            raise ValueError("gait pattern must be 'trot' or 'crawl'")
        if not 0.6 <= self.period <= 10.0:
            raise ValueError("period must be between 0.6 and 10.0 seconds")
        if not 1 <= self.hip_amplitude <= 300:
            raise ValueError("hip amplitude must be between 1 and 300")
        if not 1 <= self.lift_amplitude <= 450:
            raise ValueError("lift amplitude must be between 1 and 450")
        if not 0 <= self.crouch_amplitude <= 150:
            raise ValueError("crouch amplitude must be between 0 and 150")
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
        path: Path | None = None,
        reference_center: int = 2048,
        pose_saved_at: dict[str, str] | None = None,
        source: str = "",
        calibration_saved_at: str = "",
    ) -> None:
        self.joints = joints
        self.poses = poses
        self.path = path
        self.reference_center = reference_center
        self.pose_saved_at = pose_saved_at or {}
        self.source = source
        self.calibration_saved_at = calibration_saved_at
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
        legacy_directions = data.get("stand45_directions", {})

        def device_direction(item: dict[str, object]) -> int:
            if "direction" in item:
                return int(item["direction"])
            joint_number = int(item["joint"])
            if joint_number == 1:
                return 1
            values = legacy_directions[item["leg"]]
            configured = int(values["upper" if joint_number == 2 else "lower"])
            return -configured if joint_number == 2 else configured

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
                direction=device_direction(item),
            )
            for item in data["joints"]
        ]
        poses = {
            name: {int(servo_id): int(position) for servo_id, position in pose.items()}
            for name, pose in data["poses"].items()
        }
        return cls(
            joints,
            poses,
            path=source,
            reference_center=reference_center,
            pose_saved_at=dict(data.get("pose_saved_at", {})),
            source=str(data.get("source", "")),
            calibration_saved_at=str(data.get("calibration_saved_at", "")),
        )

    @property
    def servo_ids(self) -> tuple[int, ...]:
        return tuple(joint.servo_id for joint in self.joints)

    def joint(self, leg: str, joint_number: int) -> JointConfig:
        return self._by_leg_joint[(leg, joint_number)]

    def pose(self, name: str) -> dict[int, int]:
        """Return a legacy saved pose in raw servo coordinates."""
        try:
            return dict(self.poses[name])
        except KeyError as exc:
            available = ", ".join(sorted(self.poses))
            raise KeyError(f"unknown pose {name!r}; available: {available}") from exc

    def joint_direction(self, servo_id: int) -> int:
        """Return the raw-servo sign for positive logical joint motion."""
        return self._by_id[servo_id].direction

    def logical_to_raw(self, targets: dict[int, int]) -> dict[int, int]:
        """Convert zero-centered robot joint coordinates to STS3215 positions."""
        self.validate_logical_targets(targets)
        return self.logical_subset_to_raw(targets)

    def logical_subset_to_raw(self, targets: dict[int, int]) -> dict[int, int]:
        """Convert one or more calibrated logical joints to raw positions."""
        if not targets or not set(targets) <= set(self.servo_ids):
            raise ValueError("logical joint targets must use configured servo IDs")
        raw = {
            servo_id: self._by_id[servo_id].center
            + self.joint_direction(servo_id) * position
            for servo_id, position in targets.items()
        }
        for servo_id, position in raw.items():
            joint = self._by_id[servo_id]
            if not joint.minimum <= position <= joint.maximum:
                raise ValueError(
                    f"{joint.name} (ID {servo_id}) target {position} is outside "
                    f"{joint.minimum}..{joint.maximum}"
                )
        return raw

    def raw_to_logical(self, targets: dict[int, int]) -> dict[int, int]:
        """Convert STS3215 positions to zero-centered robot joint coordinates."""
        self.validate_targets(targets)
        return {
            servo_id: self.joint_direction(servo_id)
            * (position - self._by_id[servo_id].center)
            for servo_id, position in targets.items()
        }

    def neutral_targets(self) -> dict[int, int]:
        """Return the default straight-down stance: every logical joint is zero."""
        return {servo_id: 0 for servo_id in self.servo_ids}

    def logical_pose(self, name: str) -> dict[int, int]:
        return self.raw_to_logical(self.pose(name))

    def stand45_targets(self) -> dict[int, int]:
        """Build a 45-degree stance in calibrated logical coordinates."""
        targets = self.neutral_targets()
        for leg in LEG_ORDER:
            upper_id = self.joint(leg, 2).servo_id
            lower_id = self.joint(leg, 3).servo_id
            targets[upper_id] = -512
            # The knee angle is relative to J2. With the upper link at -45°,
            # +90° at J3 places the lower link at +45° and forms a '<' shape.
            targets[lower_id] = 1024
        self.validate_logical_targets(targets)
        return targets

    def landing_targets(self) -> dict[int, int]:
        """Build the calibrated deep landing stance in logical coordinates."""
        targets = self.neutral_targets()
        for leg in LEG_ORDER:
            targets[self.joint(leg, 2).servo_id] = -512
            targets[self.joint(leg, 3).servo_id] = 1536
        self.validate_logical_targets(targets)
        self.logical_to_raw(targets)
        return targets

    def calibration_reference(self, reference: str = "neutral") -> dict[int, int]:
        """Return a visual calibration pose without redefining logical zero."""
        targets = self.neutral_targets()
        if reference == "neutral":
            return targets
        if reference != "setup-j2-minus90":
            raise ValueError(f"unknown calibration reference: {reference}")
        for leg in LEG_ORDER:
            targets[self.joint(leg, 2).servo_id] = -1024
        self.validate_logical_targets(targets)
        return targets

    def calibration_targets(self, reference: str = "neutral") -> dict[int, int]:
        """Build a known reference pose while preserving saved center offsets."""
        return self.logical_to_raw(self.calibration_reference(reference))

    def capture_calibration(
        self,
        positions: dict[int, int],
        reference: str = "neutral",
        servo_ids: set[int] | None = None,
    ) -> dict[int, int]:
        """Infer neutral centers from measured positions in a known reference pose."""
        self.validate_targets(positions)
        reference_targets = self.calibration_targets(reference)
        reference_deltas = {
            joint.servo_id: reference_targets[joint.servo_id] - joint.center
            for joint in self.joints
        }
        all_centers = {
            servo_id: position - reference_deltas[servo_id]
            for servo_id, position in positions.items()
        }
        selected = set(self.servo_ids) if servo_ids is None else set(servo_ids)
        if not selected or not selected <= set(self.servo_ids):
            raise ValueError("calibration servo IDs must be configured")
        centers = {servo_id: all_centers[servo_id] for servo_id in selected}
        for servo_id, center in centers.items():
            self.set_joint_center(servo_id, center)
        self._validate()
        return centers

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

    def set_pose(self, name: str, targets: dict[int, int]) -> None:
        clean_name = name.strip()
        if not clean_name or any(character in clean_name for character in "\r\n"):
            raise ValueError("pose name must be non-empty and fit on one line")
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
            "version": 4,
            "source": self.source,
            "reference_center": self.reference_center,
            "calibration_saved_at": self.calibration_saved_at,
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
            "- 관절 명령 변환: `Raw = Target + Joint Direction × Logical`",
            "",
            "| Leg | Joint | Servo ID | Offset | Target | Joint Direction |",
            "|---|---:|---:|---:|---:|---:|",
        ]
        for joint in self.joints:
            calibration.append(
                f"| {joint.leg} | {joint.joint} | {joint.servo_id} | "
                f"{joint.offset:+d} | {joint.center} | "
                f"{self.joint_direction(joint.servo_id):+d} |"
            )
        calibration.append("")

        gait = [
            "# Spot Micro 보행 방향 설정",
            "",
            "> `config/joints.json`에서 자동 생성됩니다. 직접 수정하지 마세요.",
            "",
            "- 네 다리는 같은 논리 관절 부호를 사용합니다.",
            "- 대각선 다리 쌍은 관절값 부호가 아니라 위상만 다릅니다.",
            "",
            "| Leg | J2 Kinematic Sign | J3 Bend Sign | Trot Phase |",
            "|---|---:|---:|---:|",
        ]
        for leg in LEG_ORDER:
            phase = 0.0 if leg in {"FL", "RR"} else 0.5
            hip_sign = -1 if leg in {"FL", "FR"} else 1
            gait.append(f"| {leg} | {hip_sign:+d} | +1 | {phase:.1f} |")
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

    def validate_logical_targets(self, targets: dict[int, int]) -> None:
        if set(targets) != set(self.servo_ids):
            raise ValueError(
                "logical targets must contain every configured servo exactly once"
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
        for name, targets in self.poses.items():
            if set(targets) != expected_ids:
                raise ValueError(f"pose {name!r} does not contain all servo IDs")
            self.validate_targets(targets)


class SpotRobot:
    """High-level synchronized control for the twelve-servo robot."""

    # The same fore-aft foot trajectory maps to opposite J2 changes on the
    # front and rear mechanisms. This is kinematics, not servo polarity.
    HIP_KINEMATIC_SIGNS = {"FL": -1, "FR": -1, "RL": 1, "RR": 1}

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
        """Enable torque safely after replacing stale goals with current positions."""
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

    def sync_joints(
        self,
        targets: dict[int, int],
        *,
        speed: int | dict[int, int] = 1000,
        acceleration: int = 80,
    ) -> None:
        """Move calibrated joints expressed around logical zero."""
        self.sync_move(
            self.config.logical_to_raw(targets),
            speed=speed,
            acceleration=acceleration,
        )

    def move_joint_subset_and_wait(
        self,
        targets: dict[int, int],
        *,
        speed: int = 60,
        acceleration: int = 30,
        timeout: float = 10.0,
        tolerance: int = 50,
        poll_interval: float = 0.1,
    ) -> dict[int, ServoState]:
        """Synchronously move selected logical joints without changing other goals."""
        raw_targets = self.config.logical_subset_to_raw(targets)
        values = {
            servo_id: self._motion_data(position, speed, acceleration)
            for servo_id, position in raw_targets.items()
        }
        self.bus.sync_write(ADDR_ACCELERATION, 7, values)
        deadline = time.monotonic() + timeout
        time.sleep(poll_interval)
        while True:
            states = {
                servo_id: STS3215(self.bus, servo_id).read_state()
                for servo_id in raw_targets
            }
            if all(not state.moving for state in states.values()):
                break
            if time.monotonic() >= deadline:
                raise TimeoutError("selected joints did not stop before timeout")
            time.sleep(poll_interval)
        errors = {
            servo_id: state.position - raw_targets[servo_id]
            for servo_id, state in states.items()
            if abs(state.position - raw_targets[servo_id]) > tolerance
        }
        if errors:
            detail = ", ".join(
                f"{servo_id}={error:+d}" for servo_id, error in errors.items()
            )
            raise RuntimeError(f"target position verification failed: {detail}")
        return states

    def stream_joints(self, targets: dict[int, int]) -> None:
        """Stream calibrated logical positions without rewriting motion profile."""
        self.sync_positions(self.config.logical_to_raw(targets))

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

    def move_joints_and_wait(
        self,
        targets: dict[int, int],
        *,
        speed: int = 1000,
        acceleration: int = 80,
        timeout: float = 10.0,
        tolerance: int = 30,
    ) -> dict[int, ServoState]:
        """Move logical joints and verify the resulting raw servo positions."""
        raw_targets = self.config.logical_to_raw(targets)
        return self.move_and_wait(
            raw_targets,
            speed=speed,
            acceleration=acceleration,
            timeout=timeout,
            tolerance=tolerance,
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

            hip_id = self.config.joint(leg, 2).servo_id
            knee_id = self.config.joint(leg, 3).servo_id
            targets[hip_id] += round(
                self.HIP_KINEMATIC_SIGNS[leg]
                * parameters.hip_amplitude
                * hip_wave
                * amplitude_scale
            )
            knee_bend = (
                parameters.crouch_amplitude
                + parameters.lift_amplitude * lift_wave
            ) * amplitude_scale
            targets[knee_id] += round(knee_bend)

        self.config.logical_to_raw(targets)
        return targets
