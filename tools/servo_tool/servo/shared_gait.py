"""Host binding for the HAL-independent STM32/MuJoCo gait policy."""

from __future__ import annotations

import ctypes
import hashlib
import os
from pathlib import Path
import platform
import shutil
import subprocess
import tempfile

from .attitude import ImuSample


LEGS = ("FL", "FR", "RL", "RR")
JOINTS = (1, 2, 3)


class SharedGaitPolicy:
    """Compile once and call the exact C policy used by the STM32 firmware."""

    def __init__(self) -> None:
        self._library = ctypes.CDLL(str(self._build_library()))
        for name, result_type in (("slew", ctypes.c_int16), ("period_ms", ctypes.c_uint16)):
            fn = getattr(self._library, "spot_gait_drive_" + name)
            fn.argtypes = (ctypes.c_int16, ctypes.c_int16)
            fn.restype = result_type
        float_pointer = ctypes.POINTER(ctypes.c_float)
        self._library.spot_gait_trot5_targets.argtypes = (
            ctypes.c_float, ctypes.c_float, float_pointer,
            ctypes.POINTER(ctypes.c_uint8),
        )
        self._library.spot_gait_trot5_targets.restype = ctypes.c_int
        self._library.spot_gait_sim_trot_targets.argtypes = (
            ctypes.c_float,
            ctypes.c_float,
            float_pointer,
            ctypes.POINTER(ctypes.c_uint8),
        )
        self._library.spot_gait_sim_trot_targets.restype = ctypes.c_int
        self._library.spot_gait_trot_targets.argtypes = (
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_float,
            float_pointer,
            ctypes.POINTER(ctypes.c_uint8),
        )
        self._library.spot_gait_trot_targets.restype = ctypes.c_int
        self._library.spot_gait_trot2_targets.argtypes = (
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_float,
            float_pointer,
            ctypes.POINTER(ctypes.c_uint8),
        )
        self._library.spot_gait_trot2_targets.restype = ctypes.c_int
        self._library.spot_gait_trot3_targets.argtypes = (
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_float,
            float_pointer,
            ctypes.POINTER(ctypes.c_uint8),
        )
        self._library.spot_gait_trot3_targets.restype = ctypes.c_int
        self._library.spot_gait_trot4_targets.argtypes = (
            ctypes.c_float,
            ctypes.c_float,
            float_pointer,
            ctypes.POINTER(ctypes.c_uint8),
        )
        self._library.spot_gait_trot4_targets.restype = ctypes.c_int
        self._library.spot_gait_trot4_direction_targets.argtypes = (
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_int,
            float_pointer,
            ctypes.POINTER(ctypes.c_uint8),
        )
        self._library.spot_gait_trot4_direction_targets.restype = ctypes.c_int
        self._library.spot_gait_turn_targets.argtypes = (
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_int,
            float_pointer,
            ctypes.POINTER(ctypes.c_uint8),
        )
        self._library.spot_gait_turn_targets.restype = ctypes.c_int
        self._library.spot_gait_drive_targets.argtypes = (
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_float,
            float_pointer,
            ctypes.POINTER(ctypes.c_uint8),
        )
        self._library.spot_gait_drive_targets.restype = ctypes.c_int
        self._library.spot_gait_drive_walk_targets.argtypes = self._library.spot_gait_drive_targets.argtypes
        self._library.spot_gait_drive_walk_targets.restype = ctypes.c_int
        self._library.spot_gait_drive_stride_targets.argtypes = (
            ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_float,
            ctypes.c_float, float_pointer, ctypes.POINTER(ctypes.c_uint8))
        self._library.spot_gait_drive_stride_targets.restype = ctypes.c_int
        self._library.spot_gait_drive_yaw_limit.argtypes = (ctypes.c_int16,)
        self._library.spot_gait_drive_yaw_limit.restype = ctypes.c_int16
        self._library.spot_gait_crab_targets.argtypes = (
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_int,
            float_pointer,
            ctypes.POINTER(ctypes.c_uint8),
        )
        self._library.spot_gait_crab_targets.restype = ctypes.c_int
        self._library.spot_gait_jump_targets.argtypes = (
            ctypes.c_float,
            ctypes.c_float,
            float_pointer,
            ctypes.POINTER(ctypes.c_uint8),
        )
        self._library.spot_gait_jump_targets.restype = ctypes.c_int
        self._library.spot_gait_balance_targets.argtypes = (
            float_pointer,
            float_pointer,
            ctypes.c_int,
            ctypes.c_uint8,
            float_pointer,
        )
        self._library.spot_gait_balance_targets.restype = ctypes.c_int
        self._library.spot_gait_smootherstep.argtypes = (ctypes.c_float,)
        self._library.spot_gait_smootherstep.restype = ctypes.c_float

    @staticmethod
    def _paths() -> tuple[Path, Path]:
        root = Path(__file__).resolve().parents[3]
        wrapper = Path(__file__).resolve().with_name("gait_policy_host.c")
        header = root / "firmware" / "stm32-learning" / "Inc" / "gait_policy.h"
        return wrapper, header

    @classmethod
    def _build_library(cls) -> Path:
        wrapper, header = cls._paths()
        manifest=header.parents[3]/"config"/"locomotion_profiles.json"
        manifest_hash=hashlib.sha256(manifest.read_bytes()).hexdigest()
        if manifest_hash not in header.with_name("locomotion_profiles.h").read_text():
            raise RuntimeError("Stale deployed gait header: run tools/generate_locomotion_profiles.py")
        digest = hashlib.sha256(
            wrapper.read_bytes() + header.read_bytes() + b"".join(header.with_name(name).read_bytes() for name in ("heading_control.h", "locomotion.h", "locomotion_profiles.h", "balance_control.h", "drive_control.h", "attitude_control.h", "locomotion_servo.h", "robot_config.h", "motor_capability.h")) + (header.parent.parent / "Src" / "robot_config.c").read_bytes()
        ).hexdigest()[:16]
        extension = ".dylib" if platform.system() == "Darwin" else ".so"
        build_dir = Path(tempfile.gettempdir()) / "spot-omg-gait-policy"
        build_dir.mkdir(parents=True, exist_ok=True)
        library = build_dir / f"libspot_gait_{digest}{extension}"
        if library.exists():
            return library

        compiler = os.environ.get("CC") or shutil.which("cc")
        if compiler is None:
            raise RuntimeError("a C compiler is required for the shared gait policy")
        link_mode = "-dynamiclib" if platform.system() == "Darwin" else "-shared"
        command = [
            compiler,
            "-std=c11",
            "-O2",
            "-fPIC",
            link_mode,
            str(wrapper),
            str(header.parent.parent / "Src" / "robot_config.c"),
            "-I",
            str(header.parent),
            "-o",
            str(library),
            "-lm",
        ]
        completed = subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
        )
        if completed.returncode != 0:
            raise RuntimeError(
                "failed to compile shared gait policy:\n" + completed.stderr
            )
        return library

    @staticmethod
    def _unpack(values) -> dict[tuple[str, int], float]:
        return {
            (leg, joint): float(values[leg_index * 3 + joint - 1])
            for leg_index, leg in enumerate(LEGS)
            for joint in JOINTS
        }

    @staticmethod
    def _pack(targets: dict[tuple[str, int], float]):
        return (ctypes.c_float * 12)(
            *[
                float(targets[(leg, joint)])
                for leg in LEGS
                for joint in JOINTS
            ]
        )

    def sim_trot_targets(
        self,
        phase: float,
        amplitude_scale: float,
    ) -> tuple[dict[tuple[str, int], float], set[str]]:
        values = (ctypes.c_float * 12)()
        mask = ctypes.c_uint8()
        ok = self._library.spot_gait_sim_trot_targets(
            phase,
            amplitude_scale,
            values,
            ctypes.byref(mask),
        )
        if not ok:
            raise ValueError("shared C gait policy rejected the requested phase")
        support = {
            leg for index, leg in enumerate(LEGS) if mask.value & (1 << index)
        }
        return self._unpack(values), support

    def trot_targets(
        self,
        phase: float,
        amplitude_scale: float,
        travel_scale: float,
    ) -> tuple[dict[tuple[str, int], float], set[str]]:
        """Return a shared trot frame with independently scaled travel."""
        values = (ctypes.c_float * 12)()
        mask = ctypes.c_uint8()
        ok = self._library.spot_gait_trot_targets(
            phase,
            amplitude_scale,
            travel_scale,
            values,
            ctypes.byref(mask),
        )
        if not ok:
            raise ValueError("shared C trot policy rejected the requested phase")
        support = {
            leg for index, leg in enumerate(LEGS) if mask.value & (1 << index)
        }
        return self._unpack(values), support

    def trot2_targets(
        self,
        phase: float,
        amplitude_scale: float,
        fold_j2: float,
        fold_j3: float,
    ) -> tuple[dict[tuple[str, int], float], set[str]]:
        """Return one circular-foot diagonal-trot frame."""
        values = (ctypes.c_float * 12)()
        mask = ctypes.c_uint8()
        ok = self._library.spot_gait_trot2_targets(
            phase,
            amplitude_scale,
            fold_j2,
            fold_j3,
            values,
            ctypes.byref(mask),
        )
        if not ok:
            raise ValueError("shared C trot2 policy rejected the requested frame")
        support = {
            leg for index, leg in enumerate(LEGS) if mask.value & (1 << index)
        }
        return self._unpack(values), support

    def trot3_targets(
        self,
        phase: float,
        amplitude_scale: float,
        fold_j2: float,
        fold_j3: float,
    ) -> tuple[dict[tuple[str, int], float], set[str]]:
        """Return the circular-foot gait with four-foot support overlap."""
        values = (ctypes.c_float * 12)()
        mask = ctypes.c_uint8()
        ok = self._library.spot_gait_trot3_targets(
            phase,
            amplitude_scale,
            fold_j2,
            fold_j3,
            values,
            ctypes.byref(mask),
        )
        if not ok:
            raise ValueError("shared C trot3 policy rejected the requested frame")
        support = {
            leg for index, leg in enumerate(LEGS) if mask.value & (1 << index)
        }
        return self._unpack(values), support

    def trot4_targets(
        self,
        phase: float,
        amplitude_scale: float,
    ) -> tuple[dict[tuple[str, int], float], set[str]]:
        """Return the reduced-path, acceleration-smooth physical gait."""
        values = (ctypes.c_float * 12)()
        mask = ctypes.c_uint8()
        ok = self._library.spot_gait_trot4_targets(
            phase,
            amplitude_scale,
            values,
            ctypes.byref(mask),
        )
        if not ok:
            raise ValueError("shared C trot4 policy rejected the requested frame")
        support = {
            leg for index, leg in enumerate(LEGS) if mask.value & (1 << index)
        }
        return self._unpack(values), support

    def crab_targets(
        self,
        phase: float,
        amplitude_scale: float,
        direction: int,
    ) -> tuple[dict[tuple[str, int], float], set[str]]:
        """Return the sideways diagonal gait for direction +1 left/-1 right."""
        values = (ctypes.c_float * 12)()
        mask = ctypes.c_uint8()
        ok = self._library.spot_gait_crab_targets(
            phase,
            amplitude_scale,
            direction,
            values,
            ctypes.byref(mask),
        )
        if not ok:
            raise ValueError("shared C crab policy rejected the requested frame")
        support = {
            leg for index, leg in enumerate(LEGS) if mask.value & (1 << index)
        }
        return self._unpack(values), support

    def trot4_direction_targets(
        self,
        phase: float,
        amplitude_scale: float,
        direction: int,
    ) -> tuple[dict[tuple[str, int], float], set[str]]:
        """Return trot4 in direction +1 forward/-1 backward."""
        values = (ctypes.c_float * 12)()
        mask = ctypes.c_uint8()
        ok = self._library.spot_gait_trot4_direction_targets(
            phase,
            amplitude_scale,
            direction,
            values,
            ctypes.byref(mask),
        )
        if not ok:
            raise ValueError("shared C directional trot4 rejected the frame")
        support = {
            leg for index, leg in enumerate(LEGS) if mask.value & (1 << index)
        }
        return self._unpack(values), support

    def turn_targets(
        self,
        phase: float,
        amplitude_scale: float,
        direction: int,
    ) -> tuple[dict[tuple[str, int], float], set[str]]:
        """Return a differential trot turn for +1 left/-1 right."""
        values = (ctypes.c_float * 12)()
        mask = ctypes.c_uint8()
        ok = self._library.spot_gait_turn_targets(
            phase,
            amplitude_scale,
            direction,
            values,
            ctypes.byref(mask),
        )
        if not ok:
            raise ValueError("shared C turn policy rejected the requested frame")
        support = {
            leg for index, leg in enumerate(LEGS) if mask.value & (1 << index)
        }
        return self._unpack(values), support

    def trot5_targets(self, phase: float, amplitude_scale: float = 1.0):
        """Selected CAD gait, using the same C implementation as firmware."""
        values = (ctypes.c_float * 12)()
        mask = ctypes.c_uint8()
        if not self._library.spot_gait_trot5_targets(
                phase, amplitude_scale, values, ctypes.byref(mask)):
            raise ValueError("shared C trot5 policy rejected the requested frame")
        return self._unpack(values), {
            leg for index, leg in enumerate(LEGS) if mask.value & (1 << index)
        }

    def drive_slew(self, current: int, target: int) -> int:
        if max(abs(current), abs(target)) > 1000:
            raise ValueError("drive inputs must be within -1000..1000")
        return int(self._library.spot_gait_drive_slew(current, target))

    def drive_period_ms(self, linear: int, yaw: int) -> int:
        if max(abs(linear), abs(yaw)) > 1000:
            raise ValueError("drive inputs must be within -1000..1000")
        return int(self._library.spot_gait_drive_period_ms(linear, yaw))

    def drive_yaw_limit(self, requested: int) -> int:
        """Firmware joystick yaw governor, in protocol units -1000..1000."""
        if not isinstance(requested, int) or not -1000 <= requested <= 1000:
            raise ValueError("yaw input must be an integer in -1000..1000")
        return int(self._library.spot_gait_drive_yaw_limit(requested))

    def drive_stride_targets(self, phase, startup, linear, yaw, stride):
        values = (ctypes.c_float * 12)()
        mask = ctypes.c_uint8()
        if not self._library.spot_gait_drive_stride_targets(
                phase, startup, linear, yaw, stride, values, ctypes.byref(mask)):
            raise ValueError("shared C stride policy rejected frame")
        return self._unpack(values), {
            leg for i, leg in enumerate(LEGS) if mask.value & (1 << i)}

    def drive_walk_targets(self, phase, startup, linear, yaw):
        values = (ctypes.c_float * 12)()
        mask = ctypes.c_uint8()
        if not self._library.spot_gait_drive_walk_targets(
                phase, startup, linear, yaw, values, ctypes.byref(mask)):
            raise ValueError("shared C walk stance rejected frame")
        return self._unpack(values), {
            leg for i, leg in enumerate(LEGS) if mask.value & (1 << i)}

    def drive_targets(
        self,
        phase: float,
        startup_scale: float,
        linear: float,
        yaw: float,
    ) -> tuple[dict[tuple[str, int], float], set[str]]:
        """Blend continuous longitudinal and yaw control without a phase reset."""
        values = (ctypes.c_float * 12)()
        mask = ctypes.c_uint8()
        ok = self._library.spot_gait_drive_targets(
            phase,
            startup_scale,
            linear,
            yaw,
            values,
            ctypes.byref(mask),
        )
        if not ok:
            raise ValueError("shared C drive policy rejected the requested frame")
        support = {
            leg for index, leg in enumerate(LEGS) if mask.value & (1 << index)
        }
        return self._unpack(values), support

    def smootherstep(self, progress: float) -> float:
        """Return the same bounded start/stop ramp used by STM32."""
        return float(self._library.spot_gait_smootherstep(progress))

    def jump_targets(
        self,
        phase: float,
        forward_travel: float,
    ) -> tuple[dict[tuple[str, int], float], set[str]]:
        """Return one frame of the shared repeating jump trajectory."""
        values = (ctypes.c_float * 12)()
        mask = ctypes.c_uint8()
        ok = self._library.spot_gait_jump_targets(
            phase,
            forward_travel,
            values,
            ctypes.byref(mask),
        )
        if not ok:
            raise ValueError("shared C jump policy rejected the requested phase")
        support = {
            leg for index, leg in enumerate(LEGS) if mask.value & (1 << index)
        }
        return self._unpack(values), support

    def balance_targets(
        self,
        targets: dict[tuple[str, int], float],
        *,
        sample: ImuSample,
        support_legs: set[str],
        kp: float,
        kd: float,
        leg_length_limit: float,
        mode: str,
        j1_gain: float,
        j1_limit: float,
        foot_placement_gain: float,
        foot_placement_limit: float,
    ) -> dict[tuple[str, int], float]:
        sample.validate()
        values = self._pack(targets)
        imu = (ctypes.c_float * 4)(
            sample.roll,
            sample.pitch,
            sample.roll_rate,
            sample.pitch_rate,
        )
        balance = (ctypes.c_float * 7)(
            kp,
            kd,
            leg_length_limit,
            j1_gain,
            j1_limit,
            foot_placement_gain,
            foot_placement_limit,
        )
        support_mask = sum(
            1 << LEGS.index(leg) for leg in support_legs
        )
        ok = self._library.spot_gait_balance_targets(
            imu,
            balance,
            int(mode == "contact-aware"),
            support_mask,
            values,
        )
        if not ok:
            raise ValueError("shared C balance policy produced an invalid target")
        return self._unpack(values)
