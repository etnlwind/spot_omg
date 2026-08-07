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
        float_pointer = ctypes.POINTER(ctypes.c_float)
        signs_pointer = ctypes.POINTER(ctypes.c_int8)
        self._library.spot_gait_sim_trot_targets.argtypes = (
            ctypes.c_float,
            ctypes.c_float,
            signs_pointer,
            float_pointer,
            ctypes.POINTER(ctypes.c_uint8),
        )
        self._library.spot_gait_sim_trot_targets.restype = ctypes.c_int
        self._library.spot_gait_trot_targets.argtypes = (
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_float,
            signs_pointer,
            float_pointer,
            ctypes.POINTER(ctypes.c_uint8),
        )
        self._library.spot_gait_trot_targets.restype = ctypes.c_int
        self._library.spot_gait_trot2_targets.argtypes = (
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_float,
            ctypes.c_float,
            signs_pointer,
            float_pointer,
            ctypes.POINTER(ctypes.c_uint8),
        )
        self._library.spot_gait_trot2_targets.restype = ctypes.c_int
        self._library.spot_gait_jump_targets.argtypes = (
            ctypes.c_float,
            ctypes.c_float,
            signs_pointer,
            float_pointer,
            ctypes.POINTER(ctypes.c_uint8),
        )
        self._library.spot_gait_jump_targets.restype = ctypes.c_int
        self._library.spot_gait_balance_targets.argtypes = (
            float_pointer,
            float_pointer,
            ctypes.c_int,
            ctypes.c_uint8,
            signs_pointer,
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
        digest = hashlib.sha256(
            wrapper.read_bytes() + header.read_bytes()
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
    def _sign_array(forward_signs: dict[str, int]):
        values = [int(forward_signs[leg]) for leg in LEGS]
        if any(value not in (-1, 1) for value in values):
            raise ValueError("forward signs must be -1 or +1 for every leg")
        return (ctypes.c_int8 * 4)(*values)

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
        forward_signs: dict[str, int],
    ) -> tuple[dict[tuple[str, int], float], set[str]]:
        values = (ctypes.c_float * 12)()
        mask = ctypes.c_uint8()
        ok = self._library.spot_gait_sim_trot_targets(
            phase,
            amplitude_scale,
            self._sign_array(forward_signs),
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
        forward_signs: dict[str, int],
    ) -> tuple[dict[tuple[str, int], float], set[str]]:
        """Return a shared trot frame with independently scaled travel."""
        values = (ctypes.c_float * 12)()
        mask = ctypes.c_uint8()
        ok = self._library.spot_gait_trot_targets(
            phase,
            amplitude_scale,
            travel_scale,
            self._sign_array(forward_signs),
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
        forward_signs: dict[str, int],
    ) -> tuple[dict[tuple[str, int], float], set[str]]:
        """Return one circular-foot diagonal-trot frame."""
        values = (ctypes.c_float * 12)()
        mask = ctypes.c_uint8()
        ok = self._library.spot_gait_trot2_targets(
            phase,
            amplitude_scale,
            fold_j2,
            fold_j3,
            self._sign_array(forward_signs),
            values,
            ctypes.byref(mask),
        )
        if not ok:
            raise ValueError("shared C trot2 policy rejected the requested frame")
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
        forward_signs: dict[str, int],
    ) -> tuple[dict[tuple[str, int], float], set[str]]:
        """Return one frame of the shared repeating jump trajectory."""
        values = (ctypes.c_float * 12)()
        mask = ctypes.c_uint8()
        ok = self._library.spot_gait_jump_targets(
            phase,
            forward_travel,
            self._sign_array(forward_signs),
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
        forward_signs: dict[str, int],
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
            self._sign_array(forward_signs),
            values,
        )
        if not ok:
            raise ValueError("shared C balance policy produced an invalid target")
        return self._unpack(values)
