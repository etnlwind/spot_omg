"""Audit V4/V5 steady commanded kinematics through the shared firmware C code.

No hardware, servo dynamics, floor contact, IMU residual or start/stop simulation.
The 50 Hz differences include the production 0.1-degree and servo-tick rounding.
Run with the project's Python environment; only NumPy and a C compiler are needed.
"""
import argparse
import csv
import ctypes
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools/servo_tool"))
from servo.host_build import build_shared, library_suffix

LEGS = ("FL", "FR", "RL", "RR")
OFFSETS = np.array((0., .5, .5, 0.))
JOINTS = tuple(f"{leg}_J{joint}" for leg in LEGS for joint in (1, 2, 3))
PROFILES = ("attitudepd_v4", "attitudepd_v5")
# Matches the observed configured speed and the repository ServoProfile model.
# Register readback is not a loaded speed measurement.
PROFILE_SPEED_TICKS_S = 3400
PROFILE_SPEED_DEG_S = PROFILE_SPEED_TICKS_S * 360 / 4096
SOURCES = (
    "config/locomotion_profiles.json",
    "firmware/stm32-learning/Inc/locomotion_profiles.h",
    "firmware/stm32-learning/Inc/locomotion.h",
    "firmware/stm32-learning/Inc/center_pivot.h",
    "firmware/stm32-learning/Inc/foot_lift.h",
    "firmware/stm32-learning/Inc/arc_swing_shape.h",
    "firmware/stm32-learning/Inc/arc_turn.h",
    "firmware/stm32-learning/Inc/locomotion_servo.h",
    "firmware/stm32-learning/Inc/motor_capability.h",
    "firmware/stm32-learning/Src/robot_config.c",
    "scripts/analysis/compare_v4_v5_cadence.py",
)
C_SOURCE = r'''
#include "foot_lift.h"
#include "locomotion_servo.h"
int profile_id(const char *name) { return locomotion_profile_id(name); }
float profile_period(int profile, float linear) {
    return locomotion_period(profile, linear, 0);
}
float next_phase(float phase, float period) {
    return fmodf(phase + .02f / period, 1.f);
}
int sample(int profile, float phase, float linear, unsigned front_mm,
           float angles[12], float feet[12], unsigned short ticks[12]) {
    GaitPolicyLegTarget target[4];
    uint32_t lift[4] = {front_mm, front_mm, 0, 0};
    if (!locomotion_targets(profile, phase, 1, linear, 0, target)) return 0;
    if (!foot_lift_profile(lift, profile, phase, fminf(1,fabsf(linear)/.15f),
                           linear, 0, target)) return 0;
    for (int leg = 0; leg < 4; leg++) {
        angles[leg*3] = target[leg].j1_deg;
        angles[leg*3+1] = target[leg].j2_deg;
        angles[leg*3+2] = target[leg].j3_deg;
        arc_foot(leg, &angles[leg*3], &feet[leg*3], 0);
    }
    return locomotion_servo_targets(target, ticks);
}
void parameters(int profile, float linear, float out[7]) {
    locomotion_params(profile, linear, out);
}
float forward_cadence_gain(int profile) {
    return locomotion_forward_cadence_gain[profile];
}
void joint_directions(float out[12]) {
    for (int i=0; i<12; i++) out[i]=g_robot_joints[i].direction;
}
'''


def source_hashes():
    return {name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest()
            for name in SOURCES}


class Kernel:
    def __init__(self, directory):
        source = directory / "comparison.c"
        source.write_text(C_SOURCE)
        library = build_shared(
            [source, ROOT / "firmware/stm32-learning/Src/robot_config.c"],
            ROOT / "firmware/stm32-learning/Inc",
            directory / ("comparison" + library_suffix()),
            extra=("-ffp-contract=off",),
        )
        self.lib = ctypes.CDLL(str(library))
        fp = ctypes.POINTER(ctypes.c_float)
        self.lib.profile_id.argtypes = (ctypes.c_char_p,)
        self.lib.profile_id.restype = ctypes.c_int
        self.lib.profile_period.argtypes = (ctypes.c_int, ctypes.c_float)
        self.lib.profile_period.restype = ctypes.c_float
        self.lib.next_phase.argtypes = (ctypes.c_float, ctypes.c_float)
        self.lib.next_phase.restype = ctypes.c_float
        self.lib.parameters.argtypes = (ctypes.c_int, ctypes.c_float, fp)
        self.lib.forward_cadence_gain.argtypes = (ctypes.c_int,)
        self.lib.forward_cadence_gain.restype = ctypes.c_float
        self.lib.sample.argtypes = (
            ctypes.c_int, ctypes.c_float, ctypes.c_float, ctypes.c_uint,
            fp, fp, ctypes.POINTER(ctypes.c_ushort),
        )
        self.lib.sample.restype = ctypes.c_int
        self.lib.joint_directions.argtypes = (fp,)
        directions = (ctypes.c_float * 12)()
        self.lib.joint_directions(directions)
        self.directions = np.array(directions)

    def sample(self, profile, phase, linear, lift):
        angles, feet = (ctypes.c_float * 12)(), (ctypes.c_float * 12)()
        ticks = (ctypes.c_ushort * 12)()
        if not self.lib.sample(profile, phase, linear, lift, angles, feet, ticks):
            raise ValueError(f"Infeasible target: profile={profile}, "
                             f"phase={phase}, linear={linear}, lift={lift}")
        return np.array(angles), np.array(feet).reshape(4, 3), np.array(ticks)


def analyze(kernel, name, linear, lift, samples, seconds):
    profile = kernel.lib.profile_id(name.encode())
    if profile < 0:
        raise ValueError(f"Missing profile {name}")
    period = float(kernel.lib.profile_period(profile, linear))
    params = (ctypes.c_float * 7)()
    kernel.lib.parameters(profile, linear, params)
    duty = float(params[1])
    phase = np.arange(samples) / samples
    outputs = [kernel.sample(profile, float(p), linear, lift) for p in phase]
    angles = np.array([row[0] for row in outputs])
    feet = np.array([row[1] for row in outputs])
    joint_low = np.tile((-30., -45., 0.), 4)
    joint_high = np.tile((30., 100., 150.), 4)
    nominal_limits_valid = bool(np.all(angles >= joint_low-.0001)
                                and np.all(angles <= joint_high+.0001))
    # Central differences are for X velocity only; acceleration uses 50 Hz.
    velocity = (np.roll(feet, -1, axis=0) - np.roll(feet, 1, axis=0)) / (2*period/samples)
    leg_results = {}
    for leg, label in enumerate(LEGS):
        local = (phase + OFFSETS[leg]) % 1
        # Exclude two fine samples at both joins from the peak calculation.
        stance = (local > 2/samples) & (local < duty-2/samples)
        swing = (local > duty+2/samples) & (local < 1-2/samples)
        start = kernel.sample(profile, float(-OFFSETS[leg]), linear, lift)[1][leg]
        end = kernel.sample(profile, float(duty-OFFSETS[leg]), linear, lift)[1][leg]
        leg_results[label] = {
            "stance_signed_mean_x_mm_s": float((end[0]-start[0])*1000/(duty*period)),
            "stance_mean_abs_x_mm_s": float(np.mean(abs(velocity[stance, leg, 0]))*1000),
            "stance_peak_abs_x_mm_s": float(np.max(abs(velocity[stance, leg, 0]))*1000),
            "stance_x_displacement_mm": float((end[0]-start[0])*1000),
            "swing_peak_abs_x_mm_s": float(np.max(abs(velocity[swing, leg, 0]))*1000),
            "body_frame_z_excursion_mm": float(np.ptp(feet[:, leg, 2])*1000),
        }

    # Seed directly at the settled input; do not mix startup/stop into cadence.
    timed, phase_now = [], 0.
    for _ in range(round(seconds/.02)+1):
        timed.append(kernel.sample(profile, phase_now, linear, lift))
        phase_now = kernel.lib.next_phase(phase_now, period)
    smooth_angles = np.array([row[0] for row in timed])
    ticks = np.array([row[2] for row in timed], dtype=float)
    tick_steps = np.diff(ticks, axis=0)*kernel.directions
    steps = tick_steps*360/4096
    quantized_velocity = steps/.02
    quantized_acceleration = np.diff(quantized_velocity, axis=0)/.02
    smooth_velocity = np.diff(smooth_angles, axis=0)/.02
    smooth_acceleration = np.diff(smooth_velocity, axis=0)/.02
    joint_results = {}
    for j, label in enumerate(JOINTS):
        quantized_excess = abs(quantized_velocity[:, j]) > PROFILE_SPEED_DEG_S + 1.e-6
        unrounded_excess = abs(smooth_velocity[:, j]) > PROFILE_SPEED_DEG_S + 1.e-6
        joint_results[label] = {
            "sampled_min_deg": float(angles[:, j].min()),
            "sampled_max_deg": float(angles[:, j].max()),
            "max_servo_tick_step_50hz": int(np.max(abs(tick_steps[:, j]))),
            "max_command_step_deg_50hz": float(np.max(abs(steps[:, j]))),
            "max_command_velocity_deg_s_50hz": float(np.max(abs(quantized_velocity[:, j]))),
            "max_command_acceleration_deg_s2_50hz": float(np.max(abs(quantized_acceleration[:, j]))),
            "max_unrounded_velocity_deg_s_50hz": float(np.max(abs(smooth_velocity[:, j]))),
            "max_unrounded_acceleration_deg_s2_50hz": float(np.max(abs(smooth_acceleration[:, j]))),
            "quantized_velocity_exceeds_configured_speed": bool(np.any(quantized_excess)),
            "unrounded_velocity_exceeds_configured_speed": bool(np.any(unrounded_excess)),
            "quantized_speed_exceed_fraction": float(np.mean(quantized_excess)),
            "unrounded_speed_exceed_fraction": float(np.mean(unrounded_excess)),
        }
    result = {
        "profile": name, "linear": linear, "foot_lift_mm": [lift, lift, 0, 0],
        "period_s": period, "duty": duty,
        "configured_forward_cadence_gain": float(kernel.lib.forward_cadence_gain(profile)),
        "interpolated_base_parameters": list(params),
        "stance_duration_s": duty*period, "swing_duration_s": (1-duty)*period,
        "sampled_all_targets_and_servo_angle_limits_valid": True,
        "sampled_nominal_joint_limits_valid": nominal_limits_valid,
        "command_speed_exceeds_configured_profile": any(
            j["quantized_velocity_exceeds_configured_speed"] for j in joint_results.values()),
        "unrounded_command_speed_exceeds_configured_profile": any(
            j["unrounded_velocity_exceeds_configured_speed"] for j in joint_results.values()),
        "legs": leg_results, "joints": joint_results,
    }
    return result, angles, feet


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path,
                        default=ROOT / "artifacts/attitudepd-v5/cadence-analysis")
    parser.add_argument("--phase-samples", type=int, default=1000)
    parser.add_argument("--seconds", type=float, default=12.)
    parser.add_argument("--candidate-label", default="current working-tree candidate")
    args = parser.parse_args()
    if args.phase_samples < 1000 or not np.isfinite(args.seconds) or args.seconds < 6:
        parser.error("Use at least 1000 phase samples and 6 seconds")
    hashes = source_hashes()
    cases, comparisons = [], []
    with tempfile.TemporaryDirectory() as temporary:
        kernel = Kernel(Path(temporary))
        for lift in (0, 30):
            for linear in (1., -1., .588, -.588):
                pair = [analyze(kernel, name, linear, lift, args.phase_samples, args.seconds)
                        for name in PROFILES]
                cases.extend(item[0] for item in pair)
                before, after = pair[0][0], pair[1][0]
                angle_error = float(np.max(abs(pair[1][1]-pair[0][1])))
                position_error = float(np.max(abs(pair[1][2]-pair[0][2]))*1000)
                comparisons.append({
                    "linear": linear, "foot_lift_mm": [lift, lift, 0, 0],
                    "max_matched_phase_angle_difference_deg": angle_error,
                    "max_matched_phase_foot_coordinate_difference_mm": position_error,
                    "matched_phase_geometry_equal_within_float_tolerance":
                        angle_error <= .0001 and position_error <= .001,
                    "cadence_multiplier": before["period_s"]/after["period_s"],
                    "FL_stance_mean_abs_x_speed_multiplier":
                        after["legs"]["FL"]["stance_mean_abs_x_mm_s"] /
                        before["legs"]["FL"]["stance_mean_abs_x_mm_s"],
                })
    if hashes != source_hashes():
        raise RuntimeError("Source changed during analysis; rerun against a frozen revision")
    report = {
        "scope": "Host shared-C steady target kinematics; no hardware motion or measured servo response",
        "candidate_label": args.candidate_label,
        "source_commit": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=ROOT, text=True).strip(),
        "source_sha256": hashes,
        "profiles": {name: json.loads((ROOT / SOURCES[0]).read_text())["profiles"][name] for name in PROFILES},
        "phase_samples": args.phase_samples, "steady_50hz_duration_s": args.seconds,
        "configured_speed_comparison": {
            "assumed_speed_register_ticks_s": PROFILE_SPEED_TICKS_S,
            "equivalent_deg_s": PROFILE_SPEED_DEG_S,
            "source": "Observed profile setting; ticks/s interpretation matches simulation/mujoco/runtime/servo_profile.py",
            "hardware_readback_in_this_analysis": False,
        },
        "limits": [
            "CAD feet are in the fixed robot body frame; these are not floor clearance or body travel measurements.",
            "Steady full amplitude, yaw zero, IMU residual zero; startup, stop, balance, contact and load are excluded.",
            "Fine-grid X velocity uses central differences; peaks exclude two samples at each stance/swing join.",
            "50 Hz commanded derivatives include 0.1-degree angle rounding and 360/4096-degree servo ticks.",
            "One tick corresponds to 4.39453125 deg/s and 219.7265625 deg/s^2 in successive 20 ms differences.",
            "Unrounded 50 Hz derivatives are also provided to expose quantization; sampled peaks are not continuous-time maxima.",
            "Exceeding the configured speed means the target sequence requires more angular travel per 20 ms than that speed permits; isolated one-tick excess can reflect quantization.",
            "Remaining below the configured speed does not imply followability: actual motor speed, acceleration ramps, load, voltage and latency can impose tighter limits.",
            "Valid targets and numerical limits do not prove motor speed/torque headroom, physical clearance, or safe floor gait.",
        ],
        "comparisons": comparisons, "cases": cases,
    }
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "summary.json").write_text(json.dumps(report, indent=2)+"\n")
    compact = []
    for case in cases:
        compact.append({
            "profile": case["profile"], "linear": case["linear"],
            "front_lift_mm": case["foot_lift_mm"][0], "period_s": case["period_s"],
            "FL_stance_mean_mm_s": case["legs"]["FL"]["stance_mean_abs_x_mm_s"],
            "FL_stance_peak_mm_s": case["legs"]["FL"]["stance_peak_abs_x_mm_s"],
            "FL_swing_peak_mm_s": case["legs"]["FL"]["swing_peak_abs_x_mm_s"],
            "peak_joint_velocity_deg_s": max(j["max_command_velocity_deg_s_50hz"] for j in case["joints"].values()),
            "peak_joint_acceleration_deg_s2": max(j["max_command_acceleration_deg_s2_50hz"] for j in case["joints"].values()),
        })
    with (args.output / "compact.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=compact[0])
        writer.writeheader()
        writer.writerows(compact)
    print(json.dumps({"output": str(args.output), "comparisons": comparisons,
                      "compact": compact}, indent=2))


if __name__ == "__main__":
    main()
