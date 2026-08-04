#!/usr/bin/env python3
"""Preview Spot OMG canonical poses in MuJoCo using the checked-in URDF."""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import mujoco
import mujoco.viewer as mj_viewer


ROOT = Path(__file__).resolve().parents[2]
URDF = ROOT / "hardware" / "urdf" / "spot_omg.urdf"
LEGS = ("fl", "fr", "rl", "rr")
POSES = {
    "stand": {1: 0.0, 2: 0.0, 3: 0.0},
    "stand45": {1: 0.0, 2: 45.0, 3: 90.0},
    "landing": {1: 0.0, 2: 40.0, 3: 130.0},
}


def set_pose(model: mujoco.MjModel, data: mujoco.MjData, pose: str) -> None:
    """Apply canonical degrees by joint name, independent of qpos ordering."""
    for leg in LEGS:
        for joint_number, angle_deg in POSES[pose].items():
            name = f"{leg}_j{joint_number}"
            joint_id = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_JOINT, name
            )
            if joint_id < 0:
                raise ValueError(f"URDF joint not found: {name}")
            qpos_address = model.jnt_qposadr[joint_id]
            data.qpos[qpos_address] = math.radians(angle_deg)
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "pose",
        choices=tuple(POSES),
        nargs="?",
        default="stand45",
        help="canonical pose to preview (default: stand45)",
    )
    parser.add_argument(
        "--check",
        action="store_true",
        help="load and apply the pose without opening the viewer",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    model = mujoco.MjModel.from_xml_path(str(URDF))
    data = mujoco.MjData(model)

    # This is a kinematic pose comparison. Gravity would collapse the joints
    # because a plain URDF has no MuJoCo position actuators.
    model.opt.gravity[:] = 0.0
    set_pose(model, data, args.pose)

    angles = POSES[args.pose]
    print(
        f"MuJoCo pose: {args.pose} "
        f"(J1={angles[1]:g}deg, J2={angles[2]:g}deg, "
        f"J3={angles[3]:g}deg)"
    )
    if args.check:
        return 0

    mj_viewer.launch(model, data)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
