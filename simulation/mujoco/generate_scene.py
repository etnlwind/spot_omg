#!/usr/bin/env python3
"""Generate a dynamic MuJoCo scene from the canonical Spot OMG URDF."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import mujoco


ROOT = Path(__file__).resolve().parents[2]
URDF = ROOT / "hardware" / "urdf" / "spot_omg.urdf"
PARAMETERS = ROOT / "hardware" / "urdf" / "spot_omg_parameters.json"
OUTPUT = Path(__file__).resolve().parent / "spot_omg_scene.xml"
LEGS = ("fl", "fr", "rl", "rr")


def padded(*values: float) -> list[float]:
    return [*values, *([0.0] * (10 - len(values)))]


def build_scene() -> mujoco.MjSpec:
    parameters = json.loads(PARAMETERS.read_text(encoding="utf-8"))
    actuator = parameters["actuator"]
    simulation = parameters["simulation"]

    spec = mujoco.MjSpec.from_file(str(URDF))
    spec.modelname = "spot_omg_dynamic"
    spec.comment = "Generated from hardware/urdf/spot_omg.urdf; do not hand-edit."
    # Preserve fixed sensor and foot bodies, including their measured masses.
    spec.compiler.fusestatic = False
    spec.option.timestep = 0.002
    spec.option.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    spec.option.solver = mujoco.mjtSolver.mjSOL_NEWTON
    spec.option.iterations = 100
    spec.option.noslip_iterations = 10

    base = spec.body("base_link")
    if base is None:
        raise ValueError("URDF base_link was not found")
    base.add_freejoint(name="root")

    imu = spec.body("imu_link")
    if imu is None:
        raise ValueError("URDF imu_link was not found")
    imu.add_site(
        name="body_imu_site",
        type=mujoco.mjtGeom.mjGEOM_SPHERE,
        size=[0.005, 0.0, 0.0],
        rgba=[0.1, 0.7, 1.0, 0.35],
    )
    # Match the useful BNO086 outputs: fused orientation plus calibrated gyro.
    spec.add_sensor(
        name="body_imu_orientation",
        type=mujoco.mjtSensor.mjSENS_FRAMEQUAT,
        objtype=mujoco.mjtObj.mjOBJ_SITE,
        objname="body_imu_site",
    )
    spec.add_sensor(
        name="body_imu_gyro",
        type=mujoco.mjtSensor.mjSENS_GYRO,
        objtype=mujoco.mjtObj.mjOBJ_SITE,
        objname="body_imu_site",
    )

    foot_friction = [
        simulation["foot_static_friction_initial"],
        0.005,
        0.0001,
    ]
    for leg in LEGS:
        foot = spec.body(f"{leg}_foot_link")
        if foot is None or len(foot.geoms) != 1:
            raise ValueError(f"expected one collision geom on {leg}_foot_link")
        foot_geom = foot.geoms[0]
        foot_geom.name = f"{leg}_foot_geom"
        foot_geom.friction = foot_friction
        foot_geom.condim = 3

    spec.worldbody.add_geom(
        name="ground",
        type=mujoco.mjtGeom.mjGEOM_PLANE,
        size=[5.0, 5.0, 0.1],
        friction=foot_friction,
        condim=3,
        rgba=[0.18, 0.22, 0.26, 1.0],
    )
    spec.worldbody.add_light(
        name="sun",
        pos=[0.0, 0.0, 3.0],
        dir=[0.0, 0.0, -1.0],
        type=mujoco.mjtLightType.mjLIGHT_DIRECTIONAL,
        diffuse=[0.8, 0.8, 0.8],
        ambient=[0.2, 0.2, 0.2],
    )

    kp = 35.0
    kv = 0.8
    effort = actuator["stall_effort_nm"]
    for leg in LEGS:
        for joint_number in (1, 2, 3):
            joint_name = f"{leg}_j{joint_number}"
            joint = spec.joint(joint_name)
            if joint is None:
                raise ValueError(f"URDF joint not found: {joint_name}")
            joint.armature = 0.002
            joint.actfrclimited = mujoco.mjtLimited.mjLIMITED_TRUE
            joint.actfrcrange = [-effort, effort]
            spec.add_actuator(
                name=f"{joint_name}_position",
                gaintype=mujoco.mjtGain.mjGAIN_FIXED,
                gainprm=padded(kp),
                biastype=mujoco.mjtBias.mjBIAS_AFFINE,
                biasprm=padded(0.0, -kp, -kv),
                trntype=mujoco.mjtTrn.mjTRN_JOINT,
                target=joint_name,
                ctrllimited=mujoco.mjtLimited.mjLIMITED_TRUE,
                ctrlrange=list(joint.range),
                forcelimited=mujoco.mjtLimited.mjLIMITED_TRUE,
                forcerange=[-effort, effort],
            )

    # Compile here so generation fails immediately on an invalid dynamic model.
    spec.compile()
    return spec


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=OUTPUT)
    parser.add_argument(
        "--check",
        action="store_true",
        help="fail if the checked-in scene differs from generated output",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    generated = build_scene().to_xml()
    if not generated.endswith("\n"):
        generated += "\n"
    if args.check:
        if not args.output.exists() or args.output.read_text(encoding="utf-8") != generated:
            print(f"out of date: {args.output}", file=sys.stderr)
            return 1
        print(f"up to date: {args.output}")
        return 0
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(generated, encoding="utf-8")
    print(f"generated: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
