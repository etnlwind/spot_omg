#!/usr/bin/env python3
"""Generate the Spot OMG URDF from one measured-parameter JSON file."""

from __future__ import annotations

import argparse
import json
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


HERE = Path(__file__).resolve().parent
DEFAULT_PARAMETERS = HERE / "spot_omg_parameters.json"
DEFAULT_OUTPUT = HERE / "spot_omg.urdf"

LEGS = {
    # side: +1 left / -1 right; end: +1 front / -1 rear
    "fl": {"side": 1, "end": 1, "servo_ids": (1, 2, 3)},
    "fr": {"side": -1, "end": 1, "servo_ids": (4, 5, 6)},
    "rl": {"side": 1, "end": -1, "servo_ids": (7, 8, 9)},
    "rr": {"side": -1, "end": -1, "servo_ids": (10, 11, 12)},
}


def fmt(value: float) -> str:
    """Format a finite float without noisy trailing zeroes."""
    if not math.isfinite(value):
        raise ValueError("URDF values must be finite")
    return f"{value:.9g}"


def xyz(values: tuple[float, float, float]) -> str:
    return " ".join(fmt(value) for value in values)


def add_origin(parent: ET.Element, position: tuple[float, float, float]) -> None:
    ET.SubElement(parent, "origin", {"xyz": xyz(position), "rpy": "0 0 0"})


def box_inertia(mass: float, size: tuple[float, float, float]) -> dict[str, str]:
    x, y, z = size
    return {
        "ixx": fmt(mass * (y * y + z * z) / 12.0),
        "ixy": "0",
        "ixz": "0",
        "iyy": fmt(mass * (x * x + z * z) / 12.0),
        "iyz": "0",
        "izz": fmt(mass * (x * x + y * y) / 12.0),
    }


def sphere_inertia(mass: float, radius: float) -> dict[str, str]:
    moment = 0.4 * mass * radius * radius
    return {
        "ixx": fmt(moment),
        "ixy": "0",
        "ixz": "0",
        "iyy": fmt(moment),
        "iyz": "0",
        "izz": fmt(moment),
    }


def add_material(robot: ET.Element, name: str, rgba: str) -> None:
    material = ET.SubElement(robot, "material", {"name": name})
    ET.SubElement(material, "color", {"rgba": rgba})


def add_box_link(
    robot: ET.Element,
    *,
    name: str,
    size: tuple[float, float, float],
    mass: float,
    center: tuple[float, float, float],
    material: str,
) -> None:
    link = ET.SubElement(robot, "link", {"name": name})
    visual = ET.SubElement(link, "visual")
    add_origin(visual, center)
    visual_geometry = ET.SubElement(visual, "geometry")
    ET.SubElement(visual_geometry, "box", {"size": xyz(size)})
    ET.SubElement(visual, "material", {"name": material})

    collision = ET.SubElement(link, "collision")
    add_origin(collision, center)
    collision_geometry = ET.SubElement(collision, "geometry")
    ET.SubElement(collision_geometry, "box", {"size": xyz(size)})

    inertial = ET.SubElement(link, "inertial")
    add_origin(inertial, center)
    ET.SubElement(inertial, "mass", {"value": fmt(mass)})
    ET.SubElement(inertial, "inertia", box_inertia(mass, size))


def add_sphere_link(
    robot: ET.Element,
    *,
    name: str,
    radius: float,
    mass: float,
    material: str,
) -> None:
    link = ET.SubElement(robot, "link", {"name": name})
    visual = ET.SubElement(link, "visual")
    add_origin(visual, (0.0, 0.0, 0.0))
    visual_geometry = ET.SubElement(visual, "geometry")
    ET.SubElement(visual_geometry, "sphere", {"radius": fmt(radius)})
    ET.SubElement(visual, "material", {"name": material})

    collision = ET.SubElement(link, "collision")
    add_origin(collision, (0.0, 0.0, 0.0))
    collision_geometry = ET.SubElement(collision, "geometry")
    ET.SubElement(collision_geometry, "sphere", {"radius": fmt(radius)})

    inertial = ET.SubElement(link, "inertial")
    add_origin(inertial, (0.0, 0.0, 0.0))
    ET.SubElement(inertial, "mass", {"value": fmt(mass)})
    ET.SubElement(inertial, "inertia", sphere_inertia(mass, radius))


def add_revolute_joint(
    robot: ET.Element,
    *,
    name: str,
    parent: str,
    child: str,
    origin: tuple[float, float, float],
    axis: tuple[float, float, float],
    lower_deg: float,
    upper_deg: float,
    effort: float,
    velocity: float,
    damping: float,
    friction: float,
) -> None:
    joint = ET.SubElement(robot, "joint", {"name": name, "type": "revolute"})
    ET.SubElement(joint, "parent", {"link": parent})
    ET.SubElement(joint, "child", {"link": child})
    add_origin(joint, origin)
    ET.SubElement(joint, "axis", {"xyz": xyz(axis)})
    ET.SubElement(
        joint,
        "limit",
        {
            "lower": fmt(math.radians(lower_deg)),
            "upper": fmt(math.radians(upper_deg)),
            "effort": fmt(effort),
            "velocity": fmt(velocity),
        },
    )
    ET.SubElement(
        joint,
        "dynamics",
        {"damping": fmt(damping), "friction": fmt(friction)},
    )


def add_fixed_joint(
    robot: ET.Element,
    *,
    name: str,
    parent: str,
    child: str,
    origin: tuple[float, float, float],
) -> None:
    joint = ET.SubElement(robot, "joint", {"name": name, "type": "fixed"})
    ET.SubElement(joint, "parent", {"link": parent})
    ET.SubElement(joint, "child", {"link": child})
    add_origin(joint, origin)


def load_parameters(path: Path) -> dict:
    parameters = json.loads(path.read_text(encoding="utf-8"))
    if parameters.get("schema_version") != 1:
        raise ValueError("unsupported parameter schema_version")
    for section in ("dimensions", "masses", "actuator", "joint_limits_deg"):
        if section not in parameters:
            raise ValueError(f"missing parameter section: {section}")
    for name, value in parameters["dimensions"].items():
        if not isinstance(value, (int, float)) or value <= 0:
            raise ValueError(f"dimension {name} must be positive")
    for name, value in parameters["masses"].items():
        if not isinstance(value, (int, float)) or value <= 0:
            raise ValueError(f"mass {name} must be positive")
    for joint, limits in parameters["joint_limits_deg"].items():
        if len(limits) != 2 or limits[0] >= limits[1]:
            raise ValueError(f"invalid {joint} limits")
    return parameters


def build_robot(parameters: dict) -> ET.Element:
    dims = parameters["dimensions"]
    masses = parameters["masses"]
    actuator = parameters["actuator"]
    limits = parameters["joint_limits_deg"]

    robot = ET.Element("robot", {"name": "spot_omg"})
    robot.append(
        ET.Comment(
            " Generated from spot_omg_parameters.json. "
            "Do not hand-edit this URDF; regenerate it. "
        )
    )
    add_material(robot, "body_dark", "0.08 0.10 0.13 1")
    add_material(robot, "leg_yellow", "0.95 0.72 0.05 1")
    add_material(robot, "foot_black", "0.03 0.03 0.03 1")

    body_size = (
        dims["body_visual_length"],
        dims["body_visual_width"],
        dims["body_visual_height"],
    )
    add_box_link(
        robot,
        name="base_link",
        size=body_size,
        mass=masses["body"],
        center=(0.0, 0.0, 0.0),
        material="body_dark",
    )

    # Sensor frame for one body-centred BNO086. The fixed frame intentionally
    # has no mass; its electronics mass belongs to base_link until measured.
    ET.SubElement(robot, "link", {"name": "imu_link"})
    add_fixed_joint(
        robot,
        name="imu_joint",
        parent="base_link",
        child="imu_link",
        origin=(0.0, 0.0, dims["body_visual_height"] / 2.0),
    )

    effort = actuator["rated_effort_nm"]
    velocity = actuator["no_load_velocity_rad_s"]
    damping = actuator["joint_damping_nm_s_rad"]
    friction = actuator["joint_friction_nm"]

    for leg, values in LEGS.items():
        side = values["side"]
        end = values["end"]
        servo_ids = values["servo_ids"]
        robot.append(
            ET.Comment(
                f" {leg.upper()}: J1/J2/J3 servo IDs "
                f"{servo_ids[0]}/{servo_ids[1]}/{servo_ids[2]} "
            )
        )

        hip_size = (
            dims["hip_link_size_x"],
            dims["hip_link_length"],
            dims["hip_link_size_z"],
        )
        add_box_link(
            robot,
            name=f"{leg}_hip_link",
            size=hip_size,
            mass=masses["hip_link"],
            center=(0.0, side * dims["hip_link_length"] / 2.0, 0.0),
            material="leg_yellow",
        )
        upper_size = (
            dims["upper_link_size_x"],
            dims["upper_link_size_y"],
            dims["upper_leg_length"],
        )
        add_box_link(
            robot,
            name=f"{leg}_upper_link",
            size=upper_size,
            mass=masses["upper_leg"],
            center=(0.0, 0.0, -dims["upper_leg_length"] / 2.0),
            material="leg_yellow",
        )
        lower_size = (
            dims["lower_link_size_x"],
            dims["lower_link_size_y"],
            dims["lower_leg_length"],
        )
        add_box_link(
            robot,
            name=f"{leg}_lower_link",
            size=lower_size,
            mass=masses["lower_leg"],
            center=(0.0, 0.0, -dims["lower_leg_length"] / 2.0),
            material="leg_yellow",
        )
        add_sphere_link(
            robot,
            name=f"{leg}_foot_link",
            radius=dims["foot_radius"],
            mass=masses["foot"],
            material="foot_black",
        )

        # Logical positive J1 always abducts (widens) the leg. J2/J3 use the
        # same pitch axes on every leg so all four knees flex toward -X
        # (rearward), matching the physical robot's knee-backward layout.
        add_revolute_joint(
            robot,
            name=f"{leg}_j1",
            parent="base_link",
            child=f"{leg}_hip_link",
            origin=(
                end * dims["body_joint_length"] / 2.0,
                side * dims["body_joint_width"] / 2.0,
                0.0,
            ),
            axis=(side, 0.0, 0.0),
            lower_deg=limits["j1"][0],
            upper_deg=limits["j1"][1],
            effort=effort,
            velocity=velocity,
            damping=damping,
            friction=friction,
        )
        add_revolute_joint(
            robot,
            name=f"{leg}_j2",
            parent=f"{leg}_hip_link",
            child=f"{leg}_upper_link",
            origin=(0.0, side * dims["hip_link_length"], 0.0),
            axis=(0.0, 1.0, 0.0),
            lower_deg=limits["j2"][0],
            upper_deg=limits["j2"][1],
            effort=effort,
            velocity=velocity,
            damping=damping,
            friction=friction,
        )
        add_revolute_joint(
            robot,
            name=f"{leg}_j3",
            parent=f"{leg}_upper_link",
            child=f"{leg}_lower_link",
            origin=(0.0, 0.0, -dims["upper_leg_length"]),
            axis=(0.0, -1.0, 0.0),
            lower_deg=limits["j3"][0],
            upper_deg=limits["j3"][1],
            effort=effort,
            velocity=velocity,
            damping=damping,
            friction=friction,
        )
        add_fixed_joint(
            robot,
            name=f"{leg}_foot_fixed",
            parent=f"{leg}_lower_link",
            child=f"{leg}_foot_link",
            origin=(0.0, 0.0, -dims["lower_leg_length"]),
        )

    return robot


def serialize(robot: ET.Element) -> str:
    ET.indent(robot, space="  ")
    xml = ET.tostring(robot, encoding="unicode", short_empty_elements=True)
    return '<?xml version="1.0" encoding="utf-8"?>\n' + xml + "\n"


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--parameters", type=Path, default=DEFAULT_PARAMETERS)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument(
        "--check",
        action="store_true",
        help="fail when the checked-in URDF differs from generated output",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    generated = serialize(build_robot(load_parameters(args.parameters)))
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
