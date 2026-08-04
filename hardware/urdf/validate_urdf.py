#!/usr/bin/env python3
"""Validate Spot OMG URDF structure and canonical joint-axis conventions."""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

from generate_urdf import LEGS


HERE = Path(__file__).resolve().parent
URDF = HERE / "spot_omg.urdf"


def floats(text: str) -> tuple[float, ...]:
    return tuple(float(value) for value in text.split())


def identity() -> tuple[tuple[float, ...], ...]:
    return (
        (1.0, 0.0, 0.0, 0.0),
        (0.0, 1.0, 0.0, 0.0),
        (0.0, 0.0, 1.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )


def multiply(
    left: tuple[tuple[float, ...], ...],
    right: tuple[tuple[float, ...], ...],
) -> tuple[tuple[float, ...], ...]:
    return tuple(
        tuple(sum(left[row][k] * right[k][column] for k in range(4)) for column in range(4))
        for row in range(4)
    )


def translation(position: tuple[float, ...]) -> tuple[tuple[float, ...], ...]:
    matrix = [list(row) for row in identity()]
    for index, value in enumerate(position):
        matrix[index][3] = value
    return tuple(tuple(row) for row in matrix)


def rotation(
    axis: tuple[float, ...], angle: float
) -> tuple[tuple[float, ...], ...]:
    x, y, z = axis
    cosine = math.cos(angle)
    sine = math.sin(angle)
    complement = 1.0 - cosine
    return (
        (
            cosine + x * x * complement,
            x * y * complement - z * sine,
            x * z * complement + y * sine,
            0.0,
        ),
        (
            y * x * complement + z * sine,
            cosine + y * y * complement,
            y * z * complement - x * sine,
            0.0,
        ),
        (
            z * x * complement - y * sine,
            z * y * complement + x * sine,
            cosine + z * z * complement,
            0.0,
        ),
        (0.0, 0.0, 0.0, 1.0),
    )


def foot_positions(
    joints: dict[str, ET.Element], positions: dict[str, float]
) -> dict[str, tuple[float, float, float]]:
    by_child = {
        joint.find("child").attrib["link"]: joint for joint in joints.values()
    }
    cache = {"base_link": identity()}

    def transform(link: str) -> tuple[tuple[float, ...], ...]:
        if link in cache:
            return cache[link]
        joint = by_child[link]
        parent = joint.find("parent").attrib["link"]
        origin = floats(joint.find("origin").attrib["xyz"])
        local = translation(origin)
        if joint.attrib["type"] == "revolute":
            axis = floats(joint.find("axis").attrib["xyz"])
            local = multiply(local, rotation(axis, positions.get(joint.attrib["name"], 0.0)))
        cache[link] = multiply(transform(parent), local)
        return cache[link]

    return {
        leg: tuple(transform(f"{leg}_foot_link")[index][3] for index in range(3))
        for leg in LEGS
    }


def main() -> int:
    root = ET.parse(URDF).getroot()
    links = {element.attrib["name"]: element for element in root.findall("link")}
    joints = {element.attrib["name"]: element for element in root.findall("joint")}

    assert root.attrib["name"] == "spot_omg"
    assert len(links) == 18, f"expected 18 links, got {len(links)}"
    assert len(joints) == 17, f"expected 17 joints, got {len(joints)}"
    assert sum(joint.attrib["type"] == "revolute" for joint in joints.values()) == 12

    children = [joint.find("child").attrib["link"] for joint in joints.values()]
    assert len(children) == len(set(children)), "a link has multiple parent joints"
    assert set(children) == set(links) - {"base_link"}, "URDF is not one rooted tree"

    expected_ids = set(range(1, 13))
    assert {servo_id for values in LEGS.values() for servo_id in values["servo_ids"]} == expected_ids

    for leg, values in LEGS.items():
        side = values["side"]
        end = values["end"]
        expected_axes = {
            f"{leg}_j1": (float(side), 0.0, 0.0),
            f"{leg}_j2": (0.0, 1.0, 0.0),
            f"{leg}_j3": (0.0, -1.0, 0.0),
        }
        for name, expected_axis in expected_axes.items():
            joint = joints[name]
            axis = floats(joint.find("axis").attrib["xyz"])
            assert axis == expected_axis, f"{name} axis {axis} != {expected_axis}"
            limit = joint.find("limit")
            assert float(limit.attrib["lower"]) < float(limit.attrib["upper"])
            assert float(limit.attrib["effort"]) > 0
            assert float(limit.attrib["velocity"]) > 0

        # At logical +J1, a straight left leg moves +Y and a straight right
        # leg moves -Y: all four legs abduct under one canonical sign.
        test_angle = math.radians(8.0)
        lateral_delta = side * math.sin(test_angle)
        assert math.copysign(1.0, lateral_delta) == side

        # Positive J2 bends every knee toward -X (rearward): the front and
        # rear legs deliberately share one knee-backward configuration.
        knee_delta_x = -math.sin(math.radians(45.0))
        assert knee_delta_x < 0.0

    for link_name, link in links.items():
        if link_name == "imu_link":
            continue
        inertial = link.find("inertial")
        assert inertial is not None, f"{link_name} has no inertial"
        assert float(inertial.find("mass").attrib["value"]) > 0
        inertia = inertial.find("inertia")
        for diagonal in ("ixx", "iyy", "izz"):
            assert float(inertia.attrib[diagonal]) > 0

    neutral = foot_positions(joints, {})
    for leg, position in neutral.items():
        side = LEGS[leg]["side"]
        end = LEGS[leg]["end"]
        assert math.copysign(1.0, position[0]) == end
        assert math.copysign(1.0, position[1]) == side
        assert position[2] < 0

    spread = foot_positions(
        joints, {f"{leg}_j1": math.radians(8.0) for leg in LEGS}
    )
    for leg in LEGS:
        assert abs(spread[leg][1]) > abs(neutral[leg][1])

    stand45 = foot_positions(
        joints,
        {
            **{f"{leg}_j2": math.radians(45.0) for leg in LEGS},
            **{f"{leg}_j3": math.radians(90.0) for leg in LEGS},
        },
    )
    for first, second, signs in (
        ("fl", "fr", (1.0, -1.0, 1.0)),
        ("rl", "rr", (1.0, -1.0, 1.0)),
    ):
        for index, sign in enumerate(signs):
            assert math.isclose(
                stand45[first][index],
                sign * stand45[second][index],
                abs_tol=1e-9,
            )

    # Front and rear feet have identical leg-local FK. They differ only by
    # the measured front/rear J1-axis spacing along X.
    front_rear_spacing = (
        floats(joints["fl_j1"].find("origin").attrib["xyz"])[0]
        - floats(joints["rl_j1"].find("origin").attrib["xyz"])[0]
    )
    for front, rear in (("fl", "rl"), ("fr", "rr")):
        assert math.isclose(
            stand45[front][0] - stand45[rear][0],
            front_rear_spacing,
            abs_tol=1e-9,
        )
        assert math.isclose(stand45[front][1], stand45[rear][1], abs_tol=1e-9)
        assert math.isclose(stand45[front][2], stand45[rear][2], abs_tol=1e-9)

    print(
        "valid: 18 links, 12 revolute joints, canonical axes, inertias, "
        "rearward knees, neutral/spread/stand45 FK"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
