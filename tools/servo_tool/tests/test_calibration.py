"""Guard the calibration table against silent drift.

The project is built in two layers.  gait_policy.h decides what the robot
does, in degrees, and the simulator and the firmware run that same code, so a
gait that works in MuJoCo is meant to work on the robot unchanged.  Below it,
robot_config.c does one job and no more: turn a policy angle into a servo tick
for the joint it is wired to.  Centre, direction, limits -- unit conversion and
mounting, nothing about behaviour.

That split is what makes these tests possible.  Because the layer only
translates, the simulator model settles questions the calibration table cannot
answer for itself: the MJCF says what a leg does at a given angle, so the
table's job is to reproduce that on real servos.  Where the table contradicts
the model, the table is wrong -- the alternative is behaviour leaking into a
layer that is supposed to be a unit conversion.

These values are measurements of the physical robot, so nothing here checks
whether a centre is *correct* -- only that the table stays internally
consistent, that its two copies agree, and that it still matches the model it
translates for.  A test that recomputed a centre from arithmetic would defeat
the point of measuring it.

Three things can go wrong without anyone noticing:

  * the firmware table and joints.json drift apart, because the same numbers
    are written out twice and only one copy gets edited;
  * a centre lands far from 2048, which servo_calibration.md documents as the
    reference the robot is posed at before calibrating;
  * a joint's direction stops matching the mounting rule.

Static inspection on 2026-08-09 confirmed the mounting rule: front and rear on
the same side are identical, while left and right are mirrored.
"""

import json
from pathlib import Path
import re
import unittest

ROOT = Path(__file__).resolve().parents[3]
JOINTS_JSON = ROOT / "tools" / "servo_tool" / "config" / "joints.json"
FIRMWARE = ROOT / "firmware" / "stm32-learning" / "Src"
FIRMWARE_TABLE = FIRMWARE / "robot_config.c"
SCENE = ROOT / "simulation" / "mujoco" / "spot_omg_scene.xml"

LEGS = ("FL", "FR", "RL", "RR")
LEG_INDEX = {"FL": 0, "FR": 1, "RL": 2, "RR": 3}

# How far a calibrated centre may sit from the 2048 reference.  Every measured
# offset is inside +/-200; this leaves room for a re-calibration without
# leaving room for a value that was computed rather than measured.
CENTRE_TOLERANCE = 300

FIRMWARE_ROW = re.compile(
    r"\{\s*(\d+)U,\s*(\d+)U,\s*(\d+)U,\s*(\d+)U,\s*"
    r"(\d+)U,\s*(\d+)U,\s*(-?\d+)\s*\}"
)


def load_joints_json():
    """Map (leg, joint number) to the record in joints.json."""
    document = json.loads(JOINTS_JSON.read_text())
    joints = {(row["leg"], row["joint"]): row for row in document["joints"]}
    return document, joints


def load_firmware_table():
    """Map (leg, joint number) to the row in g_robot_joints[]."""
    text = FIRMWARE_TABLE.read_text()
    start = text.index("g_robot_joints[ROBOT_JOINT_COUNT] = {")
    body = text[start:text.index("};", start)]

    rows = {}
    for match in FIRMWARE_ROW.finditer(body):
        servo_id, leg_index, joint, centre, low, high, direction = (
            int(group) for group in match.groups()
        )
        leg = LEGS[leg_index]
        rows[(leg, joint)] = {
            "id": servo_id,
            "center": centre,
            "min": low,
            "max": high,
            "direction": direction,
        }
    return rows


class CalibrationTableTest(unittest.TestCase):
    def setUp(self):
        self.document, self.json_joints = load_joints_json()
        self.firmware_joints = load_firmware_table()

    def test_both_copies_describe_the_same_twelve_joints(self):
        expected = {(leg, joint) for leg in LEGS for joint in (1, 2, 3)}
        self.assertEqual(set(self.json_joints), expected)
        self.assertEqual(set(self.firmware_joints), expected)

    def test_firmware_table_matches_joints_json(self):
        """It is generated from joints.json; keep the two copies together."""
        for key, expected in sorted(self.json_joints.items()):
            actual = self.firmware_joints[key]
            for field in ("id", "center", "min", "max", "direction"):
                self.assertEqual(
                    actual[field], expected[field],
                    f"{key[0]} J{key[1]} {field}: robot_config.c has "
                    f"{actual[field]}, joints.json has {expected[field]}",
                )

    def test_centres_stay_near_the_reference_position(self):
        reference = self.document["reference_center"]
        self.assertEqual(reference, 2048)
        for (leg, joint), row in sorted(self.json_joints.items()):
            distance = abs(row["center"] - reference)
            self.assertLessEqual(
                distance, CENTRE_TOLERANCE,
                f"{leg} J{joint} centre {row['center']} is {distance} ticks "
                f"from {reference}. The robot is posed at {reference} before "
                f"calibrating, so a measured centre lands close to it -- this "
                f"one looks computed, not measured.",
            )

    def test_offset_is_the_distance_from_the_reference(self):
        reference = self.document["reference_center"]
        for (leg, joint), row in sorted(self.json_joints.items()):
            self.assertEqual(
                row["offset"], row["center"] - reference,
                f"{leg} J{joint}: offset and centre disagree",
            )

    def test_zero_degrees_commands_the_calibrated_centre(self):
        """The straight-legged pose is the centres, whatever direction says."""
        for (leg, joint), row in sorted(self.json_joints.items()):
            target = row["center"] + row["direction"] * 0
            self.assertEqual(target, row["center"], f"{leg} J{joint}")


class LegSymmetryTest(unittest.TestCase):
    """Keep the motor signs established by the physical mounting rule.

    The policy hands every leg the same angle for the same job, and the MJCF
    turns that angle into the same motion on all four.  Anything that makes
    one leg move differently has to live in the translation layer, and the
    only thing there that can do it is direction.

    Every joint matches front-to-rear on the same side, while left and right
    use opposite signs.
    """

    def setUp(self):
        _, self.json_joints = load_joints_json()

    def directions(self, joint):
        return tuple(self.json_joints[(leg, joint)]["direction"]
                     for leg in LEGS)

    def mirrors(self, joint):
        front_left, front_right, rear_left, rear_right = self.directions(joint)
        return (front_left == rear_left
                and front_right == rear_right
                and front_left == -front_right)

    def test_directions_mirror_left_to_right(self):
        for joint in (1, 2, 3):
            with self.subTest(joint=joint):
                self.assertTrue(
                    self.mirrors(joint),
                    f"J{joint} directions {self.directions(joint)} over "
                    f"{LEGS} are not (a, -a, a, -a). The legs are identical, "
                    f"so front and rear cannot disagree. If this is a real "
                    f"measurement, document and verify it on the robot before "
                    f"changing this invariant.",
                )

class SimulatorPremiseTest(unittest.TestCase):
    """Keep the simulator's canonical geometry explicit.

    The MJCF gives every leg the same J2 and J3 axis, which is what makes
    canonical motion.  Motor direction remains a separate physical calibration.
    """

    def setUp(self):
        self.text = SCENE.read_text()

    def axis(self, joint_name):
        match = re.search(
            rf'<joint name="{joint_name}"[^>]*axis="([^"]+)"', self.text)
        self.assertIsNotNone(match, f"no joint {joint_name} in the scene")
        return match.group(1)

    def test_upper_and_knee_axes_are_identical_on_every_leg(self):
        for joint in (2, 3):
            axes = {leg: self.axis(f"{leg.lower()}_j{joint}") for leg in LEGS}
            self.assertEqual(
                len(set(axes.values())), 1,
                f"J{joint} axes differ between legs: {axes}",
            )

    def test_hip_axis_is_mirrored_left_to_right(self):
        left = {self.axis(f"{leg.lower()}_j1") for leg in ("FL", "RL")}
        right = {self.axis(f"{leg.lower()}_j1") for leg in ("FR", "RR")}
        self.assertEqual(len(left), 1, f"left hips disagree: {left}")
        self.assertEqual(len(right), 1, f"right hips disagree: {right}")
        self.assertNotEqual(left, right, "hip axes are not mirrored")


if __name__ == "__main__":
    unittest.main()
