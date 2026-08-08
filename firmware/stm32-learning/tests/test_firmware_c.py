"""Compile and run the firmware's host-testable C units.

safety.c and feetech_protocol.c carry no HAL or bus dependency on purpose, so
the decision logic that cuts torque can be exercised on a workstation rather
than only on the robot.  This wrapper exists so those tests run alongside the
Python suite instead of being a separate step someone has to remember.

    pytest tools/servo_tool/tests simulation/mujoco/test_trot2.py \
           firmware/stm32-learning/tests/test_firmware_c.py -q
"""

from __future__ import annotations

import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

import pytest

PROJECT = Path(__file__).resolve().parents[1]
CFLAGS = ["-std=c11", "-O1", "-Wall", "-Wextra", "-Werror", f"-I{PROJECT/'Inc'}"]

CASES = [
    ("safety", ["Src/safety.c", "tests/test_safety.c"]),
    ("feetech_protocol",
     ["Src/feetech_protocol.c", "tests/test_feetech_protocol.c"]),
]


LEGS = ("FL", "FR", "RL", "RR")
DIAGONALS = (("FL", "RR"), ("FR", "RL"))


def _firmware_gait_signs() -> dict[str, int]:
    """Read g_robot_gait_forward_signs straight out of robot_config.c."""
    source = (PROJECT / "Src/robot_config.c").read_text()
    match = re.search(
        r"g_robot_gait_forward_signs\[4\]\s*=\s*\{([^}]*)\}", source)
    assert match, "could not find g_robot_gait_forward_signs"
    values = [int(part.strip()) for part in match.group(1).split(",")]
    assert len(values) == 4
    return dict(zip(LEGS, values))


def test_gait_signs_keep_diagonal_legs_together() -> None:
    """The two legs of a diagonal must do the same thing at the same time.

    This is what a trot is, and it is checkable without a robot.  A sign
    combination that breaks it produced identical joint angles for the two legs
    on one side while the policy marked one in stance and the other in swing --
    impossible, since equal angles put both feet at the same height, and it
    took several bench runs to recognise.
    """
    sys.path.insert(0, str(PROJECT.parents[1] / "tools/servo_tool"))
    from servo import SharedGaitPolicy

    policy = SharedGaitPolicy()
    signs = _firmware_gait_signs()

    for step in range(20):
        phase = step / 20.0
        angles, stance = policy.trot2_targets(phase, 1.0, 78.0, 108.0, signs)
        for left, right in DIAGONALS:
            assert (left in stance) == (right in stance), (
                f"phase {phase}: {left} and {right} disagree on stance"
            )
            for joint in (1, 2, 3):
                assert abs(angles[(left, joint)] - angles[(right, joint)]) < 0.01, (
                    f"phase {phase}: {left} J{joint} != {right} J{joint}"
                )


@pytest.mark.parametrize("name,sources", CASES, ids=[case[0] for case in CASES])
def test_firmware_unit(name: str, sources: list[str]) -> None:
    compiler = shutil.which("cc") or shutil.which("gcc")
    if compiler is None:
        pytest.skip("no host C compiler available")

    with tempfile.TemporaryDirectory() as workdir:
        binary = Path(workdir) / name
        build = subprocess.run(
            [compiler, *CFLAGS, *[str(PROJECT / src) for src in sources],
             "-o", str(binary)],
            capture_output=True,
            text=True,
        )
        assert build.returncode == 0, f"compile failed:\n{build.stderr}"
        # -Werror above already fails the build on a warning; surface any that
        # slipped through as diagnostics rather than letting them pass silently.
        assert not build.stderr.strip(), f"unexpected diagnostics:\n{build.stderr}"

        run = subprocess.run([str(binary)], capture_output=True, text=True)
        assert run.returncode == 0, (
            f"{name} failed:\n{run.stdout}\n{run.stderr}"
        )
