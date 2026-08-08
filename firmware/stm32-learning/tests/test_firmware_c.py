"""Compile and run the firmware's host-testable C units.

safety.c and feetech_protocol.c carry no HAL or bus dependency on purpose, so
the decision logic that cuts torque can be exercised on a workstation rather
than only on the robot.  This wrapper exists so those tests run alongside the
Python suite instead of being a separate step someone has to remember.

    pytest tools/servo_tool/tests simulation/mujoco/test_trot2.py \
           firmware/stm32-learning/tests/test_firmware_c.py -q
"""

from __future__ import annotations

import shutil
import subprocess
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
