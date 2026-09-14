"""Run the real new sensor reader against a HAL mock, with no physical I/O.

The mock checks timeout arguments and fail-fast ordering; it is not a hardware
I2C timing measurement or a validation of the installed gyro mounting axes.
"""
from pathlib import Path
import shutil
import subprocess
import pytest

PROJECT = Path(__file__).resolve().parents[1]


@pytest.fixture(scope="module")
def sensor_executable(tmp_path_factory):
    cc = shutil.which("cc")
    assert cc, "A C11 host compiler is required"
    output = tmp_path_factory.mktemp("bno-body") / "sensor"
    subprocess.run([
        cc, "-std=c11", "-O1", "-Wall", "-Wextra", "-Werror",
        "-fsanitize=undefined", "-fno-sanitize-recover=all",
        f"-I{PROJECT / 'Inc'}", "-include",
        str(PROJECT / "tests/bno055_host_hal.h"),
        str(PROJECT / "Src/bno055.c"),
        str(PROJECT / "tests/test_bno055_body.c"),
        "-lm", "-o", str(output),
    ], check=True, capture_output=True, text=True)
    return output


@pytest.mark.parametrize("case", [
    "units", "rotations", "metadata", "cache", "failures", "busy", "legacy",
])
def test_body_sensor_contract(sensor_executable, case):
    subprocess.run([str(sensor_executable), case], check=True,
                   capture_output=True, text=True)


def test_axis_verification_default_is_explicitly_unverified():
    source = (PROJECT / "Inc/bno_imu_config.h").read_text()
    assert "#define BNO055_BODY_AXES_VERIFIED 0" in source
