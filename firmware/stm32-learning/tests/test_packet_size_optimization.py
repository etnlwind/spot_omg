"""Packet adapters remain equivalent under size optimization for the OTA slot."""
from pathlib import Path
import subprocess
import pytest
from servo.host_build import build_executable

ROOT=Path(__file__).resolve().parents[1]

@pytest.mark.parametrize('optimization',['-O0','-Os'])
def test_packet_adapter_optimization(tmp_path,optimization):
    paths=['Src/sts3215.c','Src/robot_config.c','Src/feetech_protocol.c','tests/test_stow_servo_coordinates.c']
    include=ROOT/'Inc';extra=['-include',str(ROOT/'tests/host_hal.h')]
    exe=build_executable([ROOT/p for p in paths],include,tmp_path/'coordinates',extra=[optimization,*extra])
    subprocess.run([str(exe)],check=True)
