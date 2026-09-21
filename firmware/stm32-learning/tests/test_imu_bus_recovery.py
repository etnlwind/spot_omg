from pathlib import Path
import subprocess
from servo.host_build import build_executable

def test_bus_recovery_actual_helper_with_hal_faults(tmp_path):
    root=Path(__file__).resolve().parents[1]
    exe=build_executable([root/'tests/test_imu_bus_recovery.c'],root/'Inc',tmp_path/'imu-recovery',extra=['-Wall','-Wextra','-Werror'])
    subprocess.run([str(exe)],check=True,capture_output=True,text=True)
