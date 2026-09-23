"""Execute the production IRQ transport and gait retry policy against a UART model."""
from pathlib import Path
import subprocess
import pytest
from servo.host_build import build_executable

ROOT=Path(__file__).resolve().parents[1]

@pytest.mark.parametrize('optimization',['-O0','-Os'])
def test_servo_async_fault_injection(tmp_path,optimization):
    source=(ROOT/'Src/robot.c').read_text()
    first=source.index('typedef struct {\n    uint32_t fresh[12],attempt_frame;')
    last=source.index('#if defined(__GNUC__) && !defined(__clang__)',first)
    (tmp_path/'gait_feedback_under_test.inc').write_text(source[first:last])
    files=['Src/servo_bus.c','Src/servo_bus_async.c','Src/feetech_protocol.c',
           'Src/sts3215.c','Src/robot_config.c','Src/safety.c','tests/test_servo_async.c']
    exe=build_executable([ROOT/f for f in files],ROOT/'Inc',tmp_path/'servo_async',
        extra=[optimization,'-UNDEBUG','-Wall','-Wextra','-Werror','-I'+str(tmp_path),
               '-include',str(ROOT/'tests/servo_bus_host_hal.h')])
    result=subprocess.run([str(exe)],capture_output=True,text=True,timeout=30)
    print(result.stdout)
    assert result.returncode==0,result.stdout+result.stderr


@pytest.mark.parametrize('optimization',['-O0','-Os'])
def test_sensor_optimization_contract(tmp_path,optimization):
    exe=build_executable([ROOT/'Src/bno055.c',ROOT/'tests/test_bno055_body.c'],
        ROOT/'Inc',tmp_path/'sensor',extra=[optimization,'-ffp-contract=off','-UNDEBUG',
        '-Wall','-Wextra','-Werror','-include',str(ROOT/'tests/bno055_host_hal.h')])
    for case in ('units','rotations','metadata','cache','failures','busy','legacy'):
        result=subprocess.run([str(exe),case],capture_output=True,text=True,timeout=10)
        assert result.returncode==0,result.stdout+result.stderr
