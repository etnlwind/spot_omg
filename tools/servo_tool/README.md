# Spot OMG Servo Tool

STS3215 서보를 검색하고 설정하며 움직이기 위한 작은 Python 도구입니다.
기본 통신 속도는 `1,000,000 bps`, 위치 범위는 `0..4095`입니다.

## 설치

```bash
cd tools/servo_tool
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

USB-to-TTL 어댑터는 STS3215의 half-duplex TTL 통신을 지원해야 합니다.
서보와 어댑터의 GND를 공통으로 연결하고, 서보 전원은 정격에 맞는 별도
전원을 사용하세요.

## 사용

먼저 포트와 연결된 서보 ID를 확인합니다.

```bash
python examples/list_ports.py
python examples/scan.py /dev/ttyUSB0
python examples/read_id.py /dev/ttyUSB0 --id 1
python examples/diagnose_servo.py /dev/ttyUSB0 --id 1
```

토크와 위치를 제어합니다.

```bash
python examples/torque.py /dev/ttyUSB0 on --id 1
python examples/move.py /dev/ttyUSB0 2048 --id 1 --speed 1000
python examples/torque.py /dev/ttyUSB0 off --id 1
```

ID 변경은 충돌 방지를 위해 대상 서보 하나만 버스에 연결한 뒤 실행하세요.

```bash
python examples/change_id.py /dev/ttyUSB0 1 2 --yes
```

macOS에서는 포트가 보통 `/dev/cu.usbserial-*`, Windows에서는 `COM3` 같은
이름으로 나타납니다.

## Python API

```python
from servo import STS3215, ServoBus

with ServoBus("/dev/ttyUSB0") as bus:
    servo = STS3215(bus, 1)
    servo.enable_torque()
    servo.move(2048, speed=1000, acceleration=50)
    print(servo.read_state())
```

`config/joints.json`은 로봇 관절별 ID, 중앙값, 제한 범위와 방향을 기록하는
시작 예시입니다. 실제 조립 상태에 맞게 값을 수정하세요.
