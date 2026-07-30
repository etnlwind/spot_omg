# Spot OMG Tools

Spot OMG 로봇을 설정하고 점검하기 위한 개발 도구 모음입니다.

## 도구 목록

### Servo Tool

[`servo_tool`](./servo_tool/README.md)은 STS3215 서보의 연결 상태를 확인하고
설정하거나 제어하는 Python 도구입니다.

주요 기능:

- 사용 가능한 시리얼 포트 조회
- 연결된 서보 검색
- 서보 ID 조회 및 변경
- 토크 활성화 및 비활성화
- 목표 위치 이동
- 위치, 속도, 부하, 전압, 온도 등의 상태 진단

## 빠른 시작

저장소 루트에서 다음 명령을 실행합니다.

```bash
cd tools/servo_tool
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

시리얼 포트와 연결된 서보를 확인합니다.

```bash
python examples/list_ports.py
python examples/scan.py /dev/ttyUSB0
```

서보를 중앙 위치로 이동합니다.

```bash
python examples/move.py /dev/ttyUSB0 2048 --id 1
```

포트 이름은 환경에 따라 달라집니다.

- Linux: `/dev/ttyUSB0`
- macOS: `/dev/cu.usbserial-*`
- Windows: `COM3`

자세한 사용법과 하드웨어 연결 주의사항은
[`servo_tool/README.md`](./servo_tool/README.md)를 참고하세요.

## 안전

- 서보 정격에 맞는 별도 전원을 사용하세요.
- USB-to-TTL 어댑터와 서보 전원의 GND를 공통으로 연결하세요.
- 서보 ID를 변경할 때는 충돌을 방지하기 위해 대상 서보 하나만 연결하세요.
- 로봇에 장착된 서보를 움직이기 전에 동작 범위와 기구적 간섭을 확인하세요.
