# MuJoCo BLE와 iPhone 앱 V0.3.0 (6)

## 현재 상태

- iPhone에 **V0.3.0 (6) Debug** 설치 완료.
- 앱의 **실제 로봇 · BLE / 가상 로봇 · BLE / 가상 로봇 · TCP** 선택 구현.
- Mac의 `SpotOMG-Sim` BLE 광고와 iPhone → BLE → Mac → MuJoCo 왕복 동작 확인.
- iPhone 기록에서 `@D` 갱신, `@S` 정지, `$SPOTDRIVE stopped reason=requested`,
  `backend=sim`, `source=simulated` 전압 응답 확인. 사용자가 연동 정상 동작 확인.
- 앱 테스트 34개 통과. 실제 로봇 펌웨어를 새로 설치하지 않음.

## 실행 및 연결

프로젝트 루트에서 실행하면 MuJoCo 창과 Mac BLE 수신 앱이 함께 열린다.
처음에는 Mac의 Bluetooth 권한을 허용해야 한다.

```sh
PYTHONPATH=tools/servo_tool:simulation/mujoco /opt/anaconda3/envs/spot_omg/bin/mjpython simulation/mujoco/virtual_robot.py
```

iPhone 앱에서 **가상 로봇 · BLE → 선택한 대상 연결**을 누른다. IP 주소 입력은 필요 없다.
MuJoCo와 Mac BLE 수신 앱을 모두 실행한 상태로 둔다.
MuJoCo 종료 시 함께 실행한 BLE 수신 프로세스도 종료한다.
서버가 이미 실행 중일 때 같은 포트로 중복 실행하지 않는다.

TCP도 유지된다. Mac에서 사용하는 spotctl은 로컬 TCP로 연결하면 된다.
앱 BLE 세션과 spotctl TCP 세션은 같은 MuJoCo 제어 소유권을 사용하므로 한 번에 하나만 연결한다.

```sh
PYTHONPATH=tools/servo_tool /opt/anaconda3/envs/spot_omg/bin/python -m servo.cli --sim-host 127.0.0.1 --tcp-port 8765 console send syncstate
```

BLE 없는 실행은 `--no-ble`, GUI 없는 자동 테스트는 `--no-ble --headless`를 지정한다.
TCP를 iPhone Wi-Fi에서 쓰려면 별도로 `--host 0.0.0.0`을 지정한다.
BLE 기본 구성은 Mac 내부 `127.0.0.1:8765`에만 TCP를 바인딩한다.

## 대상 분리와 전송

| 항목 | 실제 | 가상 BLE |
|---|---|---|
| 장치 이름 | SpotOMG-Bridge | SpotOMG-Sim |
| 서비스 UUID 첫 부분 | 6e400001 | 6e400101 |
| RX UUID 첫 부분 | 6e400002 | 6e400102 |
| TX UUID 첫 부분 | 6e400003 | 6e400103 |

UUID의 공통 뒷부분은 `-b5a3-f393-e0a9-e50e24dcca9e`이다.
가상 앱 연결은 별도 서비스와 `backend=sim protocol=1` 식별 응답을 모두 요구한다.
Mac에서 장치 이름 광고가 생략되어도 서비스 UUID로 찾을 수 있다.
대상을 바꾸면 기존 연결과 입력 큐를 정리하며 실제 장치로 자동 대체 연결하지 않는다.

Mac 수신 앱도 내부 TCP 서버의 simulator 식별을 먼저 확인한다.
BLE 쓰기는 ACK와 순서 보장, TCP 쓰기는 완료 콜백 직렬 전송,
BLE 알림은 central의 MTU와 backpressure를 따른다.
상향 큐 16KiB, 하향 큐 64KiB를 넘으면 연결을 종료하여 오래된 입력이 쌓이지 않게 한다.
앱 구독 해제/연결 종료 시 TCP를 닫아 MuJoCo의 정지 전환을 유발한다.
MuJoCo 연결이 끊기면 `$SIMLINK disconnected`를 앱에 알리고 조작 가능 상태를 해제한다.

## 검증의 한계

이번 검증은 BLE/앱/시뮬레이터 연동 검증이다. 실제 로봇의 동역학 정확도를 보증하지 않는다.
물성은 여전히 추정값이며, 가상 IMU는 기울기 관측과 정지 판정에 사용한다.
STM32의 전체 자세 전환·IMU 보정·안전 제어와 동일하게 재호스팅된 상태는 아니다.
공유 범위는 [가상 로봇 문서](VIRTUAL-ROBOT-2026-09-09.md)를 참고한다.
