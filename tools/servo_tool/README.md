# Spot OMG Servo Tool

STS3215 서보를 검색하고 설정하며 움직이기 위한 작은 Python 도구입니다.
기본 통신 속도는 `1,000,000 bps`, 위치 범위는 `0..4095`입니다.

2026-08-01~02의 실제 URT-2 연결, 보정과 무부하 트롯 시험 결과는
[`HARDWARE_TEST_LOG.md`](./HARDWARE_TEST_LOG.md)에 정리되어 있습니다.

## 설치

macOS 또는 Linux에서는 준비 스크립트를 한 번 실행합니다.

```bash
cd tools/servo_tool
./setup.sh
```

스크립트는 `.venv` 폴더에 가상환경을 만들지만 터미널 프롬프트에는 `(somg)`로
표시되도록 설정합니다. 이후 새 터미널을 열 때마다 환경을 활성화합니다.

```bash
cd tools/servo_tool
source .venv/bin/activate
```

활성화되면 다음처럼 표시되고 `spotctl` 명령을 바로 사용할 수 있습니다.

```text
(somg) $
```

수동으로 설치해야 한다면 다음 명령을 사용합니다.

```bash
python3 -m venv --prompt somg .venv
source .venv/bin/activate
pip install -e .
```

## 빠른 명령

| 목적 | 명령 |
|---|---|
| 도움말 | `spotctl --help` |
| 시리얼 포트 확인 | `spotctl ports` |
| ID 1~12 검색 | `spotctl --port PORT scan --max-id 12` |
| 단일 서보 정밀 진단 | `spotctl --port PORT diagnose --id ID` |
| 서보 ID 변경 | `spotctl --port PORT change-id OLD_ID NEW_ID` |
| 관절 ID 배치 설정 | `spotctl configure-mapping` |
| 관절 중립점 보정 | `spotctl --port PORT calibrate` |
| 현재 자세를 중립점으로 저장 | `spotctl --port PORT capture-stand` |
| 보행 방향 설정 | `spotctl configure-directions` |
| 전체 상태 확인 | `spotctl --port PORT status` |
| 보정된 중립 자세 | `spotctl --port PORT stand` |
| 보정값 기반 45도 자세 | `spotctl --port PORT stand45` |
| 보정값 기반 착지 자세 | `spotctl --port PORT landing` |
| 저장 포즈 적용 | `spotctl --port PORT pose NAME` |
| 현재 자세 저장 | `spotctl --port PORT save-pose NAME` |
| 전체 raw 2048 이동 | `spotctl --port PORT raw-center` |
| 대각선 트로트 시험 | `spotctl --port PORT walk --gait trot --preset test --cycles 1` |
| 전체 토크 해제 | `spotctl --port PORT relax` |

포트는 `--port`로 지정하거나 `SPOT_SERVO_PORT` 환경 변수에 저장할 수 있습니다.

현재 macOS 시험 장치에서는 다음처럼 한 번 지정하면 편리합니다.

```bash
export SPOT_SERVO_PORT=/dev/cu.usbmodem5B790788341
spotctl status
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
명령은 전체 ID 범위를 검색해 지정한 기존 ID 하나만 응답하는지 확인한 후
EEPROM을 변경합니다. 변경 직후와 포트 재연결 후에 새 ID가 응답하고 이전 ID가
사라졌는지 다시 확인합니다.

```bash
spotctl --port /dev/ttyUSB0 change-id 1 2
```

macOS에서는 포트가 보통 `/dev/cu.usbserial-*`, Windows에서는 `COM3` 같은
이름으로 나타납니다.

## 조립 및 보정 작업

새로 조립했거나 서보 ID 배치가 바뀌었다면 다리별 ID를 J1, J2, J3 순서로
등록합니다. 이 명령은 서보를 움직이지 않습니다.

```bash
spotctl configure-mapping
```

중립점 보정은 12개 서보의 연결을 먼저 확인한 후 토크를 활성화합니다.

```bash
spotctl --port /dev/cu.usbmodem5B790788341 calibrate
```

서보혼을 손으로 정렬한 현재 자세를 그대로 `stand` 중앙값으로 저장하려면 다음을
실행합니다. `capture-stand`는 모터를 움직이거나 토크를 켜지 않고 현재 위치를
읽어 각 관절의 `center`, `offset`, `neutral` 포즈에 함께 저장합니다.

```bash
spotctl --port /dev/cu.usbmodem5B790788341 capture-stand
```

`save-stand`도 같은 명령의 별칭으로 사용할 수 있습니다. 저장 후 `calibrate`로
필요한 관절만 미세 조정하세요.

보정 화면에서 사용할 수 있는 명령은 다음과 같습니다.

```text
leg FL       조정할 다리 선택
joint 1      조정할 관절 선택
+1 / -1      1 tick 조정
+5 / -5      5 tick 조정
+10 / -10    10 tick 조정
zero         해당 관절 offset을 0으로 복원
show         전체 보정값과 현재 위치 확인
quit         종료
```

성공한 조정은 매번 `config/joints.json`에 자동 저장됩니다. 보행 방향도 대화형
명령으로 설정할 수 있습니다.

```bash
spotctl configure-directions
```

## 관절 좌표계

애플리케이션과 시뮬레이터는 네 다리에 같은 의미의 관절각(degree)을 사용합니다.
모터마다 다른 장착 방향은 각 관절의 `direction` 하나로만 관리하며, 모든 포즈와
보행이 같은 변환을 사용합니다.

```text
raw = center + direction * round(angle_deg * 4096 / 360)
```

전진·후진은 모터 `direction`을 바꾸지 않고 보행 궤적의 진행 부호로 결정합니다.
현재 제어기는 Cartesian IK 전 단계의 관절 파형 방식이므로 앞·뒤 다리의 기구학적
차이는 `gait_forward_signs`로 분리합니다. 실제 관찰에 따라 FL/FR은 `-1`,
RL/RR은 `+1`을 사용하며, 이 값은 자세용 모터 방향과 독립적입니다.

예를 들어 네 다리 J3에 모두 같은 각도를 명령해도 하드웨어 raw 값은 각 모터의
`center`와 `direction`에 따라 서로 다르게 계산됩니다. `stand45`는 11자 중립
자세에서 J2를 `+45°`, 상대 관절인 J3를 `+90°`로 접어 위·아래 링크가 각각
반대쪽 45°를 향하는 `>` 형태를 만듭니다. `walk --hip`, `--lift`, `--crouch`도
모두 degree 단위입니다.

Python에서 같은 변환을 사용할 수 있습니다.

```python
target = config.angle_to_position("FL", 3, 90.0)
angle = config.position_to_angle("FL", 3, target)
targets = config.angles_to_targets({("FL", 3): 45.0})
```

JSON을 저장할 때 사람이 확인하기 쉬운 `config/servo_calibration.md`,
`config/gait_config.md`, `config/servo_poses.md`도 함께 갱신됩니다.

`raw-center`는 보정값을 무시하고 모든 서보를 정확히 2048로 이동합니다.
기구적으로 안전한 상태에서 초기 조립을 확인할 때만 사용하세요.

```bash
spotctl --port /dev/cu.usbmodem5B790788341 raw-center
```

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
설정 파일입니다. 현재 값은 2026-08-01 URT-2 하드웨어 테스트에서 확인한
12개 서보의 배치, 보정값, 보행 방향과 `landing`/`stand45` 포즈를 반영합니다.
다른 기구 조립 상태에서는 움직이기 전에 반드시 값을 다시 검증하세요.

## 12관절 하드웨어 테스트

먼저 로봇을 들어 다리가 바닥이나 프레임에 닿지 않게 한 뒤 상태를 읽습니다.

```bash
spotctl ports
spotctl --port /dev/cu.usbmodem5B790788341 status
```

다음 명령은 실행 즉시 로봇 상태를 변경합니다.

```bash
spotctl --port /dev/cu.usbmodem5B790788341 stand
spotctl --port /dev/cu.usbmodem5B790788341 stand45
spotctl --port /dev/cu.usbmodem5B790788341 landing
spotctl --port /dev/cu.usbmodem5B790788341 relax
```

현재 자세를 새 이름으로 저장한 뒤 같은 `pose` 명령으로 다시 적용할 수 있습니다.

```bash
spotctl --port /dev/cu.usbmodem5B790788341 save-pose crouch
spotctl --port /dev/cu.usbmodem5B790788341 pose crouch
```

`stand45` 명령과 보행 기준 자세는 현재 중립 보정값에 canonical J2 `+45°`,
J3 `+90°`를 적용해 매번 계산합니다. `pose landing`은 다리를 더 낮게 눕히는
하단 링크가 바닥과 수평이 되도록 J2 `+40°`, J3 `+130°`를 최신 보정값에서
동적으로 계산합니다. 두 상대 관절각의 차이는 `90°`입니다. 과거에
저장된 동명의 raw 포즈는 실행하지 않습니다. `neutral`, `stand`, `stand45`,
`landing` 이름은 일반 `save-pose`로 덮어쓸 수 없습니다.

```bash
spotctl --port PORT landing --speed 100 --accel 5
```

기존 `spotctl pose landing`도 같은 동적 자세를 적용하는 호환 명령입니다.

포즈 이동은 모든 서보의 `Moving` 상태가 끝날 때까지 기다린 후 목표 위치와 실제
위치의 차이를 검사합니다. 기본 제한은 10초와 30 tick이며 필요하면 조절합니다.

```bash
spotctl --port PORT pose stand45 --timeout 15 --tolerance 50
```

보행 기본값은 대각선 트로트입니다. `FL+RR`과 `FR+RL`을 각각 같은 위상으로
묶고 두 그룹을 반 주기 차이로 움직입니다. 먼저 진폭과 속도가 작은 `test`
프리셋으로 확인하세요.

```bash
spotctl --port /dev/cu.usbmodem5B790788341 walk \
  --gait trot --preset test --cycles 1
```

더 큰 동작 범위를 사용하는 자연 보행 프리셋은 다음과 같이 실행합니다.

```bash
spotctl --port /dev/cu.usbmodem5B790788341 walk \
  --gait trot --preset natural --cycles 1
```

기존 한 다리씩 움직이는 크롤 패턴도 유지됩니다.

```bash
spotctl --port /dev/cu.usbmodem5B790788341 walk \
  --gait crawl --preset test --cycles 1
```

### 단순 트롯 제어

트롯은 `stand45`를 기준으로 J1을 고정하고 `FL+RR`과 `FR+RL`을 정확히 반
주기 차이로 움직입니다. 각 발은 주기의 65% 동안 지면에 머물고 35% 동안만
스윙하므로, 대각선 쌍이 바뀌기 전에 주기의 15% 동안 네 발이 모두 접지합니다.
시작과 종료에서는 0.5초 동안 진폭을 부드럽게 증감합니다.

- 시작할 때 speed와 acceleration을 한 번 설정
- 보행 중에는 기본 30Hz로 12개 Goal Position만 Sync Write
- 상태 읽기, Moving 확인, 목표 도착 대기 없음
- 종료 시 `stand45` 목표를 한 번 전송

주기가 2초이면 각 대각선 쌍의 스윙은 0.7초, 다음 쌍이 들리기 전 네 발 접지는
0.3초입니다.

공중에 고정한 무부하 고속 시험에서는 주기를 최소 0.6초까지 지정할 수 있습니다.
하단 링크와 발을 조립한 뒤에는 1.8~2.0초부터 다시 확인하세요.

```bash
spotctl --port PORT walk --gait trot --preset test --cycles 1
```

현재 공중 고정 무부하 시험에서 움직임과 실제 관절 이동 폭이 모두 양호했던 비교
기준은 다음과 같습니다.

```bash
spotctl walk --gait trot --preset natural --cycles 10 \
  --hip 24.6 --lift 35.2 \
  --period 1.0 --speed 1000 --accel 100 --rate 100
```

속도 비교는 나머지 값을 고정하고 `--period`만 바꿉니다.

```bash
# 빠른 무부하 후보
spotctl walk --preset natural --cycles 10 --hip 24.6 --lift 35.2 \
  --period 0.8 --speed 1000 --accel 100 --rate 100

# 중간값
spotctl walk --preset natural --cycles 10 --hip 24.6 --lift 35.2 \
  --period 0.9 --speed 1000 --accel 100 --rate 100

# 보폭과 속도의 균형값
spotctl walk --preset natural --cycles 10 --hip 24.6 --lift 35.2 \
  --period 1.0 --speed 1000 --accel 100 --rate 100
```

`period=0.6`도 무부하에서 실행됐지만 목표 위치를 모두 따라가기 전에 다음
명령이 들어가 실제 이동 폭이 짧아 보였습니다. 현재 무부하 실용 후보는
0.8~1.0초입니다. 실제 하중이 걸린 보행값으로 해석하면 안 됩니다.

시작 직후 짧게 빠르게 움직이는 구간은 현재 위치에서 `stand45`로 이동하는 준비
동작입니다. 이 이동에는 지정한 speed와 acceleration이 바로 적용되고, 이후
트롯 템포는 speed가 아니라 period로 결정됩니다.

포트를 매번 입력하지 않으려면 셸에서 한 번 지정합니다.

```bash
export SPOT_SERVO_PORT=/dev/cu.usbmodem5B790788341
spotctl status
spotctl scan --min-id 1 --max-id 12
spotctl pose stand45
```

기존 `python examples/spot.py ...` 진입점도 호환용으로 남아 있지만 새 사용법은
`spotctl`을 기준으로 합니다.

12관절 명령은 STS3215의 `Sync Write`를 사용해 같은 패킷에서 적용됩니다.
보행 도중 `Ctrl+C`를 누르면 계산된 `stand45` 포즈 복귀 명령을
전송합니다. 통신이나 전원이 끊긴 경우에는 이 복귀가 보장되지 않으므로 실제
전원 차단 수단도 손이 닿는 곳에 두세요.

`status`에는 위치, 속도, 부하, 전압, 온도, 전류, 이동 상태와 STS3215 주소
65의 하드웨어 오류 값이 함께 표시됩니다. `HW` 값이 `0`이 아니면 보행 전에
전압·온도·과부하 상태를 점검하세요.

## 시험 결과 요약

- 공중 고정 상태에서 period 2.0, 1.8, 1.6, 1.4초를 각 3주기 실행했습니다.
- period 1.2, 1.0, 0.8, 0.6초를 각 10주기 실행했습니다.
- 0.8초와 1.0초는 무부하에서 움직임과 이동 범위가 양호해 보였습니다.
- 0.6초는 동작했지만 실제 이동 폭이 줄어드는 현상이 보였습니다.
- FL이 한 차례 느리고 작게 보였으나 설정과 EEPROM은 다른 J2/J3와 같았고 이후
  같은 조건에서 재현되지 않았습니다.
- RR J1(ID 10)에서 오류 값 `8`이 간헐적으로 검출됐습니다.
- 단위 테스트 21개, Python 문법 검사와 diff 공백 검사가 통과했습니다.

세부 조건, 단계별 시간과 최종 조립 후 체크리스트는
[`HARDWARE_TEST_LOG.md`](./HARDWARE_TEST_LOG.md)를 참고하세요.
