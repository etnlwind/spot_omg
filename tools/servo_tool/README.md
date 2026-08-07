# Spot OMG Servo Tool

STS3215 서보를 검색하고 설정하며 움직이기 위한 작은 Python 도구입니다.
기본 통신 속도는 `1,000,000 bps`, 위치 범위는 `0..4095`입니다.

2026-08-01~07의 실제 URT-2 연결, 보정, 무부하 트롯, STM32/MuJoCo 공용
보행·점프 정책과 IMU 시험 결과는
[`HARDWARE_TEST_LOG.md`](./HARDWARE_TEST_LOG.md)에 정리되어 있습니다.

## 설치

Servo Tool, 단위 시험과 MuJoCo는 저장소 루트의 Conda `spot_omg` 환경을 공유합니다.
최초 설치는 저장소 루트에서 실행합니다.

```bash
conda env create -f environment.yml
conda activate spot_omg
spotctl --help
```

환경 정의가 변경됐거나 로컬 패키지를 다시 동기화할 때는 다음을 실행합니다.

```bash
conda env update -f environment.yml --prune
```

별도의 `.venv` 설치 흐름은 사용하지 않습니다. `environment.yml`이 이 디렉터리를
editable package로 설치하므로 소스 수정은 즉시 `spotctl`에 반영됩니다.

## 빠른 명령

| 목적 | 명령 |
|---|---|
| 도움말 | `spotctl --help` |
| 시리얼 포트 확인 | `spotctl ports` |
| ID 1~12 검색 | `spotctl --port PORT scan --max-id 12` |
| 단일 서보 정밀 진단 | `spotctl --port PORT diagnose --id ID` |
| 서보 ID 변경 | `spotctl --port PORT change-id OLD_ID NEW_ID` |
| 장착된 두 서보 ID 교환 | `spotctl --port PORT swap-ids FIRST_ID SECOND_ID` |
| 관절 ID 배치 설정 | `spotctl configure-mapping` |
| 관절 중립점 보정 | `spotctl --port PORT calibrate` |
| 현재 자세를 중립점으로 저장 | `spotctl --port PORT capture-stand` |
| 선택한 다리만 중립점 저장 | `spotctl --port PORT capture-stand --leg RL` |
| 보행 방향 설정 | `spotctl configure-directions` |
| 전체 상태 확인 | `spotctl --port PORT status` |
| 모터 부하 기반 발 접촉 확인 | `spotctl --port PORT contacts` |
| 공중 보행 동적 부하 기록 | `spotctl --port PORT walk ... --profile-loads` |
| 보정된 중립 자세 | `spotctl --port PORT stand` |
| 보정값 기반 45도 자세 | `spotctl --port PORT stand45` |
| 보정값 기반 착지 자세 | `spotctl --port PORT landing` |
| 저장 포즈 적용 | `spotctl --port PORT pose NAME` |
| 현재 자세 저장 | `spotctl --port PORT save-pose NAME` |
| 전체 raw 2048 이동 | `spotctl --port PORT raw-center` |
| 대각선 트로트 시험 | `spotctl --port PORT walk --gait trot --preset test --cycles 1` |
| 전체 토크 해제 | `spotctl --port PORT relax` |
| 선택한 다리 토크 해제 | `spotctl --port PORT relax --leg RL` |
| 현재 위치에서 토크 유지 | `spotctl --port PORT hold [--leg RL]` |
| 선택한 다리만 중립 자세 | `spotctl --port PORT stand --leg RL` |

포트는 `--port`로 지정하거나 `SPOT_SERVO_PORT` 환경 변수에 저장할 수 있습니다.

현재 macOS 시험 장치에서는 다음처럼 한 번 지정하면 편리합니다.

```bash
export SPOT_SERVO_PORT=/dev/cu.usbmodem5B790788341
spotctl status
```

USB-to-TTL 어댑터는 STS3215의 half-duplex TTL 통신을 지원해야 합니다.
서보와 어댑터의 GND를 공통으로 연결하고, 서보 전원은 정격에 맞는 별도
전원을 사용하세요.

`spotctl`은 컴퓨터와 URT-2를 USB로 직접 연결했을 때 사용하는 Feetech raw bus
도구입니다. 컴퓨터가 ST-LINK USB를 통해 STM32에 연결된 구성에서는 STM32
USART2 콘솔의 `scan`, `read`, `stand`, `trot`, `trotplace`, `jump` 명령을
사용하며, 해당 포트에 `spotctl`을 실행하지 않습니다.

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

장착된 두 서보의 ID가 서로 뒤바뀐 경우에는 중간 임시 ID를 사용해 안전하게
교환할 수 있습니다. 세 ID가 버스에서 충돌하지 않는지 확인한 후 EEPROM을
순서대로 변경하고 결과를 다시 검사합니다.

```bash
spotctl --port /dev/ttyUSB0 swap-ids 9 12
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
spotctl --port /dev/cu.usbmodem5B790788341 capture-stand --leg RL
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
보행 제어기는 상·하단 링크 길이를 각각 1로 정규화한 planar 2-link IK를 사용합니다.
지지 구간에는 발끝 높이를 유지하면서 앞뒤로 이동하고, 스윙 구간에는 발끝을
위로 들어 이동합니다. 앞·뒤 다리의 기구학적 진행 차이는
`gait_forward_signs`로 분리합니다. 실제 관찰에 따라 FL/FR은 `-1`, RL/RR은
`+1`을 사용하며, 이 값은 자세용 모터 방향과 독립적입니다.

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

### 모터 부하 기반 발 접촉 감지

STS3215 Load Estimate (모터 부하 추정값)로 각 발의 접촉을 관찰할 수 있습니다.
이 명령은 모터 목표 위치와 Torque Enable (토크 활성화) 상태를 바꾸지 않습니다.
접촉력이 관절에 전달되려면 먼저 현재 위치에서 토크가 걸려 있어야 합니다.

```bash
spotctl --port /dev/cu.usbmodem5B790788341 hold
spotctl --port /dev/cu.usbmodem5B790788341 contacts \
  --baseline 2 --duration 15 --rate 10
```

`contacts` 실행 후 처음 2초 동안에는 몸체를 지지대에 두고 네 발 모두 지면에서
떨어뜨립니다. 이때 각 다리 J2/J3의 Unloaded baseline (무부하 기준값)을
측정합니다. `Monitoring` 문구가 출력된 뒤 한 발씩 손으로 눌러 접촉 판정을
확인합니다.

출력의 `ON`은 Estimated contact (추정 접촉), `off`는 비접촉이며 숫자는 해당
다리 J2/J3 중 큰 Baseline delta (기준 대비 부하 변화량)입니다. `--verbose`에서는
점수 뒤 괄호에 `(J2 delta,J3 delta)`도 표시합니다. 기본 판정은 2회 연속 `24`
이상일 때 ON, 3회 연속 `8` 이하일 때 OFF입니다. 이는 실제 힘의 단위가 아니라
STS3215 내부 추정값이므로 실측 결과에 맞춰 조정합니다.

2026-08-05 지지대 위 `stand45` 정지 시험에서는 무부하 변화가 거의 `0`, 발을
눌렀을 때 Peak score (최대 변화량)가 FL `32`, FR `36`, RL `56`, RR `52`로
관측되어 위 기본값을 선택했습니다. 이 값은 정지 접촉 감지용이며 보행 중에는
관절 가속과 기어 마찰에 의한 Dynamic load (동적 부하)를 따로 측정해야 합니다.

```bash
# 접촉이 약해 기본값 24에 도달하지 않을 때만 더 민감하게
spotctl --port PORT contacts --threshold 16 --release 4 --duration 15

# 모든 10 Hz 표본과 점수를 확인할 때
spotctl --port PORT contacts --duration 15 --verbose
```

Hysteresis (히스테리시스)와 Debounce (연속 표본 확인)를 사용하므로 임계값 주변의
작은 진동이 ON/OFF를 반복하는 것을 줄입니다. 모터 움직임과 기어 마찰도 부하로
나타나므로 이 단계는 접촉 관찰용이며 아직 실제 보행 피드백에는 연결하지 않습니다.

### 공중 보행 동적 부하 프로파일

정지 접촉 임계값은 실제 보행에 직접 사용하지 않습니다. 몸체를 지지대에 고정하고
발이 아무것에도 닿지 않는 상태에서 실제 트롯을 실행하면서 No-contact dynamic
baseline (공중 보행 동적 기준값)을 CSV로 기록합니다.

```bash
spotctl --port /dev/cu.usbmodem5B790788341 walk \
  --gait trot --preset power --cycles 10 \
  --period 2.0 --hip 8 --lift 20 \
  --speed 800 --accel 80 --rate 50 \
  --profile-loads --load-rate 5
```

파일은 기본적으로 `tools/servo_tool/logs/air_gait_loads_YYYYMMDD_HHMMSS.csv`에
생성됩니다. 위치를 지정하려면 `--profile-output PATH`를 사용하며, 이 옵션만으로도
프로파일 기록이 활성화됩니다.

각 행에는 다음 값이 기록됩니다.

- Global/leg phase (전체·다리별 보행 위상)와 진폭 Ramp (램프)
- Planned contact (보행 위상상 예정된 지지/스윙 상태)
- 목표 위치, 현재 위치, Position error (위치 오차)
- 속도, Load estimate (부하 추정), Current (전류), Moving 상태
- 모터 상태 읽기 시간과 Control-frame lateness (제어 프레임 지연)

50Hz Sync Write (동기 위치 전송)는 그대로 유지합니다. J2/J3 8개는 한 프레임에
최대 한 모터만 Round-robin sampling (순환 표본화)하며 기본적으로 모터당 5Hz로
읽습니다. 요청한 `--load-rate`가 `control rate / 8`보다 높으면 보행을 방해하지
않도록 자동으로 낮춥니다. 종료 출력의 `max read`와 `late frames`로 실제 URT-2
읽기가 보행 주기를 방해했는지 확인합니다.

기록이 끝나면 Phase-aligned baseline (위상 정렬 기준 모델)을 생성합니다. 이
명령은 시리얼 포트나 실제 로봇을 사용하지 않습니다.

```bash
spotctl analyze-loads tools/servo_tool/logs/air_gait_loads_YYYYMMDD_HHMMSS.csv
```

같은 이름의 `.baseline.json`이 생성되고 관절별 반복 잡음과 권장 Engage/Release
(접촉/해제) 임계값이 출력됩니다. 같은 공중 보행을 한 번 더 실행하며 모델의
False positive (오접촉 판정)를 검증할 수 있습니다.

```bash
spotctl --port /dev/cu.usbmodem5B790788341 walk \
  --gait trot --preset power --cycles 10 \
  --period 2.0 --hip 8 --lift 20 \
  --speed 800 --accel 80 --rate 50 \
  --load-rate 5 \
  --load-baseline tools/servo_tool/logs/air_gait_loads_YYYYMMDD_HHMMSS.baseline.json
```

두 번째 CSV에는 Expected load (위상별 예상 부하), Load residual (부하 잔차),
관절별 임계값과 초과 여부가 추가됩니다. 종료 시 다리별 `peak / threshold
exceedances`가 출력됩니다. 프로파일과 비교 보행의 자세·주기·보폭·리프트·속도·
가속도는 반드시 같아야 합니다. 이 검증도 관찰 전용이며 접촉 결과가 보행 목표를
바꾸지는 않습니다.

STS3215는 같은 동작에서도 Load direction (부하 방향 부호)이 바뀔 수 있으므로
모델은 Signed load (부호 부하)가 아니라 Absolute load magnitude (절대 부하
크기)를 사용합니다. 시작·종료 진폭 램프와 첫 Warm-up cycle (준비 1주기)은
접촉 판정에서 제외합니다.

2026-08-05 동일 조건의 두 번째 공중 보행 검증에서는 안정 구간 8주기 동안 다리당
160개 표본을 비교해 FL/FR/RL/RR 모두 임계값 초과 `0회`를 기록했습니다. 최대
잔차는 각각 `26`, `32`, `26`, `20`이었습니다.

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
위치의 차이를 검사합니다. 이동을 시작하기 전에 현재 위치와 목표 위치의 차이를
읽고, STS3215의 speed/acceleration profile (속도·가속도 프로파일)을 역산해
이동거리가 서로 다른 J2와 J3가 같은 시점에 도착하도록 관절별 속도를 지정합니다.
`--speed`는 가장 멀리 이동하는 관절의 speed cap (속도 상한)입니다. 기본 제한은
10초와 30 tick이며 필요하면 조절합니다.

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

실제 하중을 지지하면서 천천히 시험할 때는 power stance (고하중 자세)를 사용합니다.
이 프리셋은 J2 `30°`, J3 `60°`로 다리를 더 펴서 수직 하중에 대한 mechanical
advantage (기계적 이득)를 높이고, 보폭과 발 들기 높이는 줄입니다. 지지율은
`58%`로 두어 대각선 전환 전에 주기의 `16%` 동안 네 발이 함께 지면을 지지합니다.
이 구간에서 J1 weight shift (J1 하중 이동)와 stance preload (지지 다리 선행 하중)를
먼저 적용한 뒤 스윙 발을 듭니다. 완전히 편 다리는 특이점과 충격 흡수 문제가
있으므로 사용하지 않습니다.

```bash
spotctl --port PORT walk --gait trot --preset power --cycles 1
```

기본값은 `period=2.4`, `hip=10°`, `lift=16°`, `speed=800`, `accel=80`,
`rate=50Hz`, `stance-j1=4°`입니다. 고정 시험 후 실제 바닥에서는 먼저
1주기만 실행하세요.
자세와 지지율은 다음처럼 직접 조정할 수 있습니다.

```bash
spotctl --port PORT walk --preset power --cycles 1 \
  --stance-j1 4 --stance-j2 32 --stance-j3 64 --duty 0.60 \
  --weight-shift 1.5 --preload 1.5
```

`--stance-j1`은 J1 abduction (J1 외전) 논리 각도입니다. 양수값을 주면 네 다리
모두 동일한 `+` 각도를 사용합니다. 실제 장착 방향 차이는 저수준 `direction`에서
FL/RR `+1`, FR/RL `-1`로 변환합니다. 먼저 `2~5°` 범위에서 시험하세요.

`--weight-shift`는 대각선 지지 쌍에 먼저 적용하는 J1 각도입니다. 네 발이
모두 닿은 overlap (중복 지지) 구간에서 다음 지지 쌍으로 부드럽게 넘기고,
반대 대각선 발이 공중에 있는 동안 그 보정값을 유지합니다.
`--preload`는 지지 다리를 먼저 밀어 하중을 넘기는 degree-equivalent
(각도 상당값)입니다. IMU가 없는 open-loop 값이므로 둘 다 `1~2°`부터 조정하세요.

Swing trajectory (스윙 궤적)는 발을 먼저 완전히 든 뒤 앞으로 옮기고 마지막에
내리는 3단계 경로를 사용합니다. 따라서 `--lift`는 발을 앞으로 옮기는 동안
유지되는 clearance (지면 여유) 크기를 조절합니다.

기존 한 다리씩 움직이는 크롤 패턴도 유지됩니다.

```bash
spotctl --port /dev/cu.usbmodem5B790788341 walk \
  --gait crawl --preset test --cycles 1
```

### Cartesian IK 트롯 제어

트롯은 선택한 stance (지지 자세)를 기준으로 J1 체중 이동을 적용하고
`FL+RR`과 `FR+RL`을 정확히 반
주기 차이로 움직입니다. 각 발은 주기의 65% 동안 지면에 머물고 35% 동안만
스윙하므로, 대각선 쌍이 바뀌기 전에 주기의 15% 동안 네 발이 모두 접지합니다.
시작과 종료에서는 0.5초 동안 진폭을 부드럽게 증감합니다.

`--hip`은 전후 보폭, `--lift`는 스윙 높이를 degree 상당값으로 지정합니다.
두 값은 직접 관절에 더하지 않고 정규화된 발끝 좌표로 변환된 뒤 IK를 통해
J2와 J3에 함께 반영됩니다. 따라서 지지 발의 높이는 유지되고 스윙 발만 들립니다.
각 발은 몸체 좌표계에서 앞쪽 `\` 자세로 착지하고, 지지 구간에는 뒤쪽 `/`로
지면을 밀며, 발을 든 뒤 다시 앞쪽 `\`로 복귀합니다.

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

12관절 명령은 STS3215의 `Sync Write`를 사용해 같은 패킷에서 출발하며, 일반
포즈 전환은 이동거리와 가속도를 고려한 관절별 속도로 동시 도착을 맞춥니다.
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
- 현재 Conda `spot_omg` 환경에서 단위 테스트 63개와 diff 공백 검사가 통과했습니다.

세부 조건, 단계별 시간과 최종 조립 후 체크리스트는
[`HARDWARE_TEST_LOG.md`](./HARDWARE_TEST_LOG.md)를 참고하세요.
