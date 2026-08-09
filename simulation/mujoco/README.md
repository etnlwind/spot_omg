# Spot OMG MuJoCo Preview

저장소 루트의 Conda `spot_omg` 환경에서 URDF를 Canonical Pose (논리 자세)로
확인합니다. Servo Tool과 시뮬레이션은 동일한 `environment.yml`을 사용합니다.

```bash
conda env create -f environment.yml
conda activate spot_omg

python simulation/mujoco/preview_pose.py stand45
```

`environment.yml`은 MuJoCo와 `tools/servo_tool` editable package를 함께
설치합니다. 이는 STM32 코드를 Python으로 바꾸는 것이 아니라 MuJoCo 호스트가
저장소의 공용 C 헤더와 Python 바인딩을 찾도록 하는 구성입니다. 로컬
`.venv-mujoco`는 사용하지 않습니다.

지원 자세는 실제 `spotctl`의 논리 각도와 같습니다.

| Pose | J1 | J2 | J3 |
|---|---:|---:|---:|
| `stand` | 0° | 0° | 0° |
| `stand45` | 0° | 45° | 90° |
| `landing` | 0° | 40° | 130° |

URDF는 Initial Joint Position (초기 관절 위치)을 저장하지 않으므로 파일을 직접
열면 항상 `stand`, 즉 모든 관절이 0°인 자세로 보입니다. 이 실행기는 관절 이름을
기준으로 원하는 각도를 넣은 뒤 MuJoCo Viewer (무조코 화면)를 엽니다.

현재 실행기는 Kinematic Preview (운동학 미리보기) 용도라 중력을 끕니다. 지면,
접촉, 위치 구동기와 STS3215 응답 모델은 별도의 동역학 scene에 추가합니다.

## 보행 궤적 재생

`sim-trot`은 STM32와 MuJoCo가 함께 사용하는 `gait_policy.h`의 HAL 독립 C
정책을 MuJoCo 12개 관절에 연결합니다. 호스트에서는 작은 공유 라이브러리를
임시 디렉터리에 자동 컴파일해 같은 C 함수를 직접 호출합니다. macOS의 실시간
Viewer는 `mjpython`으로 실행해야 합니다.

```bash
mjpython simulation/mujoco/walk.py \
  --gait trot --preset sim-trot --cycles 10
```

기존 `test`, `power`, `natural` 프리셋은 비교와 과거 시험 재현을 위해 Python
정책을 유지합니다. `--controller python`으로 `sim-trot`의 이전 Python 구현도
명시적으로 선택할 수 있습니다.

```bash
mjpython simulation/mujoco/walk.py \
  --gait trot --preset power --cycles 10 \
  --stance-j1 4 --stance-j2 25 --stance-j3 50 \
  --duty 0.82 --hip 8 --lift 20 --period 2.0 --rate 50
```

이 단계는 Kinematic Gait Replay (운동학 보행 재생)이므로 몸체를 고정한 채 다리
위상과 발 궤적을 비교합니다. 아직 지면 반력으로 몸체가 전진하는 Dynamic Walking
(동역학 보행)은 아닙니다.

## 지면 위 동역학 보행

URDF가 바뀌면 Dynamic Scene (동역학 장면)을 다시 생성합니다.

```bash
python simulation/mujoco/generate_scene.py
python simulation/mujoco/generate_scene.py --check
```

`--dynamic`을 추가하면 중력, 평면 지면, 발 마찰, Floating Base (자유 몸체),
12개 STS3215 Position Actuator (위치 구동기)가 활성화됩니다. `test` 프리셋은
4초 주기의 느린 Phase/Actuator Test (위상·구동기 시험)이며 실제 접촉 기준 트롯은
아닙니다.

```bash
mjpython simulation/mujoco/walk.py \
  --dynamic --gait trot --preset test --cycles 10
```

화면 없이 결과만 확인할 수도 있습니다.

```bash
python simulation/mujoco/walk.py \
  --dynamic --gait trot --preset test --cycles 10 --check
```

2026-08-05 초기 모델의 Headless Test (화면 없는 시험) 결과는 다음과 같습니다.

| 설정 | 결과 | 대각선 접촉 | 전방 X 이동 | 종료 몸체 높이 |
|---|---|---:|---:|---:|
| `test`, 10 cycles | non-trot contact | 11.8% | +0.429 m | 0.222 m |
| 하드웨어 시험용 `power`, 10 cycles | upright (직립 유지) | 미측정 | +0.157 m | 0.276 m |
| `natural`, 5 cycles | fallen (넘어짐) | 유효하지 않음 | 유효하지 않음 | 0.035 m |

`natural` 결과는 시뮬레이터 오류가 아니라 현재 Open-Loop Control (개루프 제어)이
질량·마찰 오차와 좌우 기울기를 복구하지 못한다는 진단입니다. IMU Feedback
(IMU 피드백)이나 더 넓은 J1 stance (J1 지지폭)를 적용하기 전에는 `test` 또는
검증된 `power` 설정으로 시작합니다.

현재 물리 초기값은 `2 ms` timestep, 발 미끄럼 마찰 `0.9`, STS3215 최대 토크
`2.942 Nm`, position gain `35`, velocity damping `0.8`입니다. 배터리 위치와
각 링크 질량이 확정되면 다시 맞춰야 합니다.

## 실제 접촉 기준 트롯

`sim-trot`은 STM32와 MuJoCo의 공용 Dynamic Trot (동적 트롯) 정책입니다.
`period=0.8 s`, `duty=0.50`, `lift=30°`, `rate=50 Hz`, `J1 stance=4°`를
사용합니다. 공용 C 함수는 같은 canonical 발끝 궤적과 IK를 계산하고, 전진 부호만
URDF에서는 네 다리 `-1`, 실제 STM32 기체에서는 `FL/FR +1`, `RL/RR -1`을
입력해 서로 다른 기구 배치를 보정합니다.

```bash
mjpython simulation/mujoco/walk.py \
  --dynamic --balance --gait trot --preset sim-trot --cycles 10
```

공용 C 정책의 10주기 Headless Test (화면 없는 시험) 결과는 대각선 접촉 `74.7%`, 전진
`+1.034 m`, 최대 Roll `7.61°`, 최대 Pitch `5.44°`, `state=UPRIGHT`,
`gait=TROT`입니다. 출력의 `diagonal_contact`는 실제 지면 접촉이 정확히
`FL+RR` 또는 `FR+RL`인 물리 프레임 비율이며, 50% 이상일 때 접촉 기준 트롯으로
판정합니다.

## 원형 발끝 궤적 `trot2`

`trot2`는 기존 `sim-trot`을 변경하지 않고 새 `gait_policy_trot2_targets()`를
MuJoCo와 STM32가 함께 호출하는 공용 C 프리셋입니다. 접지 중에는 발이 지면을 따라 앞에서 뒤로 이동하고, 스윙 중에는 L3
발끝이 상반원 궤적을 따라 `toe-off → 원 꼭대기 → touchdown`으로 이동합니다.
J2/J3를 따로 파형으로 움직이지 않고 매 frame 발끝 좌표를 IK로 풀기 때문에 L2도
연속적으로 접혔다가 다시 펴집니다.

기본 원 꼭대기 자세는 `J2=78°`, `J3=108°`입니다. J2 90°가 정확한 수평이므로
기본 L2는 몸체 수평보다 약 12° 아래까지 접힙니다. 먼저 몸체를 고정한 운동학
화면에서 궤적을 확인합니다.

```bash
conda activate spot_omg
mjpython simulation/mujoco/walk.py --preset trot2 --cycles 5
```

지면 접촉과 IMU 균형 보정을 포함한 동역학 시험은 다음과 같습니다.

```bash
mjpython simulation/mujoco/walk.py \
  --dynamic --balance --preset trot2 --cycles 10
```

원 꼭대기에서 L2를 더 수평에 가깝게 하거나 L3 접힘을 바꾸려면 다음 옵션을
사용합니다. 값이 커질수록 원과 보폭도 커질 수 있으므로 먼저 `--check`로
검증합니다.

```bash
python simulation/mujoco/walk.py \
  --dynamic --balance --preset trot2 --cycles 10 \
  --trot2-fold-j2 80 --trot2-fold-j3 110 --check
```

STM32 `balance full`과 같은 이득을 사용한 기본값의 10주기 결과는
`state=UPRIGHT`, `gait=TROT`, 대각선 접촉 `56.3%`, 전진 `+2.225m`,
최대 Roll `3.90°`, 최대 Pitch `2.47°`입니다. `--controller python`은 공용 C와
비교하기 위한 기준 구현이며 기본 `auto`는 `shared-c`를 선택합니다.

STM32에는 같은 정책을 사용하는 `trot2 [cycles [period_ms]]` 명령이 있습니다.
실기는 거치대에서 `profile 800 80`, `trot2 1 1600` 순서로 먼저 확인합니다.
`spotctl` 보행에는 포함하지 않았습니다.

`test_trot2.py`는 원 궤적, 대각선 동기화와 공용 C/Python 구현 일치를 확인하는
회귀 시험입니다.

### STS3215 속도 feasibility

공용 C 정책의 800ms 한 사이클을 실제 50Hz frame으로 샘플링해 관절 속도를
계산할 수 있습니다. motor limit은 policy가 아니라 STM32의
`motor_capability.h`에서 읽습니다.

```bash
conda run -n spot_omg python -m tools.servo_tool.servo.gait_analysis
```

현재 최대값은 `trot J3=585.7°/s`, `trot2 J3=278.2°/s`, 기본 1400ms
`trot3 J3=227.4°/s`입니다. `trot3`는 원형 발끝은 유지하되 duty 0.65로 네 발
지지 중첩을 추가하고, 그 뒤 actuator limiter를 거칩니다. `test_gait_velocity.py`가
기존 trot/trot2 canonical cycle hash, trot3 support mask, joint/phase별 속도와
정격 270°/s 대비 판정이 바뀌지 않는지 검사합니다.

```bash
pytest tools/servo_tool/tests simulation/mujoco/test_trot2.py -q
```

## 제자리 및 전진 점프

`jump.py`는 STM32 `jump`와 같은 `gait_policy.h` 공용 C 궤적을 50Hz로 실행합니다.
기본은 1200ms 제자리 점프 3회입니다.

```bash
python simulation/mujoco/jump.py --check
```

현재 제자리 기본값의 headless 결과는 몸체 시작 높이 약 `0.222m`, 최대 높이
`0.306m`, 실제 무접촉 frame 약 `28.7%`, `state=UPRIGHT`입니다. 전진 확장은
STM32에 적용하기 전에 다음처럼 작은 값부터 시뮬레이션합니다.

```bash
python simulation/mujoco/jump.py \
  --forward-travel 0.02 --cycles 3 --check
```

`forward-travel`은 정규화된 2-link 다리 좌표이며 허용 범위 `±0.30`은 수학적 IK
범위일 뿐 안전 권장값이 아닙니다.

## 몸체 수평 유지

기본 `--dynamic`은 Open-Loop Control (개루프 제어)이라 몸체 기울기를 관절
명령에 되먹임하지 않습니다. `--balance`를 추가하면 몸체 중앙 `imu_link`에 단
하나 설치한 Virtual IMU (가상 IMU)의 Orientation Quaternion (자세 쿼터니언)과
Gyroscope (자이로스코프)를 읽습니다. 이 값으로 네 다리의 J2/J3 IK 목표 높이를
연속적으로 차등 보정하고, 지면 접촉과 보행 위상을 이용해 J1의 역할을 나누는
Body Attitude Feedback (몸체 자세 피드백)이 활성화됩니다.

- Stance leg (지지 다리): 고정된 발을 통해 몸체를 기울기의 반대쪽으로 밉니다.
- Swing leg (스윙 다리, 내딛는 발): 넘어지는 쪽으로 발을 옮겨 다음 지지점을
  만듭니다.
- J2/J3: Position-controlled Servo (위치제어 서보)의 착지 목표가 끊기지 않도록
  스윙과 지지 구간 모두에서 연속적인 높이 보정을 유지합니다.

```bash
mjpython simulation/mujoco/walk.py \
  --dynamic --balance --gait trot --preset sim-trot --cycles 10
```

`sim-trot` 10주기 비교 결과입니다. 세 방식은 최종 `Kp=1.0`, `Kd=0.04`,
다리 길이 보정 제한 `0.15`를 기준으로 다시 측정했습니다.

| 자세 제어 | Max Roll | Max Pitch | Attitude RMS | Body Z range | 대각선 접촉 | 전방 X | 좌우 Y |
|---|---:|---:|---:|---:|---:|---:|---:|
| 없음 | 30.15° | 8.30° | 13.08° | 0.056m | 41.3% | +0.884m | +0.102m |
| J2/J3 전체 다리 보정 | 9.96° | 6.34° | 3.51° | 0.032m | 73.6% | +1.004m | -0.125m |
| 접촉 인식 J1 포함 | 7.61° | 5.44° | 3.03° | 0.031m | 74.7% | +1.034m | -0.042m |

### 제자리 트롯 보상

공용 C 트롯의 `travel-scale`은 리프트 높이와 대각선 위상을 유지하면서 전후 발끝
이동량만 조절합니다. `0`은 관절 궤적상 제자리지만 접촉 동역학에서는 10주기 동안
`X=-0.893m` 뒤로 밀렸습니다. 탐색 결과 `0.39`에서 `X=-0.014m`, `Y=+0.018m`,
대각선 접촉 `76.1%`, `state=UPRIGHT`로 순이동이 가장 작았습니다.

```bash
python simulation/mujoco/walk.py \
  --dynamic --balance --gait trot --preset sim-trot \
  --cycles 10 --travel-scale 0.39 --check
```

STM32 `trotplace` 기본값도 `0.39`이며 실제 바닥 마찰과 무게 중심에 맞춰
`ROBOT_TROT_IN_PLACE_TRAVEL_SCALE`을 다시 조정해야 합니다.

일반 프리셋의 기본 이득은 `Kp=0.6`, `Kd=0.04`, 다리 길이 보정 제한 `0.10`이며,
`sim-trot`/`trot2`는 각각 `1.0`, `0.04`, `0.15`를 사용합니다. `trot3`는 실기
full mode와 같은 `1.0`, `0.04`, 길이 제한 `0.08`, J1 이득 `15deg/rad`를
사용합니다. 다른 프리셋의 J1 Roll Compensation 이득은 `5.0`이고 모든 모드의
제한은 `5°`입니다. 필요하면
`--balance-kp`, `--balance-kd`, `--balance-limit`으로 변경할 수 있지만, 강한
이득은 발 접촉을 방해해 진행 방향을 바꿀 수 있으므로 기본값부터 사용합니다.

1400ms/5-cycle `trot3` 동역학 비교에서 기존 길이 `0.15`/J1 `5`는 Max Roll
`3.64°`, Max Pitch `2.38°`, RMS `0.98°`였고, 새 배분은 각각 `3.21°`, `1.87°`,
`0.85°`였습니다. +5° Roll impulse의 10ms 검증에서도 현재 J1 부호는 Roll을
`4.776°`로 줄였고, 무보정 `4.848°`, 반대 부호 `4.909°` 순이어서 실제 URDF 축과
접촉 기준으로 restoring 방향임을 확인했습니다.

Sagittal Foot Placement (전후 착지 위치 보정)는 구현되어 있지만 현재 질량 모델의
시험에서 Pitch와 좌우 표류를 증가시켜 기본 이득을 `0`으로 두었습니다. 배터리
위치와 링크 질량을 실측한 뒤 `--foot-placement-gain`으로 다시 조정합니다.

트롯은 두 대각선 발만 지지하는 구간이 있어 Roll/Pitch가 항상 `0°`일 수는
없습니다. 목표는 화면상 완전 고정이 아니라 기울기 진폭과 누적 Drift (표류)를
줄이면서 대각선 접촉을 유지하는 것입니다.

공용 C 정책은 Cartesian 발끝 궤적, 2-link IK, Roll/Pitch PD 다리 길이 보정과
J1 보정을 모두 계산합니다. 시뮬레이터는 MuJoCo Virtual IMU와 실제 접촉 다리
마스크를 전달합니다. STM32는 장착된 BNO086 Roll/Pitch와 수치 미분 각속도,
보행 stance 마스크를 같은 함수에 전달합니다. 실제 발 접촉 센서가 추가되면
STM32도 위상 마스크 대신 측정 접촉 마스크를 넣을 수 있습니다.

2026-08-05의 이득 탐색, 실패한 지지 다리 전용 보정, 실제 STS3215 정지·동적
부하 측정과 위상 기준 모델 결과는
[`HARDWARE_TEST_LOG.md`](../../tools/servo_tool/HARDWARE_TEST_LOG.md)에
수치와 함께 기록했습니다.

## 초기 실험 예제

MuJoCo 설치, 관절축, sit/stand 보간을 확인했던 초기 스크립트는
[`examples/`](./examples/)에 보관합니다. 현재 로봇 자세와 보행 검증에는 위의
`preview_pose.py`와 `walk.py`를 사용합니다.
