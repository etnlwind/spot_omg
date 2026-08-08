# Spot OMG URT-2 하드웨어 시험 기록

이 문서는 2026-08-01부터 2026-08-06까지 Feetech URT-2와 STS3215 서보
12개로 확인한 설정, 보행·시뮬레이션·부하 시험과 판단을 정리한 기록입니다.

## 시험 범위와 현재 한계

- 로봇 몸체는 고정되어 있으며 다리는 공중에서 움직였습니다.
- 다리 하단 부품과 발은 아직 출력 중이므로 지면 접촉 시험을 하지 않았습니다.
- 따라서 현재 결과는 모터 연결, 관절 방향, 대각선 동기, 무부하 이동 범위와
  궤적의 매끄러움에 대한 결과입니다.
- 실제 보폭, 미끄러짐, 하중 지지와 균형은 최종 조립 후 다시 검증해야 합니다.
- 하단 링크가 장착되면 관성과 부하가 증가하므로 무부하 최고속도를 그대로
  사용하지 않습니다.

## 하드웨어와 통신

| 항목 | 확인값 |
|---|---|
| 서보 | Feetech STS3215, 12개 |
| 어댑터 | Feetech URT-2 |
| 포트 | `/dev/cu.usbmodem5B790788341` |
| 통신 속도 | 1,000,000 bps |
| 위치 범위 | 0~4095, 총 4096 tick |
| 중앙 기준 | 2048 |
| ID | 1~12 |

정지 상태에서 12개 서보가 모두 응답했습니다. 관찰된 전압은 11.7~12.0V,
온도는 약 29~40°C였습니다. RR J1인 ID 10에서 하드웨어 오류 값 `8`이
간헐적으로 검출됐다가 재조회 시 사라졌습니다. 반복될 경우 전원, 배선과
기계적 부하를 우선 확인해야 합니다.

## 관절 배치와 방향

| Leg | J1 | J2 | J3 | Trot pair |
|---|---:|---:|---:|---|
| FL | 1 | 2 | 3 | A |
| FR | 4 | 5 | 6 | B |
| RL | 7 | 8 | 9 | B |
| RR | 10 | 11 | 12 | A |

- Pair A: FL + RR
- Pair B: FR + RL
- Pair A와 B는 반 주기 차이로 움직입니다.
- J1은 현재 트롯에서 고정하고 J2가 전후 이동, J3가 들기 동작을 담당합니다.
- 정확한 중심값과 방향은 `config/servo_calibration.md`와
  `config/gait_config.md`를 기준으로 합니다.

## 최종 트롯 방식

초기의 8개 키프레임 방식과 모든 프레임에서 속도·가속도를 다시 쓰는 방식은
끊김이 커서 사용하지 않습니다. 현재 구현은 다음과 같습니다.

1. 계산된 `stand45` 목표와 speed/acceleration을 한 번 전송합니다.
2. 토크를 켜고 1초 동안 기준 자세 이동 시간을 둡니다.
3. 보행 중에는 지정한 rate로 12개 Goal Position만 Sync Write합니다.
4. FL+RR과 FR+RL은 반 주기 차이의 intermittent trot으로 움직입니다.
5. 각 발의 지지율은 65%, 스윙 비율은 35%입니다.
6. 대각선 쌍 전환 전에 주기의 15% 동안 네 발이 모두 접지하도록 계산합니다.
7. 보행 루프에서는 상태 읽기나 도착 대기를 하지 않습니다.
8. 종료 또는 Ctrl+C 시 `stand45` 목표를 다시 전송합니다.

주기 `T`에 대한 타이밍은 다음과 같습니다.

```text
대각선 쌍 스윙 = 0.35 × T
다음 쌍 이륙 전 4발 접지 = 0.15 × T
```

## 프리셋

| Preset | Period | Hip (tick) | Lift (tick) | Crouch (tick) | Speed | Accel | Rate |
|---|---:|---:|---:|---:|---:|---:|---:|
| test | 4.0s | 100 | 140 | 0 | 60 | 30 | 30Hz |
| natural | 2.4s | 220 | 340 | 70 | 60 | 25 | 30Hz |

고속 무부하 시험에서는 프리셋을 기준으로 다음 값을 명시적으로 덮어썼습니다.

```text
hip=280 tick, lift=400 tick, speed=1000, accel=100, rate=100Hz
```

`speed`는 보행 주기가 아니라 서보의 속도 상한입니다. 실제 동작 템포는
`period`가 결정합니다. 지나치게 짧은 period에서는 모터가 목표 진폭에 도달하기
전에 다음 명령을 받아 실제 이동 범위가 작아질 수 있습니다.

## 2026-08-01 설정 반영

- 원본 압축 파일의 ID 배치, 중심 보정, 방향과 저장 포즈를 반영했습니다.
- 최초 설정은 FL J1 중심 2068, 나머지 관절 중심 2048이었습니다.
- `landing`과 직접 저장한 `stand45` 포즈를 보존했습니다.
- ID 변경은 EEPROM unlock → ID 쓰기 → lock → 재연결 검증 순서로 보완했습니다.
- `spotctl` 명령과 당시의 `(somg)` venv 설치 흐름을 추가했습니다. 현재 개발
  환경은 아래 2026-08-08 기록의 Conda `spot_omg`로 대체했습니다.

## 2026-08-02 12관절 캘리브레이션 및 좌표 계층 (구 좌표계 기록)

> 이 절은 조립 중 사용한 임시 설정 자세의 이력입니다. 2026-08-03부터는 아래의
> canonical joint coordinate (표준 관절 좌표계)를 사용하므로
> `setup-j2-minus90` 기준과 tick 단위 논리값은 현재 명령에 적용하지 않습니다.

- 몸체를 지지대에 고정하고 토크를 해제한 뒤, 네 다리가 뒤로 수평한 11자 자세가
  되도록 손으로 맞췄습니다.
- 현재 위치를 `setup-j2-minus90` 기준으로 캡처해 12개 중립점을 갱신했습니다.
- 이 자세는 J1/J3가 0°, J2가 -90°인 설정용 자세이며 논리 원점이 아닙니다.
- J2 -90° 위치에서 1024 tick을 역산한 다리 아래쪽 직선 자세를 논리값 0으로
  사용합니다.
- 현재 중심과 오프셋은 `config/servo_calibration.md`를 기준으로 합니다.
- 포즈와 보행은 관절별 중립을 논리값 0으로 사용합니다.
- 하드웨어 경계에서만 `raw = center + direction × logical`을 계산합니다.
- `raw-center`, 진단, ID 변경과 캘리브레이션만 raw 위치를 직접 사용합니다.
- 생성형 `stand45`는 모든 다리에서 J2 -45°, 상대 관절인 J3 +90°로 계산합니다.
- 같은 `<` 모양에는 네 다리 모두 같은 논리 데이터 `(J1, J2, J3) =
  (0, -512, +1024)`를 사용합니다. 모터별 회전 부호는 `direction`에만 저장합니다.
- 실제 조립 상태에서 ID 3·6의 회전 방향이 반대임을 확인해 J3 변환 부호에
  반영했습니다. ID 8·11은 캘리브레이션 변경 없이 J2 목표를 -45°로 복원했습니다.
- J1 `+20°` 시험에서 FL·RR은 벌어지고 FR·RL은 좁혀지는 것을 확인했습니다.
  J1의 `+`를 네 다리 모두 외전으로 통일하기 위해 ID 4·7의 `direction`을
  반전했습니다.
- J3 디바이스 방향을 보정한 뒤에도 기존 `gait_directions`가 다시 부호를
  적용해 앞다리는 펴지고 뒷다리는 굽는 중복 보정이 확인됐습니다. 이를 제거해
  네 다리 모두 같은 J2/J3 논리 궤적을 사용하고 대각선 쌍은 위상만 다르게 했습니다.
- 수정 후 뒷다리는 정상이나 앞다리가 뒤로 걷는 것처럼 보이는 현상을 확인했습니다.
  동일한 전후 발 궤적을 만들도록 기구학 계층에서 front J2 변환 부호를 반전하고,
  디바이스 방향·캘리브레이션·정지 포즈는 유지했습니다.

## 2026-08-03 표준 관절 좌표계와 병합 결과

- 네 다리는 같은 canonical angle (표준 관절 각도) 데이터를 사용합니다.
- 다리를 아래로 곧게 내린 `stand`는 모든 관절이 `0°`입니다.
- `<` 형태의 `stand45`는 모든 다리에서 `(J1, J2, J3) = (0°, +45°, +90°)`입니다.
- 모터 장착 방향 차이는 저수준 `direction` 변환에서만 처리합니다.
- 좌표계와 중립점은 회사에서 실제 기체를 기준으로 다시 수행한 calibration
  (캘리브레이션)을 최종값으로 사용합니다. 이후 J1 `+4°` 외전 시험에서 확인한
  실제 장착 방향에 따라 저수준 J1 direction은 FL/RR `+1`, FR/RL `-1`입니다.
  논리 계층에서는 네 다리 모두 동일한 양수 각도를 사용합니다.
- trot gait (트로트 보행)의 전후 궤적은 FL/FR에 `-1`, RL/RR에 `+1`을 적용해
  대각선 쌍 `FL+RR`, `FR+RL`이 같은 방향으로 진행하도록 정리했습니다.
- 회사에서 갱신한 중심점과 degree 기반 포즈를 기준으로 삼고, 로컬의 ID 교환,
  다리별 토크/중립 제어, 현재 위치 유지 기능을 함께 병합했습니다.

## 2026-08-07 STM32 앞뒤 전후 보폭 부호 수정

- STM32 대각선 trot 실기에서 뒷다리는 전진 궤적이지만 FL/FR가 뒤로 걷는 것처럼
  움직이는 현상을 확인했습니다.
- 첫 수정에서 FL/FR를 `+1`로 바꾸자 앞다리는 정상 전진했지만 RL/RR가 뒤로
  걷는 것처럼 보였습니다. 앞뒤 다리가 거울 대칭인 실제 기구를 기준으로 RL/RR도
  반대 방향인 `-1`로 변경했습니다.
- 최종 STM32 전진 부호는 FL/FR=`+1`, RL/RR=`-1`이며 다리별 설정은
  `g_robot_gait_forward_signs`로 분리했습니다. 대각선 쌍은 같은 위상을 유지하고
  실제 발끝 전진 방향만 일치시킵니다.
- 이 변경은 J2 전후 보폭에만 적용하며 IMU Pitch/Roll 및 J1 균형 보정 부호는
  변경하지 않았습니다.

## 2026-08-07 STM32 매 step 실제 위치 동기화

- 시간 기반 15% 네 발 지지 구간만으로는 서보별 부하와 추종 지연 때문에 실제
  대각선 스윙 시작 시점이 어긋날 수 있음을 확인했습니다.
- 각 대각선 liftoff 직전에 12개 Present Position을 읽고 직전 지지 목표의
  `24 tick` 안에 모두 들어올 때까지 phase를 정지하는 step barrier를 추가했습니다.
- 최대 대기는 `500ms`이며 실패 시 가장 오차가 큰 서보 ID와
  `step synchronization timeout`을 출력하고 stand 목표를 요청합니다.

## 2026-08-07 STM32 전방 스윙 L3 여유 높이

- 발이 앞으로 뻗을 때 L3/J3가 충분히 굽혀지지 않아 발끝을 끌 수 있다는 실기
  관찰을 반영했습니다.
- 지지 구간에서만 보폭에 따른 다리 높이 보정을 유지하고, 스윙 중에는 보정을
  부드럽게 제거해 전후 이동에서 생기는 자연스러운 상승을 사용합니다.
- 스윙 후반 전방 이동에 L3/J3 최대 `+6°` 굽힘을 추가하고 발끝 전후 밀림을
  줄이기 위해 L2/J2에 절반을 함께 적용합니다. 착지 직전에는 두 보정을
  smoothstep으로 원래 지지 궤적에 연결합니다.

## 2026-08-02 무부하 트롯 시험

아래 시험은 모두 몸체를 고정하고 다리를 공중에 둔 상태에서 수행했습니다.
각 명령은 정상 종료 후 `stand45` 목표를 전송했습니다.

| Period | Cycles | Swing | 4-leg overlap | 관찰 |
|---:|---:|---:|---:|---|
| 2.0s | 3 | 0.70s | 0.30s | 안정적으로 동작 |
| 1.8s | 3 | 0.63s | 0.27s | 정상 동작 |
| 1.6s | 3 | 0.56s | 0.24s | 정상 동작 |
| 1.4s | 3 | 0.49s | 0.21s | 정상 동작 |
| 1.2s | 10 | 0.42s | 0.18s | 정상 동작 |
| 1.0s | 10 | 0.35s | 0.15s | 양호 |
| 0.8s | 10 | 0.28s | 0.12s | 양호, 무부하 실용 후보 |
| 0.6s | 10 | 0.21s | 0.09s | 동작하지만 실제 이동 범위가 짧아 보임 |

### 속도값 비교

- 느린 `period=4.0`에서는 네 다리가 비슷한 범위로 움직였습니다.
- 빠른 시험 중 FL이 한 차례 작고 느리게 보였으나 이후 같은 조건에서 재현되지
  않았습니다.
- FL J2/J3와 나머지 J2/J3의 운전 모드, 속도, 가속도, 토크 제한이 같았습니다.
- J2/J3 8개 서보의 EEPROM 주소 9~33 원시값도 모두 같았습니다.
- `speed=60`, `1000`, `1200`에서 네 다리가 균일한 경우를 확인했습니다.
- `speed=3000`에서 관찰된 FL 차이는 반복 시험에서 일정하지 않았으므로 특정
  speed 값의 문제로 확정하지 않았습니다.
- 현재 비교 기준값은 `speed=1000`, `accel=100`입니다.

### 현재 판단

- 공중 무부하에서 보폭과 속도의 균형은 period 0.8~1.0초가 가장 좋아 보였습니다.
- period 0.6초는 명령 주기는 빨라지지만 실제 관절이 전체 목표 범위를 따라가지
  못해 이동 폭이 작아질 가능성이 큽니다.
- 이 값은 실제 보행 권장값이 아닙니다. 최종 조립 후에는 period 1.8~2.0초,
  짧은 cycles부터 다시 시작해야 합니다.

## 재현 명령

포트를 셸에 한 번 저장합니다.

```bash
export SPOT_SERVO_PORT=/dev/cu.usbmodem5B790788341
```

상태와 기준 자세:

```bash
spotctl status
spotctl stand45
spotctl hold
spotctl relax
```

현재 위치를 12관절 중립점으로 캡처한 뒤 미세 조정하는 절차(몸체 고정, 다리는
공중에 둔 상태):

```bash
spotctl capture-stand
spotctl calibrate --speed 60 --accel 30
```

특정 다리만 손으로 조정했다면 `spotctl capture-stand --leg RL`처럼 해당 다리만
저장할 수 있습니다. 이어서 `calibrate`에서 각 관절을 눈으로 비교하며
`+/-1`, `+/-5`, `+/-10` tick으로 중립 오프셋을 미세 조정합니다.

기본 소진폭 시험:

```bash
spotctl walk --gait trot --preset test --cycles 1
```

현재 무부하 비교 기준:

> v3 CLI는 관절 진폭을 degree로 받으므로 당시 raw 280/400 tick을
> 각각 24.6°/35.2°로 환산했습니다.

```bash
spotctl walk --gait trot --preset natural --cycles 10 \
  --hip 24.6 --lift 35.2 \
  --period 1.0 --speed 1000 --accel 100 --rate 100
```

period 0.8/0.9/1.0 비교:

```bash
spotctl walk --preset natural --cycles 10 --hip 24.6 --lift 35.2 \
  --period 0.8 --speed 1000 --accel 100 --rate 100

spotctl walk --preset natural --cycles 10 --hip 24.6 --lift 35.2 \
  --period 0.9 --speed 1000 --accel 100 --rate 100

spotctl walk --preset natural --cycles 10 --hip 24.6 --lift 35.2 \
  --period 1.0 --speed 1000 --accel 100 --rate 100
```

## 소프트웨어 검증

### 고하중 보행 자세

- `power` 프리셋은 J2 `30°`, J3 `60°`의 power stance (고하중 자세)를 사용합니다.
- 현재 `45°/90°` 자세보다 다리를 펴 수직 하중의 모멘트 암을 줄이되, 완전 직선
  특이점을 피하도록 각 관절에 `30°`의 여유를 남겼습니다.
- duty factor (지지율)는 `0.58`로 두어 주기의 `16%`를 네 발 전환 지지 구간으로
  사용합니다. 보폭은 `10°` 상당값, 스윙 높이는 `16°` 상당값으로 제한했습니다.
- 전환 구간에는 다음 지지 대각선에 J1 `1.5°`와 preload `1.5°` 상당값을 먼저
  적용하고, 반대 대각선의 하중을 뺀 뒤 J2/J3 스윙을 시작합니다.
- 이는 토크 제한을 올리는 기능이 아니라 기구학적 자세와 접지 시간을 조정하는
  프리셋입니다. 실제 발과 하단 링크를 장착한 뒤에는 `cycles=1`부터 시험해야 합니다.
- J1 외전의 논리값은 네 다리 모두 동일한 양수를 사용합니다. 실제 장착 방향은
  저수준 motor direction에서 FL/RR `+1`, FR/RL `-1`로만 변환합니다. 회사에서
  측정한 calibration center는 변경하지 않습니다.

2026-08-04 고하중 자세와 동시 도착 속도 프로파일 반영 기준 단위 테스트 44개가
통과했습니다.

- 일반 포즈 전환은 12개 서보 목표를 한 Sync Write (동기식 일괄 쓰기)로 보내
  동시에 출발합니다.
- 현재 위치에서 목표까지의 tick 이동량과 STS3215 acceleration
  (`설정값 × 100 step/s²`)을 이용해 관절별 speed를 역산합니다.
- 기본 `speed=1000`, `accel=80`에서 `stand → stand45`는 J2 약 `470`,
  J3 `1000`; `stand → landing`은 J2 약 `290`, J3 `1000`으로 계산됩니다.
- 이 계산은 무부하 기준 trapezoidal velocity profile (사다리꼴 속도 프로파일)
  모델입니다. 실제 장착 상태에서는 하중, 전압과 마찰에 따른 작은 도착 오차가
  생길 수 있으므로 하드웨어 시험으로 확인해야 합니다.

```bash
python -m unittest discover -s tests -v
```

검증 항목:

- 12개 ID 배치, 중심 보정과 저장 포즈 로딩
- STS3215 7바이트 위치 명령 패킷
- Goal Position 전용 Sync Write
- FL+RR 및 FR+RL 대각선 동기
- 65% 지지율과 네 발 접지 전환 구간
- 시작·종료 진폭 0에서 기준 자세 유지
- 지정 rate의 보행 패킷 수
- 최종 위치 검증이 필요한 일반 포즈 명령
- EEPROM ID 변경 안전 절차
- CLI 명령과 별칭

## 2026-08-05 단일 IMU 자세 제어와 모터 부하 기반 접촉 추정

### 목표와 시험 조건

오늘 작업의 목표는 Spot처럼 몸체가 공중에 고정된 것처럼 보이는 Body
stabilization (몸체 안정화)을 단계적으로 구현하고, 별도 Foot contact sensor
(발 접촉 센서) 없이 STS3215 Load estimate (모터 부하 추정값)로 실제 발 접촉을
구분할 수 있는지 확인하는 것이었습니다.

실제 하드웨어 부하 시험 조건은 다음과 같습니다.

| 항목 | 값 |
|---|---|
| 로봇 상태 | 몸체를 지지대에 고정, 다리는 공중에서 동작 |
| 서보 | STS3215 12개, J2/J3 부하 측정은 8개 |
| 어댑터와 포트 | URT-2, `/dev/cu.usbmodem5B790788341` |
| Baud rate (통신 속도) | 1,000,000 bps |
| 보행 | Diagonal trot (대각선 트롯), FL+RR / FR+RL |
| 하드웨어 동적 시험 자세 | Power preset (고하중 프리셋), J1 4°, J2 30°, J3 60° |
| 동적 시험 궤적 | period 2.0s, hip 8°, lift 20°, duty 0.58 |
| 모터 프로파일 | speed 800, acceleration 80 |
| 제어와 측정 | 위치 전송 50Hz, 모터별 상태 읽기 5Hz |
| 횟수 | 10 cycles, 총 20초 |

MuJoCo 비교는 Simulator-only `sim-trot`을 사용했습니다. 조건은 period `0.8s`,
duty `0.50`, lift `30°`, control rate `50Hz`, stance `(J1,J2,J3) =
(4°,45°,90°)`, 10 cycles입니다.

### 중앙 Virtual IMU (가상 IMU) 1개 구현

URDF의 몸체 중앙 `imu_link`에 MuJoCo sensor site를 추가하고 다음 두 센서값만
읽도록 했습니다.

- Orientation quaternion (자세 쿼터니언): Roll/Pitch 계산
- Gyroscope (자이로스코프): Roll/Pitch angular rate (각속도) 계산

제어 입력은 실제 BNO086으로 교체할 수 있도록 `ImuSample`과
`AttitudeController`로 분리했습니다. 좌표계는 X 전방, Y 좌측, Z 위쪽이며,
PD attitude control (PD 자세 제어)은 다음 형태입니다.

```text
roll effort  = Kp × roll  + Kd × roll rate
pitch effort = Kp × pitch + Kd × pitch rate
```

초기 네 다리 J2/J3 높이 보정에서 `sim-trot`의 이득을 탐색했습니다. 실제
Gyroscope rate를 사용한 뒤 기존 수치 미분용 `Kp=1.2`, `Kd=0.08`은 과보정되어
전진 방향과 대각선 접촉을 망쳤습니다. 탐색 결과 초기 안정값은 `Kp=0.9`,
`Kd=0.04`, normalized leg-length correction limit (정규화 다리 길이 보정 제한)
`0.15`였습니다.

초기 이득 탐색의 대표 결과는 다음과 같습니다.

| Kp | Kd | Limit | Max Roll | Max Pitch | Diagonal contact | Delta X | Delta Y |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 0.5 | 0.00 | 0.10 | 14.96° | 7.21° | 62.5% | +1.043m | -0.048m |
| 0.8 | 0.02 | 0.12 | 12.13° | 6.91° | 65.3% | +1.062m | -0.045m |
| 0.9 | 0.04 | 0.15 | 9.45° | 5.95° | 73.6% | +1.066m | +0.015m |
| 1.2 | 0.04 | 0.15 | 9.35° | 5.99° | 71.9% | +0.988m | +0.098m |

### Contact-aware J1 control (접촉 인식 J1 제어)

처음에는 보행 위상상 Stance leg (지지 다리)에만 J2/J3 높이 보정을 적용했습니다.
그러나 Position-controlled servo (위치제어 서보)에서 스윙과 착지 경계의 목표
높이가 불연속적으로 바뀌면서 결과가 나빠졌습니다.

| 방식 | Max Roll | Max Pitch | Attitude RMS | Body Z range | Diagonal contact | Delta Y |
|---|---:|---:|---:|---:|---:|---:|
| 예정 지지 다리 J2/J3만 보정 | 15.55° | 9.13° | 8.39° | 0.044m | 50.1% | +0.455m |
| 실제 접촉 다리 J2/J3만 보정 | 13.72° | 8.57° | 6.78° | 0.040m | 64.9% | +0.403m |

따라서 J2/J3의 수직 IK 보정은 네 다리에 연속 적용하고, 실제 접촉 여부에 따라
J1의 역할만 나눴습니다.

- Stance leg (지지 다리): 고정된 발을 통해 몸체를 기울기의 반대쪽으로 밀기
- Swing leg (스윙 다리): 넘어지는 방향으로 발을 옮겨 다음 지지점 만들기
- MuJoCo 접촉값이 없을 때는 Gait phase (보행 위상)를 fallback (대체값)으로 사용

J1 roll gain (J1 좌우 보정 이득)은 `2~10`을 비교했습니다. `5`에서 전진거리와
좌우 표류를 유지하면서 Max Roll, Attitude RMS와 대각선 접촉이 함께 개선되어
기본값으로 선택했습니다. 최종 이득은 `Kp=1.0`, `Kd=0.04`, leg correction
limit `0.15`, J1 gain `5`, J1 limit `5°`입니다.

최종 동일 조건 비교는 다음과 같습니다.

| 제어 | Max Roll | Max Pitch | Attitude RMS | Body Z range | Diagonal contact | Delta X | Delta Y | 판정 |
|---|---:|---:|---:|---:|---:|---:|---:|---|
| Open-loop (개루프) | 30.15° | 8.30° | 13.08° | 0.056m | 41.3% | +0.884m | +0.102m | UPRIGHT / NON_TROT |
| J2/J3 all-leg feedback | 9.96° | 6.34° | 3.51° | 0.032m | 73.6% | +1.004m | -0.125m | UPRIGHT / TROT |
| Contact-aware J1 포함 | 7.61° | 5.44° | 3.03° | 0.031m | 74.7% | +1.034m | -0.042m | UPRIGHT / TROT |

Sagittal foot placement (전후 착지 위치 보정)도 구현해 시험했지만 현재 질량
모델에서는 Pitch와 표류가 증가했습니다. 예를 들어 forward placement gain
`0.05`만 사용했을 때 Max Roll `11.14°`, Max Pitch `6.66°`, Attitude RMS
`3.82°`, diagonal contact `72.3%`였습니다. 따라서 기능은 유지하되 기본 이득은
`0`으로 두었습니다. 배터리 위치와 링크 질량을 실측 모델에 반영한 후 다시
조정합니다.

### 정지 Motor-load contact detection (모터 부하 접촉 감지)

`spotctl contacts`를 추가해 모터 목표나 Torque Enable (토크 활성화) 상태를
바꾸지 않고 J2/J3 부하를 관찰하도록 했습니다. `spotctl hold`로 현재 위치에
토크를 건 뒤, 네 발이 공중인 상태에서 2초 동안 무부하 중앙값을 측정했습니다.

시험 시작 위치는 다음과 같았습니다.

```text
ID 1=2088, 2=1726, 3=1040, 4=2086, 5=2505, 6=3003,
ID 7=2107, 8=1635, 9=3020, 10=1957, 11=2548, 12=1024
```

무부하 기준은 FL J2 `+32`, FL J3 `0`, 나머지 FR/RL/RR J2/J3는 모두 `0`으로
측정됐습니다. 한 발씩 힘을 가했을 때 각 다리의 Static peak score (정지 최대
점수)는 FL `32`, FR `36`, RL `56`, RR `52`였습니다.

초기 공통 임계값 `engage=120`, `release=80`은 모든 접촉을 놓쳤습니다. 정지
관찰용 기본값을 다음처럼 수정했습니다.

```text
Engage: 부하 변화가 2개 연속 표본에서 24 이상
Release: 부하 변화가 3개 연속 표본에서 8 이하
Sample rate: 10Hz
```

Hysteresis (히스테리시스)와 Debounce (연속 표본 확인)를 사용합니다. FR은 첫 힘
입력 후 약 `32`, RL은 `24`, RR은 `32~40`의 잔류값이 관찰됐습니다. 실제로 힘을
제거했는데도 이 값이 남는 경우에는 Load만으로 Release를 확정할 수 없으므로
Position error (위치 오차)와 Current (전류)를 함께 사용해야 합니다.

### 공중 Dynamic load profile (동적 부하 프로파일)

정지 임계값이 실제 보행에 유효한지 확인하기 위해 `walk --profile-loads`를
구현했습니다. 50Hz Sync Write (동기 위치 전송)를 유지하면서 J2/J3 8개를 한
제어 프레임당 최대 한 개씩 Round-robin sampling (순환 표본화)했습니다.

첫 프로파일 명령은 다음과 같습니다.

```bash
spotctl --port /dev/cu.usbmodem5B790788341 walk \
  --gait trot --preset power --cycles 10 \
  --period 2.0 --hip 8 --lift 20 \
  --speed 800 --accel 80 --rate 50 \
  --profile-loads --load-rate 5
```

생성 파일은 `logs/air_gait_loads_20260805_024609.csv`입니다.

- 실행 시간: 20초
- 총 표본: 800개와 header 1행
- 모터별 표본: 100개, 주기당 10개
- 최대 상태 읽기 시간: `1.247ms`
- 늦어진 50Hz Control frame (제어 프레임): `0개`

공중 보행 중 절대 부하는 정지 접촉 점수보다 훨씬 컸습니다.

| Leg/Joint | Signed load range | Absolute median | Absolute P95 | Absolute max | Position error P95 / max |
|---|---:|---:|---:|---:|---:|
| FL J2 | -232..+204 | 108 | 220 | 232 | 87 / 89 tick |
| FL J3 | -284..+276 | 60 | 272 | 284 | 152 / 155 tick |
| FR J2 | -220..+244 | 112 | 236 | 244 | 89 / 106 tick |
| FR J3 | -280..+268 | 86 | 264 | 280 | 170 / 173 tick |
| RL J2 | -184..+208 | 92 | 204 | 208 | 79 / 81 tick |
| RL J3 | -280..+256 | 128 | 272 | 280 | 170 / 173 tick |
| RR J2 | -228..+184 | 132 | 216 | 228 | 107 / 108 tick |
| RR J3 | -272..+264 | 72 | 264 | 272 | 223 / 225 tick |

따라서 `abs(load) > static threshold` 방식은 보행 중 사용할 수 없습니다. 대신
동일한 Gait phase (보행 위상)의 공중 예상 부하를 빼는 Dynamic-load residual
(동적 부하 잔차)을 사용하도록 변경했습니다.

```text
residual = abs(measured load) - expected abs(load at the same leg phase)
```

STS3215는 같은 위상에서도 부하 방향 부호가 바뀌었습니다. FL J3 local phase
`0.52`에서 첫 시험은 `-20, -24, +20, +20, 0, +24, +32, +24, +28, +28`, 두 번째
시험은 `-20, +28, -32, +20, -20, +20, -24, -20, +28, -20`이었습니다. 부호는
불안정하지만 크기는 대부분 `20~32`로 반복됐기 때문에 Signed load가 아니라
Absolute load magnitude (절대 부하 크기)를 모델링했습니다.

Amplitude ramp (진폭 램프)와 첫 Warm-up cycle (준비 1주기)을 제외한 완전한
주기만 사용해 `air_gait_loads_20260805_024609.baseline.json`을 생성했습니다.
관절별 Noise residual (반복 잡음)과 최종 임계값은 다음과 같습니다.

| Leg/Joint | Noise P90 | Noise P95 | Noise max | Engage | Release |
|---|---:|---:|---:|---:|---:|
| FL J2 | 8 | 10 | 12 | 28 | 12 |
| FL J3 | 8 | 12 | 24 | 28 | 12 |
| FR J2 | 10 | 10 | 12 | 28 | 16 |
| FR J3 | 14 | 18 | 32 | 36 | 20 |
| RL J2 | 8 | 12 | 26 | 28 | 12 |
| RL J3 | 8 | 10 | 20 | 28 | 12 |
| RR J2 | 8 | 10 | 12 | 28 | 12 |
| RR J3 | 8 | 14 | 36 | 32 | 12 |

Engage 값은 기본적으로 `max(24, P95 + 16을 4단위 올림)`으로 계산했습니다.
Release는 P90을 기준으로 더 낮게 설정해 경계에서 상태가 반복되는 것을 막습니다.

### 두 번째 공중 재현 시험

같은 보행 명령과 생성한 baseline을 사용해 두 번째 20초 시험을 수행했습니다.
파일은 `logs/air_gait_loads_20260805_025228.csv`입니다.

- 총 표본: 800개
- 최대 읽기 시간: `1.779ms`
- 늦어진 50Hz 제어 프레임: `0개`
- 시작·종료 램프와 첫 주기를 제외한 평가 구간: 8 cycles
- 평가 표본: 다리당 160개, 총 640개

최종 Absolute residual validation (절대 부하 잔차 검증) 결과입니다.

| Leg | Max residual | Threshold exceedance | False-positive rate |
|---|---:|---:|---:|
| FL | 26 | 0 / 160 | 0% |
| FR | 32 | 0 / 160 | 0% |
| RL | 26 | 0 / 160 | 0% |
| RR | 20 | 0 / 160 | 0% |

즉 동일한 자세와 속도의 공중 보행에서는 Phase-aligned absolute-load baseline
(위상 정렬 절대 부하 기준 모델)이 재현됐습니다. 오늘 결과는 False positive를
제거한 단계이며, 실제 접촉에 대한 True positive (정접촉 판정)는 아직 시험하지
않았습니다.

### 구현 파일과 현재 안전 경계

- `servo/attitude.py`: 실제 BNO086과 MuJoCo가 공유할 IMU sample 및 PD 제어
- `servo/contact.py`: 정지 부하 기준 Hysteresis/Debounce 접촉 판정
- `servo/load_profile.py`: 보행 위상별 절대 부하 baseline 생성·저장·조회
- `simulation/mujoco/generate_scene.py`: 몸체 중앙 IMU site와 quaternion/gyro 센서
- `simulation/mujoco/walk.py`: 단일 IMU, J1/J2/J3 자세 보정과 접촉 통계
- `spotctl contacts`: 정지 상태 접촉 관찰
- `spotctl walk --profile-loads`: 공중 동적 부하 CSV 기록
- `spotctl analyze-loads`: CSV에서 `.baseline.json` 생성
- `spotctl walk --load-baseline`: 실시간 잔차 비교 기록

접촉 판정은 아직 Observation only (관찰 전용)이며 실제 보행 목표나 모터 위치를
변경하지 않습니다. BNO086 실제 드라이버, 과도 기울기 정지, 접촉 결과를 사용한
J1/J2/J3 폐루프 보정도 아직 실제 로봇에는 연결하지 않았습니다.

오늘 최종 검증 시 Servo tool unit test (서보 도구 단위 테스트) `55개`, URDF
validation, MuJoCo 10-cycle `UPRIGHT/TROT` 검증이 모두 통과했습니다.

### 다음 시험

몸체를 계속 지지대에 고정하고 완성된 발이 평평한 판에 가볍게 닿도록 한 뒤,
동일한 느린 보행에서 Supported-contact profile (지지대 고정 접촉 프로파일)을
수집합니다. 발 부품이 모두 장착됐고 스윙 다리가 판에 걸리지 않을 때만 실행합니다.

```bash
spotctl --port /dev/cu.usbmodem5B790788341 walk \
  --gait trot --preset power --cycles 5 \
  --period 2.0 --hip 8 --lift 20 \
  --speed 800 --accel 80 --rate 50 \
  --load-rate 5 \
  --load-baseline tools/servo_tool/logs/air_gait_loads_20260805_024609.baseline.json \
  --profile-output tools/servo_tool/logs/supported_contact.csv
```

이 데이터에서 stance 구간 True-positive rate, swing 구간 False-positive rate,
접촉 판정 지연을 확인한 뒤에만 실제 자세 제어 입력으로 연결합니다.

## 2026-08-05 저녁 STM32F446RE 서보 제어 계층 구현

기존 Mac/Python → URT-2 직접 제어를 Jetson/STM32 구조로 옮기기 위한 첫 단계로
STM32F446RE용 제어 계층을 구현했습니다. 목표 연결은 다음과 같습니다.

```text
Mac 또는 Jetson
  ↓ USB / ST-LINK VCP, 텍스트 명령 115200 bps
STM32F446RE USART2
  ↓ 명령 해석
STM32F446RE USART1, Feetech binary protocol 1 Mbps
  ↓ UART TX/RX
URT-2
  ↓ half-duplex TTL bus
STS3215 ×12
```

URT-2가 STS3215 half-duplex TTL 버스 전환을 담당하므로 STM32 USART1은
Single-wire mode가 아니라 일반 Asynchronous TX/RX로 사용합니다.

### 구현 범위

- `feetech_protocol.*`: instruction/status packet, checksum, little-endian 및
  sign-magnitude 변환
- `servo_bus.*`: blocking UART request/response, timeout, checksum 검증, URT-2
  송신 echo 대응
- `sts3215.*`: Ping, Torque Enable, Position/State Read, Position Write,
  Sync Write
- `robot_config.*`: `config/joints.json`의 ID·center·direction 12축 설정 이식
- `robot.*`: 전체 서보 확인, 현재 위치 hold, torque rollback, 단일 서보 안전
  이동, 2초 stand ramp, 최종 위치 검증
- `app_console.*`: USART2 interrupt 기반 텍스트 명령 콘솔

부팅만으로 모터가 움직이지 않으며 `move`, `hold`, `stand`처럼 명시적인 명령을
입력해야 토크가 활성화됩니다. `move`는 현재 위치에서 최대 256 tick까지만
허용합니다. `stand`는 ID 1~12가 모두 응답해야 시작하며, 현재 위치를 먼저 Goal
Position으로 전송한 뒤 토크를 켜 오래된 목표값으로 튀는 동작을 방지합니다.

현재 Python canonical coordinate와 동일한 stand 목표는 `(J1, J2, J3) =
(0°, +45°, +90°)`이고 raw target은 다음과 같습니다.

```text
ID 1..12 = 2091, 1723, 1036, 2087, 2511, 3001,
            2103, 1630, 3020, 1958, 2552, 1023
```

ARM Cortex-M4 대상 경고 오류 컴파일, C packet 단위 시험, STM32/Python 보정값
일치 검사를 통과했고 기존 Servo tool 단위 시험 55개도 통과했습니다. 이 시점에는
실제 STM32 → URT-2 Ping은 아직 검증하지 않았습니다.

## 2026-08-06 STM32 시리얼 및 CubeMX 통합 점검

### Mac 연결 경로와 `spotctl` 사용 범위

Mac에 나타난 `/dev/cu.usbmodem...` 장치는 `STM32 STLink` VCP였습니다. 이 포트는
USART2 텍스트 콘솔이며 URT-2의 Feetech raw serial port가 아닙니다. 따라서 현재
구성에서 다음 명령은 사용할 수 없습니다.

```text
spotctl --port /dev/cu.usbmodem... scan
```

`spotctl`은 Mac → URT-2 USB 직접 연결에서만 사용합니다. Mac → STM32 구조에서는
115200 bps 터미널로 연결한 뒤 펌웨어의 `ping 1`, `scan`, `read 1` 명령을
사용합니다. STM32 펌웨어는 현재 transparent serial bridge가 아닙니다.

### 콘솔 RX 문제 분석

USART2 TX는 부팅 로그를 출력했지만 터미널에서 입력한 `help`에 응답하지 않는
현상이 있었습니다. 터미널의 Local Echo는 입력 문자를 화면에 표시할 뿐 STM32가
수신했다는 증거가 아닙니다. CubeMX 재생성 후 다음 설정이 제거된 것이 원인이었습니다.

- `USART2 global interrupt`
- `USART2_IRQHandler()`와 `HAL_UART_IRQHandler(&huart2)` 호출

USART2 NVIC를 다시 활성화하고 우선순위를 1로 설정했습니다. 콘솔 개행 처리도
터미널 종류와 무관하게 `CR`, `LF`, `CRLF`를 모두 허용하도록 수정했습니다.

### IMU 로그 제어

BNO055는 I2C 주소 `0x28`, CHIP_ID `0xA0`으로 검출됐고 NDOF 초기화 성공 로그를
확인했습니다. 부팅 후 자세 로그가 콘솔 명령을 방해하지 않도록 연속 출력 기본값을
OFF로 변경하고 다음 명령을 추가했습니다.

```text
imu on
imu off
imu status
```

`imu on`일 때만 10Hz로 Yaw/Roll/Pitch를 출력합니다.

### USART1 baud 회귀와 최종 `.ioc` 설정

첫 STM32 `scan`에서는 ID 1~12가 모두 timeout이었습니다. 이후 생성된 코드를
검토한 결과 CubeMX 재생성 과정에서 USART1 baud가 `1,000,000`에서 `115200`으로
되돌아간 사실을 확인했습니다. STS3215 버스와 속도가 맞지 않았으므로 이 timeout을
URT-2 배선이나 서보 고장으로 확정할 수 없습니다.

최종 `.ioc` 기준 UART 설정은 다음과 같습니다.

| 항목 | USART1: URT-2 | USART2: ST-LINK 콘솔 |
|---|---|---|
| Pins | PA9 TX, PA10 RX | PA2 TX, PA3 RX |
| Mode | Asynchronous TX/RX | Asynchronous TX/RX |
| Baud | 1,000,000 | 115200 |
| Frame | 8-N-1 | 8-N-1 |
| Flow control | None | None |
| Oversampling | 16 | 16 |
| Global interrupt | OFF, blocking I/O | ON, priority 1 |

CubeMX가 다시 기본 baud를 생성하더라도 실행 시 USART1을 1Mbps로 보정하는 guard를
USER CODE 영역에 추가했습니다. CubeMX 생성 IRQ 코드와 임시 USER CODE IRQ가
중복됐던 부분도 제거해 handler와 NVIC 설정이 한 번만 생성되도록 정리했습니다.
STM32CubeIDE에 포함된 GNU Arm GCC 14.3으로 관련 소스의 ARM 컴파일을 통과했습니다.

현재 시스템 클록은 HSI 16MHz이고 USART1 1Mbps에는 사용할 수 있습니다. TIM2는
초기화만 되고 시작되지 않으며 현재 prescaler/period도 1kHz 설정이 아닙니다.
향후 실시간 1kHz 제어 단계에서는 PLL 84MHz, TIM 주기, watchdog, 명령 timeout을
별도로 설계합니다.

### 다음 STM32 실기 확인

수정된 `.ioc`로 Clean → Build → Flash한 뒤 아래 순서로 재검증합니다.

```text
help
imu status
ping 1
scan
read 1
```

`ping 1`이 계속 timeout이면 PA9에서 `FF FF 01 02 01 FB`가 실제 1Mbps로
출력되는지 확인한 뒤 URT-2 로직 전원, Feetech 표기 기준 TX-TX/RX-RX 연결,
공통 GND, TTL 버스와 서보 12V 전원을 순서대로 점검합니다. 12개 Ping과 Position
Read가 확인되기
전에는 `hold`와 `stand`를 실행하지 않습니다.

## 2026-08-06 STM32 ↔ URT-2 실기 연결 문제 해결 기록

### 최초 증상과 하드웨어 분리

STM32 콘솔의 `ping 1`, `scan`, `stand`에서 처음에는 ID 1~12가 모두 다음처럼
timeout이었습니다.

```text
ID 1: timeout, servo_error=0x00
```

`servo_error=0x00`은 서보가 오류 상태를 응답한 것이 아니라 status packet 자체를
STM32가 받지 못했다는 뜻입니다. 같은 URT-2를 Mac에 USB로 직접 연결한 뒤 Python
도구를 실행하면 12개가 모두 검출됐습니다.

```text
spotctl scan
Found: 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12
```

따라서 서보 ID, 서보 전원, URT-2의 TTL 버스 구간은 정상이고 문제 범위를
STM32 USART1 ↔ URT-2 UART 헤더로 좁혔습니다.

### 최종 배선과 전원 원칙

NUCLEO-F446RE의 Arduino 헤더 기준 연결은 다음과 같습니다. Feetech UART 헤더는
신호 이름 기준으로 표기되어 MCU와 `TX-TX`, `RX-RX`로 연결합니다.

```text
NUCLEO D8 / PA9  / USART1_TX  → URT-2 TX
NUCLEO D2 / PA10 / USART1_RX  → URT-2 RX
NUCLEO GND                     → URT-2 GND
URT-2 signal-level switch      → 3.3V
URT-2 RTS                      → 연결하지 않음
```

전원 공급은 한 경로만 사용합니다. 재현성이 좋은 시험 구성은 URT-2 Type-C를 USB
충전기/보조배터리로 공급하고 UART 헤더의 VCC는 연결하지 않는 방식입니다. UART
헤더 VCC와 Type-C 전원을 동시에 공급하지 않습니다. STS3215의 약 12V 구동 전원은
URT-2 로직 전원과 별도이며 모든 계통의 GND는 공통이어야 합니다.

URT-2 보드 중앙의 `TX1`, `RX1` SMD LED가 `scan` 때 함께 점멸해 STM32 요청과
서보 응답이 URT-2까지 왕복하는 것도 확인했습니다. `TX2/RX2`는 RS485용입니다.

### 측정 장비 없이 사용한 진단 명령

로직 애널라이저나 오실로스코프가 없었기 때문에 다음 두 명령을 추가했습니다.

1. URT-2를 분리하고 PA9-PA10을 직결한 `uarttest`
2. Ping 후 PA10의 원시 수신 바이트를 출력하는 `busprobe ID`

PA9-PA10 loopback은 다음처럼 통과해 STM32 USART1과 두 핀 자체가 정상임을
확인했습니다.

```text
uarttest
UARTTEST PASS: USART1 PA9/PA10 loopback OK
```

초기 `busprobe 1`은 정상 6바이트 중 일부만 받았습니다.

```text
BUSPROBE RX (2): FF 00
```

수신 코드 수정 후에는 ID 1의 완전한 status packet을 받았습니다.

```text
BUSPROBE RX (6): FF FF 01 02 00 FC
ping 1
ID 1: ok, servo_error=0x00
```

### 실제 소프트웨어 원인

USART1은 1Mbps이므로 8-N-1 한 바이트가 약 10us마다 도착합니다. Debug 빌드는
`-O0`인데 기존 코드는 바이트마다 `HAL_UART_Receive()` 또는 별도 함수 호출과
`HAL_GetTick()` 검사를 수행했습니다. 이 오버헤드 때문에 다음 바이트를 제때 읽지
못해 UART overrun, protocol error, timeout이 무작위 ID에서 발생했습니다. 실패
ID가 scan마다 바뀐 것이 개별 서보 고장이 아니라 공통 수신 경로 문제라는 중요한
단서였습니다.

최종 수정은 다음과 같습니다.

- USART1 `SR.RXNE`를 직접 폴링하고 `DR`을 즉시 읽음
- timeout 검사보다 RXNE 처리를 먼저 수행
- 1바이트 수신 함수를 `always_inline`으로 강제 인라인
- unicast request/response 사이에 URT-2 안정화 간격 10ms 적용
- broadcast Sync Write에는 이 간격을 적용하지 않아 2초 stand ramp 주기 유지

콘솔 `scan`에만 10ms를 넣었을 때는 scan은 성공해도 `stand` 내부의 연속
Ping/Read/Torque 요청에서 다시 protocol error가 발생했습니다. 따라서 간격을
`servo_bus_request()` 공통 경로로 옮겼습니다.

### 최종 실기 검증

수정 펌웨어를 Clean → Build → Flash한 뒤 `busprobe 1`과 연속 scan 3회를
검증했습니다.

```text
BUSPROBE RX (6): FF FF 01 02 00 FC

Scanning configured IDs 1..12
  ID 1 OK
  ...
  ID 12 OK
```

세 번 모두 ID 1~12가 OK였고 이어서 다음 동작도 성공했습니다.

```text
hold
OK
stand
Starting calibrated 2-second stand ramp
OK
relax
OK
```

`relax` 후 다시 stand했을 때 ID 2 final verification이 한 번 실패했지만 직후
`read 2`는 현재 `1726`, 목표 `1723`, 오차 3 tick으로 정상이었습니다. 고정
300ms 검증이 실제 수렴보다 빨랐던 별도 문제였으므로, 최종 위치 검증을 최대 2초
동안 반복 polling하는 방식으로 변경했습니다.

### 함께 확인한 ST-LINK와 콘솔 사항

- MacBook USB-C 직결에서는 ST-LINK VCP가 열거되지 않았지만 외장 USB 허브를
  통하면 `/dev/cu.usbmodem...`이 정상 생성됐습니다. STM32/펌웨어 문제가 아니라
  직결 USB 2.0 호환 또는 케이블 경로 문제로 분리했습니다.
- ST-LINK Flash 로그에서 NUCLEO-F446RE, target voltage 3.23V, download/verify
  성공을 확인했습니다.
- USART2 콘솔은 115200 8-N-1, CR/LF/CRLF를 허용합니다.
- 사용한 터미널은 BS/ANSI 제어문자를 렌더링하지 않으므로 펌웨어 echo 기본값을
  OFF로 하고 터미널 Local Echo를 사용합니다.

### 재발 시 최소 점검 순서

```text
1. spotctl scan                 # URT-2 USB 직접 연결로 서보 12개 확인
2. uarttest                     # URT-2 분리, PA9-PA10 직결
3. busprobe 1                   # FF FF 01 02 00 FC 확인
4. ping 1
5. scan을 3회 반복
6. read 1
7. 지지대 위에서 hold → stand → relax
```

12개 scan이 반복해서 안정되기 전에는 `hold`와 `stand`를 실행하지 않습니다.

## 2026-08-06 direct stand와 BNO055 장착 방향 확인

### stand 목표 일괄 전송

기존 stand는 100개 frame을 20ms 간격으로 보내는 2초 보간 ramp였습니다. 최대
profile로도 전체 동작 시간이 2초로 제한됐기 때문에 사용자 실기 요청에 따라
중간 보간을 제거했습니다. 현재 `stand`는 다음 순서로 동작합니다.

1. ID 1~12 응답 확인
2. 현재 위치를 Goal Position으로 설정
3. 토크 활성화
4. 12개 stand 목표를 한 번의 broadcast SYNC_WRITE로 전송
5. 최대 2초 동안 최종 위치 허용 오차 확인

부팅 기본 profile인 `speed=3400`, `acceleration=254`에서 direct stand 속도가
실기에 적절함을 확인했습니다. 최종 위치 검증과 통신 오류 처리는 그대로
유지합니다.

### BNO055 배선

```text
BNO055 VCC/VIN       → NUCLEO 3V3
BNO055 GND           → NUCLEO GND
BNO055 SCL           → D15 / PB8
BNO055 SDA           → D14 / PB9
BNO055 COM3/I2C-SEL  → GND
```

COM3 LOW는 주소 `0x28`, HIGH는 `0x29`입니다. 현재 펌웨어의 detect 함수는 두
주소를 모두 확인하지만 실기 배선은 COM3를 GND로 연결한 `0x28`입니다. NUCLEO의
모든 GND는 공통이므로 URT-2, IMU GND, COM3를 서로 다른 GND 핀이나 공통 rail로
분기할 수 있습니다. 사진에서 확인한 오른쪽 Arduino 헤더의 AREF-D13 사이 GND도
사용할 수 있습니다.

### 장착 방향과 실제 축 부호

IMU는 인쇄면을 위로 향하게 수평 장착했습니다. 위에서 본 사진에서 로봇 앞쪽이
아래일 때 보드 글자가 정상 방향으로 보이고, 글자의 윗방향은 로봇 뒤쪽입니다.
상하로 뒤집힌 장착은 아닙니다. `imu on`으로 직접 기울이고 회전해 다음 부호를
확인했습니다.

```text
앞으로 기울임  → Pitch 증가
뒤로 기울임    → Pitch 감소
오른쪽 기울임  → Roll 증가
왼쪽 기울임    → Roll 감소
오른쪽 회전    → Yaw 증가
왼쪽 회전      → Yaw 감소, 0° 아래에서 359°대로 wrap
```

현재 장착 좌표계가 로봇 제어에 사용할 부호와 일치하므로 axis remap 코드는
추가하지 않았습니다. 향후 Yaw 폐루프 제어에서는 0/360 경계를 넘는 오차를
`-180..+180°`로 정규화해야 합니다.

## 2026-08-07 STM32/MuJoCo 공용 sim-trot 정책

MuJoCo에서 안정적으로 보인 `sim-trot`을 STM32에 별도로 다시 작성하지 않고
`firmware/stm32-learning/Inc/gait_policy.h`의 HAL 독립 C 함수로 분리했습니다.
STM32 `robot_trot()`와 MuJoCo `walk.py --preset sim-trot`이 같은 Cartesian
발끝 궤적, 2-link IK, Roll/Pitch PD 다리 길이 보정과 J1 보정 함수를 호출합니다.

공용 기본값은 `period=0.8s`, `rate=50Hz`, `duty=0.50`, `J1=4°`,
`J2/J3=45°/90°`, 보폭 입력 `8.8°`, 리프트 입력 `30°`입니다. 기구 배치 차이는
정책을 복제하지 않고 다리별 forward sign 입력으로 처리합니다. MuJoCo URDF는
네 다리 `-1`, 실제 STM32 기체는 `FL/FR +1`, `RL/RR -1`입니다.

MuJoCo는 실제 접촉 다리를 지지 마스크로 전달하고 STM32는 보행 stance 위상을
전달합니다. STM32의 BNO055 읽기, 서보 raw 변환, Present Position 기반 step
barrier와 오류 시 stand 복귀는 하드웨어 계층에 남겼습니다.

검증 결과는 다음과 같습니다.

- 공용 C 관절각과 기존 Python `sim-trot` 기준 오차: 최대 `0.05°` 이내
- Servo tool 단위 시험: `57/57 PASS`
- GNU Arm GCC 14.3 `robot.c`, `app_console.c` 경고 없이 컴파일
- MuJoCo 공용 C 10주기: `UPRIGHT`, `TROT`, 대각선 접촉 `74.7%`
- 전진 `+1.034m`, 최대 Roll `7.61°`, 최대 Pitch `5.44°`

첫 실기에서 두 가지 보호 오류를 확인해 수정했습니다.

- `configuration error; servo=0`: 1200/1600ms ramp의 진행률 `0.96`에서 기존
  정수 smootherstep 근삿값이 `1.004`로 넘쳤습니다. 공용 C의 대칭형 bounded
  smootherstep으로 바꾸고 최종 per-mille 값도 `1000`으로 clamp했습니다.
- `step synchronization timeout; servo=6`: 통신/서보 오류가 아니라 ID 6의
  Present Position이 `24 tick`/`500ms` 조건을 만족하지 못한 경우였습니다.
  12축 장벽은 유지하고 실제 1Mbps 순차 polling 시간과 부하 추종을 고려해
  허용 오차를 `48 tick`, 제한 시간을 `1000ms`로 조정했습니다.

Ramp `-0.02..1.02` 범위의 bounded·단조성·Python 기준 일치 회귀 시험을 추가해
Servo tool 단위 시험은 `58/58 PASS`가 되었습니다.

위 수치는 모델 안의 결과이며 실제 기체 성공을 보장하지 않습니다. 실제 첫 시험은
거치대에서 `trot 1 1200`으로 방향과 간섭을 확인한 다음 기본 `trot 1` 800ms로
진행합니다.

## 2026-08-07 STM32 제자리 반복 점프 정책

전진 점프로 확장 가능한 공용 Cartesian 점프 궤적과 STM32 `jump` 명령을
추가했습니다. 한 주기는 `stand`, 압축, 도약 신전, 공중 tuck, 착지 준비, 충격
흡수, stand 복귀의 7개 waypoint를 50Hz smootherstep으로 연결합니다. 제자리
명령은 `forward_travel=0`을 사용하며 네 다리의 canonical 목표가 완전히 같습니다.

```text
jump 1 2000       # 첫 거치대 시험
jump 3 1500       # 유한 반복
jump              # 기본 1200ms 무한 반복
Ctrl+C            # 현재 frame 뒤 stand 요청 및 중단
```

무한 반복 중에도 USART2 RX ISR이 `0x03`을 직접 감지해 motion abort flag를
설정합니다. 같은 중단 경로를 기존 `trot`에도 연결했습니다. IMU balance가 켜진
경우 Roll/Pitch 기준 오차 `30°` 초과 또는 연속 3회 읽기 실패 시 stand 목표를
요청하고 종료합니다. 점프 중 IMU는 아직 관절 능동 보정이 아니라 tilt guard로만
사용합니다.

공용 호스트 바인딩과 다음 회귀 시험을 추가했습니다.

- phase `0/1`에서 J1/J2/J3가 `0°/45°/90°` stand와 일치
- 제자리 모드에서 네 다리 각도 일치
- 도약/공중 구간 support mask 해제와 착지 구간 복귀
- `forward_travel` 적용 시 다리별 mirror sign 일치
- forward 한계 `±0.30` 거부 검사

Servo tool 시험은 `61/61 PASS`, GNU Arm GCC 14.3 경고 오류 검사와 전체 링크를
통과했습니다. 실제 기체 점프 성공 여부는 아직 확인하지 않았으므로 첫 시험은 반드시
지지대에서 `jump 1 2000`으로 수행합니다.

## 2026-08-07 STM32 제자리 트롯

공용 트롯 정책의 전후 이동량을 `travel_scale`로 분리하고 STM32 콘솔에
`trotplace [cycles [period_ms]]`를 추가했습니다. 리프트 높이, 대각선 위상,
J1/IMU 보정과 step barrier는 기존 `trot`과 동일합니다.

단순 `travel_scale=0` 동역학 시험은 발끝 명령상 전후 이동이 없어도 접촉과 관성으로
10주기 `X=-0.893m`가 발생했습니다. `0.30..0.60` 범위를 탐색한 뒤 세부 시험한
결과 `0.39`에서 `X=-0.014m`, `Y=+0.018m`, 대각선 접촉 `76.1%`,
`state=UPRIGHT`였습니다. 이를 `ROBOT_TROT_IN_PLACE_TRAVEL_SCALE` 기본값으로
설정했습니다.

```text
trotplace 1 1600     # 첫 거치대 시험
trotplace 5 1000     # 반복 시험
Ctrl+C               # stand 요청 후 중단
```

실제 기체의 질량 분포와 바닥 마찰은 MuJoCo 모델과 다르므로 전진하면 scale을 낮추고
후진하면 올립니다. 큰 방향 조정은 `0.05`, 정지점 주변 미세 조정은 `0.01` 단위로
진행합니다. 공용 C wrapper 회귀 시험을 2개 추가해 Servo tool 시험은 `63/63
PASS`가 되었습니다.

## 2026-08-07 Python 환경 통합

Servo Tool과 MuJoCo에 나뉘어 있던 `.venv`, `.venv-mujoco` 안내를 제거하고 저장소
루트의 `environment.yml`로 통합했습니다. 당시 Conda 환경 이름은 `somg`였고
(2026-08-08에 `spot_omg`로 변경) Python 3.10, MuJoCo 3.11.0, pyserial, pytest와
editable `tools/servo_tool`을 설치합니다.

```bash
conda env create -f environment.yml
conda activate somg   # 현재 이름은 spot_omg
pytest tools/servo_tool/tests -q
```

검증 환경은 Python `3.10.20`, MuJoCo `3.11.0`, pyserial `3.5`이며 단위 시험
`63/63 PASS`와 MuJoCo `sim-trot` 10주기 `UPRIGHT/TROT`를 확인했습니다. macOS
실시간 Viewer는 같은 환경에 설치된 `mjpython`을 사용합니다. STM32 ST-LINK VCP는
여전히 텍스트 콘솔이며 `spotctl` 대상 포트가 아닙니다.

## 2026-08-07 MuJoCo 원형 발끝 `trot2`

기존 STM32 공용 `sim-trot`을 보존한 채 `trot2` 정책을 먼저 MuJoCo에
추가했습니다. 접지 구간은 직선으로 지면을 뒤로 밀고, 스윙 구간의 L3 발끝은
끝점 속도가 0인 상반원을 따라 움직입니다. 원 꼭대기 좌표를 원하는 J2/J3 접힘
자세의 순기구학으로 구한 뒤 모든 frame을 2-link IK로 다시 풀기 때문에 L2가
연속적으로 접히고 펴집니다.

초기 `J2=80°/J3=120°`, period `1.2s`, duty `0.55`는 직립을 유지했지만 대각선
접촉이 `35.0%`였습니다. 8개 조합을 비교한 뒤 기본값을 `J2=78°`, `J3=108°`,
period `0.8s`, duty `0.50`으로 정했습니다. 이때 L2는 몸체 수평에서 약 12° 아래까지
접힙니다.

- 원 반지름 오차: raw tick 양자화 포함 최대 `0.002` 이하
- STM32 `balance full`과 같은 이득의 10주기 이동: `X=+2.225m`, `Y=+0.271m`
- 최대 Roll/Pitch: `3.90°/2.47°`
- 대각선 접촉: `56.3%`
- 종료: `UPRIGHT/TROT`
- 기존 `sim-trot`: 기존 결과 `UPRIGHT/TROT`, 대각선 접촉 `74.7%` 유지
- Servo Tool 63개와 `trot2` 회귀 시험 5개: 합계 `68 PASS`

이후 원형 궤적을 `gait_policy_trot2_targets()` 공용 C 함수로 옮기고 STM32
`robot_trot2()`와 콘솔 `trot2 [cycles [period_ms]]`를 추가했습니다. 기존 트롯의
50Hz IMU/J1 balance, step barrier, `Ctrl+C`와 stand 복귀 경로를 재사용합니다.
MuJoCo `auto` controller도 같은 C 함수를 호출하며 Python 기준 구현과 phase·진폭별
최대 `0.06°` 이내로 일치합니다. GNU Arm GCC 14.3 전체 링크 결과는 text
`62,708B`, data `104B`, bss `2,400B`입니다. `spotctl`에는 이 보행을 추가하지
않았습니다.

운동학 화면은 `mjpython simulation/mujoco/walk.py --preset trot2 --cycles 5`,
동역학 화면은 여기에 `--dynamic --balance`를 추가해 확인합니다. 실제 첫 시험은
거치대에서 `profile 800 80`, `trot2 1 1600` 순서로 진행합니다.

## 2026-08-08 Conda 환경 이름 `spot_omg`로 변경

저장소 이름과 맞추기 위해 Conda 환경 이름을 `somg`에서 `spot_omg`로 변경했습니다.
`environment.yml`과 모든 문서, `walk.py`의 macOS viewer 안내 문구를 새 이름으로
갱신했습니다. 패키지 구성은 그대로입니다.

```bash
conda env create -f environment.yml
conda activate spot_omg
pytest tools/servo_tool/tests simulation/mujoco/test_trot2.py -q
```

`simulation/mujoco/test_trot2.py`는 `simulation.mujoco.walk`를 import 했는데
pytest rootdir가 `tools/servo_tool/pyproject.toml` 기준으로 잡혀 저장소 루트가
`sys.path`에 없어 수집 단계에서 실패했습니다. `jump.py`와 같이 자기 디렉터리를
`sys.path`에 넣고 `walk`를 직접 import 하도록 바꿔 두 시험 경로를 한 번에
실행할 수 있습니다. Servo Tool 63개와 `trot2` 5개, 합계 `68 PASS`를 확인했습니다.

## 2026-08-08 `spotctl console`로 STM32 콘솔 구동

지금까지 STM32 명령은 시리얼 터미널에서 직접 타이핑했습니다. 같은 명령을
호스트에서 보내고 응답을 기록할 수 있도록 `servo/console.py`와 `spotctl
console` 하위 명령을 추가했습니다. 서보 버스의 주인은 그대로 STM32이므로
50Hz IMU 균형, step barrier, `Ctrl+C` stand 복귀가 모두 유지됩니다. URT-2
직결 경로(`ServoBus`, Feetech 1 Mbps)는 건드리지 않았습니다.

프로토콜에서 확인한 점:

- 응답의 끝을 알리는 유일한 표시는 개행 없이 출력되는 `"# "` 프롬프트입니다.
- 빈 줄은 펌웨어가 버리고 프롬프트를 내지 않으므로 동기화에 쓸 수 없습니다.
  접속 직후 `echo off`를 보내 배너를 버리고 프롬프트를 맞춥니다.
- `imu on` 상태의 10Hz `Yaw=...` 로그는 명령 응답과 섞여 들어오므로 본문과
  분리해 보관합니다.
- 대기 시간은 cycles와 period로 계산합니다(`trot2 1 1600` → `21.6s`).
  `jump 0`은 완료 시점이 없어 `Ctrl+C` 또는 `--timeout`이 필요합니다.

포트 선택은 예상보다 까다로웠습니다. 이 Mac에서는 URT-2도
`/dev/cu.usbmodem5B790788341`로 잡혀 ST-LINK와 이름이 같은 계열입니다. 그래서
이름 대신 USB descriptor를 조회해 판별합니다. 실측한 ST-LINK 값은 다음과
같습니다.

```text
device        /dev/cu.usbmodem312103
hwid          USB VID:PID=0483:374B SER=066EFF575380535067195536
manufacturer  STMicroelectronics
product       STM32 STLink
```

vendor `0x0483`이면 콘솔, 그 외 USB 시리얼 장치는 URT-2 후보로 봅니다. 둘 다
꽂혀 있어도 각각 올바르게 선택되며, `spotctl ports`가 `0483:374b`와 함께 어느
쪽인지 표시합니다. macOS의 pyserial은 `vid`/`pid`를 `int`가 아니라 `'0x0483'`
문자열로 돌려주므로 두 형식을 모두 받아 정규화합니다.

검증은 pty를 raw 모드로 열어 펌웨어 응답을 흉내내는 방식으로 진행했습니다.
`targets` 12줄 파싱, `trot2 1 1600`의 0.4초 지연 후 `OK`, unknown command의
종료 코드 `1`, `script`와 `--log` 기록, 그리고 연속 `jump 0` 중 SIGINT →
`0x03` 전송 → `STOPPED: stand target requested` → 종료 코드 `130`을 확인했습니다.
단위 시험은 Servo Tool `78`개와 `trot2` `5`개로 합계 `83 PASS`입니다.

실기 확인(읽기 전용 명령만)에서 포트 자동 선택과 응답 파싱이 동작했습니다.

```text
$ spotctl console send targets     -> ID 1..12 target=... , exit 0
$ spotctl console send profile     -> Profile: speed=3400 acceleration=254
$ spotctl console send balance status
   Balance: on, mode: full, target: level, IMU: available, policy: required
$ spotctl console send scan        -> ID 1..12 timeout, exit 1
```

`scan`의 12축 timeout은 서보 버스 전원/URT-2 연결 문제이며 콘솔 자체는
정상입니다. 이 결과 덕분에 분류기의 빈틈을 하나 찾았습니다. `print_bus_result`가
쓰는 `ID 3: timeout, servo_error=0x00`에는 `ERROR:` 접두사가 없어 처음에는
성공으로 분류되어 `scan`이 종료 코드 `0`을 냈습니다. `^\s*ID \d+: (?!ok,)`를
추가해 실패로 판정하도록 고쳤습니다. `ID 3 target=...`, `ID 1 pos=...`,
`  ID 3 OK`는 콜론이 없어 영향받지 않습니다.

서보를 실제로 움직이는 `stand`, `trot2`, `jump`는 아직 확인하지 않았습니다.
서보 전원을 연결한 뒤 거치대에서 다음 순서로 진행합니다.

```bash
spotctl console send scan            # 12축 OK 확인이 먼저
spotctl console send profile 800 80
spotctl console send stand
spotctl console --log tools/servo_tool/logs/bench.log send trot2 1 1600
```

## 2026-08-08 연결된 장치에 따른 자동 분기

USB descriptor로 STM32와 URT-2를 구분할 수 있게 되었으므로 `console`을 따로
칠 이유가 없어졌습니다. `spotctl`이 붙어 있는 장치를 보고 링크를 정합니다.
ST-LINK면 115200 bps 콘솔, URT-2면 기존 1 Mbps Feetech 경로입니다.

- 양쪽 모두 지원: `scan`, `stand`, `relax`, `hold`
- STM32 전용 하위 명령 추가: `trot`, `trotplace`, `trot2`, `jump`, `targets`,
  `profile`, `imu`, `balance`
- URT-2 직결 전용: `walk`, `calibrate`, `capture-stand`, `pose` 등 나머지

둘 다 꽂혀 있으면 추측하지 않고 `--via stm32`/`--via urt2`를 요구합니다. 한쪽만
꽂힌 흔한 경우에 아무 것도 지정할 필요가 없다는 것이 핵심입니다. 전역 옵션
`--port`, `--via`, `--stm32-port`, `--log`, `--console-timeout`은 하위 명령보다
앞에 옵니다. `console`은 하위 명령이 없는 펌웨어 명령(`ping`, `read`, `move`,
`echo`, `uarttest`, `busprobe`)과 대화형 프롬프트, 절차 파일용으로 남았습니다.

ST-LINK만 연결한 상태에서 확인한 결과입니다.

```text
$ spotctl targets          -> ID 1..12 target=... , exit 0
$ spotctl profile          -> Profile: speed=3400 acceleration=254
$ spotctl balance status   -> Balance: on, mode: full, IMU: available
$ spotctl imu status       -> IMU log: off
$ spotctl scan             -> ID 1..12 timeout, exit 1
$ spotctl walk --cycles 1  -> error: 'walk' needs a URT-2 ... , exit 1
$ spotctl calibrate        -> error: 'calibrate' needs a URT-2 ... , exit 1
$ spotctl scan --max-id 12 -> error: the STM32 console always scans 1..12
$ spotctl --via urt2 status-> error: no likely URT-2 port found
```

`--leg`, `--min-id/--max-id`처럼 이 링크에서 의미가 없는 옵션은 조용히
무시하지 않고 거절합니다. 펌웨어가 궤적을 소유하므로 속도·가속도는 `profile`로
설정합니다. 단위 시험은 합계 `94 PASS`입니다. 서보를 실제로 움직이는 명령은
서보 전원 연결 후 `spotctl scan`으로 12축 OK를 먼저 확인하고 진행합니다.

## 2026-08-08 서보 버스 무응답 재발 (조사 중)

`spotctl`이 STM32로 정상 분기되고 콘솔 명령은 모두 응답하지만 서보 버스가
죽어 있습니다.

```text
$ spotctl scan     -> ID 1..12 전부 timeout, servo_error=0x00
$ spotctl stand    -> ERROR: missing servo; servo=1, bus=timeout
$ spotctl console send busprobe 1
BUSPROBE RX: no bytes
```

2026-08-06 기록과는 증상이 다릅니다. 그때는 `BUSPROBE RX (2): FF 00`처럼 일부
바이트가 들어와 수신 타이밍(overrun) 문제였지만, 이번에는 **한 바이트도 들어오지
않습니다**. STM32는 요청을 내보내고 있으므로 PA10으로 돌아오는 신호가 아예 없는
물리 계층 문제로 봅니다. `targets`, `profile`, `balance status`가 정상인 것은
이들이 서보 버스를 쓰지 않기 때문입니다.

이 과정에서 분류기의 빈틈을 또 하나 찾았습니다. `BUSPROBE RX: no bytes`에는
`ERROR:` 접두사도 `FAIL`도 없어 성공으로 분류되어 종료 코드 `0`이 나왔습니다.
이 문장을 실패로 판정하도록 고쳤고, 정상 응답인 `BUSPROBE RX (6): FF FF 01 02
00 FC`는 영향받지 않습니다. 단위 시험 `94 PASS`.

소프트웨어 쪽은 다음을 확인해 용의선상에서 제외했습니다.

- 보드에 올라간 빌드는 최신입니다. 콘솔 `help`에 `trotplace`, `trot2`, `jump`가
  모두 있습니다.
- `MX_USART1_UART_Init()`은 1 Mbps, 8-N-1, `UART_MODE_TX_RX`이고 CubeMX
  재생성 대비 런타임 보정도 남아 있습니다.
- `HAL_UART_MspInit()`의 PA9/PA10은 `GPIO_MODE_AF_PP` + `GPIO_AF7_USART1`
  입니다.
- 2026-08-06 수정 이후 `servo_bus.c`는 변경되지 않았습니다. 이번 보행·점프
  커밋은 `gait_policy.h`, `robot.*`, `app_console.c`, `main.c`만 건드렸습니다.

`uarttest`를 URT-2가 붙은 채로 실행하면 `RX status=3`(`HAL_TIMEOUT`)이 나오는데,
PA9는 URT-2로 들어가고 PA10은 URT-2에서 나오므로 loopback 경로가 없어 당연한
결과입니다. 이 명령은 URT-2를 분리하고 PA9-PA10을 직결해야 의미가 있습니다.
마찬가지로 `--via urt2 scan`은 URT-2를 Mac USB에 직접 꽂아야 합니다.

### `linestate` 진단 명령 추가

`uarttest`와 `--via urt2 scan`은 케이블을 옮겨야 의미가 있고, URT-2 LED를 눈으로
확인하기 어려운 상황이 있어 계측기 없이 배선을 판별할 명령을 추가했습니다.
`GPIOx_IDR`은 핀이 alternate function 모드여도 실제 패드 레벨을 반영하므로,
UART 설정을 전혀 건드리지 않고 PA9/PA10의 대기 레벨을 50ms 동안 샘플링합니다.
UART 유휴 상태는 HIGH입니다.

```text
# linestate
LINESTATE samples=... PA10_RX_high=100% PA9_TX_high=100%
PA10 idle high: URT-2 is driving the line; suspect servo power or the TTL bus
```

첫 측정은 `PA10_RX_high=100%`였습니다. 그런데 이 결과만으로는 결론을 낼 수
없었습니다. PA10은 `GPIO_NOPULL`이라 **플로팅이어도 안정적으로 HIGH로 읽힐 수
있기** 때문입니다. 또한 이때 출력하던 "suspect servo power or the TTL bus"는
바로 위의 분리 시험에서 이미 정상으로 확인된 구간을 가리키는 잘못된 안내였습니다.

그래서 풀다운을 걸어 판별하도록 고쳤습니다. 유휴 UART 라인도 HIGH, 플로팅 입력도
HIGH지만, 풀다운을 걸면 **실제로 구동 중인 핀만 HIGH를 유지**하고 열린 핀은
LOW로 끌려갑니다.

```text
# linestate
LINESTATE PA10 float=100% pulldown=100% | PA9 pulldown=0%
PA10 driven high: URT-2 TX reaches the MCU; the request path PA9 -> URT-2 is
the remaining suspect
```

| PA10 pulldown | 해석 |
|---|---|
| `100%` | URT-2 TX가 실제로 MCU까지 도달 → 남은 의심은 PA9 → URT-2 요청 경로 |
| `0%` | 아무도 구동하지 않음 → RX 배선, GND, URT-2 로직 전원 개방 |
| 중간값 | 약한 구동 또는 노이즈 → GND, 3.3V 레벨 스위치 |

PA9는 정상이라면 URT-2의 입력 핀을 구동하므로 풀다운을 따라 LOW가 됩니다.
HIGH로 읽히면 무언가가 PA9를 구동한다는 뜻이라 TX/RX 교차를 의심하지만, URT-2
입력의 풀업으로도 같은 값이 나올 수 있어 단정하지 않고 힌트로만 출력합니다.

측정 동안 두 핀을 잠시 일반 입력으로 바꿨다가 USART1 alternate function으로
되돌립니다. 콘솔 명령 실행 중에는 서보 버스가 유휴 상태이므로 중단되는 전송은
없습니다. 이 명령은 서보를 움직이지 않습니다.

빌드는 CubeIDE에 포함된 GNU Arm 14.3으로 전체 컴파일·링크까지 확인했습니다.
`Src`와 HAL 드라이버 전체에서 새 코드의 경고는 없고, 남은 경고 3개는 ST의
`stm32f4xx_hal_flash_ex.c`에 원래 있던 `unused parameter 'Banks'`입니다.

### 분리 시험 결과: 범위가 STM32 ↔ URT-2 헤더로 확정

URT-2를 Mac에 USB로 직접 연결하면 `scan`이 정상입니다. 같은 URT-2를 STM32에
연결하면 시리얼 터미널이든 `spotctl`이든 12축 전부 timeout입니다.

이는 2026-08-06과 동일한 분리 결과이며, 다음이 모두 정상임을 뜻합니다.

- 서보 12개와 각 ID
- 서보 12V 구동 전원
- URT-2 자체와 half-duplex TTL 버스 구간

따라서 남은 범위는 **STM32 USART1 ↔ URT-2 UART 헤더 구간**뿐입니다. 다만
2026-08-06의 원인이었던 수신 타이밍 문제와는 증상이 다릅니다. 그때는
`BUSPROBE RX (2): FF 00`처럼 일부 바이트가 도착했지만 지금은 0바이트이고,
`servo_bus.c`는 그때 수정 이후 변경되지 않았습니다. 그래서 소프트웨어 재발이
아니라 이 구간의 배선 또는 전원으로 봅니다.

중요한 비대칭이 하나 있습니다. URT-2를 Mac에 꽂으면 USB에서 로직 전원을 받지만,
STM32에 연결할 때는 UART 헤더로 신호만 오가므로 **URT-2가 자체 전원을 따로 받아야
합니다**. Type-C 전원이 빠져 있으면 Mac에서는 정상이고 STM32에서만 무응답인
지금 증상과 정확히 일치합니다. 2026-08-06 기록의 전원 원칙도 Type-C 공급이고
UART 헤더 VCC는 연결하지 않는 구성이었습니다.

### 펌웨어 회귀 여부: 이전 커밋을 플래시해 확인

"보행·점프 커밋 이후 안 되기 시작했다"는 가능성을 실제로 시험했습니다. 마지막으로
서보가 정상 동작하던 커밋 `7fe2cb0`을 그대로 빌드해 플래시했습니다.

```text
$ spotctl console send help    # trot2/trotplace/jump/linestate 0건 → 구 빌드 확인
$ spotctl console send scan
ID 1..12: timeout, servo_error=0x00
```

**구 펌웨어에서도 동일하게 12축 전부 timeout입니다.** 따라서 이번 보행·점프
커밋과 `linestate` 추가는 원인이 아닙니다. `git diff 7fe2cb0..HEAD -- firmware/`
에서도 `servo_bus.c`, `sts3215.c`, `feetech_protocol.c`, `stm32f4xx_hal_msp.c`는
변경분이 0입니다.

확인 후 현재 빌드로 되돌렸습니다. 앞으로 이 판별은 다음으로 재현합니다.

```bash
git worktree add /tmp/spot_good <commit>
# scratchpad/build_any.sh <project-dir> <output-dir> 로 빌드
STM32_Programmer_CLI -c port=SWD mode=UR -w firmware.elf -v -rst
```

CubeIDE 번들 도구의 경로는 다음과 같습니다.

```text
/Applications/STM32CubeIDE.app/Contents/Eclipse/plugins/
  com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.*/tools/bin
  com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.*/tools/bin
```

또한 앞서 "분리 시험으로 범위가 배선으로 확정됐다"고 적은 것은 과장이었습니다.
URT-2를 호스트에 꽂아 동작하는 결과는 STM32 쪽 펌웨어 회귀와도 똑같이
일치하므로, 그것만으로는 펌웨어를 배제할 수 없었습니다. 배제는 위의 구 펌웨어
플래시 시험으로 비로소 성립합니다.

### 시리얼 포트 동시 점유 주의

조사 중 `lsof`로 확인해 보니 시리얼 터미널 앱이 `/dev/cu.usbmodem312103`을
열어둔 채였고 `spotctl`도 같은 포트를 쓰고 있었습니다. macOS의 `/dev/cu.*`는
여러 프로세스가 동시에 열 수 있고 들어오는 바이트는 먼저 읽는 쪽이 가져가므로,
양쪽 다 응답을 반쪽씩 받게 됩니다. 플래시 후 재부팅 배너가 터미널에 안 보였던
것도 이 때문으로 보입니다.

재부팅 자체는 확실히 이루어졌습니다. `7fe2cb0`을 올린 직후 `help`에는
`trot2`/`trotplace`/`jump`/`linestate`가 0건이었고 HEAD 복구 후에는 4건이므로,
같은 보드에서 실행 코드가 실제로 바뀌었습니다.

`Stm32Console`은 이제 포트를 `exclusive=True`로 엽니다. 다만 이 잠금은 다른
`spotctl` 실행만 막고, 잠금을 쓰지 않는 터미널 프로그램은 막지 못합니다.
진단 중에는 한쪽만 열어두고, 의심되면 `lsof /dev/cu.usbmodem...`으로 확인합니다.

터미널 앱을 닫고 포트를 단독 점유한 상태에서 `scan`, `busprobe 1`, `linestate`,
`profile`을 다시 측정한 결과는 이전과 **완전히 동일**했습니다. 따라서 포트 공유가
진단을 왜곡하지는 않았고 지금까지의 측정값은 유효합니다.

`balance status`의 `IMU: unavailable`은 BNO055를 의도적으로 단선한 결과입니다.
BNO055는 NUCLEO `3V3`에서 전원을 받으므로(README 배선표) 서보 외부 전원과는
무관하며, 이 증상은 전원 계통의 단서가 아닙니다.

IMU가 없는 상태에서 부팅하면 `balance_required`가 참으로 남아 `trot`, `trot2`,
`jump`가 잠깁니다. 개방 루프로 시험하려면 `balance off`를 명시적으로 실행해야
합니다. `robot_set_balance_enabled(false)`가 `balance_required`도 함께 내리도록
되어 있어 그때 잠금이 풀립니다. `stand`, `hold`, `relax`, `scan`은 이 검사를
받지 않습니다.

### 원인: URT-2에 자체 전원이 없었음

배선을 확인한 결과 Mac ↔ STM32는 USB-C(ST-LINK), STM32 ↔ URT-2는 점퍼였고
**URT-2의 Type-C에는 아무것도 연결되어 있지 않았습니다.**

앞서 `linestate`에서 두 핀이 모두 풀다운을 이기는 것을 보고 "URT-2에 로직
전원이 있다"고 판단해 무전원 가설을 기각했는데, 이 추론이 틀렸습니다. 전원이
없는 CMOS 칩도 입력 핀에 VDD로 향하는 ESD 보호 다이오드가 있습니다. STM32의
PA9는 유휴 상태에서 3.3V를 push-pull로 내보내므로, 이 전류가 URT-2 입력 →
ESD 다이오드 → URT-2 VDD 레일로 흘러들어 보드를 다이오드 강하만큼 낮은 전압으로
**기생 구동**합니다.

| 관측 | 기생 전원 상태의 설명 |
|---|---|
| PA10 pulldown `100%` | URT-2 출력이 어중간한 VDD로 HIGH 유지 |
| PA9 pulldown `100%` | STM32 TX가 공급원이므로 HIGH |
| `BUSPROBE RX: no bytes` | 정상 전압이 아니라 실제 송수신 불가 |
| Mac 직결 시 12축 정상 | USB가 정규 전원을 공급 |

즉 "풀다운을 이기면 전원이 있다"는 단정이 성립하지 않습니다. 두 핀이 동시에
HIGH인 것은 오히려 무전원 URT-2의 특징적인 서명입니다. `linestate`가 이 경우를
먼저 짚도록 판정 순서를 고쳤습니다.

```text
Both pins held high: check the URT-2 Type-C supply first; an unpowered URT-2
is parasitically pulled high through PA9 and looks like this
```

현재 기체의 URT-2 로직 전원은 NUCLEO `5V` 핀에서 가져가는 구조입니다.
2026-08-06 기록의 Type-C 공급 구성과 다르므로 그 절의 전원 원칙은 현재 배선에
그대로 적용되지 않습니다. 확인 대상은 NUCLEO `5V` → URT-2 `VCC` 배선과 그
레일의 실제 전압입니다.

### `linestate` 측정 결과와 소프트웨어 진단의 한계

```text
LINESTATE PA10 float=100% pulldown=100% | PA9 pulldown=100%
```

두 핀 모두 내부 풀다운(약 40kΩ)을 이기고 HIGH를 유지합니다. URT-2에 로직
전원이 들어와 있다는 것은 확실하므로 Type-C 무전원 가설은 기각합니다. 그런데
이 값은 서로 다른 두 배선 상태와 **모두** 들어맞아 판별이 되지 않습니다.

Feetech UART 헤더는 MCU 신호 이름 기준 표기라 `TX` 핀이 URT-2의 **입력**,
`RX` 핀이 URT-2의 **출력**입니다.

| 가설 | PA9 (MCU TX) 연결 | PA10 (MCU RX) 연결 | 풀다운 측정 예상 |
|---|---|---|---|
| 정상 배선 | URT-2 입력(풀업 있음) | URT-2 출력(유휴 HIGH) | 둘 다 `100%` |
| TX/RX 교차 | URT-2 출력(유휴 HIGH) | URT-2 입력(풀업 있음) | 둘 다 `100%` |

즉 관측값이 동일해 구분되지 않습니다. PA9를 GPIO 출력으로 LOW를 걸면 갈릴 수
있지만, 교차 상태라면 URT-2 출력과 푸시풀끼리 맞붙게 되어 드라이버 손상 위험이
있으므로 시도하지 않습니다.

여기서부터는 배선을 실제로 만져야 합니다. 남은 가설이 TX/RX 교차이므로 가장
빠른 판별은 **두 신호선을 서로 바꿔 꽂고 `scan`을 다시 실행**하는 것입니다.
공구 없이 즉시 확인되며, 되면 그것이 원인이고 안 되면 원위치한 뒤 URT-2를
분리하고 PA9-PA10 직결 `uarttest`로 MCU 쪽을 확인합니다.

함께 확인할 항목은 다음과 같습니다.

1. 신호 레벨 스위치 `3.3V`
2. STM32와 URT-2의 공통 GND (별도 전원이므로 GND가 빠지면 정확히 이 증상)
3. UART 헤더 VCC와 Type-C 동시 공급 금지
4. 점퍼선 접촉 불량 (Mac 연결로 옮길 때 헤더 쪽만 뽑았다면 그 자리)

## 2026-08-08 해결: PA9/PA10이 교차 연결되어 있었음

URT-2 헤더에서 두 신호선을 서로 바꿔 꽂자 `scan`이 즉시 12축 OK로 돌아왔습니다.
원인은 **TX/RX 교차**입니다.

올바른 결선은 교차입니다. URT-2 UART 헤더의 표기는 URT-2 자신의 신호 기준이라
`TX`가 출력, `RX`가 입력입니다.

```text
PA9  (USART1_TX, MCU 출력) → URT-2 RX (입력)
PA10 (USART1_RX, MCU 입력) → URT-2 TX (출력)
```

펌웨어 README에 `TX-TX`, `RX-RX`로 적혀 있던 서술이 틀렸고 이를 고쳤습니다.

### 결정적 단서는 URT-2 `TX1` LED였습니다

`scan` 중 `TX1`이 **점멸하지 않는다**는 관측 하나로 범위가 확정됐습니다. URT-2가
서보 버스로 요청을 내보내지 않는다는 뜻이므로, URT-2가 STM32의 요청을 애초에
받지 못하고 있다는 결론이 나옵니다. 교차 상태에서는 STM32 TX가 URT-2의 출력
핀을 밀고 있고 URT-2의 입력은 MCU 수신 핀에 물려 아무도 구동하지 않으므로
정확히 이 증상이 됩니다.

2026-08-06 성공 기록에 `TX1`, `RX1`이 함께 점멸했다고 적혀 있었는데, 그 대조가
가장 값싸고 결정적인 판별이었습니다. 다음부터는 이 LED 확인을 `busprobe`나
`linestate`보다 **먼저** 합니다.

### 철회하는 결론 두 가지

이번 조사에서 세웠던 다음 두 결론은 실기에서 반증됐습니다. 기록은 남기되
결론으로 인용하지 않습니다.

1. **"URT-2 무전원 → 기생 급전"** — 별도 USB 전원을 연결해 LED가 켜진 상태에서도
   12축 전부 timeout이 그대로였습니다. `linestate`의 두 핀 HIGH는 무전원의
   서명이 아니었습니다.
2. **"URT-2 로직 전원을 NUCLEO 5V로 바꾼 것이 회귀 원인"** — 전원 경로는 원인이
   아니었습니다.

### `linestate` 정상 기준값과 그 한계

수리 후 12축이 정상 동작하는 상태에서 `linestate`를 측정해 기준값을 확보했습니다.

```text
$ spotctl console send linestate
LINESTATE PA10 float=100% pulldown=100% | PA9 pulldown=100%
```

**고장 상태(교차 배선, 12축 timeout)의 측정값과 완전히 동일합니다.**

| 상태 | PA10 float | PA10 pulldown | PA9 pulldown |
|---|---|---|---|
| 교차 배선, 12축 timeout | `100%` | `100%` | `100%` |
| 정상 배선, 12축 OK | `100%` | `100%` | `100%` |

즉 두 핀이 모두 HIGH인 것은 고장의 서명이 아니라 **건강한 버스의 정상
signature**입니다. 이 값을 보고 "URT-2 무전원"으로 판정했던 것은 정상 상태를
고장으로 읽은 것이었습니다.

정상 배선, 교차 배선, 무전원 세 가지가 모두 같은 값을 내므로 측정 하나로는
셋을 가를 수 없습니다. 이 분기에서 원인을 단정하지 않고 `scan` 실행과 `TX1`
LED 확인, 선 교차 시험을 안내하도록 고쳤습니다.

한편 2026-08-08 초기 측정의 `PA9 pulldown=0%`는 정상도 교차도
아닌 **세 번째 물리 상태(PA9 선 개방)**였던 것으로 보입니다. 정상 배선에서도
`100%`가 나오므로 `0%`는 아무도 구동하지 않는 상태뿐입니다.

교차 시험은 안전합니다. 이미 교차 상태라면 두 출력이 서로 밀고 있는 것이므로
바꿔 꽂아서 더 나빠질 수 없습니다.

### 소프트웨어가 무죄임을 확인한 방법

배선을 의심하기 전에 STM32 → URT-2 송신 경로를 정적으로 검증했습니다.
`feetech_protocol.c`를 호스트에서 컴파일해, URT-2 직결로 12축을 구동하는
`servo/protocol.py`와 20,004건을 대조했고 불일치 0건이었습니다.

```text
ping 1        ff ff 01 02 01 fb
read 1        ff ff 01 04 02 38 0f b1
sync_write    44 bytes, ff ff fe 28 83 2a 02 01 ... 0c 54 08 5a
```

USART1 보율도 확인했습니다. HSI 16MHz, PLL 없음이므로 PCLK2가 16MHz이고
`BRR=0x10`이 되어 분주 오차 없이 정확히 1.000000 Mbps입니다. 같은 클럭을 쓰는
USART2 115200 콘솔이 안정적이므로 HSI 드리프트도 배제됩니다.

### 재발 시 최소 점검 순서 (개정)

```text
1. scan 중 URT-2 TX1/RX1 LED 관찰    # 가장 값싸고 결정적
   - 둘 다 미점멸 → STM32 → URT-2 요청 경로 (배선/GND/레벨)
   - TX1만 점멸  → 서보 전원 또는 서보 버스
   - 둘 다 점멸  → PA10 복귀선
2. 두 신호선을 서로 바꿔 꽂고 scan   # 교차 판별, 10초
3. 점퍼 2선을 URT-2에서 빼 서로 직결 후 uarttest   # 선 자체 판별
4. spotctl --via urt2 scan           # URT-2를 Mac에 직결해 서보 확인
5. busprobe 1 → FF FF 01 02 00 FC
6. scan 3회 반복 후 read 1
```

## 2026-08-08 IMU를 BNO086(SPI)으로 교체

BNO055(I2C1, PB8/PB9)를 BNO086(SPI1)으로 바꿨습니다. 아직 실기 검증 전이며 현재
확인된 것은 전체 빌드·링크까지입니다.

```text
BNO086 SCK  → D13 / PA5      BNO086 CS   → D10 / PB6
BNO086 SO   → D12 / PA6      BNO086 INT  → D7  / PA8
BNO086 SI   → D11 / PA7      BNO086 RST  → D3  / PB3
                             BNO086 WAK  → D4  / PB5
```

### `D2`를 쓰지 않은 이유

처음 배선안은 `INT`가 `D2`였는데, `D2`는 `PA10`이고 이 핀은 URT-2에서 돌아오는
`USART1_RX`입니다. 그대로 뒀으면 이번에 이틀 걸려 고친 서보 버스가 다시 죽고
증상도 `BUSPROBE RX: no bytes`로 동일하게 나왔을 것입니다. `INT`를 `D7`(`PA8`)로
옮겼습니다.

`D13/PA5`는 사용자 LED `LD2` 핀이기도 해서, B1 버튼이 LD2를 토글하던 코드를
제거했습니다. 남겨 두면 SPI 프레임마다 SCK를 밀게 됩니다.

### Game Rotation Vector 선택

Rotation Vector(`0x05`)가 아니라 Game Rotation Vector(`0x08`)를 구독합니다.
전자는 지자기를 융합에 포함하는데 STS3215 12개의 영구자석이 IMU 옆에 있어
자기 방위가 오염됩니다. 균형 제어는 roll/pitch만 쓰고 이 둘은 자이로/가속도만으로
나옵니다. 대신 Yaw는 절대 방위가 아니라 전원 투입 시점 기준 상대값입니다.

### 균형 루프의 블로킹 읽기 제거

BNO055 경로는 `BNO055_ReadAttitude()`가 균형 루프 안에서 블로킹 I2C 읽기를
했습니다. 새 드라이버는 `H_INTN`이 내려왔을 때만 SPI를 건드려 캐시를 갱신하고,
`bno086_read_attitude()`는 캐시를 읽습니다. 대기 중인 리포트가 없으면 GPIO 한 번
읽는 비용입니다.

`H_INTN`은 EXTI가 아니라 폴링합니다. 메인 루프와 자세 리더가 둘 다 레벨을 보므로
ISR을 두면 공유 상태만 늘고 샘플까지의 경로는 짧아지지 않습니다. 제어 루프를
타이머 기반으로 바꾸면 그때 EXTI로 옮깁니다.

### 빌드 관련 주의

`.ioc`에는 SPI1이 없습니다. `bno086.c`가 GPIO·클럭·`HAL_SPI_Init()`을 직접 하므로
CubeMX 재생성이 이 코드를 지우지는 못하지만, 다음 둘은 되돌아갑니다.

```text
Inc/stm32f4xx_hal_conf.h  →  #define HAL_SPI_MODULE_ENABLED
Drivers/STM32F4xx_HAL_Driver/{Src,Inc}/stm32f4xx_hal_spi.{c,h}
```

HAL SPI 드라이버는 프로젝트에 없어서 `STM32Cube_FW_F4_V1.28.3`에서 복사했습니다.
같은 판임은 `stm32f4xx_hal_i2c.c`가 바이트 단위로 동일한 것으로 확인했습니다.

### 첫 실기 시험: BNO086 무응답 (조사 중)

플래시 후 `trot2`가 `ERROR: IMU balance error`로 거부됐습니다. `balance status`는
`IMU: unavailable`, 부팅 배너는 `BNO086 unavailable: not present`였습니다.
`balance=off/full`에서 balance가 off인 것이 단서였습니다. IMU가 잡히면
`robot_set_attitude_reader()`가 `balance_enabled`를 켜므로, off라는 것은 애초에
리더가 등록되지 않았다는 뜻입니다.

부팅 배너는 `Stm32Console.sync()`가 버리므로, ST-LINK로 SWD 리셋을 걸면서 포트를
열어둬 배너를 잡았습니다. 버튼을 누를 필요가 없습니다.

```bash
STM32_Programmer_CLI -c port=SWD mode=HOTPLUG -rst   # 포트를 연 채로
```

### 진단 명령 `imuprobe`와 `spitest` 추가

`bno086_init()`은 모든 실패를 `not present` 하나로 뭉개서 원인을 못 가립니다.
URT-2 때 `busprobe`/`linestate`가 했던 역할을 IMU에도 만들었습니다.

측정 결과는 다음과 같습니다.

```text
IMUPROBE INT float=100% pulldown=100% in_reset=100%
IMUPROBE blind read (24 tries): blank 00 00 00 00
IMUPROBE FAIL: H_INTN never went low after reset
```

세 항목의 의미는 이렇습니다.

| 관측 | 해석 |
|---|---|
| `pulldown=100%` | PA8을 뭔가가 HIGH로 잡고 있음 (구동 또는 풀업) |
| `in_reset=100%` | RST를 누른 채로도 HIGH — 센서 출력이 아니라 **보드 풀업**이 잡고 있거나 RST가 센서에 도달하지 않음 |
| `blind read blank` | H_INTN을 무시하고 SPI로 24회 읽어도 MISO에 아무것도 없음 |

`in_reset` 항목이 중요합니다. 센서는 리셋 중 출력을 high-Z로 두므로, 센서가
INT를 구동하고 있었다면 이때 풀다운을 따라 내려가야 합니다. 내려가지 않았으므로
INT 레벨은 센서 상태에 대해 아무 정보도 주지 못합니다.

`blind read`는 INT 배선을 완전히 우회합니다. SPI 모드로 부팅한 센서는
advertisement가 큐에 있으므로 INT 선이 끊겨 있어도 바이트가 나와야 합니다.
전부 `00`이므로 **센서가 SPI로 말하고 있지 않습니다.**

### SPI1을 `.ioc`로 옮김

처음에는 `bno086.c`가 GPIO·클럭·`HAL_SPI_Init()`을 직접 했습니다. 이 구성이
원인일 수 있다는 의심이 나와, SWD로 실리콘의 레지스터를 직접 읽어 확인했습니다.

```text
RCC_APB2ENR  0x00005010   bit12 SPI1EN = 1
SPI1_CR1     0x00000357   SPE MSTR SSM SSI = 1, CPOL=1 CPHA=1, BR=/8, 8-bit MSB
SPI1_SR      0x00000002   TXE only; MODF/OVR 없음
GPIOA_MODER  0xA828A8A0   PA5/PA6/PA7 = AF, PA8 = input
GPIOA_AFRL   0x55507700   PA5/PA6/PA7 = AF5 (SPI1)
GPIOB_MODER  0x000A1640   PB3/PB5/PB6 = output
GPIOB_ODR    0x00000068   RST=1 WAKE=1 CS=1
```

전부 의도대로였습니다. 특히 `PB3`이 기본 대체기능인 `JTDO`에서 빠져나와 output으로
잡혀 있고 HIGH를 출력 중인 것도 확인돼, "RST가 구동되지 않는다"는 가설은
소프트웨어 측면에서는 배제됩니다. 즉 **무응답의 원인은 SPI 설정이 아닙니다.**

그럼에도 `servo_bus`가 `huart1`을 넘겨받는 구조와 어긋나므로 `.ioc`로 옮겼습니다.
이제 `SPI1`과 제어선 4개(`IMU_CS`/`IMU_INT`/`IMU_RST`/`IMU_WAKE`)가 `.ioc`에 있고
초기화는 CubeMX 생성 코드가, `bno086.c`는 `hspi1` 핸들만 빌려 씁니다.

이전과 동작이 같음은 레지스터를 다시 읽어 확인했습니다. `SPI1_CR1`이 `0x0317`로
읽히는 순간이 있는데 이는 `SPE`(bit6)만 다른 것이고, HAL은 `HAL_SPI_Init()`이
아니라 첫 전송에서 `SPE`를 켜기 때문입니다. 전송을 한 번 하면 `0x0357`로 이전과
완전히 같아집니다.

### `deselected read`: CS를 바꿔도 MISO가 변하지 않음

```text
IMUPROBE blind read (24 tries): blank 00 00 00 00
IMUPROBE deselected read: 00 00 00 00
```

살아 있는 센서라면 CS가 HIGH일 때 MISO를 놓으므로 두 값이 달라야 합니다.
동일하다는 것은 **MISO(D12/PA6)가 구동되는 출력에 닿아 있지 않다**는 뜻입니다.
`GPIOA_IDR`에서 PA6이 유휴 시 HIGH로 읽히는 것도 풀업만 걸린 개방 입력과
일치합니다.

### `spitest` PASS: STM32 쪽 무죄 확정

```text
SPITEST PASS: SPI1 PA5/PA6/PA7 loopback OK
```

첫 시도는 `D12`-`D13`을 연결해 MISO를 SCK에 물린 상태였고, `SPITEST FAIL`이
나오면서 "SPI1 자체 문제"라고 잘못 안내했습니다. 끼우지 않은 점퍼는 고장난
주변장치와 똑같이 패턴 시험을 실패시키므로, 도통을 먼저 확인하지 않은 판정은
링크의 반대편을 가리킵니다. 이후 DC 도통 검사를 앞에 두도록 고쳤고, 같은 상태를
다시 돌리자 "D11 and D12 are not connected"로 정확히 잡아냈습니다.

`D11`-`D12`로 옮긴 뒤 PASS. 이로써 MCU 측은 레지스터 실측과 실제 데이터 왕복
두 방향에서 검증됐고, 남은 범위는 **센서와 그 사이 배선**입니다.

교훈은 URT-2 때와 같습니다. 어떤 loopback/probe든 **먼저 배선이 도통하는지를
확인해야** 그 결과에 의미가 생깁니다.

### `SO`/`SI` 교차: 세 번째 배선 방향 문제

두 선을 서로 바꿔 꽂자 `imuprobe`의 값이 처음으로 변했습니다.

```text
before:  deselected read: 00 00 00 00
after :  deselected read: FF FF FF FF
```

CS를 올렸을 때 센서가 MISO를 놓고, 내렸을 때 구동합니다. **센서가 SPI로
응답하기 시작했습니다.** 그 전까지 CS를 어떻게 하든 MISO가 같았던 것은 SO가
MCU의 MISO가 아니라 MOSI에 물려 있었기 때문입니다.

이 프로젝트에서 배선 방향으로 막힌 것이 이번이 세 번째입니다.

| 시점 | 문제 |
|---|---|
| URT-2 | `TX-TX`, `RX-RX`로 직결 (실제로는 교차가 정답) |
| BNO086 INT | SPI 헤더가 아닌 반대쪽 헤더에 삽입 |
| BNO086 SO/SI | 서로 바뀜 |

공통점은 **눈으로 확인한 배선을 신뢰했다는 것**입니다. 세 번 모두 측정으로만
갈렸습니다. 새 배선은 연결 후 도통과 방향을 먼저 찍고 시작합니다.

### 남은 문제: advertisement 없음

센서는 길이 0 헤더로 "보낼 것 없음"을 답합니다. 리셋 직후라면 advertisement가
큐에 있어야 하므로, `D3/PB3`의 리셋 펄스가 센서에 도달하지 않는 것으로 봅니다.
이 보드는 `RST` 핀이 위쪽 SPI 헤더와 아래쪽 I2C 헤더 양쪽에 있어 INT와 같은
오삽 가능성이 있습니다.

### CEVA 공식 `sh2` 드라이버로 교체: 구현 오류 배제

직접 작성한 최소 SHTP 클라이언트가 원인일 가능성을 제거하기 위해, CEVA의 공식
`sh2` 드라이버(Apache 2.0)를 `Drivers/sh2/`에 벤더링하고 SHTP/SH-2 계층을
통째로 교체했습니다. `bno086.c`에는 SPI 전송 계층(CS/RST 제어, 바이트 왕복)만
남기고 `sh2_Hal_t`로 넘깁니다. SparkFun 라이브러리도 같은 드라이버를 씁니다.

결과는 동일했습니다. 그리고 이때 판정을 가르는 사실을 하나 얻었습니다.

`sh2_setSensorConfig()`가 `SH2_OK`를 돌려주는 것은 통신 확인이 아닙니다.
`setSensorConfigOp`에는 응답 핸들러가 없어 명령을 쓰기만 하고 즉시 성공을
반환합니다. **응답을 실제로 기다리는 명령**으로 확인해야 합니다.

```text
BNO086 unavailable: timeout (prodIds=-6, resets=0)
```

`sh2_getProdIds()`는 응답 핸들러가 있는 명령이고, `-6`은 `SH2_ERR_TIMEOUT`입니다.
**검증된 벤더 구현으로도 센서가 응답 요구 명령에 답하지 않습니다.**

### 벤더 코드 국소 수정

`getProdIdOp`에는 `timeout_us`가 없어 `opProcess()`가 무한 루프를 돕니다.
응답하지 않는 부품에서는 펌웨어가 부팅 중 영구히 멈추고, 콘솔도 서보 버스도
함께 죽습니다. `.timeout_us = 2000000`을 넣고 `Drivers/sh2/LOCAL_CHANGES.md`에
기록했습니다. 재벤더링 시 다시 적용해야 합니다.

### 현재 결론

| 항목 | 상태 |
|---|---|
| STM32 SPI1 | ✅ 레지스터 실측 + loopback PASS |
| 배선 6선 | ✅ 전원 내린 상태 도통, SparkFun 공식 표와 일치 |
| PS0/PS1 = SPI | ✅ 공식 조합표 + 칩 핀에서 3.3V |
| BOOT | ✅ 3.3V (부트로더 모드 아님) |
| RST·CS 도달 | ✅ MISO 응답 변화로 확인 |
| SHTP 구현 | ✅ CEVA 공식 드라이버로 교체해도 동일 |
| **센서 응답** | ❌ `sh2_getProdIds` 타임아웃 |

소프트웨어와 배선에서 확인할 수 있는 것은 전부 확인했습니다. 남은 것은 부품
자체이며, 교체 전에 Arduino/ESP32와 SparkFun 라이브러리로 부품만 따로 시험하는
것이 가장 값싼 확인입니다.

### BNO055로 복귀, 그리고 센서 세 개가 모두 무응답

BNO086이 끝내 응답하지 않아 BNO055로 되돌렸습니다. 펌웨어는 부팅 시 붙어 있는
센서를 자동 선택합니다. BNO055는 I2C1, BNO086은 SPI1이라 충돌하지 않습니다.
BNO055 탐색을 먼저 하는 것은 선호가 아니라 시간 때문입니다 — 없는 BNO086은
SH-2 타임아웃으로 수 초가 걸립니다.

그런데 **BNO055 두 개를 연달아 물려도 I2C에 아무것도 응답하지 않았습니다.**
`i2cscan`으로 주소 1~126을 전수 조사해도 0개입니다.

### MCU 쪽 I2C1 전면 검증

SPI1 때와 같은 방식으로 실리콘을 직접 읽었습니다.

```text
RCC_APB1ENR bit21 I2C1EN = 1
I2C1_CR1   0x00000001   PE = 1
I2C1_CR2   0x00000010   FREQ = 16 (PCLK1과 일치)
I2C1_CCR   0x00000050   100 kHz
I2C1_TRISE 0x00000011   16MHz 기준 정확
I2C1_SR1   0x00000000   래치된 오류 없음 (AF/BERR/ARLO 모두 0)
I2C1_SR2   0x00000000   버스 busy 아님
GPIOB PB8/PB9 = AF4 (I2C1)
```

물리 계층도 확인했습니다. `PB8`/`PB9`를 open-drain 출력으로 바꿔 LOW로 끌면
둘 다 `0`으로 내려가고, 놓으면 풀업으로 `1`로 복귀합니다. **MCU가 START 조건을
만들 수 있다는 뜻**이며 핀 손상이 아닙니다.

버스 유휴 레벨도 정상입니다. 센서를 연결하기 전에는 `PB8`/`PB9`가 `0`이었고
연결 후 `1`이 됐습니다. 풀업은 센서 보드 위에 있고 그 보드 전원에서 나오므로,
**센서 보드에 전원이 있고 두 신호선이 MCU까지 이어져 있다**는 증거입니다.
`3V3`도 NUCLEO와 센서 양쪽에서 3.3V로 실측했습니다.

### 해결: BNO055 동작, BNO086만 불량

배선을 다시 잡은 뒤 `i2cscan`이 응답을 잡았습니다.

```text
I2CSCAN found 1: 0x29
BNO055 NDOF OK at 0x29
Yaw=281.1, Roll=-1.2, Pitch=0.0 deg    (4초에 39 샘플 = 10Hz)
```

`COM3`를 연결하지 않아 주소가 `0x29`로 잡혔습니다. 펌웨어가 두 주소를 모두
탐색하고 칩 ID로 확인하므로 그대로 동작합니다.

**이것이 "공통 원인" 가설을 기각합니다.** 조사 중에 센서 셋이 연달아 침묵하자
전원이나 조립체 쪽의 공통 원인을 의심했는데, 같은 NUCLEO·같은 3V3/GND·같은
케이블에서 BNO055가 정상 동작합니다. 침묵의 원인은 각각 달랐습니다.

| 센서 | 실제 원인 |
|---|---|
| BNO086 | 부품 불량 — 아래 참조 |
| BNO055 1차 시도 | 배선 |
| BNO055 최종 | 정상 |

BNO086은 전원·PS0/PS1·BOOT·배선 6선·MCU SPI1을 모두 측정으로 배제하고 CEVA
공식 드라이버로 교체해도 응답하지 않았습니다. I/O 패드는 CS와 RST에 정상
반응하는데 내부 로직만 죽어 있는 형태로, ESD나 과전압으로 다이가 손상됐을 때
흔한 패턴입니다. `PS0`/`PS1` 솔더 점퍼 작업 중 손상됐을 가능성이 높습니다.

다음에 BNO086을 다시 시도할 때는 **납땜 전에 기본 I2C 모드로 동작을 먼저
확인**해 기준점을 만들고, ESD 대책(접지 인두)을 갖춘 뒤 점퍼를 닫습니다.

### BNO055 부호 검증

```text
수평:  Roll=-0.7  Pitch=0.5 deg
```

앞으로 숙였을 때 Pitch가 음수로 나와 `BNO055_PITCH_SIGN`을 `-1`로 뒤집었습니다.
Roll은 오른쪽 `+`, 왼쪽 `-`로 규약과 일치해 그대로 두었습니다. 수정 후 실기에서
앞으로 숙임 → Pitch 양수를 재확인했습니다.

`imu on`은 10Hz로 스스로 출력하므로 `spotctl console send`로는 볼 수 없습니다.
`spotctl console watch imu on`을 사용합니다.

### 첫 궤적 동작 성공 (거치대 고정, 무부하)

IMU 복구 후 처음으로 다리 궤적이 끝까지 돌았습니다. **몸체를 거치대에 고정한
상태이므로 보행 시험이 아니라 궤적 시험입니다.**

```text
Balance: on, full, IMU available
Last error:  Roll=-1.3  Pitch=0.4 deg
Peak:        Roll=1.3   Pitch=0.5  J1=0.1  Knee=5.2  late=0
Step sync:   barriers=20  wait=9504ms  peak_error=314ticks
Profile:     speed=800  acceleration=80
```

해석에 주의할 점이 둘 있습니다.

**자세 오차가 작은 것은 균형 제어의 성과가 아닙니다.** 몸체가 고정돼 IMU가
움직임을 거의 보지 못했으므로 균형 루프는 사실상 시험되지 않았습니다. Roll/Pitch
1.3° 이내라는 값을 보행 성능의 근거로 인용하면 안 됩니다.

**배리어 대기가 깁니다.** `barriers=20`에 `wait=9504ms`, 배리어당 평균 475ms를
기다렸고 `peak_error`는 314 tick입니다. 서보가 명령 궤적을 상당히 뒤따라오고
있고 step barrier가 그것을 흡수했습니다. `late=0`은 지연 frame이 없었다는
뜻이지 여유가 있었다는 뜻이 아닙니다. **이 값은 무부하에서 나온 것이므로 체중이
실리면 나빠집니다.**

`profile 800 80`은 최대치(3400/254) 대비 낮은 설정입니다. 조정 방향은 주기를
늘려 서보에 시간을 주거나, profile speed를 올리는 것입니다.

### 다음에 확인할 것

1. 하중을 실은 상태에서 같은 궤적 — 배리어 대기와 `peak_error`가 얼마나 늘어나는지
2. 그때 비로소 균형 루프가 시험됩니다. 몸체가 움직여야 IMU 오차가 생깁니다
3. `profile`을 올렸을 때 배리어 대기가 줄어드는지, 대신 서보 온도/전류가 어떤지

### 조사에서 얻은 교훈

배선 방향 문제가 네 번 나왔고 전부 눈으로는 멀쩡해 보였습니다.

| 시점 | 문제 |
|---|---|
| URT-2 | `TX-TX`, `RX-RX` 직결 (교차가 정답) |
| BNO086 INT | SPI 헤더가 아닌 반대쪽 헤더에 삽입 |
| BNO086 SO/SI | 서로 바뀜 |
| `spitest` 점퍼 | `D11`-`D12` 대신 `D12`-`D13` |

마지막 항목 때문에 `spitest`가 한 번 SPI1을 잘못 지목했습니다. 끼우지 않은
점퍼는 고장난 주변장치와 똑같이 패턴 시험을 실패시킵니다. 그래서 패턴보다
**DC 도통을 먼저 확인**하도록 고쳤습니다. 어떤 loopback/probe든 배선이
도통한다는 것을 먼저 세우지 않으면 그 결과에 의미가 없습니다.

또 하나. 전원이 켜진 상태의 도통 시험은 신뢰할 수 없습니다. 조사 중 `SCK`와
`3V3`이 도통하는 것처럼 보였는데, 두 점이 같은 전위(유휴 HIGH)라 테스터가
도통으로 읽은 것이었습니다. 전원을 내리자 사라졌습니다.

### 아직 확정되지 않은 것

`spitest`(D11-D12 직결 loopback)를 점퍼 없이 돌렸더니 바이트 0~3
(`00 FF 55 AA`)이 왕복하고 `0x01`에서 실패했습니다. 점퍼가 없으므로 이는 인접한
PA6/PA7 점퍼선 사이 용량성 결합으로 보이며, MISO가 실제로 구동받지 않는다는
방증입니다. **점퍼를 끼운 정식 `spitest`는 아직 수행 전입니다.**

PS1을 3V3으로 납땜한 뒤에도 증상은 동일했습니다. 따라서 인터페이스 선택
가설만으로는 설명되지 않으며, 다음을 순서대로 갈라야 합니다.

1. BNO086 분리 후 D11-D12 직결 → `spitest` — MCU 쪽 SPI 판별
2. 센서 3V3/GND 실측 — 전원 확인
3. SCK/MISO/MOSI/CS 4선 재확인 — 특히 `SO`와 `SI`가 바뀌지 않았는지
4. 브레이크아웃의 PS0/PS1 실제 상태 확인 (제조사마다 점퍼 방식이 다름)

### 실기에서 먼저 확인할 것

1. 브레이크아웃의 `PS1`이 HIGH인지 (기본은 I2C 모드)
2. 부팅 배너가 `BNO086 game rotation vector OK at 200Hz`인지
3. `imu on`으로 부호 확인 — 앞으로 기울였을 때 Pitch가 양수인지. 반대면
   `bno086.c`의 `BNO086_PITCH_SIGN`만 `-1`로 바꿉니다. 균형 게인은 건드리지 않습니다.
4. `scan` 12축 OK — SPI 배선이 서보 버스를 건드리지 않았는지 확인

## 최종 조립 후 다시 확인할 항목

1. period 2.0초, cycles 1에서 발 방향과 기구 간섭 확인
2. 네 발 접지 구간에 실제로 네 발이 지면에 닿는지 확인
3. 몸체 무게가 실린 상태에서 전압, 전류, 온도와 위치 추종 확인
4. period를 2.0 → 1.8 → 1.6초 순서로만 낮추기
5. RR ID 10의 하드웨어 오류 값 `8` 재발 여부 확인
6. 실제 보폭과 몸체 흔들림을 기준으로 hip/lift/crouch 재조정
