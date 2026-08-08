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

## 최종 조립 후 다시 확인할 항목

1. period 2.0초, cycles 1에서 발 방향과 기구 간섭 확인
2. 네 발 접지 구간에 실제로 네 발이 지면에 닿는지 확인
3. 몸체 무게가 실린 상태에서 전압, 전류, 온도와 위치 추종 확인
4. period를 2.0 → 1.8 → 1.6초 순서로만 낮추기
5. RR ID 10의 하드웨어 오류 값 `8` 재발 여부 확인
6. 실제 보폭과 몸체 흔들림을 기준으로 hip/lift/crouch 재조정
