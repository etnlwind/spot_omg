# Spot OMG MuJoCo Preview

저장소 루트의 `somg` 환경에서 URDF를 Canonical Pose (논리 자세)로 확인합니다.

```bash
source .venv/bin/activate
python simulation/mujoco/preview_pose.py stand45
```

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

실제 `spotctl walk`가 사용하는 궤적 생성기를 MuJoCo의 12개 관절에 그대로
연결합니다. macOS의 실시간 Viewer는 `mjpython`으로 실행해야 합니다.

```bash
.venv/bin/mjpython simulation/mujoco/walk.py \
  --gait trot --preset test --cycles 10
```

실제 하드웨어 시험값과 같은 옵션도 사용할 수 있습니다.

```bash
.venv/bin/mjpython simulation/mujoco/walk.py \
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
12개 STS3215 Position Actuator (위치 구동기)가 활성화됩니다. 첫 시험은 안정적인
`test` 프리셋을 사용합니다.

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

| 설정 | 결과 | 전방 X 이동 | 종료 몸체 높이 |
|---|---|---:|---:|
| `test`, 10 cycles | upright (직립 유지) | +0.429 m | 0.222 m |
| 하드웨어 시험용 `power`, 10 cycles | upright (직립 유지) | +0.157 m | 0.276 m |
| `natural`, 5 cycles | fallen (넘어짐) | 유효하지 않음 | 0.035 m |

`natural` 결과는 시뮬레이터 오류가 아니라 현재 Open-Loop Control (개루프 제어)이
질량·마찰 오차와 좌우 기울기를 복구하지 못한다는 진단입니다. IMU Feedback
(IMU 피드백)이나 더 넓은 J1 stance (J1 지지폭)를 적용하기 전에는 `test` 또는
검증된 `power` 설정으로 시작합니다.

현재 물리 초기값은 `2 ms` timestep, 발 미끄럼 마찰 `0.9`, STS3215 최대 토크
`2.942 Nm`, position gain `35`, velocity damping `0.8`입니다. 배터리 위치와
각 링크 질량이 확정되면 다시 맞춰야 합니다.

## 초기 실험 예제

MuJoCo 설치, 관절축, sit/stand 보간을 확인했던 초기 스크립트는
[`examples/`](./examples/)에 보관합니다. 현재 로봇 자세와 보행 검증에는 위의
`preview_pose.py`와 `walk.py`를 사용합니다.
