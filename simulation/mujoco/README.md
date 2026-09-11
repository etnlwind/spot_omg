> V16 펌웨어 공유 C 보행 자세 검증은 `check_walk_stance.py`로 실행합니다. 기존 일반 drive 재생은 V15 비교용입니다. [V16 반영·한계](../../docs/UPDATE-2026-09-09-V16.md).

> 2026-09-09: [Stand 각도 비교 및 보행 정책 제안](../../docs/GAIT-STANCE-2026-09-09.md). 40°/80° 평지 보행 후보, 0.5ms 물리 간격 기준. 실제 설치된 V15 자세는 변경하지 않았습니다.

> 조이스틱 기본 전진 궤적이 V15 후보(앞뒤 보폭 1.6배)로 변경되었습니다. [비교·재생·한계](../../docs/UPDATE-2026-09-08-V15.md). 기존 비교는 `--stride-scale 1`을 사용합니다.

# Spot OMG MuJoCo Preview

## 개선 보행과 기존 보행 비교 UI

V13부터 개선 보행 재생은 Python 궤적 복사본 대신 STM32의 공용 C
`gait_policy_trot5_targets()`를 직접 호출합니다. Python 구현은 탐색과 수치 대조용으로
남겨두었습니다. 전 주기/시작 진폭 비교 통과, C 동역학 20초 재검증에서 약 0.189m/s로
자세를 유지했습니다. `gait_search/v13_shared_c.json`에 결과를 보관합니다.

```bash
conda activate spot_omg
mjpython simulation/mujoco/gait_lab.py
```

**R 재시작 / 1 기존 보행 / 2 개선 보행 / P 또는 Space 일시정지**.
같은 창에서 물리 상태와 제어 이력을 초기화하며, 정지 준비 후 30초씩 재생합니다.
새 STEP 동역학과 11.1V 3S 기준 전원을 사용합니다. 개선 보행은 V13의 별도 `trot5`
명령에도 포함됩니다. 기존 보행 비교는 공용 C drive 최대 전진 입력 1.0입니다.

총 320개 설정 탐색 후 여러 물리 조건으로 검증했습니다. 기준 조건의 정상상태 속도는
기존 0.028 → 개선 0.189m/s, 약 6.7배입니다. 60초 보행 및 별도 조건에서도 자세를
유지했습니다. 다만 정격 토크 초과 비율은 16.4% → 28.1%로 증가했습니다.
전역 최적이나 실물 검증 완료를 의미하지 않습니다.
자세한 조건·선택 기준·한계는 [보행 최적화 보고서](gait_search/REPORT.md)에 있습니다.

현재 전원 기준은 사용자가 확인한 **명목 11.1V의 3S LiPo**입니다. 이를 시뮬레이션
기준 소스 전압으로 사용하며 실제 현재 잔량/전압을 측정한 것은 아닙니다.
아래 초기 물리 실험 기록은 변경 전 12.5V 가정에서 얻은 과거 결과입니다.

## 새 STEP 동역학 보행

```bash
conda activate spot_omg
mjpython simulation/mujoco/cad_physics.py --duration 120 --linear 0.6
python simulation/mujoco/cad_physics.py --check --duration 12 --linear 0.6
```

이 장면은 새 STEP 메시와 임시 관절축을 사용합니다. 몸체는 6자유도로 움직이며
초기 자세 배치 이후에는 관절 위치를 덮어쓰지 않고 `mj_step`으로 계산합니다.
W/S 전후진, A/D 회전, Space 중립이며 지정 시간이 끝나거나 넘어지면 정지합니다.
입력은 키를 놓아도 유지됩니다. IMU 보정은 아래 검증 결과에 따라 기본적으로 꺼져 있습니다.
`--balance`로 공용 IMU 보정을 비교할 수 있으며 `--no-balance`로 명시적으로 끕니다.

반영한 물리 요소:

- 중력 9.81m/s², 자유 몸체, 부품별 질량·무게중심·관성, 배터리 질량.
- 지면/발/다리/몸체 접촉, 비인접 링크 간 충돌, 미끄럼·비틀림 마찰과 감쇠 접촉.
- 관절 가동 범위, 점성 감쇠, 마찰, 모터의 회전 관성 추정치.
- J1/J3 STS3215 및 J2 STS3250의 서로 다른 토크·속도 한계.
- PD 서보, 속도에 따른 토크 감소 근사, 목표 속도·가속도 제한, 명령 지연 20ms.
- 전류 기반 배터리/배선 전압 강하 근사, 50Hz 공용 C drive 정책.

파라미터는 `cad_300mm/physics_parameters.json`에 있으며 기본 추정 총질량은 **4.398kg**입니다.
몸체/전자부품/프레임 1.8kg, 배터리 0.6kg, 다리마다 J1 0.12kg / J2 0.2245kg /
J3 0.155kg입니다. 다리 그룹 질량에 서보 배분이 포함되므로 모터 무게를 다시 더하지 않습니다.
PLA·PETG는 강체로 근사했고 출력 벽 두께·채움 비율을 모르므로 솔리드 CAD 부피에
재료 밀도를 곱하지 않았습니다. 배터리 0.6kg 외의 질량 및 배터리 위치는 추정입니다.
관성은 부품 외곽 상자를 이용한 균일 질량 근사이고, 접촉은 박스·캡슐·구로 단순화했습니다.

모터의 12V 토크·속도·전류 기준은 제조사의 [STS3215 사양](https://www.feetechrc.com/525603.html)과
[STS3250 사양](https://www.feetechrc.com/en/562636.html)을 사용합니다. 서보 제어기와
전압-토크 곡선은 실제 측정이 없는 근사이며, 전압 범위 밖 외삽도 보장되지 않습니다.
정격 토크 초과 비율을 기록하지만 과열·과전류 보호를 재현하지 않으므로 장시간 성능은 미검증입니다.
PLA/PETG 변형·파손·층간 강도, 기어 백래시, 센서 노이즈, 실제 펌웨어 전체 제어 및
배터리 방전 화학은 포함하지 않았습니다. **실물 전체 물리를 완전히 재현한 모델은 아닙니다.**

2026-09-08 검증(기본 추정치):

| 조건 | 결과 |
|---|---|
| 정지, IMU 보정 끔, 5초 | 자세 유지; 평균 바닥 반력 43.144N (무게 43.144N) |
| 전진 0.6, IMU 보정 끔, 12초 | 자세 유지; X +0.113m; 최대 추종 오차 4.11° |
| 전진 0.6, IMU 보정 켬 | 약 3.86초 보행 후 넘어짐 (그 전에 2초 정지 준비) |
| 정지, 시간 간격 2ms / 1ms 비교 | 자세 및 반력 일치 |

보정 켬에서 발생하는 불안정은 새 모델의 축/영점·질량·보정 부호와 이득을 검증해야 하는
결과이며, 실물 원인으로 확정하지 않았습니다. 넘어지지 않도록 정책을 임의로 수정하지 않았습니다.
자유낙하, 질량에 맞는 바닥 반력, 실제 몸체 전진, 영점 배치 및 12관절 재생 검증 5개 통과.
로그 `summary.json` / `frames.json`은 기본 `/private/tmp/spot-cad-physics`에 저장되며
추정값, 자세, 접촉, 침투량, 목표/실제 각도, 토크, 전류·전압 근사를 포함합니다.

## 새 STEP 모델의 치수 기준 (2026-09-08)

`Spot OMG 300mm Frame.step`의 **300mm는 중앙 알루미늄 프레임 길이**입니다.
로봇 전체 길이나 앞뒤 관절 회전축 사이 거리를 의미하지 않습니다. 두 값은 STEP
형상에서 별도로 측정해야 하며 아직 확정하지 않았습니다. 모터 12개가 관절이며
CAD에 관절 정의는 없습니다. 회전축·부품 연결·영점 자세는 별도 검증 대상입니다.

확인된 정보는 `hardware/urdf/spot_omg_300mm_cad.json`에 기록했습니다.
관절 구동용 URDF 및 `spot_omg_scene.xml`은 이전 모델 기준입니다. 기존 파라미터의
`body_joint_length=0.500`과 `body_visual_length=0.500`은 새 STEP의 측정값이
아니며, 이를 프레임 길이인 0.300으로 일괄 치환해서는 안 됩니다.

### 새 STEP 형상 실행

새 STEP의 12개 관절에 공용 C `drive_targets()` 보행 정책을 재생하려면:

```bash
mjpython simulation/mujoco/cad_gait.py --duration 120 --linear 0.6
python simulation/mujoco/cad_gait.py --check --duration 12
```

W/S 전후진, A/D 회전, Space 중립입니다. 입력은 키를 뗀 뒤에도 유지됩니다.
지정 시간 후 마지막 자세로 정지하며 창을 닫으면 종료합니다.
몸체를 고정하고 목표 각도를 직접 적용하는 **기구학적 정책 재생**입니다.
접촉·추진력·토크·균형 제어를 계산하는 동역학 보행 실험은 아닙니다.
화면의 바닥은 시각적 기준이며 발이 바닥에 닿도록 몸체 높이를 보정하지 않습니다.

`cylinder_candidates.json`의 CAD 원통면에서 hip bearing 및 모터 축 후보를 골라
`joint_mapping.json`과 `gait_scene.xml`을 생성합니다. 축 방향은 기존 논리 부호와 맞추되
CAD의 미세한 기울기를 유지합니다. CAD +X→로봇 +Y, CAD -Y→로봇 +X입니다.
내보낸 거의 직선인 자세를 임시 0/0/0으로 사용하며, 그룹을 J1→J2→J3로 연결했습니다.
실물 영점과 모터 하우징의 소속은 추가 확인이 필요합니다. 관성값은 컴파일용 임시값입니다.
원래 조립체가 영점에서 유지되는지 검사했으며, 12초/600프레임 정책 재생과 4개 위상
렌더를 확인했습니다. 결과는 기본 `/private/tmp/spot-cad-gait`에 저장됩니다.

정적 형상만 보려면:

```bash
conda activate spot_omg
mjpython simulation/mujoco/preview_cad.py
python simulation/mujoco/preview_cad.py --check
```

`cad_300mm/scene.xml`은 제공된 STEP의 실제 조립 배치를 메시로 가져온 별도 장면입니다.
1,334개 부품 인스턴스를 몸체와 FL/FR/RL/RR 각각의 J1/J2/J3, 총 13개 그룹으로
분류했습니다. MuJoCo STL당 삼각형 제한 때문에 몸체를 나눠 총 16개 메시를 사용합니다.
STEP 좌표의 외곽 크기는 약 X 240.214 / Y 512.687 / Z 354.809mm이며,
이는 내보낸 자세에서의 외곽 상자입니다. 상세 부품 경로는 `cad_300mm/assembly.json`에 있습니다.

`scene.xml`은 **정적 형상 확인용**입니다. 관절축·영점·질량·충돌 형상을 아직 확정하지
않았으므로 보행이나 토크 실험에는 연결하지 않았습니다. 색은 다리를 구별하기 위한
표시입니다. 몸체 변환 시 OCCT가 삼각분할할 수 없는 면 2개를 생략했다고 보고했습니다.
MuJoCo 로딩 검사 및 렌더 이미지에서 조립 배치를 확인했습니다.

재생성에는 `cadquery-ocp`가 필요합니다. 설치된 Python 환경에서 다음을 실행합니다.

```bash
python simulation/mujoco/import_step.py "/Users/etnlwind/Downloads/Spot OMG 300mm Frame.step"
```

## 현재 조이스틱 drive 동역학 실험

`drive_lab.py`는 공용 C `drive_targets()`를 기존 자유 몸체·중력·접촉 장면에
연결합니다. 실제 로봇 연결 기능은 없습니다. macOS에서는 다음처럼 실행합니다.

```bash
conda activate spot_omg
mjpython simulation/mujoco/drive_lab.py --balance --duration 30
```

- W: 전진 입력, S: 후진 입력, A/D: 회전 입력, Space: 중립 입력.
- 입력은 키를 뗀 후에도 유지되며 Space로 중립화합니다. 기본 시작 입력은 전진 0.6입니다.
- 마우스로 카메라를 조절할 수 있습니다. 시간 종료/넘어짐 시 장면을 정지하고 창은 유지합니다.
- 창을 닫으면 종료합니다. `--linear 0`으로 정지 상태에서 시작할 수 있습니다.

화면 없이 결과를 비교하려면:

```bash
python simulation/mujoco/drive_lab.py --check --balance --linear 0.6 \
  --duration 12 --output /tmp/spot-drive-forward
python simulation/mujoco/drive_lab.py --check --linear 0 \
  --duration 12 --output /tmp/spot-drive-idle
```

각 출력 디렉터리에 `summary.json`과 `frames.csv`를 저장합니다. CSV에는 50Hz로
몸체 위치/기울기, 발 접촉, 관절별 목표·실제 각도·토크를 기록합니다.
`--torque-scale`은 기존 모델 토크 한계의 민감도 비교용이며 실제 모터 출력 설정이 아닙니다.
토크 한계 비율은 각 제어 프레임 끝에서 95% 이상 도달한 구동기 비율의 평균입니다.

**실물 재현 범위:** 공용 궤적과 입력 slew/주기(2.4~1.8초)를 사용하지만 STM32 전체
제어기를 실행하지는 않습니다. 펌웨어의 actuator limiter·통신 지연·전압 강하·watchdog은
포함하지 않았습니다. 기존 scene은 모든 모터를 STS3215로 가정하며 실제 J2의 STS3250과
다릅니다. 0.6kg 배터리의 무게중심도 미반영입니다. 실물 추진력을 단정하는 데 사용하지 않습니다.

2026-09-08 초기 기준(12초): 중립 X 이동 -0.000033m, 전진 0.6 + balance에서는
X=-0.511m, 최대 추종 오차 10.28°, 토크 한계 비율 1.11%, 자세 유지였습니다.
전진 입력과 모델 X 이동 부호가 반대이므로 좌표/기구 방향 검증이 우선입니다.
단순히 입력 부호를 뒤집어 실물 정책까지 수정하지 않습니다.

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

## 앱 / spotctl 가상 로봇 연결

실행 명령, 앱의 실제/가상 대상 선택, 지원 명령과 실물 동일성의 현재 한계는
[가상 로봇 연결 문서](../../docs/VIRTUAL-ROBOT-2026-09-09.md)를 참고하십시오.
`virtual_robot.py`는 STEP 기반 물리 모델을 실시간으로 구동하며 TCP 콘솔을 제공합니다.

### Bluetooth 가상 로봇

기본 실행은 이제 MuJoCo 창과 `SpotOMG-Sim` Mac BLE 수신 앱을 함께 실행합니다.
iPhone **V0.3.0 (6)**에서 **가상 로봇 · BLE**를 선택하면 IP 입력 없이 연결합니다.
자동 테스트는 `--headless --no-ble`로 실행합니다.
[BLE 실행 방법·UUID·검증](../../docs/VIRTUAL-BLE-2026-09-09.md).

### 다양한 보행 정책 (V0.3.1 앱)

가상 로봇 기본 조이스틱 정책은 크루즈입니다. 앱의 가상 로봇 보행 메뉴 또는 MuJoCo `1`~`5`로
기존 V16 / 크롤 / 크루즈 / 빠른 트롯(실험) / 높은 발 들기를 선택합니다.
`W`는 8초 데모(원격 연결 없을 때), `Space`는 정지입니다.
[측정 결과와 검증 범위](../../docs/GAIT-PROFILES-2026-09-09.md)를 참고하십시오.

### BNO055 능동 수평 보정 (V0.3.2 앱)

`virtual_robot.py`는 이제 지연된 BNO055 측정으로 수평을 보정하며 기본 ON입니다.
`imudiag`로 센서와 보정량을 확인하고, 정지 중 `simbalance on|off`로 비교합니다.
앱의 가상 로봇 보행 항목에도 토글이 있습니다. [설계·검증](../../docs/ACTIVE-BALANCE-2026-09-09.md)을 참고하십시오.
이 변경은 `virtual_robot.py` 경로에 적용되며 단독 `cad_physics.py --balance` 실험과 구분합니다.

### balance-v3-sim / iOS V0.3.3 (9)

고속 보행은 낮은 PI 보정으로 변경했고 정지 시 수평 보정은 유지합니다. 트롯·하이 스텝의 후진은 60%로 제한합니다. 안전 정지 시 직전 30초 기록을 `/private/tmp/spot-omg-sim`에 저장합니다. [수정 및 60초 보행 검증](../../docs/BALANCE-V3-2026-09-09.md) 참조.

### balance-v4-smooth-sim

크루즈는 빠르고 낮은 전진 궤적과 기존 회전/후진 궤적을 연속 혼합합니다. [동일 조건 비교 및 17개 장시간 검증](../../docs/SMOOTH-CRUISE-2026-09-09.md) 참조. 화면에는 실제 시간 대비 재생 배율이 표시됩니다. 기존 V0.3.3 앱에서 그대로 크루즈를 선택하면 됩니다.

### balance-v5-pivot-sim

좌우 입력 시 앞뒤 발의 횡방향 궤적과 J1을 사용해 제자리 회전을 만듭니다. 이동 입력이 커지면 기존 이동 궤적으로 연속 전환합니다. 앱 V0.3.4 (10)과 연동되며, [회전량 비교 및 44개 동역학 검증](../../docs/PIVOT-TURN-2026-09-10.md)을 참고하십시오.

### heading-v6-sim / 앱 V0.3.5 (11)

조이스틱 전진 시 지연된 IMU yaw로 시작 방향을 유지합니다. 앱의 ‘IMU 직진 방향 유지’ 토글이나 정지 중 `heading on/off`로 변경합니다. 기본 ON이며 수평 보정과 별개입니다. 회전 입력은 사용자 명령이 우선합니다. [설계와 A/B 검증](../../docs/HEADING-HOLD-2026-09-10.md).

### shared-locomotion-v17-sim

STM32 V17과 발 궤적·보행 위상·IMU 필터·방향 유지·수평 보정·서보 tick 변환을 공유합니다. 배포 설정은 `config/locomotion_profiles.json`이며 앱 V0.4.0 (12)에서 실제/가상 양쪽을 설정할 수 있습니다. [통합 범위와 검증](../../docs/SHARED-LOCOMOTION-V17-2026-09-10.md).

### IMU 보행 비교 정책

`--profile imu`로 위상별 J1/착지 보정을 사용하는 별도 모드를 실행합니다. MuJoCo 키 7=IMU, 6=lift입니다. 기존 기본 정책은 유지됩니다. 제어 범위와 20개 A/B 결과는 [IMU 정책 기록](../../docs/IMU-GAIT-POLICY-2026-09-10.md)을 참조하십시오.

### 수평 우선 Level 모드 (v19)

`--profile level` 또는 MuJoCo 키 8로 선택합니다. 7mm 발 높이와 주기 1초의 별도 궤적, J1 roll 보정을 사용합니다. [측정 및 한계](../../docs/LEVEL-GAIT-2026-09-10.md)를 참조하십시오.

## Level15 (2026-09-10)

V0.4.2 (14) / shared-locomotion-v20: `level15` (**수평 + 발 들기 · 15mm**) 추가. 기존 Level은 유지합니다. 15mm는 명령 높이이며 기본 모델의 실제 발높이 중앙값은 앞발 약 6mm, 뒷발 약 10mm입니다. 상세 측정과 영상은 `docs/RAISED-LEVEL-2026-09-10.md`를 참고하십시오.

## J2·J3 협응 보행 (v21)

`--profile joint` 또는 숫자 0. 기존 Level15를 보존한 J2/J3 속도 선행 보상 정책입니다. 자세한 측정·영상·한계는 `docs/J2-J3-COORDINATION-2026-09-10.md`를 참고하십시오.

## J2 협응 · 빠르게 (v22)

앱 V0.4.4 (16), `jointfast` 모드 추가. 기존 joint 유지. 최대 전진 보폭 85mm, 주기 1.5초로 기본 모델에서 약 25% 속도 향상. 측정값·제약·영상은 `docs/J2-FAST-GAIT-2026-09-10.md`. 실제 폰 UJIN17만 사용하며 iOS Simulator를 실행하지 않습니다.

## 스포츠 모드 / 전체 정책 속도 (v23)

앱 V0.4.5 (17): `jointsport` (**J2 협응 · 스포츠**) 추가. 전체 정책 이름에 동일 조건 측정 속도를 표시하며 실패 시 `(검증실패)`로 표시합니다. `docs/J2-SPORT-AND-SPEED-LABELS-2026-09-10.md` 참고. 실제 폰 UJIN17만 사용합니다.

## Upright 전진 실험 (2026-09-11)

높이240mm·목표 발 들림30mm·주기1.8초의 별도 실험 정책을 추가했다.
기존 정책은 변경하지 않으며 실제 펌웨어에 설치하지 않는다.

```bash
mjpython simulation/mujoco/preview_upright.py --viewer
```

자동 전진/정지 데모 및 검증 수치와 제한은 [Upright 기록](../../docs/UPRIGHT-GAIT-2026-09-11.md)을 참고한다.

1cm 탄성 의자발 쿠션 가정은 다음과 같이 재생한다. 재질은 미실측 접촉 근사다.

```bash
mjpython simulation/mujoco/preview_upright.py --viewer \
  --foot-cushion simulation/mujoco/foot_cushion_10mm.json
```

비교 결과와 모델 한계: [쿠션 실험 기록](../../docs/FOOT-CUSHION-10MM-2026-09-11.md).

원형37.3mm 쿠션의 어깨 앞 착지 실험은 [CUSHION-REACH-GAIT](../../docs/CUSHION-REACH-GAIT-2026-09-11.md)를 참고하십시오. 프로필 `cushion_reach`는 전진 검증용이며 후진/회전에는 발 끌림이 남습니다.
# 큰 스텝 · 지지 전환 보정 (2026-09-11)

`cushion_support_shift`는 140mm 보폭/4.8초 주기를 유지하는 **시뮬레이터 전용 실험 정책**이다.
7초 구간의 기울기는 개선됐지만 60초 전체 기울기·뒷발 접촉·접지 이동 기준에 실패하여
앱에 `(검증실패)`로 표시한다. 실물 펌웨어에는 적용하지 않았다.
구현, 동일 조건 비교, 영상과 실행 명령은
[지지 전환 검증 기록](../../docs/SUPPORT-SHIFT-GAIT-2026-09-11.md)을 참고한다.

- [V2 J1 고정 보행: 변경 분리·60초 결과·미달 항목](../../docs/SUPPORT-SHIFT-V2-2026-09-11.md)

### 80mm 동역학 지지력 예측 실험

`cushion_dynamics_wbc80`은 질량중심 MPC, 전신 역동역학 QP와 위치 서보용 변환을 사용하는 별도 실험 정책이다. **검증실패** 상태이며 기본 보행과 실물 펌웨어를 대체하지 않는다. 지지력 해의 성공과 실제 보행 합격을 구분한다.

추가 의존성은 `requirements-wbc.txt`에 있으며, 실행·검증 명령과 한계는 [동역학 제어 기록](../../docs/DYNAMICS-WBC-2026-09-11.md)에 정리했다.
