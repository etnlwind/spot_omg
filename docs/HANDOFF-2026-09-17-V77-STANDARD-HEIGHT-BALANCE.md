# V77 보행 / 기본 높이·J1 유지 — 집 컴퓨터 인수인계

2026-09-17. 저장소 `https://github.com/etnlwind/spot_omg.git`, 브랜치 `develop`.
오늘 작업의 시작 커밋은 `5fd48d4`다. 이 문서가 포함된 커밋에 아래 코드·설정·
실기 원본·시뮬레이션 결과를 함께 보존한다. 최신 진입점은 `readme.md`와
`docs/HANDOFF-LATEST.md`다.

## 1. 재개 전에 알아야 할 결론

**보행 개선은 아직 완성되지 않았다.** 현재 부족한 것은 발끝 궤적과 몸체의
동적 균형·실제 하중 교대를 함께 다루는 제어다. 모터 힘이 부족해서 불가능하다는
결론은 내리지 않았다. 실패한 각도 조합을 계속 조금씩 바꾸는 작업부터 반복하지 않는다.

- 기존 시뮬레이션의 무전도 결과에는 스윙 중 뒷발의 의도치 않은 접촉 지지가
  포함되어 있었다. 입력344, 30초 재현에서 정상 중간 스윙 표본의
  RL **87.14%**, RR **89.02%**에 0.5N 초과 접촉이 있었다.
- 큰 회수 후보는 FR/RL이 둘 다 0N으로 떠 있는 동안 roll이 먼저 증가했다.
  4.40→4.60초에 −2.87→−10.54°, RL 재접촉은 4.68초다. 이 사례를
  “발끝이 먼저 걸려 넘어짐”으로 설명할 수 없다.
- 고정 J1 경로는 기존 support Y 목표를 최종 명령에 반영하지 않는다.
  support Y를 10mm 바꿔도 명령 차이 0°인 것을 회귀 검사로 확인했다.
  **J1 고정 요구가 잘못됐다는 뜻이 아니라, 그 조건에 맞는 균형 제어가 필요하다.**
- 추가 J2=85°/J3=104° 목표는 발을 뗀 뒤의 회수 동작이다. 접지 중 추진력이
  더 커졌음을 구현·측정한 결과가 아니다. 접지 중 추진과 공중 회수를 구분해야 한다.
- 실제 로그에서 J2가 조금 움직인 구간은 목표도 작았다. 첫 RL J2 목표53.00°에
  실측52.91°였고, 긴 보행의 정상 구간 목표도 대략61°였다. 이 구간에서
  “수평 가까운 명령을 보냈는데 서보가 못 올라감”으로 판단하면 안 된다.

상세 근거: [기본 높이 유지 분석](STANDARD-HEIGHT-BALANCE-2026-09-17.md),
[기존 성공 판정 감사](V77-SIMULATOR-QUALIFICATION-AUDIT-2026-09-17.md),
[앞뒤 발 높이 차이 재구성](V77-REAR-CLEARANCE-2026-09-17.md).

## 2. 반드시 유지할 사용자 조건

1. **기본 몸체 높이를 낮추지 않는다.** 이전 45mm 낮춤 후보는 사용자에게
   거절된 비교 실험이다. `large_recovery_balanced_candidate.json`은 현재 해결안이 아니다.
2. 첫 FR/RL 동시 진입에서 FR만 정상 J1 오므림의 두 배, 다른 세 J1은 S.
   두 번째 FL/RR 걸음으로 네 다리의 정상 J1 각도를 맞춘 뒤 유지한다.
   준비 두 걸음은 균형에 집중하고, 큰 회수는 그 이후 지지/추진을 마친 발부터다.
3. 정지 신호 뒤 대각선 두 쌍을 “하나, 둘” 제자리 배치하며 각각 한 번에 S로
   맞춘다. 멈춘 뒤 바닥에서 J1을 천천히 벌리는 동작을 추가하지 않는다.
4. 목표는 J2를 뒤로 높게 들면서 J3를 빠르게 접고, 전진 스윙 중 자연스럽게
   펴서 착지하는 것이다. 목표에 도착한 뒤 별도로 펴는 방식이나 교차 대기는 금지.
5. J2 회수 85°는 CAD 상 윗링크가 수평 아래 약9.45°인 제어각이다.
   더 작은 각도로 통과하면 그 차이를 명시한다. 사진 투영각·링크 내각·모터
   원시값·제어각을 같은 것으로 취급하지 않는다.
6. 실제 시험은 바닥·실제 속도로 한다. 사용자는 공중/감속 시험을 원하지 않는다.
   시뮬레이터의 충돌·보호·추적 감속을 꺼서 성공시키거나 미측정 모터 성능을
   임의로 낮춰 실기 실패를 설명하지 않는다.
7. 서보/좌표 변경 전 `AGENTS.md`와 `docs/STS3250-POSITION-CONTROL.md`를 읽는다.
   앞 J1의 실기↔CAD 부호 변환과 J2 누적 위치 원점을 보존한다.

## 3. 실제 설치 상태와 오늘 확보한 실기 자료

- 마지막 확인 펌웨어: **`s-native-v6-2-7-v77`**, 프로필 `s_native_v6_2_7`.
- 마지막 `floor-six-step02` 종료 응답: 정상 Stop, Stand, torque ON, safety OK,
  error19ticks, fault_code0. 이는 **당시 마지막 기록**이며 다음 컴퓨터의 현재
  자세/전원/준비 상태를 보장하지 않는다. 사용자는 보행 중 뒷발 끌림을 관찰했다.
- 짧은 바닥 비교 `floor01`, `floor02`와 6걸음 요청에 따른 긴 바닥 시험
  `floor-six-step01`, `floor-six-step02`의 목표·순차 관절 피드백·IMU를 보존했다.
  긴 시험의 호스트 구동 창은8.6초다. 정확한 접지6회를 센 폐루프 종료가 아니다.
- 관절 trace 용량은 약5.1초, IMU는 약10초여서 긴 시험 전체를 같은 밀도로
  관측하지 못한다. 순차 관절 측정도 동시 자세 측정과 구분한다.
- `artifacts/imu-trace-v78/`는 **미설치 폐기 진단 후보**다. 사용자가 감속 시험을
  취소했으며 펌웨어 소스 변경도 제거했다. V78 바이너리를 설치하지 않는다.
- 오늘의 균형 후보는 모두 오프라인이다. 앱/공식 프로필/STM32 보행 코어를
  새 후보로 교체하지 않았다. 앱에서 V6.2.7을 실행하면 기존 정책이 실행된다.

실기 업데이트를 다시 하게 되면 **Landing 명령 → 실제 도착 확인 → 토크 OFF
확인 → 설치 → Landing 확인** 순서다. Stand에서 몸체 지지만 확인하고 바로
토크를 끄거나 Landing 실패를 생략하면 안 된다. 이번 인수인계 후 자동 재구동하지 않는다.

## 4. 오늘 바뀐 코드

| 파일 | 변경 목적 |
|---|---|
| `scripts/hardware/capture_v627_first_step.py` | 선택적 Landing 선행, 제한된 구동 창, IMU/관절 trace를 독립 저장해 한쪽 다운로드 실패에도 다른 증거 보존 |
| `tools/servo_tool/servo/gait_alignment.py` | 같은 시험의 MCU joint/IMU 시간축 정렬, wrap 처리, invalid/누락 배제, 앞 J1 좌표 변환 |
| `simulation/mujoco/scripts/visualization/replay_joint_trace.py` | 실기 원시 목표의 앞 J1 부호 누락 수정, 앱 기본 물성 적용, 실측 초기 IMU/초기1초 오차와 접촉 비교 |
| `simulation/mujoco/runtime/gait_evidence.py` | 발별 접촉력·스윙 중간 여유 검사, 비어 있는 근거를 성공으로 보지 않음, 다리별 위상 지원 |
| `simulation/mujoco/scripts/validation/validate_s_native_firmware.py` | 기존 C 커널 실제 명령 경로에 접촉/여유/보호/정지 도착 평가를 분리해 기록 |
| `simulation/mujoco/scripts/validation/validate_s_native_v627.py` | 출력 경로 옵션, 보행 품질 미달 시 명확한 실패 종료 |
| `simulation/mujoco/scripts/analysis/reconstruct_v77_feet.py` | 실제 관절각과 IMU로 CAD 발 높이 차이 재구성, 축/초기 높이 불확실성 표시 |
| `simulation/mujoco/scripts/analysis/test_ground_clearance_correction.py` | 실측 경로와 같은 속도에서 단순 지면 여유 보정 가설을 비교한 오프라인 실험 |
| `simulation/mujoco/scripts/analysis/capture_j2_85_candidate.py` | 첫 지지 이후/두 진입 걸음 이후의 큰 회수를 분리해 기존 정책 보존 |
| `simulation/mujoco/scripts/analysis/analyze_large_recovery_balance.py` | 높이·J1·지지 시간·회수·피드백 원인 분리, 원본20ms 데이터와 요약 저장, fixed_frame 제약 |
| `simulation/mujoco/scripts/analysis/capture_s_native_balance.py` | 후보 설치 hook, 다리별 실제 계획 위상 표시, 몸체 높이 기록 |
| `simulation/mujoco/scripts/analysis/capture_large_recovery_candidate.py` | 명시한 설정/후보를 4방향 녹화; 과거45mm 낮춤 제목을 새 후보에 잘못 붙이지 않도록 수정 |
| `simulation/mujoco/scripts/analysis/plot_large_recovery_cause.py` | 큰 회수 실패와 과거 낮춤 비교 그림 재생성 |

새 설정은 `config/large_recovery_*.json`, `config/standard_height_*.json`이다.
실험별 개별 결과 JSON에 실제 적용한 설정·입력·전압·시험 길이가 함께 저장되어
있다. 이름에 candidate/balanced가 있어도 검증 완료를 뜻하지 않는다.

## 5. 비교 결과와 배제한 해석

| 조건 | 결과 |
|---|---|
| 몸체45mm 낮춤·지지70%·큰 J2 회수 | 30초 무보호, 최대기울기5.17°. 발 여유 미달. **사용자 조건에 어긋나 채택하지 않음** |
| 기본 높이·고정J1·지지70%·J2 회수70° | 8초 무보호지만30초 시험13.66초에 보호 |
| 같은 조건·J2 회수75° / 80° | 각각11.22초 / 9.02초에 보호 |
| 기본 높이·고정J1·J2 회수70°·지지70%·공통 X 보정 | 30초 무보호와 S 복귀. 실제J2 최대약68.2°, 최대기울기9.73°, 발 접촉 미달 |
| 기본 높이·고정J1·J2 회수85°·지지85%·X 보정 | 16.58초에 보호, 이후 전도 |

마지막 무보호 X 후보는 `standard_height_final_comparison.json`의
`x20-short-lead-j270`이다. **추천 배포 모델이 아니라 제약과 한계를 확인한 비교점**이다.
정상 J1 명령 변화0°지만 실제 J1 변화폭 약1.6~2.0°. 몸체 높이 평균변화−2.77mm,
순간−14.81mm까지 변하므로 실제 높이 유지도 미완성이다. 평균 위상 속도 계수0.789로
자동 추적 감속이 있다. 전진0.573m, 측면0.055m, 최종 S 오차0.99°다.

다른 지지 비율, 빠른 주기, 일정 지지발 속도, 앞뒤 위치 고정 이동, 초기 진입
시간 변경, 순간 Z 보정, 지지 하중 선행 배치, 자연 낙하형 X 곡선 등을 비교했다.
실패 데이터도 저장했다. 작은 변경을 반복하면 된다는 결론도, J1 고정은
물리적으로 불가능하다는 결론도 아니다. 전체 동적 균형을 따로 설계해야 한다.

**기존 평가를 수정한 이유:** 무전도와 S 복귀만 보던 검사는 뒷발 접촉 문제를
놓쳤다. 현 자격평가는 중간 스윙의 네 발 하중≤0.5N, 여유>2mm 및 전 구간
보호 없음/S 복귀를 요구한다. 이 기준에서도 접촉 유무는 실제 미끄럼 측정과 다르다.

## 6. 자료 위치와 영상

| 위치 | 내용 |
|---|---|
| `artifacts/imu-trace-v77/alignment-2026-09-17/` | 실제 floor01/02, 긴 보행 두 회, 정렬 데이터, 실제 명령 재생, CAD 재구성 |
| `artifacts/imu-trace-v77/simulator-audit/` | 기존 C 커널 재평가, 실제 입력344 비교, 초기 여유 보정 실패 |
| `artifacts/imu-trace-v77/j2-horizontal/` | 큰 회수의 전도 원인 분리, 과거 낮춤 후보, 4방향 영상과20ms 원본 |
| `artifacts/imu-trace-v77/standard-height-balance/` | 기본 높이·고정J1 조건의 모든 비교와30초 검증 |

사용자 iPhone 원본 영상도 저장소로 복사했다:
`artifacts/imu-trace-v77/alignment-2026-09-17/floor02/video/IMG_3591.MOV`.
28,984,788bytes, SHA256
`20c64db49b5642545ad2a6f8c547f5767c50ba24c395c03b43c421d92bf246f4`.
원래 iCloud 파일과 해시가 일치한다. `config/v77_floor_validation.json`의 영상
경로도 저장소 상대 경로로 갱신했다. 같은 폴더의 quarter-speed 영상은 분석용
재생 속도 변경이며 실제 구동을 감속한 실험이 아니다. 영상과 MCU의 정밀 동기는 미확보다.

큰 회수 실패 영상:
`artifacts/imu-trace-v77/j2-horizontal/j2-85-after-symmetric-entry/four-views.mp4`.
과거45mm 낮춤 영상:
`artifacts/imu-trace-v77/j2-horizontal/balanced-candidate-video/four-views.mp4`.
후자를 기본 높이 유지 결과로 제시하지 않는다.

## 7. 집에서 바로 재현하기

작업 트리 변경을 먼저 확인하고, 덮어쓰기 없이 최신 develop을 가져온다.
충돌/분기 시 `reset --hard`로 집 작업을 지우지 않는다.

```powershell
git status --short
git switch develop
git pull --ff-only origin develop
conda activate spot_omg
$env:PYTHONUTF8='1'
$env:PYTHONPATH='.;tools/servo_tool'
```

환경이 없다면 저장소 루트에서 `conda env create -f config/environment.yml`로
만든다. 영상/그림 도구에는 `python -m pip install imageio-ffmpeg matplotlib`가
추가로 필요하다. 확인 환경: Python3.10.20, MuJoCo3.11.0, NumPy2.2.6,
pytest9.0.3, Pillow12.3.0, imageio-ffmpeg0.6.0, matplotlib3.10.9.
Windows C 커널 검사에서 컴파일러가 없으면 `apps/windows/setup.ps1`의 환경
설치를 참고한다(`.toolchain/zig/zig.exe`는 Git에 넣지 않는다).

관련 호스트 검사 **53개 통과**:

```powershell
python -m pytest tools/servo_tool/tests/test_joint_trace.py tools/servo_tool/tests/test_imu_trace.py tools/servo_tool/tests/test_gait_alignment.py simulation/mujoco/tests/test_hardware_gait_evidence.py simulation/mujoco/tests/test_j2_after_push.py simulation/mujoco/tests/test_large_recovery_balance.py simulation/mujoco/tests/test_s_native_v627.py -q
```

실제 바닥 명령을 원래 시간대로 재생하고, 원본 결과와 별도 폴더에 저장:

```powershell
python simulation/mujoco/scripts/visualization/replay_joint_trace.py artifacts/imu-trace-v77/alignment-2026-09-17/floor02/jointtrace.txt --imu artifacts/imu-trace-v77/alignment-2026-09-17/floor02/imutrace.txt --initial-attitude --output artifacts/home-resume/floor02-replay
```

기본 높이 후보30초 비교 및 특정 후보4방향14초 영상(2초 대기+8초 보행+4초 정지):

```powershell
python simulation/mujoco/scripts/analysis/analyze_large_recovery_balance.py --configs config/standard_height_final_comparison.json --output artifacts/home-resume/standard-height-comparison --seconds 30
python simulation/mujoco/scripts/analysis/capture_large_recovery_candidate.py --config config/standard_height_final_comparison.json --case x20-short-lead-j270 --output artifacts/home-resume/standard-height-video
```

기존 공식 C 커널의 강화된 자격평가(현재 실패가 예상되며 근거 파일을 저장함):

```powershell
python simulation/mujoco/scripts/validation/validate_s_native_v627.py --output artifacts/home-resume/v627-qualification
```

`analyze_large_recovery_balance.py`는 실패 후보도 자료를 저장하고 계속 비교한다.
프로세스 종료코드0을 보행 성공으로 해석하지 말고 `summary.json`의
`safety_ok`, `stop_ok`, `gait_quality`, `steady_quality`를 읽는다.
`validate_s_native_v627.py`의 자격평가 실패 종료코드는 의도한 동작이다.

## 8. 다음 구현 순서

1. 기존 궤적의 의도한 지지 쌍과 실제 접촉을 겹쳐 보고, 스윙 발이 하중을
   받는 구간을 성공 사례에서 제외한다. 동일 입력344와1000을 섞지 않는다.
2. 기본 높이·정상 J1을 제약으로 둔 몸체 운동/접촉 계획을 만든다. 자세뿐 아니라
   각속도와 다음 발이 받칠 수 있는 시각을 포함한다. 접지 중 J2/J3 추진과
   공중 회수 목표를 한 스케줄 안에서 함께 계산한다.
3. 실기에서도 얻을 수 있는 IMU·관절 피드백으로 사용할 제어/추정 경로를
   만든다. 시뮬레이터의 실제 접촉력을 제어에 몰래 사용하지 않는다. 모델의
   접촉력은 추정기와 결과를 독립 평가하는 기준으로 사용할 수 있다.
4. 동적 몸체 높이, 실제 J1 추종, 목표/실측 발 X/Z, 접지력, 접촉 중 발의
   미끄럼 속도, 몸체 자세/각속도, 실제 위상 속도를 함께 검증한다. 8초로
   끝내지 말고30초와 정지까지 통과시킨다. 필요한 큰 J2 각도를 줄였다면
   원래 요청 달성으로 표시하지 않는다.
5. 시뮬레이터에서 조건을 충족한 뒤 C 커널·앱에 새 버전으로 반영하고
   명령 일치 검사를 거친다. 기존 모델 이름·프로필 인덱스를 덮어쓰지 않는다.
6. 실기 재개 시 새 준비 상태를 확인하고 실제 바닥·속도에서 검증한다.
   설정 readback, 위치 유지, 전체 보행을 구분한다. 설치만으로 완료 처리하지 않는다.

지금의 올바른 출발점은 **“더 강하게 접으면 해결된다”가 아니라, 발이 확실히
떨어진 상태에서도 몸체를 지탱할 수 있도록 지지 교대를 설계하는 것**이다.

## 9. 마지막 추가 질문: Spot의 둥근 발끝 형상

사용자는 Spot 사진을 보고 발끝 형상도 장점인지 물었다. 사진은
`artifacts/imu-trace-v77/foot-shape-reference/spot-foot-reference.png`에 보존했다.
이번에 발/쿠션 형상을 바꾸지는 않았다.

- Boston Dynamics는 Spot을 둥근 발과 고무 패드가 있는 구조로 설명한다.
  [공식 Spot Anatomy](https://support.bostondynamics.com/articles/Knowledge/Spot-Anatomy-49915).
- 기하 관점에서 둥근 접지부는 다리 자세가 바뀔 때 접촉점이 모서리에서 급히
  바뀌는 것을 줄이고, 기울어진 착지에 대응하는 데 유리할 수 있다. 고무 패드는
  적절한 재질/바닥 조건에서 마찰과 충격 완화에 도움이 된다. 사진만으로 실제
  강성·감쇠·마찰계수나 휘어진 하부 링크가 스프링인지까지 확정하지 않는다.
- 곡선 하부 링크는 같은 관절 축/발끝 위치에서도 링크 중간부의 바닥·장애물
  간격을 달리할 수 있다. 곡선이라는 이유만으로 발끝 높이가 자동으로 올라가거나
  추가 자유도가 생기는 것은 아니다. 얇다는 이유만으로 질량도 단정하지 않는다.
- 둥근 발은 구르면서 실제 접촉점이 이동한다. 넓은 발바닥보다 정적 지지
  면적이 작을 수도 있어 무조건 균형에 유리하다고 할 수 없다. 반지름/접촉점
  이동을 운동학에 포함하는 문제는 [구면 발 운동학 연구](https://arxiv.org/abs/2107.12479)에서도 다룬다.
- 우리 쿠션도 `foot_cushion_d37p3_l27mm.json`의 원형 볼록 캡(D37.3×27mm)이다.
  `elastic_cap_vertices()`는 모양을 근사한 강체 메시이며 재료 탄성체의 실측
  재현이 아니다. 접촉 solver의 유연성과 고무의 실제 변형을 동일시하지 않는다.
- 현재 `SoleKinematics`는 X/Y에 S의 고정 재료점을, Z에 회전한 표면 최저점을
  사용한다. 실제 압력 중심/COP와 동일한 점이라고 보면 안 된다. 쿠션 변경을
  검토한다면 원래 높이·질량·마찰 등 비교 조건을 통제하고, 실제 접촉점 이동과
  압축량을 함께 검증해야 한다. 이것을 이번 전도의 확정 원인으로 주장하지 않는다.

형상 개선은 추가 검토 대상이지만, 이미 발이 떠 있는 동안 발생한 동적 균형
실패까지 발 모양 하나로 해결됐다고 간주하지 않는다.
