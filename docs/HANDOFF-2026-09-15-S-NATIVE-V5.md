# S 기준 보행 V5 작업 인수인계 — 2026-09-15

**최신 인계: [V6.2.1 실기 v63 설치 후 발 끌림·넘어짐](HANDOFF-2026-09-15-V621-REAL-ROBOT.md). 아래는 V5 당시 기록이다.**

최신 실기 오류 수정: [V6.1 앞 J1 방향 변환 v60](S-NATIVE-V61-FRONT-J1.md).

실기 후속: [V6.1 STM32 v59 이식·검증·실물 설치 기록](S-NATIVE-V61-FIRMWARE.md).

후속 Windows 재현 및 원인 분리 결과는 [V5 출발 분석](S-NATIVE-ENTRY-REVIEW-2026-09-15.md)을 참고한다.
사용자의 지지 교대 완화 요청을 반영한 후속 모델은 [V6 검증 기록](../artifacts/s-native-v6/README.md)에 정리했다. 아래 V5 수치는 기존 결과로 보존한다.
V6를 보존하고 뒤쪽 도달 거리를 5mm 늘린 [V6.1 검증 기록](../artifacts/s-native-v6-1/README.md)도 추가했다.

## 현재 상태부터 확인

저장소는 `https://github.com/etnlwind/spot_omg.git`, 작업 브랜치는 `develop`이다. 이번 기록은 S 기준 보행 V1~V5, 폴더 정리, 맥 제어 앱의 최신 모델 기본 선택, 실제 물리 재생 결과를 함께 보관하는 중간 체크포인트다. **V5는 실험 구현이며 안정 보행이나 실제 대각선 접지 동기를 달성한 모델이 아니다.** 신경망 학습 모델이 아니라 CAD 역기구학과 주기식 궤적을 사용하는 절차식 정책이다.

최신 모델 ID는 **`s_native_v5`**, 앱 표시명은 **`S 출발 · FR 첫걸음 V5 · 실험`**이다. 새 모델은 목록 맨 위와 지원되는 시뮬레이터 연결의 기본으로 선택한다. V1~V4를 남겼고 실제 로봇의 기본 프로필/펌웨어 프로필 인덱스는 바꾸지 않았다.

최신 결과는 [V5 상세 기록](../artifacts/s-native-v5/README.md), 최신 영상은 [앞·뒤·위·옆 4방향 영상](../artifacts/s-native-v5/final-video/four-views.mp4)이다. `first-pass/`, `balance-video/`는 V5 각도 축소 이전 시행이므로 최신 결과와 혼동하지 않는다.

## 사용자가 정한 동작 조건

1. S는 발끝 쿠션의 접지점 X가 R2/J2 축 중심 X와 일치하는 정지 자세다. S에서 바로 보행을 시작하며 별도의 R 준비 자세를 거치지 않는다.
2. FR·RL / FL·RR이 대각선 쌍이다. 같은 쌍의 기준 앞뒤 이동과 발 높이/위상을 공유하고, 반대 쌍은 반 주기 차이로 교대한다. 목표 위상 일치와 실제 바닥 접촉 일치는 별도 검증한다.
3. 첫걸음에서는 **FR·RL이 동시에 내딛고 착지**해야 한다. FR만 정상 J1 오므림의 **2배**로 안쪽에 내딛으며, RL·FL·RR의 J1 명령은 S에 둔다. 2배는 S에서의 기구 관절각 변화량이며 발끝 Y 거리의 2배가 아니다.
4. 두 번째 걸음에서 FR의 추가 오므림을 풀고 다른 세 다리가 정상 오므림에 합류한다. 발 교대나 전진을 중간에 멈추지 않는다. 현재 구현에서는 두 번째 걸음의 지지 다리 FR/RL도 J1이 전환된다.
5. 정지할 때는 현재 오므림을 반대로 풀어 S로 돌아간다. 출발 중 정지도 현재 각도에서 이어져야 한다.
6. 보폭 확대는 S 뒤로 미는 구간에 적용한다. 최대 전진 입력에서 앞쪽 +20mm는 유지하고 뒤쪽은 -60mm다. 교대 대기는 0초, 정상 주기는 0.8초, 명목 발 높이는 25mm다.
7. 발끝 쿠션 D37.3×27mm와 배터리/쿠션 포함 총질량 2.754kg을 유지한다. 질량 분포, 마찰/접촉, 서보 응답은 추정 물리 모델이다.
8. 시뮬레이터 창은 LG FULL HD 모니터, 기본은 측면 시점이다. 로봇 앞쪽이 화면 오른쪽: azimuth=90°, elevation=0°, distance=1.05m, 추적 중심은 COM보다 0.1m 아래다. 새 컴퓨터의 화면 배치는 직접 확인한다.

사진은 동작 순서를 이해하기 위한 참고다. Spot의 실제 내부 정책이나 정확한 3D 관절각을 사진에서 복원한 것으로 취급하지 않는다. 서보/관절 수정 전 [AGENTS.md](../AGENTS.md)와 [STS3250 실기 기준](STS3250-POSITION-CONTROL.md)을 읽는다.

## 모델별 변화와 측정 결과

| 모델 | 주요 변경 | 확인된 결과 / 한계 |
|---|---|---|
| V1 | S 기준 대각선 보행, 교대 대기 0.6→0.3초 | 입력 60% 실제 네 발 동시 접지 중앙값 680→360ms. 같은 조건 최대 대각선 착지 차이 60ms. 완전 동기는 아님. |
| V2 | 교대 대기 제거, 주기 0.8초 | 입력 60% 넘어짐 허용 32초 재생에서 동시 접지 중앙값 40ms, 최대 기울기 약 23.3°. 안정성 통과 아님. |
| V3 | 뒤쪽 밀기만 -60mm로 확대 | 2초 대기 + 입력 100% 10초 재생에서 영상 5.62초 전도. |
| V4 | 스윙하는 쌍을 차례로 J1 아래로 모음 | 첫 쌍이 FL/RR이었음. 같은 영상 조건에서 4.12초 전도. 사용자가 제시한 FR만 먼저 오므리는 순서와 달랐음. |
| V5 | 첫 FR/RL 쌍, FR만 2배 오므림, 다음 걸음 합류, Stop 시 풀기 | 최종 첫 접지는 RL 2.46초 / FR 2.54초로 **80ms 차이**. 영상 **3.20초(보행 +1.20초) 전도**. |

V1/V2와 V3~V5는 입력/길이가 다르므로 속도나 안정성의 같은 조건 순위로 비교하지 않는다. 각 폴더의 README와 JSON에 조건이 있다. 전도 뒤의 발 위치 최대/최소는 정상 보폭으로 해석하지 않는다.

## V5 각도 축소와 정지 결과

기존 정상 오므림은 S에서 약 12.9°였고 첫 FR의 2배는 약 25.8°였다. 이 각도는 FR J2와 몸체 `body_3`의 실제 CAD 삼각형이 교차했다. 단순 충돌 상자만의 문제가 아니었다. 사용자에게 확인한 뒤 **정상 기준 9° / 첫 FR 18°**로 줄여 비율을 유지했다. 10°/20° 후보의 최소 간격은 약 0.37mm, 선택한 9°/18°는 약 2.09mm였다.

이 간격 검사는 첫 스윙 51개 목표 자세에서 FR J2/J3와 몸체 네 부품을 검사한 결과다. 모든 동작의 기구 간섭/공차를 검증한 것은 아니다. 충돌을 끄거나 관절/서보 한계를 완화하지 않았다. 정상 기준각 외에 발 높이, 회전, 공통 지지 이동에 필요한 보정이 더해지므로 네 J1의 순간 명령이 항상 정확히 -9°인 것은 아니다.

첫 스윙은 발 높이를 1초 전후 보폭 진폭 증가와 분리했다. 기존에는 그 진폭 증가가 첫 발 높이도 거의 0으로 줄여 발을 충분히 들지 못했다.

Stop은 현재 J1에서 0.8초 동안 부드럽게 오므림을 풀며 전진 감속/위상 진행을 유지한 다음 S 복귀 처리를 마친다. 출발 0.3초 뒤 Stop의 실제 최종 S 관절 오차는 약 0.99°, 몸체 기울기는 약 0.04°였다. 보행 1초 뒤 Stop 시험에서는 관절 명령이 S로 복귀했지만 진행 중인 넘어짐을 막지 못했다. **S 관절각 복귀와 서 있는 상태로 정지는 같은 판정이 아니다.**

## 다른 Mac에서 준비하고 실행

아래 명령은 저장소 루트 기준이다. 기존 체크아웃에서는 현재 변경사항을 보존한 상태에서 `git pull --ff-only origin develop`로 갱신한다. 새 체크아웃은 다음과 같다.

```bash
git clone --branch develop https://github.com/etnlwind/spot_omg.git
cd spot_omg
conda env create -f config/environment.yml
conda activate spot_omg
```

이 환경 파일의 editable 패키지 경로 `../tools/servo_tool`은 환경 파일이 있는 `config/` 기준이다. 공용 C 바인딩을 만들 C 컴파일러와 맥 앱을 만들 Xcode가 필요하다. 영상 인코딩에는 PATH에서 실행 가능한 `ffmpeg`/`ffprobe`가 필요하다. 선택적인 상세 삼각형 간섭 검사는 다음 의존성을 사용한다.

```bash
python -m pip install -r simulation/mujoco/config/requirements-stow.txt
```

이번 실행 환경은 Python 3.10.20, MuJoCo 3.11.0, NumPy 2.2.6, Pillow 12.3.0, pytest 9.0.3, Xcode 26.6(17F113), ffmpeg 8.0.1이었다. 플랫폼/라이브러리 차이로 재현 수치가 달라질 수 있으므로 버전과 원시 기록을 함께 남긴다. 상세 충돌 검사의 python-fcl은 현재 컴퓨터에서는 임시 의존성 폴더에 설치했으며 그 폴더는 Git에 포함하지 않는다.

```bash
# 이번 영상처럼 넘어짐까지 관찰: 실제 로봇을 구동하는 명령이 아님
mjpython simulation/mujoco/virtual_robot.py --viewer --allow-fall

# 일반 기울기 보호를 유지하려면 --allow-fall을 생략
mjpython simulation/mujoco/virtual_robot.py --viewer
```

macOS GUI는 `python` 대신 `mjpython`으로 실행한다. 기본 프로필은 V5다. 앱에서 같은 Mac의 **가상 로봇 · TCP / 127.0.0.1 / 8765**를 선택하고 연결한다. 다른 기기에서 연결할 때 127.0.0.1은 그 기기 자신이므로 시뮬레이터 Mac의 LAN 주소를 사용한다. 제어 연결은 단일 소유자다. 앱이 연결된 상태에서 별도 테스트 TCP 클라이언트를 붙이지 않는다. 영상 포트는 8766, 프레임 경로는 `/frame.jpg`다. 최초 프레임 준비 중에는 일시적으로 응답이 없을 수 있다.

연결 뒤 상태 응답의 `profile=s_native_v5`, `caps`의 `s_native_v5`, 실험 실행이면 `fall_test=on`을 확인한다. 현재 앱의 버전 표시는 0.5.0(45)이며 보행 버전은 정책 이름으로 구분한다. GUI 기본 프로필은 `--profile s_native_v4` 등으로 비교 실행할 수도 있지만 앱이 연결 시 최신 기본을 선택하므로 비교할 때는 앱에서도 해당 모델을 선택한다.

## 맥 제어 앱 빌드

이번 실행은 iOS Simulator가 아니라 **Mac Catalyst**였다. 저장소의 iOS 프로젝트 자체는 Catalyst 기본 설정을 바꾸지 않았으며, 별도 소스 복사본의 Catalyst 지원과 기기 계열만 바꿔 빌드했다. 새 Mac에서는 다음 명령으로 같은 복사본을 만든다. 이미 사용 중인 복사본과 섞이지 않도록 새 임시 디렉터리를 사용한다.

```bash
spot_mac_work="$(mktemp -d /tmp/spot-omg-mac.XXXXXX)"
ditto apps/ios/SpotOMGController "$spot_mac_work/source"
python - "$spot_mac_work/source/SpotOMGController.xcodeproj/project.pbxproj" <<'PY'
from pathlib import Path
import sys
p = Path(sys.argv[1])
s = p.read_text()
s = s.replace('SUPPORTS_MACCATALYST = NO;', 'SUPPORTS_MACCATALYST = YES;')
s = s.replace('TARGETED_DEVICE_FAMILY = 1;', 'TARGETED_DEVICE_FAMILY = "1,2";')
p.write_text(s)
PY
xcodebuild \
  -project "$spot_mac_work/source/SpotOMGController.xcodeproj" \
  -scheme SpotOMGController \
  -destination 'platform=macOS,variant=Mac Catalyst' \
  -derivedDataPath "$spot_mac_work/build" \
  CODE_SIGNING_ALLOWED=NO build
open "$spot_mac_work/build/Build/Products/Debug-maccatalyst/SpotOMGController.app"
```

현재 컴퓨터에서 실행한 기존 앱은 `/tmp/spot-omg-catalyst/Build/Products/Debug-maccatalyst/SpotOMGController.app`였지만 다른 Mac에 이 경로/빌드가 있다고 가정하지 않는다. 실행 중 프로세스, 연결 상태, 임시 파일, 가상 환경은 Git으로 전달되지 않는다. Windows에서 재개한다면 [Windows 앱 안내](../apps/windows/README.md)를 사용한다. 이번 최종 연결 확인은 macOS에서 했다.

## 검증 재현과 기록 위치

```bash
python -m pytest simulation/mujoco/tests/test_s_native_gait.py \
  simulation/mujoco/tests/test_drive_transition.py \
  simulation/mujoco/tests/test_virtual_robot.py -q --disable-warnings
python tools/generate_gait_speed_labels.py --check

# 결과 경로를 새 이름으로 지정해 기존 증거를 보존
python simulation/mujoco/scripts/analysis/capture_s_native_balance.py \
  --profile s_native_v5 --command 1000 --allow-fall \
  --fourth-view side --output artifacts/s-native-v5/recheck-video

# 출발 중/일반 Stop 재생과 선택적 FCL 간섭 비교
python simulation/mujoco/scripts/validation/validate_s_native_entry.py \
  --clearance --output artifacts/s-native-v5/recheck-entry
```

최종 관련 호스트 검사는 **54 passed, 90 warnings**다. 맥 앱 빌드와 실제 TCP 연결/V5 기본 선택도 확인했다. 12초 영상은 25fps, 300프레임, 1280×840이며 디코딩 검사를 통과했다. 명령/IK 검사 통과를 물리 안정성 통과로 해석하지 않는다. 실제 로봇은 구동하지 않았다.

폴더 이동 전후 전체 시뮬레이터 검사는 모두 **568 passed, 1 failed**였다. 기존 `test_optimized_gait`에서 yaw=12.4856586635°가 기준 10°를 초과하는 실패였고 이동 전후 수치가 같았다. 당시 선택 의존성 FCL 검사는 제외했다. 이것은 이동 당시의 비교이며 V5 추가 후 전체 묶음의 새 실행 결과라고 표현하면 안 된다. 당시 로그와 최종 54개 검사 로그는 [검증 기록 폴더](../artifacts/s-native-v5/checkpoint-validation/)에 보관한다.

| 자료 | 위치 |
|---|---|
| 최신 4방향/개별 영상, 프레임, 원시 궤적 | `artifacts/s-native-v5/final-video/` |
| 접촉 하중, 실제/목표 J1, 오므림 계수 | `final-video/trajectory.json` |
| 전도/기울기 관측 시점 | `final-video/summary.json` |
| 정지 4조건의 명령/실측 기록 | `artifacts/s-native-v5/stop-validation.json` |
| 9°/18°와 10°/20°의 FCL 삼각형 검사 | `triangle-clearance.json`, `triangle-clearance-10deg.json` |
| 앞/뒷다리 제약 및 S 기준 감사 | `artifacts/s-native-v1/constraints-audit.md`, `artifacts/stance-comparison/` |
| 변경 폴더/경로 목록 | `docs/PROJECT-LAYOUT.md`, `docs/file-moves-2026-09-15.json` |

## 다음 작업에서 볼 코드와 미해결 문제

- `simulation/mujoco/runtime/s_native_gait.py`: V1~V5 등록, 첫걸음 계수, 정상각 제한, 목표 발 궤적, `fr_entry_targets`, `fr_stop_targets`, `prepare_support`.
- `simulation/mujoco/runtime/standing_pose.py`: S 생성, 쿠션 재질점 XY/최저 표면 Z, J1 고정 후 X/Z를 푸는 `solve_xz`.
- `simulation/mujoco/runtime/virtual_robot.py`: phase=0.5 시작, Stop 연결, 기본 프로필/상태 응답, 시뮬레이터 안전 처리.
- `simulation/mujoco/runtime/cad_physics.py`: 서보 지연/가속도/속도·토크 한계, 충돌/접촉, 기구별 질량. 실측되지 않은 부분은 추정치다.
- `simulation/mujoco/tests/test_s_native_gait.py`: S 직접 출발, 첫 FR 각도 2배, RL/나머지 J1 유지, 대각선 기준 X/Z, Stop 복귀 회귀.
- `apps/ios/SpotOMGController/SpotOMGController/Models/ConnectionState.swift`: 목록 맨 위/지원 판별/표시명. `BLE/RobotBluetoothManager.swift`: 시뮬레이터 연결의 최신 기본 선택. 속도 표시 생성기는 `tools/generate_gait_speed_labels.py`.

가장 먼저 해결할 것은 **첫 FR/RL 실제 착지 80ms 차이**와 **두 번째 걸음에서 시작되는 기울기 증가**다. 정상/첫걸음 각도 도달은 확인했지만 실제 높이와 하중 분배가 목표와 다르다. 현재 지지 예측은 완성된 정상 발 배치의 주기 해이며 비대칭 첫걸음/전환 배치를 예측하지 않는다. 서보의 실제 추종, 몸체 반력과 자세 운동, 접촉 시점을 분리해서 확인해야 한다. 이를 특정 하나의 원인으로 확정하지 않았다.

이미 첫 J1 완료 시점, 첫 발 들기 높이/모양, 첫 스윙 기간, 한정된 자세 보정 후보를 확인했으나 착지 동기를 해결하지 못했다. 시험한 후보를 모두 활성 코드로 착각하지 않는다. 초기 각도 충돌을 숨기기 위해 충돌을 끄거나, 목표 위상 일치만으로 검증 완료라고 보고하면 안 된다. 후속 보행 변경은 기존 V5를 덮어쓰지 않고 V6 등 새 이름으로 보존한다.

사용자는 여기까지를 정리해 다른 컴퓨터에서 이어가기로 했다. 이 문서를 저장하는 작업에서는 보행 동작을 추가로 튜닝하거나 실물 펌웨어를 설치하지 않았다.
