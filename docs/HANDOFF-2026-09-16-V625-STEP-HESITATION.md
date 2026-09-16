# 2026-09-16 인수인계: V6.2.5 매 걸음 멈칫함과 배터리 경고

이 문서는 집에서 넘긴 V625 상태를 보존한다. 이후 Windows에서 진행한
[V626 구현·검증 및 남은 앞발 접촉 문제](HANDOFF-2026-09-16-V626-CONTINUOUS-RECOVERY.md)를 먼저 참고한다.

## 가장 먼저 이어갈 일

사용자의 마지막 동작 피드백은 **“한 발 걸을 때마다 발을 내딛고 멈칫한다”**다.
8초 시연 사이의 정지 간격을 말한 것이 아니다. **이 문제는 아직 원인 확정도 수정도 하지 않았다.**
보행 주기 숫자를 줄이는 것보다 실제 다리가 빠르게 이동하고 충분히 뒤로 뻗는 것이 요구사항이다.
사용자는 S 복귀 때의 빠른 이동에서 보행 속도 개선 가능성을 봤다. 이를 토크 해제 후 낙하로 설명하지 않는다.
사진의 약 90°는 아래 링크와 바닥의 관계이며 J3 명령 90°가 아니다.

다음 작업은 시뮬레이터의 한 걸음 안에서 목표/실제 관절각, 발 X/Z 속도, 위상 진행률,
추종 오차와 phase hold를 같은 시간축으로 비교하는 것이다. 궤적 자체의 착지 부근 감속과
추종 제어기의 정지를 분리한다. 회수 수평 이동이 swing 20~80%에 집중된 영향도 확인한다.
보호를 끄거나 물리 가속도를 이상적인 값으로 바꿔 문제를 숨기지 않는다.

## 실제 장치와 소스 상태

| 대상 | 상태 |
| --- | --- |
| 실제 로봇 | V70 `s-native-v6-2-5-v70`, 모델 V6.2.5 |
| 현재 펌웨어 소스 | V71: 배터리 telemetry 추가, 빌드 완료, **OTA 미실시** |
| 마지막 실제 상태 | torque off, safety ok, fault 0, 12축 hw 0 |
| 마지막 전압 | 정지 후 10.8~10.9V, 30초 입력 시험 최저 10.3V |
| Mac 앱 | 0.5.0(46), Catalyst 빌드/서명 확인, 로컬 Applications에 배치 |
| iPhone 앱 | 서명 빌드 성공, 기기 잠금으로 설치 실패; 설치 완료 아님 |
| Windows 앱 | 0.1.1 소스 및 Qt 호스트 검사 완료; Windows exe 배포 미검증 |

사용자는 충전을 위해 실기 시험을 중단했다. 마지막 전압은 과거 측정이며 현재 전압이 아니다.
다른 컴퓨터에서 연결했다고 곧바로 보행을 실행하지 말고 충전 및 현재 지지/연결 상태부터 확인한다.
이 인수인계 작성 중에는 실기 구동이나 OTA를 하지 않았다.

## 보행과 서보 변경 요약

- V6.2.2~V6.2.5를 새 이름으로 추가했다. V625는 펌웨어 인덱스 21이며 기존 인덱스 유지.
  지원 앱 목록 맨 위/기본값, 시뮬레이터 기본 모델도 V625다.
- 느린 동작의 확인된 원인 중 하나: RAM profile은 3400/254였지만 Stand가 실제 서보에
  speed 300 / acceleration 30을 남겼고, 위치만 쓰는 보행이 이를 복구하지 못했다.
- V69 이후 실제 레지스터 readback을 검증한다. 설치된 서보에서 확인한 값은
  speed 3400, J1/J3 acceleration 50, J2 acceleration 254다. 50을 모든 STS3250의
  제조사 공통 한계로 일반화하지 않는다. EEPROM 최대값 레지스터는 미확인이다.
- 시뮬레이터도 축별 가속도 제한과 서보 설정의 지속성을 반영했다.
- V625 추종 감속 회복은 0.2/s → 2.0/s, 관절 피드백은 약 240ms → 120ms.
  6~14° 감속, 14° 위상 정지, stale/지속 실패 보호는 유지한다.
- 기본 주기 1초, 목표 앞 +20 / 뒤 -125mm. 회수 초반 20% 들어 올림,
  수평 이동 20~80% quintic. S 출발, FR·RL 동시 첫걸음, FR 첫 J1 변화량 2배,
  이후 정상 오므림 및 Stop→S 규칙을 유지한다.
- jointtrace는 256명령+512표본, 총 768행. `jointtrace stop`은 다음 Stop 기록 예약이다.

위치/방향/좌표를 바꾸기 전에 [STS3250 실기 기준](STS3250-POSITION-CONTROL.md)과
루트 AGENTS.md를 읽는다. 세부 변경은 [V625 기록](S-NATIVE-V625-2026-09-15.md),
[이전 실기 인수인계](HANDOFF-2026-09-15-V625-FAST-RECOVERY.md),
[서보 설정 복원](SERVO-PROFILE-RESTORE-2026-09-15.md)을 따른다.

## 검증 결과와 한계

- 실제 V70의 몸체 고정/발 공중 30초 입력 시험은 정상 정지했다. 내부 S 진입 포함이며
  순수 보행 30초 또는 정확한 30회가 아니다. elapsed 28448ms, 2828 samples,
  lag/droop 121, late 8, fall no, derate recommended.
- 공통 240ms 격자의 초기 구간 비교에서 J2 평균 이동량/초 17.85→25.58°/s,
  J3 14.36→22.37°/s. 순간 최고 속도나 몸체 이동 속도가 아니며 전압 동일 A/B도 아니다.
- 추종 제어기 기록 재생 평균 위상 진행률은 약 0.356→0.508. J3 표본 오차는
  최대 약 18.5~19.5°로 여전히 크다. 실제 뒤쪽 링크 각도 목표 달성은 미확인이다.
- Stop readback error 8(약 0.70°), torque off. 독립적인 토크 유지 시험은 아니다.
- 클럭 readback은 실제 84MHz, APB1 42 / APB2 84MHz, SysTick 83999.
  compute_max 14ms, io_max 36ms. 클럭이 주원인이라는 근거는 없으며 클럭 변경 안 했다.
- 바닥 시뮬레이션은 기울기 fault가 남아 있다. 지상 보행 성공으로 보고하지 않는다.
  실제 가속도 제한 적용 후 기존 V621 물리 검사 2개 실패도 남아 있다.
- 기존 검증: V625 관련 83개, C 7개, trace 17개 통과. 추가 검사 49개 통과/위 2개 실패.
  각 로그는 artifacts/s-native-v6-2-5 아래에 있다.

## 기록 위치

- [30초 실기 원본/복구 기록](../artifacts/s-native-v6-2-5/hardware-30s/).
  첫 trace 전송은 BLE timeout이었고 summary.json의 error는 그 전송 오류다.
  추가 동작 없이 재연결해 같은 RAM trace 전체를 복구했다. 최종 분석은
  jointtrace-retry.txt, trace-recovery.txt, post-run-readback.txt, joint-analysis/를 사용한다.
- [실시간 시뮬레이터 기록](../artifacts/s-native-v6-2-5/viewer-supported-realtime-2026-09-15/).
- [초기 15초 시뮬레이터 기록](../artifacts/s-native-v6-2-5/viewer-supported-2026-09-15/):
  시작 후 11.58초에 tracking fault. fault 이후 중력 낙하 오차를 정상 추종값으로 쓰지 않는다.
- [배터리 경고 검증](../artifacts/battery-warning-2026-09-15/).
- 분석 도구는 scripts/analysis/, 실기 도구는 scripts/hardware/에 있다.
  실기 도구는 읽기 전용이 아닐 수 있으므로 실행 전 내용을 확인한다.

## 새 컴퓨터 준비와 시뮬레이터 재현

```sh
git clone https://github.com/etnlwind/spot_omg.git
cd spot_omg
git switch develop
git pull --ff-only
conda env create -f config/environment.yml
conda activate spot_omg
python -m pip install -e tools/servo_tool
```

이미 환경이 있으면 새로 생성하는 대신 필요한 의존성을 갱신한다. 이전 Mac은 Python 3.10,
MuJoCo 3.11.0을 사용했다. 로컬 Bleak은 3.0.2였고 환경 파일의 선언은 >=0.22,<2이므로
새 환경의 BLE 연결은 별도로 검증한다. editable 설치 상대 경로가 실패하면 마지막 명령으로 설치한다.

Mac에서 현재 몸체 고정 시연:

```sh
mjpython simulation/mujoco/runtime/supported_demo.py \
  --output artifacts/s-native-v6-2-5/viewer-resume-2026-09-16
```

Windows는 같은 명령에서 mjpython을 python으로 바꾼다. 출력 디렉터리는 새 이름이어야 한다.
W로 반복 시작, Space로 정지한다. MuJoCo 기본 W가 wireframe도 전환하므로 필요하면 W를 다시 누른다.
기본값은 V625, 100% 입력, 10.9V 모델, 몸체 +0.3m 고정/발 공중, 8초 구동 후 정상 S 복귀 반복이다.
이 전압은 재현용 설정이다. 실제 전압과 자동 동기화하지 않는다. 하드웨어 연결 코드는 없다.
50Hz 물리/25Hz 화면으로 1.00x 재생을 확인했다. 첫 120초만 10Hz JSONL을 기록한다.
8초 구동 안의 매 걸음 멈칫함이 다음 분석 대상이며 시연 사이의 쉬는 시간과 구분한다.

기본 측면 카메라는 azimuth 90°, elevation 0°, distance 1.05m, 중심 -0.1m이다.
사용자가 지정한 LG FULL HD 모니터에 창을 표시하고 마우스 시점 변경을 유지한다.
기존 simulation/mujoco/virtual_robot.py 및 walk.py 진입점도 유지된다.

## 배터리 경고와 배포 후속 작업

세 앱 공통: 11.0V 이하 즉시 충전 경고, 10.5V 이하 즉시 사용 중단 경고.
11.4V 이상을 최소 3회/5초 확인해야 해제한다. 확인 버튼 후에도 빨간 배너는 남는다.
보행 정지는 기존 정상 Stop이며 자동 토크 해제는 추가하지 않았다.
V71은 기존 관절 읽기에서 받은 전압을 1초 최소값으로 `$BATTERY mv=...` 전송한다.
V70에서는 보행 중 실시간 전압 경고가 제한되므로 앱 소스만 배포해 완료했다고 하지 않는다.

[배터리 기능 문서](APP-BATTERY-WARNING-2026-09-15.md)에 정책/검증/배포 한계를 기록했다.
Windows 정책/UI/연결 53개 통과는 macOS Qt offscreen 검사다. Swift 정책, C telemetry,
Mac Catalyst 및 iPhone 기기용 빌드는 통과했지만 실제 보행 중 telemetry는 미검증이다.

- Mac: [Catalyst 빌드 명령](../apps/macos/README.md). 다른 컴퓨터에서는 다시 빌드한다.
- iPhone: Xcode 서명 설정 후 잠금 해제된 실제 기기에 설치하고 경고를 확인한다.
- Windows: [앱 안내](../apps/windows/README.md)의 setup.ps1/run.ps1/build.ps1 사용.
- 펌웨어: ARM GCC 준비 후 `python firmware/stm32-learning/build_firmware.py --help` 확인.
  빌드 결과와 OTA는 구분한다. 충전 후 V71 OTA/readback/실시간 수신 검증이 남아 있다.

V71 바이너리: artifacts/battery-warning-2026-09-15/firmware-v71/s-native-v6-2-5-v71.bin
(326668 bytes), SHA256 `9a7d005b4e9feb0dd1e86ec29688ff89d9c85e872db6d7e3e49af5165e0b6b73`.
실제 설치 V70 바이너리는 artifacts/s-native-v6-2-5/firmware-initial/에 보존했다.

## 빠른 호스트 검사

저장소 루트, 활성화된 환경에서:

```sh
python tools/generate_locomotion_profiles.py --check
python tools/generate_gait_speed_labels.py --check
python -m pytest -q simulation/mujoco/tests/test_s_native_v625.py \
  simulation/mujoco/tests/test_servo_profile.py tools/servo_tool/tests/test_joint_trace.py
QT_QPA_PLATFORM=offscreen python -m pytest -q apps/windows/tests
```

마지막 명령은 Qt가 필요하다. macOS에서 Windows UI를 검사하려면 PySide6를 별도로 설치한다.
검사 수준은 설정 readback / 호스트 검사 / 실제 위치 유지 / 전체 실기 보행으로 구분한다.
빌드 캐시·호스트 실행 파일은 Git에서 제외하며 소스, 로그, 분석 결과, 펌웨어 bin은 보존한다.
기존 별도 작업인 hardware/power-junction/은 이번 인수인계 커밋에 포함하지 않는다.

### 인수인계 직전 재검사 (2026-09-16)

두 생성기 `--check` 및 소스/문서의 `git diff --check` 통과. 원본 artifacts 로그/CSV의 줄 끝 공백과 CRLF는 보존했다.
Windows 전체 tests + V625 + servo_profile + joint_trace를 함께 실행한 결과
**88 passed / 1 failed**였다. 실패는
`apps/windows/tests/test_stop_return_simulation.py::test_app_stop_waits_for_v6_1_physical_return`:
기대하는 `@S` 패킷 1개 대신 0개였다. 원인은 이번 인수인계 작업에서 진단하지 않았다.
이 결과는 앞서 배터리 관련 53개 통과와 별개이며 전체 회귀가 모두 통과한 상태는 아니다.
MuJoCo mesh 계산에 RuntimeWarning도 발생했다. 다음 작업에서 재현/분석할 항목으로 남긴다.
