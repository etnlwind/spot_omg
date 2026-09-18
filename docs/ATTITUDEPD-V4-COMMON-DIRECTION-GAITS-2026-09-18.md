# V4: 조이스틱 크기로 선택하는 전후 공통 보행

작성: 2026-09-18. 기존 `attitudepd_v3`를 보존하고 `attitudepd_v4`를
26번 프로필로 추가했다. 소스 펌웨어는 `attitudepd-v4-v80`이다.
**Windows 앱 갱신·시뮬레이션 검증과 실제 로봇 V80 설치를 완료했다.**
설치 후 V4 선택·Landing 도착·서보 영구 설정 보존을 확인했다. V4 실기 보행은
아직 시험하지 않았다. iOS는 0.5.0 빌드 51 소스와 XCTest를 갱신했지만,
이 Windows에서 사용할 Mac에 연결되지 않아 Xcode 빌드/기기 설치는 미실시다.

## 사용자 요청과 동작

V3는 전진/후진 방향에 따라 서로 다른 보행을 선택했다.
사용자는 “조이스틱 반쯤은 기존 전진 보행, 끝까지는 기존 후진 보행”을
전진·후진 모두에 적용하는 2단계 조절을 요청했다.

| 조이스틱 위치 | 전진·후진에 공통 적용 |
|---|---|
| 반쯤 | V3 전진형: 기본 주기 1.35초, 보폭 설정 80mm, 들림 24mm, 유지 구간이 있는 스윙, 앞발 +4mm |
| 반~끝 | 두 보행의 파라미터와 스윙 형태를 smootherstep으로 연속 전환 |
| 끝까지 | V3 후진형: 주기 1.05초, 보폭 설정 70mm, 들림 22mm, 아치 스윙, 별도 앞발 +4mm 없음 |

이동 방향은 signed linear가 정하고, 정책 선택은 `abs(linear)`가 정한다.
주기만 바꾸는 것이 아니라 보폭·높이·스윙 형태·앞발 추가 들림의 선택을
함께 바꿨다. 기존 전진/후진 템플릿의 IK 깊이 220/201.75mm도 함께 보간한다.
이는 지면부터 몸체까지의 실측 높이값이 아니다.

보폭 크기와 주기의 기존 입력 크기 조절은 유지한다. 앱의 dead zone=0.15,
minimum motion=0.30 매핑에서 실제 조이스틱 반경 0.5는 **588‰**다.
이 기준은 manifest의 `half_stick_linear=0.588`과 두 앱의 입력 시험으로
맞췄다. 따라서 반쯤 입력의 실제 명령 주기는 약 **1.54467초**,
평면 지지 X 이동폭은 **47.04mm**이며, 최대 입력은 **1.05초·70mm**다.
50%의 펌웨어 명령과 조이스틱 반경 50%를 혼동하지 않는다.

작은 입력은 기존 B에서 전진형으로 이어지는 준비 보간을 양방향에 똑같이
적용한다. 반 이하에서 보폭도 입력 크기에 따라 변한다. 반에서 끝으로
전환할 때 위상을 리셋하거나 보행을 일시 정지하는 분기는 없다.
두 기준 보행을 잇는 연속 전환이며, 갑작스러운 관절 목표 점프를 넣지 않았다.

기존 B Stand/진폭 0/정지 복귀를 유지했다. 제자리 회전(linear=0) 목표도
V3와 동일하다. 관절 영점·서보 방향·J2 누적 좌표·토크/속도/가속도 한계,
기울기 보호는 변경하지 않았다.

## 구현 경로

- `config/locomotion_profiles.json`: V4 추가, 기본 V4, 기존 프로필 순서/내용 보존.
- `locomotion.h`: 파라미터·주기에 공통 크기 기반 템플릿 가중치 적용.
- `center_pivot.h`: 스윙 유지형/아치형 및 앞발 +4mm도 같은 가중치로 선택.
  기존 함수는 V3 이전 방향별 가중치를 쓰는 래퍼로 보존.
- `virtual_robot.py`: V4 B Stand/PD/Stop 경로 연결. 표시·PD의 파라미터도
  공유 C에서 읽어 simulator에 별도 선택 공식을 중복 구현하지 않았다.
- Windows/iOS: 목록 첫 항목과 지원 연결의 기본 선택을 V4로 변경.
  `attitudepd_v4` capability가 없는 V79에는 V3, V78에는 V2를 선택한다.
  V80의 기존 probe/width 기능 인식도 유지한다.
- 기존 V3 분석·검증 스크립트는 기본 모델 변경 뒤에도 V3를 명시적으로
  선택하여 과거 결과를 재현한다.

## 검증 결과

선택 회귀 **73개 통과**: V2/V3/V4 C, gaitsteps, B Stand/시작/정지,
TCP 보정 토글/Stop, Windows protocol/probe/UI.

V4 C 검사는 전후 파라미터·주기 일치, 반/끝 경계, 스윙 형태와 +4mm 적용,
대각선 위상, 전후 X 방향 반전, 관절/서보 범위, 900프레임의 실제 drive
상태 진행과 Stop을 검증한다. 23,331개 입력·위상 조합도 실행했다.

수정 전에 저장한 centerpivot/attitudepd/V2/V3 **10,332조건**과 수정 후
목표각·주기를 비교하여 **정확히 일치**했다. 이 검사는 기존 보행 보존의
근거이며 실기 동작 검증을 대체하지 않는다.

MuJoCo는 기존 2.754kg 추정 물성/쿠션/모터 모델, 0.5ms 물리 적분,
20ms 제어, PD OFF·heading OFF를 사용했다. 각 조건 Stand 7초 → 12초
보행 → Stop 및 B 복귀. 출발 후 2초를 제외한 body root X 회귀 속도다.

| 입력 | 주기 | 몸체 속도 | 정상 구간 최대 roll / pitch | Stop |
|---|---:|---:|---:|---|
| 반 전진 +588 | 1.54467초 | 0.0595m/s | 4.35° / 2.25° | B 복귀 |
| 반 후진 -588 | 1.54467초 | 0.0431m/s | 4.68° / 2.26° | B 복귀 |
| 끝 전진 +1000 | 1.05초 | 0.1605m/s | 2.52° / 1.40° | B 복귀 |
| 끝 후진 -1000 | 1.05초 | 0.0895m/s | 8.16° / 4.04° | B 복귀 |

연속 시험: 반 전진 → 끝 전진 → 반 전진 → 반 후진 → 끝 후진 → Stop.
보호 오류 없이 B로 복귀했다. 최대 roll 8.16°, pitch 4.04°,
보행 중 프레임 간 관절 명령 변화 최대 2.66°/20ms. 최종 B 목표 오차 0°.

**같은 정책/주기 적용과 같은 몸체 이동 속도는 별개다.** 기구·접지·추종의
방향별 차이는 남아 있다. 시뮬레이션의 스윙 중 접촉도 여전히 기록되므로
‘발 끌림 해결’, ‘실물 속도 일치’, ‘실기 보행 합격’으로 해석하지 않는다.
이번 목적은 사용자가 지정한 보행 선택 규칙의 구현이다.

## 빌드와 적용 수준

- Windows 실행 파일 갱신: `apps/windows/dist/SpotOMGController/SpotOMGController.exe`.
- STM32 빌드 성공: `artifacts/attitudepd-v4/firmware/attitudepd-v4-v80.bin`,
  322,784바이트, SHA-256
  `79f465ff2238ba7f762adede957036d6da7c8aa35c6e5d962e1b1905d9c02f15`.
- 실기 설정 readback: V80 리비전·V4 선택·capability, 12개 서보 영구 레지스터 보존 확인.
- 실제 위치 유지 시험: 설치 전후 Landing 도착 확인. 별도 Stand 유지 시험은 미실시.
- 실기 전체 동작 시험: V4 보행 미실시.
- iPhone 소스/시험 추가, 빌드 번호 50 준비. Xcode 빌드·실기 설치는 미실시.

## 실기 V80 설치

사용자가 몸통이 바닥에 닿은 접힌 상태를 확인했고, 이후 모든 조종 앱 연결을
해제했다고 확인한 뒤 설치했다. 마지막 성공 시도는
`artifacts/attitudepd-v4/hardware-install/attempt-02/`에 보존했다.

1. V79에서 Landing 완료, 실제 최대 오차 **16틱** 확인.
2. Relax 후 12개 서보의 토크 레지스터가 모두 OFF인지 확인.
3. BLE OTA: ESP32 이미지 해시 검사 → STM32 Flash 검증 → 재부팅 성공.
4. `rev=attitudepd-v4-v80`, `profile=attitudepd_v4`, V4 capability 확인.
5. 설치 후 Landing `complete-residual` 완료, 최대 오차 **25틱**, safety OK.
6. 12개 서보의 영구 레지스터 0–39가 설치 전과 정확히 일치함을 확인.
7. Landing 확인 후 Relax, 12개 토크 OFF·정지·하드웨어 오류 0 확인.

최종 readback은 **torque OFF / safety OK / fault 0**, 전압 12.3–12.5V,
온도 28–36°C다. 토크를 푼 뒤 최대 오차가 86틱으로 안착하여 자세 표시는
`custom`이다. 이는 도착 당시 오차 25틱과 다른 시점의 값이며, 펌웨어의
80틱 자세 인식 범위를 벗어난 것이다. 라벨을 바꾸기 위한 재이동은 하지 않았다.
물리적 기구 끝점 각도를 새로 독립 측정했다는 뜻은 아니다.

첫 준비 기록 `hardware-install/prepare.json`도 보존했다. Landing은 22틱으로
완료됐지만 Relax 후 87틱·`custom`이 되어 설치 스크립트의 마지막 라벨 검사에서
중단됐고, 이 시도에서는 OTA를 하지 않았다. 후속 읽기에서 Stand·토크 ON이
관측되어 다시 중단했다. 상태를 바꾼 명령의 출처는 확인하지 못했다.
사용자의 모든 앱 연결 해제 확인 후 새 시도를 진행했다.

설치 스크립트는 **토크 해제 전 Landing 도착 검증을 유지**하고, 해제 후에는
자세 라벨 대신 12개 실제 토크 OFF·정지·온도/전압·오류 상태를 확인하도록 했다.
Landing 실패를 무시하거나 펌웨어의 자세 완료/보호 기준을 바꾸지 않았다.
보행 속도·가속도 설정의 Drive 중 readback이나 V4 Stand/보행 시험은 하지 않았다.

## iPhone 설치 대기

이전 설치가 확인된 앱은 UJIN17의 **0.5.0 (49)**다. 새 **0.5.0 (51)** 소스는
V4를 목록 맨 위와 지원 연결의 기본 선택으로 추가했다. 현재 접근 가능한
Mac/Xcode가 없어 새 앱의 서명 빌드와 설치는 완료하지 못했다.
빌드 51에는 [Stop 뒤 긴 진단 로그 수신 중 연결 해제 수정](IOS-STOP-DISCONNECT-2026-09-18.md)도
포함한다. 추가 XCTest는 아직 실행하지 못했으므로 Mac에서 검증 후 설치한다.

`scripts/hardware/install_ios_v4.py`는 Mac에서 빌드 → 설치 → 앱 목록의 빌드 51
readback을 수행하고 원문을 저장한다. Windows에서는 문법과 `--dry-run`만
확인했으며, 실제 Mac 실행은 미검증이다. 설치 후 앱을 자동 실행하지 않는다.

```bash
python3 scripts/hardware/install_ios_v4.py
```

기존 UJIN17 기기 식별자가 기본값이며, Mac과 기기가 페어링되고 Xcode 서명이
가능해야 한다. 다른 기기라면 `--device`로 명시한다. 이미 결과 폴더가 있으면
`--output`으로 새 폴더를 지정하여 이전 설치 증거를 보존한다.

`artifacts/attitudepd-v4/ios-v4-build51-source.zip`은 iOS 프로젝트와 설치 스크립트,
이 문서를 묶은 **소스 패키지**다. 서명된 IPA나 설치 완료 결과가 아니다.
기존 빌드 49에는 V4가 없고, 연결 후 idle 자세가 확인되면 기본 선택 로직이
V3를 선택할 수 있다. 로봇 V80 설치만으로 기존 iPhone 앱이 갱신되는 것은 아니다.

## 재현

저장소 루트, Windows:

```powershell
$env:PYTHONUTF8='1'
$env:PYTHONPATH='.;tools/servo_tool'
& 'C:/Users/etnlw/miniforge3/envs/spot_omg/python.exe' `
  simulation/mujoco/scripts/validation/validate_attitudepd_v4.py
```

`artifacts/attitudepd-v4/simulation/summary.json`에 요약,
방향/입력별 CSV에 관절·위상·위치·접촉,
`transitions.json`에 연속 조작 원문을 저장했다.
`previous-profile-reference.npz`는 변경 전 보존 검사 기준이다.
빌드 원문과 바이너리는 `firmware/`에 있다.

호스트 시험:

```powershell
& 'C:/Users/etnlw/miniforge3/envs/spot_omg/python.exe' -m pytest `
  firmware/stm32-learning/tests/test_attitudepd_v2.py `
  firmware/stm32-learning/tests/test_attitudepd_v3.py `
  firmware/stm32-learning/tests/test_attitudepd_v4.py `
  firmware/stm32-learning/tests/test_gaitsteps_limit.py `
  simulation/mujoco/tests/test_attitudepd_v3_controller.py `
  simulation/mujoco/tests/test_body_stabilizer_transport.py `
  apps/windows/tests/test_protocol.py apps/windows/tests/test_probe.py `
  apps/windows/tests/test_ui.py -q --tb=short
```
