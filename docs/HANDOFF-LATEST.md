# 최신 인수인계

**최신 변경: Windows 0.1.4 + STM32 V81 + ESP32 전용 제어 채널.**
사용자가 로그를 유지하고 제어 통신을 콘솔과 분리하도록 요청했다.
[V81 제어 채널 문서](CONTROL-CHANNEL-V81-2026-09-18.md)를 먼저 읽는다.
기존 0.1.3의 5초 대기 후 재조회는 구형 연결의 호환 경로로만 남긴다.
107개 호스트 검사 및 최종 전송/파서 관련 22개 검사 통과, 세 패키지 빌드 완료.
실기 설치/조회 결과는 `artifacts/control-channel-v81/`에 기록한다.
현재 설치 상태: STM32 V81, ESP32 전용 제어 브리지, Windows 0.1.4 설치 완료.
ESP32 첫 OTA는 60% 이후 ACK 시간 초과였으나, 사용자가 요청한 무선 재시도는
Landing 도착(오차 26틱) → 12개 토크 OFF 재검증 후 100% 전송/무결성 검증에 성공했다.
증거: `artifacts/control-channel-v81/bridge-retry-02/`.
새 제어 채널 실기 조회 검증도 통과했다(`hardware-control-query.json`).
로그 출력이 남은 상태에서 syncstate 응답/DONE이 0.266초에 완료됐고,
이후에도 콘솔 로그가 계속 수신됐다. Landing·토크 OFF·safety OK를 확인했다.
실제 보행 정지 후 재조작 시험은 아직 하지 않았다. 첫 OTA 중단 원인은 미확정이다.

## 다음 컴퓨터에서 이어가기 — 2026-09-18 V81

### 현재 설치와 실행

- 실제 STM32: `attitudepd-v4-v81`, 보행 프로필 `attitudepd_v4`.
- ESP32: 전용 제어 RX UUID 끝자리 `005`, TX indication `006`이 있는 브리지 설치 완료.
- Windows: 0.1.4 설치·실행 완료. 이 PC 설치 경로는
  `%LOCALAPPDATA%/Programs/SpotOMG/V4-0.1.4/SpotOMGController.exe`.
  바탕 화면/시작 메뉴 바로가기도 이 버전을 가리킨다.
- 마지막 실제 조회: Landing, torque OFF, safety OK, fault_code=0.
  다음 작업 때는 현재 상태를 다시 조회한다. 자동 보행 재개는 하지 않는다.
- iPhone: 빌드 51 수정 소스 포함. 이번 전용 제어 채널의 iOS 클라이언트는
  아직 구현하지 않았으며, 실제 iPhone 설치 완료로 해석하면 안 된다.

### 해결 내용과 검증 범위

V4는 전진/후진 모두 같은 반 입력/최대 입력 보행 정책을 사용한다.
V81 통신 변경은 콘솔 로그를 유지하면서 제어 요청/응답을 분리한다.
Windows는 신규 BLE 특성과 MCU `controlv1` 능력을 모두 확인한 뒤 전용 채널을
사용하며, 요청 ID가 일치하는 DONE으로 완료를 판정한다. 콘솔 프롬프트나
오래된 응답은 현재 제어 명령을 완료시키지 않는다.

107개 호스트 검사와 후속 관련 검사, STM32/ESP32/Windows 빌드, 패키지 시작
검사 통과. 후속 검사 개수는 중복되므로 합산하지 않는다.
실기 설치 후 조회에서 help 0.046초, syncstate 0.266초, gaitdiag 0.109초의
제어 완료를 관측했다. syncstate 완료 후에도 콘솔 출력이 계속 도착했다.
이는 조회 1회 검증이며 보행 정지 지연이 해결됐다는 전체 동작 검증은 아니다.
로그 버퍼는 유한하며 초과 시 손실량을 표시한다. 무제한 무손실 보장은 아니다.

### 다음 작업 우선순위

1. 실제 로봇 상태와 바닥 시험 준비를 확인하고 Windows 0.1.4를 연결한다.
   앱에 `전용 제어 채널` 표시가 있는지 확인한다.
2. 중간 전진 → 최대 전진 → 정상 정지 → 다음 조작을 시험하여 정지 후
   컨트롤러가 막히지 않는지, 콘솔 로그가 계속 표시되는지 기록한다.
3. 마우스를 계속 누르는데 `@S`가 발생했던 별도 문제는 실기에서 아직
   원인 확정/재현되지 않았다. 입력 해제 이유·시각 로그와 실제 마우스 상태를
   함께 확인한다. 모의 11초 유지 시험 성공만으로 해결됐다고 판단하지 않는다.
4. iOS에도 같은 분리가 필요하면 제어 특성/요청 ID/DONE 처리와 구형 연결
   호환을 구현하고 Mac에서 빌드·실제 설치/정지 시험을 별도로 수행한다.

읽기 전용 확인 도구는 `scripts/hardware/verify_control_channel.py`이다.
앱 연결을 해제한 상태에서 저장소 루트의 PowerShell에서 실행한다.
출력 경로는 기존 증거를 덮어쓰지 않는 새 이름으로 지정한다.

```powershell
$env:PYTHONPATH='.;tools/servo_tool'
python -u -m scripts.hardware.verify_control_channel --output artifacts/control-channel-v81/readonly-next.json
```

필요 패키지/새 PC 빌드 절차는 `apps/windows/README.md`를 따른다.
하드웨어 업데이트는 AGENTS.md의 Landing 실제 도착 → 토크 OFF 절차를 따른다.
성공한 V81/브리지를 검증 목적으로 다시 설치할 필요는 없다.

### 증거 위치

- `artifacts/control-channel-v81/verification.json`: 전체 검증 수준/이미지 해시.
- `hardware-install/recheck-01/verify.json`: V81 설치 후 Landing/설정 보존.
- `bridge-retry-02/prepare.json`, `install.json`, `ota.txt`: 무선 재시도 성공.
- `hardware-control-query.json`: 실기 제어 응답과 독립 콘솔 수신 시각.
- `bridge-install.json`, `bridge-ota.txt`: 첫 실패 기록. 재시도 성공으로 삭제하지 않는다.

---

아래 내용은 V80/0.1.3 작업 당시의 기록이다.

**추가: Windows 0.1.3 설치 완료, 기존 앱 종료 후 실행 대기.**
0.1.2에서도 정상 Stop 후 ID11 진단 중간에서 출력이 끊겨 프롬프트 시간 초과가
재발했다. 0.1.3은 정상 Stop ACK 이후에 한해 상태 재조회를 한 번 시도하고,
새 상태와 프롬프트가 모두 도착해야 복구한다. 예약 동작/자동 보행 재개는 없다.
Windows 85검사 통과. 실제 BLE 진단 조회는 0.61초에 끝까지 도착했으므로
항상 느린 BLE라는 결론은 철회한다. 계속 누른 중간→최대 입력에서 최초 `@S`가
발생한 원인은 미확정이며, Qt+통신 통합 시험의 11초 유지에서는 재현되지 않았다.
원인별 입력 해제 로그를 추가했다. 실제 0.1.3 재보행 검증은 미실시다.
iOS 빌드 51에는 이번 출력 누락 복구가 아직 없다. 자세한 기록:
[0.1.3 재발 분석](WINDOWS-STOP-DISCONNECT-2026-09-18.md).

[2026-09-18 V4 전후 공통 2단 보행](ATTITUDEPD-V4-COMMON-DIRECTION-GAITS-2026-09-18.md)을 먼저 읽는다.

추가: [Windows 정상 Stop 후 연결 해제 수정](WINDOWS-STOP-DISCONNECT-2026-09-18.md).
V80 정상 정지 후 진단 출력을 받던 중 앱의 고정 5초 제한으로 연결을 닫는 문제를
로그 재생으로 재현했다. Windows 0.1.2를 설치·실행하고 바로가기를 갱신했다.
77개 Windows 검사 + MuJoCo Stop 회귀 1개 통과, 실제 BLE 재보행 검증은 미실시다.
설치 위치는 `%LOCALAPPDATA%/Programs/SpotOMG/V4-0.1.2/`다.

현재 작업 소스 기본은 `attitudepd_v4`, 새 펌웨어 빌드는 `attitudepd-v4-v80`이다.
사용자 요청대로 반 조이스틱에서 V3 전진형, 끝에서 V3 후진형을 전진/후진
공통으로 적용하며 중간은 연속 전환한다. Windows 실행 파일 갱신,
호스트 73개 통과, 4조건 보행과 단계/방향 전환·Stop 시뮬레이션 완료.
기존 4개 모델 10,332조건 목표·주기는 정확히 보존했다.
**실제 로봇에 V80 설치 완료.** 설치 전 Landing 오차 16틱 → 토크 OFF → OTA,
설치 후 V4 선택·Landing 오차 25틱·12개 서보 영구 설정 보존을 확인했다.
마지막은 토크 OFF·safety OK·fault 0. 토크 해제 후 86틱으로 안착하여 자세
표시는 `custom`이며, 도착 실패와 혼동하지 않는다. V4 Stand/보행 실기 시험은
아직 하지 않았다. 증거: `artifacts/attitudepd-v4/hardware-install/attempt-02/`.

**iPhone은 빌드 51 소스 준비, 실제 설치는 Mac 연결 대기다.** 기존 기기는
0.5.0 (49)이며 V4를 모른다. Mac에서 `python3 scripts/hardware/install_ios_v4.py`로
빌드·설치·버전 확인을 진행한다. 스크립트는 Windows 문법/dry-run만 검증했다.
`artifacts/attitudepd-v4/ios-v4-build51-source.zip`은 소스이며 IPA가 아니다.
기존 iPhone 앱의 기본 선택은 연결 후 V3로 바꿀 수 있으므로 구분한다.

[iPhone Stop 연결 해제 수정](IOS-STOP-DISCONNECT-2026-09-18.md)도 빌드 51에 포함했다.
Windows와 같은 고정 5초 종료 로그 제한을 수정하고 실제 로그 fixture를 쓰는
XCTest 6개를 추가했다. Swift/Xcode 부재로 컴파일·XCTest 실행은 아직 못 했으므로
Mac에서 전체 RobotCommandTests를 먼저 실행한 뒤 설치한다.

작업 시작 기준 커밋은 `2450d32`(2026-09-18 03:08 KST), 당시 펌웨어는
`attitudepd-v3-v79`, 모델은 `attitudepd_v3`였다. 이번 V4 변경은 아직 커밋하지 않았다.
[V3 설치 기록](ATTITUDEPD-V3-GAIT-STAND-2026-09-18.md)을 함께 읽는다.
Stand와 보행 대기 자세를 B로
통일했다. 직전 [V2 실기 8걸음](ATTITUDEPD-V2-FRONT-CLEARANCE-2026-09-18.md)은
정상 종료했고 사용자가 양호하다고 평가했다.

9월18일 이 컴퓨터에서 재개하며 사용자도 “보행은 실기에서 잘 되었어”라고
확인했다. 해당 추가 관찰의 정확한 모델/버전은 지정되지 않았으므로 V3의
별도 Stand 유지 시험까지 완료한 것으로 확대하지 않는다. 처음에는 사용자
요청대로 시뮬레이션부터 확인했고, 이후 설치 요청으로 V80을 실기에 반영했다.
후속 V4는 사용자가 명시한 전후 공통 보행 선택 변경이며 V3를 보존했다.

재개 후 [V3 전후진 속도 차이](ATTITUDEPD-V3-DIRECTION-SPEED-2026-09-18.md)를
분석했다. 이전 회전용 설정이 후진에도 적용되어 후진 발 교대가 28.57%
빠르다. 다만 추정 MuJoCo의 몸체 이동은 전진이 더 빨라 사용자 실물 관찰을
재현하지 못했다. 양자를 혼동하지 않는다. 보행 파라미터/펌웨어 변경이나
실기 구동 없이, 네 조건의 속도·접촉 원문과 Windows 호스트 63개 통과를 기록했다.

[V77 인수인계](HANDOFF-2026-09-17-V77-STANDARD-HEIGHT-BALANCE.md)는 과거
`s_native_v6_2_7`의 원인 분석이다. 그 실패 결과를 현재 V2/V3의 실기 결과에
그대로 적용하지 않는다. `artifacts/imu-trace-v78/`의 과거 미설치 진단 후보와
실제로 설치·시험된 `attitudepd-v2-v78`도 다른 바이너리다.
