# 실제 로봇 / 가상 로봇 연결 (2026-09-09)

후속 업데이트: [가상 BLE와 iPhone V0.3.0 (6)](VIRTUAL-BLE-2026-09-09.md).
이제 BLE도 지원하며 iPhone 설치와 연동 검증이 완료되었다. 아래는 최초 TCP 구현의 상세 기록이다.

## 구현 상태

앱은 **실제 로봇 · BLE**와 **가상 로봇 · TCP**를 명시적으로 선택한다.
대상 변경은 기존 연결과 조이스틱 세션을 해제한다. 새 대상 연결은 별도 버튼으로 시작한다.
TCP 연결은 `$SPOTBACKEND backend=sim protocol=1` 식별 응답 후에만 제어 가능하다.
실제 장치로 자동 대체 연결하지 않는다. 앱에는 가상 전압, 추정 물성, IMU 모니터링 상태를 표시한다.

spotctl은 `--sim-host`로 가상 대상을 명시한다. 하드웨어 주소/전송 옵션과 충돌하면 거부한다.
TCP 소켓 연결 직후 식별을 검증하며 실패 시 연결을 닫는다. 펌웨어 배포는 거부한다.
기존 BLE/USB/TCP 실물 연결 경로는 유지한다.

## 실행

프로젝트 루트에서:

```sh
PYTHONPATH=tools/servo_tool:simulation/mujoco /opt/anaconda3/envs/spot_omg/bin/mjpython simulation/mujoco/virtual_robot.py --host 0.0.0.0 --port 8765 --viewer
```

화면 없이 실행하려면 `mjpython` 대신 같은 환경의 `python`을 사용하고 `--headless`를 지정한다. 기본 실행은 MuJoCo 창을 표시한다.
기본 바인드는 로컬 전용 `127.0.0.1`이다. 위 명령은 LAN에서도 접속 가능하게 연다.
동일 Wi-Fi의 iPhone에서는 앱의 **가상 로봇 · TCP**를 선택한 뒤 Mac의 LAN IP와 `8765`를 입력하고 연결한다.
iPhone에서 `127.0.0.1`은 Mac 주소가 아니다. iOS 로컬 네트워크 권한이 필요하다.
이 서버는 인증 없는 개발용 콘솔이며 동시에 한 제어 연결만 허용한다.

```sh
PYTHONPATH=tools/servo_tool /opt/anaconda3/envs/spot_omg/bin/python -m servo.cli --sim-host 127.0.0.1 --tcp-port 8765 console send syncstate
PYTHONPATH=tools/servo_tool /opt/anaconda3/envs/spot_omg/bin/python -m servo.cli --sim-host 127.0.0.1 --tcp-port 8765 stand
PYTHONPATH=tools/servo_tool /opt/anaconda3/envs/spot_omg/bin/python -m servo.cli --sim-host 127.0.0.1 --tcp-port 8765 console send trot5 3 844
```

앱이 연결 중이면 spotctl 연결은 거부된다. 다른 클라이언트를 사용하려면 먼저 연결을 해제한다.
서버 종료는 터미널 Ctrl+C 또는 MuJoCo 창 닫기이다.

## 명령과 실제 물리 처리

- `identity`, `syncstate`, `read 1`: 대상/상태/모델 전압 응답.
- `stand`, `stand11`, `landing`, `hold`, `relax`, `recover`.
- `drive LINEAR YAW SEQ`, `@D SEQ LINEAR YAW`, `@S SEQ`, Ctrl+C.
- `trot5`, `trot4`, `trot4back`, `turn left/right`, `crab left/right`와 주기/횟수.
- `gaitdiag`, `baldiag`, `targets`: **시뮬레이터 전용 JSON 진단 형식**으로 위치·각도·토크·접촉·전압을 반환. STM32 진단 출력과 동일한 스키마는 아니다.
- `echo off/on`, `log time EPOCH`: 콘솔 클라이언트의 초기화 호환. 시뮬레이션 시간은 단조 시간 기준이며, 로그 저장/RTC 기능은 제공하지 않는다.
- 나머지 명령은 `ERROR: unsupported simulator command`로 거부한다. EEPROM, OTA, 실제 버스 스캔/캘리브레이션을 흉내 내어 성공 처리하지 않는다.

기존 STEP 기반 관절 모델에 중력·관성·접촉·마찰·모터 토크/속도 제한·지연·전원 강하를 적용한다.
제어는 20ms, 물리 적분은 0.5ms이다. 동작 중 qpos를 직접 써서 목표 자세로 이동시키지 않는다.
`relax`는 구동 토크를 0으로 만들며 중력은 계속 작용한다.
명령 종료 프롬프트는 자세 전환 후 반환한다. 이는 기계적인 완벽한 추종을 의미하지 않으며 `syncstate error`와 실제 관절 상태로 오차를 확인해야 한다.

드라이브 패킷 순서 번호를 검사하고 800ms 동안 갱신이 없으면 정지 전환한다.
연결 종료도 정지 전환을 시작한다. 전환 완료 전 새 소유자를 받지 않는다.
물리 모델은 `--parameters PATH.json`으로 교체 가능하며, 현재 **항상 estimated**로 표시한다.
측정 근거 없이 calibrated라는 이름으로 바꾸지 않는다.

## 공유와 차이 — 실물 동일성의 검증 범위

| 계층 | 현재 상태 |
|---|---|
| 보행 궤적 | 펌웨어 `gait_policy.h`를 호스트 C 라이브러리로 컴파일하여 직접 호출. drive는 V16 40/80° 정책 |
| 입력 변화 제한/보행 주기 | `gait_policy_drive_slew`, `gait_policy_drive_period_ms`로 추출, STM32와 Python 모두 같은 C 정수 계산 호출 |
| 명령/세션 처리 | 가상 서버의 Python 어댑터. STM32 콘솔 전체를 재호스팅한 것이 아님 |
| 자세 전환/종료 | 가상 서버에서 1초 보간. 실제 firmware의 측정 위치 기반 전환·정지 감속·시간 양자화·제한기와 완전히 같지 않음 |
| IMU | 물리 기울기 관측과 12° 정지. 실제의 필터·참조 자세·연속 프레임 판정·균형 보정 루프는 미공유 |
| 모터/전원 | 추정 모터 모델. 실제 서보 내부 PID·발열·유격·전원 차단·셀별 방전 모델 미검증 |

따라서 **현재 구현은 실제 로봇 전체를 완벽히 대체하거나 sim-to-real 성공을 보장하지 않는다.**
프로토콜을 통한 조작과 공통 C 궤적 검증을 위한 실행 가능한 연결 기반이다.
동일 제어기 검증의 다음 경계는 STM32의 자세 전환/액추에이터 제한기/IMU 필터/안전 상태기를 HAL과 분리하여
동일 입력·시간·센서 기록에 대해 프레임별 목표값과 종료 사유가 일치하는 재생 검증을 만드는 것이다.
그 이후 새 정상 전원에서 질량·무게중심·지연·토크·마찰을 측정하고 실물 시나리오를 비교해야 한다.

실제 로봇에는 이번 변경을 설치하지 않았다. 기존 V16 릴리스 바이너리는 이전 빌드이며,
이번 공통 함수 리팩토링 소스는 별도로 재빌드하여 검증했다. 배터리가 없는 상태에서 실물 동작을 시도하지 않았다.

## 검증 기록

- Python·펌웨어 호스트 전체: TCP 포함 222개 및 subtest 26개 통과.
- 가상 서버에 spotctl `syncstate`를 실제 TCP로 전송하여 `backend=sim`과 V16 응답 확인.
- 화면이 있는 MuJoCo 서버에 `trot5 3 844` 전송 및 정상 OK 종료 확인.
- 동일 연결에서 5초 전진 입력과 200ms heartbeat 후 정지: X 약 +0.0612m, Y 약 -0.00195m.
  정지 후 roll +0.059°, pitch -0.224°, started/stopped/OK 응답 확인. 단일 시나리오이며 실물 정확도 증거는 아님.
- STM32 소스 빌드: 137812 bytes, SHA256 `97d1a00911feb0b861f7b6e041c334908b7c7497505e38e768151e6e82b3f252`.
- iOS 테스트 32개 통과. TCP 식별→syncstate→전압 왕복, 대상 변경 시 세션 제거, 잘못된 식별 거부 포함. 후속 V0.3.0 (6)에서 iPhone 설치 및 BLE 연동 검증 완료.
