# V4 / Windows 0.1.4 업데이트

V81 STM32와 새 ESP32 브리지에서는 콘솔 로그와 제어 응답을 별도 BLE 특성으로
처리합니다. 앱에 `전용 제어 채널`이 표시되는지 확인하세요. 상세 로그는 계속
출력되며, 로그 프롬프트를 기다리지 않고 명령 번호별 완료 응답으로 조작을 재개합니다.
[구조·검증·설치 기록](../../docs/CONTROL-CHANNEL-V81-2026-09-18.md).
구형 브리지/펌웨어에서는 아래 0.1.3 방식으로 연결됩니다.

최신 모델은 `attitudepd_v4`이며 실제 로봇에는 `attitudepd-v4-v80`을 설치했습니다.
[최신 인수인계](../../docs/HANDOFF-LATEST.md)와
[V4 구현·검증](../../docs/ATTITUDEPD-V4-COMMON-DIRECTION-GAITS-2026-09-18.md)을 확인하세요.
Windows 0.1.3은 정상 Stop 확인 후 진단 끝부분이 누락되면 상태를 재조회해 연결을 복구합니다.
계속 누른 상태에서 불필요한 Stop이 발생한 원인은 조사 중이며 원인별 입력 로그를 추가했습니다.
[원인·재현·설치 기록](../../docs/WINDOWS-STOP-DISCONNECT-2026-09-18.md).
빌드 결과는 `dist/SpotOMGController/SpotOMGController.exe`입니다.

## 과거 V6.2.7 기록

**실기 첫걸음 전도 실패가 보고됐습니다.** [최신 인수인계](../../docs/HANDOFF-2026-09-16-V627-FIRST-STEP-FALL.md)를 확인하세요.

최신 소스는 V6.2.7 이른 접힘 모델을 지원하며 해당 capability가 있는 연결에서 기본 선택합니다.
현재 빌드는 `dist/v627/SpotOMGController/SpotOMGController.exe`입니다.
기존 V6.2.6도 선택할 수 있습니다. [설치·검증 범위와 남은 문제](../../docs/S-NATIVE-V627-INSTALL-2026-09-16.md)를 확인하세요.

# Spot OMG Windows Controller

현재 iOS 조종기의 BLE 콘솔·조이스틱 프로토콜에 대응하는 Windows 데스크톱 앱입니다.
Python / PySide6 / Bleak로 구현했으며 STM32 펌웨어와 관절 좌표·보행 수식은 변경하지 않습니다.

## 이 컴퓨터에서 실행

빌드된 `dist/SpotOMGController/SpotOMGController.exe`를 실행합니다.
같은 폴더의 `_internal`도 필요하므로 배포할 때 폴더 전체를 복사합니다.

- **MuJoCo 시작 + 연결**: 로컬 시뮬레이터를 시작하고 TCP identity 확인, 상태 동기화,
  영상 표시를 수행합니다. 기본 포트는 제어 8765, 영상 8766입니다.
  기존 실측 총중량 2.754kg 설정(`physics_parameters_measured_total_2754g.json`)을 사용합니다.
  원형 외경 37.3mm·전체 길이 27mm 쿠션이 포함되며, 질량 분포와 접촉 물성은 추정값입니다.
  S-native V6.2.1, V6.2, V6.1과 V6~V1을 목록 상단에 표시하고, 초기 상태 확인 후 시뮬레이터가 지원하는
  최신 버전을 한 번 선택합니다. 각 버전은 실험 모델이며 안정 보행 검증은 별개입니다.
- **별도 3D 뷰어도 열기**: MuJoCo의 카메라 조작 가능한 창도 함께 엽니다.
  Windows에서는 일반 Python으로 실행하며 macOS 전용 `mjpython`이 필요하지 않습니다.
- **실제 로봇 · BLE**: iPhone 앱에서 로봇 연결을 먼저 해제하고 연결하기를 누릅니다.
  Windows의 Bluetooth가 켜져 있어야 하며 BLE를 지원하는 어댑터가 필요합니다.
  `SpotOMG-Bridge`의 Nordic UART 서비스로 연결합니다. Bluetooth SPP COM 포트는 사용하지 않습니다.
- **가상 로봇 · BLE**: 기존 Mac의 `SpotOMG-Sim` 브리지가 이미 실행 중일 때 접속할 수 있습니다.
  Windows 로컬 MuJoCo는 TCP를 사용하고 Mac 전용 Swift BLE 브리지를 실행하지 않습니다.

처음부터 로봇에 자동 연결하지 않습니다. 통신 오류 후에는 수동으로 재연결하며
이전 조이스틱 입력이나 명령을 재생하지 않습니다. STM32 v63과 실기 BLE 연결,
상태 조회 및 V6.2.1 기본 선택을 확인했습니다. 실제 보행 검증과는 별개입니다.

## 조작

- 마우스로 스틱을 누른 채 드래그합니다. 놓으면 감속 정지를 요청합니다.
- 키보드 조종을 켜면 WASD/방향키를 누른 동안 조작합니다. 마지막 키를 놓으면 정지합니다.
  명령 입력창에는 조종 키가 적용되지 않습니다. `Esc` 또는 입력창 밖의 `Space`는 Stop입니다.
- 창 비활성화, 마우스 해제, GUI 응답 지연 시 조종을 멈춥니다.
- 자세: Landing, Stow, Stand, Stand11, Recover, Relax.
- 정책 선택, IMU 수평 보정, 직진 유지, 단일 trot/crab, 진단, 로그 저장 및 콘솔을 제공합니다.
- 표시되는 자세·안전·정책·전압은 제어기 응답입니다. 전압은 서보 전원선 추정값이며
  15초가 지난 값에는 `이전`을 표시합니다. 시뮬레이터 전압과 영상은 실기 측정이 아닙니다.

현재 화면은 Windows에 맞게 재배치했으며 iPhone의 CoreDevice 원격 앱 조작,
Bonjour 영상 자동 검색은 포함하지 않습니다. 원격 MuJoCo 영상은 지정한 호스트/영상 포트로 조회합니다.

## 실행 환경 / 새 컴퓨터 설정

Windows x64, Python 3.10 이상. MuJoCo는 이 저장소 전체와 별도의 Python 환경이 필요합니다.
독립 실행 파일에는 Qt/BLE 앱 런타임이 포함되며 큰 CAD 모델과 MuJoCo 환경은 포함하지 않습니다.
왼쪽 패널에서 프로젝트 및 Python 실행 파일 경로를 바꿀 수 있습니다.

저장소 공용 Conda `spot_omg` 환경을 사용합니다. 로컬 `.venv`는 만들지 않습니다.

```powershell
conda activate spot_omg
./apps/windows/setup.ps1
./apps/windows/run.ps1
```

Conda 활성화 없이 지정할 때:

```powershell
./apps/windows/setup.ps1 -Python C:/Users/etnlw/miniforge3/envs/spot_omg/python.exe
./apps/windows/run.ps1 -Python C:/Users/etnlw/miniforge3/envs/spot_omg/python.exe
```

`setup.ps1`은 GUI/BLE/영상 의존성을 설치하고, 저장소 `.toolchain/zig`에
Zig 0.15.2 x64를 SHA-256 검증 후 설치합니다. 시스템 PATH는 변경하지 않습니다.
공용 보행 C와 IMU PD 코드를 Windows DLL로 빌드하여 로드까지 확인합니다.
기존 GCC 호환 컴파일러는 `CC` 환경변수에 실행 파일 경로를 지정할 수 있습니다.
Windows 헤더/프로파일의 CRLF는 설정 해시 검증 시 LF로 정규화합니다.

MuJoCo를 앱 밖에서 직접 실행할 때:

```powershell
python -X utf8 simulation/mujoco/virtual_robot.py --viewer --no-ble --host 127.0.0.1 --video-host 127.0.0.1
```

앱이 시작한 MuJoCo 프로세스만 앱에서 종료합니다. 외부 서버 연결 해제는 그 서버를 종료하지 않습니다.
앱이 종료되거나 응답을 잃으면 로컬 MuJoCo는 소유자 heartbeat 유실을 확인해 종료합니다.
영상 프로세스도 부모 프로세스 종료를 감지합니다.

## 프로토콜 / 동작 제한

- 실제 BLE UUID: service `6e400001-b5a3-f393-e0a9-e50e24dcca9e`, RX `…0002…`, TX `…0003…`.
  가상 BLE는 iOS와 동일한 `…0101…` / `…0102…` / `…0103…`입니다.
- GATT 쓰기는 직렬 처리하며 Write-with-response를 우선합니다. 미지원 시 협상된
  `max_write_without_response_size`로 분할합니다. ACK는 STM32 동작 완료가 아닙니다.
- `drive LINEAR YAW SEQ` 시작 후 `@D SEQ LINEAR YAW`를 200ms마다 전송합니다.
  iOS의 15% dead zone, 30% 최소 motion 및 축별 보정을 사용합니다.
- 손을 놓으면 `@S SEQ`를 전송하고 heartbeat를 중단합니다. 종료 응답 뒤 `# `까지
  확인해야 새 동작을 시작합니다. 정지 확인 5초 초과 시 Ctrl+C 후 연결을 해제합니다.
  정지/오류 종료 응답을 받은 후에는 남은 로그 수신을 별도로 기다립니다.
  패킷마다 5초 무수신 제한을 갱신하되 전체 30초로 제한하며, `# ` 전에는
  새 동작·상태 조회·heartbeat를 보내지 않습니다.
- GUI pulse가 600ms 이상 끊기면 통신 스레드가 정지합니다. 제어기의 기존
  800ms watchdog은 그대로 유지합니다. 오래된 UI 명령은 전송하지 않습니다.
- Relax는 사용자 확인 → Landing 명령 완료 ACK → 새 `syncstate`의 Landing 확인 →
  Relax 순서입니다. 실패·중단·timeout·잘못된 readback 시 토크 해제를 취소합니다.
- Stow 상태에서는 Landing으로 먼저 펼칩니다. 중단된 Stow는 Stow 재시도가 가능합니다.
  정책과 고급 기능은 `caps`로 확인하며 `cushion_*` 정책은 시뮬레이터에서만 허용합니다.
- GUI 버튼 및 직접 콘솔 입력 모두 같은 제한을 통과합니다. 연속 drive와 realtime 패킷은
  콘솔에서 수동 전송할 수 없습니다. 완전한 임의 명령 검증기는 아니며 그 밖의 콘솔
  명령 범위 검증은 기존 STM32 콘솔에 맡깁니다.

## 빌드 / 검증

```powershell
python -m pip install 'pyinstaller>=6,<7'
./apps/windows/build.ps1
python -X utf8 -m pytest apps/windows/tests -q -p no:cacheprovider
python -X utf8 apps/windows/tests/qa_desktop.py
# 선택: 실제 GLFW 3D 뷰어 창도 함께 검증
python -X utf8 apps/windows/tests/qa_desktop.py --viewer
```

GUI 테스트는 실제 BLE 장치를 검색하거나 연결하지 않습니다. `qa_desktop.py`는 실제 MuJoCo를
18875/18876에서 실행하고, PD 정책 readback, 전진·후진, 정지, GUI 지연, 영상,
종료·재시작을 확인합니다. 결과는 Git에서 제외된 `test-output`에 저장합니다.

배포 실행 파일 점검:

```powershell
./apps/windows/dist/SpotOMGController/SpotOMGController.exe --smoke-test ./apps/windows/test-output/packaged.png --smoke-mujoco
```

검증 수준은 구분합니다: 호스트 테스트 / 실제 MuJoCo 통합은 수행할 수 있지만,
로봇 설정 readback·실제 위치 유지·전체 동작 시험은 실물 연결 후 별도 수행해야 합니다.
시뮬레이션 통과는 실제 STS3250의 다회전 원점이나 하중 조건을 검증하지 않습니다.

구현 참고: [Bleak Windows backend](https://bleak.readthedocs.io/en/latest/backends/windows.html),
[Bleak GATT client](https://bleak.readthedocs.io/en/latest/api/client.html),
[Qt for Python 배포](https://doc.qt.io/qtforpython-6/deployment/index.html).

빌드 시 외부 도구의 ICU DLL이 섞이지 않도록 PATH를 제한합니다. 배포 앱에서 외부
MuJoCo Python을 실행할 때는 [PyInstaller의 Windows DLL 검색 경로](https://pyinstaller.org/en/stable/common-issues-and-pitfalls.html#launching-external-programs-from-the-frozen-application)를 분리합니다.
앱 시작 오류 로그는 `%LOCALAPPDATA%/SpotOMGController/error.log`에 기록합니다.

보행 중 STOP 버튼/Space는 `@S`를 한 번 보내고 감속·최초 자세 복귀 완료를 기다립니다.
복귀 중 같은 버튼을 반복해도 복귀를 중단하지 않습니다. Esc는 긴급 중단 경로를 유지합니다.
키보드·조이스틱 해제도 같은 정상 정지 경로를 사용합니다.
MuJoCo 및 STM32 v59의 V6.1은 S로 복귀합니다. STM32 v58 이하의 기존 보행은
기존 Stand로 복귀합니다. 실기는 `s_native_v6_1` 지원을 알리는 펌웨어에서만
V6.1을 선택할 수 있으며, 지원되는 연결에서는 목록 맨 위 모델을 기본 선택합니다.
# V6.2 실험 모델 (2026-09-15)

V6.2 최초 추가 시의 기본 모델은 `s_native_v6_2`였다. V6.1을 보존하고 최대 뒤쪽 끝점을
65mm에서 125mm로 늘렸으며, 주기는 2초·후진 입력은 60% 제한이다.
[형상 해석과 검증 한계](../../docs/S-NATIVE-V62-2026-09-15.md)를 참고한다.
V6.2는 시뮬레이터 전용이며, 실제 로봇에서는 펌웨어가 지원하는 모델을 사용한다.


## V6.2.1 연속 회수

V6.2.1 추가 당시의 시뮬레이터 기본값은 `s_native_v6_2_1`이었다. V6.2는 그대로 선택 가능하다.
실제 로봇에서도 `s_native_v6_2_1` 지원을 알리는 STM32 v63이면 목록 맨 위에서
기본 선택한다. 해당 지원이 없는 기존 펌웨어는 V6.1을 사용한다.
실기 이식과 설치 검증 수준은 [v63 기록](../../docs/S-NATIVE-V621-FIRMWARE.md)을 참고한다.
교차 시 속도 저하를 줄였으며 전진 이동 중 R3를 접었다가 펴도록 회수 높이 곡선을 조정했다.
실제 접촉은 줄었으나 완전한 끌림 제거는 미완료다. [검증과 한계](../../docs/S-NATIVE-V621-2026-09-15.md)를 참고한다.

### V6.2.3 확장 동작 실험

지원 실기에서는 `s_native_v6_2_3`를 목록 맨 위와 기본값으로 선택한다. 앞 +20mm를 유지하고 뒤 끝점을 S 기준 −125mm에서 −135mm로 늘렸다. 발 들기/내리기를 매끄럽게 연결하고 이동 범위 확보를 위해 기본 주기는 1초로 설정한다. 기존 V6.2.2와 펌웨어 인덱스를 보존한다. 서보 최고 속도 설정은 유지하지만 실제 속도 유지·각도 도달은 미검증이다. 바닥 시뮬레이션 기울기 보호 중단이 남아 있다. [검증 기록](../../docs/S-NATIVE-V623-2026-09-15.md).

### V6.2.5 신속 회수

V6.2.5 추가 당시의 기본 모델은 `s_native_v6_2_5`였다. 발을 먼저 올린 뒤 수평 이동하며, 피드백 회복과 조회 간격을 개선했다. 기존 모델도 지원 여부에 따라 선택할 수 있다. [구현과 검증](../../docs/S-NATIVE-V625-2026-09-15.md).

### V6.2.6 연속 내딛기 (2026-09-16)

현재 기본 모델은 `s_native_v6_2_6`이다. 회수 전체 구간에서 발을 이동하고 추종 오차에 더 일찍 감속한다.
실기에서는 V72가 새 capability를 알릴 때 선택되며, V70 연결은 V6.2.5를 유지한다.
배터리 경고와 V71 전압 telemetry도 포함한다. 실기 설치·동작 검증은 아직 하지 않았다.
[검증과 다음 작업](../../docs/HANDOFF-2026-09-16-V626-CONTINUOUS-RECOVERY.md).

실행 중인 구버전을 유지하면서 새 앱을 별도 빌드하려면:

```powershell
./apps/windows/build.ps1 -Python python -DistPath "$PWD/apps/windows/dist/v626"
```

결과는 `apps/windows/dist/v626/SpotOMGController/SpotOMGController.exe`다.

## V0.1.1 충전 경고

11.0V 이하에서 큰 빨간 충전 경고, 10.5V 이하에서 즉시 사용 중단 경고를 표시한다.
소리와 작업 표시줄 알림을 요청하며, 확인을 눌러도 경고는 회복까지 유지된다.
Idle에서5초마다 전압을 읽고, V71의 전압 스트림에서는 보행 중에도 경고한다.
이전 펌웨어에서는 보행 종료 후 최소 전압도 반영한다.
[기준·검증·설치 범위](../../docs/APP-BATTERY-WARNING-2026-09-15.md).

## V77-T1 파라미터 보행 시험

새 **파라미터 보행 시험** 패널에서 들림12–40mm, 전진 입력1–1000, 시간500–30000ms, 대상all/RL/RR을 설정합니다.
V6.2.5 선택 → Stand → 설정 적용 + 조회 → 적용값 확인 → 시험 시작 순서입니다. Stop으로 중단합니다.
지원 펌웨어는 `s-native-v6-2-7-v77-t1-param` 및 `s-native-v6-2-7-v77-t1-param-j1`입니다.
일반 조이스틱에는 이 설정이 적용되지 않으며 시험 중 조이스틱은 시험값을 덮어쓰지 않습니다.
설정 조회 불일치·보호 오류·Stand가 아닌 상태에서는 시작하지 않습니다.
