# Spot OMG

Spot Micro 기반의 12-DOF 4족 로봇 프로젝트입니다. 현재 STM32 실시간 제어,
URT-2/STS3215 서보 버스, BNO086 자세 피드백, MuJoCo 공용 보행 정책을 구현했으며
Jetson/ROS2와 RL 정책 연동은 다음 단계입니다.

## 🎯 목표

- STM32 펌웨어 개발
- Jetson Orin Nano 제어
- ROS2 연동
- Isaac Lab 강화학습 정책 적용
- Spot Micro 하드웨어 제작

---

## 전체 구성

```text
Jetson Orin Nano (ROS2/RL/비전)
  ↓ USB serial 또는 UART command/telemetry
STM32F446RE (실시간 관절·IMU 제어)
  ↓ USART1, 1 Mbps, 8-N-1
URT-2 (UART ↔ half-duplex TTL bus)
  ↓
STS3215 ×12
```

Mac에서 ST-LINK USB로 접속한 포트는 STM32의 115200 bps 텍스트 콘솔이고,
URT-2를 Mac/Jetson에 USB로 직접 연결한 포트는 1 Mbps Feetech 서보 버스입니다.
두 경로는 프로토콜이 달라 서로 호환되지 않습니다.

`spotctl`은 붙어 있는 USB 장치를 보고 어느 링크를 쓸지 스스로 정하므로 같은
명령을 그대로 쓰면 됩니다.

```bash
spotctl stand         # ST-LINK면 STM32 콘솔, URT-2면 Feetech 버스
spotctl landing       # 두 경로 모두 보정된 착지 자세
spotctl trot2 1 1600  # STM32 전용
spotctl walk          # URT-2 직결 전용
```

현재 기체는 ESP32-WROOM BLE GATT bridge를 사용합니다. 과거 ESP32-C3용으로 만든
디렉터리 이름이 일부 남아 있지만 현재 PlatformIO build target은 `esp32dev`입니다.
TCP transport는 별도 호환 bridge용으로 도구에 남아 있으며 현재 WROOM firmware는
BLE를 기본 경로로 사용합니다.

```bash
# 기존 USB/Serial
spotctl stand

# ESP32-C3 Wi-Fi bridge
spotctl --host 192.168.0.112 stand

# Interactive STM32 console over Wi-Fi
spotctl --host 192.168.0.112 console

# Explicit TCP port
spotctl --host 192.168.0.112 --tcp-port 3333 trot3 1 1400

# ESP32-WROOM-32D BLE GATT bridge (macOS and Windows)
spotctl --via ble targets
```

어느 포트로 나갔는지는 실행할 때마다 첫 줄에 표시됩니다.

```text
ports:[/dev/cu.usbmodem312103] link=stm32
```

macOS에서는 두 장치 모두 `/dev/cu.usbmodem...`으로 잡히므로 포트를 USB vendor
ID로 구분합니다. ST-LINK는 `0483:374b`이고, 나머지 USB 시리얼 장치를 URT-2로
봅니다. `spotctl ports`가 어느 쪽인지 표시하며, 둘 다 꽂혀 있으면 명령 지원 범위에
따라 STM32 전용은 STM32, URT-2 전용은 URT-2, 양쪽 지원은 STM32를 자동 선택하고
필요할 때만 `--via stm32` 또는 `--via urt2`로 경로를 강제할 수 있습니다.

## 📁 프로젝트 구조

```text
spot_omg/
├── apps/ios/SpotOMGController/ # SwiftUI/CoreBluetooth iPhone controller
├── firmware/stm32-learning/  # STM32F446RE 로봇 제어 펌웨어
├── hardware/urdf/            # 12-DOF URDF와 실측 파라미터
├── simulation/mujoco/        # 자세·보행·점프 시뮬레이션
├── tools/servo_tool/         # URT-2 USB 직접 제어 및 보정 도구
├── environment.yml           # 공용 Conda 환경
└── readme.md
```

---

## 현재 구현 상태

- [x] STM32 USART1 1 Mbps URT-2 통신과 STS3215 12축 Sync Write
- [x] USART2 인터럽트 콘솔과 실행 중 `Ctrl+C` 정지
- [x] BNO086 IMU(SPI)와 J1/J2/J3 자세 보정
- [x] `stand`, `trot`, `trotplace`, 원형 발끝 `trot2`, 반복 `jump`
- [x] 혼합 서보 한계를 반영한 `trot3`/`trot4`와 3점 지지 `crab` crawl
- [x] STM32/MuJoCo 공용 C 보행·점프 정책
- [x] 서보 보정·진단용 `spotctl`
- [x] 연결된 장치에 따라 STM32/URT-2로 자동 분기하는 `spotctl`
- [x] ESP32 self OTA와 ESP32 staging 기반 STM32 BLE OTA
- [x] STM32 PB5→ESP32 EN을 이용한 공유 전원 cold-boot reset
- [x] iPhone BLE 상태 동기화, 고정 콘솔, 앱 아이콘과 연속 조이스틱 UI
- [x] sequence/heartbeat 기반 STM32 50Hz 연속 전진·후진·회전 제어
- [x] STM32 Flash sector 7 persistent flight log와 host/iPhone 시간 동기화
- [ ] Jetson 명령/telemetry 프로토콜
- [ ] ROS2 hardware interface
- [ ] Isaac Lab RL 정책 배포

## 🤖 Simulation Model

Isaac Sim/Isaac Lab에서 사용할 12-DOF URDF 초안과 실측 파라미터는
[`hardware/urdf`](./hardware/urdf/README.md)에서 관리합니다.

## Python 개발 환경

Servo tool, 단위 시험과 MuJoCo 시뮬레이션은 `spot_omg` Conda 환경을 사용합니다.

```bash
conda env create -f environment.yml
conda activate spot_omg

spotctl --help
pytest tools/servo_tool/tests simulation/mujoco/test_trot2.py -q
python simulation/mujoco/walk.py --dynamic --balance \
  --gait trot --preset sim-trot --cycles 10 --check
```

macOS에서 MuJoCo GUI viewer를 열 때는 일반 `python` 대신 환경에 설치된
`mjpython`을 사용합니다.

```bash
mjpython simulation/mujoco/walk.py --dynamic \
  --balance --gait trot --preset sim-trot --cycles 3
```

환경 정의를 변경한 경우 기존 환경에 패키지를 계속 덧붙이기보다 다음 명령으로
정의 파일과 동기화합니다.

```bash
conda env update -f environment.yml --prune
```

로컬 `.venv` 또는 `.venv-mujoco`는 사용하지 않습니다. 자세한 실행법은
[`tools/servo_tool`](./tools/servo_tool/README.md),
[`simulation/mujoco`](./simulation/mujoco/README.md),
[`firmware/stm32-learning`](./firmware/stm32-learning/README.md) 문서를 참고하세요.
ESP32–STM32 연결, 부팅, flash partition과 두 BLE OTA 경로의 전체 구조는
[`firmware/FIRMWARE_ARCHITECTURE.md`](./firmware/FIRMWARE_ARCHITECTURE.md)에
정리되어 있습니다.
2026-09-06 실기 진단, trot4/crab 변경 근거와 남은 과제는
[`tools/servo_tool/HARDWARE_TEST_LOG.md`](./tools/servo_tool/HARDWARE_TEST_LOG.md)의
해당 날짜 기록을 참고하세요.

## 서보 제어 변경 전 참고

**관절각·방향·Stow/Landing을 수정하기 전에 [STS3250 위치 제어 실기 기준](./docs/STS3250-POSITION-CONTROL.md)을 먼저 확인합니다.**
현재 기체에서 확인한 누적 목표와 단회전 피드백의 차이, 실제 실패 사례, 검증 순서를 정리했습니다.

## STM32 펌웨어 직접 컴파일 및 BLE 전송 (macOS)

현재 Mac의 `spot_omg` Python 환경과 STM32CubeIDE에 포함된 ARM 컴파일러를
사용하는 절차입니다. 아래 명령은 **STM32 로봇 제어 펌웨어**를 업데이트합니다.
ESP32 브리지 펌웨어 업데이트와는 별도입니다.

### 준비

- STM32CubeIDE가 `/Applications/STM32CubeIDE.app`에 설치되어 있어야 합니다.
- 아래 Python/spotctl 경로는 현재 Mac의 `/opt/anaconda3/envs/spot_omg` 기준입니다.
- 로봇을 정지시키고 안정적으로 받친 상태에서 진행합니다. 업데이트 중 전원을 끄지 않습니다.
- iPhone/Mac 앱의 로봇 BLE 연결을 해제합니다. 다른 `spotctl` 실행이나 펌웨어 전송과 동시에 연결하지 않습니다.
- 기존 STM32 OTA 부트로더와 ESP32 BLE 브리지가 설치된 기체를 대상으로 합니다.
  최초 설치는 [펌웨어 구조 문서](./firmware/FIRMWARE_ARCHITECTURE.md)를 참고하세요.

### 1. 컴파일

```bash
cd /Users/etnlwind/project/spot_omg

/opt/anaconda3/envs/spot_omg/bin/python \
  firmware/stm32-learning/build_firmware.py \
  --output /private/tmp/spot-firmware-build
```

성공하면 `binary`, `size`, `sha256` 등이 출력되고, 출력 디렉터리에
`manifest.json`과 `build.log`가 생성됩니다. **컴파일이 실패하면 전송하지 말고
`/private/tmp/spot-firmware-build/build.log`를 확인합니다.** 이전 빌드 파일이
남아 있을 수 있으므로 `.bin` 파일이 존재한다는 사실만으로 성공을 판단하지 않습니다.

파일명은 `firmware/stm32-learning/Inc/robot.h`의 `ROBOT_CONTROL_REV`에서 정합니다.
예를 들어 V29는 `shared-locomotion-v29.bin`입니다. 다음 단계에서는 버전명을
직접 입력하지 않고 이번 빌드의 `manifest.json`에서 정확한 경로를 읽습니다.

### 2. BLE로 전송

1단계가 성공한 뒤 실행합니다.

```bash
SPOT_FIRMWARE_IMAGE="$(/opt/anaconda3/envs/spot_omg/bin/python -c 'import json; print(json.load(open("/private/tmp/spot-firmware-build/manifest.json"))["binary"])')"

/opt/anaconda3/envs/spot_omg/bin/python -u \
  /opt/anaconda3/envs/spot_omg/bin/spotctl \
  --via ble --ble-name SpotOMG-Bridge \
  firmware stm32 "$SPOT_FIRMWARE_IMAGE"
```

`-u`는 진행률을 즉시 표시합니다. ESP32에 이미지를 전송한 뒤 STM32 플래시를
기록하며, 수 분이 걸릴 수 있습니다. 마지막에 다음 문구가 나와야 완료입니다.

```text
STM32 firmware verified and rebooted
```

### 3. 설치 버전 확인

```bash
/opt/anaconda3/envs/spot_omg/bin/spotctl \
  --via ble --ble-name SpotOMG-Bridge \
  console send syncstate
```

응답의 `rev=`가 빌드한 버전인지 확인합니다. 이 명령은 상태 조회이며 움직임을
명령하지 않습니다. `BLE device ... not found`가 나오면 로봇 전원, Mac Bluetooth,
앱이나 다른 프로그램의 BLE 연결 점유 여부를 확인합니다.

앱에서 확인하려면 조이스틱 아래로 스크롤해 제어 대상·연결 설정의 **로봇 펌웨어**
항목을 확인하고, 연결 후 **현재 상태 동기화**를 누릅니다.
`shared-locomotion-v34` 같은 값이 STM32 펌웨어 버전입니다.
상단 `Spot OMG V0.x.x (빌드번호)`는 iPhone 앱 버전이며 서로 별개입니다.

### 현재 Stow / Landing 검증 상태 (2026-09-10)

최종 V34는 누적 목표와 단회전 피드백의 좌표 처리 오류를 수정하고,
Landing 완료 후 토크를 유지합니다. 느린 일반 자세 전환의 완료 허용 오차는
기존120틱 기준으로 복원했습니다. 펌웨어 회귀 테스트28개 통과 및 실제 로봇 설치를 완료했습니다.

실제 기체에서 **`OK stow` → `OK landing` 왕복1회 성공**을 확인했습니다.
최종 조회는 `pose=landing error=10 torque=on safety=ok rev=shared-locomotion-v34`이며,
최대 자세 오차는 약0.9도였습니다. 모든 하중/전원 재인가 조건의 실기 검증을 의미하지는 않습니다.

읽기 전용 진단과 저장 로그 조회:

```bash
/opt/anaconda3/envs/spot_omg/bin/spotctl --via ble --ble-name SpotOMG-Bridge console send stowdiag
/opt/anaconda3/envs/spot_omg/bin/spotctl --via ble --ble-name SpotOMG-Bridge console send 'log show 40'
```

자세한 증거와 변경 내용은 [Stow/Landing 진단 기록](./docs/STOW-LANDING-DIAG-2026-09-10.md)에 정리합니다.

## 이번 작업 내역

다른 컴퓨터의 VSCode/Codex에서 작업을 이어갈 때는
[2026-09-07 인수인계 문서](./docs/HANDOFF-2026-09-07.md)를 먼저 읽으세요.
최종 iOS 연결 수정, 테스트·기기 설치 결과, 실기 미검증 항목과 로그 조회 절차를
한곳에 정리했습니다.

이번 변경은 단발성 보행 명령을 iPhone에서 반복 호출하던 구조를 실제 원격 조종에
적합한 연속 제어 구조로 바꾸고, 연결되지 않은 동안의 로봇 사건도 나중에 확인할 수
있도록 STM32 persistent log를 추가한 작업입니다.

### 보행·자세 보정

- `stand11`을 기구 대칭 기준으로 유지하고, trot4에서만 안쪽으로 모이던 FR 다리는
  ID4 J1에 바깥 방향 2° bias를 적용했습니다. 보정 중심값이나 다른 자세에는 영향을
  주지 않습니다.
- J1/J3 STS3215와 J2 STS3250의 서로 다른 속도·가속도·추종 지연 한계를 기존 mixed
  actuator limiter에 유지했습니다.
- `trot4back`과 differential `turn left|right` 궤적을 추가하고, 전진·후진·회전을 한
  support phase에서 혼합할 수 있는 공용 drive target을 STM32와 host simulation에
  추가했습니다.
- crab은 빠른 횡이동이 아니라 안정성을 우선하는 four-beat crawl 시험 명령으로
  유지했습니다. 실제 좌우 조향은 crab이 아니라 differential turn을 사용합니다.
- IMU roll/pitch 보정, tilt/stall safety, step-sync 및 gait diagnostics를 연속 drive에도
  그대로 적용합니다.

### 연속 원격 조종

기존 `trot4 CYCLES PERIOD_MS`는 재현 가능한 실기 시험용으로 남겨 두었습니다. 실제
조이스틱은 아래 별도 lane을 사용하므로 cycle 종료, 긴 diagnostics와 console prompt를
기다리지 않습니다.

```text
drive LINEAR YAW SEQ    연속 gait 시작; LINEAR/YAW는 -1000..1000
@D SEQ LINEAR YAW      실행 중 목표와 heartbeat 갱신
@S SEQ                 입력 해제 및 감속 정지
```

- STM32가 20ms 주기의 gait phase, servo 명령, IMU와 안전 판정을 소유합니다.
- iPhone은 스틱 벡터만 갱신하며, 드래그 중 200ms마다 heartbeat를 보냅니다.
- sequence가 오래된 BLE packet은 버리고 마지막 유효 packet이 800ms 동안 없으면 STM32가
  앱이나 BLE 상태와 무관하게 자동 정지합니다.
- 입력은 frame마다 slew 제한되고 크기에 따라 보행 주기가 2400..1800ms로 변합니다.
- 전진·후진과 좌우 회전은 동시에 혼합되지만 전체 motion budget은 100%로 정규화합니다.
- 앱에서 손을 떼면 조이스틱이 중앙으로 복귀하며 `@S`를 보내고, background 전환에는
  Ctrl+C와 Stand를 추가로 요청합니다.

### STM32 persistent flight log

- STM32F446 sector 7의 첫 1KiB는 기존 IMU calibration에 남겨 두고 나머지 127KiB에
  128-byte append-only 레코드를 최대 1,016개 저장합니다.
- boot, console command/result, gait 전압·lag·tilt 요약과 peak error 상위 관절을 동작
  경계에서만 기록합니다. 50Hz motion frame 안에서는 Flash를 쓰지 않습니다.
- 각 레코드는 sequence, boot ID, uptime, Unix epoch, event text와 checksum을 포함합니다.
  전원 차단으로 마지막 레코드가 불완전해도 유효 checksum까지만 복구합니다.
- 로그가 차거나 삭제될 때 IMU calibration prefix를 보존하며 sector를 회전합니다.
- Mac의 `spotctl`과 iPhone은 연결 직후 `log time <UNIX_EPOCH_MS>`를 best-effort로 보내
  STM32 uptime을 절대 시각과 연결합니다.
- `spotctl logs [--count N] [--output FILE]`, `--status`, `--clear`로 BLE 또는 USB에서
  조회·저장·삭제할 수 있습니다. ESP32는 로그를 따로 보관하지 않고 요청 시 UART/BLE로
  중계합니다.

### iPhone 앱

- CoreBluetooth로 `SpotOMG-Bridge`를 검색·재연결하고 `syncstate` 응답으로 자세, torque,
  safety와 balance 상태를 UI에 반영합니다.
- 화면 위쪽에는 검은 배경·녹색 고정폭 글꼴의 console을 고정하고, 명령 입력과 키보드
  `완료` 버튼을 제공합니다. 아래 조작 영역은 `연결 → 조이스틱 → 안전 자세 → 단일 보행
  → 진단` 순서입니다.
- 조이스틱 변위에 비례해 저속부터 최대 속도까지 연속 제어하며 전진·후진과 좌·우 회전을
  혼합합니다. 터치 추적 중에도 heartbeat가 멈추지 않도록 main run loop common mode를
  사용합니다.
- Landing, Stand, Stand11, Hold, Recover, 확인 절차가 있는 Relax, trot4/crab 단일 시험,
  targets/scan/gaitdiag/baldiag, 저장 로그 조회와 시간 재동기화 버튼을 제공합니다.
- 제공된 `spot_omg_remote.png`를 iOS AppIcon asset으로 등록했습니다.

### 문서와 검증 상태

- 세부 제어 흐름과 Flash/OTA 배치는
  [`firmware/FIRMWARE_ARCHITECTURE.md`](./firmware/FIRMWARE_ARCHITECTURE.md), STM32 명령은
  [`firmware/stm32-learning/README.md`](./firmware/stm32-learning/README.md), 앱 프로토콜은
  [`apps/ios/SpotOMGController/README.md`](./apps/ios/SpotOMGController/README.md)에 각각
  기록했습니다.
- 프로젝트 Conda 환경에서 Python 시험 175개와 C 정책 subtest 25개가 모두
  통과했습니다.
- iOS simulator 빌드와 연결 회귀 시험 21개가 통과했고, 최종 수정 앱의 iPhone용
  서명 빌드 및 설치도 성공했습니다. 이전 CoreSimulator 접근 오류는 샌드박스 밖에서
  정상 접근해 해결했습니다.
- STM32 `continuous-drive-v10`은 BLE OTA 설치·검증·재부팅과 revision 조회까지
  완료했습니다. 최종 수정 iOS 앱의 실제 지속 보행은 아직 미확인이며, 다음 세션에서
  폰 연결 기록과 STM32 로그를 확보해 확인해야 합니다.

---

## License

MIT
