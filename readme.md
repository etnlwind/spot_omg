# Spot OMG

최신 실기는 `attitudepd-v2-v78`입니다. [IMU 자세 안정화 V2 · 앞발 들림 +4mm 및 실기 8걸음 기록](docs/ATTITUDEPD-V2-FRONT-CLEARANCE-2026-09-18.md)을 먼저 확인하세요. 8걸음 정상 종료와 사용자 관찰 양호를 확인했으며, 반복·회전 및 실제 발높이 비교는 미검증입니다. 이전 파라미터 시험은 [V77-T1 기록](docs/V77-T1-REAR-PROBE.md)을 참고하세요.

Windows 조종 앱과 로컬 MuJoCo 실행은 [Windows Controller](apps/windows/README.md)를 참고하세요.

Spot Micro 기반의 12-DOF 4족 로봇 프로젝트입니다. 현재 STM32 실시간 제어,
URT-2/STS3215 서보 버스, BNO086 자세 피드백, MuJoCo 공용 보행 정책을 구현했으며
Jetson/ROS2와 RL 정책 연동은 다음 단계입니다.

## Mac·Windows 조종 앱 실행

### Mac (Mac Catalyst)

최신 소스로 빌드하려면 Xcode가 필요합니다. 저장소 루트에서 실행하세요.

```bash
cd /Users/etnlwind/project/spot_omg
xcodebuild -project apps/ios/SpotOMGController/SpotOMGController.xcodeproj \
  -scheme SpotOMGController -destination 'platform=macOS,variant=Mac Catalyst' \
  -derivedDataPath artifacts/mac-app CODE_SIGNING_ALLOWED=NO build
open artifacts/mac-app/Build/Products/Debug-maccatalyst/SpotOMGController.app
```

빌드한 뒤에는 마지막 `open` 명령만 실행하면 됩니다. 소스를 업데이트했다면 다시 빌드하세요.
기존에 `~/Applications`에 배치한 앱은 다음 명령으로 열 수 있습니다.
이 복사본은 Git pull만으로 갱신되지 않습니다.

```bash
open ~/Applications/SpotOMGController.app
```

[Mac 앱 상세 안내](apps/macos/README.md).

### Windows (PowerShell)

저장소 위치에 맞게 경로를 바꾸세요. `spot_omg` 환경이 없다면 먼저
`conda env create -f config/environment.yml`로 생성합니다.

```powershell
cd D:/project/spot_omg
conda activate spot_omg
./apps/windows/setup.ps1
./apps/windows/run.ps1
```

`setup.ps1`은 최초 설정 또는 의존성 갱신 때 실행합니다. 이후에는 환경 활성화 후
`./apps/windows/run.ps1`만 실행하면 됩니다.

V6.2.7 실행 파일 배포 폴더가 있는 컴퓨터에서는 다음으로 실행할 수 있습니다.
배포 파일은 Git에 포함되지 않으며 `_internal`이 있는 폴더 전체가 필요합니다.

```powershell
& ./apps/windows/dist/v627/SpotOMGController/SpotOMGController.exe
```

앱에서 **실제 로봇 · BLE**를 선택해 연결합니다. 다른 Mac·Windows·iPhone 앱의
로봇 연결은 먼저 해제하세요. Windows 앱의 **MuJoCo 시작 + 연결**은 로컬
시뮬레이터를 실행합니다. [Windows 앱 상세 안내](apps/windows/README.md).

## 펌웨어 업데이트 없이 보행 시험값 변경 (V77-T1-param)

설치된 `s-native-v6-2-7-v77-t1-param-j1`에서 **들림 높이·전진 입력·실행 시간·대상 다리**를
명령으로 변경할 수 있습니다. V6.2.5를 바탕으로 한 시험 기능이며, 일반 앱 조이스틱
보행에는 이 설정이 적용되지 않습니다. 몸체 기울기/J1/발 교대 위상 조절은 아직 제공하지 않습니다.

### 준비

로봇 전원을 켜고 다른 Mac·Windows·iPhone 앱의 로봇 연결을 해제합니다.
저장소 루트에서 프로젝트 환경을 활성화합니다. Windows에서는 `cd` 경로를
자신의 저장소 경로로 바꾸세요. 아래 한 줄 명령들은 Mac 터미널과 PowerShell에서 같습니다.

```text
cd /Users/etnlwind/project/spot_omg
conda activate spot_omg
spotctl --via ble --ble-name SpotOMG-Bridge console send syncstate
spotctl --via ble --ble-name SpotOMG-Bridge console send "gaitprofile s_native_v6_2_5"
```

실행 도구는 펌웨어 버전, V6.2.5 선택, 보호 오류, 서보 상태와 전압을 확인합니다.
보호 오류가 있으면 시험을 시작하지 않습니다. 마지막 전체 보행에서는 발이 들렸지만
오른쪽으로 기울어 중단됐으므로, 다음 시험도 몸체를 받아줄 준비가 필요합니다.

### 앱에서 시험하기 (Mac · iPhone · Windows)

업데이트된 앱의 **파라미터 보행 시험** 패널을 사용합니다. Mac/iPhone은 상세 설정 목록,
Windows는 상태 표시 아래에 있습니다.

1. 실제 로봇에 연결합니다. 다른 앱의 로봇 연결은 해제합니다.
2. **V6.2.5**를 선택하고, 필요하면 **Landing → Stand**로 전환합니다.
3. 들림 높이(mm), 전진 입력(1–1000), 실행 시간(ms), 대상 다리(all/RL/RR)를 정합니다.
4. **설정 적용 + 조회**를 누르고 **적용값**이 맞는지 확인합니다.
5. **시험 시작**을 누릅니다. **Stop / 시험 정지**로 도중에 멈출 수 있습니다.

설정만 적용하면 움직이지 않습니다. 시험 시작은 지원 펌웨어·Stand·V6.2.5·정상 보호 상태·
설정 조회 완료 조건에서 허용됩니다. 입력란을 수정한 뒤에는 다시 적용해야 합니다.
실행은 화면의 **적용값**을 사용합니다. 시험 중에는 조이스틱 입력을 무시하고 앱이 통신 유지
신호와 시간 제한을 관리합니다. 앱 비활성화/연결 종료 시 기존 정지 처리를 사용합니다.
보호 중단을 자동 복구하거나 재시도하지 않습니다.

이 패널은 기존 일반 조이스틱 보행과 별도이며, 시뮬레이터 연결에서는 지원하지 않습니다.
앱 콘솔에 실행 결과가 남습니다. 관절·IMU 원시 기록과 분석 파일까지 수집하려면 아래 Python 도구를 사용하세요.

### 실제 실행과 기록 수집

**다음 명령은 로봇을 실제로 움직입니다.** 네 다리 들림 목표 20mm, 전진 입력 344,
보행 4초 후 정지하는 예입니다. S 준비와 정지 복귀 시간은 별도입니다.

```text
python -m scripts.hardware.capture_v77_t1_params --execute-authorized --lift-mm 20 --linear 344 --duration-ms 4000 --legs all --output artifacts/v77-t1/param-walk20-01
```

왼쪽 뒷다리만 움직이고 나머지 세 다리는 S 목표로 고정하려면:

```text
python -m scripts.hardware.capture_v77_t1_params --execute-authorized --lift-mm 20 --linear 344 --duration-ms 4000 --legs rl --output artifacts/v77-t1/param-rl20-01
```

오른쪽 뒷다리는 `--legs rr`를 사용합니다. 다시 실행할 때는 `--output` 끝의
`01`을 `02`처럼 바꾸세요. 기존 결과 폴더를 덮어쓰지 않습니다.

| 옵션 | 뜻 | 허용 범위 / 값 |
| --- | --- | --- |
| `--lift-mm` | 발 회수 시 최대 들림 목표(mm), 실제 바닥 틈과는 다름 | 12–40 |
| `--linear` | 전진 입력. 344는 34.4% 입력이며 실제 속도 측정값이 아님 | 1–1000 |
| `--duration-ms` | 보행 루프 시작부터 정지 요청까지 시간(ms) | 500–30000 |
| `--legs` | 전체 보행 / 왼쪽 뒤 / 오른쪽 뒤 | `all` / `rl` / `rr` |

도구가 설정 전달 → 적용값 조회 → S 준비 → 시험 실행 → 정지 → 기록 수집을 수행합니다.
통신 유지 신호도 자동 전송합니다. 정상 종료 시 S로 복귀하며 토크는 켜져 있습니다.
기울기 등 보호로 중단되면 S 복귀가 완료되지 않을 수 있으므로 출력의 종료 사유를 확인하세요.
출력 폴더에는 `summary.json`, `events.jsonl`, 관절·IMU 기록과 분석 결과가 저장됩니다.
허용 범위 전체에서 안정적인 보행을 검증했다는 의미는 아닙니다.

### 움직이지 않고 설정만 변경·조회

```text
spotctl --via ble --ble-name SpotOMG-Bridge console send "probeconfig set 20 344 4000 all"
spotctl --via ble --ble-name SpotOMG-Bridge console send "probeconfig show"
spotctl --via ble --ble-name SpotOMG-Bridge console send "probeconfig reset"
```

`set` 뒤 순서는 **들림 mm / 전진 입력 / 시간 ms / 대상**입니다. 조회 예:

```text
$PROBECONFIG lift_mm=20 linear=344 duration_ms=4000 legs=all storage=ram
```

설정은 정지 상태에서만 변경하며, 잘못된 입력은 기존 설정을 유지합니다.
`reset`은 기본값 `20 344 4000 all`로 돌립니다. RAM 설정이므로 재부팅하면 초기화됩니다.
실행 도구는 매번 명령줄의 값으로 다시 설정합니다.

펌웨어 실행 명령은 `walkprobe`이지만, 단독 전송하면 800ms 통신 감시 때문에 중단됩니다.
실제 시험에는 위 Python 도구를 사용하세요. 설치 및 설정 변경·조회는 실기 검증했으며,
파라미터 보행도 실행했으나 20mm 전체 보행은 기울기 보호로 중단됐고, 사용자는 발 들림이 부족했다고 확인했습니다. 안정적인 보행은 아직 검증되지 않았습니다.
[상세 프로토콜과 검증 기록](docs/V77-T1-RUNTIME-PARAMETERS.md).

## 시뮬레이터 실행 (Mac)

### 기본 시뮬레이터

저장소 위치에 맞게 `cd` 경로를 바꾼 뒤 실행합니다.

```bash
cd /Users/etnlwind/project/spot_omg
conda activate spot_omg
mjpython simulation/mujoco/virtual_robot.py
```

### 큰 회수 비교 후보 재현·녹화

위와 같이 저장소 루트에서 `spot_omg` 환경을 활성화한 뒤 실행합니다.
영상·그림 도구가 없다면 먼저 `python -m pip install imageio-ffmpeg matplotlib`로 설치합니다.

```bash
python simulation/mujoco/scripts/analysis/capture_large_recovery_candidate.py \
  --config config/standard_height_final_comparison.json \
  --case x20-short-lead-j270 \
  --output artifacts/home-resume/standard-height-video
```

두 번째 명령은 기본 높이·정상 J1을 유지하는 **J2 70° 비교 후보**를 재현하고
4방향 영상을 저장합니다. 목표 J2 85°를 달성한 모델이나 보행 품질 검증을
통과한 모델은 아닙니다. 이전 결과를 보존하려면 재실행 시 `--output`을 새 경로로 지정하세요.
자세한 조건과 한계는 [기본 높이 유지 분석](docs/STANDARD-HEIGHT-BALANCE-2026-09-17.md)을 참고하세요.

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
├── apps/                    # iPhone/macOS 및 Windows 제어 앱
├── firmware/                # STM32, ESP32 펌웨어
├── hardware/                # URDF, 배선·전원 회로
├── simulation/mujoco/       # 가상 로봇 실행·물리 모델·시험·실험 도구
├── tools/                   # 서보 제어 및 공용 코드 생성 도구
├── config/                  # 공용 설정, environment.yml
├── docs/                    # 설계·사용법·참고 자료(references/)
├── artifacts/               # 실행 기록, 검증 결과, 이미지·영상
├── AGENTS.md                # 작업 시 유지해야 할 구현 기준
├── platformio.ini           # 루트에서 실행하는 ESP32 빌드 설정
├── pytest.ini              # 공용 시험 경로 및 캐시 설정
└── readme.md
```

시뮬레이터 내부 구성과 이동 기준은 [폴더 구조](docs/PROJECT-LAYOUT.md)를 참고하세요.
기존 가상 로봇 실행 경로 `simulation/mujoco/virtual_robot.py`는 유지됩니다.


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
conda env create -f config/environment.yml
conda activate spot_omg

spotctl --help
pytest tools/servo_tool/tests simulation/mujoco/tests/test_trot2.py -q
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
conda env update -f config/environment.yml --prune
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

## IMU 자세 안정화 · PD (실험)

앱에서 **IMU 자세 안정화 · PD · 실험 (검증실패)** 정책을 선택할 수 있습니다.
기존 명목 보행에 지지 발 높이 PD를 추가하며, `kp=0.1`, `kd=0.01s`, 50Hz,
최대 발 보정 5mm를 사용합니다. 앱의 수평 버튼은 해당 정책의 ON/OFF 설정을
가리키며, 실기 보행 중 실시간 전환·조회에는 전용 CLI 경로를 사용합니다.

```bash
spotctl --host 127.0.0.1 console send gaitprofile attitudepd
spotctl --host 127.0.0.1 stabilize status
spotctl --host 127.0.0.1 stabilize on
spotctl --host 127.0.0.1 stabilize off
```

MuJoCo 60초 전진에서 RMS 기울기와 관절 추종은 개선됐지만 roll 최대 진폭은
커졌고 목표 기준은 미달했습니다. **V58 펌웨어와 iOS 앱은 빌드만 했으며
미설치 상태입니다. 실물 gyro 축도 미검증이므로 새 PD가 활성화되지 않습니다.**
[구현·사용법·검증 결과·영상](./docs/BODY-STABILIZATION-PD-2026-09-14.md)에
OFF/ON 수치, 실패한 튜닝, 실물 검증과 구분한 현재 상태를 정리했습니다.

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


## 페어링된 아이폰 앱 원격 연결 제어

앱 V0.5.0 (34)부터 Mac에서 아이폰 앱의 연결 상태를 조회하고 연결/해제할 수 있습니다.
아이폰을 Mac에 개발 기기로 페어링하고 USB 또는 개발용 Wi-Fi로 접근할 수 있어야 합니다.
아이폰이 잠금 해제되어 앱을 실행할 수 있어야 하며, Tailscale만으로 이 기능이 제공되지는 않습니다.
휴대폰 에뮬레이터는 사용하지 않습니다.

```bash
spotctl app pair --device UJIN17
spotctl app status
spotctl app disconnect
spotctl app connect --target robot
```

`pair` 성공 후 기기를 `~/.config/spot_omg/app-control.json`에 저장합니다.
이후 `spotctl --via ble ...` 및 `spotctl firmware ...` 실행 시 앱의 로봇 연결을 먼저 해제합니다.
작업 성공 후에는 원래 연결되어 있었던 경우에만 앱을 재연결합니다. 작업 실패 시에는 재연결하지 않습니다.
기기별 일회성 지정은 전역 옵션 `--app-device UJIN17`, 자동 제어 생략은 `--no-app-control`입니다.
원격 요청 확인에 실패하면 자동 BLE 작업을 시작하지 않습니다. 이미 수동 해제했다면 `--no-app-control`로 실행할 수 있습니다.

연결 해제는 모터 토크 해제와 다릅니다. 움직이는 중이면 먼저 Stop 응답을 기다린 뒤 BLE를 해제하며,
기존 앱을 강제 종료하지 않습니다. 앱이 중지 확인을 받지 못하면 연결 해제 실패를 보고합니다.

**펌웨어 업데이트가 우선입니다.** 업데이트 전에 Landing을 시도하지만 Landing 실패·중지·응답 시간 초과가
발생해도 경고 후 업데이트를 계속합니다. 업데이트를 가로막는 자세 검증 조건으로 사용하지 않습니다.
앱의 일반 Relax 버튼은 Landing 완료 후 힘을 해제하며, Stop/오류 시 예약된 힘 해제를 취소합니다.

검증 및 제한 사항은 [원격 연결 제어 기록](docs/APP-REMOTE-CONTROL-2026-09-10.md)을 참고하십시오.


## V39 원호 턴 실물 시험

앱 V0.5.0 (42)의 **원호 턴 · 실물 시험 (검증실패)**는 `arcturn` 정책이다.
CAD/쿠션 원호 궤적을 STM32와 MuJoCo의 같은 C 코드로 계산한다.
Python 전용 80mm 전신 보정 정책 전체와는 구분한다.
실물 첫 시험은 회전 전에 Stand의 FL-J2 추종 보호 정지로 중단되었다.
실물 좌·우 턴 성공으로 검증한 상태가 아니다.
[구현·검증 및 실물 로그](docs/ARC-TURN-HARDWARE-2026-09-11.md)를 참고한다.

보호 정지 후 모터를 다시 움직이지 않고 펌웨어만 업데이트해야 한다면
`spotctl ... firmware stm32 <image.bin> --skip-landing`을 사용할 수 있다.
이 옵션도 실제 상태 조회에서 `torque=off`가 확인되어야 진행한다.
옵션을 생략한 기존 업데이트는 Landing을 먼저 시도한다.

### 실측 추종 보정 실험 (V42, 기본 OFF)

[구현·비교·미달 항목](docs/GAIT-TRACKING-SUPERVISOR-2026-09-11.md). 정지 상태의 `tracking on|off`와 `trackingdiag`를 실제 펌웨어 코드/시뮬레이터에 추가했다. 추종 지연에 따른 공통 진행 조절이며 접촉 검출이나 하중 제어 완성판이 아니다. 지연 조건에서 추종 오차는 감소했지만 일부 자세/접촉 품질이 악화되어 기본 OFF, 실물 미설치 상태다.

[원호 턴 하중 보상 후속 검증](docs/ARC-GRAVITY-PRELOAD-2026-09-11.md): CAD 중력 보상을 공통 C로 구현하고 좌우 60초 비교를 진행했다. 발 들림·회전 속도는 개선됐지만 기울기·접촉 기준을 만족하지 못해 실험 상태로 보존했다. 실제 로봇 및 기본 보행에는 적용하지 않았다.

[지지 전환의 추가 구현·검증 상태](docs/ARC-SUPPORT-TRANSITION-2026-09-12.md): 목표 위상과 CAD 보정 경로를 정리하고, 매 스텝의 발 높이·접촉으로 평가를 강화했다. 보존한 후보는 좌우60초 최대 roll 약1.7°/회전18.15°/s를 기록했으나 발 들림·접촉·추종 기준은 아직 미달이다. 비교 영상과 실패한 후속 시험도 함께 기록했으며 기본 보행·실물에는 승격하지 않았다.

### 원호 턴 · 지지 전환 보정 · 실험

앱 V0.5.0 (43)의 **원호 턴 · 지지 전환 보정 · 실험 (검증실패)**는 `arcsupport`다.
기존 원호 턴 정책은 보존한다. 지원 펌웨어가 `arcsupport` capability를 보고해야 실물에서 선택할 수 있다.
발 들림·접촉·추종 기준은 아직 미달이며, 설치 여부와 실물 동작 검증은 구분한다.
현재 적용 상태와 계산 시간 문제는 [앱·실물 배포 기록](docs/ARC-SUPPORT-DEPLOYMENT-2026-09-12.md)을 확인한다.

### 실제 관절 성능 기록 및 전진/턴 비교 (V47 진단 기능)

`jointtrace arm`으로 STM32 RAM 기록을 준비한 뒤 앱에서 보행하고, Stop 후 `python tools/download_joint_trace.py --output 기록.log`로 실제 전송 목표·시간별 서보 피드백을 내려받습니다. V47은 최대12개씩 나누어 수신 개수를 검증합니다. 기록 명령 자체는 로봇을 움직이지 않습니다. 약5초 기록이며 전원 차단 전에 내려받아야 합니다.

`spotctl analyze-joints 기록.log --compare 전진.log --output 결과폴더`로 관절별 목표/실제 이동량·추종 오차·전압을 비교합니다. 12축 순차 조회의 시간 해상도 한계를 표시하며 모터 토크나 지연을 임의로 확정하지 않습니다.

설치 전후 구분, 실행 명령, 시험 순서와 판정 기준: [실제 관절 동작 능력 측정](docs/JOINT-CAPABILITY-MEASUREMENT.md). V47 설치 및 전진·좌우 단기 실측은 완료됐으며, 정밀 지연 식별과 시뮬레이터 물성 보정은 아직 미완료입니다.

### 실측 비교 기반 보행 실험 (2026-09-12)

MuJoCo 전용 **실측 기반 · 선행 발 들기 · 실험 (검증실패)** (`measured_lift`)을 추가했다.
전진/좌우 회전 실측 명령 재생, 응답 모델 후보 비교, 60초 보행과 지연·하중·쿠션 민감도 시험 결과 및 실행 명령은
[실측 기반 보행 실험 보고서](docs/MEASURED-GAIT-EXPERIMENT-2026-09-12.md)를 참고한다.
기존 정책과 실물 펌웨어를 대체하지 않는다. 발 끌림은 줄었지만 수평 유지·반복 발 들림·무게 증가 조건에 미달했다.

회전 앞발을 더 드는 후속 실험 `measured_front_lift`도 같은 실험 정책 파일에 저장했다.
[앞발 높이 보강 결과와 실행 명령](docs/FRONT-TURN-LIFT-2026-09-12.md)을 참고한다.
앞발 높이는 증가했지만 몸체 흔들림·뒷발 끌림·낮게 들리는 스텝이 남아 `(검증실패)`로 표시한다.

후속 [제자리 회전 중심 이동 실험](docs/PIVOT-CENTER-EXPERIMENT-2026-09-12.md)은 고정 카메라와 몸통 중심 측정으로 평가했다.
정상 회전 중 이동은 줄었지만 시작 자세 이동과 앞발 들림 부족이 남아 기존 정책을 교체하지 않았다.

사용자가 선택한 영상 후보는 V48의 `centerpivot` 공용 C 정책으로 이식했다.
아이폰 빌드 44의 **몸체 중심 회전 보정 · 실험 (검증실패)**에서 선택한다.
[V48 이식·설치 검증 기록](docs/CENTER-PIVOT-DEPLOYMENT-2026-09-12.md)을 참고한다.

## 수직 기준 간격과 FR 출발 옵션

`v77-t1-width` 시험 펌웨어의 `walkprobe` 전용 설정입니다. 일반 보행 모델은 변경하지 않습니다.

- 한쪽 발 기준 **0mm = 수직 기준**, 음수 = 안쪽, 양수 = 바깥쪽. 범위 −40~+20mm, 새 기본값 0mm.
- 기존 정상 오므림은 수직 기준 약 −37.8mm/발입니다. 기존 첫걸음 비대칭과 보정 때문에 순간값은 다릅니다.
- **출발 시 FR만 추가 오므림** 체크박스: 기본 해제. 해제하면 좌우 공통 간격을 적용합니다.
- 체크하면 음수 간격에서 첫걸음 FR만 기구 J1 변화량을 2배 적용하고 다른 다리는 S의 J1을 유지합니다. 다음 걸음에 공통 간격으로 전환합니다. 0mm/양수에서는 추가 오므림을 적용하지 않습니다.
- 설정은 RAM에 저장합니다. 단일 뒷다리 시험은 선택된 다리만 움직이며 나머지는 S를 유지합니다. 실제 위치와 접지는 별도로 확인해야 합니다.

```text
probeconfig set 28 344 4000 all -20 0
probeconfig show
```

마지막 값은 FR 옵션(0=해제, 1=체크)입니다. Mac/iPhone/Windows의 시험 패널에서 간격과 체크박스를 설정하고 **설정 적용 + 조회** 후 적용값을 확인합니다. 새 펌웨어가 설치돼야 옵션을 사용할 수 있습니다.

### 시험 시간 로그

호스트 시험 도구 `scripts/hardware/capture_v77_t1_params.py`는 `--width-mm -20 --fr-extra`로 같은 설정을 전달합니다(`--fr-extra` 생략 시 해제).

- `events.jsonl`: 한국시간 ISO 8601 `timestamp`(+09:00), 시험 도구 시작 후 `elapsed_ms`, 송수신 명령 및 응답.
- `summary.json`: 시작/종료 `started_at`/`ended_at`, 총 `elapsed_ms`, 적용 파라미터와 FR 옵션, 정지 결과 및 오류.
- 센서 자체 시간은 원본 trace에 보존합니다. 호스트 수신 시간은 측정 순간과 같지 않으므로 영상과 자동 동기화됐다고 간주하지 않습니다.
- 빌드·호스트 테스트 기록: `artifacts/v77-t1/width-validation/tests.jsonl`. 실기 동작과 구분해 기록합니다.

## 앱 저장과 보행 모드

- **기본 보행**: 초기 선택. 기존 조이스틱 보행을 사용하며 파라미터 메뉴는 비활성화됩니다.
- **파라미터 보행**: 저장/불러오기/로봇 적용 메뉴 활성화. V6.2.5·Stand·설정 적용 및 조회 완료 후 조이스틱을 앞으로 밀면 적용값으로 시작합니다. 전체 다리(all)를 선택해야 합니다.
- **파라미터 저장**은 해당 앱의 로컬 저장소에 저장합니다. 앱을 다시 열면 불러옵니다. 기기 간 자동 동기화나 로봇 영구 저장은 아닙니다.
- 편집/불러오기 후 **설정 적용 + 조회**를 눌러야 로봇에 반영됩니다. 적용값 표시는 로봇 readback입니다.
- 파라미터 보행은 전진 전용이며 전진 입력은 설정값을 사용합니다. 조이스틱 해제/중립 또는 설정 시간 도달 시 정지합니다. 자동 반복하지 않으며 다시 조작해야 시작합니다.
- 기본 모드로 전환하려면 먼저 정지합니다. 새 연결은 기본 모드로 시작합니다. 현재 `v77-t1-width` 펌웨어에서 사용할 수 있어 펌웨어 재설치는 필요 없습니다.

### 조이스틱 자동 복귀

조이스틱 왼쪽 상단의 **자동 복귀 ON/OFF** 버튼을 사용합니다. 기본 ON이며 OFF는 손을 떼어도 입력을 유지합니다. ON으로 전환하거나 Stop/중앙 입력으로 정지할 수 있습니다. 앱 비활성화·연결 해제 시 입력을 해제합니다. 파라미터 보행의 설정 시간 제한은 그대로 적용되며 자동 반복하지 않습니다.
