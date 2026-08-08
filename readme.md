# Spot OMG

Spot Micro 기반의 12-DOF 4족 로봇 프로젝트입니다. 현재 STM32 실시간 제어,
URT-2/STS3215 서보 버스, BNO055 자세 피드백, MuJoCo 공용 보행 정책을 구현했으며
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

```bash
spotctl --port PORT status        # URT-2 직결: Feetech 패킷
spotctl console send trot2 1 1600 # ST-LINK: STM32 콘솔 명령
```

macOS에서는 두 장치 모두 `/dev/cu.usbmodem...`으로 잡히므로 포트를 USB vendor
ID로 구분합니다. ST-LINK는 `0483:374b`이고, 나머지 USB 시리얼 장치를 URT-2로
봅니다. `spotctl ports`가 어느 쪽인지 표시합니다.

## 📁 프로젝트 구조

```text
spot_omg/
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
- [x] BNO055 IMU와 J1/J2/J3 자세 보정
- [x] `stand`, `trot`, `trotplace`, 원형 발끝 `trot2`, 반복 `jump`
- [x] STM32/MuJoCo 공용 C 보행·점프 정책
- [x] 서보 보정·진단용 `spotctl`
- [x] 호스트에서 STM32 콘솔을 구동하는 `spotctl console`
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

---

## License

MIT
