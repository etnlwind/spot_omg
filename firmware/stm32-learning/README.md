# Spot OMG STM32 Servo Firmware

STM32F446RE가 URT-2 UART 헤더를 통해 STS3215 12개를 제어하는 시험
펌웨어입니다. 부팅만으로 모터가 움직이지 않으며, USART2 콘솔 명령을 명시적으로
입력해야 토크가 활성화됩니다.

```text
Mac/Jetson → ST-LINK VCP/USART2 → STM32 → USART1 → URT-2 → STS3215 ×12
```

ST-LINK VCP는 텍스트 명령 콘솔이며 Feetech raw serial bridge가 아닙니다.
따라서 STM32의 `/dev/cu.usbmodem...` 포트에 `spotctl`을 실행하지 않고 일반
115200 bps 터미널로 접속해 아래 콘솔 명령을 사용합니다.

호스트 측 Servo Tool, 공용 보행 정책 회귀 시험과 MuJoCo 비교는 저장소 루트의
Conda `spot_omg` 환경에서 실행합니다.

```bash
conda activate spot_omg
pytest tools/servo_tool/tests simulation/mujoco/test_trot2.py -q
python simulation/mujoco/walk.py --dynamic --balance \
  --gait trot --preset sim-trot --cycles 10 --check
```

## 인터페이스

| Peripheral | Pins | Purpose | Configuration |
|---|---|---|---|
| USART1 | PA9 TX, PA10 RX | URT-2 UART header | 1 Mbps, 8-N-1 |
| USART2 | PA2 TX, PA3 RX | ST-LINK VCP debug console | 115200, 8-N-1 |
| SPI1 | PA5 SCK, PA6 MISO, PA7 MOSI | BNO086 | 1 MHz, mode 3 |
| GPIO | PB6 CS, PA8 INT, PB2 RST, PB5 WAKE | BNO086 제어선 | 모두 active low |
| I2C1 | PB8 SCL, PB9 SDA | BNO055 | 100 kHz |

NUCLEO-F446RE Arduino 헤더 기준 `D8/PA9 → URT-2 RX`, `D2/PA10 → URT-2 TX`로
**교차** 연결합니다. URT-2 UART 헤더의 표기는 URT-2 자신의 신호 기준이라
`TX`가 출력, `RX`가 입력이므로 일반적인 UART 교차 결선입니다. 2026-08-08에
`TX-TX`, `RX-RX`로 잘못 적혀 있던 것을 실기에서 바로잡았습니다.
STM32, URT-2, 외부 서보 전원은 GND를 반드시 공통으로 연결하고, 서보 전원은
STM32나 USB에서 공급하지 않습니다. URT-2를 Type-C로 공급할 때 UART 헤더 VCC는
연결하지 않으며 signal-level switch는 3.3V로 둡니다.

URT-2 Type-C USB와 STM32 UART 헤더가 같은 서보 버스를 동시에 구동하지 않도록
합니다.

## IMU 선택

펌웨어는 부팅 시 붙어 있는 센서를 자동으로 고릅니다. 리컴파일이 필요 없습니다.

```text
BNO055 탐색 (I2C1, 빠름)  →  있으면 사용
      ↓ 없으면
BNO086 탐색 (SPI1, 느림)  →  있으면 사용
      ↓ 없으면
IMU 없음 — trot/trot2/jump 잠금, balance off로만 개방 루프 시험
```

BNO055를 먼저 보는 것은 탐색 비용 때문입니다. 없는 BNO086은 SH-2 타임아웃으로
수 초가 걸립니다. 두 센서는 서로 다른 페리페럴에 있어 동시에 연결해도 됩니다.

BNO055 배선은 `VIN→3V3`, `GND→GND`, `SCL→D15/PB8`, `SDA→D14/PB9`이며
`COM3`를 GND에 묶으면 주소가 `0x28`로 고정됩니다. 펌웨어는 `0x28`과 `0x29`를
모두 탐색하고 칩 ID `0xA0`으로 확인하므로 `COM3` 연결은 필수가 아닙니다.
현재 기체는 `COM3`를 연결하지 않아 `0x29`로 잡힙니다.

### 부호 규약

```text
Pitch +  : 로봇 앞쪽으로 기울어짐
Pitch -  : 로봇 뒤쪽으로 기울어짐
Roll  +  : 로봇 오른쪽으로 기울어짐
Roll  -  : 로봇 왼쪽으로 기울어짐
```

2026-08-08 실기에서 확인한 결과 Roll은 이 규약과 일치했고 Pitch는 반대였습니다.
`bno055.c`의 `BNO055_PITCH_SIGN`을 `-1`로 두어 맞췄고, 앞으로 숙였을 때 Pitch가
양수로 커지는 것을 재확인했습니다. 부호는 이 상수에서 맞추며 균형 게인으로
보정하지 않습니다 — 게인에서 뒤집으면 불일치가 드러나지 않고 숨습니다.

`imu on`의 출력은 `spotctl console watch`로 봅니다.

## BNO086 배선과 장착 좌표계

2026-08-08에 IMU를 BNO055(I2C)에서 BNO086(SPI)으로 교체했습니다. NUCLEO-F446RE
Arduino 헤더 기준 배선은 다음과 같습니다.

```text
BNO086 3V3  → NUCLEO 3V3
BNO086 GND  → NUCLEO GND
BNO086 SCK  → D13 / PA5  / SPI1_SCK
BNO086 SO   → D12 / PA6  / SPI1_MISO
BNO086 SI   → D11 / PA7  / SPI1_MOSI
BNO086 CS   → D10 / PB6  / GPIO
BNO086 INT  → D7  / PA8  / GPIO
BNO086 RST  → PB2 / GPIO
BNO086 WAK  → 연결하지 않음 (PB5는 High 출력으로만 예약)
```

**`D2`는 비워 둡니다.** `D2`는 `PA10`이고 이 핀은 URT-2에서 돌아오는
`USART1_RX`입니다. 여기에 `INT`를 물리면 서보 버스가 죽고, 증상은 2026-08-08
기록의 `BUSPROBE RX: no bytes`와 똑같이 나타납니다.

주의할 핀이 둘 더 있습니다.

- `D13/PA5`는 NUCLEO 사용자 LED `LD2`가 달려 있는 핀입니다. 이제 SPI1_SCK이므로
  펌웨어에서 이 핀을 구동하던 코드(B1 버튼 → LD2 토글)를 제거했습니다. LED와
  직렬저항이 SCK에 부하로 걸리므로 최초 bring-up은 1MHz로 수행합니다.
- Reset은 CubeMX label `IMU_RST`인 `PB2`입니다. 과거 문서의 `D3/PB3` 표기는
  현재 배선과 맞지 않으므로 사용하지 않습니다.

현재 보드는 후면 `PS0`, `PS1` 점퍼를 모두 납땜해 두 핀이 약 3.2V HIGH이며 SPI
모드로 부팅합니다. 이 구성에서는 `WAK`를 호스트가 구동하지 않으므로 BNO086의
WAK 핀은 연결하지 않습니다. CubeMX의 `PB5/IMU_WAKE` High 출력은 예약 상태일
뿐 실제 보드와 배선되지 않습니다.

### Game Rotation Vector를 쓰는 이유

펌웨어는 Rotation Vector(`0x05`)가 아니라 **Game Rotation Vector(`0x08`)**를
구독합니다. 전자는 지자기를 융합에 포함하는데, STS3215 12개의 영구자석이 IMU
몇 cm 옆에 있어 자기 방위는 없는 것만 못합니다. 균형 제어는 roll/pitch만
사용하고 이 둘은 자이로/가속도 융합만으로 나옵니다.

대신 이 리포트의 Yaw는 **북쪽이 아니라 전원 투입 시점 기준 상대값**입니다.
BNO055의 `0..359.9°` 절대 방위와 다르므로, Yaw를 쓰는 코드를 추가할 때는 이
차이를 전제해야 합니다.

### 부호 규약

로봇 좌표계 목표는 BNO055 때와 동일합니다.

```text
Pitch +  : 로봇 앞쪽으로 기울어짐
Pitch -  : 로봇 뒤쪽으로 기울어짐
Roll +   : 로봇 오른쪽으로 기울어짐
Roll -   : 로봇 왼쪽으로 기울어짐
```

`bno086.c`의 `BNO086_ROLL_SIGN`, `BNO086_PITCH_SIGN`이 이 규약을 맞추는
자리입니다. **아직 실기 검증 전이므로 `imu on`으로 먼저 확인하세요.** 실제로
앞으로 기울였을 때 Pitch가 음수로 나오면 해당 상수만 `-1`로 바꿉니다. 균형
게인으로 보정하지 않습니다.

## CubeMX 필수 설정

| Setting | USART1: URT-2 | USART2: console |
|---|---|---|
| Mode | Asynchronous TX/RX | Asynchronous TX/RX |
| Baud | 1,000,000 | 115200 |
| Frame | 8-N-1 | 8-N-1 |
| Flow control | None | None |
| Oversampling | 16 | 16 |
| NVIC global interrupt | Disabled | Enabled, priority 1 |

USART1은 URT-2가 half-duplex TTL 전환을 담당하므로 STM32에서 Single-wire mode로
설정하지 않습니다. 현재 USART1은 blocking HAL I/O를 사용하므로 global interrupt가
필요하지 않고, USART2는 `HAL_UART_Receive_IT()`를 사용하므로 interrupt가 반드시
필요합니다.

CubeMX 코드 재생성 후에는 다음 두 항목을 먼저 확인합니다.

```text
huart1.Init.BaudRate == 1000000
USART2_IRQHandler() → HAL_UART_IRQHandler(&huart2)
```

### SPI1: BNO086

| Setting | 값 |
|---|---|
| Mode | Full-Duplex Master |
| Frame | 8-bit, MSB first |
| Clock | CPOL `High`, CPHA `2 Edge` (mode 3) |
| Prescaler | `/16` → PCLK2 16MHz 기준 1 MHz |
| NSS | Software (`IMU_CS`를 GPIO로 직접 제어) |

`SPI1`과 IMU 제어선 4개(`IMU_CS`, `IMU_INT`, `IMU_RST`, `IMU_WAKE`)는
`.ioc`에 등록되어 있고 초기화는 CubeMX 생성 코드가 담당합니다. `bno086.c`는
`servo_bus`가 `huart1`을 넘겨받는 것과 같은 방식으로 `hspi1` 핸들만 빌려
씁니다.

```c
bno086_init(&imu, &hspi1, 5000U);
```

CubeMX 재생성 후 확인할 항목은 다음과 같습니다.

```text
Inc/stm32f4xx_hal_conf.h  →  #define HAL_SPI_MODULE_ENABLED
Drivers/STM32F4xx_HAL_Driver/{Src,Inc}/stm32f4xx_hal_spi.{c,h}  →  존재 확인
PA5가 LD2 GPIO_Output이 아니라 SPI1_SCK인지
PB2/IMU_RST 초기 출력이 High인지
PA8/IMU_INT가 Pull-up Input인지
PB6/IMU_CS 초기 출력이 High인지
```

### BNO086 단계별 bring-up

부팅 시 BNO055가 없으면 BNO086 경로가 자동 실행됩니다. 순서는 `CS High → RST
Low 30ms → RST High → 300ms 부팅/INT 관찰 → 최초 SHTP packet → Product ID →
Game Rotation Vector 200Hz`입니다. 정상 경로의 SPI read는 active-low `IMU_INT`가
Low일 때만 수행하며, 각 packet은 4-byte SHTP header의 continuation bit를 제외한
길이가 `4..1024` 범위인지 확인합니다.

```text
BNO086 bring-up: SPI1 mode 3, 1MHz; CS=PB6 RST=PB2 INT=PA8(active-low)
BNO086 RESET: INT before=HIGH during=HIGH after=LOW; first LOW=yes at ...ms
BNO086 SHTP: first=valid header=.. .. 00 .. len=... channel=0 seq=... packets=...
BNO086 Product ID: status=0 entries=...
  product[0]: part=... version=... build=... reset=...
BNO086 Rotation Vector: reports=... q_x10000=(...,...,...,...) angles_tenths=(...,...,...)
BNO086 bring-up result: ok
```

부팅 로그가 실패하면 먼저 `spotctl console send imuprobe`를 실행합니다. 이 명령만 INT를
무시한 blind header read와 SPI mode sweep을 수행하므로 다음처럼 범위를 나눕니다.

- `first LOW=no`, blind read `DATA`: SPI는 동작하고 PA8/INT 배선 문제입니다.
- INT가 Low지만 header가 `00 00 00 00` 또는 `FF FF FF FF`: PA6/MISO, CS, 전원,
  또는 SPI mode를 확인합니다.
- header 길이가 `4..1024` 밖: SCK edge, CS timing 또는 신호 품질 문제입니다.
- SHTP packet은 valid지만 Product ID `status != 0`: MOSI/PA7 또는 host-to-sensor
  command 경로를 우선 확인합니다.
- Product ID는 성공하지만 Rotation Vector timeout: report enable/SH-2 처리 단계
  문제이며 기본 SPI 배선은 정상입니다.

HAL SPI 드라이버는 프로젝트에 없어서 프로젝트 HAL과 같은 판인
`STM32Cube_FW_F4_V1.28.3`에서 복사해 넣었습니다. `.ioc`에 SPI1이 들어갔으므로
CubeMX에서 열면 이 드라이버는 자동으로 관리됩니다.

## 콘솔 명령

명령 종료는 `CR`, `LF`, `CRLF`를 모두 지원합니다. 일부 시리얼 터미널은 로컬
에코를 하지 않으므로 입력 글자가 화면에 보이지 않아도 Enter를 누르면 명령이
실행됩니다.

```text
ping ID          한 서보 응답 확인
scan             설정된 ID 1..12 확인
uarttest         USART1 loopback 확인 (URT-2 분리, PA9-PA10 직결)
busprobe ID      Ping 후 USART1 원시 수신 바이트 출력
linestate        PA9/PA10을 풀다운으로 눌러 구동/개방 판별; 계측기 불필요
i2cscan          I2C1에서 BNO055 탐색
spitest          SPI1 loopback 확인 (BNO086 분리, D11-D12 직결)
imuprobe         BNO086 리셋 후 H_INTN과 SHTP 헤더 확인
read ID          위치, 속도, 부하, 전압, 온도, 전류 읽기
move ID RAW      단일 서보 안전 이동; 현재 위치에서 최대 256 tick
targets          stand 목표 raw 위치 확인; 움직이지 않음
profile [S A]    이동 속도(1..3400)와 가속도(0..254) 조회/설정
echo on|off      STM32 입력 echo 켜기/끄기 (부팅 기본 off)
hold             현재 위치를 목표로 설정한 뒤 전체 토크 활성화
stand            12축 목표를 한 번에 SYNC_WRITE하는 stand 이동 (J2=45°, J3=90°)
stand11          네 다리를 곧게 폄 (J2=0°, J3=0°); 캘리브레이션 확인용
trot [C [MS]]    공용 C sim-trot; 1..10회, 주기 600..5000ms (기본 1회/800ms)
trotplace [C [MS]] 제자리 대각 트롯; 1..10회, 주기 600..5000ms
trot2 [C [MS]]   원형 발끝 대각 트롯; 1..10회, 주기 600..5000ms
trot3 [C [MS]]   65% duty 중첩 trot + limiter/진단; 기본 1400ms, 진단 최대 1800ms
gaitdiag          마지막 보행의 tracking/전원/limiter/실제 timing 통계
baldiag           최근 32 balance frame과 마지막 tilt-safety snapshot
jump [C [MS]]    제자리 반복 점프; 0=계속, 1..20회, 주기 800..5000ms
relax            전체 토크 해제
safety           stall 검출기 상태와 래치된 fault 확인
recover          safety fault 해제 후 현재 위치에서 hold
imu on            10 Hz IMU 자세 로그 출력 시작
imu off           IMU 자세 로그 출력 중지 (부팅 기본값)
imu status        현재 IMU 로그 설정 확인
balance full      절대 수평 목표와 최대 제한 보정 활성화 (부팅 기본값)
balance normal    보행 시작 자세를 목표로 일반 보정 활성화
balance on        마지막으로 선택한 보정 모드 활성화
balance off       트롯 자세 보정 비활성화(트롯 tilt 감시는 유지); 점프 IMU 보호 비활성화
balance status    IMU, 기준 자세, 최대 오차·관절 보정·지연 frame 확인
help             도움말
```

`trot`, `trotplace`, `trot2` 또는 `jump` 실행 중 `Ctrl+C`를 누르면 USART2 RX 인터럽트가 즉시 중단
플래그를 설정합니다. 현재 frame을 마친 뒤 stand 목표를 전송하고 프롬프트로
돌아옵니다.

## J3만 뒤집으면 안 되는 이유

처음에는 J3 각도만 stance 기준으로 반사해 봤는데 틀린 방법이었습니다. 발끝
위치는 J2와 J3가 **함께** 만들기 때문에, 짝지어진 둘 중 하나만 뒤집으면 궤적이
거울이 아니라 일그러집니다. 계산으로 확인한 결과:

| | 전후축 방향 전환 횟수 | 지지 구간 발 높이 |
|---|---|---|
| 뒷다리 | 2 (정상) | 일정 |
| 앞다리 그대로 | 2 | 일정 |
| **앞다리 J3만 반사** | **4** | **변함** |

깨끗한 원 궤적은 한 주기에 정확히 두 번 방향을 바꿉니다. 여분의 두 번이 실기에서
**"박찬 직후 살짝 앞으로 움직이는"** 현상으로 나타났습니다.

## 캘리브레이션은 측정값입니다

`joints.json`의 `center`와 `direction`은 실물을 재서 얻은 값입니다. 계산으로
유도하거나 증상을 없애려고 옮기면 안 됩니다 — 그 순간 이 파일은 기계의 기록이
아니라 로직 오류를 숨기는 자리가 되고, 그 위의 모든 추론이 무너집니다.

지켜야 할 불변식이 `config/servo_calibration.md`에 적혀 있습니다.

```text
기준 위치: 2048
목표 위치 계산: 기준 위치 + Offset
```

모든 관절의 `center`는 `2048` 근처여야 하고 `offset`은 작은 보정값입니다.
`offset`이 수백을 넘어가면 캘리브레이션이 아니라 실수입니다.

값을 바꿔야 한다면 `spotctl calibrate`로 **다시 측정**합니다. 그 자리에서 어떤
값이어야 하는지 계산으로 알아냈더라도, 그것은 측정으로 확인할 가설입니다.

### 실기 확인: 앞뒤는 동일하고 좌우는 반대

2026-08-09 장착 기준을 다시 확인했습니다. 같은 쪽의 앞/뒤 관절은 동일하고 좌/우
관절만 서로 반대입니다. 교차 장착됐던 앞 J3 서보 두 개를 물리적으로 교환해
`ID 3=FL J3`, `ID 6=FR J3`의 표준 매핑으로 복원했습니다. center/offset은 물리
ID를 따라 유지하며, 혼을 다시 장착했으므로 실기에서 재확인합니다.

```text
J3 stand  FL(3) 3084   RL(9) 3020
          FR(6)  953   RR(12) 1023

방향      FL = RL = (-1, -1, +1)
          FR = RR = (+1, +1, -1)
```

`stand11`은 각도 0이라 direction과 무관하게 같은 center를 명령합니다. `stand`는
앞뒤 모두 같은 논리각과 같은 쪽의 동일한 raw tick 방향을 사용합니다.

## `stand11`: 캘리브레이션 확인용 1자 자세

`stand`는 저장된 raw 자세가 아니라 표준 각도 `J2=45°, J3=90°`를 계산합니다.
`stand11`은 모든 관절을 **표준 각도 0**으로 보내 두 링크를 일직선으로 만듭니다.

이 자세가 유용한 이유는 **물리적으로 애매하지 않기** 때문입니다. 다리가 곧은지
아닌지는 보면 압니다. 그래서 캘리브레이션을 대조할 기준으로 쓸 수 있습니다.

명령은 **관절이 가동 한계에 100 tick 이내로 붙으면 거부**합니다. 각도 0이 가동
끝에 있는 캘리브레이션은 그 각도에 도달하는 대신 하드 스톱을 밀게 되고, 그건
stall 검출기가 잡으라고 만든 상황이라 애초에 명령하지 않는 편이 낫습니다.

```text
ERROR: position limit; servo=3
```

2026-08-09 현재는 통과합니다.

## 레이어 경계: 정책과 하드웨어

`gait_policy.h`는 STM32와 MuJoCo가 **한 벌을 공유**합니다. 그 경계가 의미를 가지려면
정책에 하드웨어 사실이 들어가면 안 됩니다.

예전에는 들어가 있었습니다. 정책 함수들이 다리별 `forward_signs` 배열을 받았고,
그 내용은 **"각 다리가 어느 방향으로 장착됐는가"** 였습니다. 결과가 나쁩니다:

- MuJoCo는 자기 URDF에 맞추려고 그 값을 **덮어썼습니다**
- 그래서 시뮬레이터가 **바로 그 질문에 대해서만 권위를 잃었습니다**
- 같은 물리적 사실을 부호로도, 캘리브레이션 `direction`으로도 표현할 수 있어
  양쪽에서 반쯤씩 고쳐지는 상황이 생겼습니다

2026-08-09에 인자를 제거했습니다. 이동 방향은 `GAIT_POLICY_STANCE_TRAVEL` 하나로
고정되고, 다리 장착 정보는 **캘리브레이션의 `center`/`direction`에만** 있습니다.

```text
정책  gait_policy.h        하드웨어 손잡이 없음, 시뮬과 펌웨어가 동일 입력
  ↓ 관절 각도
액추에이터  actuator_control.c / motor_capability.h   속도·가속도 feasibility
  ↓ 실행 가능한 관절 각도
캘리브레이션  robot_config.c / joints.json   center, direction (장착 정보 전부)
  ↓ 서보 tick
```

동작이 바뀌지 않았음은 리팩터링 전 출력을 저장해 대조했습니다 — `trot`, `trot2`,
`jump` 150프레임이 전부 동일하고, MuJoCo 10주기가 `+2.225m / 56.3% / TROT`로
기록값을 그대로 재현합니다.

호스트의 레거시 Python gait(`SpotRobot.gait_targets`)는 공용 정책과 **별개 구현**이라
자체 `gait_forward_signs`를 씁니다. 이 궤적 부호는 뒷다리 기준인 네 다리 `+1`로
통일했으며, 관절 `direction`은 앞뒤 동일·좌우 반대 규칙을 따릅니다.

## STS3215 속도 계측과 `trot3`

`motor_capability.h`가 12V STS3215의 actuator 설정을 한곳에서 관리합니다.
정격 최대 속도는 `270 deg/s`, 실기 command limit은 그 90%인 `243 deg/s`입니다.
가속도는 세 개의 20ms frame에 command limit까지 도달하는 `4050 deg/s²`로
제한합니다. 10% 여유는 체중 부하, 버스 지연과 12V 레일 전압 강하를 위한 것이며,
정책에는 이 숫자가 들어가지 않습니다.

```bash
conda run -n spot_omg python -m tools.servo_tool.servo.gait_analysis
```

기본 주기/50Hz의 full-amplitude 한 사이클을 frame 차분한 결과입니다.

| 정책 | J1 최대 | J2 최대 | J3 최대 | 판정 |
|---|---:|---:|---:|---|
| `trot` | 0.0°/s | 352.3°/s | 585.7°/s | 정격 크게 초과 |
| `trot2` | 0.0°/s | 224.6°/s | 278.2°/s | 정격보다 3.0% 높음 |
| `trot3` (1400ms) | 0.0°/s | 182.8°/s | 227.4°/s | 243°/s command limit 이내 |

회귀 시험의 5% transient margin은 20ms 유한 차분과 무부하 사양 편차를 구분하기
위한 분석 허용치일 뿐 실제 command limit이 아닙니다. `trot`은 이 허용치도
초과하므로 빠른 실기에 적합하지 않습니다. `trot2`는 분석 허용치 안이지만 실제
보행은 부하가 있으므로 243°/s limiter를 거치는 `trot3`가 실험 경로입니다.

`trot3`는 원형 발끝 형상은 유지하면서 duty를 `0.50`에서 `0.65`로 늘립니다.
사이클 시작과 중간에 각각 15%씩, 합계 30%의 네 발 stance를 두고 첫 명령부터
대각 두 발을 들지 않습니다. 기존 `trot`/`trot2` canonical 출력은 변경하지
않았습니다. IMU balance까지 끝난 canonical angle을 `actuator_control.c`가
속도/가속도 제한한 다음에만 `robot_config.c`가 tick으로 변환합니다.

두 번째 바닥 시험은 phase 0.65에서 FL+RR이 swing으로 전환된 직후 오른쪽 tilt로
중단됐습니다. 이때 leg-length/knee 보정은 14.3°였지만 J1 횡보정은 0.7°뿐이었습니다.
따라서 `trot3`는 네 발 overlap 동안 다음 support diagonal로 하중을 넘기고, swing
구간에는 그 값을 유지하는 1.5° canonical J1 preload를 사용합니다. 이는 좌우 servo
방향 보정이 아니라 몸체 좌표의 보행 형상이므로 shared policy 안에 있습니다.

```text
gait phase → trot3 overlap canonical → balance → actuator limiter
           → center/direction calibration → sync_positions
```

`sts3215_sync_move()`는 활성화하지 않았습니다. 현재 구현은 모든 서보에 같은
speed/acceleration 값을 반복하고, STS3215 내부 position profile이 매 20ms 새
목표를 받을 때의 추종/jerk 개선 근거도 아직 없습니다. 먼저 위치-only Sync Write를
유지해 command와 actual의 차이를 측정합니다. actuator command frame에는 이미
관절별 position/velocity/acceleration이 있으므로, 실측 결과가 이득을 보일 때
관절별 profile packet으로 확장할 수 있습니다.

`trot3`에서는 STS3215 내부 profile이 outer limiter보다 느린 병목이 되지 않도록
`profile 3400 254`를 요구합니다. `profile 800 80`은 거치대에서 궤적 방향을 보는
용도였고 실제 기록에서도 무부하 peak error가 314 tick이었습니다. 이 상태로
바닥 보행을 시작하면 software limiter가 243°/s로 제한해도 servo 내부 profile이
훨씬 뒤처집니다. 새 firmware는 이 설정에서 움직이지 않고 오류를 반환합니다.

대각 두 발만 지지하는 trot은 느리게 실행할수록 정적으로 안정해지는 보행이
아닙니다. 2000ms 바닥 시험에서 오른쪽으로 전도된 실측과 MuJoCo 주기 sweep을
반영해 `trot3` 기본값은 1400ms입니다. 1800ms는 원인 분리를 위한 비교 진단에만
허용하며 정상 보행 권장값이 아닙니다. roll 또는 pitch가
12°를 2 frame 연속 넘으면 즉시 보행을 중단하고 stand 목표를 요청합니다.
0.65 duty의 실제 swing 시작(phase 0.15/0.65)에 non-blocking step-sync monitor를
맞춥니다.
단일-cycle 안전 시험은 시작과 종료가 대부분이므로 `trot3`만 amplitude ramp를
500ms에서 700ms로 늘립니다. 1400ms 한 사이클의 실제 ramp 포함 peak 요구 속도는
222.0°/s에서 140.9°/s로 줄며, 기존 `trot`/`trot2` ramp는 바뀌지 않습니다.

`trot`, `trot2`, `trot3`는 기존 한 축/frame round-robin state read를 공유합니다.
진단 때문에 bus read를 추가하지 않으며 deadline도 기존 absolute 20ms 계산을
그대로 사용합니다. `gaitdiag` 또는 `trot3` 종료 직후 출력에서 다음을 확인합니다.

- servo ID, leg/joint, gait phase
- commanded/measured position과 signed error
- measured speed/current/load register 값과 voltage
- joint별 peak/평균 절대 error를 tick과 degree로 표시
- joint별 peak error phase, peak current, minimum voltage
- 96 tick 이상의 tracking lag와 11V 이하 droop가 같은 sample에서 발생했는지
- 같은 joint가 두 sample 연속 lag일 때의 향후 derate 권고 hook
- nominal/실제 gait 시간, step-sync miss/오차, balance update 최대 공백
- limiter의 관절별 각도 변형과 FK로 환산한 발끝 변형

`baldiag`는 UART를 매 frame 출력하지 않고 RAM의 최근 32-frame ring buffer를
보행 뒤에 덤프합니다. phase, support pair, raw/filtered Roll/Pitch와 rate, PD 출력,
J1/다리 길이/knee/foot-placement 보정, saturation, limited joint mask와 누적 lag를
포함합니다. Tilt Safety가 발생하면 그 frame과 당시 최악의 sampled servo error 및
최저 전압을 별도 snapshot으로 보존합니다.

정상 종료의 마지막 phase도 IMU balance를 계속 적용합니다. gait amplitude가 이미
0인 마지막 target이 balance된 stand geometry이므로 이를 그대로 hold하며, 별도의
raw stand packet으로 leg-length correction을 한 frame에 제거하지 않습니다. Tilt,
bus fault 또는 사용자 중단은 기존 안전 stand 복귀 경로를 계속 사용합니다.

이 tracking 진단은 정상 동작 중 성능을 평가할 뿐 토크를 끄지 않습니다. 큰 오차와
큰 effort가 지속되면 토크를 끄는 기존 stall protection과 역할이 분리되어 있습니다.
개별 servo current는 sampling 시점도 서로 다르므로 합산해 전체 소비전류로 해석하지
않습니다. 전원 한계는 minimum voltage와 lag+droop 동시 발생을 중심으로 판단합니다.

## Stall 안전 보호

서보 전원이 12V 80W PD(약 6.7A)이고 STS3215 하나의 stall 전류가 약 2.7A입니다.
다리가 걸리면 몇 백 ms 안에 레일 전체가 내려가고 **12축이 동시에 전원을 잃습니다.**
한 관절이 고장 나는 것보다 나쁩니다 — 로봇이 무동력 상태로 그대로 주저앉습니다.
그 전에 STM32가 스스로 토크를 끊는 것이 이 기능의 목적입니다.

### 판정 방식

**stall과 정상 보행을 가르는 것은 부하의 크기가 아닙니다.** 체중을 받는 착지는
부하가 높아도 서보가 목표에 **도달**합니다. 걸린 다리는 부하가 높으면서 위치
오차가 **줄지 않습니다.** 그래서 위치 오차가 주 신호이고 load/current는 보조입니다.

이 선택 덕분에 gait phase별 load baseline이 필요 없고, 보행을 재조정해도 낡지
않습니다.

```text
위치 오차 >= 240 ticks          (배리어의 "따라잡음" 기준은 48 ticks)
    AND (load >= 500 OR current >= 700)
    AND 150ms 지속
        → SAFETY_FAULT_STALL
```

착지 충격이나 swing 시작의 부하 스텝은 수십 ms라 150ms 창을 넘지 못합니다.
서보가 스스로 올린 hardware error와 온도 초과는 지속 시간 없이 즉시 fault입니다.

### 감지 후 동작

```text
stall 감지 → 12축 전부 토크 OFF → FAULT latch → 이후 모션 명령 거부
```

**한 다리만 끄지 않습니다.** 나머지 셋이 계속 구동하는 채로 한 다리가 풀리면
동력이 걸린 채 넘어지므로 더 위험합니다.

fault가 걸리면 `return_to_stand_best_effort()`는 아무것도 하지 않습니다. 다른
실패는 다리를 정렬하는 게 맞지만 stall은 정반대입니다 — 걸린 관절에 stand 목표를
다시 보내면 전류가 곧바로 되돌아옵니다.

`relax`는 **막지 않습니다.** 토크를 끊는 방향은 언제나 안전하므로 어떤 상태에서도
동작해야 합니다. 반면 `stand`, `hold`, `move`, 보행 명령은 fault 중 거부됩니다.

### 복구

```bash
spotctl console send safety     # 무엇이 걸렸는지 확인
spotctl console send recover
```

`recover`는 **먼저 ping**합니다. 래치된 stall과 전원이 내려간 상태는 콘솔에서
똑같아 보이지만 대응이 정반대이기 때문입니다 — 레일이 죽었으면 명령을 보낼 대상이
없으므로 `servo power lost`로 보고하고 그만둡니다.

서보가 응답하면 latch를 풀고 **현재 위치로** hold합니다. 걸렸을 때 쫓던 gait
목표로 토크를 켜면 그 순간 다시 장애물로 돌진합니다.

### 버스 예산

unicast 읽기 하나가 약 10ms입니다([servo_bus.c](Src/servo_bus.c)의 안정화 지연).
12축 전수는 123ms라 20ms 프레임 안에 들어가지 않습니다. 그래서:

| 지점 | 주기 | 내용 |
|---|---|---|
| 프레임 루프 | 20ms | 라운드로빈 1축 (일순 240ms) |
| `wait_for_step_sync` | 주기당 2회 | 12축 전수 스냅샷 |

`read_state`(15바이트)는 `read_position`(2바이트)과 **비용이 같습니다** — 10ms
안정화가 지배적이고 추가 13바이트는 1Mbps에서 0.13ms입니다. 배리어는 이미 전수
읽기를 하고 있었으므로 검출기는 그 자리에서 공짜로 스냅샷을 받습니다.

의심 축이 생기면 라운드로빈을 멈추고 **그 축에 lock-on** 합니다. 일순을 기다리지
않고 다음 프레임에서 바로 확인하므로 150ms 창 안에 판정이 끝납니다.

프레임 읽기는 원래 `HAL_Delay`로 버리던 여유를 쓰며, 마감이 절대 시각이라 명령이
나가는 시점은 바뀌지 않습니다. 여유가 부족해지면 `balance status`의 `late=`가
0보다 커지므로 그 값으로 확인합니다.

### `jump`에는 적용하지 않았습니다

점프는 탄도 운동이라 서보가 따라올 수 없는 위치를 의도적으로 명령합니다. 큰 위치
오차와 높은 부하가 **정상**이므로 같은 판정을 걸면 이륙마다 오탐이 납니다.
점프 전용 기대값이 필요하며 아직 없습니다. `jump`는 이미 래치된 fault가 있을 때
시작을 거부하기만 합니다.

### 실기 시험 순서

강한 stall부터 시험하지 않습니다.

```text
1. 호스트 단위 시험            pytest ... firmware/stm32-learning/tests/test_firmware_c.py
2. 공중(거치대) 보행           오탐이 없는지 — safety 로 candidates 확인
3. 손으로 아주 약한 저항        검출은 되되 즉시 풀리는지
4. 낮은 토크 / 느린 gait        profile 800 80, trot2 1 3000
5. 마지막에 실제 바닥
```

2단계에서 `safety`의 `candidates`가 계속 늘어나면 임계가 낮은 것이고,
`peak_error`가 240에 한참 못 미치면 임계를 낮출 여지가 있다는 뜻입니다.

## 호스트에서 콘솔 명령 보내기

터미널에서 직접 타이핑하는 대신 호스트의 `spotctl console`로 같은 명령을 보낼
수 있습니다. 서보 버스의 주인은 그대로 STM32이므로 IMU 균형, step-sync monitor,
`Ctrl+C` 복귀가 모두 유지됩니다.

ST-LINK가 붙어 있으면 `spotctl`이 알아서 이 콘솔로 명령을 보냅니다. 포트나
`console` 같은 키워드를 따로 칠 필요가 없습니다.

```bash
conda activate spot_omg
spotctl profile 800 80
spotctl stand
spotctl trot2 1 1600
spotctl balance status
```

하위 명령이 없는 펌웨어 명령(`ping`, `read`, `move`, `echo`, `uarttest`,
`busprobe`)이나 대화형 프롬프트는 `console`을 사용합니다.

```bash
spotctl console send read 1
spotctl console shell
```

`imu on`처럼 펌웨어가 스스로 계속 출력하는 것을 보려면 `watch`를 씁니다.
`send`는 프롬프트가 돌아올 때까지만 읽고 포트를 닫으므로 이후 스트림을 놓칩니다.

```bash
spotctl console watch imu on    # 명령을 보낸 뒤 계속 수신, Ctrl+C로 종료
spotctl console watch           # 이미 켜져 있으면 수신만
```

`spotctl`은 접속 직후 `echo off`를 보내 부팅 배너를 버리고 프롬프트를 맞춥니다.
빈 줄은 펌웨어가 무시하고 프롬프트를 출력하지 않으므로 동기화에 쓰지 않습니다.
자세한 사용법은
[`tools/servo_tool/README.md`](../../tools/servo_tool/README.md)를 참고하세요.

부팅 기본 profile은 최대값 `speed=3400`, `acceleration=254`입니다. 예를 들어
더 부드러운 시험값으로 낮추거나 현재 값을 확인할 수 있습니다.

```text
profile 800 80
profile
```

콘솔은 수신 문자를 직접 echo하고 Backspace/Delete 편집을 지원합니다. 제어 문자를
렌더링하지 못하는 터미널을 위해 부팅 기본값은 `echo off`이며 터미널의 Local
Echo를 켭니다. 부팅 도움말과 `help` 출력 마지막에 현재 echo 상태가 표시됩니다.
Enter는 CR, LF, CRLF를 모두 처리합니다.

부팅 완료 후와 각 명령 처리 후에는 `# ` 프롬프트가 출력됩니다.

권장 첫 시험 순서는 다음과 같습니다.

```text
ping 1
read 1
move 1 PRESENT_POSITION_PLUS_SMALL_DELTA
read 1
relax
```

`stand`는 12개 ID가 모두 응답해야 시작합니다. 먼저 현재 위치를 목표 위치로
설정한 뒤 토크를 켜므로 오래된 Goal Position으로 튀는 동작을 방지합니다. 이후
12개 stand 목표를 한 번의 SYNC_WRITE로 전송하며 중간 보간 ramp는 사용하지
않습니다. 실제 이동 속도는 `profile`의 speed/acceleration 제한을 따릅니다.
부팅 기본 profile은 `speed=3400`, `acceleration=254`이며 direct stand 실기
속도가 적절함을 확인했습니다.

STM32와 MuJoCo의 `sim-trot`은 `gait_policy.h`에 있는 동일한 HAL 독립 C 정책을
호출합니다. 기본 주기는 `800ms`, 제어율은 `50Hz`, stance duty는 `50%`, J1
기본 외전은 `4°`, J2/J3 기준 자세는 `45°/90°`, 보폭 입력은 `8.8°`, 리프트
입력은 `30°`입니다. FL+RR과 FR+RL 대각선 쌍은 정확히 반 주기 차로 움직입니다.

J2/J3를 독립 각도 파형으로 만들지 않습니다. 기준 자세의 발끝 위치에서 시작해
`들기 → 든 채 앞으로 보내기 → 내리기`의 Cartesian 발끝 궤적을 만든 뒤 2-link
IK로 J2/J3를 함께 계산합니다. 따라서 보폭에 따른 다리 높이 변화가 자동으로
상쇄되고 MuJoCo와 STM32가 같은 canonical 관절 목표를 생성합니다. 시작과 종료는
최대 `500ms` smootherstep 진폭 ramp를 적용하며 종료 후 stand 자세로 복귀합니다.

각 대각선이 스윙을 시작하기 직전에는 실제 위치 기반 step-sync monitor를
실행합니다. 이전 구현은 오차가 `48 tick` 이내가 될 때까지 모든 12축을 반복
읽으며 phase를 막았습니다. 실기에서 550ms 대기 동안 IMU balance도 멈춰 Roll
전도를 키운 것이 확인되어 blocking barrier를 제거했습니다. 현재 monitor는 기존
round-robin의 최근 위치를 현재 목표와 비교해 transition/miss/peak error만 기록하고,
추가 bus read나 `HAL_Delay` 없이 다음 20ms deadline을 유지합니다. 큰 lag는
`gaitdiag`와 derate hook으로 처리하며 공용 궤적 수식은 바꾸지 않습니다.

공용 정책은 네 다리에 같은 canonical 관절 목표를 냅니다. 앞뒤 장착 차이는
`robot_config.c`의 center/direction에서만 처리하며, 현재 같은 쪽 앞뒤 다리는 같은
direction을 사용합니다. 보행 전후 진행 방향은 정책의 단일
`GAIT_POLICY_STANCE_TRAVEL` 값으로 결정됩니다.

J1은 고정하지 않습니다. 기본 외전 `+4°` 위에 IMU Roll PD 출력을 적용해 지지
다리는 기울기의 반대쪽으로 몸체를 밀고 스윙 다리는 넘어지는 쪽으로 다음 발을
놓습니다. J1 보정 제한은 `±5°`입니다. 실제 서보 속도와 가속도는 `profile`을
따릅니다.

```text
trot                 # 1회, 공용 정책 기본 800ms 주기
trot 3               # 3회, 800ms 주기
trot 3 1200          # 같은 궤적을 더 느린 1200ms 주기로 실행
```

### 제자리 트롯

`trotplace`는 `trot`과 동일한 FL+RR / FR+RL 대각선 위상, 리프트, J1 및 IMU
균형 보정, 실제 위치 step-sync monitor를 사용합니다. 단순히 보폭을 0으로 만들면 발을
드는 동안 몸체가 수동적으로 뒤로 밀리기 때문에, MuJoCo 10주기 순이동이 가장 작았던
전진 보상 비율 `0.39`를 기본값으로 사용합니다.

```text
trotplace 1 1600     # 거치대에서 첫 1회 시험
trotplace            # 기본 1회/800ms
trotplace 5 1000     # 5회 반복; Ctrl+C 중단 가능
```

보상 비율은 `robot.h`의 `ROBOT_TROT_IN_PLACE_TRAVEL_SCALE`에서 조절합니다. 실제
기체가 앞으로 흐르면 값을 낮추고 뒤로 흐르면 값을 올립니다. 처음에는 `0.05`씩,
정지점 부근에서는 `0.01`씩 변경합니다. `0`은 관절 궤적상 전후 이동이 전혀 없는
값이지만 동역학적으로 제자리를 보장하지 않습니다.

### 원형 발끝 트롯

`trot2`는 기존 `trot`을 대체하지 않는 별도 공용 C 궤적입니다. 접지 구간에는
발끝이 지면을 따라 앞에서 뒤로 움직이고, 스윙 구간에는 상반원을 따라 들렸다가
내려옵니다. 각도 파형을 직접 만들지 않고 원 위의 발끝 좌표를 매 frame IK로 풀어
J2/J3를 함께 움직입니다. 원 꼭대기의 기본 접힘은 `J2=78°`, `J3=108°`이며 L2는
몸체 수평에서 약 12° 아래까지 접힙니다.

MuJoCo와 STM32는 모두 `gait_policy_trot2_targets()`를 호출합니다. STM32에서는
네 다리에 같은 canonical 진행 방향, 50Hz IMU/J1 보정, 실제 위치 step-sync monitor와
`Ctrl+C` stand 복귀를 그대로 사용합니다.

첫 실기 시험은 반드시 거치대에서 느리게 실행합니다.

```text
scan
balance full
balance status
profile 800 80
stand
trot2 1 1600
```

방향, 원형 스윙과 기구 간섭이 정상이면 한 단계씩 올립니다.

```text
profile 3400 254
trot2 1 1200
trot2 3 800
trot3 1 1400      # 먼저 거치대; actuator limit 및 tracking report
```

STM32 `balance full`과 같은 이득의 MuJoCo 기본값은 10주기에 약 `2.225m` 전진하는
큰 보폭이므로 로봇을 바닥에 놓고
처음부터 `trot2 3 800`을 실행하지 않습니다.

BNO086이 정상 초기화되면 `balance full`이 부팅 기본값이며 다음 `trot`부터
Roll/Pitch를 50Hz로 읽습니다. full 모드는 IMU가 나타내는 절대 Roll/Pitch `0°`를
목표로 몸체 수평을 유지합니다. 보드와 몸체 사이의 기계적 장착 오차가 있으면
`robot_config.h`의 `ROBOT_IMU_LEVEL_*_TENTHS`를 0.1° 단위로 보정할 수 있습니다.
`balance normal`은 시험 시작 시의 자세를 기준으로 유지합니다. 확인된 기체
좌표계는 앞쪽이 내려가면 Pitch `+`, 오른쪽이 내려가면 Roll `+`입니다. Yaw는
수평 유지에 사용하지 않습니다.

기본 정책은 IMU 필수입니다. BNO086 리셋 또는 리포트 구독이 실패하면 `trot`, `trot2`와 `jump`는
`IMU balance error`로 실행을 거부합니다. 센서 없는 개루프 시험이 꼭 필요할 때만
`balance off`를 명시하면 허용되며, `balance full`, `balance normal`, `balance on`
중 하나를 실행하면 다시 IMU 필수 정책으로 돌아갑니다. 빌드 기본값은 `robot.h`의
`ROBOT_IMU_BALANCE_DEFAULT_ENABLED`와 `ROBOT_IMU_BALANCE_DEFAULT_MODE`로 바꿀 수
있습니다.

Roll/Pitch 각도와 수치 미분 각속도를 결합한 공용 PD/IK 출력을 사용합니다. full
모드는 `Kp=1.0`, `Kd=0.04`입니다. Roll 8.3°에서 기존 J1 이득 `5deg/rad`는
약 0.72°만 만들면서 정규화 다리 길이 `0.145`가 knee 약 14°로 확대됐습니다.
동역학 부호 검증 뒤 full 모드는 J1 이득을 `15deg/rad`, 다리 길이 제한을
`0.08`로 조정해 같은 입력을 J1 약 2.17°, knee 약 7.3°로 분담합니다. normal
모드는 `Kp=0.6`, `Kd=0.04`, 길이 제한 `0.10`, J1 이득 `5deg/rad`를 유지합니다.
두 모드 모두 J1 제한 `±5°`, 4-sample 각도·각속도 필터,
오차 `±30°`, 각속도 `±120°/s` 입력 제한을 적용합니다. MuJoCo는 실제 발 접촉을
지지발 입력으로 쓰고 STM32는 공용 정책의 stance 위상을 사용합니다.
각속도 차분은 명시적인 signed 32-bit 연산을 사용합니다. 20ms unsigned 상수 때문에
감소하는 자세 오차가 큰 양의 rate로 바뀌는 Cortex-M 승격 버그가 실기 로그에서
확인되어 수정했으며, 최초 IMU 오차로 previous sample을 초기화해 시작 D-kick도
방지합니다.
`balance status`는 선택 모드와 목표, 마지막 기준·오차, 최대 Roll/Pitch, 최대 J1/J3
보정과 20ms deadline을 넘긴 frame 수를 출력합니다. IMU 읽기가 3회 연속
실패하면 stand 목표를 요청하고 명령을 오류로 종료합니다. IMU 없이 부팅했거나
비교 시험이 필요할 때만 `balance off`로 개루프 보행을 선택합니다.

```text
balance status
trot 1
balance status
balance off
balance on
balance normal
balance full
```

Roll 원인 분리 비교는 같은 firmware/profile에서 다음 순서로 한 cycle씩 수행합니다.
각 실행은 결과와 `gaitdiag`를 자동 출력하며 `baldiag`로 ring buffer를 다시 볼 수
있습니다. `balance off`에서도 trot의 IMU 관측과 12° Tilt Safety는 유지됩니다.
다만 이미 바닥 전도가 재현됐으므로 먼저 거치대, 그 다음 몸체를 실제로 받는 상부
하네스에서만 수행합니다.

```text
balance off
trot3 1 1400
balance full
trot3 1 1400
balance off
trot3 1 1800
balance full
trot3 1 1800
```

비교할 값은 `balance=on/off`, Peak Roll/Pitch, joint별 peak/mean tracking error,
step-sync miss/peak recent error, blocking wait(0ms), max balance gap,
limited joint/foot distortion와 Tilt snapshot
유무입니다. 1800ms 허용은 이 A/B 진단을 위한 것이며 반복 보행 승인이 아닙니다.

## 제자리 반복 점프

`jump`는 네 다리를 같은 위상으로 움직이는 50Hz 공용 C 궤적입니다. 기본 호출은
`cycles=0`이므로 `Ctrl+C`를 누를 때까지 1200ms 주기로 반복합니다. 횟수를 지정하면
해당 횟수만 실행하고 stand로 돌아옵니다.

```text
jump 1 2000       # 거치대에서 가장 먼저 확인할 느린 1회 시험
jump 3 1500       # 3회 반복
jump              # 1200ms로 계속 반복; Ctrl+C로 중지
```

한 주기는 `stand → 압축 → 도약 신전 → 공중 tuck → 착지 준비 → 충격 흡수 →
stand` 순서입니다. J1은 시작과 종료에서 `0°`이고 압축 구간 동안 `4°`까지
smootherstep으로 넓어집니다. 제자리 모드에서는 네 다리의 canonical J2/J3 각도가
항상 같습니다.

BNO086 balance가 켜져 있으면 시작 기준으로 Roll 또는 Pitch가 `30°`를 넘거나 IMU
읽기가 3회 연속 실패할 때 stand 목표를 요청하고 종료합니다. `balance off`에서는
이 보호가 비활성화되므로 거치대 시험 외에는 권장하지 않습니다. 이 점프 정책의 IMU는
현재 능동 자세 보정이 아니라 넘어짐 감시에 사용됩니다.

전진 확장을 위해 공용 함수는 정규화된 `forward_travel` 인자를 이미 받습니다.
현재 `robot.h`의 `ROBOT_JUMP_FORWARD_TRAVEL` 기본값은 `0.0f`입니다. 처음에는
`0.02f`처럼 작은 값부터 시험하고, 부호는
`gait_policy.h`의 `GAIT_POLICY_STANCE_TRAVEL`이 하중을 받는 발의 이동 방향을 정하며, **다리별 인자가 아닙니다.** 네 다리가 동일한 기구이므로 모두 같은 방향으로 밉니다. 대각선 짝이 각도와 지지 위상을 공유하는지는 `test_firmware_c.py`가 검사합니다. 허용 범위는
`±0.30`이지만 이는 소프트웨어 IK 한계일 뿐 실기 안전 범위가 아닙니다.

실제 도약은 배터리 전압, 바닥 마찰, STS3215 토크와 링크 질량에 크게 좌우됩니다.
첫 시험은 로봇을 지지대에 고정하고 `jump 1 2000`으로 관절 방향과 간섭부터
확인하며 비상 전원 차단을 준비합니다.

대부분의 콘솔 명령은 실행이 끝날 때까지 blocking됩니다. `trot`, `trot2`와 `jump`는 RX
인터럽트에서 `Ctrl+C`를 별도로 감지하므로 중간 정지가 가능하지만, 첫 실기 시험은
여전히 로봇을 지지대에 고정하고 1회만 실행하며 비상 전원 차단을 준비합니다.

서보 버스가 모두 timeout이고 측정 장비가 없다면 모든 전원을 끄고 URT-2를
STM32에서 분리한 뒤 PA9와 PA10을 점퍼선으로 직접 연결합니다. 다시 전원을 켜고
`uarttest`를 실행해 USART1 송수신을 loopback 방식으로 확인합니다. 시험 후에는
전원을 끄고 점퍼선을 반드시 제거한 다음 URT-2를 다시 연결합니다.

`busprobe 1`의 정상 Ping 원시 응답은 다음과 같습니다.

```text
BUSPROBE RX (6): FF FF 01 02 00 FC
```

`FF 00`처럼 일부 바이트만 보이거나 scan 실패 ID가 매번 바뀌면 개별 서보보다
1Mbps 수신 overrun을 먼저 의심합니다. 현재 구현은 RXNE 우선 직접 polling,
강제 inline, unicast 요청 간 10ms 안정화 간격을 사용합니다.

## 2026-08-05~07 개발 및 점검 결과

- Feetech packet, URT-2 bus, STS3215 register API와 12축 Sync Write 구현
- `joints.json`의 ID·center·direction을 STM32 설정으로 이식
- 부팅 시 torque OFF, 안전 단일 이동, current-position hold, direct stand 구현
- MuJoCo와 STM32가 함께 호출하는 Cartesian IK 기반 공용 C trot 정책 구현
- 원형 스윙 발끝과 L2 접힘을 갖는 공용 C `trot2` 및 STM32 콘솔 명령 구현
- 20ms 목표 갱신, 500ms 이하 진폭 ramp, non-blocking step-sync monitor 구현
- BNO055 `0x28`, CHIP_ID `0xA0`, NDOF 초기화 확인 (2026-08-08 BNO086 SPI로 교체)
- IMU 연속 로그 기본 OFF 및 `imu on|off|status` 추가
- 콘솔 명령 종료를 `CR`, `LF`, `CRLF` 모두 지원
- CubeMX 재생성으로 사라진 USART2 IRQ 복구
- CubeMX 재생성으로 115200이 된 USART1을 1Mbps로 복구하고 runtime guard 추가
- 생성 코드와 USER CODE에 중복된 USART2 IRQ/NVIC 정의 제거
- GNU Arm GCC 14.3 전체 링크, Servo Tool 63개와 `trot2` 5개 시험 통과

수정 전 STM32 `scan`에서는 ID 1~12가 모두 timeout이었고 USART1 baud 회귀와
1Mbps Debug 수신 overrun을 차례로 수정했습니다. 최종 펌웨어에서는 완전한 Ping
응답, ID 1~12 연속 scan 3회, `hold`, `stand`, `relax` 실기 성공을 확인했습니다.

상세한 날짜별 기록은
[`tools/servo_tool/HARDWARE_TEST_LOG.md`](../../tools/servo_tool/HARDWARE_TEST_LOG.md)를
참조합니다.

## 코드 구조

- `feetech_protocol.*`: 패킷, checksum, little-endian 변환
- `servo_bus.*`: UART request/status 및 URT-2 송수신 전환
- `sts3215.*`: STS3215 레지스터 API와 SYNC_WRITE
- `gait_policy.h`: MuJoCo/STM32 공용 trot/trot2/jump 궤적, IK, IMU 균형 정책
- `robot_config.*`: 12관절 ID, center, direction, stand 목표
- `robot.*`: 공용 정책의 서보 변환, step-sync monitor, jump, hold/stand/relax
- `app_console.*`: USART2 인터럽트 기반 진단 콘솔

관절 보정값은 `tools/servo_tool/config/joints.json`에서 옮겼습니다. 기구 조립이나
서보 ID가 달라지면 `robot_config.c`를 먼저 갱신해야 합니다.

`config/*.md`는 `joints.json`에서 자동 생성되는 사람이 읽기 위한 표이며 펌웨어가
런타임에 Markdown을 읽지는 않습니다. 값 변경 시 JSON과 `robot_config.c`를 함께
갱신하고 위 호스트 회귀 시험과 ARM 빌드를 모두 다시 수행합니다.
