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
| SPI1 | PA5 SCK, PA6 MISO, PA7 MOSI | BNO086 | 2 MHz, mode 3 |
| GPIO | PB6 CS, PA8 INT, PB3 RST, PB5 WAKE | BNO086 제어선 | 모두 active low |
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
BNO086 RST  → D3  / PB3  / GPIO
BNO086 WAK  → D4  / PB5  / GPIO
```

**`D2`는 비워 둡니다.** `D2`는 `PA10`이고 이 핀은 URT-2에서 돌아오는
`USART1_RX`입니다. 여기에 `INT`를 물리면 서보 버스가 죽고, 증상은 2026-08-08
기록의 `BUSPROBE RX: no bytes`와 똑같이 나타납니다.

주의할 핀이 둘 더 있습니다.

- `D13/PA5`는 NUCLEO 사용자 LED `LD2`가 달려 있는 핀입니다. 이제 SPI1_SCK이므로
  펌웨어에서 이 핀을 구동하던 코드(B1 버튼 → LD2 토글)를 제거했습니다. LED와
  직렬저항이 SCK에 부하로 걸리지만 2MHz에서는 문제되지 않습니다.
- `D3/PB3`은 SWD의 `SWO` 트레이스 핀입니다. GPIO로 쓰는 데 지장은 없지만 SWO
  트레이스를 쓰려면 다른 핀으로 옮겨야 합니다.

브레이크아웃 보드는 기본이 I2C 모드이므로 **`PS1`을 HIGH로 묶어 SPI 모드**로
바꿔야 합니다. SPI 모드에서 `PS0`은 `WAKE` 역할을 하며 위 배선의 `D4`가 그것입니다.

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
| Prescaler | `/8` → PCLK2 16MHz 기준 2 MHz |
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
```

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
stand            12축 목표를 한 번에 SYNC_WRITE하는 stand 이동
trot [C [MS]]    공용 C sim-trot; 1..10회, 주기 600..5000ms (기본 1회/800ms)
trotplace [C [MS]] 제자리 대각 트롯; 1..10회, 주기 600..5000ms
trot2 [C [MS]]   원형 발끝 대각 트롯; 1..10회, 주기 600..5000ms
jump [C [MS]]    제자리 반복 점프; 0=계속, 1..20회, 주기 800..5000ms
relax            전체 토크 해제
imu on            10 Hz IMU 자세 로그 출력 시작
imu off           IMU 자세 로그 출력 중지 (부팅 기본값)
imu status        현재 IMU 로그 설정 확인
balance full      절대 수평 목표와 최대 제한 보정 활성화 (부팅 기본값)
balance normal    보행 시작 자세를 목표로 일반 보정 활성화
balance on        마지막으로 선택한 보정 모드 활성화
balance off       트롯 자세 보정 및 점프 IMU 보호 비활성화
balance status    IMU, 기준 자세, 최대 오차·관절 보정·지연 frame 확인
help             도움말
```

`trot`, `trotplace`, `trot2` 또는 `jump` 실행 중 `Ctrl+C`를 누르면 USART2 RX 인터럽트가 즉시 중단
플래그를 설정합니다. 현재 frame을 마친 뒤 stand 목표를 전송하고 프롬프트로
돌아옵니다.

## 호스트에서 콘솔 명령 보내기

터미널에서 직접 타이핑하는 대신 호스트의 `spotctl console`로 같은 명령을 보낼
수 있습니다. 서보 버스의 주인은 그대로 STM32이므로 IMU 균형, step barrier,
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

각 대각선이 스윙을 시작하기 직전에는 실제 위치 기반 step barrier를 실행합니다.
직전 목표와 12개 관절의 Present Position 차이가 모두 `48 tick` 이내가
될 때까지 다음 phase로 진행하지 않습니다. 따라서 느린 관절이 있으면 빠른 관절도
지지 자세에서 기다린 뒤 대각선 두 발이 함께 스윙을 시작합니다. `1000ms` 안에
동기화되지 않으면 `step synchronization timeout`과 가장 오차가 큰 서보 ID를
출력하고 stand 목표를 요청합니다. 이 장벽은 실제 서보 추종 오차를 처리하는
STM32 전용 하드웨어 계층이며 공용 궤적 수식은 바꾸지 않습니다. `balance status`의 `Step sync` 항목에서 barrier
횟수, 누적 대기 시간, 진입 시 최대 위치 오차를 확인할 수 있습니다.

실기 시험을 기준으로 앞뒤 거울 대칭 기구의 전진 부호를 다시 정리했습니다. 현재
STM32 기체의 FL/FR는 `+1`, RL/RR는 `-1`입니다. 따라서 동일 위상의 대각선 쌍
`FL+RR`, `FR+RL`은 관절 각도상 서로 반대로 회전하지만 실제 발끝은 같은 방향으로
들려 앞으로 이동합니다. 설정은 `robot_config.c`의
`g_robot_gait_forward_signs`에서 다리별로 관리합니다. 이 값은 보행의 전후
궤적에만 적용되며 IMU Pitch/Roll 균형 보정 부호는 변경하지 않습니다.

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
균형 보정, 실제 위치 step barrier를 사용합니다. 단순히 보폭을 0으로 만들면 발을
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
기존 기체의 전진 부호 `FL/FR +1`, `RL/RR -1`, 50Hz IMU/J1 보정, 실제 위치 step
barrier와 `Ctrl+C` stand 복귀를 그대로 사용합니다.

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
모드는 MuJoCo `sim-trot`과 같은 `Kp=1.0`, `Kd=0.04`, 정규화 다리 길이 보정
제한 `0.15`를 사용합니다. normal 모드는 각각 `0.6`, `0.04`, `0.10`입니다.
두 모드 모두 J1 보정 이득 `5.0`, 제한 `±5°`, 4-sample 각도·각속도 필터,
오차 `±30°`, 각속도 `±120°/s` 입력 제한을 적용합니다. MuJoCo는 실제 발 접촉을
지지발 입력으로 쓰고 STM32는 공용 정책의 stance 위상을 사용합니다.
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
`g_robot_gait_forward_signs`가 앞·뒤 거울 대칭 기구에 맞게 적용합니다. 허용 범위는
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
- 20ms 목표 갱신, 500ms 이하 진폭 ramp, 실제 위치 step barrier 구현
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
- `robot.*`: 공용 정책의 서보 변환, step barrier, jump, hold/stand/relax
- `app_console.*`: USART2 인터럽트 기반 진단 콘솔

관절 보정값은 `tools/servo_tool/config/joints.json`에서 옮겼습니다. 기구 조립이나
서보 ID가 달라지면 `robot_config.c`를 먼저 갱신해야 합니다.

`config/*.md`는 `joints.json`에서 자동 생성되는 사람이 읽기 위한 표이며 펌웨어가
런타임에 Markdown을 읽지는 않습니다. 값 변경 시 JSON과 `robot_config.c`를 함께
갱신하고 위 호스트 회귀 시험과 ARM 빌드를 모두 다시 수행합니다.
