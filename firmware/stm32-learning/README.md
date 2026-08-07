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

## 인터페이스

| Peripheral | Pins | Purpose | Configuration |
|---|---|---|---|
| USART1 | PA9 TX, PA10 RX | URT-2 UART header | 1 Mbps, 8-N-1 |
| USART2 | PA2 TX, PA3 RX | ST-LINK VCP debug console | 115200, 8-N-1 |
| I2C1 | PB8 SCL, PB9 SDA | BNO055 | 100 kHz |

NUCLEO-F446RE Arduino 헤더 기준 `D8/PA9 → URT-2 TX`, `D2/PA10 → URT-2 RX`로
연결합니다. Feetech UART 헤더는 신호 이름 기준이므로 `TX-TX`, `RX-RX`입니다.
STM32, URT-2, 외부 서보 전원은 GND를 반드시 공통으로 연결하고, 서보 전원은
STM32나 USB에서 공급하지 않습니다. URT-2를 Type-C로 공급할 때 UART 헤더 VCC는
연결하지 않으며 signal-level switch는 3.3V로 둡니다.

URT-2 Type-C USB와 STM32 UART 헤더가 같은 서보 버스를 동시에 구동하지 않도록
합니다.

## BNO055 배선과 장착 좌표계

NUCLEO-F446RE Arduino 헤더 기준 BNO055 I2C 배선은 다음과 같습니다.

```text
BNO055 VCC/VIN       → NUCLEO 3V3
BNO055 GND           → NUCLEO GND
BNO055 SCL           → D15 / PB8 / I2C1_SCL
BNO055 SDA           → D14 / PB9 / I2C1_SDA
BNO055 COM3/I2C-SEL  → GND (주소 0x28)
```

GND는 URT-2와 IMU가 NUCLEO의 서로 다른 GND 핀을 사용하거나 공통 GND rail에서
분기해도 됩니다. 오른쪽 Arduino 디지털 헤더의 AREF와 D13 사이 GND도 사용할 수
있습니다. COM3는 LOW일 때 `0x28`, HIGH일 때 `0x29`이며 펌웨어는 두 주소를 모두
탐색하지만 검증 구성은 GND에 연결한 `0x28`입니다.

IMU는 인쇄면을 위로 향하게 수평 장착했습니다. 위에서 본 사진에서 로봇 앞쪽을
아래로 놓았을 때 보드 글자가 정상 방향으로 읽히며, 글자의 윗방향은 로봇
뒤쪽입니다. 이는 상하로 뒤집힌 장착이 아닙니다. 실제 `imu on` 시험에서 확인한
부호는 다음과 같습니다.

```text
Pitch +  : 로봇 앞쪽으로 기울어짐
Pitch -  : 로봇 뒤쪽으로 기울어짐
Roll +   : 로봇 오른쪽으로 기울어짐
Roll -   : 로봇 왼쪽으로 기울어짐
Yaw 증가 : 로봇 시점에서 오른쪽으로 회전
Yaw 감소 : 로봇 시점에서 왼쪽으로 회전
```

Yaw는 `0..359.9°` 범위이므로 왼쪽 회전으로 0°를 지나면 359°대로 wrap합니다.
자세 제어에서 Yaw 오차는 `-180..+180°`로 정규화해야 합니다. 현재 장착 방향은
원하는 로봇 좌표계와 일치해 BNO055 axis remap을 적용하지 않습니다.

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

## 콘솔 명령

명령 종료는 `CR`, `LF`, `CRLF`를 모두 지원합니다. 일부 시리얼 터미널은 로컬
에코를 하지 않으므로 입력 글자가 화면에 보이지 않아도 Enter를 누르면 명령이
실행됩니다.

```text
ping ID          한 서보 응답 확인
scan             설정된 ID 1..12 확인
uarttest         USART1 loopback 확인 (URT-2 분리, PA9-PA10 직결)
busprobe ID      Ping 후 USART1 원시 수신 바이트 출력
read ID          위치, 속도, 부하, 전압, 온도, 전류 읽기
move ID RAW      단일 서보 안전 이동; 현재 위치에서 최대 256 tick
targets          stand 목표 raw 위치 확인; 움직이지 않음
profile [S A]    이동 속도(1..3400)와 가속도(0..254) 조회/설정
echo on|off      STM32 입력 echo 켜기/끄기 (부팅 기본 off)
hold             현재 위치를 목표로 설정한 뒤 전체 토크 활성화
stand            12축 목표를 한 번에 SYNC_WRITE하는 stand 이동
trot [C [MS]]    공용 C sim-trot; 1..10회, 주기 600..5000ms (기본 1회/800ms)
relax            전체 토크 해제
imu on            10 Hz IMU 자세 로그 출력 시작
imu off           IMU 자세 로그 출력 중지 (부팅 기본값)
imu status        현재 IMU 로그 설정 확인
balance full      절대 수평 목표와 최대 제한 보정 활성화 (부팅 기본값)
balance normal    보행 시작 자세를 목표로 일반 보정 활성화
balance on        마지막으로 선택한 보정 모드 활성화
balance off       트롯 자세 보정 비활성화
balance status    IMU, 기준 자세, 최대 오차·관절 보정·지연 frame 확인
help             도움말
```

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

BNO055가 정상 초기화되면 `balance full`이 부팅 기본값이며 다음 `trot`부터
Roll/Pitch를 50Hz로 읽습니다. full 모드는 IMU가 나타내는 절대 Roll/Pitch `0°`를
목표로 몸체 수평을 유지합니다. 보드와 몸체 사이의 기계적 장착 오차가 있으면
`robot_config.h`의 `ROBOT_IMU_LEVEL_*_TENTHS`를 0.1° 단위로 보정할 수 있습니다.
`balance normal`은 시험 시작 시의 자세를 기준으로 유지합니다. 확인된 기체
좌표계는 앞쪽이 내려가면 Pitch `+`, 오른쪽이 내려가면 Roll `+`입니다. Yaw는
수평 유지에 사용하지 않습니다.

기본 정책은 IMU 필수입니다. BNO055 탐색 또는 초기화가 실패하면 `trot`은
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

명령 실행 중 콘솔은 blocking되므로 소프트웨어 명령으로 중간 정지할 수 없습니다.
첫 시험은 로봇을 지지대에 고정하고 1회만 실행하며 비상 전원 차단을 준비합니다.

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
- 20ms 목표 갱신, 500ms 이하 진폭 ramp, 실제 위치 step barrier 구현
- BNO055 `0x28`, CHIP_ID `0xA0`, NDOF 초기화 확인
- IMU 연속 로그 기본 OFF 및 `imu on|off|status` 추가
- 콘솔 명령 종료를 `CR`, `LF`, `CRLF` 모두 지원
- CubeMX 재생성으로 사라진 USART2 IRQ 복구
- CubeMX 재생성으로 115200이 된 USART1을 1Mbps로 복구하고 runtime guard 추가
- 생성 코드와 USER CODE에 중복된 USART2 IRQ/NVIC 정의 제거
- GNU Arm GCC 14.3 대상 컴파일과 Servo tool 시험 58개 통과

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
- `gait_policy.h`: MuJoCo/STM32 공용 trot 궤적, IK, IMU 균형 정책
- `robot_config.*`: 12관절 ID, center, direction, stand 목표
- `robot.*`: 공용 정책의 서보 변환, step barrier, hold/stand/relax
- `app_console.*`: USART2 인터럽트 기반 진단 콘솔

관절 보정값은 `tools/servo_tool/config/joints.json`에서 옮겼습니다. 기구 조립이나
서보 ID가 달라지면 `robot_config.c`를 먼저 갱신해야 합니다.
