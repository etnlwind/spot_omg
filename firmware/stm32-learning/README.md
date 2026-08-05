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

STM32 TX는 URT-2의 MCU 입력, STM32 RX는 URT-2의 MCU 출력에 연결합니다.
제품 리비전에 따라 UART 헤더의 TX/RX 표기 방향이 다를 수 있으므로 실제 핀맵을
확인하세요. STM32, URT-2, 외부 서보 전원은 GND만 반드시 공통으로 연결하고,
서보 전원은 STM32나 USB에서 공급하지 않습니다.

URT-2 Type-C USB와 STM32 UART 헤더가 같은 서보 버스를 동시에 구동하지 않도록
합니다.

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
read ID          위치, 속도, 부하, 전압, 온도, 전류 읽기
move ID RAW      단일 서보 안전 이동; 현재 위치에서 최대 256 tick
targets          stand 목표 raw 위치 확인; 움직이지 않음
hold             현재 위치를 목표로 설정한 뒤 전체 토크 활성화
stand            2초 동안 12축 SYNC_WRITE stand 이동
relax            전체 토크 해제
imu on            10 Hz IMU 자세 로그 출력 시작
imu off           IMU 자세 로그 출력 중지 (부팅 기본값)
imu status        현재 IMU 로그 설정 확인
help             도움말
```

권장 첫 시험 순서는 다음과 같습니다.

```text
ping 1
read 1
move 1 PRESENT_POSITION_PLUS_SMALL_DELTA
read 1
relax
```

`stand`는 12개 ID가 모두 응답해야 시작합니다. 먼저 현재 위치를 목표 위치로
설정한 뒤 토크를 켜므로 오래된 Goal Position으로 튀는 동작을 방지합니다.

## 2026-08-05~06 개발 및 점검 결과

- Feetech packet, URT-2 bus, STS3215 register API와 12축 Sync Write 구현
- `joints.json`의 ID·center·direction을 STM32 설정으로 이식
- 부팅 시 torque OFF, 안전 단일 이동, current-position hold, 2초 stand ramp 구현
- BNO055 `0x28`, CHIP_ID `0xA0`, NDOF 초기화 확인
- IMU 연속 로그 기본 OFF 및 `imu on|off|status` 추가
- 콘솔 명령 종료를 `CR`, `LF`, `CRLF` 모두 지원
- CubeMX 재생성으로 사라진 USART2 IRQ 복구
- CubeMX 재생성으로 115200이 된 USART1을 1Mbps로 복구하고 runtime guard 추가
- 생성 코드와 USER CODE에 중복된 USART2 IRQ/NVIC 정의 제거
- GNU Arm GCC 14.3 대상 컴파일과 기존 Servo tool 시험 55개 통과

수정 전 STM32 `scan`에서는 ID 1~12가 모두 timeout이었지만 당시 USART1이
115200으로 잘못 생성된 상태였습니다. 따라서 수정 펌웨어를 Flash한 뒤 Ping을
재확인해야 하며, 아직 STM32 → URT-2 → STS3215 실기 성공으로 기록하지 않습니다.

상세한 날짜별 기록은
[`tools/servo_tool/HARDWARE_TEST_LOG.md`](../../tools/servo_tool/HARDWARE_TEST_LOG.md)를
참조합니다.

## 코드 구조

- `feetech_protocol.*`: 패킷, checksum, little-endian 변환
- `servo_bus.*`: UART request/status 및 URT-2 송수신 전환
- `sts3215.*`: STS3215 레지스터 API와 SYNC_WRITE
- `robot_config.*`: 12관절 ID, center, direction, stand 목표
- `robot.*`: require-all, hold, relax, safe move, stand
- `app_console.*`: USART2 인터럽트 기반 진단 콘솔

관절 보정값은 `tools/servo_tool/config/joints.json`에서 옮겼습니다. 기구 조립이나
서보 ID가 달라지면 `robot_config.c`를 먼저 갱신해야 합니다.
