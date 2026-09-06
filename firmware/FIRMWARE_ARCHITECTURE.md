# Spot OMG 펌웨어 및 무선 업데이트 아키텍처

이 문서는 Spot OMG 로봇에서 호스트, ESP32 브리지, STM32 실시간 제어기, IMU와
12개 서보가 어떻게 연결되는지와 ESP32/STM32 펌웨어를 BLE로 업데이트하는 전체
구조를 설명한다. 설명은 현재 저장소의 실제 구현을 기준으로 한다.

## 1. 현재 하드웨어와 빌드 타깃

현재 PlatformIO 설정의 활성 타깃은 다음과 같다.

```ini
[env:esp32dev]
platform = espressif32@6.12.0
board = esp32dev
framework = arduino
```

보드 변경 이력은 **초기 ESP32-C3 Mini에서 현재 ESP32-WROOM 계열로 교체**된 것으로
확인되었다. 따라서 `firmware/esp32-c3-mini/`는 과거 하드웨어에서 이어진 디렉터리
이름이고, 현재 빌드는 **ESP32 Dev Module/ESP32-WROOM 계열**을 대상으로 한다.
ESP32 UART 핀도 현재 소스의 `GPIO16 RX`, `GPIO17 TX`를 기준으로 한다. 다시 C3를
사용하려면 PlatformIO board, CPU architecture, UART 핀과 파티션을 모두 재검증해야
한다.

현재 ESP32 브리지 소스는 BLE GATT와 STM32 USART3 사이를 연결한다. `spotctl`에
TCP transport가 남아 있지만 이 `.ino`는 Wi-Fi나 TCP server를 시작하지 않는다.
TCP 경로는 별도의 호환 브리지용이며 현재 BLE 펌웨어의 기능으로 간주하면 안 된다.

## 2. 전체 구성

```text
macOS/Windows host
  spotctl + Bleak
       │
       │ BLE GATT (Nordic UART 형태, ATT payload 180 bytes)
       ▼
ESP32-WROOM bridge
  - BLE 광고/연결 복구
  - STM32 console byte bridge
  - ESP32 A/B self OTA
  - STM32 image SPIFFS staging
       │
       │ UART 115200 8-N-1
       │ ESP TX17 ───────> STM32 PC11 / USART3_RX
       │ ESP RX16 <─────── STM32 PC10 / USART3_TX
       ▼
STM32F446RE
  immutable OTA bootloader @ 0x08000000
  robot application       @ 0x08010000
       │
       ├── USART2 115200 ── ST-LINK VCP debug console
       ├── I2C1/SPI1 ────── BNO055/BNO086 IMU
       └── USART1 1 Mbps ── URT-2 ── half-duplex servo bus
                                      │
                                      ├── STS3215 ×8: J1/J3
                                      └── STS3250 ×4: J2
```

ESP32와 STM32의 UART는 TX와 RX를 교차 연결하고 GND를 공통으로 묶어야 한다.
STM32 USART3에는 hardware flow control이 없으므로 BLE 쪽의 명시적 ACK가 펌웨어
전송 속도를 제한한다.

## 3. 관절과 서보 구성

로봇은 다리 네 개에 J1/J2/J3 세 관절씩, 총 12축을 사용한다.

| Leg | J1 | J2 | J3 |
|---|---:|---:|---:|
| FL (front-left) | ID 1, STS3215 | ID 2, STS3250 | ID 3, STS3215 |
| FR (front-right) | ID 4, STS3215 | ID 5, STS3250 | ID 6, STS3215 |
| RL (rear-left) | ID 7, STS3215 | ID 8, STS3250 | ID 9, STS3215 |
| RR (rear-right) | ID 10, STS3215 | ID 11, STS3250 | ID 12, STS3215 |

J2는 J1/J3와 다른 모터를 사용하므로 속도·가속도·lag 판정 한계가 다르다. 이 차이는
STM32의 `motor_capability.h`, actuator limiter와 gait diagnostics에서 처리한다.
ESP32는 관절 제어에 관여하지 않고 byte transport와 업데이트만 담당한다.

## 4. 전원 구조와 콜드 부팅

서보 전원은 별도 12V 계통에서 공급하며 STM32나 USB에서 공급하지 않는다. STM32,
ESP32, URT-2와 서보 전원의 GND는 공통이어야 한다.

현재 기체에서 ESP32 전원은 STM32/NUCLEO 쪽에서 공급된다고 확인되었지만, 실제로
`5V/VIN`을 쓰는지 `3V3`을 쓰는지는 아직 문서화되지 않았다. 결선을 확인한 뒤 아래
항목을 확정해야 한다.

```text
TODO: NUCLEO의 실제 공급 핀 = 5V / 3V3 중 확인
TODO: ESP32의 실제 입력 핀 = VIN/5V / 3V3 중 확인
```

권장 구성은 충분한 전류를 낼 수 있는 안정된 5V rail을 ESP32의 5V/VIN에 공급하고
GND를 공통으로 하는 것이다. NUCLEO 3V3 regulator의 여유 전류가 불명확한 상태에서
ESP32 3V3에 직접 공급하면 BLE 송신 순간 brownout이 날 수 있다. GPIO 핀으로 ESP32
전원을 공급하면 안 된다.

현재 ESP32 firmware의 콜드 부팅 순서는 다음과 같다.

1. CPU reset 후 USB serial을 115200으로 초기화한다.
2. 전원 rail이 안정될 시간을 주기 위해 2초 기다린다.
3. reset reason을 `power-on`, `brownout`, `watchdog` 등으로 출력한다.
4. STM32 UART와 SPIFFS를 초기화한다.
5. BLE GATT service를 만들고 광고를 시작한다.
6. 광고 시작 성공 이벤트가 오지 않으면 5초 간격으로 재시도한다.
7. 6회 연속으로 광고를 복구하지 못하면 ESP32를 software reset한다.
8. BLE 연결이 끊기면 500ms 뒤 광고를 다시 시작한다.

2초 대기는 CPU가 정상적으로 실행을 시작한 경우에만 효과가 있다. 전압 상승이 너무
느려 EN/reset 회로가 정상 threshold를 만들지 못하거나 brownout loop에 빠지면 코드가
실행되지 않으므로 bulk capacitor 또는 ESP32 EN RC delay 같은 하드웨어 보완이 필요하다.

## 5. BLE GATT 인터페이스

장치 이름과 UUID는 다음과 같다.

| Item | Value |
|---|---|
| Device name | `SpotOMG-Bridge` |
| Service | `6e400001-b5a3-f393-e0a9-e50e24dcca9e` |
| Host → ESP RX | `6e400002-b5a3-f393-e0a9-e50e24dcca9e` |
| ESP → Host TX | `6e400003-b5a3-f393-e0a9-e50e24dcca9e` |
| Requested MTU | 185 |
| Application chunk | 최대 180 bytes |

RX characteristic는 `WRITE`와 `WRITE_NR`, TX characteristic는 `NOTIFY`를 사용한다.
일반 명령은 ESP32가 내용을 해석하지 않고 STM32 USART3으로 전달한다. STM32의 응답은
ESP32가 최대 180바이트로 잘라 TX notification으로 보낸다.

OTA 제어 header만 ESP32가 가로챈다.

```text
$ESPOTA BEGIN ...     ESP32 self OTA
$STM32OTA BEGIN ...   STM32 application OTA
그 외 모든 byte      STM32 text console로 전달
```

일반 UART→BLE notification에는 end-to-end flow control이나 ACK가 없다. 긴 gait
diagnostics를 한 번에 출력하면 BLE notification queue가 넘쳐 마지막 줄 또는 `# `
prompt가 유실될 수 있고, 이때 동작은 끝났어도 `spotctl`이 timeout으로 표시할 수 있다.
두 OTA 경로는 각 chunk마다 ACK를 기다리므로 이 문제를 피한다.

## 6. ESP32 플래시 배치

현재 `esp32dev` 기본 4MiB partition table은 다음과 같다.

| Partition | Offset | Size | Purpose |
|---|---:|---:|---|
| `nvs` | `0x009000` | 20KiB | NVS |
| `otadata` | `0x00E000` | 8KiB | 활성 OTA slot 선택 |
| `app0` | `0x010000` | 1,280KiB | ESP32 application slot A |
| `app1` | `0x150000` | 1,280KiB | ESP32 application slot B |
| `spiffs` | `0x290000` | 1,408KiB | STM32 image staging |
| `coredump` | `0x3F0000` | 64KiB | crash dump |

ESP32 self OTA는 현재 실행 중이지 않은 app slot에 새 이미지를 쓴다. STM32 이미지는
app slot이 아니라 SPIFFS에 저장하므로 두 업데이트 영역은 겹치지 않는다.

2026-09-06 기준 self OTA가 포함된 reference build는 다음과 같다. 이 값은 코드가
바뀌면 달라지므로 진단용 기준일 뿐 고정 규격이 아니다.

```text
image:  .pio/build/esp32dev/firmware.bin
file:   1,183,456 bytes
usage:  1,176,877 bytes / 1,310,720 bytes (89.8%)
sha256: 0ef322fb6a12038a23a29c1b9be4ec5dfc6c463b5446320d184b8f54a69c07bb
```

## 7. ESP32 BLE self OTA

### 7.1 사전 검증

호스트는 전송 전에 다음을 확인한다.

- 파일 확장자가 `.bin`인지
- 크기가 36바이트 이상, 1,280KiB 이하인지
- byte 0이 ESP image magic `0xE9`인지
- offset 32의 application descriptor magic이 `0xABCD5432`인지
- 전체 파일의 SHA-256

descriptor 검사는 `bootloader.bin`을 application OTA slot에 잘못 쓰는 실수를 막는다.
업로드 대상은 PlatformIO가 생성한 `.pio/build/esp32dev/firmware.bin`이어야 한다.

### 7.2 Wire protocol

호스트가 보내는 header는 한 BLE packet에 들어간다.

```text
$ESPOTA BEGIN <decimal-size> <64-char-sha256>\n
```

ESP32 응답과 이후 전송은 다음과 같다.

```text
ESP → host: $ESPOTA READY\n
host → ESP: binary chunk, 36..180 bytes for the first chunk
ESP → host: $ESPOTA ACK <cumulative-bytes>\n
...
host → ESP: final binary chunk
ESP → host: $ESPOTA OK\n
```

오류는 `$ESPOTA ERROR <reason>\n`으로 반환한다. 예시는 `busy`, `invalid-image`,
`not-application-image`, `sha256`, `timeout`과 ESP32 Update library의 오류 문자열이다.

### 7.3 장치 내부 처리

1. `$ESPOTA BEGIN`을 받으면 STM32 OTA가 실행 중인지 검사한다.
2. Arduino `Update.begin(size, U_FLASH)`가 비활성 app partition을 선택한다.
3. SHA-256 context를 시작하고 `READY`를 보낸다.
4. 각 BLE packet을 inactive partition에 쓰고 누적 byte 수를 ACK한다.
5. 첫 packet에서 ESP image/application descriptor magic을 다시 검사한다.
6. 전송 중 STM32 UART 출력은 버려 OTA ACK stream과 섞이지 않게 한다.
7. 15초 동안 다음 chunk가 없으면 업데이트를 abort한다.
8. 마지막 chunk 뒤 전체 SHA-256을 비교한다.
9. `Update.end()`가 ESP image를 검증하고 `otadata`의 다음 boot slot을 바꾼다.
10. `OK` notification 전달을 위해 1.5초 기다린 뒤 `esp_restart()`를 호출한다.

연결이 중간에 끊기거나 SHA 검증이 실패하면 `Update.abort()`를 호출한다. ESP Update
library는 새 image의 첫 magic block을 완료 전까지 bootable 상태로 만들지 않으므로
불완전한 inactive slot 대신 현재 slot이 계속 부팅된다.

### 7.4 제한과 복구

- self OTA 기능이 없는 기존 ESP32에는 BLE로 이 기능을 설치할 수 없다. 최초 한 번은
  USB serial로 새 bridge firmware를 설치해야 한다.
- 새 firmware가 정상 검증됐지만 실행 직후 자체적으로 crash하는 경우를 위한 automatic
  rollback은 현재 활성화되지 않았다. 이 경우 USB로 known-good image를 복구한다.
- SHA-256은 전송 오류를 검출하지만 서명 검증은 아니다. 현재 BLE service에는 OTA
  인증이나 signed image 검증이 없어 신뢰할 수 없는 주변 환경에 그대로 노출하면 안 된다.

## 8. STM32 플래시 배치

STM32F446RE의 512KiB flash는 다음처럼 사용한다.

| Sector | Address range | Size | Purpose |
|---|---|---:|---|
| 0 | `0x08000000..0x08003FFF` | 16KiB | immutable bootloader |
| 1 | `0x08004000..0x08007FFF` | 16KiB | immutable bootloader reserve |
| 2 | `0x08008000..0x0800BFFF` | 16KiB | immutable bootloader reserve |
| 3 | `0x0800C000..0x0800FFFF` | 16KiB | application metadata |
| 4 | `0x08010000..0x0801FFFF` | 64KiB | relocated application |
| 5 | `0x08020000..0x0803FFFF` | 128KiB | relocated application |
| 6 | `0x08040000..0x0805FFFF` | 128KiB | relocated application |
| 7 | `0x08060000..0x0807FFFF` | 128KiB | IMU calibration record |

application 최대 크기는 sector 4–6의 320KiB이다. application linker origin은
`0x08010000`, VTOR은 `FLASH_BASE + 0x00010000`이어야 한다.

sector 3 metadata의 첫 16바이트는 다음 의미를 가진다.

```text
offset +0x00: magic    = 0x53504657
offset +0x04: size     = application byte size
offset +0x08: crc32    = application CRC-32
offset +0x0C: version  = 1
```

metadata magic은 image와 CRC 검증이 모두 끝난 뒤 **마지막으로** 기록한다. 따라서
erase/program 중 전원이 꺼지면 bootloader는 application을 유효하다고 보지 않는다.
sector 7은 bootloader가 지우지 않으므로 IMU calibration이 firmware update와 분리된다.

## 9. STM32 BLE OTA

STM32 OTA는 host→ESP 단계와 ESP→STM32 단계로 나뉜다. ESP32가 전체 파일을 먼저
저장하는 이유는 STM32 flash를 지우기 전에 BLE 전송 무결성을 확정하기 위해서다.

### 9.1 Host → ESP32 staging

호스트는 raw binary의 vector table을 먼저 검사한다.

- initial stack pointer가 `0x20000000..0x20020000` 범위인지
- reset vector의 Thumb bit가 1인지
- reset handler가 `0x08010000..0x08060000` 범위인지
- image가 8바이트 이상, 320KiB 이하인지

프로토콜은 다음과 같다.

```text
host → ESP: $STM32OTA BEGIN <decimal-size> <64-char-sha256>\n
ESP  → host: $STM32OTA READY\n
host → ESP: binary chunk, 최대 180 bytes
ESP  → host: $STM32OTA ACK <cumulative-bytes>\n
...
ESP  → host: $STM32OTA STORED\n
```

ESP32는 `/stm32.tmp`에 쓰면서 SHA-256을 계산한다. 크기와 SHA가 맞으면 기존
`/stm32.bin`을 교체하고 temporary file을 `/stm32.bin`으로 rename한다. 검증 전에는
STM32 flash를 건드리지 않는다.

### 9.2 ESP32 → STM32 bootloader

1. ESP32가 STM32 USART3으로 `fwupdate\n`을 보낸다.
2. STM32 application은 12개 서보에 torque-off를 시도한다.
3. SRAM `0x2001FFF0`에 request magic `0x53504F54`를 기록한다.
4. STM32가 reset되면 bootloader가 request를 소비하고 application jump를 생략한다.
5. bootloader가 `SPOTBOOT 1\n`을 보낸다.
6. ESP32는 staged image의 CRC-32를 계산한다.
7. ESP32가 `SPOTFW 1 <size> <8-char-crc32>\n`을 보낸다.
8. bootloader가 sector 3–6을 erase하고 `SPOTBOOT READY\n`을 보낸다.
9. ESP32가 image를 1,024바이트 block으로 USART3에 전송한다.
10. bootloader는 매 1,024바이트와 마지막 block에서 누적 byte ACK를 보낸다.
11. bootloader가 수신 CRC, flash read-back CRC와 vector table을 검증한다.
12. size, CRC, version을 기록하고 metadata magic을 마지막으로 기록한다.
13. `SPOTBOOT OK\n`을 보낸 뒤 STM32를 reset한다.
14. 다음 boot에서 metadata/CRC가 유효하면 application으로 jump한다.

내부 UART protocol은 다음과 같다.

```text
STM32 → ESP: SPOTBOOT 1\n
ESP → STM32: SPOTFW 1 <decimal-size> <8-char-crc32>\n
STM32 → ESP: SPOTBOOT READY\n
ESP → STM32: 1,024-byte binary blocks
STM32 → ESP: SPOTBOOT ACK <cumulative-bytes>\n
STM32 → ESP: SPOTBOOT OK\n
```

application으로 jump하기 직전에 bootloader는 USART3을 끄고 VTOR/MSP를 application
값으로 바꾼다. MSP 설정 과정에는 interrupt를 잠시 막지만 application Reset_Handler를
호출하기 전 `PRIMASK`를 다시 enable한다. 이 순서가 없으면 SysTick interrupt가 멈춰
application 첫 `HAL_Delay()`에서 영구 정지한다.

### 9.3 전원 차단 시 동작

| Failure point | Result | Recovery |
|---|---|---|
| BLE staging 중 | 실행 중 STM32 image 유지 | 명령 재실행 |
| SHA 불일치 | STM32 flash 미변경 | 올바른 image 재전송 |
| sector erase/program 중 | metadata invalid, bootloader 대기 | 같은 명령 재실행 |
| metadata magic 기록 후 | 새 image committed | 다음 전원/reset에서 새 app boot |
| sector 7 calibration | OTA가 접근하지 않음 | 보정값 유지 |

STM32가 이미 bootloader 대기 상태여도 ESP32가 먼저 보내는 `fwupdate\n`은 잘못된
header로 처리되고 bootloader가 banner를 다시 출력한다. 이후 정상 `SPOTFW` header를
받아 재시도할 수 있다.

## 10. 정상 명령과 OTA의 동시성

- ESP32 OTA와 STM32 staging은 동시에 시작할 수 없으며 상대 작업이 있으면 `busy`를
  반환한다.
- ESP32 OTA 중에는 STM32 UART console 출력을 drain하여 `$ESPOTA` ACK와 섞이지 않게
  한다.
- STM32 image를 SPIFFS에 저장하는 동안에는 STM32 application이 계속 실행된다.
- STM32 flash 단계가 시작되면 ESP32 loop가 bootloader programming을 완료할 때까지
  해당 작업에 전념한다.
- firmware update는 로봇이 걷는 중에 시작하지 않는다. 평평한 바닥에서 정지시키고
  전원과 BLE 연결을 안정시킨 뒤 실행한다.

## 11. 빌드와 설치

### 11.1 ESP32 bridge

```bash
cd ~/project/spot_omg
pio run -e esp32dev
```

최초 bootstrap 또는 BLE가 복구 불가능할 때 USB로 설치한다.

```bash
pio run -e esp32dev -t upload --upload-port /dev/cu.usbserial-XXXX
```

self OTA 지원 firmware가 한 번 설치된 다음부터는 BLE로 갱신한다.

```bash
spotctl firmware esp32 .pio/build/esp32dev/firmware.bin
```

정상 출력은 `Uploading: 10%`부터 진행되어 마지막에 다음 메시지가 나온다.

```text
ESP32 firmware verified; bridge is restarting
```

재부팅에는 OTA restart delay 1.5초와 power stabilization 2초가 포함되므로 BLE에 다시
나타날 때까지 수 초 기다린다.

### 11.2 STM32 application과 bootloader

STM32 application은 `0x08010000`에 link한 뒤 raw binary를 만든다.

```bash
arm-none-eabi-objcopy -O binary \
  firmware/stm32-learning/Debug/stm32-learning.elf \
  firmware/stm32-learning/Debug/stm32-learning.bin
```

bootloader build:

```bash
cd firmware/stm32-ota-bootloader
make PREFIX=/path/to/arm-none-eabi-
```

최초 factory image는 bootloader, committed metadata와 application을 합친다.

```bash
make factory PREFIX=/path/to/arm-none-eabi-
STM32_Programmer_CLI -c port=SWD mode=UR \
  -w build/spot-factory.bin 0x08000000 -v -rst
```

이후 STM32 application update:

```bash
spotctl firmware stm32 \
  firmware/stm32-learning/Debug/stm32-learning.bin
```

## 12. 운영 점검 절차

### ESP32 self OTA 전

1. 로봇을 평평한 바닥에 정지시킨다.
2. 배터리와 ESP32 공급 전압이 안정적인지 확인한다.
3. `SpotOMG-Bridge`가 BLE scan에 나타나는지 확인한다.
4. `pio run -e esp32dev`가 성공하고 image가 1,280KiB 이하인지 확인한다.
5. 반드시 `.pio/build/esp32dev/firmware.bin`을 선택한다.
6. OTA가 끝나고 bridge가 다시 광고될 때까지 전원을 끄지 않는다.

### STM32 OTA 전

1. 로봇을 넘어지지 않는 자세로 두고 보행을 중지한다.
2. relocated application의 reset vector가 `0x08010000` 영역인지 확인한다.
3. ESP32 SPIFFS와 BLE가 정상인지 확인한다.
4. OTA 중 12V servo rail과 제어기 전원을 유지한다.
5. 완료 후 `spotctl targets`, `spotctl scan`과 control revision을 확인한다.

## 13. 장애 진단

### 전원을 켠 뒤 BLE 장치가 보이지 않음

1. 최소 5초 기다린다. 정상 firmware는 광고를 자동 재시도한다.
2. 약 30초 동안 계속 보이지 않으면 ESP32가 자동 reset되는지 기다린다.
3. ESP32 reset 버튼 뒤 나타나면 power ramp 또는 EN timing 가능성이 높다.
4. USB serial에서 `reset=brownout`인지 확인한다.
5. 실제 공급 핀과 5V/3V3 rail 전압을 측정한다.
6. software가 전혀 시작되지 않으면 bulk capacitor/EN RC 회로를 검토한다.

### ESP32 OTA가 `timeout` 또는 ACK mismatch로 종료됨

- 다른 `spotctl` session을 모두 닫는다.
- BLE 신호가 좋은 가까운 거리에서 다시 실행한다.
- 기본 `--chunk-size 180`을 유지한다.
- 전송이 끊겨도 기존 app slot은 유지되므로 ESP32를 reset하고 재시도한다.

### STM32가 `SPOTBOOT 1`만 출력함

metadata 또는 application CRC가 유효하지 않아 bootloader가 update를 기다리는 상태다.
ESP32 BLE가 살아 있으면 `spotctl firmware stm32 ...bin`을 다시 실행한다. BLE도 없으면
ESP32부터 USB로 복구하고, 필요하면 ST-LINK로 factory image를 다시 쓴다.

### 긴 gait 진단 뒤 `spotctl` timeout

motion 완료 시간과 `OK`가 이미 출력됐다면 servo bus timeout이 아니라 BLE의 일반
notification queue에서 마지막 diagnostics/prompt가 유실된 것일 수 있다. 현재 일반
console stream에는 OTA와 같은 ACK flow control이 없다는 점을 구분해서 진단한다.

## 14. 보안과 향후 개선

현재 구현은 SHA-256/CRC32로 **전송 무결성**을 확인하지만 **image 제작자 신뢰성**은
확인하지 않는다. 다음 개선이 필요하다.

1. BLE bonding 또는 application-level challenge authentication
2. signed manifest와 public-key signature 검증
3. ESP-IDF bootloader rollback 활성화 및 새 image health confirmation
4. 일반 UART→BLE console stream의 sequence/ACK/backpressure
5. ESP32 전원 핀, regulator 용량, bulk capacitor와 EN RC의 회로도 확정
6. `esp32-c3-mini` 디렉터리 이름을 실제 ESP32-WROOM 타깃과 일치하도록 정리

## 15. 구현 파일 안내

| File | Responsibility |
|---|---|
| `platformio.ini` | ESP32 board/framework/build environment |
| `firmware/esp32-c3-mini/esp32_c3_stm32_uart_bridge/esp32_c3_stm32_uart_bridge.ino` | BLE bridge, 광고 복구, ESP32 OTA, STM32 staging/programming |
| `tools/servo_tool/servo/transport.py` | Bleak 기반 synchronous BLE byte transport |
| `tools/servo_tool/servo/cli.py` | `spotctl firmware esp32/stm32`, image 사전 검증과 ACK protocol |
| `firmware/stm32-ota-bootloader/bootloader.c` | STM32 immutable bootloader, UART writer, CRC/vector 검증 |
| `firmware/stm32-ota-bootloader/bootloader.ld` | bootloader 48KiB flash/RAM 배치 |
| `firmware/stm32-ota-bootloader/make_factory_image.py` | factory image와 metadata 생성 |
| `firmware/stm32-learning/STM32F446RETX_FLASH.ld` | relocated application 320KiB 배치 |
| `firmware/stm32-learning/Src/system_stm32f4xx.c` | application VTOR offset `0x10000` |
| `firmware/stm32-learning/Src/app_console.c` | `fwupdate`, torque-off와 boot request/reset |
| `firmware/stm32-learning/Src/main.c` | USART1/2/3, IMU와 console 초기화 |
| `tools/servo_tool/tests/test_spot.py` | ESP32 OTA header/validation/ACK host tests |
