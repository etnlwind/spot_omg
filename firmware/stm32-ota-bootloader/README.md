# Spot OMG STM32 BLE update bootloader

ESP32 bridge, BLE GATT, ESP32 self OTA와 STM32 OTA를 함께 설명한 전체 설계 문서는
[`../FIRMWARE_ARCHITECTURE.md`](../FIRMWARE_ARCHITECTURE.md)를 참고하십시오.

This immutable bootloader occupies STM32F446 sectors 0–2. Sector 3 contains
the committed application size and CRC32. The application is linked at
`0x08010000` (sector 4), may use sectors 4–6, and sector 7 remains reserved for
the existing IMU calibration record.

The ESP32 receives the complete application over BLE, stores it in SPIFFS and
checks SHA-256 before asking the STM32 application to reboot. The bootloader
then erases only sectors 3–6, writes the image over USART3, verifies CRC32 and
the vector table, and writes the metadata magic last. A power loss before that
last write leaves the image invalid and the bootloader waiting for a retry.

## Build

The compiler prefix can be omitted when `arm-none-eabi-gcc` is on `PATH`.

```bash
cd firmware/stm32-ota-bootloader
make PREFIX=/path/to/arm-none-eabi-
```

Build the normal `stm32-learning` project after its linker relocation, then
create the raw OTA image:

```bash
arm-none-eabi-objcopy -O binary \
  firmware/stm32-learning/Debug/stm32-learning.elf \
  firmware/stm32-learning/Debug/stm32-learning.bin
```

## One-time installation

This bootstrap step requires USB/SWD. Install the updated ESP32 bridge first
(replace the port with the ESP32 USB serial device):

```bash
pio run -e esp32dev -t upload --upload-port /dev/cu.usbserial-XXXX
```

After building the relocated STM32 application, create one factory image that
contains the bootloader, committed metadata and application:

```bash
cd firmware/stm32-ota-bootloader
make factory PREFIX=/path/to/arm-none-eabi-
cd ../..
```

Write that combined image at the physical Flash base:

```bash
STM32_Programmer_CLI -c port=SWD mode=UR \
  -w firmware/stm32-ota-bootloader/build/spot-factory.bin 0x08000000 -v -rst
```

The combined image boots the relocated app immediately. Later application
updates use BLE:

```bash
spotctl firmware stm32 \
  firmware/stm32-learning/Debug/stm32-learning.bin
```

All later STM32 application updates use the same `spotctl` command and need no
ST-LINK. Do not flash the relocated application without the bootloader: the
MCU reset vector remains at `0x08000000`.

## Recovery and security

- Power loss while BLE is staging is harmless; the running STM32 image is not
  touched.
- Power loss after STM32 erase leaves metadata invalid. Power the robot again
  and repeat the `spotctl firmware stm32` command; ESP32 retains the staged
  image until it is replaced.
- Sector 7 is never erased by this bootloader.
- Version 1 provides transport integrity (SHA-256 on ESP32 and CRC32 on STM32),
  but not image authenticity. BLE pairing/authentication or signed manifests
  must be added before exposing the updater to untrusted nearby clients.
