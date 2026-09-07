# SpotOMGController for iOS

SwiftUI와 CoreBluetooth로 `SpotOMG-Bridge`에 직접 연결하는 iPhone 앱입니다.
현재 버전은 기존 STM32 text console 명령을 사용하며, 조이스틱용 realtime binary
protocol과 OTA 화면은 후속 단계로 분리합니다.

## 실행

1. Xcode에서 `SpotOMGController.xcodeproj`를 엽니다.
2. Signing & Capabilities에서 개인 Team을 선택합니다.
3. Bluetooth가 가능한 실제 iPhone을 실행 대상으로 선택합니다.
4. 로봇 전원을 켜고 앱을 실행한 뒤 Bluetooth 접근을 허용합니다.

iOS Simulator에서는 실제 BLE peripheral 연결을 검증할 수 없습니다.

## 현재 기능

- service UUID로 `SpotOMG-Bridge` 검색, 자동 연결과 연결 해제 후 재검색
- 조종 버튼과 같은 화면에서 STM32 console 명령 송신 및 BLE notification 로그 표시
- STM32 `syncstate` snapshot을 이용한 실제 자세·torque·safety·balance 상태 동기화
- Landing/Stand/Stand11/Hold/Recover와 확인 절차가 있는 Relax
- 보수적인 단일-cycle trot4/crab 명령
- targets, scan, gaitdiag, baldiag
- 앱이 비활성화될 때 best-effort `stand` 요청

## 안전 및 다음 단계

현재 앱의 background `stand`는 BLE 연결이 이미 끊긴 경우 전달을 보장하지 않습니다.
실시간 조이스틱을 추가하기 전에 firmware에 heartbeat watchdog과 연결 상실 시 정지,
sequence가 있는 binary control/telemetry characteristic을 먼저 구현해야 합니다.
긴 console diagnostics에는 notification tail 유실 가능성이 있으므로 조종 telemetry로
사용하지 않습니다.
