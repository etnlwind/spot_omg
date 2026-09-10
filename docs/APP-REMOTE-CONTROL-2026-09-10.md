# 앱 연결 원격 제어 — 2026-09-10

## 경로

Mac의 페어링된 CoreDevice USB/개발 Wi-Fi 연결로 앱 Documents/RemoteControl에 요청을 전달한다.
앱 V0.5.0 (34)가 0.3초마다 요청을 읽고 UUID별 응답 파일을 쓴다.
`spotomg://remote-control`은 읽기를 깨우기만 하며 URL 자체에 로봇 명령은 없다.
새 네트워크 수신 포트나 ESP32 펌웨어 변경은 필요 없다.
요청은 UUID, 발행/만료 시간, 명령/대상 allowlist를 검증하고 최근 64개 요청의 재실행을 막는다.
Mac 측에서는 파일 잠금으로 요청을 직렬화하고 UUID가 같은 응답만 인정한다.

`spotctl app pair --device UJIN17` 성공 시 기기를 저장한다. `app status/connect/disconnect`를 제공한다.
로봇 BLE를 사용하는 spotctl 명령은 등록된 앱 상태를 조회하고 연결을 해제한 후 작업한다.
성공한 작업에 한해, 원래 앱이 로봇에 연결된 경우에만 재연결한다.
자동 선택이 BLE인 경우도 포함하고 TCP/직접 시리얼 작업에는 적용하지 않는다.
`--no-app-control`로 자동 처리를 생략할 수 있다.

## 정지와 모터 해제

원격 연결 해제 중에는 일반 조작을 차단한다. 진행 중 동작은 기존 Ctrl-C로 중지하고
`STOPPED:`, `$SPOTDRIVE stopped`, `ERROR: motion aborted` 확인 후 연결을 해제한다.
BLE 해제는 CoreBluetooth의 disconnect 완료를 기다린다. 앱을 강제 종료하지 않는다.
연결 해제 자체는 Relax가 아니다.

일반 앱 Relax는 Landing 완료 후 토크를 해제한다. Stop, 오류, 연결 종료 또는 75초 완료 확인
시간 초과 시 예약된 토크 해제를 취소한다. Stow에서는 기존 Landing 전용 조작 제한을 유지한다.

**사용자 최종 지시: 펌웨어 업데이트가 우선이다.** STM32/ESP32 BLE 업데이트는 Landing을
먼저 시도하되, 실패·중지·응답 시간 초과 시 경고 후 업데이트를 계속한다.
Landing 완료를 업데이트 필수 조건으로 사용하지 않는다. 긴급 보호/고장 경로의 토크 해제는 변경하지 않았다.

## 검증

- iOS Debug 기기 빌드 성공, 실제 UJIN17에 V0.5.0 (34) 설치.
- Swift 호스트 검사: Landing 전 Relax 미전송, 실패 시 미전송, 완료 후 전송,
  원격 해제 시 Stop ACK 전 연결 유지, 일반 명령 차단, ACK 후 해제.
- Python 도구 전체 131개 검사와 31개 하위 사례 통과. 회귀: BLE 작업 전 앱 해제/성공 후 복구, 실패 시 복구 금지,
  원래 해제된 앱은 해제 유지, Landing 실패·중지·시간 초과 시 업데이트 차단 금지.
- 실제 원격 status 성공: V36, robot, ready=true 확인. `pair` 등록 성공.
- 자동 disconnect는 Ctrl-C 송신 후 응답이 없어 시간 초과했다. V37 설치 시에는 앱 프로세스를 종료한 뒤 BLE를 확보했다. 전체 자동 해제/복구의 성공 검증은 남아 있다.

## 제한

페어링된 개발 기기 접근과 잠금 해제/앱 실행이 필요하다. Tailscale만 연결된 임의의 외부 폰을
인터넷에서 제어하는 기능은 아니다. 기기 잠금이나 연결 불가를 우회하지 않는다.
