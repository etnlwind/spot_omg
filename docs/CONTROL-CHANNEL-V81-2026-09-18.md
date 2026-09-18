# V81: 제어 응답과 콘솔 로그 분리

사용자 요청: 정지 후 컨트롤러가 잠기는 문제를 해결하되 상세 로그는 계속 출력하고,
제어 통신을 콘솔과 별개로 처리한다. V80 보행 수식·서보 설정은 바꾸지 않았다.

## 동작

- Windows 0.1.4, STM32 `attitudepd-v4-v81`, 새 ESP32 브리지가 함께 필요하다.
- 기존 BLE 콘솔 RX/TX UUID 끝자리 `002/003`은 그대로 유지한다.
- 새 제어 RX `005`는 write-with-response, 제어 TX `006`은 indication을 사용한다.
- 초기 콘솔 `syncstate`에서 `controlv1` capability와 두 특성을 확인한 뒤,
  이후 앱 명령/상태 조회/보행/정지/완료 확인은 제어 채널로 보낸다.
- RPC 요청은 `@C <번호> <명령>\n`, 응답은 `RS <번호> DATA <응답> US`와
  `RS <번호> DONE <빈 문자열> US`이다. RS/US는 바이트 `0x1e/0x1f`이다.
  `@D`, `@S`, Ctrl+C는 같은 전용 제어 RX에서 기존 실시간 수신 경로로 처리한다.
- 콘솔 `#`나 이전 명령의 DONE으로 현재 명령을 끝내지 않는다. 명령 번호가
  일치하는 DONE만 내부 완료 이벤트로 사용한다. 사용자 명령은 ASCII 95바이트 이내다.
- 보행 종료 후 기존 상세 진단을 모두 생성하고 대기열에 보존한다. 현재 상태를
  별도 제어 응답으로 보낸 뒤 DONE을 전송한다. Windows는 이 상태를 받았다면
  불필요한 추가 syncstate/즉시 전압 조회 없이 새 조작을 허용한다.

물리적인 STM32↔ESP32 UART 배선은 하나를 공유한다. 논리 채널/수신 대기열/
완료 판정과 BLE 특성을 분리한 것이며 독립된 UART 배선을 추가한 것은 아니다.
STM32는 로그를 최대 32바이트씩, 전송 오류 시에도 5ms 이내로 처리한다.
보행 중에는 기존 프레임 여유 시간 검사 안에서만 서비스한다. ESP32는 프레임을
분리하고 콘솔 알림 전송량을 제한해 제어 indication을 로그 대기열 뒤에 넣지 않는다.

로그 대기열은 STM32 각 콘솔 8KiB, ESP32 16KiB로 유한하다. 정상 진단은 그대로
출력하지만 과부하에서는 제어를 막지 않고 `LOG overflow: ... bytes lost`를 명시한다.
무제한 로그 무손실을 보장하는 설계는 아니다. Windows에서도 로그 큐/미완성 줄의
오류가 제어 연결을 끊지 않도록 했으며, 제어 채널 자체의 실패는 기존 보호를 유지한다.

## 호환성과 한계

구형 펌웨어/브리지는 기존 콘솔 방식으로 연결한다. Windows 상태에 `전용 제어 채널`
문구가 있어야 새 방식으로 연결된 것이다. GATT 캐시로 신규 특성이 가려지지 않도록
Windows BLE 서비스 검색은 캐시를 쓰지 않는다.

iPhone 기존 앱은 기존 콘솔 특성으로 계속 연결할 수 있으나, 이번 전용 제어
채널의 iOS 클라이언트 구현은 포함하지 않았다. iOS 빌드 51의 타이머 수정만으로
이 방식이 적용되었다고 해석하지 않는다.

## 검증

- Windows/STM32/ESP32 호스트 검사 107개 통과.
- 최종 전송 시간 제한 및 파서 수정 후 관련 검사 22개 추가 통과.
- 실제 생산 C/C++ 함수로 명령 수신, ISR과 송신 분리, 중복 요청 무시, UART 오류,
  Ctrl+C로 대기 요청 취소, 5KiB 로그 앞서 다음 명령 완료, 로그 overflow 중
  제어 응답 보존을 검사했다. 서보/물리는 이 호스트 시험의 대상이 아니다.
- 앱 통신 루프 시험에서 정지 후 ID11에서 끊긴 로그를 남겨도 0.5초 이내에
  조작 가능 상태로 복귀했다. 이는 mock 측정 기준이며 실제 무선 지연 수치가 아니다.
- STM32/ESP32/Windows 패키지 빌드 통과. 실기 설치·조회 결과는
  `artifacts/control-channel-v81/`에 별도 기록한다.

## 설치 절차

`scripts/hardware/install_attitudepd_v4.py`는 기존 기본값을 보존하고 명시적인
`--image`, `--sha256`, `--from-revision`, `--to-revision`을 추가했다.
V81에서는 `--from-revision attitudepd-v4-v80 --to-revision attitudepd-v4-v81`을 사용한다.
Landing 실제 도착 → 12개 토크 OFF 확인 → STM32 OTA → Landing 재검증/설정 보존
→ 토크 OFF → ESP32 OTA → 전용 채널 읽기 전용 검증 순서다.

보행 재개 시험은 설치/상태 조회와 구분한다. 과거의 완료된 보행 로그 재출력을
새 보행 시험으로 세지 않는다.

## 설치 진행 및 복구 상태

STM32 V81 설치와 설치 후 Landing 도착(오차 26틱), 12개 토크 OFF,
서보 영구 설정 보존을 확인했다. 증거는
`artifacts/control-channel-v81/hardware-install/recheck-01/verify.json`이다.
Windows 0.1.4 설치 및 패키지 시작 검사도 완료했다.

ESP32 OTA는 60% 진행 표시 후 ACK 시간 초과로 실패했다
(`bridge-install.json`, `bridge-ota.txt`). 중단 원인은 미확정이다.
재연결에서는 V81·torque=off·safety=ok·fault_code=0, 12개 서보 정지/하드웨어 오류
없음을 확인했다. 캐시 없는 GATT 검색 결과 신규 제어 UUID 005/006은 없다
(`bridge-recovery-discovery.json`). 기존 브리지 통신은 가능하지만 새 제어 채널은
아직 설치되지 않았다. USB 연결을 요청하여 유선 복구를 준비 중이다.
전용 채널 실기 조회와 정지 후 재조작/보행 검증은 미완료다.

### 무선 재시도 완료

이후 사용자 요청으로 BLE OTA를 다시 수행했다. `bridge-retry-02/prepare.json`에
Landing 실제 도착(26틱)과 12개 토크 OFF 재검증을 기록하고 같은 해시의 이미지를
전송했다. `bridge-retry-02/install.json` 및 `ota.txt`에 100% 전송과 무결성 검증
성공을 기록했다. 첫 실패 원인은 이번 성공만으로 확정하지 않는다.

재시작 후 `hardware-control-query.json`의 실기 읽기 전용 검증이 성공했다.
새 GATT 제어 특성으로 help DONE 0.046초, syncstate DATA/DONE 0.266초,
gaitdiag DONE 0.109초를 관측했다. help와 syncstate 완료 뒤에도 콘솔 로그가
계속 도착하여 로그 소진을 기다리지 않고 다음 조회를 마치는 동작을 확인했다.
이는 이번 1회 측정이며 보행 정지 응답 시간이나 최대 지연 보장은 아니다.
현재 Landing·토크 OFF·safety OK이고, 실제 보행 정지 후 재조작 시험은 미실시다.
