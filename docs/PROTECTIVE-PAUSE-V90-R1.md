# 보호정지를 사용자 재시도 가능한 일시정지로 변경 (2026-09-21)

사용자 요청: 보호정지 후 새 조작을 반영하고, 전압 경고처럼 문제를 화면에
표시한다. 재개 여부는 사용자가 결정하며 이전 입력으로 자동 재개하지 않는다.

## 원인과 변경

기존 `robot_prepare_new_command()`는 fault latch를 해제했지만 자세 전환은
절대 기울기 15°, 공용 보행은 12° 검사에 다시 걸렸다. 기울어진 채 정지하면
Stand/Landing/새 drive가 실제 이동 전에 취소되는 문제가 있었다.

- 기울기 정지 후 **새 foreground 명령**은 서보 건강 상태와 새 IMU 측정값을
  확인하고 현재 기울기를 재시도 기준으로 기록한다. heartbeat는 기준을 바꾸지 않는다.
- 재시도 중 같은 방향의 허용 범위는 시작 기울기 + 기존 한계
  (자세 15°, 공용 보행 12°)다. 반대 방향은 기존 한계를 유지한다.
  두 축 모두 ±12° 안으로 돌아오면 기존 절대 기준으로 복귀한다.
- 실제 roll/pitch 및 수평 보정 입력은 변경하지 않는다. 센서 불능, 서보 건강,
  위치 제한, watchdog은 유지된다. 시작 각도는 매 프레임 따라가지 않는다.
- 자세 전환에서 발생한 tilt도 latch에 기록하여 다음 명령에 같은 재시도를 적용한다.
- MuJoCo의 공용 IMU 필터와 명시적 재시도는 동일 C `tilt_pause.h`를 사용한다.
  재시도 기준을 보행 필터 초기화로 잃지 않도록 보존한다.
- Windows 0.1.11: 화면 상단 주황 배너에 일시정지 원인과 재조작 안내를 표시.
  스틱은 활성 상태를 유지한다. safety=ok 일반 조회만으로 경고를 없애지 않고,
  새 drive 시작 ACK 또는 자세 완료 ACK+readback 확인 시 해제한다.

이 정책은 보호 기능 전체를 끄거나 실제 탈출 성공을 보장하는 기능이 아니다.
사용자 새 명령마다 제한된 재시도를 허용하며 추가 기울기에는 다시 정지한다.

## 검증 및 배포 상태

- Windows 전체 테스트 + 공유 필터 시뮬레이터 테스트: **115 passed**.
- Windows Zig 펌웨어 호스트 검사: **8 suites passed**. 실제 자세 감독 코드에서
  20° 기울기 정지 후 새 명령으로 모터 쓰기가 수행되고 완료되는 mock 검증 포함.
- 실기 관측(기울어진 상태에서 즉시 재정지)을 mock 및 공유 필터에서 재현했다.
  자동 재개 없음, 새 명령 재시도, 추가 기울기 정지, 수평 후 기준 복귀,
  IMU 무응답 유지, UI 경고 지속/해제를 검사했다.
- STM32 빌드: 326764 bytes / 327680 bytes.
  SHA256 `a6c44a37b0b88bcbb69fb319c9e7687b0909412b59b4411b50a44019d9dc7e77`.
  OTA 영역은 확장하지 않았다. 공간 확보를 위해 command_recovery.c와
  flight_log.c만 기존 console/진단 코드처럼 -Os로 빌드한다. 보행 코드는 -O0 유지.
- 새 실행 파일: `apps/windows/dist/controls-0111/SpotOMGController/SpotOMGController.exe`.
- **새 펌웨어 실기 설치 및 Landing 검증 완료**. 독립 위치 유지/전체 보행 및
  기울기 보호정지 재시도 실기 시험은 아직 수행하지 않았다.
  위 테스트는 실기 성공을 뜻하지 않는다.
- 마지막 관측은 기존 펌웨어에서 torque ON, custom, tilt 보호정지였다.
  이전 문서의 torque OFF는 설치 직후 상태이며 현재 상태가 아니다.
  설치 전 반드시 새 상태를 읽고 Landing 실제 도착 → torque OFF 절차를 따른다.
  기존 펌웨어가 Landing을 기울기로 거부하면 이를 생략해서 설치하지 않는다.

산출물: `artifacts/protective-pause-v90-r1/`.

## 사용자 설치 요청 후 진행

- 기존 앱을 정상 종료하여 BLE 연결을 해제하고 **Windows 0.1.11 교체·실행 완료**.
- `install-01/prepare.json`: 12축 정지·정상 통신·온도 30~40°C·11.4~11.6V.
  Landing은 `reason=unstable elapsed=0` 및 tilt error로 취소되었다.
  따라서 토크 OFF와 OTA는 실행하지 않았다. 기존 펌웨어/토크 ON 유지.
- IMUDIAG: present=1, 0x29, HAL error=0, SCL/SDA=1.
  이는 버스 상태이며 실제 기울기/IMU 자세 측정의 정확성 증명은 아니다.
- 사용자에게 몸체 수평 정리를 요청했다. 새 앱은 설치 통신과 경합하지 않도록
  연결 전 상태로 두었다. 새 준비 확인 후 별도 install-02 디렉터리로 진행한다.

## install-02 완료

사용자가 Landing 근처로 재시작한 후 다시 진행했다.
설치 전 Landing complete 2576ms/error21 → 12축 torque OFF 확인 → OTA 전송,
이미지 검증, STM32 기록·검증·재부팅 완료. SHA256은 위 빌드와 동일하다.
설치 후 Landing complete 2153ms/error21, 서보 12개 주소0~39 설정 동일 확인.
검증 후 Relax, 12축 torque OFF/정지/정상 건강 상태를 재확인했다.
최종 syncstate custom error86 torque=off safety=ok. 토크 해제 후 자세 라벨 변화이며
Landing 완료 실패로 해석하지 않는다. `install-02/verify.json` success=true.
Windows 0.1.11 실제 BLE 연결 및 전용 제어 채널 활성화를 확인했다.
저장된 발 들림 FL30/FR30/RL10/RR10mm 자동 적용 및 syncstate 일치 확인.
앱 연결 후 torque=off/safety=ok/약11.3V. 보행은 자동으로 시작하지 않았다.
