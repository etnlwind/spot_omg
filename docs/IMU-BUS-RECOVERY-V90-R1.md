# V90-R1 IMU 통신 진단 / 수동 복구 (2026-09-21)

## 현재 실기 관측

설치된 기존 펌웨어에서 15:20:12 `imucal status`가 읽기에 실패했고,
15:20:42 `i2cscan`은 응답 장치 0개였다. 서보 조회/BLE는 정상이다.
이는 현재 IMU 통신 실패의 증거이며, 배선/전원 불량과 MCU I2C 상태 고착을
구분하는 증거는 아니다. 증거는
`artifacts/foot-lift-v90-r1/app-diagnosis/imu-failure-live.txt`에 보존했다.

## 추가 명령

- `imudiag`: 거래를 시작하지 않고 BNO055 present/address, HAL state/error,
  I2C SR1/SR2와 실제 PB8 SCL/PB9 SDA 레벨을 표시한다. 오류 발생 순간의
  기록이 아닌 명령 시점 스냅샷이다. SR1/SR2는 디바이스 상태 레지스터다.
- `imurecover`: 정지 상태의 foreground 명령. 이전 진단값 출력 →
  HAL_I2C_DeInit/Init → chip ID 0xA0 / IMUPLUS mode 0x08 검증 →
  캐시를 무효화한 body sample과 Euler 읽기 3회(간격20ms) → 이후 진단 출력.
  어느 단계든 실패하면 성공으로 처리하지 않는다. 보행/시험 보행 중 거절한다.
  일반 자세 전환과 콘솔 명령은 foreground에서 직렬 실행된다.
- 센서 모드/보정값/서보/토크를 변경하지 않고 fault도 해제하지 않는다.
  GPIO bus-clear 펄스, 센서 전원 리셋, 센서 재설정, 자동 보행 재시작은 없다.
- IMUDIAG/IMURECOVER 결과는 전용 제어 DATA와 일반 로그 모두에 전달한다.

I2C 재초기화만으로 회복되면 전원을 끄지 않은 소프트웨어 복구가 가능하다는
증거가 된다. 최초 원인이 전기적 교란이 아니었다는 증명은 아니다.
실패해도 하드웨어 고장으로 단정할 수 없다(센서 자체/버스 고착이 남을 수 있음).

## 검증 및 설치 상태

실제 helper를 HAL mock으로 실행해 동작 중 거절, deinit/init 실패,
읽기 실패/잘못된 ID, body/Euler 실패, 연속3회 성공과 보정값 보존을 검사했다.
복구 helper + STM32/ESP32 제어 경로 호스트 검사 총3개 통과.
실제 지연/물리적 센서 복구/위치 유지 시험은 아직 하지 않았다.

ARM 빌드 성공: 327644 bytes (320KiB OTA 한계까지36 bytes 남음).
이미지: `artifacts/imu-recovery-v90-r1/firmware/attitudepd-v4-v90-r1.bin`
SHA256: `df2696fe996173b6dc1e39977760d39aa18529df77ad97ff71273775182c6108`.
버전 문자열은 기존 V90-R1을 유지하므로 설치 이미지 해시로 구분한다.
보행 코드 최적화/OTA 슬롯 확장 없이 도움말 문자열을 줄여 용량을 맞췄다.

## 실기 설치 및 수동 명령 시험 완료

사용자 전원 재시작 후 0x29 센서 응답과 보정 읽기가 회복됐다.
첫 Landing은 `OK landing`으로 완료됐으나 기존 설치 도구가 verbose POSE 형식만
허용하여 중단됐다. 별도 readback에서 Landing/오차30틱/토크ON을 확인하고,
12축 토크OFF까지 확인한 기록으로 설치를 재개했다. 실패 기록은 보존했다.
OTA 이미지 검증/STM32 기록/재부팅 성공. 설치 뒤 Landing complete-residual,
오차25틱, 12축 주소0..39 동일을 확인하고 Relax 및 12축 토크OFF를 확인했다.

`imudiag`: addr=0x29, HAL state=0x20, error=0, SR1/SR2=0, SCL/SDA=HIGH.
`imurecover` 두 번 모두 deinit/init 성공, chip=0xA0, mode=0x08까지 성공했으나
body_ok=0, samples=0/3으로 **실패**했다. 앞뒤 imucal status 읽기는 정상이다.
이는 body reader의 짧은 I2C 읽기/metadata/decoder 중 추가 구분이 필요한 결과다.
현재 명령은 단계별 세부 실패 이유를 모두 제공하지 않아 원인은 미확정이다.
정상 상태 시험도 최종 성공하지 않았으므로 고장 상태에서의 복구 효과를 주장하지 않는다.
로봇 보행 시험은 하지 않았다. 최종 torque=off/safety=ok/fault_code=0,
자유 안착 후 error81로 pose=custom 표시(직전 Landing 도착 확인과 구분).

증거: `artifacts/imu-recovery-v90-r1/reboot-check.json`,
`install-attempt-02/prepare.json`, `install-attempt-03/{prepare,ota-result,post-install,recovery-followup}.json`
(위 install 경로들은 `artifacts/imu-recovery-v90-r1/` 아래).
`post-install.json` success=false는 새 복구 명령 시험의 실패이며,
OTA 실패가 아니다. 재시도 전 BLE notification 등록 실패는 verify.json에 별도 보존했다.
