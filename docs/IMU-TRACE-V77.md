# V77: 보행 중 IMU 관측 기록

## 원인과 변경

V76의 `imu on`은 main 루프의 10Hz 콘솔 출력이다. `app_console_poll()`이
동기식 `robot_drive()`를 수행하는 동안 main 루프 출력은 실행되지 않는다.
보행 내부 `shared_observe()`의 IMU 읽기와 기울기 보호는 계속 동작했다.
따라서 센서 단절이 아니라 분석용 시계열 수집 경로가 빠져 있었다.

V77은 이미 읽은 원시 roll/pitch, 유효 여부, 기울기/IMU fault, 보행 제어기 위상,
추종 속도 비율을 MCU uptime과 함께 RAM에 기록한다. 센서 추가 읽기나 보행 중
UART 전송은 하지 않는다. `imu on`의 라이브 출력 동작 자체는 바꾸지 않았다.
이 수정은 보행 중 자료를 보존해 정지 후 분석할 수 있게 하는 수정이다.

- `jointtrace arm`으로 관절/IMU 기록을 함께 예약한다.
- `jointtrace stop`은 다음 정지에서 두 기록을 함께 시작한다.
- `jointtrace off`로 두 기록을 정지한다.
- `imutrace status`, `imutrace dump OFFSET COUNT`로 읽는다. 페이지당 최대12개.
- 약50Hz, 512개(약10초). 용량에 도달하면 full 표시 후 고정하며 과거를 덮어쓰지 않는다.
- shared_observe의 초기 관측 및 매 프레임 관측을 저장하고, fault 판정 프레임도
  종료 전에 저장한다. drive 종료 시 고정하므로 이후 idle 관측이 기록을 바꾸지 않는다.
- 시각은 IMU 센서 내부 샘플 시각이 아니라 MCU가 관측한 시각이다.
  관절 trace의 MCU 시각과 맞출 수 있지만 센서/버스 지연까지 같다는 뜻은 아니다.
- phase/rate는 해당 관측 시점의 제어기 상태이며 센서가 측정한 보행 위상이 아니다.
  초기 관측의 rate는 보행 tracking reset 이전 값일 수 있다.

## 형식

```
$IT,M,1,COUNT,ARMED,FULL
$IT,P,OFFSET,COUNT,TOTAL
$IT,S,INDEX,MCU_MS,ROLL10,PITCH10,PHASE1000,RATE1000,VALID,FAULT
$IT,END
```

roll/pitch는0.1도 단위, phase/rate는1000분율. fault0=정상,1=IMU 실패,2=기울기 한계.
VALID=0의 roll/pitch는 측정값으로 쓰지 않는다. 기존 jointtrace schema는 그대로 유지한다.
`servo.imu_trace.download()`가 순서/개수/종료/시간 역전(32bit wrap 허용)을 검사하고
원본 및 CSV를 저장한다. 오류 때는 partial을 남긴다.
`scripts/hardware/capture_v627_first_step.py --revision s-native-v6-2-7-v77 ...`은
capability를 보고 기존 IMU 기록을 먼저 보존하며 시험 후 IMU도 내려받는다.
이 스크립트는 실기 구동 도구다. 오프라인 분석 도구와 구분한다.

## 검증·설치 상태

V77 빌드 성공: 316780bytes, SHA256
`7ba82f3b0ff209d7084204797f19c66ea04cb14b7a99f09a10a855f48dfbea9c`.
바이너리: `artifacts/imu-trace-v77/firmware/s-native-v6-2-7-v77.bin`.
RAM 링크 성공, _ebss=0x2000b150, 스택 상단0x2001fff0.
C recorder 검사: 비활성/예약/용량/동결/재예약/시각wrap/기울기fault 보존.
Python 검사: 페이지 검증, 손상·누락 거부, 읽기 전용 다운로드/CSV.
기존 C 검사의 단일 관절 조회 가정은 V625부터 적용된 두 조회와 맞지 않아 수정했다.
이번 IMU 변경으로 추가된 서보 조회는 없다.

**실제 보드는 V76이다. V77 OTA 및 실기 기록 검증은 아직 하지 않았다.**
사용자가 충전하기로 했으며 마지막 시험 후 Landing 확인/토크 OFF 상태로 마쳤다.
충전 후 설치할 때도 AGENTS.md의 Landing 도착 확인 → torque OFF → OTA →
Landing 확인 절차를 따른다. 설치 후 짧은 구동에서 IMU/관절 시간축 일치와
실제 프레임 지연을 검증해야 한다. 보행 전도 원인이 해결됐다는 의미는 아니다.
