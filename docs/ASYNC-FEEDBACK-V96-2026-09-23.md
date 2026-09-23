# V96: 보행 중 서보 조회 대기 분리 — 2026-09-23

## 현재 상태

사용자 요청에 따라 로봇을 끈 상태에서 구현·호스트 검증·펌웨어 빌드까지 완료했다.
**실기 연결, 설치, 레지스터 readback, 위치 유지, 하중 보행은 수행하지 않았다.**
다른 컴퓨터에서는 이 문서와 `HANDOFF-LATEST.md`를 먼저 읽고 설치를 이어간다.

펌웨어 식별자는 `attitudepd-v9-v96-async-feedback`이다. 새 보행 모델이 아니라
공유 보행의 통신 처리 변경이다. 앞서 요청한 전 축 ACC254 변경도 포함한다.
기존 V7~V10 소스가 함께 들어 있지만 V10 옆걸음의 미해결 문제를 고친 빌드는 아니다.
마지막 설치 확인 기록은 V93(67)이며 이번에 장치 상태를 다시 조회하지 않았다.

## 무엇을 고쳤는가

[과거 V70의 두 46ms 사건](CONTROL-GAP-46MS-2026-09-23.md)은 각각 한 축의
동기 조회가 31ms를 차지했다. 위치 쓰기 1ms + 조회 33ms + 나머지 처리 12ms였다.
25ms 응답 제한과 같은 호출 안의 재시도가 20ms 제어 루프를 막는 구조를 수정했다.
당시 첫 오류가 전기적 미응답인지 MCU 수신 누락인지는 아직 확정하지 않았다.

`robot_shared_drive()`는 이제 응답을 기다리지 않고 조회 요청을 예약한다.
USART1 인터럽트가 요청 TX와 응답 RX를 처리하고, 완성된 샘플만 전경 코드가
가져가 기존 안전·추종·배터리 처리에 넘긴다. 실패한 조회를 그 자리에서 재시도하지 않는다.
제어 주기는 기존 **20ms(50Hz)** 그대로다. 100Hz 최적값 검증을 완료한 것이 아니다.

주요 구현 위치:

- `firmware/stm32-learning/Src/servo_bus_async.c`: 버스 소유, IRQ 송수신, 마감, 오류 기록.
- `Src/servo_bus.c`: 동기 읽기에서도 RXNE보다 UART 오류를 먼저 판정하고,
  비동기 보행 중 동기 조회/버스 충돌을 차단한다.
- `Src/robot.c`: 완료 샘플 소비, 다음 프레임 이후 재시도, 피드백 신선도 확인.
- `Src/sts3215.c`: 동기/비동기 상태 해석을 공유한다. 앞 J2 누적 원점 정규화를 유지한다.
- `Src/stm32f4xx_it.c`, `Src/stm32f4xx_hal_msp.c`, `.ioc`: USART1 및 SysTick 연결.
- `Src/app_console.c`: 정지 후 `busdiag` 조회.

### 시간 예산과 버스 충돌 처리

|항목|구현값·동작|
|---|---|
|위치 명령|12축 44B Sync Write 유지. 1Mbps 선로 시간 약 0.44ms|
|위치 송신 API|`HAL_UART_Transmit`, 보행 중 HAL 제한 1ms. tick 양자화에 따라 실제 반환은 더 늦을 수 있음|
|상태 조회|요청 8B, 응답 21B(상태 15B 포함), TXE/TC/RXNE 인터럽트|
|요청 전 무통신 간격|마지막 RX/위치 쓰기 완료 이후 최소 1,000µs|
|조회 예약 대기|예약 후 4,000µs 이내 시작하지 못하면 실패|
|조회 TX/RX 마감|TX 시작 후 4,000µs. 무응답은 1ms SysTick에서 종료하므로 약 1ms 양자화 가능|
|오류 후 새 조회 격리|실패 완료 후 25,000µs 동안 새 조회 금지. 늦은 바이트는 버리고 계수|
|재시도|최대 1회, 다음 프레임 이후이며 격리 해제 후. 즉시 재시도 없음|
|피드백 신선도|어느 축이든 마지막 성공 샘플이 500ms보다 오래되면 기존 버스 오류 처리|
|정상 조회 빈도|일반 공유 보행 1축/프레임, 12축 순회 약 240ms|
|V6.2.5~6.2.7|이전 두 슬롯 유지: 직전 명령 샘플을 다음 계산 중에도 수집, 정상 순회 약 120ms|

4ms는 과거 정상 전체 조회 2~3ms(요청 전 HAL 간격 포함) 및 0.29ms 패킷 시간을
근거로 둔 초기 통신 예산이다. **실기 최적값이나 모든 서보의 응답 상한으로 실증한 값이 아니다.**
오류 후 25ms는 기존 응답 대기 범위를 다음 조회와 분리하는 보수적 격리다.
프로토콜에 요청 순번이 없어 그보다 훨씬 늦은 같은 ID 응답까지 완전히 식별할 수는 없다.
현재 서보의 return-delay 설정과 실제 응답 분포는 설치 후 확인해야 한다.

상태가 IDLE이며 마지막 선로 활동 후 1ms가 지난 경우에만 위치 쓰기를 허용한다.
격리 중에도 이 조건을 만족하면 위치 쓰기는 계속한다. 조회가 아직 진행 중이거나
선로가 조용해지지 않았다면 `SERVO_BUS_BUSY`로 기존 버스 오류 처리에 넘긴다.
응답 도중 위치 명령을 강제로 끼워 보내거나 오류를 무시하지 않는다.
따라서 계속되는 잡음/누락에서는 보행이 정지할 수 있다.

추가 슬롯은 다음 프레임 계산과 겹쳐 실행된다. 계산이 아주 짧거나 응답이 비정상적으로
늦어 다음 쓰기까지 버스를 점유하면 위 BUSY 처리에 따른다. 모든 모델에서 실제 프레임
간격이 반드시 20ms라는 보장은 호스트 검사만으로 할 수 없다.

### 수신 오류와 인터럽트

- SR을 DR보다 먼저 저장하여 RXNE와 ORE/FE/NE/PE가 함께 발생해도 오류를 보존한다.
- 비동기 모드에서는 RX를 계속 켜 둔다. 송신 후 RE 복구가 늦어 응답 첫 바이트를
  놓치는 구간을 없앴다. 어댑터의 정확한 8B 요청 에코는 선택적으로 제거한다.
- ID·길이·checksum·서보 오류 필드를 검증한다. 실패한 응답은 위치 데이터로 사용하지 않는다.
- USART1 우선순위 0, USART3 제어/콘솔 1, USART2 콘솔 2로 설정했다.
  서보 RX가 콘솔 줄 처리에 밀리지 않도록 했다. 제어/STOP 수신 경로는 유지한다.
- ISR은 고정 크기 패킷 처리와 RAM 기록만 한다. printf, HAL 대기, 콘솔 파싱은 없다.
  DMA로 변경한 구현은 아니다. IRQ 최대 실행 cycle을 계측하도록 추가했다.
- 취소는 응답을 기다리지 않고 IRQ 작업을 해제한다. 다만 **보행 종료 후** 동기
  자세/설정/토크 통신으로 넘어갈 때는 남은 격리 시간(최대 약 25ms)을 기다린다.
  이를 모든 STOP·토크 OFF의 지연이 0이라는 뜻으로 해석하지 않는다.

Landing/Stow 등 자세 처리, 초기 프로필 쓰기/readback, 구형 `robot_trot_scaled()`
경로의 동기 조회는 유지한다. 이번 개선을 모든 명령에 적용한 것으로 보고하지 않는다.
공유 보행의 IMU·계산·다른 콘솔 처리가 만드는 지연은 별도로 측정해야 한다.

## 정지 후 진단 방법

보행 중에는 출력하지 않는다. 종료 후 다음 명령으로 RAM 기록을 받는다.
새 보행을 시작하면 이 진단은 초기화되므로 먼저 저장한다.

```text
busdiag
busdiag dump 0 8
busdiag dump 8 8
busdiag dump 16 8
busdiag dump 24 8
```

첫 `busdiag`의 `trace_count`만큼만 페이지를 요청한다. 32개 미만이면 위 명령을
전부 보낼 필요 없다. 마지막 페이지에 `$BUS,END`가 나온다.

```text
$BUS,M,1,ok,timeouts,errors,late_bytes,refused_writes,max_us,trace_count,trace_triggered,trace_frozen,irq_peak_cycles
$BUS,R,index,start_ms,id,attempt,result,duration_us,first_rx_us,last_rx_us,uart_errors,received,stage,sent
```

- M의 `1`은 포맷 버전. timeout과 errors는 개별 시도 집계다.
- R의 `start_ms`는 MCU HAL tick 절대 시각. 시간 차는 DWT cycle로 계산한 µs다.
  예약 상태에서 실패하면 예약 시점을 시작으로 기록한다.
- `attempt`: 최초 0 / 재시도 1.
- `result`: OK 0, 인자 오류 1, UART/HAL 오류 2, timeout 3, 프로토콜 오류 4,
  서보 상태 오류 5, BUSY 6. BUSY 거절은 완료 행 대신 `refused_writes` 등으로 관찰한다.
- `stage`: IDLE 0 / QUEUED 1 / TX 2 / RX 3 / DONE 4. 완료 직전 단계를 저장한다.
- `sent`: IRQ에서 DR로 쓴 요청 바이트 수(0~8), `received`: 수신 버퍼에 저장한 수.
  오류가 난 바이트 자체는 버퍼에 포함되지 않을 수 있다.
- `first_rx_us`/`last_rx_us`에는 어댑터 에코도 포함된다. 아무 바이트도 저장하지
  못한 경우 `4294967295`다. 이를 서보 첫 응답 시각으로 무조건 해석하지 않는다.
- `uart_errors`: SR 원시 오류 비트. PE=1, FE=2, NE=4, ORE=8.
  프로토콜 오류의 세부 ID/checksum 구분은 이 행에 별도 코드로 담지 않는다.
- `late_bytes`: 활성 조회 외의 수신 바이트도 포함한다. 정확히 ‘지연 응답 수’가 아니다.
- `irq_peak_cycles`: ISR 내부 최대 측정 cycle. 현재 84MHz에서는 84로 나누면 µs.
  진입 전 지연/예외 진입·복귀 비용은 포함하지 않으므로 로직 분석기 계측을 병행한다.
- 최근 32시도 원형 버퍼. 최초 오류 뒤 8개의 완료 시도까지 저장한 다음 고정한다.
  집계는 계속 증가하므로 전체 시도 수와 덤프 행 수가 다를 수 있다.

`jointtrace`와 `busdiag`의 HAL 시각을 맞추면 어느 축의 어느 시도가 실패했고,
이때 다음 위치 명령 간격이 실제로 유지됐는지 구분할 수 있다.

## 완료한 검증

결과 파일은 `artifacts/async-feedback-v96/`에 보관했다. `verification.json`에
검사 파일 목록과 실기 미검증 범위를 기록했다. 진단 링 검사를 추가한 마지막
부분 재실행은 `async-final.log`(4개 통과)이며 아래 전체 수에 추가 합산하지 않는다.

1. 관련 Python/호스트 회귀 검사 **141 통과, 13 건너뜀** (`regression.log`).
   기존 래퍼가 Windows Zig를 찾지 못해 생긴 13개 skip은 아래에서 별도 실행했다.
2. 프로젝트의 Zig C 도구로 **14개 C 실행 파일** 빌드/실행 성공
   (`c-tests/results.json` 및 각 log). 위 13개 항목과 프로필 레지스터 검사를 포함한다.
3. 실제 `servo_bus.c`, `servo_bus_async.c`, `sts3215.c` 및 `robot.c`에서 추출한
   재시도 함수를 UART 주변장치/시계 모형으로 실행했다. O0와 Os 모두 통과.
   정상, 무응답, 부분 응답, 첫 바이트 누락, ID/checksum 오류, RXNE+ORE/FE/NE,
   7ms 지연 응답, 중복 응답, 에코, TX 정체의 13조건을 확인했다.
4. 12ms의 전경 계산을 가정한 80프레임 모형에서 응답 한 번을 누락시키고 재시도했다.
   **80개 쓰기의 간격은 모두 20ms**, 재시도 1회/회복 1회였다.
   V6.2.5 두 슬롯 모형도 40프레임/79조회, 명령 간격 20ms를 확인했다.
5. 반복 실패/500ms 오래된 피드백은 오류로 전환, 취소는 응답 대기 없음,
   DWT counter wrap, 오류 기록 원형 버퍼/동결, 앞 J2 원점 해석 보존을 검사했다.
6. BNO055의 O0/Os 양쪽에서 단위, 회전 변환, metadata, cache, 실패, busy,
   legacy 동작 검사 통과. 펌웨어 OTA 슬롯/시작 주소/revision 포함 검사 통과.

**이 20ms 결과는 실제 MCU 실행시간이나 실제 로봇의 측정값이 아니다.**
호스트 모형은 통신 상태기와 오류 정책을 검증하며 전체 `robot_shared_drive()`의
실제 계산 시간, IRQ 최악 실행시간, 전기적 파형, 발 접지와 하중을 재현하지 않는다.
실기에서 46ms 문제가 해소됐는지는 설치 후 같은 입력으로 다시 측정해야 한다.

주요 테스트 재실행(저장소 루트, Python 환경 준비 후):

```powershell
$env:PYTHONPATH='.;tools/servo_tool;apps/windows'
python -m pytest firmware/stm32-learning/tests/test_servo_async.py -q -s
python -m pytest firmware/stm32-learning/tests/test_packet_size_optimization.py firmware/stm32-learning/tests/test_ota_layout.py simulation/mujoco/tests/test_servo_profile.py simulation/mujoco/tests/test_shared_locomotion.py -q
```

macOS/Linux에서는 PYTHONPATH 구분자로 `:`를 사용한다. 기존 C 테스트 래퍼는
`cc`/`gcc`가 PATH에 있어야 한다. Windows에서는 `servo.host_build.build_executable`
의 프로젝트 Zig 경로를 사용한 별도 실행 결과가 `c-tests/results.json`에 있다.

## 설치용 산출물과 다른 컴퓨터에서의 후속 작업

- 파일: `artifacts/async-feedback-v96/firmware/attitudepd-v9-v96-async-feedback.bin`
- 크기: **327328B / 327680B**, 남은 슬롯 352B.
- SHA256: `fd70b04878976a5a4e0e2e1aed88672fbb3d2df610f6e43a66c2ae746a155126`
- 시작 SP `0x2001fff0`, reset `0x080374f1`. manifest와 build.log를 같이 보관한다.
- 재빌드: `python firmware/stm32-learning/build_firmware.py --output artifacts/async-feedback-v96/firmware --compiler <arm-none-eabi-gcc 경로>`.
  사용 컴파일러는 STM32CubeIDE의 ARM GCC 14.3이다.

고정 OTA 슬롯에 맞추기 위해 통신·센서·진단 처리에 Os 크기 최적화를 적용했다.
보행 순서 본체는 O0를 유지한다. 기존 actuator/heading 최적화도 포함한다.
BNO055와 부동소수점 계산의 fp-contract는 껐다. CubeIDE 기본 설정으로 임의
재빌드하면 같은 크기/옵션이 아닐 수 있으므로 위 빌드 스크립트를 사용한다.

다른 컴퓨터에서 진행할 순서:

1. Git 최신 코드를 받은 뒤 bin SHA256와 revision을 확인한다. V95 ACC 전용
   파일은 이전 결과이므로 V96 파일과 혼동하지 않는다.
2. 사용자가 로봇을 준비했을 때 실제 상태를 읽는다. **Landing으로 전환하고
   실제 도착 확인 → 토크 OFF 확인 → 설치** 순서를 지킨다. Landing 실패를
   몸체 지지 확인만으로 생략하지 않는다. 설치 후에도 먼저 Landing을 확인한다.
3. V96 revision, IMU, 12축 상태, 원점/영구 설정 보존을 확인한다. 기본 ACC254는
   요청값이므로 보행 프로필 전환 시 주소41 및46/47의 실제 readback을 확인한다.
4. 사용자가 정상으로 확인한 모델과 동일한 속도/저장 발 보정으로 비교한다.
   V10을 검증된 모델로 자동 선택해 시험하지 않는다. 보행/정지 후 `jointtrace`,
   `busdiag`, `gaitdiag`, `baldiag`를 함께 저장한다.
5. 콘솔 수신이 겹치는 조건에서 20ms 송신 간격의 분포·최댓값·late frame,
   응답 오류/재시도/격리, IRQ 최대 cycle, 제어·STOP 반응을 비교한다.
   정상 응답도 4ms를 넘는다면 그 원인을 먼저 측정한다.
6. 가능하면 USART1 TX/RX와 단일 서보 선로를 함께 캡처하여 MCU 누락인지
   서보 무응답인지 판정한다. 실제 하중 보행 성공은 별도로 보고한다.

이번 인계는 코드/빌드/호스트 검증 완료 상태이며 실기 배포 완료 상태가 아니다.
