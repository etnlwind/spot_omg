# 원호 턴 지지 전환 정책: 앱·실물 배포 기록

2026-09-12. `arcsupport`는 `wave60_rank3`를 공유 C 제어 경로에 옮긴 **실험 정책**이다. 앱 표시는 **원호 턴 · 지지 전환 보정 · 실험 (검증실패)**다. 기존 `arcturn`과 기본 `cruise`는 보존한다.

## 적용 내용

- 고정 CAD 몸체 XY 파형과 연속 중력 하중 전환을 공유 C에서 계산한다. 상태를 매 프레임 보존하며, simulator 전용 접촉력은 제어에 입력하지 않는다.
- 다른 기구 모델을 쓰던 기존 balance 보정은 이 정책의 이동 중 중복 적용하지 않는다. 독립 IMU 기울기 안전 정지는 유지한다. 잔여 IMU 수평 보정까지 완성됐다는 뜻은 아니다.
- 앱은 펌웨어 `arcsupport` capability를 확인한 경우에만 실물 선택을 허용한다. 오래된 펌웨어에 새 정책 명령을 전송하지 않는다.
- iPhone UJIN17에 앱 V0.5.0 (43)을 빌드·설치했다. 휴대폰 에뮬레이터는 사용하지 않았다.
- 원격 제어 mailbox 요청 파일은 처리 직후 삭제하지 않고 ID로 중복 처리를 막는다. CoreDevice 파일 복사 검증과 삭제의 경쟁을 제거했다. 실제 iPhone에서 상태 조회와 연결 해제 응답을 확인했다.

## 검증 구분

- **호스트:** 새 상태 보존 구현은 앞/뒤/좌/우/혼합 입력과 Stop 350프레임에서 기존 실험과 관절각 0.002도 이내. O0/O2와 서보 tick 일치 검사 통과. Swift 원격 해제·Landing-before-Relax·mailbox 중복 방지·capability 검사 통과. XCTest를 실행한 것으로 간주하지 않는다.
- **MuJoCo:** 공유 정책으로 양방향 60초 재실행. 약 18.15도/초, 최대 roll 약 1.70도, pitch 약 0.79도. 안전 정지 없음. 하지만 매 스텝 최소 들림 약 12mm, 중간 스윙 접촉 최대 16.7%, 추종 RMS 약 1.95도이므로 합격이 아니다.
- **연결:** simulator TCP에서 새 정책 선택·명령·Stop과 JPEG 영상 응답을 확인했다. iPhone의 모든 터치 조작과 TCP/BLE 조합을 완전 검증한 것은 아니다.
- **실물 V43:** 이미지 검증·재부팅 후 `syncstate`에서 V43, `arcsupport` capability, Landing·torque off·safety ok를 확인했다. `gaitprofiles`에 새 정책이 나타난다. 실제 보행은 실행하지 않았다.
- **실물 계산 시간:** V43 `arcsupporttiming` 평균 41.148ms, 최대 44ms, 실패 0. 모터 명령 0인 계산 검사다. 기존 `arctiming` 평균 11.343ms, 최대 13ms. 새 정책은 계산만으로 20ms 주기를 넘었다. 설치 성공을 실물 사용 가능 판정으로 혼동하지 않는다.

실물 위치 유지 시험과 전체 보행 시험은 아직 미수행이다. `(검증실패)` 표기를 유지하며, 무게중심·서보 지연·모터 성능·쿠션 변화에 대한 합격도 주장하지 않는다.

증거: `artifacts/audits/arc-support-deployment-v43/`, `artifacts/audits/arc-control-2026-09-12/arcsupport_v43_y*.json`.

## V44/V45 계산 시간 원인 수정

V44는 `arc_support.h`와 `drive_control.h`에 다른 CAD 제어 함수와 같은 GCC O2 최적화를 적용했다. 호스트 동작 일치22개와 하중/원호 계약49개는 통과했지만 실기 평균41.109ms, 최대44ms로 시간 문제는 남았다.

`main.c`의 실제 시스템 클록이 PLL OFF, HSI 16MHz였다. V45는 외부 크리스털 없이 HSI /16 ×336 /4 = 84MHz를 사용한다. APB1은42MHz, APB2는84MHz이며 전압 스케일2, flash wait2다. STM32F446의 공식 클록·flash 제약은 [ST 데이터시트](https://www.st.com/resource/en/datasheet/stm32f446vc.pdf)의 버스 클록 및 전원/flash 표를 참조한다.

- HAL이 전환 후 USART1 1Mbaud, USART2/3 115200baud, I2C 100kHz를 설정한다.
- SysTick은 HAL에서1ms로 재설정한다. 보행 제어 주기20ms·속도/가속 한도·관절 목표·서보 위치 인코딩은 바꾸지 않는다.
- SPI1은 /128로 낮춰656.25kHz로 설정한다. CPU 상승 때문에 기존1MHz 센서 버스가5.25MHz가 되는 것을 방지한다. 실제 사용 중인 BNO055는I2C이며, BNO086/SPI 동작을 확인한 것으로 주장하지 않는다.
- 현재 시작하지 않는TIM2도 기존 주기를 보존하도록 prescaler44099로 조정했다. `.ioc`에도 관련 클록/분주 설정을 기록했다. CubeMX 재생성 후 `main.c` 설정을 반드시 비교한다.
- `clockdiag`는 시스템/버스 클록, SysTick reload, PLL ready, flash wait를 읽기만 한다.

V45의 실기 readback 및 시간 측정 결과는 아래에 추가한다.

### V45 실기 확인 결과

- 이미지237248bytes, SHA-256 `d713a45165a21707ed86427aa11e8d6581212dab9b2a321d21005c8eb9ff767f`. 최종 소스를 다시 빌드한 이미지도 같은 해시다. BLE 전송·STM32 이미지 검증·재부팅 성공.
- `clockdiag`: HCLK84000000, PCLK1 42000000, PCLK2 84000000, SysTick reload83999, PLL ready1, flash wait2.
- `arcsupporttiming`:128샘플, 평균8187µs, 최대9ms, 실패0, 모터 명령0. V43/44의 계산 단독20ms 초과는 해소됐다. 보행 중 전체I/O 지연까지 합격했다는 뜻은 아니다.
- 모든12개 서보 readback 응답, hardware error0, 읽기 재시도0. 전압 약11.2~11.4V. BNO055 I2C calibration 상태 응답(gyro3/accel3), 저장 설정 복원 확인. 정지 중 heading 캐시는 새 샘플이 없어 `locomotiondiag valid=0`; 이 조회를 보행 중 센서 스트림 시험으로 간주하지 않는다.
- 실물에서 `gaitprofile arcsupport` 선택 후 `syncstate`로 일치를 확인하고 원래 `cruise`로 복귀했다. 최종Landing, torque off, safety ok. 실제 보행·위치 유지 시험은 수행하지 않았다.
- 최종 호스트 회귀: 공유 정책22, 하중/원호 계약49, 펌웨어23 통과. 기존SPI /16 문자열 고정 검사는 실제APB2/분주로 계산한 센서 클록이1MHz 이하인지와 `.ioc`/C 일치 여부를 검증하도록 수정했다.

증거: `artifacts/audits/arc-support-deployment-v45/`.

최종 iPhone 원격 상태 조회는 `target=simulatorBluetooth`, `ready=true`, `firmware=shared-locomotion-v43-sim`, `motion_active=true`였다. 사용자가 시뮬레이터를 조종 중이므로 실물 연결 전환 요청은 `motion_in_progress`로 거부되었으며 강제로 중단하지 않았다. 앱의 V45 실물 재연결은 이 시점에 별도 수행하지 않았다. 실물 V45 확인은 직접BLE readback 근거다.
