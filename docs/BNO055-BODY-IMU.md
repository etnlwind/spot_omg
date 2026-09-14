# BNO055 실제 자이로와 BodyImuState

작성: 2026-09-14. 새 몸체 PD용 센서 입력만 추가했다. 이 작업에서는 실물 조회, 구동, 플래시를 하지 않았다. 현재 실물 gyro→몸체 축은 **미검증**이며 `axis_verified=false`가 기본이다. 전송이 정상이어도 새 PD는 축 검증 전 활성화하면 안 된다. 기존 Euler reader, 서보 좌표, 안전 정지 기준, 영구 보정 데이터는 유지한다.

## API와 단위

`bno055_read_body_imu(void *context, BodyImuState *sample)`은 `0x14..0x1f`의 12바이트를 한 번에 읽어 실제 각속도 XYZ와 융합 Euler H/R/P를 얻는다. `BodyImuState`는 `roll`, `pitch`를 rad, `gx`, `gy`를 rad/s로 담는다. 각속도는 Euler 차분으로 만들지 않는다. Z 각속도도 `Bno055.body_gyro_rad_s[2]`에 보존한다.

SI 변환은 아래와 같다. 단위 선택 비트는 Bosch 공식 드라이버와 교차 확인했다. [Bosch 데이터시트 §3.6.1, §3.6.4.3, §3.6.5](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf), [Bosch 공식 드라이버의 unit mask 정의](https://github.com/boschsensortec/BNO055_driver/blob/master/bno055.h).

| 입력 | UNIT_SEL | SI 변환 |
|---|---|---|
| gyro XYZ | bit 1=0, °/s | raw × π/(180×16) |
| gyro XYZ | bit 1=1, rad/s | raw / 900 |
| Euler H/R/P | bit 2=0, ° | raw × π/(180×16) |
| Euler H/R/P | bit 2=1, rad | raw / 900 |

모두 signed 16-bit little endian으로 디코딩한다. 새 Euler 값은 기존 설치 기준인 `body roll=sensor Euler pitch`, `body pitch=sensor Euler roll`을 유지하며 저장된 level offset을 rad로 빼 준다. 기존 reader는 0.1° 정수로 먼저 버림하지만 새 경로는 원본 센서 정밀도를 보존하므로 정수 변환 시 최대 0.1° 정도의 양자화 차이가 있을 수 있다. 이 Euler 이름 교환을 gyro XYZ에 그대로 복사하지 않는다.

`valid=true`는 읽기와 설정·단위 검사가 성공했다는 뜻이다. `axis_verified`는 별도다. gyro mounting 미검증 상태의 값도 진단에는 표시할 수 있지만 이를 PD에 넣으면 안 된다. 이 flag는 센서 융합 정확도나 calibration level을 보증하지 않는다.

`timestamp_ms`는 **MCU가 burst를 받은 시각(HAL_GetTick), 센서 내부 생성 시각이 아니다.** `sequence`는 성공한 burst 관측 횟수이며 센서 내부 sample counter가 아니다. 동일한 원시값을 두 번 읽어도 새 관측 번호가 붙을 수 있다. BNO055의 gyro 및 융합 Euler에는 내부 지연이 존재하며 한 burst 사용이 두 신호의 내부 생성 시점 일치나 지연 제거를 보증하지 않는다. 10ms 미만의 캐시 재조회는 값, timestamp, sequence 모두 그대로 반환한다. uint32 wrap을 고려한 경과시간으로 캐시를 판정한다.

## 설정 읽기와 제한 시간

uncached 관측마다 `UNIT_SEL(0x3b)..AXIS_MAP_SIGN(0x42)` 8바이트를 먼저 읽는다. IMUPLUS/normal power 및 유효한 축 회전인지 검사한 뒤 payload를 읽는다. 단위나 축 설정이 바뀐 뒤 예전 환산 계수를 재사용하지 않도록 했다. verified로 등록한 AXIS_MAP_CONFIG/SIGN과 Euler orientation bit 7이 일치해야 축 검증 flag를 통과한다. 설정을 쓰거나 임의로 기본값으로 되돌리는 동작은 없다. 초기화 후의 metadata 검사 실패는 기존 BNO 초기화 성공 여부를 바꾸지 않는다.

새 경로는 두 거래에 각각 3ms timeout을 쓰고 재시도·sleep은 하지 않는다. vendor HAL은 Timeout 인자와 무관하게 BUSY를 최대 25ms 먼저 기다리므로, 새 경로는 HAL READY 및 BUSY flag를 진입 전에 검사하여 이미 점유되거나 걸린 버스는 즉시 실패시킨다. HAL의 `elapsed > timeout` 판정으로 거래당 약 한 tick이 추가될 수 있다. 20ms 프레임에 들어가는 설계이지만, 이 결과는 SysTick이 실행되고 다른 ISR/마스터가 동시에 I2C를 점유하지 않는 현재 단일 마스터 사용을 전제로 한다. **호스트 테스트는 실제 I2C의 최악 실행시간 측정이 아니다.** 실패 시 새 출력은 invalid이고 sequence를 새로 만들지 않는다. 상위의 기존 Euler fallback까지 포함한 시간은 별도로 평가해야 한다.

새 burst의 yaw도 기존 heading 캐시에 저장하므로 성공한 새 reader 뒤에 heading을 조회할 때 추가 I2C read가 필요 없다.

## 실제 gyro 축 검증과 설정 방법

설정 파일은 `firmware/stm32-learning/Inc/bno_imu_config.h`다. 현재 `{+1,+2,+3}`는 **identity 임시값**이며 설치 방향을 추정하여 인증한 값이 아니다. `BNO055_BODY_AXES_VERIFIED`는 0으로 유지한다. `±1/±2/±3`은 각각 이미 BNO hardware remap이 적용된 출력 X/Y/Z의 양/음 채널을 뜻한다. 설정은 소프트웨어 회전이며 BNO axis register를 또 쓰지 않는다.

현재 저장소의 2026-08-27 bench 자료는 센서 Euler Roll→몸체 전후 기울기, Euler Pitch→몸체 좌우 기울기를 설명한다. gyro X/Y/Z 3축의 부호 및 몸체 좌표로의 proper rotation을 확인한 기록은 찾지 못했다. 두 축만 교환하면 determinant −1인 반사가 되며 오른손 좌표계 회전과 다르다. 새 helper는 중복 축, 0, 범위 밖 값, 반사를 거부하고 24가지 signed proper rotation만 허용한다. [Bosch 데이터시트 §3.4 axis remap 및 §3.6.5 각속도](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf).

향후 별도 실기 검증은 다음 기록으로 수행한다. 이 문서는 지금의 물리 시험·플래시 실행을 의미하지 않는다.

1. 서보 토크를 끈 상태에서 몸체를 안전하게 지지한다. register readback `UNIT_SEL`, `AXIS_MAP_CONFIG`, `AXIS_MAP_SIGN`, 설치 사진과 날짜를 남긴다.
2. 몸체 +X 전방, +Y 좌측, +Z 위를 표시한다. 거의 수평에서 한 축씩 작은 +회전과 −회전을 가하고 원시 gyro XYZ 및 Euler를 동시에 기록한다. +roll은 오른쪽이 내려가는 방향, +pitch는 앞쪽이 내려가는 방향, +yaw는 위에서 보아 반시계 방향이다.
3. 각 몸체 축에 대응하는 센서 gyro 채널과 부호를 세 축 모두 확인한다. 부호가 바뀌고 나머지 축 혼입이 작은지, 천천히 회전시킨 gyro 적분과 수평 부근 Euler 변화의 부호가 일치하는지 확인한다. Euler 차분은 이 검증의 비교 자료일 뿐 실행 중 D 항 입력을 대체하지 않는다.
4. `BNO055_BODY_GYRO_X_AXIS`, `Y_AXIS`, `Z_AXIS` 및 `EXPECT_AXIS_CONFIG`, `EXPECT_AXIS_SIGN`, `EXPECT_ORIENTATION`을 실제 기록으로 정한다. 회전 행렬 조건을 통과하고 6방향 시험이 맞은 후에만 `BNO055_BODY_AXES_VERIFIED=1`로 한다. 축 이름만 보고 예시 매핑을 복사하지 않는다.
5. runtime에서 설정하려면 검증한 값으로 `Bno055BodyFrame`을 채워 `bno055_set_body_frame(&imu, &frame)`을 호출한다. 잘못된 회전은 false를 반환한다. 성공하면 body cache가 무효화되어 다음 reader가 새 observation을 얻는다. 이 API는 RAM 설정만 바꾸며 영구 저장하거나 센서/서보에 쓰지 않는다.

설정 readback 성공, 호스트 변환 검사, 실제 gyro 축 시험, PD가 켜진 전체 보행 검증은 서로 다른 검증 단계다. 지금 완료한 단계는 호스트 검사뿐이다.

## 호스트 검증

```bash
/opt/anaconda3/envs/spot_omg/bin/python -m pytest \
  firmware/stm32-learning/tests/test_bno055_body.py -q
```

8개 검사가 통과했다. 실제 `bno055.c`를 HAL mock과 함께 C11 및 undefined-behavior sanitizer로 컴파일하여 독립 gyro/Euler 단위 조합, signed raw 극값, proper rotation 24종과 잘못된 축, metadata 변화·reset mode, cache/sequence/timestamp wrap, 센서 실패/복구, BUSY 즉시 반환·timeout 인자, 기존 Euler/level mapping 보존을 검사한다. mock이 HAL을 대체하므로 실기 축/버스 응답/보행 성공의 증거로 쓰지 않는다.

STM32CubeIDE의 실제 `arm-none-eabi-gcc`로 Cortex-M4/hard-float, `-O0 -Wall -Wextra -Werror` 설정에서 `bno055.c` 단독 컴파일도 통과했다. 이는 전체 펌웨어 링크·배포·실물 시험과 별개의 검사다.
