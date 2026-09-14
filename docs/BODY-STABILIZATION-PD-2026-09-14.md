# IMU 자세 안정화 · PD 구현과 검증

작성: 2026-09-14. 새 정책은 `attitudepd`이며 앱에는 **IMU 자세 안정화 · PD · 실험 (검증실패)**로 표시한다. 기존 `centerpivot`의 명목 보행을 보존하고, 지지 발의 높이를 IMU 기반 PD로 보정한다. 최종 설정 `kp=0.1`, `kd=0.01s`에서 MuJoCo 전진 60초를 OFF/ON 모두 안전 정지 없이 완료했다. ON은 RMS 기울기·각속도·관절 추종을 개선했지만 roll 최대 진폭은 커졌으며, 최대 기울기 3°·RMS 1.5° 목표를 충족하지 못했다.

STM32 **V58(`shared-locomotion-v58`)은 ARM 빌드까지 완료했으며 설치하지 않았다.** iOS 앱과 테스트 타깃도 빌드만 했고 실제 iPhone에 설치하지 않았다. 실물 gyro→몸체 축은 미검증이므로 현재 펌웨어에서는 새 PD가 활성 상태로 들어가지 않는다. 시뮬레이션 결과를 실물 보행 성공으로 해석하지 않는다.

## 제어 경로

```text
기존 centerpivot 명목 관절 목표
  → CAD FK: 쿠션을 포함한 발의 명목 위치
  → 지지 단계 가중치 × IMU PD의 발 높이 보정 ΔZ
  → 정밀 CAD IK: 최종 J1·J2·J3 목표
  → 기존 관절각→서보 목표 변환·전송 경로
```

명목 궤적은 [locomotion.h](../firmware/stm32-learning/Inc/locomotion.h), PD는 [body_stabilizer.h](../firmware/stm32-learning/Inc/body_stabilizer.h), CAD 어댑터는 [attitude_pd.h](../firmware/stm32-learning/Inc/attitude_pd.h)에 있다. MCU와 Python 시뮬레이터가 같은 C PD·CAD 어댑터를 사용한다. Python이 별도 제어 수식을 복제하지 않는다.

PD 입력은 지연된 roll/pitch와 실제 자이로 채널 `gx/gy`다. Euler 각도의 차분을 D 항으로 사용하지 않는다. 명목 발 위치와 예정 보행 위상으로 지지 발을 결정하며, 시뮬레이터의 정답 접촉력·실제 관절각·몸체 상태를 제어 입력으로 넣지 않는다. 이 값들은 물리 적분과 평가·기록에 사용한다. 기존 관절 보정과 새 PD를 중복 적용하지 않는다.

공유 정책 ID는 `centerpivot=14`, `attitudepd=15`이며 기존 기본 정책 `cruise`는 유지한다. 별도 명목 동작 검사에서 2,121개 명령·위상·크기 조합의 목표각과 주기가 같았고, 최종 OFF/ON 60초 기록에서도 명목 목표가 전체 구간 정확히 같았다. 보행을 느리게 바꾸거나 보폭을 줄여 비교하지 않았다.

이 구현은 **위치 서보를 위한 발 위치 PD 보정**이다. 접촉력 최적화, 전신 동역학 제어(WBC), 토크 MPC/WBIC를 구현한 것으로 표현하지 않는다. 서보 내부 이득·토크 한계·모터 사양·관절 부호·Stow/Landing 원점은 이 정책의 튜닝 대상이 아니다. 좌표 변환을 수정할 때는 [STS3250 실기 기준](./STS3250-POSITION-CONTROL.md)을 먼저 확인한다.

## 단위·부호·필터·제한

몸체 좌표는 +X 전방, +Y 왼쪽, +Z 위이며 각도는 rad, 각속도는 rad/s, 길이는 m이다. PD의 각도 오차와 출력은 다음과 같다.

```text
e_roll  = target_roll  - filtered_roll
e_pitch = target_pitch - filtered_pitch
u_roll  = kp_roll  × e_roll  - kd_roll_s  × filtered_gx
u_pitch = kp_pitch × e_pitch - kd_pitch_s × filtered_gy
ΔZ_i    = w_i × clip(-u_roll × y_i + u_pitch × x_i, ±max_foot_offset_m)
```

실제 적용 전에 `u`에 축별 제한과 시간에 비례한 변화율 제한을 적용한다. 위상은 다음 프레임으로 증가시킨 값이 아니라 **현재 보정할 명목 목표의 위상**이다. 다리 순서는 FL/FR/RL/RR, 대각선 위상 차이는 0/0.5/0.5/0이다. 지지 시작·끝의 가중치를 80ms 동안 매끄럽게 변화시키며, 예정 스윙 전체에서 가중치가 정확히 0이다. 실제 접촉을 측정한 가중치라는 뜻은 아니다.

설정 원본은 [config/body_stabilization.json](../config/body_stabilization.json), 생성 C 설정은 [body_stabilization_config.h](../firmware/stm32-learning/Inc/body_stabilization_config.h)다.

| 항목 | 최종 설정과 의미 |
|---|---|
| 제어 주기 | 20ms, 명목 50Hz. 실물 최악 실행시간을 측정한 결과는 아님 |
| 목표 roll/pitch | 각각 0rad |
| `kp_roll`, `kp_pitch` | 각각 0.1 |
| `kd_roll_s`, `kd_pitch_s` | 각각 0.01s |
| 각도 LPF / gyro LPF | 이전 값 가중치 각각 0.5 / 0.2 |
| LPF 수식 | `filtered = alpha × previous + (1-alpha) × raw`; 새 표본에만 갱신 |
| 축별 보정 한도 | 각각 ±0.05rad |
| 보정 변화율 | 0.15rad/s, 실제 `dt`에 비례 |
| 발 위치 보정 한도 | 각 발 최대 0.005m. 최종 CAD FK 변위도 검사 |
| 각도 입력 guard | 목표와의 오차 0.1745329252rad, 약 10° |
| gyro 입력 guard | 3.141592654rad/s, 약 180°/s |
| 표본 timeout | 100ms |
| 유효 `dt` | 0보다 크고 100ms 이하 |

기존 보행 IK의 약 0.1~0.5mm 잔차 허용은 작은 PD 보정을 무시하거나 최종 5mm 한도를 넘길 수 있었다. 새 어댑터는 잔차 1µm까지 재계산하고 목표에 5µm 여유를 둔다. 최종 `||FK(q_final)-FK(q_nominal)|| ≤ 5mm`를 별도로 확인한다. 어느 한 다리의 IK가 실패하면 일부 다리만 갱신하지 않고 실패를 반환한다.

OFF·표본 누락·지연·축 미검증·각도/gyro guard에서는 유효한 `dt`로 내부 보정을 0으로 감소시킨다. 감소 중에도 스윙 발에는 보정이 유입되지 않는다. NaN/Inf, 잘못된 설정·기하·시간, 역행 표본도 검사한다. 수학적 입력 오류와 IK 실패는 정상 OFF와 구분하며, 제어 함수 자체가 서보 토크를 끄는 명령을 만들지는 않는다. 기존 Stop·watchdog·기울기 안전 처리와 상위 오류 처리는 별도로 유지한다.

## 실제 IMU 입력의 현재 범위

BNO055 경로는 실제 gyro XYZ와 Euler를 burst로 읽고, 설정 readback에 맞춰 SI로 변환한다. `valid`와 `axis_verified`는 별개다. [bno_imu_config.h](../firmware/stm32-learning/Inc/bno_imu_config.h)의 identity 축은 임시값이며 **`BNO055_BODY_AXES_VERIFIED=0`**을 유지한다. 전송 성공만으로 장착 축·회전 부호가 검증된 것은 아니다.

실물 타임스탬프는 MCU 수신 완료 시각이며 센서 내부 생성 시각이 아니다. 성공한 burst 번호도 내부 센서 표본 번호가 아니다. 캐시·지연·단위·축 회전 검사와 향후 토크 OFF 상태의 3축 검증 절차는 [BNO055 실제 자이로 문서](./BNO055-BODY-IMU.md)를 따른다. 기존 Euler reader의 축 이름 교환을 자이로 축에 그대로 복사하지 않는다. 현재 새 body gyro reader는 BNO055 경로에 연결되어 있으며, 기존 BNO086 Euler fallback이 새 PD용 실제 gyro 입력을 대신하는 것은 아니다.

## 정책 선택과 ON/OFF

앱에서 제어기가 `attitudepd` capability를 알릴 때 새 정책을 선택할 수 있다. capability가 도착했다는 이유로 자동 선택하지 않으며, 속도는 아직 `nil`이고 `(검증실패)` 표시를 유지한다. 기존 **수평** 버튼의 `balance on/off`는 이 정책에서 새 PD 설정을 가리킨다.

앱의 일반 `balance` 명령과 CLI의 실시간 경로는 구분한다. 시뮬레이터는 일반 명령도 처리하지만 실제 STM32 보행 루프가 일반 콘솔을 점유할 때 앱 버튼의 즉시 처리까지 검증한 것은 아니다. 이번 변경의 보행 중 ON/OFF/상태 조회 경로는 다음 CLI와 `@B` 패킷이다.

```bash
conda activate spot_omg

# 연결된 가상 로봇을 예로 든 정책 선택. 보행을 시작하는 명령은 아니다.
spotctl --host 127.0.0.1 console send gaitprofile attitudepd
spotctl --host 127.0.0.1 stabilize status
spotctl --host 127.0.0.1 stabilize off
spotctl --host 127.0.0.1 stabilize on
```

| 명령 | 전송 패킷 |
|---|---|
| `spotctl stabilize on` | `@B 1` |
| `spotctl stabilize off` | `@B 0` |
| `spotctl stabilize status` 또는 `spotctl stabilize` | `@B 2` |

대상은 기존 `--host`, `--via ble`, `--via stm32 --stm32-port …` 옵션으로 정한다. BLE는 기존 앱 연결 관리 절차를 그대로 사용한다. `stabilize`는 보행 중 막히는 일반 콘솔 동기화·시각 조회를 생략하며, `$STABILIZE enabled=… status=… rate_hz=50`와 prompt를 확인한다. 예전 prompt만 수신한 것을 성공으로 처리하지 않는다. `enabled=1`이어도 `axes-unverified`, `stale`, `other-policy` 등은 실제 보정 중이라는 뜻이 아니다.

MCU UART 수신 경로에서는 요청만 반영하고 ACK 문자열은 foreground의 제한된 서비스 예산에서 보낸다. 호스트 검사는 실제 RX/parser/service 코드를 실행하지만 물리 UART/BLE 연결 및 실물 보행 중 지연을 측정한 시험은 아니다. TCP loopback에서는 정책 선택·보행 중 ON/OFF/status·Stop·다른 정책으로 전환을 검사했다.

## 펌웨어 로그

STM32는 원시/필터 IMU, 오차, 적용 PD, 다리별 ΔZ·가중치, 상태와 clamp/IK 실패를 **최근 32프레임 RAM 링 버퍼**에 남긴다. 50Hz 기준 약 0.64초이며 새 공유 보행 시작 시 비워진다. 전원이 꺼져도 남는 flash 기록이 아니다. 보행 중 긴 CSV를 전송하지 않고 Stop 후 idle 콘솔에서 읽는다.

```bash
# 로봇을 Stop한 뒤 실행. top-level stabilize의 mode에는 log가 없으므로 콘솔로 보낸다.
spotctl --host 127.0.0.1 console send stabilize log
```

nano printf의 부동소수점 출력에 의존하지 않도록 값은 단위가 붙은 정수로 출력한다. `time_ms/sample_ms`는 ms, 각도·오차·PD의 `*_urad`는 µrad, `*_urad_s`는 µrad/s, `*_z_um`은 µm, `*_weight_ppm`은 가중치×1,000,000이다. 시간과 상태·mask를 제외한 해당 수치는 원래 SI 값으로 되돌릴 때 1,000,000으로 나눈다. 시뮬레이터 JSON/CSV는 전체 기록이며 이 32프레임 펌웨어 로그와 구분한다.

## 60초 OFF/ON 물리 시뮬레이션

같은 추정 물성·총질량 2.754kg·쿠션·모터·보행 명령으로 각각 70초 실행했다. 처음 10초 이후 **10–70초의 전진 60초, 3,000표본**을 비교했다. 명목 보행은 주기 1.35초, duty 0.52, 보폭 80mm, 발 들림 목표 24mm다. 기존 균형·방향 유지 보정은 끄고 새 PD만 OFF/ON 했다. 양쪽 모두 안전 정지 없이 완료했으며 명목 목표는 정확히 동일했다. 물리 적분된 관절과 몸체를 기록했으며 영상에 목표 자세를 대신 넣지 않았다.

| 지표 | OFF | ON | 변화 |
|---|---:|---:|---:|
| Roll RMS | 3.432° | 1.909° | −44.4% |
| Pitch RMS | 1.598° | 0.900° | −43.7% |
| Roll 최대−최소(P-P) | 8.294° | 10.711° | **+29.1% 악화** |
| Pitch 최대−최소(P-P) | 5.877° | 5.291° | −10.0% |
| Gyro X RMS | 28.123°/s | 19.979°/s | −29.0% |
| Gyro Y RMS | 14.056°/s | 10.206°/s | −27.4% |
| 최대 절대 roll | 6.328° | 5.698° | −10.0% |
| 최대 절대 pitch | 3.172° | 2.746° | −13.4% |
| 최대 관절 추종 오차 | 8.784° | 8.162° | −7.1% |

전방 변위는 OFF 5.938m, ON 7.232m이며 ON의 적용 발 보정 최대는 0.737mm였다. RMS는 0° 기준 오차이므로 평균 기울기의 감소와 진동 감소가 함께 반영된다. RMS 감소만으로 모든 순간의 흔들림이 작아졌다고 판단하지 않는다. **최대 roll 5.698°와 roll RMS 1.909°가 목표를 넘으므로 최종 판정은 검증실패다.** 추정 질량 분포·관성·마찰·서보 지연을 포함한 시뮬레이션이며 실물 파라미터 식별을 완료한 모델은 아니다.

### 실패한 튜닝도 보존

기존 IK에서 `(.1,.01)`, `(.1,.05)`, `(.3,.01)`, `(.3,.05)`, `(.5,.015)`, `(.5,.05)`의 동일한 양축 kp/kd를 시험했다. kd=.05s 세 후보는 각각 24.06초, 27.02초, 27.44초에 tilt 안전 정지했다. 이 결과를 짧아진 정상 구간만으로 비교하지 않고 동일 10–30초 구간과 실패 상태를 함께 남겼다.

정밀 IK 적용 후 기존 kp=.2/kd=.03은 20초 전진에서 roll P-P 19.155°, 최대 추종 오차 17.128°로 악화됐다. 낮은 kp=.1/kd=.01은 각각 10.190°, 8.162°였고 60초 추가 시험도 완료하여 최종 기본값으로 선택했다. 기존 IK 6회와 정밀 IK 3회는 실행 전후 소스 SHA가 모두 같았으며 서로 다른 solver 결과로 분리했다. 전체 표·실패 기록·원본 설정은 [튜닝 보고서](../artifacts/audits/body-pd-2026-09-14/tuning/TUNING.md)와 [색인](../artifacts/audits/body-pd-2026-09-14/tuning/index.json)에 있다.

## 검증 수준과 재현

집중 회귀 104개와 설정 생성물 검사가 통과했다. 호스트 C 단위·센서 HAL mock·자이로 에뮬레이터·정밀 CAD IK·스윙 보존·제한·필터·오류/복구·콘솔 parser·TCP loopback을 검사했다. CLI 검사는 별도로 통과했다. 이 결과는 실제 센서 설정 readback, gyro 장착 축 시험, 서보 위치 유지 시험, 실제 전체 보행 시험을 대체하지 않는다. 이번 변경에서는 그 실물 시험들을 수행하지 않았다.

```bash
conda activate spot_omg
python tools/generate_body_stabilization_config.py --check
python tools/generate_locomotion_profiles.py --check
python tools/generate_gait_speed_labels.py --check

python -m pytest -q \
  firmware/stm32-learning/tests/test_body_stabilizer.py \
  firmware/stm32-learning/tests/test_bno055_body.py \
  firmware/stm32-learning/tests/test_stabilize_console.py \
  simulation/mujoco/tests/test_body_stabilizer_integration.py \
  simulation/mujoco/tests/test_body_stabilizer_transport.py \
  simulation/mujoco/tests/test_bno055_emulator.py

python -m pytest -q tools/servo_tool/tests/test_stabilize_cli.py

# GUI·실물 연결 없이 현재 설정으로 새 물리 기록 생성
python simulation/mujoco/scripts/validation/validate_body_stabilization.py off \
  --duration 70 --output /private/tmp/spot-pd-off-70s.json
python simulation/mujoco/scripts/validation/validate_body_stabilization.py on \
  --duration 70 --output /private/tmp/spot-pd-on-70s.json
```

V58 최종 ARM 빌드는 [manifest](../artifacts/firmware/body-pd-v58-final/manifest.json)에 기록되어 있다. 바이너리 크기는 271,088바이트이며 SHA-256은 `e472b9e9bbcf2e87ed44537e38a22c7393776f53ae5197388b97111d82de9210`이다. 빌드 성공은 설치·부팅·실기 주행 확인과 구분한다. 직접 빌드/전송 절차는 [README](../readme.md)의 STM32 절차를 따른다.

## 영상과 원본 기록

- [OFF 네 시점 30초 영상](../artifacts/gait-videos/2026-09-14/body-pd-off-four-views-30s.mp4)
- [ON 네 시점 30초 영상](../artifacts/gait-videos/2026-09-14/body-pd-on-four-views-30s.mp4)
- [최종 60초 비교 JSON](../artifacts/audits/body-pd-2026-09-14/final-comparison.json), [비교 표](../artifacts/audits/body-pd-2026-09-14/final-comparison.md)
- [OFF 70초 원본](../artifacts/audits/body-pd-2026-09-14/final-off-70s.json), [ON 70초 원본](../artifacts/audits/body-pd-2026-09-14/tuning/precise_kp0.1_kd0.01_70s_attempt1.json)
- [앱·CLI 빌드 및 검사 기록](../artifacts/audits/attitudepd-2026-09-14/PROFILE-CLI-APP.md)

30초 영상은 동작을 살펴보기 위한 자료이며, 위 성능 표는 별도 60초 전체 전진 구간을 집계한 결과다.
