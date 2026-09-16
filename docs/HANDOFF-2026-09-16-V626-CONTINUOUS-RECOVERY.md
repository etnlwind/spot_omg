# V6.2.6 연속 내딛기 — 2026-09-16 후속 작업

> **최종 후속 상태: V6.2.7/v76 설치 후 실제 첫걸음에서 전도했다.**
> [최신 전도 인수인계](HANDOFF-2026-09-16-V627-FIRST-STEP-FALL.md)를 먼저 읽는다.

> 최신 작업: V6.2.7/v76을 별도 추가하여 실기 설치와 버전/기본 프로필 readback을 완료했다.
> 최종 기체 상태는 Landing·토크ON·safety OK다. 실기 보행은 이번에 실행하지 않았다.
> 아래의 “실기 변경 없음”은 각 이전 분석 시점의 기록이다.
> 현재 설치 상태·실제 검증 범위는 [V6.2.7 실기 반영](S-NATIVE-V627-INSTALL-2026-09-16.md)을 먼저 읽는다.

> 후속 사용자 요청: J2를 더 뒤로 드는 동시에 J3를 빠르게 접고, J2를 내릴 때
> 쿠션이 바닥에 닿지 않을 최소 접힘각을 계산했다. 수평 S 높이에서 J2=50°일 때
> J3≥96.49°가5mm 조건이다. 실제 몸체 자세 및 실제 J3 도달 시점을 함께 봐야 한다.
> [기하 계산·기존 끌림 재검증](S-NATIVE-J2-J3-CLEARANCE-2026-09-16.md). 실기 변경 없음.

> 최신 보행 작업: 사용자의 Landing 기구 설명 이후 오프라인 보행 검증을 재개했다.
> r2 후보에서 회수 초반 약2.88mm 목표 역행을 발견해 r3 후보에서 제거했다.
> 30초/STOP은 완료했지만 끌림·흔들림은 남아 있다. 실기 명령/설치는 하지 않았다.
> [r3 변경·영상·잔여 문제](S-NATIVE-RECOVERY-MONOTONIC-2026-09-16.md)를 먼저 읽는다.

> 후속 갱신: 실기에 v75(V6.2.6+빠른 Stand+RR J3 측정 기능)를 설치했다.
> 빠른 Stand는 실기 확인했고, 바닥 J3 작은 응답 시험은 목표 미달로 중단됐다.
> 가속도 식별/연속 보행 검증 완료가 아니다. 최신 기체 상태·증거·남은 문제는
> [응답 측정 기록](SERVO-ACCELERATION-VALIDATION-2026-09-16.md) 하단을 먼저 읽는다.
> 종료 상태는 Landing·토크ON·safety OK다. 아래 v72 미설치 설명은 당시 기록이다.

집에서 올린 `a9fbba1`을 `develop`에 fast-forward로 가져온 뒤
[V625 인수인계](HANDOFF-2026-09-16-V625-STEP-HESITATION.md)를 이어서 작업했다.
**V6.2.6/v72는 빌드·호스트·시뮬레이터 검증 단계이며 실기에는 설치하지 않았다.**
마지막 문서상 실기 설치 버전은 v70이다. v71 배터리 telemetry 변경은 v72에도 포함했다.
이 문서 작성 시 새 변경은 아직 커밋/푸시하지 않았다.

## 결론과 남은 문제

- 매 걸음 멈칫함: 회수 궤적의 정체와 추종 감독기의 급정지를 줄였다. 동일한 추정 물리에서
  몸체 고정 30초 시험의 위상 정지 누적 시간이 **5.36초 → 0.24초**로 줄었다.
- 바닥 시뮬레이션: 8개 전진/후진/회전/조기 정지 조건과 30초 전진에서 보호 중단 없이 S로 복귀했다.
  이는 **끌림 없는 보행 합격을 뜻하지 않는다.** 앞발 회수 중 접촉이 남는다.
- 발 들기 높이만 12mm에서 18/24/30mm로 늘린 비교는 각각 4.60/4.48/3.88초에 기울기 보호로
  중단됐다. **채택하지 않았다.** 지지 다리의 움직임, 몸체 피치, 실제 회수 접촉을 함께 다룰 필요가 있다.
- Windows 앱/시뮬레이터 실행 파일 연동을 확인했다. iOS는 소스만 변경했고 Mac 빌드/설치는 하지 않았다.
- 실제 위치 유지, 실기 정상 STOP, 실기 바닥 보행은 이번 작업에서 수행하지 않았다.

## 수정 내용

V625의 수평 회수는 swing의 20~80%에 집중됐고 양 끝에서는 X가 고정됐다.
뒤쪽 확장을 비선형으로 몰아넣은 궤적과 늦은 감속이 겹쳐 추종 오차가 14°에 도달하면 위상이 멈췄다.

`s_native_v6_2_6`은 기존 모델을 보존하고 다음을 적용한다.

1. 전체 quintic 반 주기에 +20mm ↔ −125mm 이동을 분산한다. 회수 중 X를 고정하는 구간을 없앤다.
2. 추종 감속률을 0.6/s에서 6/s로 바꿔 14° 급정지에 도달하기 전에 속도를 낮춘다.
   회복률 2/s, 6~14° 감속 범위, 14° 정지, 피드백 노후/지속 정체 보호는 유지한다.
3. 원래 1초 주기, 12mm 들기, 반 주기 대각선 X/Z 동기, FR 첫걸음 J1 두 배,
   두 걸음 STOP→S, 역방향 60% 한도, 실제 앞 J1 부호와 J2 누적 좌표를 유지한다.
4. 펌웨어 프로필 **22**로 추가한다(V625는 21). Windows/iOS 목록 맨 위와 지원 연결의 기본값,
   시뮬레이터 기본값을 갱신했다. 실제 v70 연결은 capability에 따라 V625를 선택한다.
5. 측면 지지 보정은 기존 V623 참조를 상속한다. **지지 보정을 새 궤적에 맞춰 재설계한 것은 아니다.**

주요 소스: `s_native_gait.py`, `s_native_impl.h`, `gait_tracking.h`, `robot.c`,
`virtual_robot.py`, `config/locomotion_profiles.json`, 두 앱의 모델 목록.

## 서보 시뮬레이터 오류와 실기 기록 비교

기존 물리 코드는 목표에 가까워지면 위치를 목표로 잘라 붙이면서 내부 목표 속도는 남겨두었다.
따라서 위치 변화와 저장된 속도가 불일치했고, 다음 명령에서 과도한 왕복이 생겼다.
`ServoProfile.advance_reference()`에 제동 거리를 고려한 가속도 제한 참조를 넣었다.
위치 변화는 항상 `velocity * dt`이고 설치 기체의 **J1/J3=50, J2=254**, 속도3400 제한을 유지한다.
수치 회귀에서 속도·가속도 한도, 역전, 최종 정착을 검사했다.

실기 `hardware-30s`에 들어 있는 joint trace는 **전체 30초가 아니라 처음 약 5.13초**다
(256개 목표 프레임). 같은 기록 명령을 재생하고 2.0~5.129초의 실측 엔코더와 비교했다.
12관절 RMS 오차의 평균은 기존 참조 **7.630°**, 제동 참조 **0.830°**였다.
이 비교를 위해 물리 매개변수를 맞춰 바꾸지는 않았다. 수평 고정 몸체, 20ms 명령 표본화,
10.9V 추정 모델이라는 한계가 있으며 서보 내부 제어기 식별 완료를 뜻하지 않는다.

- 실행 도구: `scripts/analysis/compare_servo_reference.py`
- 원본/해시/관절별 결과: [재생 결과](../artifacts/s-native-v6-2-6/validation/v70-servo-reference-replay.json)

## 검증 결과

### 호스트 / 앱 / 빌드

- V621~V626 관련 검사, 설치 서보 제한, Windows 전체 테스트, joint trace: **162 passed**.
- 별도 C 안전/좌표/Stow/복구/프로필 레지스터/배터리 telemetry: **7 passed**.
- C/Python V626 목표 일치: 전진/후진/회전/조기 STOP 포함 10조건, 최대 차이 0.15° 미만.
- Windows EXE가 로컬 MuJoCo에 접속하고 V626 상태와 실제 렌더링 프레임 수신 후 정상 종료:
  [화면](../artifacts/s-native-v6-2-6/validation/windows-mujoco.png),
  [로그](../artifacts/s-native-v6-2-6/validation/windows-mujoco.log).
  화면의 10.7V 빨간 경고는 시뮬레이션 전압이며 실기 전압이 아니다.

이전 인수인계의 Windows STOP 테스트 실패는 자유 바닥 V6.1이 버튼을 누르기 전에
기울기 보호로 끝나는 것이 원인이었다. STOP 통신/관절 복귀 검사는 몸체를 고정한 시험으로
분리했고 서보 제한은 유지했다. 바닥 보행 합격으로 바꿔 보고하지 않는다.

전체 오래된 테스트까지 확장 실행한 결과는 **901 passed / 47 failed / 29 errors**
(별도 32 subtests passed)로 전부 통과하지는 않았다.
63건은 Windows에 없는 `cc` 호출, 1건은 저장된 Mac 절대 STL 경로,
2건은 기존 보행 수치 해시, 나머지는 과거 모델/기본값/물리 기대치 관련이다.
관련 물리 실패를 기존 서보 참조로 되돌린 별도 검사에서도 Stow 3건, optimized/turn/upright,
이전 virtual_robot 기대치 실패가 재현됐다. 전체 실패를 새 변경의 성공 증거로 제외하지 않는다.
세부는 `validation/full-tests.xml`, `full-tests.log`, `legacy-reference-tests.xml`에 있다.

### 몸체 고정, 동일한 추정 물리·12.6V·30초 전진

초기 전환을 제외한 video 4~32초, 총28초 구간의 비교다.

| 측정 | V625 | V626 |
|---|---:|---:|
| 위상 rate < 0.001 누적 | 5.36s | 0.24s |
| 평균 위상 rate | 0.508 | 0.601 |
| 최대 순간 목표/관절 오차 | 21.94° | 13.73° |
| 몸체 기준 발 속도 <20mm/s 비율 | 8.88% | 0.23% |
| 최종 S 실제 관절 최대 오차 | 0.219° | 0.219° |

이 발 속도는 네 발의 **3차원 몸체 기준 속도**다. 바닥에서의 미끄럼 속도가 아니다.
순간 목표/관절 오차는 순차 조회되는 firmware 감독기의 오차와 시점이 다르다.
[비교 그림](../artifacts/s-native-v6-2-6/validation/hesitation-comparison.png).

### 자유 바닥

8조건: 전진473/1000, 0.3초/1.2초 조기정지, 후진−600, 회전±500, 곡선600/250.
모두 보호 중단 없이 정상 S 복귀, 최종 목표 오차0°, 실제 최대 약0.99°, 내부 충돌0이다.
30초 전진: X5.285m, 횡방향9.8mm, yaw−0.11°, 보행 최대 기울기4.56°, 정지 중5.91°.
단일 추정 조건의 결과이며 실기 속도나 전원/마찰 변화에 대한 보장을 뜻하지 않는다.

회수 중앙 위상0.60~0.90에서 접지력1N 초과 비율은 8초 시험 앞발 FL76%/FR70%, 뒷발0%였다.
앞발 끌림을 배제할 수 없다. 이 수치는 목표 위상 기준 접촉이고, 실제 이동 방향/속도까지
포함한 끌림 판정 및 지지 교대 동기를 다음 작업에서 추가해야 한다.
[접촉 결과](../artifacts/s-native-v6-2-6/floor-validation/recovery-contact.json).

## 실행과 산출물

Windows 실행 파일:
`apps/windows/dist/v626/SpotOMGController/SpotOMGController.exe`
(실행 중이던 구버전은 교체/종료하지 않았다).

펌웨어:
`artifacts/s-native-v6-2-6/firmware-v72/s-native-v6-2-6-v72.bin`
327164 bytes, SHA256 `c3a2995442d931bd0b5ba2500a2407ab62a50177fc4282195ea79ece85e8864a`.
OTA 슬롯320KiB 중 **516 bytes**만 남았다. 기능 추가 전 용량을 다시 확인한다.

동일 시뮬레이션 4시점, **2초 정지 + 8초 전진 + 4초 STOP/S 복귀**:
[영상](../artifacts/s-native-v6-2-6/video/four-views.mp4).
질량2.754kg, 쿠션D37.3×27mm, 기울기, 위상, 발 하중/높이, 뒷발 J3/X, STOP 단계를 포함한다.

```powershell
python tools/generate_locomotion_profiles.py --check
python tools/generate_gait_speed_labels.py --check
python -m pytest -q simulation/mujoco/tests/test_s_native_v626.py simulation/mujoco/tests/test_servo_profile.py apps/windows/tests
python simulation/mujoco/scripts/validation/check_firmware_windows.py --output artifacts/s-native-v6-2-6/host-safety
python simulation/mujoco/scripts/analysis/analyze_s_native_hesitation.py --profile s_native_v6_2_6 --seconds 30 --voltage 12.6 --output artifacts/s-native-v6-2-6/recheck.json
python scripts/analysis/compare_servo_reference.py --output artifacts/s-native-v6-2-6/replay.json
```

Windows Python은 이번 PC에서 `C:/Users/etnlw/miniforge3/envs/spot_omg/python.exe`를 사용했다.
다른 PC에서는 경로를 바꾼다. MuJoCo 3.11.0이며 SciPy/OSQP는 기존 추가 회귀에,
Matplotlib는 비교 그림에 사용했다. 모터 모델 수치는 추정값이다.

## 다음 진행 순서

1. 충전 및 현재 물리적 준비 상태를 새로 확인한다. 이전 날짜의 지지/연결 해제 답변을 재사용하지 않는다.
2. v72 설치가 허용되는 지지 상태에서 Relax/torque OFF 확인 후 설치하고 revision/caps/profile을 readback한다.
   시작 시 12개 서보 실제 속도/가속도 readback과 앞 J1 방향/J2 누적 목표를 확인한다.
3. 발이 바닥에 닿지 않는 몸체 고정 짧은 동작부터 V625/V626을 비교한다.
   위상 rate, 실제 왕복량, 지연, 전압, STOP→S 완료를 기록한다.
   `$BATTERY` 수신/경고도 확인한다. 설정 readback과 실제 위치 유지/전체 동작을 구분해 보고한다.
4. 바닥 시험 전 앞발 회수 접촉과 지지 보정을 해결한다. 단순 들기 증가 실패를 되풀이하지 말고
   몸체 피치·다리 실제 X속도·하중·늦은 착지를 함께 분석한다. 기울기 보호/관절 제한/충돌을 끄지 않는다.
5. 기존 V625/V626을 덮어쓰는 새 보행 정책은 만들지 않는다. 새로운 정책은 버전 이름으로 추가하고
   앱 기본값/목록과 펌웨어 인덱스 보존 규칙을 따른다.

`hardware/power-junction/`의 별도 작업은 수정하지 않았다.
