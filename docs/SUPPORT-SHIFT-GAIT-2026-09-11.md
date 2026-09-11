> **후속 실험 상태 (2026-09-11):** 아래 V1 수치와 영상은 `cushion_support_shift_v1` 보관 설정의 결과입니다. 현재 `cushion_support_shift`는 지연 IMU·엔코더로 이륙 위치를 읽는 `footstep_tracker.py` 실험으로 변경됐으며, 아래 V1 성능을 계승했다고 볼 수 없습니다. 보폭 60/100/140mm × 주기 1.8/2.4/3.2/4.8초 × 높이 파라미터 230/240mm의 24개 물리 시험은 모두 기울기 정지가 발생했습니다. 현재 실행 중인 GUI는 재시작 전 V1 코드일 수 있습니다.
>
> `analyze_stride_ik.py`와 `preview_stride_ik.py`는 별도의 **수평 몸체 기하 계산/미리보기**입니다. 중력·접촉 동역학으로 수평을 달성한 영상이 아닙니다. 관절 CSV, 그림, 영상은 `artifacts/upright/2026-09-11/cushion/stride-*`에 저장합니다. 물리 시험 결과는 `footstep-tracker-search.json`이며, 최적 보행 또는 검증 통과 정책으로 승격하지 않았습니다. 실물 펌웨어는 변경하지 않았습니다.

# 큰 스텝 · 지지 전환 보정 — 2026-09-11

**결론: 실험 정책 구현 및 비교 검증. 합격 실패. 실물 펌웨어 변경 없음.**

`cushion_support_shift`를 기존 정책과 별도로 추가했다. 앱 표시는 **큰 스텝 · 지지 전환 보정 · 실험 (검증실패)**이다. 기본 주기 4.8초, 보폭 명령 140mm를 유지한다. 보폭 자동 축소를 사용하지 않는다.

## 구현 범위

- 이륙 전 0.2초 준비, 착지 후 0.2초 해제하는 주기적 smootherstep 창. 다음 대각 지지선에 대한 몸체 기준 이동은 좌우 최대 5mm, 하강 최대 10mm이며 코드 상한은 각각 10mm이다.
- 승인한 37.3mm 원형 밑면·27mm 길이 탄성 쿠션의 CAD mesh를 그대로 사용한다. 발 중심 XY와 mesh 최저점 Z, 세 관절 Jacobian을 계산한다. 전체 몸체 기준 변환을 네 발에 공통 적용하고, 지지 발 등식부터 각 다리의 세 관절을 함께 푼다. 고정된 몸체 기준에서 다리 Jacobian 블록은 독립적이다. 자유 몸체의 동역학을 최적화하는 전체 WBC는 아니다.
- 지지 구간의 기본 궤적은 직선, 스윙은 속도 경계가 이어지는 매끄러운 아치다. 양 대각 발의 계획 지면 여유는 공통 기준면에서 계산한다. 실제 동시 이륙은 아직 달성하지 못했다.
- 오프라인 7차례 시뮬레이션에서 보행 위상별 roll/pitch Fourier 계수를 학습했다. 마지막 미시험 업데이트는 별도 보관하고 실제 시험한 계수만 정책에 넣었다. `fit_support_shift.py`는 동일 절차를 재현한다.
- 온라인에는 지연된 BNO055 신호와 40ms 지연·4096틱 양자화 관절 피드백만 공급한다. IMU 잔여 자세 오차를 발 좌표 보정 후 IK로 변환한다. 관절 피드백은 큰 추종 지연에서 보정량을 제한하는 데 사용하며, 완전한 접촉/몸체 위치 추정기는 아니다. 센서 누락 시 잔여 보정을 제거하며 기존 IMU 안전 정지를 유지한다.
- 실제 MuJoCo 접촉력·몸체 정답 자세는 오프라인 학습과 평가에만 쓴다. 실행 컨트롤러는 별도 정적 MjData를 사용한다. torque MPC/WBIC, 힘 제어, InEKF를 구현했다고 주장하지 않는다.
- 기준 이동 10mm는 명령 한도다. 실제 몸체 변위나 수평을 물리적으로 10mm 안에 강제하는 구속조건은 아니다. 해당 한도로 자세 안정화를 보장하지 못한 결과를 실패로 남겼다.

## 7초 기준 구간

동일 시작·명령에서 6.6~7.4초를 20ms 간격으로 기록했다. 7초에서 기존 roll +4.27°, pitch −2.67°, FL 35.7mm / RR 0.9mm를 재현했다. 새 정책은 roll −1.06°, pitch +0.02°, FL 31.3mm / RR 9.9mm였다. 해당 순간은 개선됐지만 대각 두 발 높이가 같지는 않다.

## 60초 직진 비교

총 65.02초 실행, 2초에서 전진 시작, 5~65초의 동일 60초 구간을 측정한다. 발 중심 속도는 쿠션 굴림까지 포함한 **미끄러짐 대용 지표**다.

| 항목 | 기존 큰 스텝 | 지지 전환 보정 | 기준/판정 |
|---|---:|---:|---|
| 최대 roll (°) | 8.5350 | 5.0641 | ≤3 / 실패 |
| 최대 pitch (°) | 4.1733 | 3.4839 | ≤3 / 실패 |
| RMS roll (°) | 3.0130 | 1.9848 | ≤1.5 / 실패 |
| RMS pitch (°) | 1.3973 | 1.1383 | ≤1.5 / 통과 |
| 관절 추종 RMS (°) | 2.0824 | 2.0182 | 악화 없음 / 통과 |
| 관절 추종 최대 (°) | 16.8099 | 16.1748 | 악화 없음 / 통과 |
| 접지 발 중심 속도 (m/s) | 0.0123 | 0.0166 | 악화 없음 / 실패 |
| 측정 이동 속도 (m/s) | 0.0340 | 0.0256 | 합격 속도로 표시하지 않음 |

| 발 | 착지 중심: J1보다 앞 (mm) | 스윙 최대 여유 (mm) | 중간 스윙 접촉 |
|---|---:|---:|---:|
| FL | 30.56 | 67.58 | 6.6% |
| FR | 31.92 | 67.46 | 7.1% |
| RL | -1.11 | 12.50 | 49.6% |
| RR | -4.31 | 14.42 | 49.0% |

앞발 착지 중앙값 ≥30mm는 통과했지만, 뒷발 최대 여유 ≥15mm와 중간 스윙 접촉 <10%를 충족하지 못했다. 안전 정지 없이 전진을 완료했다.

## 조건 변화 비교

| 조건 | 기존 최대 roll/pitch (°) | 새 최대 roll/pitch (°) | 새 정책 기준 미달 항목 |
|---|---:|---:|---|
| forward | 8.53 / 4.17 | 5.06 / 3.48 | tilt_max, tilt_rms, clearance, middle_swing_contact, stance_motion_not_worse, torque_feasible |
| com_offset | 8.27 / 4.12 | 5.12 / 3.49 | tilt_max, tilt_rms, large_step, middle_swing_contact, torque_feasible |
| servo_delay | 8.71 / 4.22 | 5.26 / 3.78 | tilt_max, tilt_rms, clearance, middle_swing_contact, stance_motion_not_worse, tracking_not_worse, torque_feasible |
| motor_loss | 8.38 / 4.04 | 6.37 / 5.09 | tilt_max, tilt_rms, large_step, clearance, middle_swing_contact, tracking_not_worse, torque_feasible |
| cushion_soft_slippery | 7.67 / 3.59 | 4.65 / 2.40 | tilt_max, tilt_rms, clearance, middle_swing_contact, stance_motion_not_worse, tracking_not_worse, torque_feasible |
| imu_delay | 8.75 / 4.19 | 5.22 / 3.60 | tilt_max, tilt_rms, large_step, clearance, middle_swing_contact, stance_motion_not_worse, torque_feasible |
| forward_stop | 8.53 / 4.17 | 5.05 / 3.02 | tilt_max, tilt_rms, large_step, clearance, middle_swing_contact, stance_motion_not_worse, torque_feasible |

- COM: 배터리 CAD X +25mm, Y −50mm 및 기존 시나리오 명령 지연 40ms.
- 서보 지연: 명령 지연 80ms. 모터 저하: stall torque·무부하 속도 각각 15% 감소.
- 쿠션 변화: 접촉 시간상수 40ms, damping 0.8, 마찰 0.6. IMU 지연: fusion 60ms.
- 전진→정지: 18초에 Stop, 24초까지 확인. 정지 전환 완료, 안전 상태 OK. 표의 실패는 보행 품질 기준을 함께 적용한 결과다.

## 실현 가능성과 제외한 후보

새 정책의 최대 계획/지지 IK 잔차는 0.100mm. 기존 관절 범위와 명령 속도 검사를 통과했다. 최대 명령 속도 92.8°/s, 모터 사양 속도 대비 34.3%.
시뮬레이션 토크는 전압·속도 의존 한계에 의해 제한된다. 평균 모터 포화 비율 1.32%, 추정 정격 초과 비율 17.51%로 지속 구동 여유를 입증하지 못했다. 검증기는 양쪽 각각 1% 미만이라는 보수적인 승격 기준을 사용한다. 이는 제조사의 검증된 열 한계가 아니다. 마찰 원뿔 사용률은 거의 1에 도달하므로 여유가 넉넉하다고 볼 수 없다.
질량·토크·마찰·쿠션 물성은 추정치다. 발 접촉 기하 통과, 호스트 시험, 시뮬레이터 보행 통과, 실기 위치 유지/전체 동작 시험은 서로 다르다. 이번에는 실기 조회·구동·펌웨어 설치를 하지 않았다.
더 큰 뒷발 lift(60~72mm)와 스윙/지지 반대 방향 보정 후보는 초기 tilt 안전 정지로 제외했다. 작은 lift 재배분(앞 0.95배, 뒤 1.10배)도 최대 기울기 6.68° 및 뒷발 접촉을 악화시켜 제외했다. `support-shift-world.json`, `support-shift-lifts.json`, `support-shift-small-lift.json`에 실패를 보존했다.

## J2·J3와 발끝 목표에 관한 사용자 관찰

사용자가 관찰한 “발끝 목표를 따라 J2·J3가 움직이는 느낌”을 제어 기준으로 삼는다. J3를 무조건 고정하거나 토크 해제하는 순서를 흉내 내지 않는다. 발끝 목표 위치·속도를 먼저 정하고 J2/J3를 계산하며, 평지 직진에서 J1은 필요한 측방/자세 보정에 사용한다. 사용자가 말한 J3를 “푼다”는 토크 해제가 아니라 발을 내딛고 하중을 받아 추진하는 의미다. 착지 후 J2·J3의 토크와 발끝 지면 상대속도를 함께 평가한다. 영상만으로 특정 상용 로봇의 토크 명령을 판별할 수는 없다.
이번 정책도 발 좌표 기반이지만 실제 지면 기준 발 위치 추정과 하중 전환은 충분하지 않다. 남은 뒷발 끌림을 J1 진폭 증가만으로 해결됐다고 취급하지 않는다. 착지 전 발의 지면 상대속도와 지지 중 신전/하중을 함께 개선해야 한다. 참고: [Kim et al., 2019](https://arxiv.org/abs/1909.06586), 발/몸체 목표와 관절 위치·속도·토크 명령의 분리. 해당 논문의 전체 구현과 현재 위치 서보용 IK는 다르다.

## 재현 및 결과 파일

```sh
/opt/anaconda3/envs/spot_omg/bin/python simulation/mujoco/validate_support_shift.py
/opt/anaconda3/envs/spot_omg/bin/python simulation/mujoco/preview_support_shift.py
/opt/anaconda3/envs/spot_omg/bin/python tools/generate_gait_speed_labels.py
```

GUI / iPhone TCP·Bluetooth·영상:

```sh
/opt/anaconda3/envs/spot_omg/bin/mjpython simulation/mujoco/virtual_robot.py --viewer --host 0.0.0.0 --port 8765 --video-host 0.0.0.0 --video-port 8766 --experimental-profiles simulation/mujoco/upright_profiles.json --profile cushion_support_shift --foot-cushion simulation/mujoco/foot_cushion_10mm.json
```

- [동시 측면·정면 비교 영상](../artifacts/upright/2026-09-11/cushion/support-shift-comparison.mp4)
- [7초 비교 화면](../artifacts/upright/2026-09-11/cushion/support-shift-7.0s.png)
- [새 정책 전체 판정](../artifacts/upright/2026-09-11/cushion/support-shift-validation.json)
- [동일 조건 기존 정책](../artifacts/upright/2026-09-11/cushion/support-shift-baseline.json)
- 같은 폴더의 `support-shift-*-window.json`: 6.6~7.4초 목표·실측 각도, 각속도, 접촉/토크/자세. `support-shift-*.csv`: 각 발 접촉·여유·착지 위치의 시간 기록.
- 앱은 모든 선언된 조건의 합격, 전체 정책·쿠션 일치, 컨트롤러/물리 입력 해시 일치 전까지 속도를 숨기고 `(검증실패)`로 표시한다.

## 앱 및 호스트 검증

기하·센서·Stop·영상 검사 19개, 기존 가상 로봇/전송 회귀 34개 통과. Swift 모든 정책 속도 표시 검사 및 실제 iOS Debug 빌드 통과. 실제 UJIN17에 V0.5.0 (38) 설치 완료. 휴대폰 에뮬레이터는 사용하지 않았다. 연결 검증 상태는 아래에 별도 기록한다.

Mac TCP에서 새 정책 선택·기존 정책 전환·Stop 완료·syncstate OK를 확인했다. 영상은 유휴 상태 첫 요청에 503을 반환한 뒤 재요청에서 새 JPEG와 증가하는 프레임/시뮬레이션 시각을 확인했다. 영상 서버가 요청이 없으면 렌더링을 쉬는 기존 동작이다. `support-shift-live-tcp.txt`, `support-shift-video-check.json` 참조.

사용자가 잠금을 해제한 뒤 실제 UJIN17에서 TCP → 연결 해제 → Bluetooth → 연결 해제 → TCP 전환을 완료했다. 각 연결은 ready=true, pose=stand, motion_active=false 및 simulator revision 수신으로 확인했다. 최종 TCP 192.168.0.62:8765 연결 상태다. `support-shift-iphone-connections.json` 참조. iPhone 화면에서 영상 재생을 눈으로 확인하거나 모든 버튼을 직접 터치한 시험과는 구분한다.


## 지지 후반 추진 / 스윙 제동 추가 분석

사용자의 J3 “내딛기” 설명에 따라, `diagnose_support_propulsion.py`에서 MuJoCo 접촉 좌표 힘을 월드 좌표로 바꾸고 몸체 전진축 성분을 구했다. J2/J3의 기계적 출력은 관절 토크 × 관절 각속도로 기록한다. 이 값들은 평가 전용이며 컨트롤러로 피드백하지 않는다. 동일 24초 실행 중 5~24초를 비교했다.

| 구간 | 기존 평균 전진 방향 힘 (N) | 새 정책 (N) |
|---|---:|---:|
| 지지 전반 | −0.869 | −1.172 |
| 지지 후반 | +2.971 | +2.965 |
| 스윙 예정 구간 | −2.528 | −2.128 |

네 발의 해당 구간 샘플을 평균한 값이다. 몸체 전체 순간 합력과 동일하지 않다. 지지 후반 J2/J3 평균 출력은 새 정책에서 각각 +0.060W/+0.071W였다. 추진이 전혀 없는 것이 아니라, 스윙 접촉에 의한 제동이 함께 나타난다. 토크를 더 주는 것만으로 해결할 근거는 없으며, 발 회수·착지 궤적과 J2/J3 추종을 함께 개선해야 한다. 추진력이 생기는 원리를 기존 위치 기반 시뮬레이터에서 관측한 것이지, 실제 모터 토크/힘 제어를 새로 구현한 것은 아니다.

[집계](../artifacts/upright/2026-09-11/cushion/support-propulsion.json) 및 같은 폴더의 `support-propulsion-*.csv`에 발별 힘·J2/J3 토크·출력을 보존했다.

## 고정 없는 물리 영상과 높이 비교

`preview_stride_physics.py`는 freejoint 몸체를 `Simulation.step`의 실제 토크 및 `mj_step`으로 적분한다. 렌더링 중 몸체 자세나 관절 위치를 강제로 지정하지 않는다. 보폭 60/100/140mm × 높이 파라미터 240/220mm, 동일 4.8초 주기의 10초 영상을 `stride-physics-height-comparison.mp4`에 저장했다. 쿠션·질량·모터 물성은 추정값이다.

이륙 전 3.96초의 실제 COM 높이는 약217→201mm로 감소했으나, 여섯 경우 모두 4.88~5.00초에 기울기 정지가 발생했다. 140mm의 정지 시각은 기존높이4.92초, 낮춘높이4.88초다. 정지 전 목표 IK 잔차는 모든 경우0.171mm 미만이므로 단순 기하 도달 불가만으로 실패를 설명할 수 없다. 두 대각선 발 지지로 전환한 뒤 몸체 기울기와 관절 추종 오차가 커졌다. 낮춘 것만으로 균형 문제가 해결되지는 않았다. 이는 이 제어기·모델·시험 조건의 관측이며 모든 높이의 안정성을 배제하는 결론은 아니다. 상세 기록은 `stride-physics-height-trace.json`, 요약은 `stride-physics-height-summary.json`이다.

## J1 고정 목표 / J2·J3 전용 IK 비교

사용자 요청에 따라 `cushion_j1_fixed` 실험 프로필을 추가했다. `footstep_tracking.lock_j1=true`이면 기준 서기 자세의 네 J1 목표각을 유지하고, IK Jacobian의 J2/J3 열만 사용한다. 계산 후 J1만 덮어쓰는 방식이 아니다. 남는 3차원 위치 오차도 그대로 기록한다. J1의 실제 위치를 물리 엔진에 강제로 고정하지 않으며 기존 서보 토크·속도·지연 모델이 유지한다. 몸체는 freejoint, 기존 안전 정지 조건도 유지한다.

`preview_j1_fixed_physics.py`로 동일 보폭140mm·높이 파라미터240mm·주기4.8초의 8초 물리 비교를 촬영했다. 왼쪽 기존 / 오른쪽 J1 고정. 영상은 `artifacts/upright/2026-09-11/cushion/j1-fixed-physics-comparison.mp4`다.

- 정지 전 네 J1 목표 변화폭: 모두0°. 실제 J1 변화폭: 약0.99~1.82°(서보의 하중 추종 오차 포함).
- 최초 기울기 정지: 기존4.92초 / J1 고정4.76초. 이 조건에서는 개선되지 않았다.
- 정지 전 최대 목표 IK 잔차: 기존0.170mm / J1 고정17.174mm. 두 관절만으로 기존의 모든 3차원 발 위치 요구를 만족시키지는 못했다.
- 회귀 검사15개 통과. 물리 보행 검증은 실패이며 실물 펌웨어·실물 로봇에는 적용하지 않았다.
