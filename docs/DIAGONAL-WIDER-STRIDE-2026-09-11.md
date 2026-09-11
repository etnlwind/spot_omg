# 대각선 분리 정책 보폭 확대 — 2026-09-11

현재 60mm 정책을 보존하고 80mm(+33%)와 100mm(+67%)를 추가했다. 보폭 매개변수만 변경했다. J1 허용·지지/스윙 보정·주기 1.44초·지지율 0.5·발 들림 20mm·명목 몸체 높이 매개변수 201.75mm·쿠션·물리·안전정지 조건은 동일하다. 실제 몸체 높이는 별도 측정값이며 명목 높이 매개변수와 다르다.

## 20초 시험

| 보폭 | 최초 안전정지 | 최대 roll | 최대 pitch |
|---|---|---:|---:|
| 80mm | 없음 | 7.27° | 3.25° |
| 100mm | 8.98초 기울기 | 12.93° | 5.58° |

100mm는 약 8.98초에 보호 정지했으므로 80mm를 추가 비교했다. 80mm도 기존 안정성 합격 기준(최대 roll/pitch 각 3° 이하)을 통과하지 못했다. 둘 다 `experimental_validation_failed`이며 실기나 기본 정책으로 승격하지 않았다.

## 파일

- 정책: `cushion_diagonal_sync_wide80`, `cushion_diagonal_sync_wide100` (`upright_profiles.json`)
- 결과: `diagonal-split-wide80.json`, `diagonal-split-wide100.json`
- 영상: `wide80-review-comparison.mp4`, `wide100-review-comparison.mp4` (위 60mm, 아래 확대 후보)
- 모든 영상은 자유 몸체 물리 시뮬레이션이며 몸체 높이를 따라가지 않는 카메라로 촬영했다.

## 별도 실행

```sh
/opt/anaconda3/envs/spot_omg/bin/mjpython simulation/mujoco/virtual_robot.py --viewer --no-ble --no-video --host 127.0.0.1 --port 8875 --experimental-profiles simulation/mujoco/upright_profiles.json --profile cushion_diagonal_sync_wide80 --foot-cushion simulation/mujoco/foot_cushion_10mm.json
```

W: 8초 보행, Space: Stop. 100mm를 보려면 profile 이름 끝을 wide100으로 바꾼다. 현재 앱용 서버/앱 메뉴/실기 펌웨어는 이번 변경으로 교체하지 않았다.

## 80mm 60초 확인

65.02초 실행 중 5초 이후 60초를 평가했다. 안전정지 없음. 최대 roll 7.91°, pitch 3.33°, RMS roll 2.70°, pitch 1.15°. 높이 변동 10.74mm, 표준편차 2.26mm. 안전정지가 없다는 것은 전체 안정성 기준 합격과 다르다. `diagonal-split-wide80-60s.json`에 저장했다. 실기 시험은 수행하지 않았다.

## 80mm 대각선 동기 재검증

20.02초를 재현하여 5초 이후 각 쌍의 완전한 스윙 10회를 비교했다. 명령 스윙 위상은 같지만 착지 절대 시차 중앙값은 FL–RR 170ms, FR–RL 200ms, 최대는 둘 다 220ms였다. 접촉은 0.2N 기준 3연속 표본으로 판정했다(20ms 간격).

15.32~16.02초 스텝: FL 최대 여유 36.56mm / RR 6.64mm, RR 15.76초 / FL 15.98초 접촉. 5.96~6.66초 스텝: FR 43.47mm / RL 3.86mm, RL 6.38초 / FR 6.60초 접촉. 뒷발의 낮은 여유와 조기 접촉이 양쪽 대각선에 남았다. FL–RR 높이 차이 RMS 16.32mm, FR–RL 22.39mm.

60초 안전정지 없는 보행은 동기 보행 성공을 뜻하지 않는다. 정책에 diagonal_sync_validation.passed=false를 명시했다. 제어 코드를 이번 진단에서 고쳤다는 의미가 아니다. 원자료는 diagonal-split-wide80-sync.json, -sync.csv, -sync-joints.json이다.
