# Upright — 높은 자세와 발 들림 전진 실험

실제 로봇에 적용하지 않은 MuJoCo 실험 정책이다. 기존 보행 정책과 기본 선택값은 보존했다.
관절 숫자를 크게 만드는 대신 다리를 더 곧게 펴는 의미로 해석했다.

## 선택한 설정

- 발끝의 몸체 기준 수직 길이: 240mm (기존 Cruise 201.75mm).
- 중립 목표: J1 0.75°, J2 41.14°, J3 68.00°. Stand 기준은 J2 45°, J3 90°.
  이 좌표계에서 다리를 펴면 J2/J3 숫자는 작아진다.
- 목표 발 들림30mm, 보폭40mm, 주기1.8초, 지지 비율0.60, 전후 offset −25mm.
- 기존 C 발끝 궤적/IK, level 기반 IMU 보정, BNO055 지연 모델, 전압/서보 토크 한계,
  CAD 질량/관성/접촉 모델을 사용한다. 실험 파라미터는 Python 실험 실행 경로를 사용하며
  STM32 전체 제어 루프와 동일하게 배포된 정책이라는 의미는 아니다.
- 정책 파일: `simulation/mujoco/upright_profiles.json`, 이름 `upright`.

## 결과와 제한

60초 전진 명령 시험(초기2초 정착 포함):

| 지표 | 값 |
|---|---:|
| 이동 속도(평면 변위/시간) | 약0.034m/s |
| FL / FR 최대 실제 발 여유 | 29.8 / 29.1mm |
| RL / RR 최대 실제 발 여유 | 39.2 / 40.6mm |
| 최대 roll/pitch 절댓값 | 약5.2° |
| 앞발 swing 중간20~80% 접촉 비율 | 약4~5% |

발 높이는 목표값이 아닌 MuJoCo 구형 발 접촉 형상의 최저점과 지면 사이 거리로 측정했다.
최대 여유이므로 모든 스텝에서30mm가 보장된다는 뜻은 아니다. speed는 누적 경로 속도가 아닌
측정 구간 시작/끝의 평면 변위다. 후진/회전 속도와 직접 비교할 때 주의한다.

기존 Cruise14초 기준에서는 앞발 최대 여유15.5/10.6mm, 중간 swing 접촉16/37%였다.
서로 측정 길이가 다르므로 엄격한 동일 조건의 정량 비교가 아닌 참고값이다.

- 후진·좌우 회전에서도 안전 정지는 없었으나 앞발 끌림이 남았다. 전방 발 들림 실험으로 분류한다.
- 질량 증가/낮은 전압 조건에서 앞발 여유19~21mm로 감소했다.
- 배터리 위치 변화/40ms 지연 조건에서 앞발 접촉이 늘고 속도가0.016m/s로 감소했다.
- 카펫 섬유 걸림이나 부착한 의자용 고무의 실제 치수/경도는 모델링하지 않았다.
  따라서 카펫 실기 성공으로 해석하면 안 된다. 전원/서보 실측이나 실기 보행을 하지 않았다.

## 재생

자동24초 전진/정지 데모(물리 로봇 IO 없음):

```bash
cd /Users/etnlwind/project/spot_omg
/opt/anaconda3/envs/spot_omg/bin/mjpython simulation/mujoco/preview_upright.py --viewer
```

지속적인 MuJoCo 조작 화면(W:8초 전진, Space:중지), 별도 로컬 포트:

```bash
/opt/anaconda3/envs/spot_omg/bin/mjpython simulation/mujoco/virtual_robot.py \
  --viewer --no-ble --no-video --host 127.0.0.1 --port 8875 \
  --experimental-profiles simulation/mujoco/upright_profiles.json --profile upright
```

영상 생성:

```bash
/opt/anaconda3/envs/spot_omg/bin/python simulation/mujoco/preview_upright.py \
  --video artifacts/upright/2026-09-11/upright.mp4
```

결과는 `artifacts/upright/2026-09-11/`의 search/slow-search/controller-search/refined/validation.json,
PNG,24초MP4에 저장했다. 높은 자세와 큰 보폭을 동시에 적용하다 실패한 후보도 기록했다.
`test_upright_profile.py`에서 기존 정책 보존,중립 길이/발 들림,실제 접촉 회귀를 검사한다.
