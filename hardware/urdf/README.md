# Spot OMG URDF

이 디렉터리는 Spot OMG의 `URDF (Unified Robot Description Format, 로봇 구조
모델 파일)`와 생성에 필요한 실측 파라미터를 관리합니다.

## 좌표계와 논리 관절

- `X`: 전방
- `Y`: 좌측
- `Z`: 위쪽
- `J1`: Hip Abduction/Adduction (고관절 외전·내전)
- `J2`: Hip Pitch (고관절 전후 회전)
- `J3`: Knee Pitch (무릎 전후 회전)
- `J1=J2=J3=0`: 네 다리가 아래로 곧게 내려간 calibration neutral
  (보정 중립 자세)

네 다리에 같은 논리 각도를 주면 같은 관절 자세가 됩니다. J1 양수는 네
다리를 모두 바깥쪽으로 벌립니다. J2/J3는 앞·뒤 다리 모두 같은 축을 사용하여
양수 자세에서 Knee Flexion (무릎 굽힘)이 몸체 뒤쪽 `-X`를 향합니다. 즉 네
무릎 모두 사람 무릎과 반대인 Knee-Backward Configuration (후방 굽힘 무릎)을
사용합니다. 실제 서보 장착 방향과 raw tick 부호는
`tools/servo_tool/config/joints.json`에서만 처리합니다.

## 파일

- `spot_omg_parameters.json`: 치수, 질량, 구동기와 관절 제한의 단일 원본
- `generate_urdf.py`: 파라미터로 `spot_omg.urdf` 생성
- `spot_omg.urdf`: Isaac Sim에 가져올 생성 결과
- `validate_urdf.py`: 링크 트리, 12개 관절축, 관성값 검증

현재 형상은 CAD mesh가 아니라 box/sphere primitive (기본 충돌 형상)입니다.
2026-08-04에 다시 실측한 J1 축 간격 `500 x 75 mm`, J1-J2 `50 mm`,
J2-J3 `140 mm`, J3-foot center (J3-발 중심) `150 mm`를 적용했습니다.
J1 축 간격은 임시 body box 외형에도 동일하게
사용했으므로 실제 몸체 외곽 치수를 얻으면 별도로 교체해야 합니다.

아래 값은 아직 실제 로봇을 측정한 뒤 반드시 교체해야 합니다.

- 실제 몸체 외곽 길이·너비·높이
- 몸체와 각 링크의 질량
- 몸체 Center of Mass (무게중심)
- 발 반지름과 바닥 마찰계수

배터리 질량 `0.600 kg`은 실측값으로 기록했습니다. 배터리 크기와 몸체 중심
기준 장착 위치 `(X, Y, Z)`를 측정한 뒤 별도 fixed link (고정 링크)로 URDF에
추가합니다. 위치가 확정되기 전에는 무게중심을 왜곡하지 않도록 물리 질량 합계에
포함하지 않습니다.

## 생성 및 검증

저장소 루트에서 실행합니다.

```bash
conda activate spot_omg
python hardware/urdf/generate_urdf.py
python hardware/urdf/generate_urdf.py --check
python hardware/urdf/validate_urdf.py
```

## Servo ID와 URDF Joint

| Leg | J1 | J2 | J3 |
|---|---|---|---|
| FL | `fl_j1` / ID 1 | `fl_j2` / ID 2 | `fl_j3` / ID 3 |
| FR | `fr_j1` / ID 4 | `fr_j2` / ID 5 | `fr_j3` / ID 6 |
| RL | `rl_j1` / ID 7 | `rl_j2` / ID 8 | `rl_j3` / ID 9 |
| RR | `rr_j1` / ID 10 | `rr_j2` / ID 11 | `rr_j3` / ID 12 |

## Isaac Sim 가져오기

1. Isaac Sim에서 `File > Import` 또는 URDF Importer를 엽니다.
2. `spot_omg.urdf`를 선택합니다.
3. `Fix Base`는 끄고 floating base (부유 베이스)로 가져옵니다.
4. `Merge Fixed Joints`는 켜도 되지만 `imu_link`를 별도 센서 프레임으로 유지하려면
   끕니다.
5. 관절 drive는 position control (위치 제어)로 설정합니다.
6. 발 Physics Material (물리 재질)에 static friction `0.9`, dynamic friction
   `0.8`, restitution `0.0`을 초기값으로 적용합니다.

URDF의 effort는 STS3215 12V 모델의 rated torque (정격 토크) 약 `0.981 Nm`,
velocity는 공식 무부하 속도에서 계산한 약 `4.717 rad/s`를 사용합니다. 실제
하중 응답과 통신 지연은 실측 후 Isaac Lab actuator model (구동기 모델)에
반영해야 합니다.

## MuJoCo에서 자세 확인

URDF를 직접 열면 Initial Joint Position (초기 관절 위치)이 모두 0°이므로
`spotctl stand` 자세가 표시됩니다. `spotctl stand45`와 같은 J2 45°/J3 90°
자세로 비교하려면 저장소 루트에서 다음을 실행합니다.

```bash
conda activate spot_omg
python simulation/mujoco/preview_pose.py stand45
```

프로젝트는 별도 `.venv`를 사용하지 않습니다. 환경이 없다면 저장소 루트에서
`conda env create -f environment.yml`로 생성합니다. macOS에서 실시간 MuJoCo
Viewer가 필요한 명령은 같은 환경의 `mjpython`으로 실행합니다.
