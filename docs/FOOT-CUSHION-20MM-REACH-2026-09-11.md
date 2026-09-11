# 2cm 쿠션 및 앞쪽 착지 비교

사용자 요청에 따라 기존10mm에10mm를 추가한20mm 설정을 별도 파일로 만들었다.
`foot_cushion_20mm.json`: 두께20mm,가정 반지름15mm,질량10g/발.
재질의 실측 경도가 없으므로 기준 contact time constant20ms를 유지한 비교와
두 겹의 추가 유연성을 가정한28.28ms 조건을 각각 시험했다. 재료 물성의 실측 대응은 아니다.

`upright_reach`는 기존 upright를 보존하고 보폭40→50mm,forward offset −25→−20mm로
조정한 별도 실험 정책이다. 목표 들림30mm,다리 길이240mm,주기1.8초는 유지한다.

60초 전진 결과:

| 조건 | 안전 상태 | 속도 | 최대 기울기 | FL/FR 쿠션 앞끝 착지 위치 중앙값 |
|---|---|---|---|---|
| 기준 접촉 |ok|0.038m/s|8.3°|J1 수직선 앞9.8/9.5mm|
| 더 부드러운 두 겹 |ok|0.044m/s|5.9°|앞11.5/11.1mm|

위 값은 쿠션의 **앞 가장자리**다. 앞발 쿠션 중심은 J1 수직선보다 뒤7~9mm에 남아 있다.
중심 자체가 앞에 놓이도록 크게 이동한 후보는 추진력이 거의 없어지거나 흔들려 채택하지 않았다.
접촉 재발도 착지 사건에 포함되므로 모든 걸음의 최초 착지를 보장하는 수치가 아니다.
뒷발 들림과 접촉 잔류가 남아 실기 배포 후보로 승인한 상태는 아니다.

영상: `artifacts/upright/2026-09-11/cushion/upright-reach-20mm.mp4`.
후보/결과: 같은 폴더의 reach20-search/stride/fine/validation.json.

```bash
mjpython simulation/mujoco/preview_upright.py --viewer --profile upright_reach \
  --foot-cushion simulation/mujoco/foot_cushion_20mm.json
```

## 사진 확인 후 판단

사용자 사진 IMG_3565.HEIC에서는 흰 쿠션이 발끝을 넓게 감싸며 모서리가 둥글다.
사진으로 실제 치수/재료를 확정하지 못했다. 현재 타원체 외곽은 근사이며
사진의 캡/패드 형상을 정확히 재현한 모델은 아니다.
실기 기본 조건은1cm 한 겹부터 권장하고2cm는 비교용 실험으로 유지한다.
두 겹의 전단 변형/층간 접착/카펫 섬유 걸림은 현재 모델로 검증하지 않았다.
실제 로봇/펌웨어는 변경하지 않았다.
