# V6.1 — 뒤쪽 도달 거리 65mm

최신 실기 진단: [앞 J1 두 개의 방향 변환 v60](../../docs/S-NATIVE-V61-FRONT-J1.md).
`front-j1-diagnosis-2026-09-15/`에 이전/수정 변환 비교와 검증 결과가 있다.

실기 후속: 2026-09-15 STM32 v59 설치 및 버전/모델 readback 완료.
[실기 이식·설치 기록](../../docs/S-NATIVE-V61-FIRMWARE.md)과
`hardware-deployment-2026-09-15/` 로그를 참고한다. 아래 수치와 영상은 시뮬레이션 검증이다.

V6 정의와 결과를 보존하고 `s_native_v6_1`을 추가했다. S 기준 앞쪽 +20mm는
유지하고 뒤쪽을 −60mm에서 −65mm로 5mm 늘렸다. 총 X 범위는 80→85mm다.
발 들기 12mm, 최대 입력 주기 1.2초, 첫 FR의 2배 오므림 및 9°/18° 제한은 V6와 같다.
대각선의 기준 X/Z와 타이밍, S 복귀 및 물리 한계는 유지한다.

70mm 후보는 최대 입력에서 기울기가 21.20°로 증가하여 채택하지 않았다.
`candidate-70mm/`는 이 폐기 후보의 당시 기록이며 최종 V6.1 결과가 아니다.
최종 정의는 65mm이고, 루트의 `summary.json`과 영상은 이 정의로 재생했다.

## 동일한 2초 정지 + 8초 보행 비교

| 모델 / 입력 | 최대 기울기 | X 변위 | Y 변위 | 전도 |
|---|---:|---:|---:|---|
| V6 / 100% | 3.44° | 0.835m | 0.222m | 없음 |
| V6.1 / 30% | 3.32° | 0.137m | 0.093m | 없음 |
| V6.1 / 60% | 2.88° | 0.420m | 0.132m | 없음 |
| V6.1 / 100% | 3.21° | 0.880m | 0.267m | 없음 |

전진 거리는 약 5.4% 늘었지만 옆 방향 편류도 증가했다. 저속 뒷발 들림 부족은
계속 남아 있다. 한 조건의 거리 증가를 전체 보행 성능 향상이나 실기 검증으로 해석하지 않는다.
실측 총질량 2.754kg, D37.3×27mm 쿠션, 추정 접촉 물성·서보 모델을 사용했다.
전도 관찰용 `--allow-fall` 재생이며, 일반 앱 실행의 기울기 보호는 유지된다.

출발 0.3초/1.2초 뒤 Stop은 모두 S로 복귀했다. 첫 스윙 51개 목표 자세에서
FR J2/J3 대 몸체 네 부품의 FCL 삼각형 검사는 충돌 없이 최소 2.106mm였다.
전체 동작/모든 링크/제작 공차 검증은 아니다. 호스트 검사 69개 통과.
실기 readback·위치 유지·전체 구동과 Windows에서 불가능한 Xcode 빌드는 수행하지 않았다.

## 재현

```powershell
python simulation/mujoco/scripts/validation/validate_s_native_v6.py --profiles s_native_v6_1 --output artifacts/s-native-v6-1/recheck
python -X utf8 simulation/mujoco/scripts/analysis/capture_s_native_balance.py --profile s_native_v6_1 --command 1000 --allow-fall --walk-seconds 8 --settle-seconds 0 --fourth-view side --output artifacts/s-native-v6-1/video-2s-wait-8s-walk
```

영상에 모델, 입력, 시간, 위상, 자세, 발별 하중/높이, 질량/쿠션 치수를 표시했다.
인코더가 PATH에 없으면 `--ffmpeg`로 경로를 지정한다.
앱 목록 맨 위와 지원되는 시뮬레이터의 기본 선택은 V6.1이며 V6로 다시 전환할 수 있다.

## 정지 후 최초 S 자세 복귀

`video-stop-return-s/four-views.mp4`는 2초 대기 + 8초 보행 + 4초 정지/복귀 영상이다.
10.00초에 실제 제어기의 `@S` 명령을 보내고 추가 보행 명령을 중단했다.
기존 감속·오므림 해제·S 전환을 거쳐 12.18초에 S 목표 복귀가 완료되었다.
마지막 목표 관절 오차는 0°, 시뮬레이션 실제 관절의 최대 오차는 0.990°다.
전도 없이 정지했으며 물리 상태나 월드 위치를 초기화하지 않았다.
영상에 WAIT, 보행, STOP / RETURN TO S, S HOLD와 12개 관절의 최대 S 오차를 표시한다.
실제 로봇에서의 복귀 시험은 수행하지 않았다.

캡처 도구는 이제 기본 4초 복귀 구간을 기록한다. 기존 보행만 촬영은
`--settle-seconds 0`, 복귀 촬영은 `--settle-seconds 4`를 사용한다.
정지 로직 자체는 이미 S로 복귀하므로 이번 변경은 캡처의 정지 명령과 검증 기록에 적용했다.
