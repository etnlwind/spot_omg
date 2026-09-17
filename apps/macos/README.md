# Spot OMG Mac 앱

아이폰 앱과 같은 SwiftUI 소스를 Mac Catalyst로 빌드한다. Xcode 프로젝트에서
Catalyst를 지원하므로 임시 복사본의 프로젝트 설정을 수정할 필요가 없다.
배터리 경고도 아이폰과 같은 정책과 화면을 사용한다.

```sh
xcodebuild -project apps/ios/SpotOMGController/SpotOMGController.xcodeproj \
  -scheme SpotOMGController -destination 'platform=macOS,variant=Mac Catalyst' \
  -derivedDataPath artifacts/mac-app CODE_SIGNING_ALLOWED=NO build
open artifacts/mac-app/Build/Products/Debug-maccatalyst/SpotOMGController.app
```

[충전 경고 동작·검증·펌웨어 적용 범위](../../docs/APP-BATTERY-WARNING-2026-09-15.md).

## V77-T1 파라미터 보행 시험

새 **파라미터 보행 시험** 패널에서 들림12–40mm, 전진 입력1–1000, 시간500–30000ms, 대상all/RL/RR을 설정합니다.
V6.2.5 선택 → Stand → 설정 적용 + 조회 → 적용값 확인 → 시험 시작 순서입니다. Stop으로 중단합니다.
지원 펌웨어는 `s-native-v6-2-7-v77-t1-param` 및 `s-native-v6-2-7-v77-t1-param-j1`입니다.
일반 조이스틱에는 이 설정이 적용되지 않으며 시험 중 조이스틱은 시험값을 덮어쓰지 않습니다.
설정 조회 불일치·보호 오류·Stand가 아닌 상태에서는 시작하지 않습니다.
