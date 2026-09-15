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
