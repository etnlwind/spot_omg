# Attitude PD profile, CLI and app integration

The separate `attitudepd` profile uses manifest ID 15. Existing `centerpivot` remains ID 14, and the default remains `cruise`. Its nominal period, parameters, turn/reverse parameters and turn input limit match `centerpivot`. `locomotion_targets` dispatches both to the same nominal center-pivot kinematics; the stabilizer itself is integrated separately.

The app offers “IMU 자세 안정화 · PD · 실험 (검증실패)” only when runtime capabilities include `attitudepd`. Its measured speed is nil until physics validation passes. Advertising this capability does not select it automatically. No existing profile was removed.

`spotctl stabilize on`, `off`, `status` (or bare `stabilize`) route to `@B 1`, `@B 0`, `@B 2`. These use the realtime console lane and bypass ordinary echo sync and log-clock queries that block behind a running firmware motion loop. A success requires a `$STABILIZE ` acknowledgement plus the console prompt; an unrelated prompt is insufficient. Firmware errors remain errors. Existing BLE app-control lease behavior is unchanged.

## Verification

- 52 CLI/console tests and 25 subtests passed (`cli-tests.log`).
- 15 new profile/CLI tests passed (`cli-profile-tests.log`), including mocked TCP, BLE and STM32 routing, fragmented acknowledgement, stale prompt, missing acknowledgement timeout, and firmware error handling.
- Independent host C test compared both profile IDs at 2,121 command/phase/scale combinations; joint outputs and command periods were exactly equal. Manifest default and numeric IDs were checked.
- Profile and gait-speed generated-file check modes passed.
- Unsigned iPhone SDK Debug app build passed (`ios-build.log`).
- Unsigned iPhone SDK app and XCTest target build-for-testing passed (`ios-test-build.log`). XCTest was compiled, not run on a device.

No app was installed or launched, no phone emulator was started, and no servo motion/firmware update was performed. Runtime TCP/Bluetooth with the parent-integrated stabilizer, physical readback, position holding, and full motion remain separate validation levels. The first BLE routing unit-test attempt reached the existing app-control discovery wrapper and failed before opening the mocked console; subsequent tests explicitly disabled app-control discovery and stayed isolated.
