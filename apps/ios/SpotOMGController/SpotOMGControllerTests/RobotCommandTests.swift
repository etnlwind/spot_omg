import XCTest
import SwiftUI
import Network
@testable import SpotOMGController

final class RobotCommandTests: XCTestCase {
    func testV8RequiresAdvertisedCapability() {
        XCTAssertFalse(SimulatorGaitProfile.attitudepd_v8.isSupported(capabilities:["attitudepd_v7"]))
        XCTAssertTrue(SimulatorGaitProfile.attitudepd_v8.isSupported(capabilities:["attitudepd_v8"]))
    }
    func testV7NavigationLabelsAndCapability() {
        XCTAssertEqual(RobotDriveVector.make(x:1,y:0)?.navigationTitle,"오른쪽 옆걸음")
        XCTAssertEqual(RobotDriveVector.make(x:-1,y:0)?.navigationTitle,"왼쪽 옆걸음")
        XCTAssertEqual(RobotDriveVector.make(x:0.7,y:-0.7)?.navigationTitle,"제자리 우회전")
        XCTAssertEqual(RobotDriveVector.make(x:-0.7,y:-0.7)?.navigationTitle,"제자리 좌회전")
        XCTAssertEqual(RobotDriveVector.make(x:0,y:-1)?.navigationTitle,"후진 · 보행 속도 50%")
        XCTAssertFalse(SimulatorGaitProfile.attitudepd_v7.isSupported(capabilities:["attitudepd_v6"]))
        XCTAssertTrue(SimulatorGaitProfile.attitudepd_v7.isSupported(capabilities:["attitudepd_v7"]))
    }
    func testLongStandWaitsForPromptBeforeRefreshAndAllowsDriveAndFootLift() {
        var sent: [String] = []
        let manager = RobotBluetoothManager { sent.append(String(decoding: $0, as: UTF8.self)) }
        defer { manager.disconnect() }
        manager.send(.stand)
        // Real Stand took 6.8 seconds: its old four-second refresh was discarded.
        RunLoop.main.run(until: Date().addingTimeInterval(4.3))
        XCTAssertEqual(sent, ["stand\n"])
        manager.receiveConsoleText("POSE stand reason=complete\r\nOK\r\n")
        // Mechanical diagnostics continue after OK; wait for the actual prompt.
        RunLoop.main.run(until: Date().addingTimeInterval(0.3))
        XCTAssertEqual(sent, ["stand\n"])
        manager.receiveConsoleText("$MECHLOG label=stand complete=1\r\n# ")
        RunLoop.main.run(until: Date().addingTimeInterval(0.3))
        XCTAssertEqual(sent.last, "syncstate\n")
        manager.receiveConsoleText("$SPOTSTATE pose=stand torque=on safety=ok rev=attitudepd-v4-v90-r2 caps=footlift,footliftpersist,commandretry profile=attitudepd_v4 lift_fl=0 lift_fr=0 lift_rl=0 lift_rr=0\r\n# ")
        if sent.last == "read 1\n" { manager.receiveConsoleText("ID 1 voltage=11800mV moving=0\r\n# ") }
        XCTAssertTrue(manager.canConfigureFootLift)
        manager.updateDrive(x: 0, y: 1)
        XCTAssertTrue(sent.contains { $0.hasPrefix("drive ") })
    }

    func testSlowV80DiagnosticsKeepConnectionUntilPrompt() throws {
        // Actual V80 diagnostic prefix, truncated by the old Windows timeout.
        // Chunk pacing and the final CRLF/prompt below are simulated inputs.
        let url = try XCTUnwrap(Bundle(for: RobotCommandTests.self)
            .url(forResource: "V80StopDiagnostics", withExtension: "txt"))
        let bytes = Array(try Data(contentsOf: url))
        for (chunkSize, interval) in [(20, 0.03), (180, 0.30)] {
            var now = 0.0
            var sent: [String] = []
            let manager = RobotBluetoothManager(commandWriter: { sent.append(String(decoding: $0, as: UTF8.self)) })
            manager.driveCompletionClock = { now }
            defer { manager.disconnect() }
            manager.updateDrive(x: 0, y: 1)
            manager.stopDrive(reason: "gesture-ended")
            let count = sent.count
            for offset in stride(from: 0, to: bytes.count, by: chunkSize) {
                now += interval
                let end = min(bytes.count, offset + chunkSize)
                manager.receiveConsoleText(String(decoding: bytes[offset..<end], as: UTF8.self))
                manager.checkDriveCompletionTimeout()
                XCTAssertEqual(manager.state, .ready)
                XCTAssertEqual(sent.count, count)
            }
            XCTAssertGreaterThan(now, 6)
            manager.updateDrive(x: 0, y: -1)
            XCTAssertEqual(sent.count, count)
            manager.receiveConsoleText("\r\n#")
            manager.updateDrive(x: 0, y: -1)
            XCTAssertEqual(sent.count, count)
            manager.receiveConsoleText(" ")
            // An obsolete timeout callback must not disconnect an idle session.
            now += 60
            manager.checkDriveCompletionTimeout()
            XCTAssertEqual(manager.state, .ready)
            manager.updateDrive(x: 0, y: -1)
            XCTAssertEqual(sent.filter { $0.hasPrefix("drive ") }.count, 2)
            XCTAssertFalse(sent.contains("\u{03}"))
        }
    }

    func testPartialDiagnosticsRenewIdleDeadlineButEmptyDataDoesNot() {
        var now = 0.0
        var sent: [String] = []
        let manager = RobotBluetoothManager(commandWriter: { sent.append(String(decoding: $0, as: UTF8.self)) })
        manager.driveCompletionClock = { now }
        defer { manager.disconnect() }
        manager.updateDrive(x: 0, y: 1)
        manager.stopDrive()
        manager.receiveConsoleText("$SPOTDRIVE stopped reason=ok\r\n")
        for time in [4.0, 8.0, 12.0] {
            now = time
            manager.receiveConsoleText("part of an unfinished diagnostic line ")
            manager.checkDriveCompletionTimeout()
            XCTAssertEqual(manager.state, .ready)
        }
        now = 16.9
        manager.receiveConsoleText("")
        manager.checkDriveCompletionTimeout()
        XCTAssertEqual(manager.state, .ready)
        now = 17.1
        manager.checkDriveCompletionTimeout()
        XCTAssertEqual(manager.state, .disconnected)
        XCTAssertEqual(sent.last, "\u{03}")
        XCTAssertTrue(manager.lastError?.contains("종료 로그") == true)
    }

    func testTelemetryCannotExtendUnconfirmedStopDeadline() {
        var now = 0.0
        var sent: [String] = []
        let manager = RobotBluetoothManager(commandWriter: { sent.append(String(decoding: $0, as: UTF8.self)) })
        manager.driveCompletionClock = { now }
        defer { manager.disconnect() }
        manager.updateDrive(x: 0, y: 1)
        manager.stopDrive()
        for time in [2.0, 4.0, 5.1] {
            now = time
            manager.receiveConsoleText("$BATTERY mv=12000\r\n")
            manager.checkDriveCompletionTimeout()
        }
        XCTAssertEqual(manager.state, .disconnected)
        XCTAssertEqual(sent.last, "\u{03}")
        XCTAssertTrue(manager.lastError?.contains("보행 종료 확인") == true)
    }

    func testContinuousDiagnosticsAndDuplicateStopHaveThirtySecondLimit() {
        var now = 0.0
        var sent: [String] = []
        let manager = RobotBluetoothManager(commandWriter: { sent.append(String(decoding: $0, as: UTF8.self)) })
        manager.driveCompletionClock = { now }
        defer { manager.disconnect() }
        manager.updateDrive(x: 0, y: 1)
        manager.stopDrive()
        manager.receiveConsoleText("$SPOTDRIVE stopped reason=ok\r\n")
        for second in 1...29 {
            now = Double(second)
            manager.receiveConsoleText(second.isMultiple(of: 3)
                ? "$SPOTDRIVE stopped reason=ok\r\n" : "$BATTERY mv=12000\r\n")
            manager.checkDriveCompletionTimeout()
            XCTAssertEqual(manager.state, .ready)
        }
        now = 30.1
        manager.receiveConsoleText("$BATTERY mv=12000\r\n")
        manager.checkDriveCompletionTimeout()
        XCTAssertEqual(manager.state, .disconnected)
        XCTAssertEqual(sent.last, "\u{03}")
    }

    func testPromptCancelsDrainDeadlineBeforeQueuedPosture() {
        var now = 0.0
        var sent: [String] = []
        let manager = RobotBluetoothManager(commandWriter: { sent.append(String(decoding: $0, as: UTF8.self)) })
        manager.driveCompletionClock = { now }
        defer { manager.disconnect() }
        manager.updateDrive(x: 0, y: 1)
        manager.send(.stand)
        manager.receiveConsoleText("$SPOTDRIVE stopped reason=ok\r\n")
        for time in [4.0, 8.0] {
            now = time
            manager.receiveConsoleText("diagnostic fragment ")
            manager.checkDriveCompletionTimeout()
            XCTAssertFalse(sent.contains("stand\n"))
        }
        manager.receiveConsoleText("\r\n# ")
        XCTAssertEqual(sent.last, "stand\n")
        now = 40
        manager.checkDriveCompletionTimeout()
        XCTAssertEqual(manager.state, .ready)
        XCTAssertFalse(sent.contains("\u{03}"))
    }

    func testRealDispatchTimerDoesNotExpireWhileDiagnosticsArrive() {
        var sent: [String] = []
        let manager = RobotBluetoothManager(commandWriter: {
            sent.append(String(decoding: $0, as: UTF8.self))
        })
        defer { manager.disconnect() }
        manager.updateDrive(x: 0, y: 1)
        manager.stopDrive()
        manager.receiveConsoleText("$SPOTDRIVE stopped reason=ok\r\n")
        for _ in 0..<6 {
            RunLoop.main.run(until: Date().addingTimeInterval(1))
            manager.receiveConsoleText("diagnostic fragment ")
            XCTAssertEqual(manager.state, .ready)
        }
        manager.receiveConsoleText("\r\n# ")
        XCTAssertFalse(sent.contains("\u{03}"))
    }

    func testV92SelectsV6OnlyWhenAdvertisedAndWaitsForReadback() {
        var sent: [String] = []
        let manager = RobotBluetoothManager { sent.append(String(decoding: $0, as: UTF8.self)) }
        defer { manager.disconnect() }
        manager.receiveConsoleText("$SPOTSTATE pose=stand torque=on safety=ok rev=attitudepd-v6-v92 caps=gaitprofiles,attitudepd_v4,attitudepd_v5,attitudepd_v6 profile=attitudepd_v5\r\n# ")
        XCTAssertEqual(Array(SimulatorGaitProfile.allCases.prefix(6)), [.attitudepd_v9, .attitudepd_v8, .attitudepd_v7, .attitudepd_v6, .attitudepd_v5, .attitudepd_v4])
        XCTAssertEqual(SimulatorGaitProfile.attitudepd_v6.title, "IMU 자세 안정화 V6 · 복귀 발들림 개선 · 실험")
        XCTAssertTrue(sent.contains("gaitprofile attitudepd_v6\n"))
        XCTAssertEqual(manager.runtimeState.simulationProfile, "attitudepd_v5", "Selection must continue to reflect firmware readback")
        XCTAssertTrue(manager.supportsIMURecovery)
        XCTAssertTrue(manager.supportsProbe)
        XCTAssertTrue(manager.supportsProbeWidth)
        manager.receiveConsoleText("$SPOTSTATE pose=stand torque=on safety=ok rev=attitudepd-v6-v92 caps=gaitprofiles,attitudepd_v4,attitudepd_v5,attitudepd_v6 profile=attitudepd_v6\r\n# ")
        XCTAssertEqual(manager.runtimeState.simulationProfile, "attitudepd_v6")
    }

    func testV91NeverSelectsUnsupportedV6() {
        var sent: [String] = []
        let manager = RobotBluetoothManager { sent.append(String(decoding: $0, as: UTF8.self)) }
        defer { manager.disconnect() }
        manager.receiveConsoleText("$SPOTSTATE pose=stand torque=on safety=ok rev=attitudepd-v5-v91 caps=gaitprofiles,attitudepd_v4,attitudepd_v5 profile=attitudepd_v4\r\n# ")
        XCTAssertTrue(sent.contains("gaitprofile attitudepd_v5\n"))
        XCTAssertFalse(SimulatorGaitProfile.attitudepd_v6.isSupported(capabilities: manager.runtimeState.capabilities))
        sent.removeAll()
        manager.send(.simulatorProfile(.attitudepd_v6))
        XCTAssertFalse(sent.contains("gaitprofile attitudepd_v6\n"))
        XCTAssertTrue(manager.lastError?.contains("업데이트") == true)
    }

    func testV91AdvertisesV5AndOlderFirmwareHidesIt() {
        let manager = RobotBluetoothManager(commandWriter: { _ in })
        manager.receiveConsoleText("$SPOTSTATE pose=stand torque=on safety=ok rev=attitudepd-v5-v91 caps=gaitprofiles,attitudepd_v4,attitudepd_v5 profile=attitudepd_v5\r\n# ")
        XCTAssertEqual(SimulatorGaitProfile.newest, .attitudepd_v9)
        XCTAssertTrue(SimulatorGaitProfile.attitudepd_v5.isSupported(capabilities: manager.runtimeState.capabilities))
        XCTAssertFalse(SimulatorGaitProfile.attitudepd_v5.isSupported(capabilities: ["attitudepd_v4"]))
        XCTAssertTrue(manager.supportsIMURecovery)
        XCTAssertTrue(manager.supportsProbeWidth)
    }

    func testV80AdvertisesV4AndHalfStickMapsToTemplateBoundary() {
        let manager = RobotBluetoothManager(commandWriter: { _ in })
        manager.receiveConsoleText("$SPOTSTATE pose=stand torque=on safety=ok rev=attitudepd-v4-v80 caps=gaitprofiles,attitudepd_v2,attitudepd_v3,attitudepd_v4 profile=attitudepd_v4\r\n# ")
        XCTAssertTrue(manager.supportsProbe)
        XCTAssertTrue(manager.supportsProbeWidth)
        XCTAssertEqual(SimulatorGaitProfile.newest, .attitudepd_v9)
        XCTAssertTrue(SimulatorGaitProfile.attitudepd_v4.isSupported(capabilities: manager.runtimeState.capabilities))
        XCTAssertFalse(SimulatorGaitProfile.attitudepd_v4.isSupported(capabilities: ["attitudepd_v3"]))
        XCTAssertEqual(RobotDriveVector.make(x: 0, y: 0.5)?.linearPerMille, 588)
        XCTAssertEqual(RobotDriveVector.make(x: 0, y: -0.5)?.linearPerMille, -588)
        XCTAssertEqual(RobotDriveVector.make(x: 0, y: 1)?.linearPerMille, 1000)
        XCTAssertEqual(RobotDriveVector.make(x: 0, y: -1)?.linearPerMille, -1000)
    }

    func testV79AdvertisesV3AndKeepsOlderFirmwareCompatible() {
        let manager = RobotBluetoothManager(commandWriter: { _ in })
        manager.receiveConsoleText("$SPOTSTATE pose=stand torque=on safety=ok rev=attitudepd-v3-v79 caps=gaitprofiles,attitudepd_v2,attitudepd_v3 profile=attitudepd_v3\r\n# ")
        XCTAssertTrue(manager.supportsProbe)
        XCTAssertTrue(manager.supportsProbeWidth)
        XCTAssertEqual(SimulatorGaitProfile.newest, .attitudepd_v9)
        XCTAssertTrue(SimulatorGaitProfile.attitudepd_v3.isSupported(capabilities: manager.runtimeState.capabilities))
        XCTAssertFalse(SimulatorGaitProfile.attitudepd_v3.isSupported(capabilities: ["attitudepd_v2"]))
        XCTAssertTrue(SimulatorGaitProfile.attitudepd_v2.isSupported(capabilities: ["attitudepd_v2"]))
    }

    func testV78RetainsParameterModeAndAdvertisesV2() {
        let manager = RobotBluetoothManager(commandWriter: { _ in })
        manager.receiveConsoleText("$SPOTSTATE pose=landing torque=off safety=ok rev=attitudepd-v2-v78 caps=gaitprofiles,attitudepd_v2 profile=attitudepd_v2\r\n# ")
        XCTAssertTrue(manager.supportsProbe)
        XCTAssertTrue(manager.supportsProbeWidth)
        XCTAssertTrue(SimulatorGaitProfile.attitudepd_v2.isSupported(capabilities: manager.runtimeState.capabilities))
        XCTAssertFalse(SimulatorGaitProfile.attitudepd_v2.isSupported(capabilities: ["attitudepd"]))
    }

    func testProbeConfigurationWaitsForPromptThenBecomesAvailable() {
        let manager = RobotBluetoothManager(commandWriter: { _ in })
        manager.receiveConsoleText("$SPOTSTATE pose=stand torque=on safety=ok rev=s-native-v6-2-7-v77-t1-width caps= profile=s_native_v6_2_5\r\n")
        manager.receiveConsoleText("ID 1 voltage=12000mV\r\n# ")
        XCTAssertTrue(manager.canConfigureProbe)
        manager.send(.raw("read 1"))
        XCTAssertFalse(manager.canConfigureProbe)
        XCTAssertEqual(manager.probeConfigurationBlockReason, "로봇 명령 응답을 기다리고 있습니다.")
        manager.receiveConsoleText("ID 1 voltage=11200mV\r\n# ")
        XCTAssertTrue(manager.canConfigureProbe)
        XCTAssertNil(manager.probeConfigurationBlockReason)
    }

    func testProbeConfigParsingAndBounds() {
        let value=RobotProbeConfig.parse("$PROBECONFIG lift_mm=28 linear=344 duration_ms=4000 legs=rl storage=ram")
        XCTAssertEqual(value?.command,"probeconfig set 28 344 4000 rl")
        XCTAssertNil(RobotProbeConfig.parse("$PROBECONFIG lift_mm=41 linear=344 duration_ms=4000 legs=all"))
        XCTAssertNil(RobotProbeConfig.parse("$PROBECONFIG lift_mm=28 linear=344 duration_ms=4000"))
    }
    func testParameterJoystickUsesAppliedConfigAndStopsOnRelease() {
        var commands:[String]=[]
        let manager=RobotBluetoothManager(commandWriter:{ commands.append(String(decoding:$0,as:UTF8.self)) })
        manager.receiveConsoleText("$SPOTSTATE pose=stand torque=on safety=ok rev=s-native-v6-2-7-v77-t1-width caps= profile=s_native_v6_2_5\r\n")
        manager.receiveConsoleText("ID 1 voltage=12000mV\r\n# ")
        manager.configureProbe(RobotProbeConfig(width:0))
        let reply="$PROBECONFIG lift_mm=28 linear=344 duration_ms=4000 legs=all width_mm=0 fr_extra=0\r\n# "
        manager.receiveConsoleText(reply);manager.receiveConsoleText(reply)
        manager.parameterWalking=true;manager.updateDrive(x:0,y:0);manager.updateDrive(x:0,y:1)
        XCTAssertEqual(commands.last,"walkprobe\n")
        manager.stopDrive(reason:"gesture-ended")
        XCTAssertTrue(commands.last?.hasPrefix("@S ")==true)
    }

    func testProbeReadbackAndDedicatedSession() {
        var commands: [String]=[]
        let manager=RobotBluetoothManager(commandWriter:{ commands.append(String(decoding:$0,as:UTF8.self)) })
        manager.receiveConsoleText("$SPOTSTATE pose=stand torque=on safety=ok rev=s-native-v6-2-7-v77-t1-param-j1 caps= profile=s_native_v6_2_5\r\n")
        manager.receiveConsoleText("ID 1 voltage=12000mV\r\n# ")
        XCTAssertTrue(manager.canConfigureProbe)
        manager.configureProbe(RobotProbeConfig())
        manager.receiveConsoleText("$PROBECONFIG lift_mm=28 linear=344 duration_ms=4000 legs=all storage=ram\r\n# ")
        XCTAssertEqual(commands.last,"probeconfig show\n")
        XCTAssertFalse(manager.canStartProbe)
        manager.receiveConsoleText("$PROBECONFIG lift_mm=28 linear=344 duration_ms=4000 legs=all storage=ram\r\n# ")
        XCTAssertTrue(manager.canStartProbe)
        manager.startProbe();XCTAssertEqual(commands.last,"walkprobe\n")
        let count=commands.count;manager.updateDrive(x:1,y:1);XCTAssertEqual(commands.count,count)
        manager.stopWalkingOrHold();XCTAssertTrue(commands.last?.hasPrefix("@S ")==true)
        manager.receiveConsoleText("$SPOTDRIVE stopped reason=ok elapsed=6500ms\r\n# ")
        XCTAssertFalse(manager.probeRunning)
    }
    func testProbeMismatchedReadbackPreventsMotion() {
        var commands:[String]=[]
        let manager=RobotBluetoothManager(commandWriter:{commands.append(String(decoding:$0,as:UTF8.self))})
        manager.receiveConsoleText("$SPOTSTATE pose=stand torque=on safety=ok rev=s-native-v6-2-7-v77-t1-param-j1 caps= profile=s_native_v6_2_5\r\n")
        manager.receiveConsoleText("ID 1 voltage=12000mV\r\n# ")
        manager.configureProbe(RobotProbeConfig());manager.receiveConsoleText("# ")
        manager.receiveConsoleText("$PROBECONFIG lift_mm=20 linear=344 duration_ms=4000 legs=all\r\n# ")
        XCTAssertFalse(manager.canStartProbe);manager.startProbe();XCTAssertFalse(commands.contains("walkprobe\n"))
    }
    func testV621IsNewestSimulatorModelAndPreservesEarlierModels() {
        XCTAssertEqual(SimulatorGaitProfile.newest, .attitudepd_v9)
        XCTAssertFalse(SimulatorGaitProfile.s_native_v6_2_3.simulatorOnly)
        XCTAssertFalse(SimulatorGaitProfile.s_native_v6_2_3.isSupported(capabilities: ["s_native_v6_2_2"]))
        XCTAssertTrue(SimulatorGaitProfile.s_native_v6_2_3.isSupported(capabilities: ["s_native_v6_2_3"]))
        XCTAssertFalse(SimulatorGaitProfile.s_native_v6_2_2.simulatorOnly)
        XCTAssertFalse(SimulatorGaitProfile.s_native_v6_2_2.isSupported(capabilities: ["s_native_v6_2_1"]))
        XCTAssertTrue(SimulatorGaitProfile.s_native_v6_2_2.isSupported(capabilities: ["s_native_v6_2_2"]))
        XCTAssertFalse(SimulatorGaitProfile.s_native_v6_2_1.simulatorOnly)
        XCTAssertFalse(SimulatorGaitProfile.s_native_v6_2_1.isSupported(capabilities: ["gaitprofiles", "s_native_v6_2"]))
        XCTAssertTrue(SimulatorGaitProfile.s_native_v6_2_1.isSupported(capabilities: ["gaitprofiles", "s_native_v6_2_1"]))
        XCTAssertTrue(SimulatorGaitProfile.s_native_v6_2.simulatorOnly)
        XCTAssertFalse(SimulatorGaitProfile.s_native_v6_2.isSupported(capabilities: ["gaitprofiles", "s_native_v6_1"]))
        XCTAssertTrue(SimulatorGaitProfile.s_native_v6_2.isSupported(capabilities: ["gaitprofiles", "s_native_v6_2"]))
        XCTAssertFalse(SimulatorGaitProfile.s_native_v6_1.simulatorOnly)
    }
    func testAttitudePDProfileIsExperimentalAndCapabilityGated() {
        XCTAssertEqual(SimulatorGaitProfile.attitudepd.titleWithSpeed,
                       "IMU 자세 안정화 · PD · 실험 (검증실패)")
        XCTAssertNil(SimulatorGaitProfile.attitudepd.benchmarkSpeedMetersPerSecond)
        XCTAssertFalse(SimulatorGaitProfile.attitudepd.simulatorOnly)
        XCTAssertTrue(SimulatorGaitProfile.allCases.contains(.attitudepd))
        XCTAssertFalse(SimulatorGaitProfile.attitudepd.isSupported(capabilities: ["gaitprofiles", "centerpivot"]))
        XCTAssertTrue(SimulatorGaitProfile.attitudepd.isSupported(capabilities: ["gaitprofiles", "attitudepd"]))
    }

    func testNativeV61RequiresAdvertisedHardwareSupport() {
        XCTAssertFalse(SimulatorGaitProfile.s_native_v6_1.simulatorOnly)
        XCTAssertFalse(SimulatorGaitProfile.s_native_v6_1.isSupported(capabilities: ["gaitprofiles"]))
        XCTAssertTrue(SimulatorGaitProfile.s_native_v6_1.isSupported(capabilities: ["gaitprofiles", "s_native_v6_1"]))
        XCTAssertTrue(SimulatorGaitProfile.s_native_v6.simulatorOnly)
    }

    func testAttitudePDRequiresExplicitSelectionAndSupportedFirmware() {
        var sent: [String] = []
        let manager = RobotBluetoothManager { sent.append(String(decoding: $0, as: UTF8.self)) }
        manager.receiveConsoleText("$SPOTSTATE pose=stand rev=shared-locomotion-v41 caps=gaitprofiles\r\n")
        sent.removeAll()
        manager.send(.simulatorProfile(.attitudepd))
        XCTAssertTrue(sent.isEmpty)
        XCTAssertTrue(manager.lastError?.contains("업데이트") == true)
        manager.receiveConsoleText("$SPOTSTATE pose=stand torque=on safety=ok rev=attitude-pd caps=gaitprofiles,attitudepd\r\n")
        XCTAssertFalse(sent.contains("gaitprofile attitudepd\n"), "Advertising a profile must not select it automatically")
        sent.removeAll()
        manager.send(.simulatorProfile(.attitudepd))
        XCTAssertEqual(sent, ["gaitprofile attitudepd\n"])
        manager.disconnect()
        XCTAssertFalse(SimulatorGaitProfile.attitudepd.isSupported(capabilities: manager.runtimeState.capabilities))
    }

    func testSupportTransitionProfileIsExperimentalAndCapabilityGated() {
        XCTAssertEqual(SimulatorGaitProfile.arcsupport.titleWithSpeed,
                       "원호 턴 · 지지 전환 보정 · 실험 (검증실패)")
        XCTAssertFalse(SimulatorGaitProfile.arcsupport.simulatorOnly)
        XCTAssertFalse(SimulatorGaitProfile.arcsupport.isSupported(capabilities: ["gaitprofiles"]))
        XCTAssertTrue(SimulatorGaitProfile.arcsupport.isSupported(capabilities: ["gaitprofiles", "arcsupport"]))
    }

    func testSupportTransitionCannotBeSentToOldFirmware() {
        var sent: [String] = []
        let manager = RobotBluetoothManager { sent.append(String(decoding: $0, as: UTF8.self)) }
        manager.receiveConsoleText("$SPOTSTATE pose=stand rev=shared-locomotion-v41 caps=gaitprofiles\r\n")
        sent.removeAll()
        manager.send(.simulatorProfile(.arcsupport))
        XCTAssertTrue(sent.isEmpty)
        XCTAssertTrue(manager.lastError?.contains("업데이트") == true)
        manager.receiveConsoleText("$SPOTSTATE pose=stand rev=shared-locomotion-v43 caps=gaitprofiles,arcsupport\r\n")
        sent.removeAll()
        manager.send(.simulatorProfile(.arcsupport))
        XCTAssertEqual(sent, ["gaitprofile arcsupport\n"])
        manager.disconnect()
    }
    func testMissingInitialReplyRetriesReadOnlyAndReportsTimeout() {
        var sent: [String] = []
        let manager = RobotBluetoothManager { sent.append(String(decoding: $0, as: UTF8.self)) }
        manager.requestInitialState(timeout: 0.01)
        RunLoop.main.run(until: Date().addingTimeInterval(0.15))
        XCTAssertEqual(sent, Array(repeating: "syncstate\n", count: 3))
        XCTAssertTrue(manager.lastError?.contains("로봇 응답 없음") == true)
        manager.disconnect()
    }

    func testV14ReplyCancelsRetryAndDisconnectCancelsPendingRequest() {
        var sent: [String] = []
        let manager = RobotBluetoothManager { sent.append(String(decoding: $0, as: UTF8.self)) }
        manager.requestInitialState(timeout: 0.01)
        manager.receiveConsoleText("$SPOTSTATE pose=custom balance=full rev=turn-limit-")
        manager.receiveConsoleText("v14 caps=trot5\r\n")
        RunLoop.main.run(until: Date().addingTimeInterval(0.05))
        XCTAssertEqual(manager.runtimeState.revision, "turn-limit-v14")
        XCTAssertEqual(sent, ["syncstate\n", "read 1\n"])
        XCTAssertNil(manager.lastError)
        manager.disconnect()
        RunLoop.main.run(until: Date().addingTimeInterval(0.05))
        XCTAssertEqual(sent.count, 2)
    }

    func testTrot5RequiresRobotCapabilityAndEncodesSelectedProfile() {
        var sent: [String] = []
        let manager = RobotBluetoothManager { sent.append(String(decoding: $0, as: UTF8.self)) }
        manager.send(.trot5(cycles: 3, periodMilliseconds: 844))
        XCTAssertTrue(sent.isEmpty)
        manager.receiveConsoleText("$SPOTSTATE pose=stand rev=cad-trot5-v13 caps=trot5\r\n")
        XCTAssertTrue(manager.runtimeState.supportsTrot5)
        sent.removeAll()
        manager.send(.trot5(cycles: 3, periodMilliseconds: 844))
        XCTAssertEqual(sent, ["trot5 3 844\n"])
        XCTAssertEqual(RobotCommand.trot5(cycles: 3, periodMilliseconds: 844).stateRefreshDelay!, 8.532, accuracy: 0.001)
        manager.disconnect()
        XCTAssertFalse(manager.runtimeState.supportsTrot5)
    }

    func testVoltageReadUsesSnapshotAndClearsOnDisconnect() {
        var commands: [String] = []; var now = 0.0
        let manager = RobotBluetoothManager(commandWriter: { commands.append(String(decoding: $0, as: UTF8.self)) })
        manager.telemetryClock = { now }
        manager.receiveConsoleText("$SPOTSTATE pose=custom rev=forward11-v12\r\n")
        XCTAssertEqual(commands, ["read 1\n"])
        manager.receiveConsoleText("ID 1 voltage=9800mV\r\n# ")
        XCTAssertNil(manager.supplyVoltageMillivolts)
        for time in [4.0, 6.0, 8.0] {
            now = time; manager.send(.raw("read 1"))
            manager.receiveConsoleText("ID 1 pos=1941 voltage=98")
            manager.receiveConsoleText("00mV temp=29C moving=0\r\n# ")
        }
        XCTAssertEqual(manager.supplyVoltageMillivolts, 9800)
        XCTAssertNotNil(manager.lastVoltageRead)
        manager.receiveConsoleText("ID 2 voltage=12000mV\n")
        XCTAssertEqual(manager.supplyVoltageMillivolts, 9800)
        manager.disconnect()
        XCTAssertNil(manager.supplyVoltageMillivolts)
        XCTAssertNil(manager.lastVoltageRead)
    }

    func testVoltageQueryDoesNotInterruptDrive() {
        var commands: [String] = []
        let manager = RobotBluetoothManager(commandWriter: { commands.append(String(decoding: $0, as: UTF8.self)) })
        manager.updateDrive(x: 0, y: 0.8)
        manager.receiveConsoleText("$SPOTSTATE pose=custom rev=forward11-v12\n")
        XCTAssertFalse(commands.contains("read 1\n"))
        manager.disconnect()
    }

    func testCreatingBluetoothModelDoesNotStartServices() {
        let manager = RobotBluetoothManager()
        XCTAssertFalse(manager.hasStarted)
        XCTAssertFalse(manager.state.isReady)
        manager.disconnect() // Safe even before the screen starts Bluetooth.
        XCTAssertFalse(manager.hasStarted)
    }

    @MainActor
    func testControlScreenStartsServicesOnlyAfterAppearance() {
        var commands: [Data] = []
        let manager = RobotBluetoothManager(commandWriter: { commands.append($0) })
        let screen = UIHostingController(rootView: ContentView().environmentObject(manager))
        screen.loadViewIfNeeded()
        screen.view.layoutIfNeeded()
        XCTAssertFalse(manager.hasStarted, "Building the layout must not start Bluetooth")

        guard let scene = UIApplication.shared.connectedScenes
            .compactMap({ $0 as? UIWindowScene }).first else {
            XCTFail("The app test host must provide a window scene")
            return
        }
        let previousKeyWindow = scene.windows.first(where: \.isKeyWindow)
        let window = UIWindow(windowScene: scene)
        window.rootViewController = screen
        window.makeKeyAndVisible()
        defer {
            window.isHidden = true
            previousKeyWindow?.makeKeyAndVisible()
        }
        let deadline = Date().addingTimeInterval(2)
        while !manager.hasStarted && Date() < deadline {
            RunLoop.main.run(until: Date().addingTimeInterval(0.01))
        }
        XCTAssertTrue(manager.hasStarted, "The visible screen must start services")
        manager.start()
        XCTAssertTrue(commands.isEmpty, "Startup must not issue a posture command")
    }

    func testBLEQueueWaitsForWriteAcknowledgementAndKeepsLineIntact() {
        var queue = RobotBLEWriteQueue()
        XCTAssertTrue(queue.enqueue(Data("drive 500 0 1\n".utf8), kind: .command))
        XCTAssertEqual(queue.nextChunk(maximumLength: 6, acknowledged: true), Data("drive ".utf8))
        XCTAssertNil(queue.nextChunk(maximumLength: 6, acknowledged: true))
        XCTAssertTrue(queue.enqueue(Data("@S 2\n".utf8), kind: .stop))
        queue.acknowledge()
        XCTAssertEqual(queue.nextChunk(maximumLength: 100, acknowledged: true), Data("500 0 1\n".utf8))
        XCTAssertNil(queue.nextChunk(maximumLength: 100, acknowledged: true))
        queue.acknowledge()
        XCTAssertEqual(queue.nextChunk(maximumLength: 100, acknowledged: true), Data("@S 2\n".utf8))
    }

    func testBLEQueueCoalescesOldJoystickTargetsAndStopDiscardsBacklog() {
        var queue = RobotBLEWriteQueue()
        for sequence in 1...200 {
            XCTAssertTrue(queue.enqueue(Data("@D \(sequence) 500 0\n".utf8), kind: .update))
        }
        XCTAssertEqual(queue.nextChunk(maximumLength: 100, acknowledged: true), Data("@D 200 500 0\n".utf8))
        XCTAssertTrue(queue.enqueue(Data("@D 201 900 0\n".utf8), kind: .update))
        XCTAssertTrue(queue.enqueue(Data("targets\n".utf8), kind: .command))
        XCTAssertTrue(queue.enqueue(Data("@S 202\n".utf8), kind: .stop))
        queue.acknowledge()
        XCTAssertEqual(queue.nextChunk(maximumLength: 100, acknowledged: true), Data("@S 202\n".utf8))
        queue.acknowledge()
        XCTAssertNil(queue.nextChunk(maximumLength: 100, acknowledged: true))
    }

    func testBLEQueueInterruptAndResetDoNotReplayQueuedMotion() {
        var queue = RobotBLEWriteQueue()
        XCTAssertTrue(queue.enqueue(Data("drive 500 0 1\n".utf8), kind: .command))
        XCTAssertTrue(queue.enqueue(Data([3]), kind: .interrupt))
        XCTAssertEqual(queue.nextChunk(maximumLength: 20, acknowledged: false), Data([3]))
        XCTAssertNil(queue.nextChunk(maximumLength: 20, acknowledged: false))
        XCTAssertTrue(queue.enqueue(Data("drive 500 0 2\n".utf8), kind: .command))
        queue = RobotBLEWriteQueue()
        XCTAssertNil(queue.nextChunk(maximumLength: 20, acknowledged: true))
    }

    func testBLEQueueBoundsOrdinaryCommandBacklog() {
        var queue = RobotBLEWriteQueue()
        for _ in 0..<16 { XCTAssertTrue(queue.enqueue(Data("targets\n".utf8), kind: .command)) }
        XCTAssertFalse(queue.enqueue(Data("targets\n".utf8), kind: .command))
        XCTAssertTrue(queue.enqueue(Data([3]), kind: .interrupt))
        XCTAssertEqual(queue.nextChunk(maximumLength: 20, acknowledged: false), Data([3]))
    }

    func testRejectedDriveAndReconnectDoNotReplayOldInput() {
        var sent: [String] = []
        let manager = RobotBluetoothManager { sent.append(String(decoding: $0, as: UTF8.self)) }
        manager.updateDrive(x: 0, y: 1)
        manager.receiveConsoleText("# ERROR: drive requires profile 3400 254\r\n# ")
        XCTAssertEqual(manager.driveStatus, "보행 명령 거부")
        let count = sent.count
        RunLoop.main.run(until: Date().addingTimeInterval(0.3))
        XCTAssertEqual(sent.count, count)
        manager.disconnect()
        manager.updateDrive(x: 0, y: 1)
        XCTAssertEqual(sent.count, count)
    }

    func testStopButtonWaitsForWalkingReturnWithoutRepeatedInterrupt() {
        var sent: [String] = []
        let manager = RobotBluetoothManager { sent.append(String(decoding: $0, as: UTF8.self)) }
        manager.updateDrive(x: 0, y: 1)
        manager.stopWalkingOrHold()
        XCTAssertTrue(sent.last?.hasPrefix("@S ") == true)
        let count = sent.count
        manager.stopWalkingOrHold()
        XCTAssertEqual(sent.count, count)
        XCTAssertFalse(sent.contains("\u{03}"))
        manager.disconnect()
    }

    func testLostStartedNotificationDoesNotDisconnectHeldJoystick() {
        var sent: [String] = []
        let manager = RobotBluetoothManager { sent.append(String(decoding: $0, as: UTF8.self)) }
        manager.updateDrive(x: 0, y: 1)
        // Console notifications are not acknowledged. A lost banner must not
        // turn into a local disconnect while the control lane is still live.
        RunLoop.main.run(until: Date().addingTimeInterval(5.2))
        XCTAssertEqual(manager.state, .ready)
        XCTAssertTrue(sent.last?.hasPrefix("@D ") == true)
        XCTAssertFalse(sent.contains("\u{03}"))
        manager.stopDrive()
        XCTAssertTrue(sent.last?.hasPrefix("@S ") == true)
        manager.disconnect()
    }

    func testConsolePromptsAndResponsesAcrossEveryPacketBoundary() {
        let transcript = "# $SPOTDRIVE started seq=1\r\n$SPOTDRIVE stopped reason=ok\r\nOK\r\n# "
        let expected: [RobotConsoleStream.Event] = [
            .prompt, .line("$SPOTDRIVE started seq=1"),
            .line("$SPOTDRIVE stopped reason=ok"), .line("OK"), .prompt
        ]
        let bytes = Array(transcript.utf8)
        for boundary in 0...bytes.count {
            var stream = RobotConsoleStream()
            let first = stream.append(String(decoding: bytes[..<boundary], as: UTF8.self))
            let second = stream.append(String(decoding: bytes[boundary...], as: UTF8.self))
            XCTAssertEqual(first + second, expected, "packet boundary \(boundary)")
        }
        var stream = RobotConsoleStream()
        XCTAssertEqual(bytes.flatMap { stream.append(String(decoding: [$0], as: UTF8.self)) }, expected)
    }

    func testSnapshotFollowingPromptIsRecognized() {
        let manager = RobotBluetoothManager(commandWriter: { _ in })
        manager.receiveConsoleText("#")
        manager.receiveConsoleText(" $SPOTSTATE pose=stand torque=on safety=ok balance=full rev=continuous-drive-v10\r\n# ")
        XCTAssertEqual(manager.runtimeState.pose, "stand")
        XCTAssertNotNil(manager.lastStateSync)
    }

    func testStateRefreshDuringDriveDoesNotStopHeartbeat() {
        var sent: [String] = []
        let manager = RobotBluetoothManager { sent.append(String(decoding: $0, as: UTF8.self)) }
        manager.updateDrive(x: 0, y: 1)
        manager.receiveConsoleText("# $SPOTDRIVE started seq=1\r\n")
        manager.synchronizeState()
        // A held joystick must stay connected beyond five seconds.
        RunLoop.main.run(until: Date().addingTimeInterval(5.2))
        XCTAssertEqual(manager.state, .ready)
        XCTAssertTrue(sent.contains { $0.hasPrefix("@D ") })
        XCTAssertFalse(sent.contains { $0.hasPrefix("@S ") || $0 == "syncstate\n" })
        manager.disconnect()
    }

    func testScheduledPostureRefreshCannotStopNewDrive() {
        var sent: [String] = []
        let manager = RobotBluetoothManager { sent.append(String(decoding: $0, as: UTF8.self)) }
        manager.send(.hold) // schedules an idle snapshot after four seconds
        manager.receiveConsoleText("OK\n# ")
        manager.updateDrive(x: 0, y: 1)
        manager.receiveConsoleText("# $SPOTDRIVE started seq=1\r\n")
        RunLoop.main.run(until: Date().addingTimeInterval(4.2))
        XCTAssertTrue(sent.contains { $0.hasPrefix("@D ") })
        XCTAssertFalse(sent.contains { $0.hasPrefix("@S ") || $0 == "syncstate\n" })
        manager.disconnect()
    }

    func testReleaseWaitsForCompletionAndNextTouchStartsNewDrive() {
        var sent: [String] = []
        let manager = RobotBluetoothManager { sent.append(String(decoding: $0, as: UTF8.self)) }
        manager.updateDrive(x: 0, y: 1)
        manager.receiveConsoleText("# $SPOTDRIVE started seq=1\r\n")
        manager.stopDrive()
        let countAfterStop = sent.count
        manager.updateDrive(x: 0, y: -1)
        XCTAssertEqual(sent.count, countAfterStop)
        manager.receiveConsoleText("$SPOTDRIVE stopped reason=ok\r\nOK\r\n")
        manager.updateDrive(x: 0, y: -1)
        XCTAssertEqual(sent.count, countAfterStop)
        manager.receiveConsoleText("#")
        manager.receiveConsoleText(" ")
        manager.updateDrive(x: 0, y: -1)
        XCTAssertEqual(sent.filter { $0.hasPrefix("drive ") }.count, 2)
        manager.disconnect()
    }

    func testSafetyCommandStillStopsDriveBeforeSendingPosture() {
        var sent: [String] = []
        let manager = RobotBluetoothManager { sent.append(String(decoding: $0, as: UTF8.self)) }
        manager.updateDrive(x: 0, y: 1)
        manager.receiveConsoleText("# $SPOTDRIVE started seq=1\r\n")
        manager.send(.stand)
        manager.synchronizeState() // must not overwrite pending Stand
        XCTAssertTrue(sent.last?.hasPrefix("@S ") == true)
        manager.receiveConsoleText("$SPOTDRIVE stopped reason=ok\r\nOK\r\n# ")
        XCTAssertEqual(sent.last, "stand\n")
        manager.disconnect()
    }

    func testMissingCompletionDisconnectsInsteadOfLeavingSessionStuck() {
        var sent: [String] = []
        let manager = RobotBluetoothManager { sent.append(String(decoding: $0, as: UTF8.self)) }
        manager.updateDrive(x: 0, y: 1)
        manager.receiveConsoleText("# $SPOTDRIVE started seq=1\r\n")
        manager.stopDrive()
        RunLoop.main.run(until: Date().addingTimeInterval(5.2))
        XCTAssertEqual(manager.state, .disconnected)
        XCTAssertNotNil(manager.lastError)
        XCTAssertEqual(sent.last, "\u{03}")
    }

    func testSafeCrabCommands() {
        XCTAssertEqual(RobotCommand.crabLeft(cycles: 1, periodMilliseconds: 5000).consoleLine,
                       "crab left 1 5000")
        XCTAssertEqual(RobotCommand.crabRight(cycles: 1, periodMilliseconds: 5000).consoleLine,
                       "crab right 1 5000")
    }

    func testCommandHasNewlineTerminator() {
        XCTAssertEqual(RobotCommand.stand.encoded, Data("stand\n".utf8))
    }

    func testPostureCommands() {
        XCTAssertEqual(RobotCommand.landing.consoleLine, "landing")
        XCTAssertEqual(RobotCommand.stand.consoleLine, "stand")
        XCTAssertEqual(RobotCommand.stand11.consoleLine, "stand11")
    }

    func testPersistentLogAndClockCommands() {
        XCTAssertEqual(
            RobotCommand.synchronizeTime(epochMilliseconds: 1_800_000_000_123).consoleLine,
            "log time 1800000000123")
        XCTAssertEqual(RobotCommand.storedLogs(count: 64).consoleLine, "log show 64")
        XCTAssertEqual(RobotCommand.clearStoredLogs.consoleLine, "log clear")
    }

    func testStraightCorridorRejectsRecordedDriftAndPreservesTurning() {
        for y in [0.3, 0.6, 1.0, -0.3, -1.0] {
            for fraction in [-0.10, -0.03, 0, 0.06, 0.10] {
                let v = RobotDriveVector.make(x: abs(y)*fraction, y: y)!
                XCTAssertEqual(v.yawPerMille, 0)
                XCTAssertEqual(v.linearPerMille > 0, y > 0)
            }
        }
        XCTAssertEqual(RobotDriveVector.make(x: 1, y: 0)?.yawPerMille, 1000)
        XCTAssertEqual(RobotDriveVector.make(x: -1, y: 0)?.yawPerMille, -1000)
        let inside = RobotDriveVector.make(x: 0.0999, y: 1)!
        let outside = RobotDriveVector.make(x: 0.1001, y: 1)!
        XCTAssertLessThanOrEqual(abs(outside.yawPerMille-inside.yawPerMille), 1)
        XCTAssertGreaterThan(RobotDriveVector.make(x: 0.4, y: 1)!.yawPerMille, 100)
        XCTAssertNil(RobotDriveVector.make(x: .nan, y: 1))
    }

    func testJoystickForwardSpeedMapping() {
        let slow = RobotDriveVector.make(x: 0, y: 0.15)
        let fast = RobotDriveVector.make(x: 0, y: 1.0)

        XCTAssertEqual(slow?.linearPerMille, 300)
        XCTAssertEqual(slow?.yawPerMille, 0)
        XCTAssertEqual(slow?.speedFraction, 0)
        XCTAssertEqual(fast?.linearPerMille, 1000)
        XCTAssertEqual(fast?.speedFraction, 1)
    }

    func testJoystickSupportsCombinedForwardAndTurn() {
        let diagonal = RobotDriveVector.make(x: -0.5, y: 0.5)
        XCTAssertNotNil(diagonal)
        XCTAssertLessThan(diagonal?.yawPerMille ?? 0, 0)
        XCTAssertGreaterThan(diagonal?.linearPerMille ?? 0, 0)
        XCTAssertLessThanOrEqual(
            abs(diagonal?.yawPerMille ?? 0) + abs(diagonal?.linearPerMille ?? 0),
            1415)
    }

    func testJoystickBackwardSpeedMapping() {
        XCTAssertEqual(RobotDriveVector.make(x: 0, y: -1)?.linearPerMille, -1000)
        XCTAssertNil(RobotDriveVector.make(x: 0.05, y: 0.05))
    }

    func testContinuousDriveProtocol() {
        XCTAssertEqual(
            RobotCommand.drive(linearPerMille: 700, yawPerMille: -200,
                               sequence: 42).consoleLine,
            "drive 700 -200 42")
        XCTAssertEqual(
            RobotDriveRealtimePacket.update(sequence: 43,
                                            linearPerMille: 800,
                                            yawPerMille: 100).encoded,
            Data("@D 43 800 100\n".utf8))
        XCTAssertEqual(RobotDriveRealtimePacket.stop(sequence: 44).encoded,
                       Data("@S 44\n".utf8))
    }
}


extension RobotCommandTests {
    func testSimulatorIdentityRejectsHardwareAndWrongProtocol() {
        XCTAssertTrue(RobotConnectionTarget.isSimulatorIdentity("$SPOTBACKEND backend=sim protocol=1 physics=estimated"))
        XCTAssertFalse(RobotConnectionTarget.isSimulatorIdentity("$SPOTBACKEND backend=robot protocol=1"))
        XCTAssertFalse(RobotConnectionTarget.isSimulatorIdentity("$SPOTBACKEND backend=sim protocol=2"))
        XCTAssertFalse(RobotConnectionTarget.isSimulatorIdentity("$SPOTSTATE backend=sim protocol=1"))
    }

    func testSwitchingTargetRequiresNewConnectionAndDropsDriveState() {
        let previous = UserDefaults.standard.string(forKey: "robotTarget")
        defer { UserDefaults.standard.set(previous, forKey: "robotTarget") }
        let manager = RobotBluetoothManager { _ in }
        manager.updateDrive(x: 0, y: 0.6)
        manager.selectTarget(manager.target == .robot ? .simulator : .robot)
        XCTAssertEqual(manager.state, .disconnected)
        XCTAssertNil(manager.lastStateSync)
        XCTAssertEqual(manager.driveStatus, "중립")
    }
}


extension RobotCommandTests {
    func testSimulatorTCPHandshakeAndStateRoundTrip() throws {
        let previous = UserDefaults.standard.string(forKey: "robotTarget")
        defer { UserDefaults.standard.set(previous, forKey: "robotTarget") }
        let listener = try NWListener(using: .tcp, on: .any)
        var peer: NWConnection?
        var lines: [String] = []
        var parser = RobotConsoleStream()
        func receive(_ connection: NWConnection) {
            connection.receive(minimumIncompleteLength: 1, maximumLength: 8192) { data, _, done, error in
                if let data {
                    for event in parser.append(String(decoding: data, as: UTF8.self)) {
                        guard case .line(let line) = event else { continue }
                        lines.append(line)
                        let response: String
                        switch line {
                        case "identity": response = "$SPOTBACKEND backend=sim protocol=1\r\n# "
                        case "syncstate": response = "$SPOTSTATE pose=stand torque=on safety=ok balance=monitor rev=test-sim caps=trot5 backend=sim\r\n# "
                        case "read 1": response = "ID 1 voltage=11100mV source=simulated\r\n# "
                        default: response = "OK\r\n# "
                        }
                        connection.send(content: Data(response.utf8), completion: .contentProcessed { _ in })
                    }
                }
                if !done && error == nil { receive(connection) }
            }
        }
        listener.newConnectionHandler = { connection in
            peer = connection
            connection.start(queue: .main)
            receive(connection)
        }
        listener.start(queue: .main)
        let deadline = Date().addingTimeInterval(5)
        while (listener.port?.rawValue ?? 0) == 0 && Date() < deadline { RunLoop.main.run(until: Date().addingTimeInterval(0.01)) }
        let port = try XCTUnwrap(listener.port)
        let manager = RobotBluetoothManager()
        manager.selectTarget(.simulator)
        manager.simulatorHost = "127.0.0.1"
        manager.simulatorPort = String(port.rawValue)
        defer { manager.disconnect(); peer?.cancel(); listener.cancel() }
        manager.connect()
        while manager.lastStateSync == nil && Date() < deadline { RunLoop.main.run(until: Date().addingTimeInterval(0.01)) }
        XCTAssertEqual(manager.state, .ready, manager.lastError ?? manager.consoleText)
        XCTAssertEqual(manager.runtimeState.revision, "test-sim")
        XCTAssertNil(manager.supplyVoltageMillivolts, "Simulator telemetry must not become a real battery estimate")
        XCTAssertEqual(Array(lines.prefix(2)), ["identity", "syncstate"])
    }
}

extension RobotCommandTests {
    func testSimulatorBLEUsesIsolatedServiceAndKeepsTCPAvailable() {
        XCTAssertTrue(RobotConnectionTarget.simulatorBluetooth.usesBluetooth)
        XCTAssertTrue(RobotConnectionTarget.simulatorBluetooth.isSimulator)
        XCTAssertNotEqual(RobotConnectionTarget.robot.serviceID, RobotConnectionTarget.simulatorBluetooth.serviceID)
        XCTAssertNotEqual(RobotConnectionTarget.robot.receiveID, RobotConnectionTarget.simulatorBluetooth.receiveID)
        XCTAssertNotEqual(RobotConnectionTarget.robot.transmitID, RobotConnectionTarget.simulatorBluetooth.transmitID)
        XCTAssertFalse(RobotConnectionTarget.simulator.usesBluetooth)
        XCTAssertEqual(RobotConnectionTarget.simulatorBluetooth.deviceName, "SpotOMG-Sim")
    }

    func testFragmentedSimulatorLinkLossDisconnectsAndClearsDrive() {
        let previous = UserDefaults.standard.string(forKey: "robotTarget")
        defer { UserDefaults.standard.set(previous, forKey: "robotTarget") }
        UserDefaults.standard.set(RobotConnectionTarget.simulatorBluetooth.rawValue, forKey: "robotTarget")
        let manager = RobotBluetoothManager { _ in }
        manager.updateDrive(x: 0, y: 0.5)
        manager.receiveConsoleText("$SIMLI")
        XCTAssertTrue(manager.state.isReady)
        manager.receiveConsoleText("NK disconnected tcp-closed\r\n")
        XCTAssertEqual(manager.state, .disconnected)
        XCTAssertEqual(manager.driveStatus, "중립")
        XCTAssertTrue(manager.lastError?.contains("MuJoCo") == true)
    }
}

extension RobotCommandTests {
    func testSimulatorProfileCommandsAndSnapshot() {
        XCTAssertEqual(RobotCommand.simulatorProfile(.cruise).consoleLine, "simprofile cruise")
        let manager = RobotBluetoothManager { _ in }
        manager.receiveConsoleText("$SPOTSTATE pose=stand caps=trot5,simprofiles backend=sim profile=highstep\n")
        XCTAssertEqual(manager.runtimeState.simulationProfile, "highstep")
        XCTAssertTrue(manager.runtimeState.capabilities.contains("simprofiles"))
        manager.disconnect()
    }
    func testHardwareRejectsSimulatorProfileCommand() {
        let previous = UserDefaults.standard.string(forKey: "robotTarget")
        defer { UserDefaults.standard.set(previous, forKey: "robotTarget") }
        UserDefaults.standard.set("robot", forKey: "robotTarget")
        var commands = [String]()
        let manager = RobotBluetoothManager { commands.append(String(decoding: $0, as: UTF8.self)) }
        manager.send(.simulatorProfile(.trot))
        manager.send(.simulatorBalance(true))
        XCTAssertTrue(commands.isEmpty)
        XCTAssertNotNil(manager.lastError)
        manager.disconnect()
    }
    func testSimulatorBalanceCommandsAndState() {
        XCTAssertEqual(RobotCommand.simulatorBalance(true).consoleLine, "simbalance on")
        XCTAssertEqual(RobotCommand.simulatorBalance(false).consoleLine, "simbalance off")
        let manager = RobotBluetoothManager { _ in }
        manager.receiveConsoleText("$SPOTSTATE pose=stand balance=active caps=simbalance,bno055emu backend=sim\n")
        XCTAssertEqual(manager.runtimeState.balanceTitle, "수평 보정 작동 중")
        manager.disconnect()
    }

    func testTiltStopCannotRestartHeldJoystickUntilReleaseAndRecovery() {
        var sent = [String]()
        let manager = RobotBluetoothManager { sent.append(String(decoding: $0, as: UTF8.self)) }
        manager.updateDrive(x: 0, y: 1)
        manager.receiveConsoleText("$SPOTDRIVE stopped reason=tilt\r\nOK\r\n# ")
        let count = sent.filter { $0.hasPrefix("drive ") }.count
        for _ in 0..<10 { manager.updateDrive(x: 0, y: 1) }
        XCTAssertEqual(sent.filter { $0.hasPrefix("drive ") }.count, count)
        XCTAssertEqual(manager.runtimeState.safety, "tilt")
        manager.receiveConsoleText("$SPOTSTATE safety=ok\n# ") // stale snapshot cannot clear latch
        manager.updateDrive(x: 0, y: 1)
        XCTAssertEqual(sent.filter { $0.hasPrefix("drive ") }.count, count)
        manager.stopDrive(reason: "gesture-ended")
        manager.updateDrive(x: 0, y: 1)
        XCTAssertEqual(sent.filter { $0.hasPrefix("drive ") }.count, count)
        manager.send(.recover)
        manager.receiveConsoleText("OK\n# $SPOTSTATE safety=ok\n# ")
        manager.updateDrive(x: 0, y: 1)
        XCTAssertEqual(sent.filter { $0.hasPrefix("drive ") }.count, count + 1)
        manager.disconnect()
    }

    func testRejectedDriveDuringStopCompletesWithoutDisconnectTimeout() {
        var sent = [String]()
        let manager = RobotBluetoothManager { sent.append(String(decoding: $0, as: UTF8.self)) }
        manager.updateDrive(x: 0, y: 1)
        manager.stopDrive(reason: "gesture-ended")
        manager.receiveConsoleText("ERROR: stand/recover required before gait\r\n# ")
        RunLoop.main.run(until: Date().addingTimeInterval(5.2))
        XCTAssertEqual(manager.state, .ready)
        XCTAssertFalse(sent.contains("\u{03}"))
        let count = sent.filter { $0.hasPrefix("drive ") }.count
        manager.updateDrive(x: 0, y: 1)
        XCTAssertEqual(sent.filter { $0.hasPrefix("drive ") }.count, count)
        manager.disconnect()
    }

    func testWatchdogStopRequiresJoystickRelease() {
        var sent = [String]()
        let manager = RobotBluetoothManager { sent.append(String(decoding: $0, as: UTF8.self)) }
        manager.updateDrive(x: 0, y: 1)
        manager.receiveConsoleText("$SPOTDRIVE stopped reason=watchdog\nOK\n# ")
        let count = sent.filter { $0.hasPrefix("drive ") }.count
        manager.updateDrive(x: 0, y: 1)
        XCTAssertEqual(sent.filter { $0.hasPrefix("drive ") }.count, count)
        manager.stopDrive(reason: "gesture-ended")
        manager.updateDrive(x: 0, y: 1)
        XCTAssertEqual(sent.filter { $0.hasPrefix("drive ") }.count, count + 1)
        manager.disconnect()
    }

    func testHorizontalCorridorSelectsInPlaceTurnsAndIsContinuous() {
        for x in [-1.0, -0.6, -0.3, 0.3, 0.6, 1.0] {
            for fraction in [-0.10, -0.04, 0, 0.05, 0.10] {
                let v = RobotDriveVector.make(x: x, y: abs(x)*fraction)!
                XCTAssertEqual(v.linearPerMille, 0)
                XCTAssertEqual(v.yawPerMille > 0, x > 0)
                XCTAssertTrue(v.statusTitle.hasPrefix("제자리 "))
            }
        }
        let inside = RobotDriveVector.make(x: 1, y: 0.0999)!
        let outside = RobotDriveVector.make(x: 1, y: 0.1001)!
        XCTAssertLessThanOrEqual(abs(outside.linearPerMille-inside.linearPerMille), 1)
    }

    func testAllDiagonalsKeepTravelAndTurnSigns() {
        for x in [-0.7, 0.7] {
            for y in [-0.7, 0.7] {
                let v = RobotDriveVector.make(x: x, y: y)!
                XCTAssertEqual(v.linearPerMille > 0, y > 0)
                XCTAssertEqual(v.yawPerMille > 0, x > 0)
                XCTAssertFalse(v.statusTitle.contains("제자리"))
            }
        }
    }

}

extension RobotCommandTests {
    func testHeadingHoldCapabilitySnapshotAndCommands() {
        var sent = [String]()
        let manager = RobotBluetoothManager { sent.append(String(decoding: $0, as: UTF8.self)) }
        manager.send(.headingHold(true))
        XCTAssertTrue(sent.isEmpty)
        manager.receiveConsoleText("$SPOTSTATE pose=stand heading=on caps=headinghold\n")
        manager.receiveConsoleText("ID 1 voltage=12000mV\n# ")
        XCTAssertEqual(manager.runtimeState.heading, "on")
        manager.send(.headingHold(false))
        XCTAssertTrue(sent.contains("heading off\n"))
        manager.receiveConsoleText("$SPOTSTATE pose=stand heading=off caps=headinghold\n")
        XCTAssertEqual(manager.runtimeState.heading, "off")
        XCTAssertEqual(RobotCommand.headingHold(true).consoleLine, "heading on")
        manager.disconnect()
    }

    func testHeadingToggleWaitsForDriveStop() {
        var sent = [String]()
        let manager = RobotBluetoothManager { sent.append(String(decoding: $0, as: UTF8.self)) }
        manager.receiveConsoleText("$SPOTSTATE pose=stand heading=on caps=headinghold\n")
        manager.receiveConsoleText("ID 1 voltage=12000mV\n# ")
        manager.updateDrive(x: 0, y: 1)
        manager.receiveConsoleText("$SPOTDRIVE started seq=1 watchdog=800ms\n")
        manager.send(.headingHold(false))
        XCTAssertFalse(sent.contains("heading off\n"))
        XCTAssertTrue(sent.contains { $0.hasPrefix("@S ") })
        manager.receiveConsoleText("$SPOTDRIVE stopped reason=requested\nOK\n# ")
        XCTAssertTrue(sent.contains("heading off\n"))
        manager.disconnect()
    }
}

extension RobotCommandTests {
    func testSharedHardwareProfilesBalanceAndHeadingCommands() {
        let previous = UserDefaults.standard.string(forKey: "robotTarget")
        defer { UserDefaults.standard.set(previous, forKey: "robotTarget") }
        UserDefaults.standard.set("robot", forKey: "robotTarget")
        var sent = [String]()
        let manager = RobotBluetoothManager { sent.append(String(decoding: $0, as: UTF8.self)) }
        manager.receiveConsoleText("$SPOTSTATE pose=stand balance=full heading=on rev=shared-locomotion-v17 caps=gaitprofiles,balancecontrol,headinghold profile=cruise\n")
        manager.send(.simulatorProfile(.trot))
        manager.send(.simulatorBalance(false))
        manager.send(.headingHold(false))
        XCTAssertTrue(sent.contains("gaitprofile trot\n"))
        XCTAssertTrue(sent.contains("balance off\n"))
        XCTAssertTrue(sent.contains("heading off\n"))
        XCTAssertFalse(sent.contains { $0.hasPrefix("simprofile") || $0.hasPrefix("simbalance") })
        XCTAssertEqual(manager.runtimeState.balanceTitle, "수평 보정 켜짐")
        manager.disconnect()
    }
}


extension RobotCommandTests {
    func testJointProfileSelectionUsesSharedFirmwareCommand() {
        let previous = UserDefaults.standard.string(forKey: "robotTarget")
        defer { UserDefaults.standard.set(previous, forKey: "robotTarget") }
        UserDefaults.standard.set("robot", forKey: "robotTarget")
        var sent = [String]()
        let manager = RobotBluetoothManager { sent.append(String(decoding: $0, as: UTF8.self)) }
        manager.receiveConsoleText("$SPOTSTATE pose=stand caps=gaitprofiles profile=joint rev=shared-locomotion-v21\n")
        manager.send(.simulatorProfile(.joint))
        XCTAssertTrue(sent.contains("gaitprofile joint\n"))
        XCTAssertEqual(manager.runtimeState.simulationProfile, "joint")
        manager.disconnect()
    }
}


extension RobotCommandTests {
    private func paritySettings() -> UserDefaults {
        UserDefaults(suiteName: "SpotOMG-parity-" + UUID().uuidString)!
    }
    private var liftSnapshot: String {
        "$SPOTSTATE pose=custom torque=off safety=ok rev=attitudepd-v4-v90-r1 caps=footlift,footliftpersist,commandretry lift_fl=0 lift_fr=0 lift_rl=0 lift_rr=0\n"
    }
    func testRobotOwnsFootLiftAndRequiresFreshReadback() {
        let defaults = paritySettings(); defaults.set([30,30,10,10], forKey: "footLiftMm")
        var sent: [String] = []
        let m = RobotBluetoothManager(commandWriter: { sent.append(String(decoding:$0,as:UTF8.self)) }, settings: defaults)
        defer { m.disconnect() }
        m.receiveConsoleText(liftSnapshot)
        XCTAssertFalse(m.footLiftPending) // idle battery response still outstanding
        m.receiveConsoleText("ID 1 voltage=12000mV\n# ")
        XCTAssertFalse(sent.contains { $0.hasPrefix("footlift ") })
        XCTAssertEqual(m.savedFootLift,[0,0,0,0])
        m.configureFootLift([30,30,10,10])
        XCTAssertEqual(sent.last, "footlift save 30 30 10 10\n")
        XCTAssertTrue(m.footLiftPending)
        // Matching-looking snapshots before the SET completion cannot save/finish.
        let expected = liftSnapshot.replacingOccurrences(of:"lift_fl=0 lift_fr=0 lift_rl=0 lift_rr=0",with:"lift_fl=30 lift_fr=30 lift_rl=10 lift_rr=10")
        m.receiveConsoleText(expected)
        XCTAssertTrue(m.footLiftPending)
        m.receiveConsoleText("OK footlift saved\n# ")
        XCTAssertEqual(sent.last,"syncstate\n")
        m.receiveConsoleText(expected + "# ")
        XCTAssertFalse(m.footLiftPending)
        XCTAssertEqual(m.footLiftMessage,"로봇 저장·반영 완료")
        XCTAssertEqual(m.footLiftApplied,[30,30,10,10])
        XCTAssertEqual(defaults.array(forKey:"footLiftMm") as? [Int],[30,30,10,10])
        m.receiveConsoleText(liftSnapshot)
        m.receiveConsoleText("ID 1 voltage=12000mV\n# ")
        XCTAssertEqual(sent.filter { $0.hasPrefix("footlift save") }.count,1)
    }
    func testFootWidthRequiresFreshCombinedReadback() {
        var sent: [String] = []
        let m = RobotBluetoothManager(commandWriter: { sent.append(String(decoding:$0,as:UTF8.self)) })
        defer { m.disconnect() }
        let initial = liftSnapshot.replacingOccurrences(of:"caps=footlift,",with:"caps=footwidth,footlift,")
            .replacingOccurrences(of:"lift_rr=0",with:"lift_rr=0 width_fl=0 width_fr=0 width_rl=0 width_rr=0")
        m.receiveConsoleText(initial);m.receiveConsoleText("ID 1 voltage=12000mV\n# ")
        XCTAssertTrue(m.supportsFootWidth)
        m.configureFootLift([1,2,3,4],widths:[-5,6,-7,8])
        XCTAssertEqual(sent.last,"footlift save 1 2 3 4 -5 6 -7 8\n")
        m.receiveConsoleText("OK footlift saved\n# ")
        let expected=initial.replacingOccurrences(of:"lift_fl=0 lift_fr=0 lift_rl=0 lift_rr=0 width_fl=0 width_fr=0 width_rl=0 width_rr=0",
            with:"lift_fl=1 lift_fr=2 lift_rl=3 lift_rr=4 width_fl=-5 width_fr=6 width_rl=-7 width_rr=8")
        m.receiveConsoleText(expected+"# ")
        XCTAssertFalse(m.footLiftPending)
        XCTAssertEqual(m.footLiftMessage,"로봇 저장·반영 완료")
        XCTAssertEqual(m.appliedFootSettings,[1,2,3,4,-5,6,-7,8])
    }

    func testLegacyFirmwareFootLiftCannotBeSaved() {
        var sent: [String] = []
        let m = RobotBluetoothManager(commandWriter: { sent.append(String(decoding:$0,as:UTF8.self)) })
        defer { m.disconnect() }
        m.receiveConsoleText(liftSnapshot.replacingOccurrences(of:"footliftpersist,",with:""));m.receiveConsoleText("# ")
        m.configureFootLift([1,2,3,4])
        XCTAssertFalse(sent.contains { $0.hasPrefix("footlift save") })
        XCTAssertFalse(m.canConfigureFootLift)
    }
    func testFootLiftMismatchAndDisconnectNeverSaveDraft() {
        let defaults = paritySettings(); var sent: [String] = []
        let m = RobotBluetoothManager(commandWriter: { sent.append(String(decoding:$0,as:UTF8.self)) }, settings: defaults)
        m.receiveConsoleText(liftSnapshot);m.receiveConsoleText("ID 1 voltage=12000mV\n# ")
        m.configureFootLift([300,20,0,0]);XCTAssertEqual(sent.last,"footlift save 300 20 0 0\n")
        m.receiveConsoleText("OK footlift saved\n# ");m.receiveConsoleText(liftSnapshot + "# ")
        XCTAssertNil(defaults.array(forKey:"footLiftMm"));XCTAssertFalse(m.footLiftPending)
        XCTAssertTrue(m.footLiftMessage.contains("불일치"))
        m.configureFootLift([1,2,3,4]);m.disconnect()
        XCTAssertFalse(m.footLiftPending);XCTAssertNil(m.footLiftApplied)
        XCTAssertNil(defaults.array(forKey:"footLiftMm"))
    }
    func testFootLiftRejectionCannotBeSavedByLaterMatchingTelemetry() {
        let defaults = paritySettings()
        let m = RobotBluetoothManager(commandWriter: { _ in }, settings: defaults)
        defer { m.disconnect() }
        m.receiveConsoleText(liftSnapshot);m.receiveConsoleText("# ")
        m.configureFootLift([1,2,3,4]);m.receiveConsoleText("ERROR: rejected\n# ")
        m.receiveConsoleText(liftSnapshot.replacingOccurrences(of:"lift_fl=0 lift_fr=0 lift_rl=0 lift_rr=0",with:"lift_fl=1 lift_fr=2 lift_rl=3 lift_rr=4"))
        XCTAssertNil(defaults.array(forKey:"footLiftMm"));XCTAssertFalse(m.footLiftPending)
    }
    func testProtectivePausePersistsAcrossHealthyTelemetry() {
        let m = RobotBluetoothManager(commandWriter: { _ in }, settings: paritySettings())
        defer { m.disconnect() }
        m.updateDrive(x:0,y:1)
        m.receiveConsoleText("$SPOTDRIVE stopped reason=tilt\nOK\n# ")
        XCTAssertTrue(m.motionWarning?.title.contains("기울기") == true)
        m.receiveConsoleText("$SPOTSTATE safety=ok caps=commandretry\n")
        XCTAssertTrue(m.motionWarning?.title.contains("기울기") == true)
        m.receiveConsoleText("$SPOTDRIVE started seq=2\n")
        XCTAssertNil(m.motionWarning)
        XCTAssertTrue(RobotMotionWarning(reason:"ERROR: imu unavailable").title.contains("IMU"))
    }
    func testFreshPressDuringStopWaitResumesOnlyAfterPrompt() {
        var sent: [String] = []
        let m = RobotBluetoothManager(commandWriter: { sent.append(String(decoding:$0,as:UTF8.self)) }, settings: paritySettings())
        defer { m.disconnect() }
        m.updateDrive(x:0,y:1);m.stopDrive(reason:"gesture-ended")
        m.updateDrive(x:0,y:-1)
        XCTAssertEqual(sent.filter { $0.hasPrefix("drive ") }.count,1)
        m.receiveConsoleText("$SPOTDRIVE stopped reason=requested\nOK\n")
        XCTAssertEqual(sent.filter { $0.hasPrefix("drive ") }.count,1)
        m.receiveConsoleText("# ")
        XCTAssertEqual(sent.filter { $0.hasPrefix("drive ") }.count,2)
        XCTAssertTrue(sent.last?.hasPrefix("drive -1000") == true)
    }
    func testReleasedPendingInputAndSafetyStoppedInputAreNotReplayed() {
        for safety in [false,true] {
            var sent:[String]=[]
            let m = RobotBluetoothManager(commandWriter:{sent.append(String(decoding:$0,as:UTF8.self))},settings:paritySettings())
            m.updateDrive(x:0,y:1);m.stopDrive(reason:"gesture-ended");m.updateDrive(x:0,y:-1)
            if !safety {m.stopDrive(reason:"gesture-ended")}
            m.receiveConsoleText("$SPOTDRIVE stopped reason=\(safety ? "tilt" : "requested")\nOK\n# ")
            XCTAssertEqual(sent.filter{$0.hasPrefix("drive ")}.count,1)
            m.disconnect()
        }
    }
    func testFreshPressWaitsForIdleTelemetryReplyAndPosturesRemainExclusive() {
        var sent:[String]=[]
        let m=RobotBluetoothManager(commandWriter:{sent.append(String(decoding:$0,as:UTF8.self))},settings:paritySettings())
        defer {m.disconnect()}
        m.send(.raw("read 1"));m.updateDrive(x:0,y:1)
        XCTAssertFalse(sent.contains{$0.hasPrefix("drive ")})
        m.receiveConsoleText("ID 1 voltage=12000mV\n# ")
        XCTAssertTrue(sent.contains{$0.hasPrefix("drive ")})
        m.send(.landing);XCTAssertFalse(m.joystickEnabled)
    }
    func testRestingVoltageRejectsSingleDipAndRequiresSettling() {
        var r=RobotRestingVoltage();r.becameIdle(at:0)
        XCTAssertNil(r.observe(9800,at:1))
        XCTAssertNil(r.observe(11500,at:4));XCTAssertNil(r.observe(9800,at:6))
        XCTAssertNil(r.observe(11500,at:8));XCTAssertNil(r.observe(11500,at:10))
        XCTAssertEqual(r.observe(11600,at:12),11500)
        r.invalidate();XCTAssertNil(r.observe(9800,at:15))
        var warning=RobotBatteryWarning();warning.observe(9800,at:1,historical:true)
        XCTAssertEqual(warning.level,0)
    }
    func testLoadAndHistoricalBatteryTelemetryCannotRaiseWarning() {
        let m=RobotBluetoothManager(commandWriter:{_ in},settings:paritySettings())
        defer {m.disconnect()}
        m.updateDrive(x:0,y:1)
        m.receiveConsoleText("$BATTERY mv=9800\nGait diagnostics: min_voltage=9700mV\nID 1 voltage=9600mV\n")
        XCTAssertEqual(m.batteryWarning.level,0);XCTAssertNil(m.supplyVoltageMillivolts)
    }
}


extension RobotCommandTests {
    @MainActor
    func testDarkControlScreensRender() throws {
        let m = RobotBluetoothManager(commandWriter: { _ in }, settings: UserDefaults(suiteName: "SpotOMG-ui-" + UUID().uuidString)!)
        defer { m.disconnect() }
        m.receiveConsoleText("$SPOTSTATE pose=stand safety=ok torque=off rev=attitudepd-v4-v90-r1 caps=footlift,footliftpersist,commandretry,headinghold,balancecontrol,stow lift_fl=30 lift_fr=30 lift_rl=10 lift_rr=10\n# ")
        m.parameterWalking = true
        let screen = UIHostingController(rootView: ContentView().environmentObject(m))
        let scene = try XCTUnwrap(UIApplication.shared.connectedScenes.compactMap { $0 as? UIWindowScene }.first)
        let old = scene.windows.first(where: \.isKeyWindow)
        let window = UIWindow(windowScene: scene); window.rootViewController = screen; window.makeKeyAndVisible()
        defer { window.isHidden = true; old?.makeKeyAndVisible() }
        RunLoop.main.run(until: Date().addingTimeInterval(0.5))
        screen.view.layoutIfNeeded()
        XCTAssertEqual(screen.traitCollection.userInterfaceStyle, .dark)
        func capture(_ name: String) {
            let image = UIGraphicsImageRenderer(bounds: screen.view.bounds).image { _ in
                screen.view.drawHierarchy(in: screen.view.bounds, afterScreenUpdates: true)
            }
            let attachment = XCTAttachment(image: image); attachment.name = name; attachment.lifetime = .keepAlways; add(attachment)
        }
        capture("dark-controller")
        let popup = UIHostingController(rootView: FootLiftSettingsView().environmentObject(m))
        screen.present(popup, animated: false)
        RunLoop.main.run(until: Date().addingTimeInterval(0.5))
        let popupImage = UIGraphicsImageRenderer(bounds: window.bounds).image { _ in
            window.drawHierarchy(in: window.bounds, afterScreenUpdates: true)
        }
        let popupAttachment = XCTAttachment(image: popupImage)
        popupAttachment.name = "foot-lift-popup"; popupAttachment.lifetime = .keepAlways; add(popupAttachment)
        screen.dismiss(animated: false)

        func scrollViews(_ view: UIView) -> [UIScrollView] {
            ([view as? UIScrollView].compactMap { $0 }) + view.subviews.flatMap(scrollViews)
        }
        if let list = scrollViews(screen.view).filter({ $0.contentSize.height > $0.bounds.height * 1.5 }).max(by: { $0.contentSize.height < $1.contentSize.height }) {
            for (index, offset) in [list.bounds.height + 100, list.bounds.height + 450, list.bounds.height + 900].enumerated() {
                list.setContentOffset(CGPoint(x:0,y:min(list.contentSize.height-list.bounds.height, offset)),animated:false)
                RunLoop.main.run(until:Date().addingTimeInterval(0.4));capture("dark-settings-\(index)")
            }
        }
    }
}


extension RobotCommandTests {
    func testPostureDiscardsDeferredJoystickInput() {
        var sent: [String] = []
        let m = RobotBluetoothManager(commandWriter: { sent.append(String(decoding:$0,as:UTF8.self)) })
        defer { m.disconnect() }
        m.send(.raw("read 1"));m.updateDrive(x:0,y:1)
        m.requestSafeStand()
        m.receiveConsoleText("ID 1 voltage=12000mV\n# OK\n# ")
        m.receiveConsoleText("$SPOTSTATE pose=stand safety=ok torque=on\n# ")
        m.receiveConsoleText("ID 1 voltage=12000mV\n# ")
        XCTAssertFalse(sent.contains { $0.hasPrefix("drive ") })
    }
}

extension RobotCommandTests {
    private func controlReply(_ sequence: UInt32, _ kind: String, _ payload: String = "") -> Data {
        Data("\u{1e}\(sequence) \(kind) \(payload)\u{1f}".utf8)
    }
    private func requestSequence(_ text: String) -> UInt32 { UInt32(text.split(separator: " ")[1])! }

    func testControlFramesSurviveEveryChunkBoundaryAndRejectMalformedReply() throws {
        let frame = controlReply(42, "DATA", "OK\r\n") + controlReply(42, "DONE")
        for split in 0...frame.count {
            var stream = RobotControlStream()
            let replies = try stream.append(Data(frame.prefix(split))) + stream.append(Data(frame.dropFirst(split)))
            XCTAssertEqual(replies.count, 2)
            XCTAssertEqual(replies[0].sequence, 42)
            XCTAssertEqual(replies[0].payload, "OK\r\n")
            XCTAssertTrue(replies[1].done)
        }
        var stream = RobotControlStream()
        XCTAssertThrowsError(try stream.append(controlReply(0, "DONE")))
        XCTAssertThrowsError(try RobotControlStream.request(Data("stand\nrelax\n".utf8), sequence: 1))
    }

    func testDedicatedControlIgnoresDelayedLogsAndOldCompletionWhileStopping() {
        var sent: [String] = []
        let m = RobotBluetoothManager(commandWriter: { sent.append(String(decoding: $0, as: UTF8.self)) })
        defer { m.disconnect() }
        m.receiveConsoleText("$SPOTSTATE pose=stand torque=on safety=ok caps=controlv1,commandretry\n")
        m.receiveConsoleText("ID 1 voltage=11800mV\n# ")
        m.activateSeparateControl()
        XCTAssertTrue(m.separateControl)
        sent.removeAll()
        m.updateDrive(x: 0, y: 1)
        let seq = requestSequence(sent[0])
        m.receiveControlData(controlReply(seq, "DATA", "$SPOTDRIVE started seq=1\n"))
        m.stopDrive(reason: "gesture-ended")
        XCTAssertTrue(sent.last!.hasPrefix("@S "), "Stop must bypass the active command")
        m.updateDrive(x: 0, y: -1)
        for _ in 0..<50 { m.receiveDiagnosticText("OK\n# $SPOTDRIVE stopped reason=tilt\nGait diagnostics: min_voltage=9000mV\n") }
        m.receiveControlData(controlReply(seq == 1 ? 2 : seq - 1, "DONE"))
        XCTAssertEqual(sent.filter { $0.hasPrefix("@C ") }.count, 1)
        XCTAssertNil(m.motionWarning)
        m.receiveControlData(controlReply(seq, "DATA", "$SPOTDRIVE stopped reason=ok\nOK\n"))
        m.receiveControlData(controlReply(seq, "DONE"))
        XCTAssertEqual(sent.filter { $0.hasPrefix("@C ") }.count, 2)
        XCTAssertTrue(sent.last!.contains("drive -1000"))
        XCTAssertFalse(m.consoleText.contains("Gait diagnostics"))
        m.receiveDiagnosticText("ERROR: support monitoring reason=lost servo=1; corrections paused; torque preserved\n")
        XCTAssertNotNil(m.motionWarning)
        XCTAssertEqual(sent.filter { $0.hasPrefix("@C ") }.count, 2)
    }

    func testDedicatedControlSerializesQueriesAndVoltageDoesNotDependOnLogs() {
        let old = UserDefaults.standard.object(forKey: "robotTarget")
        UserDefaults.standard.set("robot", forKey: "robotTarget")
        defer { UserDefaults.standard.set(old, forKey: "robotTarget") }
        var sent: [String] = []; var now = 0.0
        let m = RobotBluetoothManager(commandWriter: { sent.append(String(decoding: $0, as: UTF8.self)) })
        defer { m.disconnect() }
        m.telemetryClock = { now }
        m.receiveConsoleText("$SPOTSTATE pose=stand torque=on safety=ok caps=controlv1\n")
        m.receiveConsoleText("ID 1 voltage=11800mV\n# ")
        m.activateSeparateControl(); sent.removeAll()
        for time in [4.0, 6.0, 8.0] {
            now = time
            m.send(.raw("read 1"))
            let seq = requestSequence(sent.last!)
            m.receiveDiagnosticText("$BATTERY mv=9000\nGait diagnostics: min_voltage=8000mV\nID 1 voltage=7000mV\n# ")
            m.receiveControlData(controlReply(seq, "DATA", "ID 1 voltage=11800mV moving=0\n"))
            m.receiveControlData(controlReply(seq, "DONE"))
        }
        XCTAssertEqual(m.supplyVoltageMillivolts, 11800)
        XCTAssertEqual(m.batteryWarning.level, 0)
        sent.removeAll()
        m.send(.raw("read 1")); m.send(.raw("targets"))
        XCTAssertEqual(sent.count, 1)
        let seq = requestSequence(sent[0])
        m.receiveControlData(controlReply(seq, "DONE"))
        XCTAssertEqual(sent.count, 2)
        XCTAssertTrue(sent[1].contains("targets"))
    }

    func testDiagnosticExportIncludesEarlierQueuedWrites() {
        let trace = RobotConnectionTrace()
        let marker = "export-test-" + UUID().uuidString
        trace.record("diagnostic", marker)
        let ready = expectation(description: "export")
        trace.export { result in
            do {
                let url = try result.get()
                XCTAssertTrue(try String(contentsOf: url, encoding: .utf8).contains(marker))
                try FileManager.default.removeItem(at: url)
            } catch { XCTFail("\(error)") }
            ready.fulfill()
        }
        wait(for: [ready], timeout: 5)
    }
}


extension RobotCommandTests {
    func testForwardReverseTwentyDegreeSnapPreservesSpeedAndOutsideSteering() {
        for direction in [-1.0, 1.0] {
            for radius in [0.3, 0.6, 1.0] {
                let straight = RobotDriveVector.make(x: 0, y: direction * radius)!
                for degrees in [-20.0, -10, 0, 10, 20] {
                    let angle = degrees * Double.pi / 180
                    let value = RobotDriveVector.make(x: radius * sin(angle), y: direction * radius * cos(angle))!
                    XCTAssertEqual(value.linearPerMille, straight.linearPerMille)
                    XCTAssertEqual(value.yawPerMille, 0)
                    XCTAssertEqual(value.speedFraction, straight.speedFraction, accuracy: 1e-12)
                }
                for degrees in [-45.0, -20.1, 20.1, 45] {
                    let angle = degrees * Double.pi / 180
                    let value = RobotDriveVector.make(x: radius * sin(angle), y: direction * radius * cos(angle))!
                    XCTAssertGreaterThan(Double(value.yawPerMille) * degrees, 0)
                    XCTAssertGreaterThan(Double(value.linearPerMille) * direction, 0)
                }
            }
        }
    }
}


extension RobotCommandTests {
    func testSidewaysTwentyDegreeSnapBothSides() {
        for side in [-1.0, 1.0] {
            for radius in [0.3, 0.6, 1.0] {
                let pure = RobotDriveVector.make(x: side * radius, y: 0)!
                for degrees in [70.0, 80, 90, 100, 110] {
                    let a = degrees * Double.pi / 180
                    let v = RobotDriveVector.make(x: side * radius * sin(a), y: radius * cos(a))!
                    XCTAssertEqual(v.linearPerMille, 0)
                    XCTAssertEqual(v.yawPerMille, pure.yawPerMille)
                    XCTAssertEqual(v.speedFraction, pure.speedFraction, accuracy: 1e-12)
                }
                for degrees in [20.1, 45, 69.9, 110.1, 120, 140, 159.9] {
                    let a = degrees * Double.pi / 180
                    let v = RobotDriveVector.make(x: side * radius * sin(a), y: radius * cos(a))!
                    XCTAssertGreaterThan(Double(v.yawPerMille) * side, 0)
                    XCTAssertGreaterThan(Double(v.linearPerMille) * (degrees < 90 ? 1 : -1), 0)
                }
            }
        }
    }
}

extension RobotCommandTests {
    func testManualIMURecoveryIsIdleOnlyAndRequiresSuccessReply() {
        var sent: [String] = []
        let m = RobotBluetoothManager(commandWriter: { sent.append(String(decoding:$0,as:UTF8.self)) })
        defer { m.disconnect() }
        m.receiveConsoleText("$SPOTSTATE pose=stand torque=on safety=fault rev=attitudepd-v4-v90-r3 caps=controlv1,commandretry\n")
        m.receiveConsoleText("ID 1 voltage=11800mV\n# ")
        XCTAssertTrue(m.canRecoverIMU)
        m.recoverIMU()
        XCTAssertEqual(sent.last,"imurecover\n")
        XCTAssertFalse(m.canRecoverIMU)
        m.updateDrive(x:0,y:1)
        XCTAssertFalse(sent.contains { $0.hasPrefix("drive ") })
        m.receiveConsoleText("IMURECOVER OK samples=3/3\nOK IMU recovered; no motion; faults unchanged\n# ")
        XCTAssertTrue(m.imuRecoveryMessage.contains("복구 완료"))
        XCTAssertFalse(sent.contains { $0.hasPrefix("drive ") })
        XCTAssertFalse(m.imuRecoveryPending)
        m.recoverIMU();m.receiveConsoleText("ERROR: IMU recovery failed; no motion\n# ")
        XCTAssertTrue(m.imuRecoveryMessage.contains("복구 실패"))
        m.updateDrive(x:0,y:1)
        XCTAssertFalse(m.canRecoverIMU)
    }
    func testAutoIMURecoveryNotificationNeverStartsMotion() {
        var sent:[String]=[]
        let m=RobotBluetoothManager(commandWriter:{sent.append(String(decoding:$0,as:UTF8.self))})
        defer {m.disconnect()}
        m.receiveDiagnosticText("IMUAUTO started delay_ms=3000; no motion\n")
        XCTAssertTrue(m.imuRecoveryMessage.contains("복구 중"))
        m.receiveDiagnosticText("IMUAUTO result=ok; new command required\n")
        XCTAssertTrue(m.imuRecoveryMessage.contains("복구 완료"))
        XCTAssertTrue(sent.isEmpty)
    }
}
