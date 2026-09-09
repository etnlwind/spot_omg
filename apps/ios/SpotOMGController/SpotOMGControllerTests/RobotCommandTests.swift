import XCTest
import SwiftUI
import Network
@testable import SpotOMGController

final class RobotCommandTests: XCTestCase {
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
        var commands: [String] = []
        let manager = RobotBluetoothManager(commandWriter: { commands.append(String(decoding: $0, as: UTF8.self)) })
        manager.receiveConsoleText("$SPOTSTATE pose=custom rev=forward11-v12\r\n")
        XCTAssertEqual(commands, ["read 1\n"])
        manager.receiveConsoleText("ID 1 pos=1941 voltage=98")
        XCTAssertNil(manager.supplyVoltageMillivolts)
        manager.receiveConsoleText("00mV temp=29C\r\n")
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
        manager.send(.relax) // schedules a snapshot after one second
        manager.updateDrive(x: 0, y: 1)
        manager.receiveConsoleText("# $SPOTDRIVE started seq=1\r\n")
        RunLoop.main.run(until: Date().addingTimeInterval(1.2))
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
        XCTAssertGreaterThan(RobotDriveVector.make(x: 0.3, y: 1)!.yawPerMille, 100)
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
        while manager.lastVoltageRead == nil && Date() < deadline { RunLoop.main.run(until: Date().addingTimeInterval(0.01)) }
        XCTAssertEqual(manager.state, .ready, manager.lastError ?? manager.consoleText)
        XCTAssertEqual(manager.runtimeState.revision, "test-sim")
        XCTAssertEqual(manager.supplyVoltageMillivolts, 11100)
        XCTAssertEqual(Array(lines.prefix(3)), ["identity", "syncstate", "read 1"])
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
