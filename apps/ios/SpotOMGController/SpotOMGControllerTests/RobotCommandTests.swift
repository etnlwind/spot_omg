import XCTest
@testable import SpotOMGController

final class RobotCommandTests: XCTestCase {
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
