import XCTest
@testable import SpotOMGController

final class RobotCommandTests: XCTestCase {
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
