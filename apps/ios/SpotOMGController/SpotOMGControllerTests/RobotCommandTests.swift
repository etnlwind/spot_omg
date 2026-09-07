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
}
