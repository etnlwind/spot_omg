import Foundation

/// Keeps each UART line intact across GATT chunks, with only one acknowledged
/// write in flight. Unsent joystick targets are replaced by the latest target.
struct RobotBLEWriteQueue {
    enum Kind { case command, update, stop, interrupt }
    private struct Message {
        let data: Data
        let kind: Kind
    }
    private var pending: [Message] = []
    private var current: Data?
    private var offset = 0
    private var awaitingAcknowledgement = false

    mutating func enqueue(_ data: Data, kind: Kind) -> Bool {
        guard !data.isEmpty else { return true }
        if kind == .update || kind == .stop || kind == .interrupt {
            pending.removeAll { $0.kind == .update }
        }
        if kind == .stop || kind == .interrupt {
            // Do not let stale queued commands follow an emergency stop.
            // Finish only the partially transmitted line to avoid corruption.
            pending.removeAll()
            pending.append(Message(data: data, kind: kind))
        } else {
            guard pending.count < 16 else { return false }
            pending.append(Message(data: data, kind: kind))
        }
        return true
    }

    mutating func nextChunk(maximumLength: Int, acknowledged: Bool) -> Data? {
        guard !awaitingAcknowledgement, maximumLength > 0 else { return nil }
        if current == nil {
            guard !pending.isEmpty else { return nil }
            current = pending.removeFirst().data
            offset = 0
        }
        guard let data = current else { return nil }
        let end = min(offset + maximumLength, data.count)
        let chunk = data.subdata(in: offset..<end)
        offset = end
        if offset == data.count { current = nil }
        awaitingAcknowledgement = acknowledged
        return chunk
    }

    mutating func acknowledge() { awaitingAcknowledgement = false }
}

/// STM32 prompts have no newline and may share a BLE notification with the
/// next response. Consume prompts separately, preserving wire order.
struct RobotConsoleStream {
    enum Event: Equatable {
        case line(String)
        case prompt
    }

    private var buffer = ""

    mutating func append(_ text: String) -> [Event] {
        buffer.append(text)
        var events: [Event] = []
        while !buffer.isEmpty {
            if buffer.hasPrefix("# ") {
                buffer.removeFirst(2)
                events.append(.prompt)
            } else if let newline = buffer.firstIndex(where: { $0.isNewline }) {
                let line = String(buffer[..<newline])
                    .trimmingCharacters(in: .whitespacesAndNewlines)
                buffer.removeSubrange(...newline)
                if !line.isEmpty { events.append(.line(line)) }
            } else {
                break
            }
        }
        // Bound damaged/unterminated input, including unsolicited console text.
        if buffer.utf8.count > 8192 { buffer = "" }
        return events
    }
}

enum RobotCommand: Equatable {
    case simulatorProfile(SimulatorGaitProfile)
    case simulatorBalance(Bool)
    case stand
    case stand11
    case landing
    case hold
    case relax
    case recover
    case syncState
    case targets
    case scan
    case gaitDiagnostics
    case balanceDiagnostics
    case synchronizeTime(epochMilliseconds: Int64)
    case storedLogs(count: Int)
    case clearStoredLogs
    case trot5(cycles: Int, periodMilliseconds: Int)
    case trot4(cycles: Int, periodMilliseconds: Int)
    case trot4Backward(cycles: Int, periodMilliseconds: Int)
    case turnLeft(cycles: Int, periodMilliseconds: Int)
    case turnRight(cycles: Int, periodMilliseconds: Int)
    case drive(linearPerMille: Int, yawPerMille: Int, sequence: UInt32)
    case crabLeft(cycles: Int, periodMilliseconds: Int)
    case crabRight(cycles: Int, periodMilliseconds: Int)
    case raw(String)

    var consoleLine: String {
        switch self {
        case .simulatorBalance(let enabled): return "simbalance \(enabled ? "on" : "off")"
        case .simulatorProfile(let profile): return "simprofile \(profile.rawValue)"
        case .stand: return "stand"
        case .stand11: return "stand11"
        case .landing: return "landing"
        case .hold: return "hold"
        case .relax: return "relax"
        case .recover: return "recover"
        case .syncState: return "syncstate"
        case .targets: return "targets"
        case .scan: return "scan"
        case .gaitDiagnostics: return "gaitdiag"
        case .balanceDiagnostics: return "baldiag"
        case .synchronizeTime(let epoch): return "log time \(epoch)"
        case .storedLogs(let count): return "log show \(count)"
        case .clearStoredLogs: return "log clear"
        case .trot5(let cycles, let period): return "trot5 \(cycles) \(period)"
        case .trot4(let cycles, let period): return "trot4 \(cycles) \(period)"
        case .trot4Backward(let cycles, let period): return "trot4back \(cycles) \(period)"
        case .turnLeft(let cycles, let period): return "turn left \(cycles) \(period)"
        case .turnRight(let cycles, let period): return "turn right \(cycles) \(period)"
        case .drive(let linear, let yaw, let sequence):
            return "drive \(linear) \(yaw) \(sequence)"
        case .crabLeft(let cycles, let period): return "crab left \(cycles) \(period)"
        case .crabRight(let cycles, let period): return "crab right \(cycles) \(period)"
        case .raw(let line): return line.trimmingCharacters(in: .whitespacesAndNewlines)
        }
    }

    var encoded: Data? {
        guard !consoleLine.isEmpty else { return nil }
        return (consoleLine + "\n").data(using: .utf8)
    }

    var stateRefreshDelay: TimeInterval? {
        switch self {
        case .stand, .stand11, .landing, .hold, .recover: return 4
        case .simulatorProfile, .simulatorBalance: return 0.4
        case .relax: return 1
        case .trot5(let cycles, let period):
            return Double(cycles * period) / 1000.0 + 6
        case .trot4(let cycles, let period),
             .trot4Backward(let cycles, let period),
             .turnLeft(let cycles, let period),
             .turnRight(let cycles, let period),
             .crabLeft(let cycles, let period),
             .crabRight(let cycles, let period):
            return Double(cycles * period) / 1000.0 + 4
        default: return nil
        }
    }
}

struct RobotDriveVector: Equatable {
    static let deadZone = 0.15
    static let minimumMotion = 0.30

    let linearPerMille: Int
    let yawPerMille: Int
    let speedFraction: Double

    static func make(x: Double, y: Double) -> Self? {
        guard x.isFinite, y.isFinite else { return nil }
        let magnitude = min(1.0, hypot(x, y))
        guard magnitude >= deadZone else { return nil }
        let speed = min(1.0, max(0.0,
            (magnitude - deadZone) / (1.0 - deadZone)))
        let motion = minimumMotion + (1.0 - minimumMotion) * speed
        let axisScale = motion * 1000.0 / max(magnitude, 0.0001)
        // A narrow vertical corridor rejects finger drift while walking.
        // The matching horizontal corridor keeps left/right input in place.
        // Both axes grow continuously outside their corridors.
        let steeringDeadZone = 0.10 * min(1.0, abs(y))
        let steering = max(0, abs(x) - steeringDeadZone) / (1 - steeringDeadZone)
        let signedSteering = x < 0 ? -steering : steering
        let longitudinalDeadZone = 0.10 * min(1.0, abs(x))
        let longitudinal = max(0, abs(y) - longitudinalDeadZone) / (1 - longitudinalDeadZone)
        let signedLongitudinal = y < 0 ? -longitudinal : longitudinal
        return Self(
            linearPerMille: Int((signedLongitudinal * axisScale).rounded()),
            yawPerMille: Int((signedSteering * axisScale).rounded()),
            speedFraction: speed)
    }

    var statusTitle: String {
        let longitudinal = linearPerMille > 80 ? "전진" :
            (linearPerMille < -80 ? "후진" : "")
        let turning = yawPerMille > 80 ? "우회전" :
            (yawPerMille < -80 ? "좌회전" : "")
        if linearPerMille == 0 && !turning.isEmpty { return "제자리 " + turning }
        return [longitudinal, turning].filter { !$0.isEmpty }.joined(separator: " + ")
    }
}

enum RobotDriveRealtimePacket: Equatable {
    case update(sequence: UInt32, linearPerMille: Int, yawPerMille: Int)
    case stop(sequence: UInt32)

    var encoded: Data {
        let line: String
        switch self {
        case .update(let sequence, let linear, let yaw):
            line = "@D \(sequence) \(linear) \(yaw)\n"
        case .stop(let sequence):
            line = "@S \(sequence)\n"
        }
        return Data(line.utf8)
    }
}
