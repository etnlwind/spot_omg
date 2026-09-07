import Foundation

enum RobotCommand: Equatable {
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
    case trot4(cycles: Int, periodMilliseconds: Int)
    case crabLeft(cycles: Int, periodMilliseconds: Int)
    case crabRight(cycles: Int, periodMilliseconds: Int)
    case raw(String)

    var consoleLine: String {
        switch self {
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
        case .trot4(let cycles, let period): return "trot4 \(cycles) \(period)"
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
        case .relax: return 1
        case .trot4(let cycles, let period),
             .crabLeft(let cycles, let period),
             .crabRight(let cycles, let period):
            return Double(cycles * period) / 1000.0 + 4
        default: return nil
        }
    }
}
