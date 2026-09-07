import Foundation

enum RobotConnectionState: Equatable {
    case bluetoothUnavailable(String)
    case scanning
    case connecting
    case discoveringServices
    case ready
    case disconnected

    var title: String {
        switch self {
        case .bluetoothUnavailable(let reason): return reason
        case .scanning: return "로봇 검색 중"
        case .connecting: return "연결 중"
        case .discoveringServices: return "서비스 확인 중"
        case .ready: return "연결됨"
        case .disconnected: return "연결 안 됨"
        }
    }

    var isReady: Bool { self == .ready }
}

struct RobotRuntimeState: Equatable {
    var pose = "unknown"
    var poseErrorTicks = 0
    var torque = "unknown"
    var safety = "unknown"
    var balance = "unknown"
    var revision = "unknown"

    var poseTitle: String {
        switch pose {
        case "stand": return "Stand"
        case "stand11": return "Stand11"
        case "landing": return "Landing"
        case "custom": return "Custom"
        default: return "Unknown"
        }
    }
}
