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
    var capabilities: Set<String> = []
    var simulationProfile = "legacy"
    var balanceTitle: String {
        switch balance {
        case "active": return "수평 보정 작동 중"
        case "suspended": return "수평 보정 대기 · 자세/센서 확인"
        case "off": return "수평 보정 꺼짐"
        case "monitor": return "IMU 관측만 적용"
        default: return "균형 제어 확인 중"
        }
    }
    var supportsTrot5: Bool { capabilities.contains("trot5") }

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

enum RobotDriveDirection: String {
    case forward
    case backward
    case left
    case right

    var title: String {
        switch self {
        case .forward: return "전진"
        case .backward: return "후진"
        case .left: return "좌회전"
        case .right: return "우회전"
        }
    }
}


enum RobotConnectionTarget: String, CaseIterable {
    case robot, simulatorBluetooth, simulator
    var isSimulator: Bool { self != .robot }
    var usesBluetooth: Bool { self != .simulator }
    var title: String {
        switch self {
        case .robot: return "실제 로봇 · BLE"
        case .simulatorBluetooth: return "가상 로봇 · BLE"
        case .simulator: return "가상 로봇 · TCP"
        }
    }
    var deviceName: String { self == .robot ? "SpotOMG-Bridge" : "SpotOMG-Sim" }
    var serviceID: String { self == .robot ? "6e400001-b5a3-f393-e0a9-e50e24dcca9e" : "6e400101-b5a3-f393-e0a9-e50e24dcca9e" }
    var receiveID: String { self == .robot ? "6e400002-b5a3-f393-e0a9-e50e24dcca9e" : "6e400102-b5a3-f393-e0a9-e50e24dcca9e" }
    var transmitID: String { self == .robot ? "6e400003-b5a3-f393-e0a9-e50e24dcca9e" : "6e400103-b5a3-f393-e0a9-e50e24dcca9e" }
    static func isSimulatorIdentity(_ line: String) -> Bool {
        let fields = Set(line.split(separator: " ").map(String.init))
        return line.hasPrefix("$SPOTBACKEND ") && fields.contains("backend=sim") && fields.contains("protocol=1")
    }
}


enum SimulatorGaitProfile: String, CaseIterable {
    case legacy, crawl, cruise, trot, highstep
    var title: String {
        switch self {
        case .legacy: return "기존 V16"
        case .crawl: return "크롤 · 한 발씩"
        case .cruise: return "크루즈 · 넓은 보폭"
        case .trot: return "빠른 트롯 · 실험"
        case .highstep: return "높은 발 들기"
        }
    }
}
