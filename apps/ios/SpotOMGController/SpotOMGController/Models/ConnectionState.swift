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
    var heading = "unknown"
    var revision = "unknown"
    var capabilities: Set<String> = []
    var simulationProfile = "legacy"
    var balanceTitle: String {
        switch balance {
        case "active": return "수평 보정 작동 중"
        case "full", "normal": return "수평 보정 켜짐"
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
        case "stow": return "Stow"
        case "stow-paused": return "Stow 중단"
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
    case attitudepd_v2, s_native_v6_2_7, s_native_v6_2_6, s_native_v6_2_5, s_native_v6_2_4, s_native_v6_2_3, s_native_v6_2_2, s_native_v6_2_1, s_native_v6_2, s_native_v6_1, s_native_v6, s_native_v5, s_native_v4, s_native_v3, s_native_v2, s_native_v1
    static var newest: Self { .allCases[0] }
    case attitudepd, centerpivot, arcsupport, arcturn, legacy, crawl, cruise, trot, highstep, lift, imu, level, level15, joint, jointfast, jointsport
    case cushion_reach, cushion_j2lift, cushion_wbc, cushion_forward, cushion_support_shift, cushion_support_shift_v2, cushion_v2_push, cushion_diagonal_sync_wide80
    func isSupported(capabilities: Set<String>) -> Bool {
        (self != .attitudepd_v2 || capabilities.contains("attitudepd_v2")) &&
        (self != .s_native_v6_2_7 || capabilities.contains("s_native_v6_2_7")) &&
        (self != .s_native_v6_2_6 || capabilities.contains("s_native_v6_2_6")) &&
        (self != .s_native_v6_2_5 || capabilities.contains("s_native_v6_2_5")) &&
        (self != .s_native_v6_2_4 || capabilities.contains("s_native_v6_2_4")) &&
        (self != .s_native_v6_2_3 || capabilities.contains("s_native_v6_2_3")) &&
        (self != .s_native_v6_2_2 || capabilities.contains("s_native_v6_2_2")) &&
        (self != .s_native_v6_2_1 || capabilities.contains("s_native_v6_2_1")) &&
        (self != .s_native_v6_2 || capabilities.contains("s_native_v6_2")) &&
        (self != .s_native_v6_1 || capabilities.contains("s_native_v6_1")) &&
        (self != .s_native_v6 || capabilities.contains("s_native_v6")) &&
        (self != .s_native_v5 || capabilities.contains("s_native_v5")) &&
        (self != .s_native_v4 || capabilities.contains("s_native_v4")) &&
        (self != .s_native_v3 || capabilities.contains("s_native_v3")) &&
        (self != .s_native_v2 || capabilities.contains("s_native_v2")) &&
        (self != .s_native_v1 || capabilities.contains("s_native_v1")) &&
        (self != .arcsupport || capabilities.contains("arcsupport")) &&
        (self != .centerpivot || capabilities.contains("centerpivot")) &&
        (self != .attitudepd || capabilities.contains("attitudepd"))
    }
    var simulatorOnly: Bool { [.s_native_v6_2, .s_native_v6, .s_native_v5, .s_native_v4, .s_native_v3, .s_native_v2, .s_native_v1, .cushion_reach, .cushion_j2lift, .cushion_wbc, .cushion_forward, .cushion_support_shift, .cushion_support_shift_v2, .cushion_v2_push, .cushion_diagonal_sync_wide80].contains(self) }
    var title: String {
        switch self {
        case .s_native_v6_2_7: return "V6.2.7 · 이른 접힘"
        case .s_native_v6_2_6: return "V6.2.6 · 연속 내딛기"
        case .s_native_v6_2_5: return "V6.2.5 · 신속 회수"
        case .s_native_v6_2_4: return "V6.2.4 · 도달 확인"
        case .s_native_v6_2_3: return "V6.2.3 · 확장"
        case .s_native_v6_2_2: return "V6.2.2 · 고속"
        case .s_native_v6_2_1: return "V6.2.1 · 연속 회수"
        case .s_native_v6_2: return "V6.2 · 뒤쪽 125mm"
        case .s_native_v6_1: return "V6.1 · 뒤쪽 65mm"
        case .s_native_v6: return "V6 · 수평 지지 교대"
        case .s_native_v5: return "V5 · FR 첫걸음"
        case .s_native_v4: return "V4 · J1 아래 착지"
        case .s_native_v3: return "V3 · 뒤로 밀기 60mm"
        case .s_native_v2: return "V2 · 연속 교대"
        case .s_native_v1: return "V1 · 대각선 동기"
        case .attitudepd_v2: return "IMU 자세 안정화 V2 · 앞발 들림 +4mm · 실험"
        case .attitudepd: return "IMU 자세 안정화 · PD · 실험"
        case .centerpivot: return "몸체 중심 회전 보정 · 실험"
        case .arcsupport: return "원호 턴 · 지지 전환 보정 · 실험"
        case .arcturn: return "원호 턴 · 실물 시험"
        case .cushion_diagonal_sync_wide80: return "대각선 · 넓은 보폭 80mm"
        case .cushion_v2_push: return "V2 · 추진 타이밍 · 실험"
        case .cushion_support_shift_v2: return "V2 · J1 고정 · 실험"
        case .cushion_support_shift: return "큰 스텝 · 지지 전환 보정 · 실험"
        case .cushion_forward: return "큰 전진 스텝 · 실험"
        case .cushion_reach: return "쿠션 · 어깨 앞 착지"
        case .cushion_j2lift: return "쿠션 · J2 높이 들기"
        case .cushion_wbc: return "전신 수평 · 아치 보행 · 실험"
        case .legacy: return "기존 V16"
        case .crawl: return "크롤 · 한 발씩"
        case .cruise: return "크루즈 · 넓은 보폭"
        case .trot: return "빠른 트롯 · 실험"
        case .jointsport: return "J2 협응 · 스포츠"
        case .jointfast: return "J2 협응 · 빠르게"
        case .joint: return "J2·J3 협응 보행"
        case .level15: return "수평 + 발 들기 · 15mm"
        case .level: return "수평 우선 · Level"
        case .imu: return "IMU 적응 · 지지 다리 보정"
        case .lift: return "리프트 · 발 높이 20mm"
        case .highstep: return "높은 발 들기"
        }
    }
}

// BEGIN GENERATED GAIT SPEEDS
// Generated by tools/generate_gait_speed_labels.py. Nominal simulation, 60s full forward.
extension SimulatorGaitProfile {
    var benchmarkSpeedMetersPerSecond: Double? {
        switch self {
        case .s_native_v6_2_7, .s_native_v6_2_6, .s_native_v6_2_5, .s_native_v6_2_4, .s_native_v6_2_3, .s_native_v6_2_2, .s_native_v6_2_1, .s_native_v6_2, .s_native_v6_1, .s_native_v6, .s_native_v5, .s_native_v4, .s_native_v3, .s_native_v2, .s_native_v1: return nil
        case .legacy: return 0.047956415
        case .crawl: return 0.028477535
        case .cruise: return 0.157175313
        case .trot: return 0.173329346
        case .highstep: return 0.089345535
        case .lift: return 0.089345535
        case .imu: return 0.078093022
        case .level: return 0.099763812
        case .level15: return 0.076044601
        case .joint: return 0.085797550
        case .jointfast: return 0.105461897
        case .jointsport: return 0.138690490
        case .arcturn: return nil
        case .arcsupport: return nil
        case .centerpivot: return nil
        case .attitudepd: return nil
        case .attitudepd_v2: return nil
        case .cushion_reach: return 0.045898422
        case .cushion_j2lift: return 0.034639744
        case .cushion_forward: return nil
        case .cushion_wbc: return nil
        case .cushion_support_shift_v2: return nil
        case .cushion_v2_push: return nil
        case .cushion_diagonal_sync_wide80: return nil
        case .cushion_support_shift: return nil
        }
    }

    static func speedSuffix(_ speed: Double?) -> String {
        guard let speed, speed.isFinite, speed >= 0 else { return "(검증실패)" }
        return "(\(String(format: "%.3f", locale: Locale(identifier: "en_US_POSIX"), speed))m/s)"
    }

    var titleWithSpeed: String { if self == .s_native_v6_2_7 || self == .s_native_v6_2_6 || self == .s_native_v6_2_5 || self == .s_native_v6_2_4 || self == .s_native_v6_2_3 || self == .s_native_v6_2_2 || self == .s_native_v6_2_1 || self == .s_native_v6_2 || self == .s_native_v6_1 || self == .s_native_v6 || self == .s_native_v5 || self == .s_native_v4 || self == .s_native_v3 || self == .s_native_v2 || self == .s_native_v1 { return "\(title) · 실험" }; return "\(title) \(Self.speedSuffix(benchmarkSpeedMetersPerSecond))" }
}
// END GENERATED GAIT SPEEDS

/// 3S LiPo warning policy. Servo-rail voltage is not a cell-level fuel gauge.
struct RobotBatteryWarning {
    static let chargeMV = 11_000
    static let criticalMV = 10_500
    static let recoveredMV = 11_400
    private(set) var level = 0
    private(set) var detectedMV: Int?
    private var recoveryStarted: TimeInterval?
    private var recoveryLast: TimeInterval?
    private var recoveryCount = 0

    mutating func observe(_ mv: Int, at now: TimeInterval, historical: Bool = false) {
        guard (1...60_000).contains(mv) else { return }
        let incoming = mv <= Self.criticalMV ? 2 : mv <= Self.chargeMV ? 1 : 0
        if incoming > 0 {
            if incoming >= level { detectedMV = min(detectedMV ?? mv, mv) }
            level = max(level, incoming)
        }
        // A completed gait's minimum can raise a warning, never clear one.
        if historical { return }
        guard level > 0, mv >= Self.recoveredMV else {
            recoveryStarted = nil; recoveryLast = nil; recoveryCount = 0; return
        }
        if let last = recoveryLast, now - last < 1 { return }
        if let last = recoveryLast, now - last > 15 {
            recoveryStarted = nil; recoveryCount = 0
        }
        if recoveryStarted == nil { recoveryStarted = now }
        recoveryLast = now; recoveryCount += 1
        if recoveryCount >= 3, now - (recoveryStarted ?? now) >= 5 {
            self = RobotBatteryWarning()
        }
    }

    var title: String { level == 2 ? "즉시 사용 중단 · 배터리 충전" : "배터리 부족 · 지금 충전하세요" }
    var message: String {
        let voltage = detectedMV.map { String(format: "감지 전압 %.1fV. ", Double($0) / 1000) } ?? ""
        return voltage + "보행을 멈추고 몸체를 지지한 뒤 전원 스위치를 끄고 충전하세요."
    }

    static func reading(in line: String) -> (millivolts: Int, historical: Bool)? {
        let prefix: String
        let historical: Bool
        if line.hasPrefix("ID 1 ") { prefix = "voltage="; historical = false }
        else if line.hasPrefix("$BATTERY ") { prefix = "mv="; historical = false }
        else if line.hasPrefix("Gait diagnostics:") { prefix = "min_voltage="; historical = true }
        else { return nil }
        guard let field = line.split(separator: " ").first(where: { $0.hasPrefix(prefix) }) else { return nil }
        var value = field.dropFirst(prefix.count)
        if prefix != "mv=" {
            guard value.hasSuffix("mV") else { return nil }
            value = value.dropLast(2)
        }
        guard let mv = Int(value), (1...60_000).contains(mv) else { return nil }
        return (mv, historical)
    }
}
