import CoreBluetooth
import Foundation
import Combine
import Network

final class RobotBluetoothManager: NSObject, ObservableObject {
    static let deviceName = "SpotOMG-Bridge"
    static let serviceUUID = CBUUID(string: "6e400001-b5a3-f393-e0a9-e50e24dcca9e")
    static let receiveUUID = CBUUID(string: "6e400002-b5a3-f393-e0a9-e50e24dcca9e")
    static let transmitUUID = CBUUID(string: "6e400003-b5a3-f393-e0a9-e50e24dcca9e")

    @Published private(set) var postureInProgress: String?
    @Published private(set) var postureQueued = false
    @Published private(set) var stopRequested = false
    private var postureAcknowledged = false
    private var relaxAfterLanding = false
    private var relaxLandingTimeout: DispatchWorkItem?
    var motionControlsLocked: Bool { remotePending != nil || stowControlLocked || postureInProgress != nil || postureQueued || stopRequested }
    @Published private(set) var remotePending: AppRemoteRequest?
    private var remoteTimer: Timer?
    private var remotePhase = ""
    private var remoteReleaseID: UUID?
    private var remoteStopConfirmed = false
    private var remoteDirectory: URL {
        FileManager.default.urls(for: .documentDirectory, in: .userDomainMask)[0].appendingPathComponent("RemoteControl")
    }

    func pollRemoteControl() {
        let directory = remoteDirectory
        try? FileManager.default.createDirectory(at: directory, withIntermediateDirectories: true)
        let file = directory.appendingPathComponent("request.json")
        if let data = try? Data(contentsOf: file), data.count <= 4096,
           let request = try? JSONDecoder().decode(AppRemoteRequest.self, from: data) {
            try? FileManager.default.removeItem(at: file)
            let now = Date().timeIntervalSince1970
            guard request.isValid(now: now) else {
                if UUID(uuidString: request.id) != nil { remoteReply(request, error: "invalid_or_expired_request") }
                return
            }
            var consumed = UserDefaults.standard.stringArray(forKey: "remoteConsumedRequests") ?? []
            if consumed.contains(request.id) { return }
            consumed.append(request.id)
            UserDefaults.standard.set(Array(consumed.suffix(64)), forKey: "remoteConsumedRequests")
            beginRemoteRequest(request)
        }
        advanceRemoteRequest()
    }

    func beginRemoteRequest(_ request: AppRemoteRequest) {
        guard request.isValid(now: Date().timeIntervalSince1970) else { remoteReply(request, error: "invalid_or_expired_request"); return }
        if request.action == "status" { remoteReply(request); return }
        guard remotePending == nil else { remoteReply(request, error: "busy"); return }
        let moving = driveSessionActive || postureInProgress != nil || postureQueued || stopRequested
        if request.action == "connect" {
            if moving { remoteReply(request, error: "motion_in_progress"); return }
            if let name=request.target, let selection=RobotConnectionTarget(rawValue:name) {
                selectTarget(selection)
            }
            remotePending=request; remotePhase="connecting"
            if !state.isReady { connect() }
        } else {
            cancelLandingRelease()
            remotePending=request
            reconnectRequested=false
            pendingCommandAfterDrive=nil; postureQueued=false
            driveHeartbeat?.invalidate(); driveHeartbeat=nil; driveVector=nil
            if moving && state.isReady {
                remoteStopConfirmed=false; remotePhase="stopping"
                sendMotionInterrupt()
                driveStatus="원격 연결 해제 · 정지 확인 중"
            } else { releaseRemoteConnection() }
        }
        advanceRemoteRequest()
    }

    private func releaseRemoteConnection() {
        remotePhase="disconnecting"
        remoteReleaseID=peripheral?.state == .disconnected ? nil : peripheral?.identifier
        disconnect()
    }

    func advanceRemoteRequest() {
        guard let request=remotePending else { return }
        if remotePhase == "stopping", remoteStopConfirmed { releaseRemoteConnection() }
        if (remotePhase == "connecting" && state.isReady && lastStateSync != nil) ||
           (remotePhase == "disconnecting" && remoteReleaseID == nil && !state.isReady) {
            remotePending=nil; remotePhase=""; remoteReply(request); return
        }
        if Date().timeIntervalSince1970 >= request.expiresAt {
            let error=remotePhase == "stopping" ? "stop_unconfirmed" : "connection_timeout"
            if remotePhase == "connecting" { disconnect() }
            remotePending=nil; remotePhase=""; remoteReply(request,error:error)
        }
    }

    private func remoteReply(_ request: AppRemoteRequest, error: String? = nil) {
        var value: [String:Any] = ["request_id":request.id,"action":request.action,"ok":error == nil,
            "ready":state.isReady,"state":state.title,"target":target.rawValue,
            "pose":runtimeState.pose,"firmware":runtimeState.revision,
            "motion_active":driveSessionActive || postureInProgress != nil,
            "responded_at":Date().timeIntervalSince1970]
        if let error { value["error"]=error }
        let url=remoteDirectory.appendingPathComponent("response-\(request.id).json")
        if let data=try? JSONSerialization.data(withJSONObject:value,options:[.sortedKeys]) {
            try? FileManager.default.createDirectory(at:remoteDirectory,withIntermediateDirectories:true)
            try? data.write(to:url,options:.atomic)
        }
    }
    @Published private(set) var stowControlLocked = false
    @Published private(set) var state: RobotConnectionState = .disconnected
    @Published private(set) var consoleText = ""
    @Published private(set) var lastError: String?
    @Published private(set) var signalStrength: Int?
    @Published private(set) var runtimeState = RobotRuntimeState()
    @Published private(set) var lastStateSync: Date?
    @Published private(set) var driveStatus = "중립"
    @Published private(set) var supplyVoltageMillivolts: Int?
    @Published private(set) var lastVoltageRead: Date?

    @Published private(set) var target: RobotConnectionTarget =
        RobotConnectionTarget(rawValue: UserDefaults.standard.string(forKey: "robotTarget") ?? "") ?? .robot
    @Published var simulatorHost = UserDefaults.standard.string(forKey: "simulatorHost") ?? "127.0.0.1"
    @Published var simulatorPort = "8765"
    private var simulatorConnection: NWConnection?
    private var simulatorTimeout: DispatchWorkItem?
    private var simulatorIdentity = RobotConsoleStream()
    private var awaitingSimulatorIdentity = false
    private var selectedServiceUUID: CBUUID { CBUUID(string: target.serviceID) }
    private var selectedReceiveUUID: CBUUID { CBUUID(string: target.receiveID) }
    private var selectedTransmitUUID: CBUUID { CBUUID(string: target.transmitID) }

    private func receiveSimulatorConsoleText(_ text: String) {
        if state.isReady {
            receiveConsoleText(text)
            return
        }
        for event in simulatorIdentity.append(text) {
            if case .line(let line) = event, line.hasPrefix("$SIMLINK disconnected") {
                disconnect(); fail("MuJoCo 연결이 종료되었습니다."); return
            }
            if case .line(let line) = event, RobotConnectionTarget.isSimulatorIdentity(line) {
                simulatorTimeout?.cancel(); simulatorTimeout = nil
                awaitingSimulatorIdentity = false
                state = .ready
                lastError = nil
                requestInitialState()
            }
        }
    }

    private func beginSimulatorBLEIdentity() {
        awaitingSimulatorIdentity = true
        simulatorIdentity = RobotConsoleStream()
        let timeout = DispatchWorkItem { [weak self] in
            guard let self, self.awaitingSimulatorIdentity else { return }
            self.disconnect(); self.fail("가상 BLE 식별 응답 시간 초과")
        }
        simulatorTimeout?.cancel(); simulatorTimeout = timeout
        DispatchQueue.main.asyncAfter(deadline: .now() + 5, execute: timeout)
        write(Data("identity\n".utf8))
    }

    func selectTarget(_ value: RobotConnectionTarget) {
        guard value != target else { return }
        disconnect()
        target = value
        UserDefaults.standard.set(value.rawValue, forKey: "robotTarget")
        lastError = nil
    }

    private func connectSimulator() {
        disconnect()
        guard let portValue = UInt16(simulatorPort), portValue > 0,
              let port = NWEndpoint.Port(rawValue: portValue),
              !simulatorHost.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty else {
            fail("시뮬레이터 주소와 포트를 확인해 주십시오."); return
        }
        UserDefaults.standard.set(simulatorHost, forKey: "simulatorHost")
        let connection = NWConnection(host: NWEndpoint.Host(simulatorHost), port: port, using: .tcp)
        simulatorConnection = connection
        simulatorIdentity = RobotConsoleStream()
        state = .connecting
        let timeout = DispatchWorkItem { [weak self, weak connection] in
            guard let self, let connection, self.simulatorConnection === connection else { return }
            self.disconnect(); self.fail("시뮬레이터 식별 응답 시간 초과")
        }
        simulatorTimeout = timeout
        DispatchQueue.main.asyncAfter(deadline: .now() + 5, execute: timeout)
        connection.stateUpdateHandler = { [weak self, weak connection] status in
            guard let self, let connection, self.simulatorConnection === connection else { return }
            self.appendConsole("[TCP] \(status)\n")
            switch status {
            case .waiting(let error):
                self.lastError = "네트워크 연결 대기: \(error.localizedDescription)"
            case .ready:
                self.sendSimulator(Data("identity\n".utf8), connection: connection)
                self.receiveSimulator(connection)
            case .failed(let error):
                self.disconnect(); self.fail("시뮬레이터: \(error.localizedDescription)")
            default: break
            }
        }
        connection.start(queue: .main)
    }

    private func sendSimulator(_ data: Data, connection: NWConnection) {
        connection.send(content: data, completion: .contentProcessed { [weak self, weak connection] error in
            guard let self, let connection, self.simulatorConnection === connection else { return }
            if let error { self.disconnect(); self.fail("시뮬레이터 전송: \(error.localizedDescription)") }
        })
    }

    private func receiveSimulator(_ connection: NWConnection) {
        connection.receive(minimumIncompleteLength: 1, maximumLength: 8192) { [weak self, weak connection] data, _, complete, error in
            guard let self, let connection, self.simulatorConnection === connection else { return }
            if let data, !data.isEmpty {
                let text = String(decoding: data, as: UTF8.self)
                self.receiveSimulatorConsoleText(text)
            }
            if complete || error != nil {
                self.disconnect(); self.fail("시뮬레이터 연결 종료")
            } else { self.receiveSimulator(connection) }
        }
    }

    private var central: CBCentralManager?
    private(set) var hasStarted = false
    private var peripheral: CBPeripheral?
    private var receiveCharacteristic: CBCharacteristic?
    private var transmitCharacteristic: CBCharacteristic?
    private var reconnectRequested = true
    private var consoleStream = RobotConsoleStream()
    private var writes = RobotBLEWriteQueue()
    private var writeTimeout: DispatchWorkItem?
    private var trace: RobotConnectionTrace?
    private var stateRefreshWorkItem: DispatchWorkItem?
    private var initialSyncTimeout: DispatchWorkItem?
    private var initialSyncAttempts = 0
    private var driveHeartbeat: Timer?
    private var driveVector: RobotDriveVector?
    private var driveRequiresRelease = false
    private var driveSafetyLatched = false
    private var recoveryRequested = false
    private var driveSequence: UInt32 = 0
    private enum DrivePhase: String { case idle, controlling, stopping, draining }
    private var drivePhase: DrivePhase = .idle {
        didSet {
            if oldValue != drivePhase {
                trace?.record("phase", "\(oldValue.rawValue) -> \(drivePhase.rawValue)")
            }
        }
    }
    private var driveSessionActive: Bool { drivePhase != .idle }
    private var driveStopRequested: Bool { drivePhase == .stopping }
    private var driveAwaitingPrompt: Bool { drivePhase == .draining }
    private var driveCompletionTimeout: DispatchWorkItem?
    private var lastDrivePacketAt = Date.distantPast
    private var pendingCommandAfterDrive: RobotCommand?

    // Optional transport injection lets host tests exercise the actual session
    // and timer paths without connecting to (or moving) a robot.
    private let commandWriter: ((Data) -> Void)?

    init(commandWriter: ((Data) -> Void)? = nil) {
        self.commandWriter = commandWriter
        super.init()
        // Explicit launch mode used for virtual-only device validation.
        if commandWriter == nil, CommandLine.arguments.contains("--simulator-ble") {
            target = .simulatorBluetooth
            UserDefaults.standard.set(target.rawValue, forKey: "robotTarget")
        }
        if commandWriter == nil, CommandLine.arguments.contains("--simulator-tcp") {
            target = .simulator
            UserDefaults.standard.set(target.rawValue, forKey: "robotTarget")
            if let index = CommandLine.arguments.firstIndex(of: "--simulator-host"),
               index + 1 < CommandLine.arguments.count {
                simulatorHost = CommandLine.arguments[index + 1]
                UserDefaults.standard.set(simulatorHost, forKey: "simulatorHost")
            }
        }
        if commandWriter != nil {
            state = .ready
            return
        }
    }

    /// Called after the control screen has appeared, never from App/StateObject init.
    /// CoreBluetooth setup and diagnostic setup must not delay the initial layout.
    func start() {
        guard !hasStarted else { return }
        if commandWriter == nil {
            remoteTimer=Timer.scheduledTimer(withTimeInterval:0.3,repeats:true) { [weak self] _ in self?.pollRemoteControl() }
        }
        hasStarted = true
        guard commandWriter == nil else { return }
        trace = RobotConnectionTrace()
        trace?.record("ui-visible", "starting Bluetooth after first appearance")
        if target == .simulator, CommandLine.arguments.contains("--simulator-tcp") {
            connectSimulator()
            return
        }
        guard target.usesBluetooth else { return }
        central = CBCentralManager(delegate: self, queue: .main,
                                   options: [CBCentralManagerOptionShowPowerAlertKey: true])
    }

    func connect() {
        guard remotePending?.action != "disconnect" else { return }
        if target == .simulator { connectSimulator(); return }
        if central == nil, hasStarted {
            central = CBCentralManager(delegate: self, queue: .main)
        }
        reconnectRequested = true
        guard let central else {
            start()
            return
        }
        guard central.state == .poweredOn else { return }
        startScanning()
    }

    func disconnect() {
        stowControlLocked = false
        trace?.record("disconnect-request", "phase=\(drivePhase.rawValue)")
        reconnectRequested = false
        simulatorTimeout?.cancel(); simulatorTimeout = nil
        let previous = simulatorConnection
        simulatorConnection = nil
        previous?.cancel()
        central?.stopScan()
        if let peripheral { central?.cancelPeripheralConnection(peripheral) }
        resetConnection()
    }

    func permitsCommand(_ command: RobotCommand) -> Bool {
        if command.consoleLine == "hold" || command.consoleLine == "\u{03}" { return true }
        if remotePending != nil {
            if remotePhase == "connecting" {
                switch command {
                case .syncState, .synchronizeTime: return true
                default: break
                }
            }
            return false
        }
        if postureInProgress != nil || postureQueued || stopRequested {
            switch command {
            case .syncState, .targets, .gaitDiagnostics, .balanceDiagnostics: return true
            case .raw(let text):
                return ["syncstate", "targets", "gaitdiag", "baldiag", "imudiag", "locomotiondiag", "read 1", "identity"].contains(text.trimmingCharacters(in: .whitespacesAndNewlines))
            default: return false
            }
        }
        guard stowControlLocked else { return true }
        if runtimeState.pose == "stow-paused", command.consoleLine.trimmingCharacters(in: .whitespacesAndNewlines) == "stow" {
            return true
        }
        switch command {
        case .landing, .syncState, .targets, .gaitDiagnostics, .balanceDiagnostics, .storedLogs:
            return true
        case .raw(let text):
            return ["landing", "syncstate", "targets", "gaitdiag", "baldiag", "imudiag", "locomotiondiag", "read 1", "identity"].contains(text.trimmingCharacters(in: .whitespacesAndNewlines))
        default: return false
        }
    }

    private func cancelLandingRelease() {
        relaxAfterLanding = false
        relaxLandingTimeout?.cancel(); relaxLandingTimeout = nil
    }

    private func finishLandingRelease() {
        guard relaxAfterLanding, runtimeState.pose == "landing", postureInProgress == nil else { return }
        cancelLandingRelease()
        if state.isReady, remotePending == nil, let data = RobotCommand.relax.encoded {
            appendConsole("> relax\n"); write(data); scheduleStateRefresh(after: 0.2)
        }
    }

    func send(_ command: RobotCommand) {
        if (command.consoleLine == "hold" || command.consoleLine == "\u{03}"),
           motionControlsLocked || driveSessionActive {
            guard state.isReady else { return }
            cancelLandingRelease()
            pendingCommandAfterDrive = nil
            postureQueued = false
            stopRequested = true
            sendMotionInterrupt()
            if driveSessionActive {
                driveHeartbeat?.invalidate(); driveHeartbeat = nil
                driveVector = nil; drivePhase = .stopping
                armDriveCompletionTimeout()
            }
            driveStatus = "긴급 중지 요청 · 응답 대기"
            return
        }
        guard permitsCommand(command) else {
            lastError = postureInProgress == nil ? "Stow 상태에서는 Landing으로 먼저 펼쳐 주십시오." : "자세 전환 중입니다. 완료될 때까지 기다려 주십시오."; return
        }
        if command == .stow {
            guard runtimeState.capabilities.contains("stow") || (target.isSimulator && runtimeState.capabilities.contains("simstow")) else {
                lastError = "연결된 제어기를 Stow 지원 버전으로 업데이트해 주십시오."; return
            }
        }
        if command == .recover { recoveryRequested = true }
        if case .headingHold = command {
            guard runtimeState.capabilities.contains("headinghold") else {
                lastError = "직진 방향 유지는 제어기 업데이트 후 사용할 수 있습니다."; return
            }
        }
        if case .simulatorBalance = command {
            guard runtimeState.capabilities.contains("balancecontrol") || (target.isSimulator && runtimeState.capabilities.contains("simbalance")) else {
                lastError = "균형 제어 설정은 지원되는 제어기에서만 가능합니다."; return
            }
        }
        if case .simulatorProfile = command {
            guard runtimeState.capabilities.contains("gaitprofiles") || (target.isSimulator && runtimeState.capabilities.contains("simprofiles")) else {
                lastError = "보행 정책 선택은 지원되는 제어기에서만 가능합니다."; return
            }
        }
        if case .trot5 = command, !runtimeState.supportsTrot5 {
            lastError = "개선 보행은 로봇 V13 업데이트 후 사용할 수 있습니다."
            return
        }

        if command == .syncState, driveSessionActive {
            // Read-only refreshes must never release the joystick or send @S.
            // Completion always schedules a fresh snapshot.
            return
        }
        if driveSessionActive {
            if case .drive = command {
                sendCommandNow(command)
            } else {
                pendingCommandAfterDrive = command
                postureQueued = ["stow", "landing", "stand", "stand11"].contains(command.consoleLine.trimmingCharacters(in: .whitespacesAndNewlines))
                stopDrive(reason: "command: \(command.consoleLine)")
            }
            return
        }
        sendCommandNow(command)
    }

    private func sendCommandNow(_ command: RobotCommand) {
        guard state.isReady, permitsCommand(command) else { return }
        if command.consoleLine.trimmingCharacters(in: .whitespacesAndNewlines) == "relax" {
            // An explicit completion is required even when the cached pose says Landing.
            relaxAfterLanding = true
            let timeout = DispatchWorkItem { [weak self] in
                guard let self, self.relaxAfterLanding else { return }
                self.cancelLandingRelease()
                self.lastError = "Landing 완료를 확인하지 못하여 모터 해제를 취소했습니다."
            }
            relaxLandingTimeout = timeout
            DispatchQueue.main.asyncAfter(deadline: .now() + 75, execute: timeout)
            sendCommandNow(.landing)
            return
        }
        var wireCommand = command
        if case .simulatorProfile(let profile) = command, runtimeState.capabilities.contains("gaitprofiles") {
            wireCommand = .raw("gaitprofile \(profile.rawValue)")
        }
        if case .simulatorBalance(let enabled) = command, runtimeState.capabilities.contains("balancecontrol") {
            wireCommand = .raw("balance \(enabled ? "on" : "off")")
        }
        guard let data = wireCommand.encoded else { return }

        let posture = wireCommand.consoleLine.trimmingCharacters(in: .whitespacesAndNewlines)
        if ["stow", "landing", "stand", "stand11"].contains(posture) {
            postureInProgress = posture
            postureAcknowledged = false
            driveStatus = "\(posture.capitalized) 전환 중"
        }
        appendConsole("> \(wireCommand.consoleLine)\n")
        write(data)
        if let delay = command.stateRefreshDelay {
            scheduleStateRefresh(after: delay)
        }
    }

    private func write(_ data: Data, kind: RobotBLEWriteQueue.Kind = .command) {
        trace?.record("tx-enqueue", String(decoding: data, as: UTF8.self))
        if let commandWriter {
            commandWriter(data)
            return
        }
        if target == .simulator {
            if let connection = simulatorConnection { sendSimulator(data, connection: connection) }
            return
        }
        guard writes.enqueue(data, kind: kind) else {
            disconnect()
            fail("전송 대기열 초과: 연결을 해제했습니다.")
            return
        }
        drainWrites()
    }

    private func drainWrites() {
        guard state.isReady || awaitingSimulatorIdentity, let peripheral, let characteristic = receiveCharacteristic else { return }
        let type: CBCharacteristicWriteType =
            characteristic.properties.contains(.write) ? .withResponse : .withoutResponse
        let maximum = peripheral.maximumWriteValueLength(for: type)
        while type == .withResponse || peripheral.canSendWriteWithoutResponse {
            guard let chunk = writes.nextChunk(maximumLength: maximum,
                                               acknowledged: type == .withResponse) else { return }
            trace?.record("tx-write", String(decoding: chunk, as: UTF8.self))
            if type == .withResponse {
                let timeout = DispatchWorkItem { [weak self] in
                    guard let self else { return }
                    self.disconnect()
                    self.fail("BLE 쓰기 응답 시간 초과: 연결을 해제했습니다.")
                }
                writeTimeout = timeout
                DispatchQueue.main.asyncAfter(deadline: .now() + 2, execute: timeout)
            }
            peripheral.writeValue(chunk, for: characteristic, type: type)
            if type == .withResponse { return }
        }
    }

    func requestSafeStand() {
        trace?.record("scene-safety", "phase=\(drivePhase.rawValue)")
        guard state.isReady else { return }
        if driveSessionActive {
            pendingCommandAfterDrive = .stand
            sendMotionInterrupt()
            driveHeartbeat?.invalidate()
            driveHeartbeat = nil
            driveVector = nil
            drivePhase = .stopping
            armDriveCompletionTimeout()
            driveStatus = "안전 정지 요청"
        } else {
            send(.stand)
        }
    }

    func updateDrive(x: Double, y: Double) {
        guard !motionControlsLocked else { return }
        guard state.isReady, x.isFinite, y.isFinite else {
            stopDrive(reason: "invalid-or-disconnected")
            return
        }
        guard let vector = RobotDriveVector.make(x: x, y: y) else {
            driveRequiresRelease = false
            // Crossing center is a zero velocity update, not a gesture release.
            // Keep the heartbeat/session alive so reverse can follow immediately.
            if driveSessionActive && !driveStopRequested && !driveAwaitingPrompt {
                driveVector = RobotDriveVector(linearPerMille: 0, yawPerMille: 0, speedFraction: 0)
                sendDriveUpdate()
                driveStatus = "중립"
            }
            return
        }
        guard !driveSafetyLatched, !driveRequiresRelease else { return }
        // A released session cannot be revived with @D while STM32 is finishing
        // diagnostics. Wait for its prompt and require a new touch update.
        guard !driveStopRequested, !driveAwaitingPrompt else { return }
        driveVector = vector
        driveStatus = "\(vector.statusTitle) · 속도 \(Int((vector.speedFraction * 100).rounded()))%"
        if !driveSessionActive {
            stateRefreshWorkItem?.cancel()
            stateRefreshWorkItem = nil
            drivePhase = .controlling
            // The started banner is an unacknowledged console notification,
            // not a control ACK. Its loss must not terminate a held joystick.
            let sequence = nextDriveSequence()
            sendCommandNow(.drive(linearPerMille: vector.linearPerMille,
                                  yawPerMille: vector.yawPerMille,
                                  sequence: sequence))
            startDriveHeartbeat()
        } else if driveHeartbeat == nil {
            startDriveHeartbeat()
            sendDriveUpdate()
        } else if Date().timeIntervalSince(lastDrivePacketAt) >= 0.08 {
            sendDriveUpdate()
        }
    }

    func stopDrive(reason: String = "joystick-release") {
        if reason == "gesture-ended" || reason == "joystick-release" || reason == "joystick-neutral-or-disconnected" {
            driveRequiresRelease = false
        }
        guard driveSessionActive, driveVector != nil else {
            if !driveSessionActive { driveStatus = "중립" }
            return
        }
        trace?.record("stop-request", reason)
        driveVector = nil
        driveHeartbeat?.invalidate()
        driveHeartbeat = nil
        drivePhase = .stopping
        armDriveCompletionTimeout()
        sendDrivePacket(.stop(sequence: nextDriveSequence()))
        driveStatus = "중립 · 감속 정지"
    }

    func clearConsole() {
        consoleText = ""
    }

    func synchronizeState() {
        send(.syncState)
    }

    func synchronizeClock() {
        let epochMilliseconds = Int64((Date().timeIntervalSince1970 * 1000).rounded())
        send(.synchronizeTime(epochMilliseconds: epochMilliseconds))
    }

    private func startScanning() {
        guard target.usesBluetooth, reconnectRequested else { return }
        guard let central, central.state == .poweredOn else { return }
        central.stopScan()
        resetConnection(keepingState: true)
        state = .scanning
        central.scanForPeripherals(withServices: [selectedServiceUUID],
                                   options: [CBCentralManagerScanOptionAllowDuplicatesKey: false])
    }

    private func resetConnection(keepingState: Bool = false) {
        cancelLandingRelease()
        postureInProgress = nil
        postureQueued = false
        stopRequested = false
        postureAcknowledged = false
        stowControlLocked = false
        awaitingSimulatorIdentity = false
        simulatorIdentity = RobotConsoleStream()
        simulatorTimeout?.cancel(); simulatorTimeout = nil
        initialSyncTimeout?.cancel()
        initialSyncTimeout = nil
        initialSyncAttempts = 0
        supplyVoltageMillivolts = nil
        lastVoltageRead = nil
        peripheral = nil
        receiveCharacteristic = nil
        transmitCharacteristic = nil
        signalStrength = nil
        stateRefreshWorkItem?.cancel()
        stateRefreshWorkItem = nil
        driveHeartbeat?.invalidate()
        driveHeartbeat = nil
        driveVector = nil
        drivePhase = .idle
        driveCompletionTimeout?.cancel()
        driveCompletionTimeout = nil
        pendingCommandAfterDrive = nil
        consoleStream = RobotConsoleStream()
        writes = RobotBLEWriteQueue()
        writeTimeout?.cancel()
        writeTimeout = nil
        driveStatus = "중립"
        runtimeState = RobotRuntimeState()
        lastStateSync = nil
        if !keepingState { state = .disconnected }
    }

    private func appendConsole(_ text: String) {
        consoleText.append(text)
        if consoleText.count > 40_000 {
            consoleText.removeFirst(consoleText.count - 40_000)
        }
    }

    private func scheduleStateRefresh(after delay: TimeInterval) {
        stateRefreshWorkItem?.cancel()
        let work = DispatchWorkItem { [weak self] in self?.send(.syncState) }
        stateRefreshWorkItem = work
        DispatchQueue.main.asyncAfter(deadline: .now() + delay, execute: work)
    }

    // A GATT write ACK only confirms delivery to ESP32, not a STM32 reply.
    func requestInitialState(timeout: TimeInterval = 2) {
        guard state.isReady, lastStateSync == nil, !driveSessionActive else { return }
        initialSyncTimeout?.cancel()
        initialSyncAttempts += 1
        send(.syncState)
        let work = DispatchWorkItem { [weak self] in
            guard let self, self.state.isReady, self.lastStateSync == nil,
                  !self.driveSessionActive else { return }
            if self.initialSyncAttempts < 3 {
                self.requestInitialState(timeout: timeout)
            } else {
                self.fail("로봇 응답 없음: BLE는 연결되었지만 상태를 수신하지 못했습니다. 다시 연결해 주세요.")
            }
        }
        initialSyncTimeout = work
        DispatchQueue.main.asyncAfter(deadline: .now() + timeout, execute: work)
    }

    private func nextDriveSequence() -> UInt32 {
        driveSequence &+= 1
        return driveSequence
    }

    private func startDriveHeartbeat() {
        driveHeartbeat?.invalidate()
        let timer = Timer(timeInterval: 0.20, repeats: true) { [weak self] _ in
            self?.sendDriveUpdate()
        }
        timer.tolerance = 0.03
        driveHeartbeat = timer
        /* Default-mode timers can pause while a finger is tracking a SwiftUI
         * drag. Common mode keeps the safety heartbeat alive for the entire
         * joystick hold. */
        RunLoop.main.add(timer, forMode: .common)
    }

    private func sendDriveUpdate() {
        guard let vector = driveVector, driveSessionActive else { return }
        sendDrivePacket(.update(sequence: nextDriveSequence(),
                                linearPerMille: vector.linearPerMille,
                                yawPerMille: vector.yawPerMille))
    }

    private func sendDrivePacket(_ packet: RobotDriveRealtimePacket) {
        guard state.isReady else { return }
        if case .stop(let sequence) = packet {
            appendConsole("[DRIVE] 정지 요청 seq=\(sequence)\n")
        }
        switch packet {
        case .update: write(packet.encoded, kind: .update)
        case .stop: write(packet.encoded, kind: .stop)
        }
        lastDrivePacketAt = Date()
    }

    private func sendMotionInterrupt() {
        guard state.isReady else { return }
        write(Data([0x03]), kind: .interrupt)
        appendConsole("^C\n")
    }

    private func armDriveCompletionTimeout() {
        driveCompletionTimeout?.cancel()
        let work = DispatchWorkItem { [weak self] in
            guard let self, self.driveSessionActive,
                  self.driveStopRequested || self.driveAwaitingPrompt else { return }
            self.sendMotionInterrupt()
            self.disconnect()
            self.fail("보행 종료 확인 시간 초과: 안전 정지 요청 후 연결을 해제했습니다. 다시 연결해 주세요.")
        }
        driveCompletionTimeout = work
        DispatchQueue.main.asyncAfter(deadline: .now() + 5, execute: work)
    }

    private func processConsolePrompt() {
        guard driveSessionActive, driveAwaitingPrompt else { return }
        driveCompletionTimeout?.cancel()
        driveCompletionTimeout = nil
        drivePhase = .idle
        if let pending = pendingCommandAfterDrive {
            pendingCommandAfterDrive = nil
            postureQueued = false
            sendCommandNow(pending)
        } else {
            scheduleStateRefresh(after: 0.8)
        }
    }

    func receiveConsoleText(_ text: String) {
        trace?.record("rx", text)
        appendConsole(text)
        for event in consoleStream.append(text) {
            guard case .line(let line) = event else {
                processConsolePrompt()
                continue
            }
            if target.isSimulator, line.hasPrefix("$SIMLINK disconnected") {
                disconnect(); fail("MuJoCo 연결이 종료되었습니다."); return
            }
            if remotePhase == "stopping", line.hasPrefix("STOPPED:") || line.hasPrefix("$SPOTDRIVE stopped ") || line == "ERROR: motion aborted" {
                remoteStopConfirmed=true
            }
            if line.hasPrefix("STOPPED:") || line.hasPrefix("$SPOTDRIVE stopped ") {
                stopRequested = false
            }
            if let pending = postureInProgress {
                if line == "OK \(pending)" {
                    postureInProgress = nil
                    runtimeState.pose = pending
                    driveStatus = pending == "stow" ? "Stow · Landing으로 펼치기" : "중립"
                    scheduleStateRefresh(after: 0.1)
                } else if line == "OK", !target.isSimulator {
                    // Real firmware reports a bare completion ACK; confirm its
                    // resulting pose with a subsequent state snapshot.
                    postureAcknowledged = true
                    scheduleStateRefresh(after: 0.1)
                } else if line.hasPrefix("ERROR:") || line.hasPrefix("STOPPED:") {
                    if line.hasPrefix("STOPPED:"), pending == "stow" || stowControlLocked {
                        stowControlLocked = true
                        runtimeState.pose = "stow-paused"
                    }
                    postureInProgress = nil
                    scheduleStateRefresh(after: 0.1)
                }
            }
            if line.hasPrefix("STOPPED:"), !driveSessionActive {
                driveStatus = "중지됨"
                if stowControlLocked { runtimeState.pose = "stow-paused" }
                scheduleStateRefresh(after: 0.1)
            }
            if line == "OK stow" {
                stowControlLocked = true
                runtimeState.pose = "stow"
                driveStatus = "Stow · Landing으로 펼치기"
                scheduleStateRefresh(after: 0.1)
            } else if line == "OK landing" {
                stowControlLocked = false
                runtimeState.pose = "landing"
                driveStatus = "중립"
                scheduleStateRefresh(after: 0.1)
            }
            if line.hasPrefix("ERROR:") || line.hasPrefix("STOPPED:") { cancelLandingRelease() }
            if line == "OK landing" { finishLandingRelease() }
            if line.hasPrefix("ERROR:") { lastError = line }
            if line.hasPrefix("$SPOTDRIVE started ") {
                // Informational only: never arm/cancel the stop timeout here.
                continue
            }
            if line.hasPrefix("ID 1 "),
               let field = line.split(separator: " ").first(where: { $0.hasPrefix("voltage=") }),
               field.hasSuffix("mV"),
               let millivolts = Int(field.dropFirst(8).dropLast(2)),
               (1...60000).contains(millivolts) {
                supplyVoltageMillivolts = millivolts
                lastVoltageRead = Date()
            }
            if line.hasPrefix("$SPOTDRIVE stopped ") {
                let stopReason = line.lowercased()
                let safetyStop = stopReason.contains("reason=tilt") || stopReason.contains("reason=imu") || stopReason.contains("reason=safety")
                if safetyStop {
                    driveSafetyLatched = !runtimeState.capabilities.contains("commandretry")
                    recoveryRequested = false
                    runtimeState.safety = stopReason.contains("reason=imu") ? "imu" : (stopReason.contains("reason=tilt") ? "tilt" : "fault")
                    lastError = "안전 정지: 원인을 확인하고 스틱을 놓았다가 다시 조작하십시오."
                }
                if !driveStopRequested { driveRequiresRelease = true }
                driveHeartbeat?.invalidate()
                driveHeartbeat = nil
                driveVector = nil
                if driveSessionActive { drivePhase = .draining }
                if driveSessionActive { armDriveCompletionTimeout() }
                driveStatus = safetyStop ? "안전 정지 · 스틱을 놓고 다시 조작" : (line.contains("watchdog") ? "통신 지연으로 자동 정지" : "중립")
                continue
            }
            if driveSessionActive,
               line.hasPrefix("ERROR:") || line == "unknown command; type help" {
                driveRequiresRelease = true
                if line.contains("recover") || line.contains("safety") || line.contains("IMU") || line.contains("tilt") {
                    driveSafetyLatched = !runtimeState.capabilities.contains("commandretry")
                    recoveryRequested = false
                }
                // A rejected drive has no started/stopped banner. Its error
                // and prompt must still terminate the local session.
                driveHeartbeat?.invalidate()
                driveHeartbeat = nil
                driveVector = nil
                let alreadyStopping = driveStopRequested || driveAwaitingPrompt
                drivePhase = .draining
                lastError = line
                driveStatus = line.contains("drive watchdog expired") ? "조종 신호 지연 · 자동 정지" :
                    (alreadyStopping ? "보행 중단 · 오류 확인" : "보행 명령 거부")
                armDriveCompletionTimeout()
                continue
            }
            guard line.hasPrefix("$SPOTSTATE ") else { continue }
            var values: [String: String] = [:]
            for field in line.dropFirst("$SPOTSTATE ".count).split(separator: " ") {
                let pair = field.split(separator: "=", maxSplits: 1).map(String.init)
                if pair.count == 2 { values[pair[0]] = pair[1] }
            }
            runtimeState = RobotRuntimeState(
                pose: values["pose"] ?? "unknown",
                poseErrorTicks: Int(values["error"] ?? "0") ?? 0,
                torque: values["torque"] ?? "unknown",
                safety: values["safety"] ?? "unknown",
                balance: values["balance"] ?? "unknown",
                heading: values["heading"] ?? "unknown",
                revision: values["rev"] ?? "unknown",
                capabilities: Set((values["caps"] ?? "").split(separator: ",").map(String.init)),
                simulationProfile: values["profile"] ?? "legacy")
            if postureAcknowledged, let pending = postureInProgress, runtimeState.pose == pending {
                postureInProgress = nil
                postureAcknowledged = false
                driveStatus = "중립"
            }
            if ["stow", "stow-paused"].contains(runtimeState.pose) {
                stowControlLocked = true
                driveStatus = runtimeState.pose == "stow-paused" ? "중지됨 · Landing / Stow 선택" : "Stow · Landing으로 펼치기"
            } else if ["landing", "stand", "stand11"].contains(runtimeState.pose) {
                stowControlLocked = false
            }
            finishLandingRelease()
            initialSyncTimeout?.cancel()
            initialSyncTimeout = nil
            lastStateSync = Date()
            if runtimeState.capabilities.contains("commandretry") {
                // Firmware rechecks faults for each fresh command. Keep the release
                // gate from the interrupted session so held input cannot resume.
                driveSafetyLatched = false
                recoveryRequested = false
            } else if runtimeState.safety == "ok", recoveryRequested {
                driveSafetyLatched = false
                recoveryRequested = false
            } else if runtimeState.safety != "ok" && runtimeState.safety != "unknown" {
                driveSafetyLatched = true
            }
            if !driveSafetyLatched { lastError = nil }
            // Existing firmware exposes voltage via a read-only servo snapshot.
            // Never enqueue a console read while realtime drive is active.
            if !driveSessionActive {
                supplyVoltageMillivolts = nil
                lastVoltageRead = nil
                sendCommandNow(.raw("read 1"))
            }
        }
    }

    private func fail(_ message: String) {
        trace?.record("error", message)
        lastError = message
        appendConsole("[\(target.title)] \(message)\n")
    }
}

extension RobotBluetoothManager: CBCentralManagerDelegate {
    func centralManagerDidUpdateState(_ central: CBCentralManager) {
        guard target.usesBluetooth else { return }
        switch central.state {
        case .poweredOn:
            connect()
        case .poweredOff:
            resetConnection()
            state = .bluetoothUnavailable("Bluetooth 꺼짐")
        case .unauthorized:
            resetConnection()
            state = .bluetoothUnavailable("Bluetooth 권한 필요")
        case .unsupported:
            resetConnection()
            state = .bluetoothUnavailable("BLE 미지원")
        default:
            resetConnection()
        }
    }

    func centralManager(_ central: CBCentralManager,
                        didDiscover peripheral: CBPeripheral,
                        advertisementData: [String: Any], rssi RSSI: NSNumber) {
        guard target.usesBluetooth, reconnectRequested else { return }
        let advertisedName = advertisementData[CBAdvertisementDataLocalNameKey] as? String
        // The simulator has its own service UUID. Its name may be omitted by
        // macOS advertising; never infer a hardware target from a similar name.
        if target == .robot {
            guard peripheral.name == target.deviceName || advertisedName == target.deviceName else { return }
        }
        central.stopScan()
        self.peripheral = peripheral
        signalStrength = RSSI.intValue
        peripheral.delegate = self
        state = .connecting
        central.connect(peripheral)
    }

    func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
        guard target.usesBluetooth, self.peripheral === peripheral else { central.cancelPeripheralConnection(peripheral); return }
        trace?.record("connected", peripheral.identifier.uuidString)
        state = .discoveringServices
        appendConsole("[BLE] \(target.deviceName) connected\n")
        peripheral.discoverServices([selectedServiceUUID])
        peripheral.readRSSI()
    }

    func centralManager(_ central: CBCentralManager,
                        didFailToConnect peripheral: CBPeripheral, error: Error?) {
        if remoteReleaseID == peripheral.identifier { remoteReleaseID=nil; advanceRemoteRequest() }
        guard target.usesBluetooth, self.peripheral === peripheral else { return }
        fail(error?.localizedDescription ?? "연결 실패")
        resetConnection()
        if reconnectRequested { startScanning() }
    }

    func centralManager(_ central: CBCentralManager,
                        didDisconnectPeripheral peripheral: CBPeripheral,
                        error: Error?) {
        if remoteReleaseID == peripheral.identifier { remoteReleaseID=nil; advanceRemoteRequest() }
        guard target.usesBluetooth, self.peripheral === peripheral else { return }
        trace?.record("disconnected", error.map { String(describing: $0) } ?? "no-error")
        if let error { fail("연결 끊김: \(error.localizedDescription)") }
        resetConnection()
        if reconnectRequested { startScanning() }
    }
}

extension RobotBluetoothManager: CBPeripheralDelegate {
    func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) {
        guard target.usesBluetooth, self.peripheral === peripheral else { return }
        if let error { fail("서비스 검색 실패: \(error.localizedDescription)"); return }
        guard let service = peripheral.services?.first(where: { $0.uuid == selectedServiceUUID }) else {
            fail("SpotOMG BLE 서비스를 찾지 못했습니다")
            return
        }
        peripheral.discoverCharacteristics([selectedReceiveUUID, selectedTransmitUUID], for: service)
    }

    func peripheral(_ peripheral: CBPeripheral,
                    didDiscoverCharacteristicsFor service: CBService, error: Error?) {
        guard target.usesBluetooth, self.peripheral === peripheral else { return }
        if let error { fail("특성 검색 실패: \(error.localizedDescription)"); return }
        for characteristic in service.characteristics ?? [] {
            if characteristic.uuid == selectedReceiveUUID { receiveCharacteristic = characteristic }
            if characteristic.uuid == selectedTransmitUUID { transmitCharacteristic = characteristic }
        }
        guard receiveCharacteristic != nil, let transmitCharacteristic else {
            fail("필수 BLE 특성을 찾지 못했습니다")
            return
        }
        // Clear a retained subscription before enabling notifications anew.
        peripheral.setNotifyValue(!transmitCharacteristic.isNotifying,
                                  for: transmitCharacteristic)
    }

    func peripheral(_ peripheral: CBPeripheral,
                    didUpdateNotificationStateFor characteristic: CBCharacteristic,
                    error: Error?) {
        guard target.usesBluetooth, self.peripheral === peripheral else { return }
        if let error { fail("알림 활성화 실패: \(error.localizedDescription)"); return }
        guard peripheral == self.peripheral,
              characteristic.uuid == selectedTransmitUUID else { return }
        trace?.record("notify-state", "isNotifying=\(characteristic.isNotifying)")
        if !characteristic.isNotifying {
            peripheral.setNotifyValue(true, for: characteristic)
            return
        }
        if characteristic.isNotifying {
            if target == .simulatorBluetooth { beginSimulatorBLEIdentity(); return }
            state = .ready
            lastError = nil
            synchronizeClock()
            requestInitialState()
        }
    }

    func peripheral(_ peripheral: CBPeripheral,
                    didUpdateValueFor characteristic: CBCharacteristic, error: Error?) {
        guard target.usesBluetooth, self.peripheral === peripheral else { return }
        if let error { fail("수신 실패: \(error.localizedDescription)"); return }
        guard characteristic.uuid == selectedTransmitUUID,
              let data = characteristic.value else { return }
        let text = String(decoding: data, as: UTF8.self)
        if target == .simulatorBluetooth { receiveSimulatorConsoleText(text) }
        else { receiveConsoleText(text) }
    }

    func peripheral(_ peripheral: CBPeripheral, didReadRSSI RSSI: NSNumber, error: Error?) {
        guard target.usesBluetooth, self.peripheral === peripheral else { return }
        if error == nil { signalStrength = RSSI.intValue }
    }

    func peripheral(_ peripheral: CBPeripheral,
                    didWriteValueFor characteristic: CBCharacteristic, error: Error?) {
        guard target.usesBluetooth, self.peripheral === peripheral else { return }
        guard peripheral == self.peripheral,
              characteristic.uuid == selectedReceiveUUID else { return }
        writeTimeout?.cancel()
        writeTimeout = nil
        if let error {
            disconnect()
            fail("전송 실패: \(error.localizedDescription)")
            return
        }
        trace?.record("tx-ack", "ok")
        writes.acknowledge()
        drainWrites()
    }

    func peripheralIsReady(toSendWriteWithoutResponse peripheral: CBPeripheral) {
        guard peripheral == self.peripheral else { return }
        drainWrites()
    }
}
