import CoreBluetooth
import Foundation
import Combine

final class RobotBluetoothManager: NSObject, ObservableObject {
    static let deviceName = "SpotOMG-Bridge"
    static let serviceUUID = CBUUID(string: "6e400001-b5a3-f393-e0a9-e50e24dcca9e")
    static let receiveUUID = CBUUID(string: "6e400002-b5a3-f393-e0a9-e50e24dcca9e")
    static let transmitUUID = CBUUID(string: "6e400003-b5a3-f393-e0a9-e50e24dcca9e")

    @Published private(set) var state: RobotConnectionState = .disconnected
    @Published private(set) var consoleText = ""
    @Published private(set) var lastError: String?
    @Published private(set) var signalStrength: Int?
    @Published private(set) var runtimeState = RobotRuntimeState()
    @Published private(set) var lastStateSync: Date?
    @Published private(set) var driveStatus = "중립"

    private var central: CBCentralManager!
    private var peripheral: CBPeripheral?
    private var receiveCharacteristic: CBCharacteristic?
    private var transmitCharacteristic: CBCharacteristic?
    private var reconnectRequested = true
    private var consoleStream = RobotConsoleStream()
    private var writes = RobotBLEWriteQueue()
    private var writeTimeout: DispatchWorkItem?
    private let trace: RobotConnectionTrace?
    private var stateRefreshWorkItem: DispatchWorkItem?
    private var driveHeartbeat: Timer?
    private var driveVector: RobotDriveVector?
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
        trace = commandWriter == nil ? RobotConnectionTrace() : nil
        super.init()
        if commandWriter != nil {
            state = .ready
            return
        }
        central = CBCentralManager(delegate: self, queue: .main,
                                   options: [CBCentralManagerOptionShowPowerAlertKey: true])
    }

    func connect() {
        reconnectRequested = true
        guard central.state == .poweredOn else { return }
        startScanning()
    }

    func disconnect() {
        trace?.record("disconnect-request", "phase=\(drivePhase.rawValue)")
        reconnectRequested = false
        central?.stopScan()
        if let peripheral { central.cancelPeripheralConnection(peripheral) }
        resetConnection()
    }

    func send(_ command: RobotCommand) {
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
                stopDrive(reason: "command: \(command.consoleLine)")
            }
            return
        }
        sendCommandNow(command)
    }

    private func sendCommandNow(_ command: RobotCommand) {
        guard state.isReady, let data = command.encoded else { return }

        appendConsole("> \(command.consoleLine)\n")
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
        guard writes.enqueue(data, kind: kind) else {
            disconnect()
            fail("전송 대기열 초과: 연결을 해제했습니다.")
            return
        }
        drainWrites()
    }

    private func drainWrites() {
        guard state.isReady, let peripheral, let characteristic = receiveCharacteristic else { return }
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
        guard state.isReady, let vector = RobotDriveVector.make(x: x, y: y) else {
            stopDrive(reason: "joystick-neutral-or-disconnected")
            return
        }
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
        guard central.state == .poweredOn else { return }
        central.stopScan()
        resetConnection(keepingState: true)
        state = .scanning
        central.scanForPeripherals(withServices: [Self.serviceUUID],
                                   options: [CBCentralManagerScanOptionAllowDuplicatesKey: false])
    }

    private func resetConnection(keepingState: Bool = false) {
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
            if line.hasPrefix("$SPOTDRIVE started ") {
                // Informational only: never arm/cancel the stop timeout here.
                continue
            }
            if line.hasPrefix("$SPOTDRIVE stopped ") {
                driveHeartbeat?.invalidate()
                driveHeartbeat = nil
                driveVector = nil
                if driveSessionActive { drivePhase = .draining }
                if driveSessionActive { armDriveCompletionTimeout() }
                driveStatus = line.contains("watchdog") ?
                    "통신 지연으로 자동 정지" : "중립"
                continue
            }
            if drivePhase == .controlling,
               line.hasPrefix("ERROR:") || line == "unknown command; type help" {
                // A rejected drive has no started/stopped banner. Its error
                // and prompt must still terminate the local session.
                driveHeartbeat?.invalidate()
                driveHeartbeat = nil
                driveVector = nil
                drivePhase = .draining
                lastError = line
                driveStatus = "보행 명령 거부"
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
                revision: values["rev"] ?? "unknown")
            lastStateSync = Date()
            lastError = nil
        }
    }

    private func fail(_ message: String) {
        trace?.record("error", message)
        lastError = message
        appendConsole("[BLE] \(message)\n")
    }
}

extension RobotBluetoothManager: CBCentralManagerDelegate {
    func centralManagerDidUpdateState(_ central: CBCentralManager) {
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
        let advertisedName = advertisementData[CBAdvertisementDataLocalNameKey] as? String
        guard peripheral.name == Self.deviceName || advertisedName == Self.deviceName else { return }
        central.stopScan()
        self.peripheral = peripheral
        signalStrength = RSSI.intValue
        peripheral.delegate = self
        state = .connecting
        central.connect(peripheral)
    }

    func centralManager(_ central: CBCentralManager, didConnect peripheral: CBPeripheral) {
        trace?.record("connected", peripheral.identifier.uuidString)
        state = .discoveringServices
        appendConsole("[BLE] SpotOMG-Bridge connected\n")
        peripheral.discoverServices([Self.serviceUUID])
        peripheral.readRSSI()
    }

    func centralManager(_ central: CBCentralManager,
                        didFailToConnect peripheral: CBPeripheral, error: Error?) {
        fail(error?.localizedDescription ?? "연결 실패")
        resetConnection()
        if reconnectRequested { startScanning() }
    }

    func centralManager(_ central: CBCentralManager,
                        didDisconnectPeripheral peripheral: CBPeripheral,
                        error: Error?) {
        trace?.record("disconnected", error.map { String(describing: $0) } ?? "no-error")
        if let error { fail("연결 끊김: \(error.localizedDescription)") }
        resetConnection()
        if reconnectRequested { startScanning() }
    }
}

extension RobotBluetoothManager: CBPeripheralDelegate {
    func peripheral(_ peripheral: CBPeripheral, didDiscoverServices error: Error?) {
        if let error { fail("서비스 검색 실패: \(error.localizedDescription)"); return }
        guard let service = peripheral.services?.first(where: { $0.uuid == Self.serviceUUID }) else {
            fail("SpotOMG BLE 서비스를 찾지 못했습니다")
            return
        }
        peripheral.discoverCharacteristics([Self.receiveUUID, Self.transmitUUID], for: service)
    }

    func peripheral(_ peripheral: CBPeripheral,
                    didDiscoverCharacteristicsFor service: CBService, error: Error?) {
        if let error { fail("특성 검색 실패: \(error.localizedDescription)"); return }
        for characteristic in service.characteristics ?? [] {
            if characteristic.uuid == Self.receiveUUID { receiveCharacteristic = characteristic }
            if characteristic.uuid == Self.transmitUUID { transmitCharacteristic = characteristic }
        }
        guard receiveCharacteristic != nil, let transmitCharacteristic else {
            fail("필수 BLE 특성을 찾지 못했습니다")
            return
        }
        peripheral.setNotifyValue(true, for: transmitCharacteristic)
    }

    func peripheral(_ peripheral: CBPeripheral,
                    didUpdateNotificationStateFor characteristic: CBCharacteristic,
                    error: Error?) {
        if let error { fail("알림 활성화 실패: \(error.localizedDescription)"); return }
        if characteristic.uuid == Self.transmitUUID, characteristic.isNotifying {
            state = .ready
            lastError = nil
            synchronizeClock()
            scheduleStateRefresh(after: 0.5)
        }
    }

    func peripheral(_ peripheral: CBPeripheral,
                    didUpdateValueFor characteristic: CBCharacteristic, error: Error?) {
        if let error { fail("수신 실패: \(error.localizedDescription)"); return }
        guard characteristic.uuid == Self.transmitUUID,
              let data = characteristic.value else { return }
        let text = String(decoding: data, as: UTF8.self)
        receiveConsoleText(text)
    }

    func peripheral(_ peripheral: CBPeripheral, didReadRSSI RSSI: NSNumber, error: Error?) {
        if error == nil { signalStrength = RSSI.intValue }
    }

    func peripheral(_ peripheral: CBPeripheral,
                    didWriteValueFor characteristic: CBCharacteristic, error: Error?) {
        guard peripheral == self.peripheral,
              characteristic.uuid == Self.receiveUUID else { return }
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
