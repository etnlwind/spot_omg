import AppKit
import CoreBluetooth
import Network

// Dedicated simulator GATT service: never advertise the real robot's UUID.
let serviceID = CBUUID(string: "6e400101-b5a3-f393-e0a9-e50e24dcca9e")
let receiveID = CBUUID(string: "6e400102-b5a3-f393-e0a9-e50e24dcca9e")
let transmitID = CBUUID(string: "6e400103-b5a3-f393-e0a9-e50e24dcca9e")

final class Bridge: NSObject, CBPeripheralManagerDelegate, NSApplicationDelegate {
    private var manager: CBPeripheralManager!
    private let rx = CBMutableCharacteristic(type: receiveID, properties: [.write], value: nil, permissions: [.writeable])
    private let tx = CBMutableCharacteristic(type: transmitID, properties: [.notify], value: nil, permissions: [])
    private var owner: CBCentral?
    private var connection: NWConnection?
    private var ready = false
    private var identity = Data()
    private var upstream = [Data]()
    private var upstreamBytes = 0
    private var sending = false
    private var downstream = Data()
    private var timeout: DispatchWorkItem?
    private var window: NSWindow!
    private let status = NSTextField(wrappingLabelWithString: "Bluetooth 초기화 중")
    private let port: NWEndpoint.Port

    init(port: UInt16) {
        self.port = NWEndpoint.Port(rawValue: port)!
        super.init()
    }

    func applicationDidFinishLaunching(_ notification: Notification) {
        window = NSWindow(contentRect: NSRect(x: 120, y: 120, width: 440, height: 150),
                          styleMask: [.titled, .closable, .miniaturizable], backing: .buffered, defer: false)
        window.title = "SpotOMG-Sim · MuJoCo Bluetooth"
        window.isReleasedWhenClosed = false
        status.frame = NSRect(x: 20, y: 20, width: 400, height: 110)
        status.font = .systemFont(ofSize: 15)
        window.contentView?.addSubview(status)
        window.makeKeyAndOrderFront(nil)
        NSApp.activate(ignoringOtherApps: true)
        manager = CBPeripheralManager(delegate: self, queue: .main)
    }

    func applicationShouldTerminateAfterLastWindowClosed(_ sender: NSApplication) -> Bool { true }
    func applicationWillTerminate(_ notification: Notification) {
        closeLink(); manager?.stopAdvertising()
    }

    private func report(_ text: String) {
        status.stringValue = "SpotOMG-Sim\n\(text)\nMuJoCo: localhost:\(port.rawValue) · 추정 물성"
        print(text); fflush(stdout)
    }

    func peripheralManagerDidUpdateState(_ peripheral: CBPeripheralManager) {
        closeLink()
        switch peripheral.state {
        case .poweredOn:
            peripheral.removeAllServices()
            let service = CBMutableService(type: serviceID, primary: true)
            service.characteristics = [rx, tx]
            peripheral.add(service)
        case .unauthorized: report("Bluetooth 권한이 필요합니다. 시스템 설정에서 허용해 주십시오.")
        case .poweredOff: report("Mac Bluetooth가 꺼져 있습니다.")
        case .unsupported: report("이 Mac은 BLE 주변 장치를 지원하지 않습니다.")
        default: report("Bluetooth 준비 중")
        }
    }

    func peripheralManager(_ peripheral: CBPeripheralManager, didAdd service: CBService, error: Error?) {
        if let error { report("BLE 서비스 등록 실패: \(error.localizedDescription)"); return }
        peripheral.startAdvertising([CBAdvertisementDataLocalNameKey: "SpotOMG-Sim",
                                     CBAdvertisementDataServiceUUIDsKey: [serviceID]])
    }

    func peripheralManagerDidStartAdvertising(_ peripheral: CBPeripheralManager, error: Error?) {
        report(error.map { "BLE 광고 실패: \($0.localizedDescription)" } ?? "BLE 검색 가능 · 앱에서 가상 로봇 · BLE를 선택하십시오.")
    }

    func peripheralManager(_ peripheral: CBPeripheralManager, central: CBCentral, didSubscribeTo characteristic: CBCharacteristic) {
        guard characteristic.uuid == transmitID else { return }
        if let owner {
            if owner.identifier != central.identifier {
                _ = peripheral.updateValue(Data("$SIMLINK disconnected busy\r\n".utf8), for: tx, onSubscribedCentrals: [central])
            }
            return
        }
        owner = central
        openLink()
    }

    func peripheralManager(_ peripheral: CBPeripheralManager, central: CBCentral, didUnsubscribeFrom characteristic: CBCharacteristic) {
        guard owner?.identifier == central.identifier else { return }
        closeLink()
        report("BLE 연결 해제 · MuJoCo 정지 요청됨")
    }

    private func closeLink() {
        timeout?.cancel(); timeout = nil
        let old = connection; connection = nil; old?.cancel()
        ready = false; identity.removeAll(); upstream.removeAll(); upstreamBytes = 0
        downstream.removeAll(); sending = false; owner = nil
    }

    private func failLink(_ message: String) {
        // Closing TCP invokes the virtual robot's disconnect stop. Notify the
        // BLE client so its UI cannot remain ready after losing the plant.
        let recipient = owner
        closeLink()
        if let recipient {
            owner = recipient
            downstream = Data("$SIMLINK disconnected \(message)\r\n".utf8)
            flushNotifications()
        }
        report("MuJoCo 연결 종료: \(message) · 앱에서 다시 연결하십시오.")
    }

    private func openLink() {
        let link = NWConnection(host: "127.0.0.1", port: port, using: .tcp)
        connection = link
        report("BLE 연결됨 · MuJoCo 식별 중")
        let work = DispatchWorkItem { [weak self, weak link] in
            guard let self, let link, self.connection === link else { return }
            self.failLink("identity-timeout")
        }
        timeout = work
        DispatchQueue.main.asyncAfter(deadline: .now() + 4, execute: work)
        link.stateUpdateHandler = { [weak self, weak link] state in
            guard let self, let link, self.connection === link else { return }
            switch state {
            case .ready:
                link.send(content: Data("identity\n".utf8), completion: .contentProcessed { [weak self, weak link] error in
                    guard let self, let link, self.connection === link else { return }
                    if error != nil { self.failLink("identity-write") }
                })
                self.receive(link)
            case .failed: self.failLink("tcp-failed")
            default: break
            }
        }
        link.start(queue: .main)
    }

    private func receive(_ link: NWConnection) {
        link.receive(minimumIncompleteLength: 1, maximumLength: 8192) { [weak self, weak link] data, _, ended, error in
            guard let self, let link, self.connection === link else { return }
            if let data, !data.isEmpty {
                if self.ready {
                    self.downstream.append(data)
                    if self.downstream.count > 65536 { self.failLink("notification-overflow"); return }
                    self.flushNotifications()
                } else {
                    self.identity.append(data)
                    if self.identity.count > 8192 { self.failLink("identity-overflow"); return }
                    if self.identity.suffix(2) == Data("# ".utf8) {
                        let lines = String(decoding: self.identity, as: UTF8.self).components(separatedBy: .newlines)
                        let verified = lines.contains { line in
                            let fields = Set(line.split(separator: " ").map(String.init))
                            return line.hasPrefix("$SPOTBACKEND ") && fields.contains("backend=sim") && fields.contains("protocol=1")
                        }
                        guard verified else { self.failLink("not-a-simulator"); return }
                        self.timeout?.cancel(); self.timeout = nil
                        self.ready = true; self.identity.removeAll()
                        self.report("앱 ↔ BLE ↔ MuJoCo 연결됨")
                        self.flushWrites()
                    }
                }
            }
            if ended || error != nil { self.failLink("tcp-closed") }
            else { self.receive(link) }
        }
    }

    func peripheralManager(_ peripheral: CBPeripheralManager, didReceiveWrite requests: [CBATTRequest]) {
        guard let first = requests.first else { return }
        guard requests.allSatisfy({ $0.central.identifier == owner?.identifier && $0.characteristic.uuid == receiveID && $0.offset == 0 }), connection != nil else {
            peripheral.respond(to: first, withResult: .writeNotPermitted); return
        }
        let payloads = requests.compactMap(\.value)
        let size = payloads.reduce(0) { $0 + $1.count }
        guard payloads.count == requests.count, upstreamBytes + size <= 16384 else {
            peripheral.respond(to: first, withResult: .insufficientResources)
            failLink("write-overflow"); return
        }
        upstream.append(contentsOf: payloads); upstreamBytes += size
        peripheral.respond(to: first, withResult: .success)
        flushWrites()
    }

    private func flushWrites() {
        guard ready, !sending, !upstream.isEmpty, let link = connection else { return }
        sending = true
        let data = upstream.removeFirst()
        link.send(content: data, completion: .contentProcessed { [weak self, weak link] error in
            guard let self, let link, self.connection === link else { return }
            self.sending = false; self.upstreamBytes -= data.count
            if error != nil { self.failLink("tcp-write") }
            else { self.flushWrites() }
        })
    }

    private func flushNotifications() {
        guard let owner else { return }
        while !downstream.isEmpty {
            let chunk = Data(downstream.prefix(owner.maximumUpdateValueLength))
            guard manager.updateValue(chunk, for: tx, onSubscribedCentrals: [owner]) else { return }
            downstream.removeFirst(chunk.count)
        }
    }

    func peripheralManagerIsReady(toUpdateSubscribers peripheral: CBPeripheralManager) { flushNotifications() }
}

var port: UInt16 = 8765
if let index = CommandLine.arguments.firstIndex(of: "--port"), index + 1 < CommandLine.arguments.count {
    guard let value = UInt16(CommandLine.arguments[index + 1]), value > 0 else { fatalError("Invalid TCP port") }
    port = value
}
let app = NSApplication.shared
let bridge = Bridge(port: port)
app.setActivationPolicy(.regular)
app.delegate = bridge
app.run()
