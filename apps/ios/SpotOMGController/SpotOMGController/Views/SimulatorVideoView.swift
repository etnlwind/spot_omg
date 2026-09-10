import SwiftUI
import UIKit

@MainActor
final class SimulatorVideoClient: NSObject, ObservableObject, @preconcurrency NetServiceBrowserDelegate, @preconcurrency NetServiceDelegate {
    // Bonjour is started on the main run loop; delegate callbacks use that run loop.
    @Published var image: UIImage?
    @Published var status = "Mac 영상 서버 검색 중"
    @Published var simulationTime = ""
    private let browser = NetServiceBrowser()
    private var services: [NetService] = []
    private var selectedService: NetService?
    private var polling: Task<Void, Never>?
    private var running = false
    private var session: URLSession = {
        let configuration = URLSessionConfiguration.ephemeral
        configuration.allowsCellularAccess = true
        configuration.timeoutIntervalForRequest = 2
        configuration.timeoutIntervalForResource = 3
        configuration.requestCachePolicy = .reloadIgnoringLocalCacheData
        return URLSession(configuration: configuration)
    }()

    func start(host: String) {
        stop()
        running = true
        let host = host.trimmingCharacters(in: .whitespacesAndNewlines)
        if !host.isEmpty {
            connect(host: host, port: 8766)
        } else {
            status = "같은 Wi‑Fi의 Mac 검색 중"
            browser.delegate = self
            browser.searchForServices(ofType: "_spotomg-video._tcp.", inDomain: "local.")
        }
    }

    func stop() {
        running = false
        polling?.cancel(); polling = nil
        browser.stop()
        services.forEach { $0.stop() }
        services.removeAll(); selectedService = nil
        image = nil; simulationTime = ""
    }

    func netServiceBrowser(_ browser: NetServiceBrowser, didFind service: NetService, moreComing: Bool) {
        guard running else { return }
        services.append(service)
        service.delegate = self
        service.resolve(withTimeout: 5)
    }

    func netServiceDidResolveAddress(_ sender: NetService) {
        guard running, selectedService == nil, let host = sender.hostName else { return }
        selectedService = sender
        connect(host: host, port: sender.port)
    }

    func netServiceBrowser(_ browser: NetServiceBrowser, didRemove service: NetService, moreComing: Bool) {
        services.removeAll { $0 == service }
        if selectedService == service {
            polling?.cancel(); polling = nil; image = nil; selectedService = nil
            status = "Mac 연결이 끊겼습니다 · 다시 검색 중"
            services.forEach { $0.resolve(withTimeout: 5) }
        }
    }

    func netServiceBrowser(_ browser: NetServiceBrowser, didNotSearch errorDict: [String: NSNumber]) {
        status = "로컬 네트워크 권한과 Wi‑Fi를 확인해 주세요"
    }

    private func connect(host: String, port: Int) {
        var components = URLComponents()
        components.scheme = "http"; components.host = host
        components.port = port; components.path = "/frame.jpg"
        guard let url = components.url else { status = "Mac 주소를 확인해 주세요"; return }
        polling?.cancel()
        status = "MuJoCo 영상 연결 중"
        polling = Task { [weak self] in
            guard let self else { return }
            var sequence = "-1"
            var received = 0
            var intervalStart = Date()
            var fps = 0.0
            while !Task.isCancelled {
                do {
                    var requestURL = URLComponents(url: url, resolvingAgainstBaseURL: false)!
                    requestURL.queryItems = [URLQueryItem(name: "after", value: sequence)]
                    let (data, response) = try await self.session.data(from: requestURL.url!)
                    try Task.checkCancellation()
                    if (response as? HTTPURLResponse)?.statusCode == 204 {
                        self.image = nil; self.simulationTime = "영상 대기"
                        continue
                    }
                    guard let response = response as? HTTPURLResponse,
                          response.statusCode == 200,
                          response.value(forHTTPHeaderField: "X-SpotOMG-Video") == "1",
                          data.count < 2_000_000,
                          let frame = UIImage(data: data) else {
                        self.image = nil
                        self.status = "새 시뮬레이터 화면을 기다리는 중"
                        try await Task.sleep(nanoseconds: 300_000_000)
                        continue
                    }
                    let nextSequence = response.value(forHTTPHeaderField: "X-Frame-Sequence") ?? "-1"
                    if nextSequence == sequence {
                        try await Task.sleep(nanoseconds: 20_000_000)
                        continue
                    }
                    sequence = nextSequence
                    received += 1
                    let elapsed = Date().timeIntervalSince(intervalStart)
                    if elapsed >= 1 {
                        fps = Double(received) / elapsed
                        received = 0; intervalStart = Date()
                    }
                    self.image = frame
                    self.status = "MuJoCo · 가상 화면"
                    let time = Double(response.value(forHTTPHeaderField: "X-Simulation-Time") ?? "") ?? 0
                    self.simulationTime = String(format: "%.0f fps · %.1f s", fps, time)
                } catch {
                    if Task.isCancelled { break }
                    self.image = nil; self.simulationTime = ""
                    let failure = error as NSError
                    self.status = "영상 오류 \(failure.code): \(failure.localizedDescription)"
                    print("[MuJoCo video] \(url.host ?? host):\(port) \(failure.domain) \(failure.code) \(failure.localizedDescription)")
                    try? await Task.sleep(nanoseconds: 500_000_000)
                }
            }
        }
    }
}

struct SimulatorVideoView: View {
    let active: Bool
    let controlHost: String?
    @StateObject private var client = SimulatorVideoClient()
    @AppStorage("simulatorVideoHost") private var host = ""
    @State private var showAddress = false
    @Environment(\.scenePhase) private var scenePhase

    var body: some View {
        VStack(spacing: 0) {
            HStack {
                Text("MUJOCO · 2/2 ↔").font(.system(size: 11, weight: .bold, design: .monospaced))
                Spacer()
                Text(client.simulationTime).font(.caption2.monospacedDigit())
                Button { client.start(host: effectiveHost) } label: { Image(systemName: "arrow.clockwise") }
                    .accessibilityLabel("시뮬레이터 영상 다시 연결")
                Button { showAddress.toggle() } label: { Image(systemName: "network") }
                    .accessibilityLabel("영상 서버 주소 설정")
            }
            .foregroundStyle(.green).padding(.horizontal, 12).frame(height: 30)
            .background(Color(white: 0.06))
            if showAddress {
                HStack {
                    if let controlHost {
                        Text("TCP와 같은 주소: \(controlHost):8766")
                            .font(.caption).foregroundStyle(.white).lineLimit(1)
                    } else {
                    TextField("Mac IP / Tailscale 주소", text: $host)
                        .textInputAutocapitalization(.never).autocorrectionDisabled()
                        .font(.caption).foregroundStyle(.white)
                        .onSubmit { client.start(host: effectiveHost) }
                    }
                    Button("연결") { client.start(host: effectiveHost); showAddress = false }
                }.padding(.horizontal, 12).frame(height: 34)
            }
            GeometryReader { area in
                ZStack {
                    Color.black
                    if let image = client.image {
                        Image(uiImage: image).resizable().scaledToFit()
                            .frame(width: area.size.width, height: area.size.height)
                            .accessibilityLabel("Mac에서 수신한 MuJoCo 시뮬레이터 화면")
                    } else {
                        VStack(spacing: 8) {
                            Image(systemName: "video").font(.title2)
                            Text(client.status).font(.caption).multilineTextAlignment(.center)
                            Text("같은 Wi‑Fi 또는 양쪽 Tailscale 연결을 확인해 주세요")
                                .font(.caption2).foregroundStyle(.secondary)
                        }.foregroundStyle(.white).padding(12)
                    }
                }
            }
        }
        .frame(height: 230)
        .background(Color.black)
        .clipShape(RoundedRectangle(cornerRadius: 3))
        .padding(.horizontal, 10).padding(.vertical, 8)
        .background(Color(white: 0.12))
        .onAppear { updateStream() }
        .onChange(of: controlHost) { _, _ in updateStream() }
        .onChange(of: active) { _, _ in updateStream() }
        .onChange(of: scenePhase) { _, _ in updateStream() }
        .onDisappear { client.stop() }
    }

    private var effectiveHost: String {
        controlHost?.trimmingCharacters(in: .whitespacesAndNewlines) ?? host
    }

    private func updateStream() {
        if active && scenePhase == .active { client.start(host: effectiveHost) }
        else { client.stop() }
    }
}
