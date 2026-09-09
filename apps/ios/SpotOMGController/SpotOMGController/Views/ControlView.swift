import SwiftUI

struct ControlView: View {
    @EnvironmentObject private var bluetooth: RobotBluetoothManager
    @State private var showRelaxConfirmation = false
    @State private var consoleCommand = ""
    @FocusState private var consoleInputFocused: Bool

    private static let appVersion: String = {
        let info = Bundle.main.infoDictionary ?? [:]
        let version = info["CFBundleShortVersionString"] as? String ?? "—"
        let build = info["CFBundleVersion"] as? String ?? "—"
        let configuration = info["SpotBuildConfiguration"] as? String ?? "—"
        return "V\(version) (\(build)) - \(configuration)"
    }()

    private var robotVersion: String {
        guard bluetooth.state.isReady else { return "연결 후 확인" }
        let revision = bluetooth.runtimeState.revision
        return revision.isEmpty || revision == "unknown" ? (bluetooth.lastError == nil ? "확인 중" : "응답 없음") : revision
    }

    var body: some View {
        VStack(spacing: 0) {
            terminalPanel

            HStack {
                (Text("Spot OMG ").font(.headline) +
                 Text(Self.appVersion).font(.caption).fontWeight(.regular))
                    .lineLimit(1)
                    .minimumScaleFactor(0.7)
                Spacer()
                if bluetooth.state.isReady {
                    VStack(alignment: .trailing, spacing: 1) {
                        Text(bluetooth.runtimeState.poseTitle)
                            .font(.subheadline.bold())
                        Text("torque \(bluetooth.runtimeState.torque) · \(bluetooth.runtimeState.safety)")
                            .font(.caption2)
                            .foregroundStyle(bluetooth.runtimeState.safety == "fault" ? .red : .secondary)
                    }
                } else if let error = bluetooth.lastError {
                    Text(error)
                        .font(.caption)
                        .lineLimit(1)
                        .foregroundStyle(.red)
                }
            }
            .padding(.horizontal, 16)
            .frame(height: 48)
            .background(Color(uiColor: .systemBackground))

            List {
                Section {
                    Picker("제어 대상", selection: Binding(get: { bluetooth.target }, set: { bluetooth.selectTarget($0) })) {
                        ForEach(RobotConnectionTarget.allCases, id: \.self) { Text($0.title).tag($0) }
                    }
                    if bluetooth.target == .simulator {
                        HStack {
                            TextField("Mac IP 주소", text: $bluetooth.simulatorHost)
                                .textInputAutocapitalization(.never).autocorrectionDisabled()
                            TextField("포트", text: $bluetooth.simulatorPort).keyboardType(.numberPad).frame(width: 65)
                        }.disabled(bluetooth.state != .disconnected)
                    }
                    if bluetooth.target.isSimulator {
                        Text("추정 물성 · C 보행 정책 공유 · IMU는 모니터링만 적용")
                            .font(.caption).foregroundStyle(.orange)
                    }
                    HStack {
                        Circle()
                            .fill(bluetooth.state.isReady ? .green : .orange)
                            .frame(width: 10, height: 10)
                        Text(bluetooth.state.title)
                        Spacer()
                        TimelineView(.periodic(from: .now, by: 5)) { context in
                            let stale = bluetooth.lastVoltageRead.map {
                                context.date.timeIntervalSince($0) > 15
                            } ?? false
                            Text(bluetooth.supplyVoltageMillivolts.map {
                                String(format: bluetooth.target.isSimulator ? "가상 전압 ≈ %.1f V" : "배터리 ≈ %.1f V", Double($0) / 1000) + (stale ? " · 이전" : "")
                            } ?? "배터리 — V")
                                .font(.caption.monospacedDigit())
                                .foregroundStyle(stale ? .secondary : .primary)
                                .accessibilityHint("서보 전원선에서 측정한 전압. 현재 상태 동기화로 갱신")
                        }
                    }
                    LabeledContent(bluetooth.target.isSimulator ? "가상 제어기" : "로봇 펌웨어") {
                        Text(robotVersion)
                            .font(.subheadline.monospaced())
                            .multilineTextAlignment(.trailing)
                            .textSelection(.enabled)
                    }
                    Button(bluetooth.state.isReady ? "연결 해제" : "선택한 대상 연결") {
                        bluetooth.state.isReady ? bluetooth.disconnect() : bluetooth.connect()
                    }
                    if bluetooth.state.isReady {
                        Button("현재 상태 동기화") { bluetooth.synchronizeState() }
                    }
                }

                Section("조이스틱") {
                    HStack {
                        Spacer()
                        VirtualJoystick(enabled: bluetooth.state.isReady) { x, y in
                            bluetooth.updateDrive(x: x, y: y)
                        } onRelease: { reason in
                            bluetooth.stopDrive(reason: reason)
                        }
                        .frame(width: 190, height: 190)
                        Spacer()
                    }
                    Text(bluetooth.driveStatus)
                        .font(.system(.subheadline, design: .monospaced).bold())
                        .frame(maxWidth: .infinity, alignment: .center)
                    Text(bluetooth.target.isSimulator ? "가상 모터로 전진·후진·회전합니다. IMU 기울기는 관측하며 보정 제어는 아직 적용하지 않습니다. 손을 떼면 정지를 요청합니다." : "위·아래는 IMU 기울기 보정 전진·후진, 좌우는 회전입니다. 직진 근처의 작은 좌우 입력은 무시합니다. 중심에서 멀수록 빨라지고 손을 떼면 스틱이 중앙으로 복귀하며 즉시 정지를 요청합니다.")
                        .font(.caption)
                        .foregroundStyle(.secondary)
                }

            Section("안전 자세") {
                postureButton("Landing", pose: "landing", command: .landing)
                postureButton("Stand", pose: "stand", command: .stand)
                postureButton("Stand11", pose: "stand11", command: .stand11)
                Button("Hold") { bluetooth.send(.hold) }
                Button("Recover") { bluetooth.send(.recover) }
                Button("Relax", role: .destructive) { showRelaxConfirmation = true }
                Text("Stand11은 다리를 곧게 펴는 캘리브레이션 확인 자세입니다. 몸체를 지지한 상태에서 사용하십시오.")
                    .font(.caption)
                    .foregroundStyle(.secondary)
            }
            .disabled(!bluetooth.state.isReady)

            Section("보행") {
                Button("개선 전진 · 3회 · 844 ms") {
                    bluetooth.send(.trot5(cycles: 3, periodMilliseconds: 844))
                }
                .disabled(!bluetooth.runtimeState.supportsTrot5)
                Text(bluetooth.runtimeState.supportsTrot5 ?
                     "3S 배터리 모델에서 개선한 전진 보행입니다. 준비 자세로 천천히 전환한 뒤 실행하며, IMU는 기울기 감시에 사용합니다." :
                     "개선 전진은 로봇 V13 업데이트 후 사용할 수 있습니다.")
                    .font(.caption)
                    .foregroundStyle(.secondary)
                Button("Trot4 · 1회 · 1800 ms") {
                    bluetooth.send(.trot4(cycles: 1, periodMilliseconds: 1800))
                }
                Button("Crab Left · 1회 · 5000 ms") {
                    bluetooth.send(.crabLeft(cycles: 1, periodMilliseconds: 5000))
                }
                Button("Crab Right · 1회 · 5000 ms") {
                    bluetooth.send(.crabRight(cycles: 1, periodMilliseconds: 5000))
                }
                Text("로봇을 바로 잡을 수 있는 상태에서만 실행하십시오. 앱이 백그라운드로 가면 Stand를 요청합니다.")
                    .font(.caption)
                    .foregroundStyle(.secondary)
            }
            .disabled(!bluetooth.state.isReady)

            Section("진단") {
                Button("Targets") { bluetooth.send(.targets) }
                Button("Servo Scan") { bluetooth.send(.scan) }
                Button("Gait Diagnostics") { bluetooth.send(.gaitDiagnostics) }
                Button("Balance Diagnostics") { bluetooth.send(.balanceDiagnostics) }
                Button("저장 로그 64개 가져오기") {
                    bluetooth.send(.storedLogs(count: 64))
                }
                Button("로봇 시간 다시 동기화") { bluetooth.synchronizeClock() }
            }
            .disabled(!bluetooth.state.isReady)
            }
            .scrollDismissesKeyboard(.interactively)
        }
        .toolbar {
            ToolbarItemGroup(placement: .keyboard) {
                Spacer()
                Button("완료") { consoleInputFocused = false }
            }
        }
        .confirmationDialog("모든 서보의 토크를 해제할까요?", isPresented: $showRelaxConfirmation,
                            titleVisibility: .visible) {
            Button("Relax", role: .destructive) { bluetooth.send(.relax) }
            Button("취소", role: .cancel) {}
        } message: {
            Text("몸체가 쓰러지지 않도록 먼저 로봇을 지지하십시오.")
        }
    }

    private var terminalPanel: some View {
        VStack(spacing: 0) {
            HStack {
                Circle()
                    .fill(bluetooth.state.isReady ? Color.green : Color.orange)
                    .frame(width: 8, height: 8)
                Text("SPOT OMG TERMINAL")
                    .font(.system(size: 11, weight: .bold, design: .monospaced))
                Spacer()
                Button("CLEAR") { bluetooth.clearConsole() }
                    .font(.system(size: 10, weight: .bold, design: .monospaced))
            }
            .foregroundStyle(terminalGreen)
            .padding(.horizontal, 12)
            .frame(height: 30)
            .background(Color(white: 0.06))

            ScrollViewReader { proxy in
                ScrollView {
                    Text(bluetooth.consoleText.isEmpty ?
                         "READY. WAITING FOR SPOTOMG-BRIDGE..." : bluetooth.consoleText)
                        .font(.system(size: 11, design: .monospaced))
                        .foregroundStyle(terminalGreen)
                        .frame(maxWidth: .infinity, alignment: .leading)
                        .textSelection(.enabled)
                        .padding(.horizontal, 12)
                        .padding(.vertical, 8)
                    Color.clear.frame(height: 1).id("consoleBottom")
                }
                .onChange(of: bluetooth.consoleText) { _, _ in
                    proxy.scrollTo("consoleBottom", anchor: .bottom)
                }
            }

            HStack(spacing: 8) {
                Text(">")
                    .font(.system(.body, design: .monospaced).bold())
                TextField("COMMAND", text: $consoleCommand,
                          prompt: Text("COMMAND").foregroundStyle(terminalGreen.opacity(0.45)))
                    .font(.system(size: 12, design: .monospaced))
                    .foregroundStyle(terminalGreen)
                    .tint(terminalGreen)
                    .autocorrectionDisabled()
                    .textInputAutocapitalization(.never)
                    .focused($consoleInputFocused)
                    .onSubmit(sendConsoleCommand)
                Button(action: sendConsoleCommand) {
                    Image(systemName: "return")
                        .font(.system(size: 12, weight: .bold))
                }
                .disabled(!canSendConsoleCommand)
            }
            .foregroundStyle(terminalGreen)
            .padding(.horizontal, 12)
            .frame(height: 38)
            .overlay(alignment: .top) {
                Rectangle().fill(terminalGreen.opacity(0.25)).frame(height: 1)
            }
        }
        .frame(height: 230)
        .background(Color.black.opacity(1.0))
        .overlay {
            RoundedRectangle(cornerRadius: 3)
                .stroke(terminalGreen.opacity(0.5), lineWidth: 1)
        }
        .padding(.horizontal, 10)
        .padding(.vertical, 8)
        .background(Color(white: 0.12).opacity(1.0))
    }

    private var terminalGreen: Color {
        Color(red: 0.25, green: 1.0, blue: 0.35)
    }

    private var canSendConsoleCommand: Bool {
        bluetooth.state.isReady &&
            !consoleCommand.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty
    }

    private func postureButton(_ title: String,
                               pose: String,
                               command: RobotCommand) -> some View {
        Button {
            bluetooth.send(command)
        } label: {
            HStack {
                Text(title)
                Spacer()
                if bluetooth.runtimeState.pose == pose {
                    Image(systemName: "checkmark.circle.fill")
                        .foregroundStyle(.green)
                }
            }
        }
    }

    private func sendConsoleCommand() {
        guard canSendConsoleCommand else { return }
        bluetooth.send(.raw(consoleCommand))
        consoleCommand = ""
    }
}
