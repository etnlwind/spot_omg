import SwiftUI

struct ControlView: View {
    @EnvironmentObject private var bluetooth: RobotBluetoothManager
    @State private var showRelaxConfirmation = false
    @State private var consoleCommand = ""
    @State private var terminalPage = CommandLine.arguments.contains("--simulator-video") ? 1 : 0
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
            TabView(selection: $terminalPage) {
                terminalPanel.tag(0)
                SimulatorVideoView(active: terminalPage == 1, controlHost: bluetooth.target == .simulator ? bluetooth.simulatorHost : nil).tag(1)
            }
            .tabViewStyle(.page(indexDisplayMode: .never))
            .frame(height: 246)
            .onChange(of: terminalPage) { _, _ in consoleInputFocused = false }

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

            GeometryReader { geometry in
            List {
                Section {
                    controllerPage(height: max(220, geometry.size.height - 28))
                        .listRowInsets(EdgeInsets())
                        .listRowSeparator(.hidden)
                }

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
                        Text("추정 물성 · " + bluetooth.runtimeState.balanceTitle)
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

                if bluetooth.runtimeState.capabilities.contains("headinghold") {
                    Section("직진 보정") {
                        Toggle("IMU 직진 방향 유지", isOn: Binding(
                            get: { bluetooth.runtimeState.heading == "on" },
                            set: { bluetooth.send(.headingHold($0)) }))
                            .disabled(!bluetooth.state.isReady || bluetooth.motionControlsLocked)
                        Text("전진 시 시작 방향을 유지합니다. 회전 입력은 우선하며, 설정 변경 시 정지합니다.")
                            .font(.caption).foregroundStyle(.secondary)
                    }
                }

                if bluetooth.runtimeState.capabilities.contains("gaitprofiles") || (bluetooth.target.isSimulator && bluetooth.runtimeState.capabilities.contains("simprofiles")) {
                    Section("보행 제어") {
                        if bluetooth.runtimeState.capabilities.contains("balancecontrol") || bluetooth.runtimeState.capabilities.contains("simbalance") {
                            Toggle("BNO055 수평 보정", isOn: Binding(
                                get: { !["off", "unknown"].contains(bluetooth.runtimeState.balance) },
                                set: { bluetooth.send(.simulatorBalance($0)) }))
                                .disabled(!bluetooth.state.isReady || bluetooth.motionControlsLocked)
                            Text("지연된 IMU 측정으로 다리를 보정합니다. 설정 변경 시 먼저 정지합니다.")
                                .font(.caption).foregroundStyle(.secondary)
                        }
                        Picker("보행 정책", selection: Binding(
                            get: { SimulatorGaitProfile(rawValue: bluetooth.runtimeState.simulationProfile) ?? .legacy },
                            set: { bluetooth.send(.simulatorProfile($0)) })) {
                            ForEach(SimulatorGaitProfile.allCases, id: \.self) { Text($0.titleWithSpeed).tag($0) }
                        }.disabled(!bluetooth.state.isReady || bluetooth.motionControlsLocked)
                        Text("괄호 속 속도는 시뮬레이션 최대 전진 기준입니다. 정책을 바꾸면 먼저 정지합니다. 빠른 트롯·하이 스텝은 후진을 60%로 제한합니다. 미끄러운 바닥에서는 방향이 틀어질 수 있습니다.")
                            .font(.caption).foregroundStyle(.secondary)
                    }
                }

            Section("조이스틱 사용법") {
                Text("위·아래는 전진·후진, 대각선은 이동과 회전, 좌우는 제자리 회전입니다. 중심에서 멀수록 빨라지고 손을 떼면 정지합니다.")
                    .font(.caption).foregroundStyle(.secondary)
            }
            Section("안전 자세") {
                postureButton("Landing", pose: "landing", command: .landing)
                if supportsStow {
                    postureButton("Stow · 수납", pose: "stow", command: .stow)
                    Text("설계 검토용 · 12초 동시 접기 / Landing으로 펼치기")
                        .font(.caption).foregroundStyle(.secondary)
                }
                postureButton("Stand", pose: "stand", command: .stand)
                postureButton("Stand11", pose: "stand11", command: .stand11)
                Button("Stop") { bluetooth.send(.hold) }.tint(.red)
                Button("Recover") { bluetooth.send(.recover) }.disabled(bluetooth.motionControlsLocked)
                Button("Relax", role: .destructive) { showRelaxConfirmation = true }.disabled(bluetooth.motionControlsLocked)
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
                     "3S 배터리 모델에서 개선한 전진 보행입니다. 준비 자세로 천천히 전환한 뒤 실행하며, IMU 보정은 상단 설정에 따릅니다." :
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
            .disabled(!bluetooth.state.isReady || bluetooth.motionControlsLocked)

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
            .listSectionSpacing(12)
            .contentMargins(.vertical, 0, for: .scrollContent)
            .scrollDismissesKeyboard(.interactively)
            .padding(.top, 16)
            .background(Color(uiColor: .systemGroupedBackground))
            }
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

    private func controllerPage(height: CGFloat) -> some View {
        VStack(spacing: 4) {
            compactControls
            HStack(spacing: 6) {
                Circle().fill(bluetooth.state.isReady ? Color.green : Color.orange)
                    .frame(width: 6, height: 6)
                Text(bluetooth.target.isSimulator ? "가상 로봇" : "실제 로봇")
                Spacer()
                TimelineView(.periodic(from: .now, by: 5)) { context in
                    let stale = bluetooth.lastVoltageRead.map { context.date.timeIntervalSince($0) > 15 } ?? false
                    Text(bluetooth.supplyVoltageMillivolts.map {
                        String(format: "≈ %.1f V", Double($0) / 1000) + (stale ? " · 이전" : "")
                    } ?? "— V")
                    .foregroundStyle(stale ? .secondary : .primary)
                }
            }
            .font(.caption2.monospacedDigit())
            Text((SimulatorGaitProfile(rawValue: bluetooth.runtimeState.simulationProfile) ?? .legacy).titleWithSpeed)
                .font(.caption).lineLimit(1).minimumScaleFactor(0.7)
            GeometryReader { area in
                let diameter = max(72, min(250, area.size.width, area.size.height))
                VirtualJoystick(enabled: bluetooth.state.isReady && !bluetooth.motionControlsLocked) { x, y in
                    bluetooth.updateDrive(x: x, y: y)
                } onRelease: { reason in
                    bluetooth.stopDrive(reason: reason)
                }
                .frame(width: diameter, height: diameter)
                .position(x: area.size.width / 2, y: area.size.height - diameter / 2)
                .accessibilityIdentifier("mainJoystick")
            }
            Text(bluetooth.driveStatus)
                .font(.system(.subheadline, design: .monospaced).bold())
                .lineLimit(1).minimumScaleFactor(0.7)
            Label("상세 설정", systemImage: "chevron.down")
                .font(.system(size: 9)).foregroundStyle(.secondary)
        }
        .padding(.horizontal, 10)
        .padding(.top, 22)
        .padding(.bottom, 6)
        .frame(height: height)
        .accessibilityIdentifier("controllerPage")
    }

    private var supportsProfiles: Bool {
        bluetooth.runtimeState.capabilities.contains("gaitprofiles") ||
        (bluetooth.target.isSimulator && bluetooth.runtimeState.capabilities.contains("simprofiles"))
    }

    private var supportsStow: Bool {
        bluetooth.target.isSimulator && bluetooth.runtimeState.capabilities.contains("simstow")
    }

    private var compactControls: some View {
        VStack(spacing: 2) {
            HStack(spacing: 0) {
                compactButton(bluetooth.state.isReady ? "해제" : "연결", icon: "link",
                              selected: bluetooth.state.isReady) {
                    bluetooth.state.isReady ? bluetooth.disconnect() : bluetooth.connect()
                }
                compactButton("동기화", icon: "arrow.clockwise", enabled: bluetooth.state.isReady) {
                    bluetooth.synchronizeState()
                }
                compactButton("직진", icon: "arrow.up", selected: bluetooth.runtimeState.heading == "on",
                              enabled: bluetooth.state.isReady && !bluetooth.motionControlsLocked && bluetooth.runtimeState.capabilities.contains("headinghold"),
                              toggle: true) {
                    bluetooth.send(.headingHold(bluetooth.runtimeState.heading != "on"))
                }
                compactButton("수평", icon: "gyroscope",
                              selected: !["off", "unknown"].contains(bluetooth.runtimeState.balance),
                              enabled: bluetooth.state.isReady && !bluetooth.motionControlsLocked &&
                                (bluetooth.runtimeState.capabilities.contains("balancecontrol") ||
                                 bluetooth.runtimeState.capabilities.contains("simbalance")), toggle: true) {
                    bluetooth.send(.simulatorBalance(["off", "unknown"].contains(bluetooth.runtimeState.balance)))
                }
                Menu {
                    Picker("보행 정책", selection: Binding(
                        get: { SimulatorGaitProfile(rawValue: bluetooth.runtimeState.simulationProfile) ?? .legacy },
                        set: { bluetooth.send(.simulatorProfile($0)) })) {
                        ForEach(SimulatorGaitProfile.allCases, id: \.self) { Text($0.titleWithSpeed).tag($0) }
                    }
                } label: {
                    compactLabel("정책", icon: "figure.walk", selected: false)
                }
                .disabled(!bluetooth.state.isReady || bluetooth.motionControlsLocked || !supportsProfiles)
                .accessibilityLabel("보행 정책 선택")
                .accessibilityValue((SimulatorGaitProfile(rawValue: bluetooth.runtimeState.simulationProfile) ?? .legacy).titleWithSpeed)
            }
            HStack(spacing: 0) {
                compactButton("Landing", icon: "arrow.down.to.line", selected: bluetooth.runtimeState.pose == "landing",
                              enabled: bluetooth.state.isReady && bluetooth.permitsCommand(.landing)) { bluetooth.send(.landing) }
                if supportsStow {
                    compactButton("Stow", icon: "shippingbox", selected: bluetooth.runtimeState.pose == "stow",
                                  enabled: bluetooth.state.isReady && bluetooth.permitsCommand(.stow)) { bluetooth.send(.stow) }
                }
                compactButton("Stand", icon: "figure.stand", selected: bluetooth.runtimeState.pose == "stand",
                              enabled: bluetooth.state.isReady && !bluetooth.motionControlsLocked) { bluetooth.send(.stand) }
                compactButton("Stand11", icon: "arrow.up.to.line", selected: bluetooth.runtimeState.pose == "stand11",
                              enabled: bluetooth.state.isReady && !bluetooth.motionControlsLocked) { bluetooth.send(.stand11) }
                compactButton("Recover", icon: "arrow.counterclockwise", enabled: bluetooth.state.isReady && !bluetooth.motionControlsLocked) { bluetooth.send(.recover) }
                compactButton("Relax", icon: "power", enabled: bluetooth.state.isReady && !bluetooth.motionControlsLocked, destructive: true) { showRelaxConfirmation = true }
                    .tint(.red)
                compactButton("Stop", icon: "stop.fill", enabled: bluetooth.state.isReady, destructive: true) {
                    bluetooth.send(.hold)
                }
                .accessibilityIdentifier("controllerStop")
            }
        }
        .buttonStyle(.plain)
        .accessibilityIdentifier("compactControls")
    }

    private func compactLabel(_ title: String, icon: String, selected: Bool, destructive: Bool = false) -> some View {
        VStack(spacing: 3) {
            Image(systemName: icon)
                .font(.system(size: 17, weight: .semibold))
            Text(title).font(.system(size: 9, weight: .medium)).lineLimit(1)
        }
        .frame(maxWidth: .infinity, minHeight: 44)
        .foregroundStyle(selected ? Color.green : (destructive ? Color.red : Color.accentColor))
        .background(selected ? Color.green.opacity(0.12) : Color.clear,
                    in: RoundedRectangle(cornerRadius: 8))
        .contentShape(Rectangle())
    }

    private func compactButton(_ title: String, icon: String, selected: Bool = false,
                               enabled: Bool = true, toggle: Bool = false, destructive: Bool = false,
                               action: @escaping () -> Void) -> some View {
        Button(role: destructive ? .destructive : nil, action: action) {
            compactLabel(title, icon: icon, selected: selected, destructive: destructive)
        }
            .disabled(!enabled)
            .opacity(enabled ? 1 : 0.35)
            .accessibilityLabel(toggle ? "IMU \(title) 보정" : title)
            .accessibilityValue(toggle ? (selected ? "켜짐" : "꺼짐") : (selected ? "선택됨" : ""))
            .accessibilityHint(toggle ? "변경하면 먼저 보행을 정지합니다" : "")
            .help(toggle ? "IMU \(title) 보정: \(selected ? "켜짐" : "꺼짐")" : title)
    }

    private var terminalPanel: some View {
        VStack(spacing: 0) {
            HStack {
                Circle()
                    .fill(bluetooth.state.isReady ? Color.green : Color.orange)
                    .frame(width: 8, height: 8)
                Text("TERMINAL · 1/2 ↔")
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
        .disabled(!bluetooth.state.isReady || !bluetooth.permitsCommand(command))
    }

    private func sendConsoleCommand() {
        guard canSendConsoleCommand else { return }
        bluetooth.send(.raw(consoleCommand))
        consoleCommand = ""
    }
}
