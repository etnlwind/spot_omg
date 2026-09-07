import SwiftUI

struct ControlView: View {
    @EnvironmentObject private var bluetooth: RobotBluetoothManager
    @State private var showRelaxConfirmation = false
    @State private var consoleCommand = ""
    @FocusState private var consoleInputFocused: Bool

    var body: some View {
        VStack(spacing: 0) {
            terminalPanel

            HStack {
                Text("Spot OMG")
                    .font(.title2.bold())
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
            Section("연결") {
                HStack {
                    Circle()
                        .fill(bluetooth.state.isReady ? .green : .orange)
                        .frame(width: 10, height: 10)
                    Text(bluetooth.state.title)
                    Spacer()
                    if let rssi = bluetooth.signalStrength { Text("\(rssi) dBm").foregroundStyle(.secondary) }
                }
                Button(bluetooth.state.isReady ? "연결 해제" : "다시 검색") {
                    bluetooth.state.isReady ? bluetooth.disconnect() : bluetooth.connect()
                }
                if bluetooth.state.isReady {
                    Button("현재 상태 동기화") { bluetooth.synchronizeState() }
                }
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

            Section("검증된 단일 보행") {
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
