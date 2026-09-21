import SwiftUI
import AudioToolbox
import UIKit

struct ControlView: View {
    @EnvironmentObject private var bluetooth: RobotBluetoothManager
    @State private var probeDraft = RobotProbeConfig()
    @State private var showRelaxConfirmation = false
    @State private var showFootLiftSettings = false
    @State private var acknowledgedBatteryLevel = 0
    @State private var consoleCommand = ""
    @State private var terminalPage = CommandLine.arguments.contains("--simulator-video") ? 1 : 0
    @FocusState private var consoleInputFocused: Bool

    private static let appVersion: String = {
        let info = Bundle.main.infoDictionary ?? [:]
        let version = info["CFBundleShortVersionString"] as? String ?? "—"
        let build = info["CFBundleVersion"] as? String ?? "—"
        let configuration = info["SpotBuildConfiguration"] as? String ?? "—"
        let parts = version.split(separator: ".")
        let release = parts.count >= 2 ? "V\(parts[0])" + (parts[1] == "0" ? "" : "-R\(parts[1])") : "V\(version)"
        return "\(release) (\(build)) - \(configuration)"
    }()

    private var robotVersion: String {
        guard bluetooth.state.isReady else { return "연결 후 확인" }
        let revision = bluetooth.runtimeState.revision
        return revision.isEmpty || revision == "unknown" ? (bluetooth.lastError == nil ? "확인 중" : "응답 없음") : revision
    }


    var body: some View {
        VStack(spacing: 0) {
            if bluetooth.target == .robot && bluetooth.state.isReady && bluetooth.batteryWarning.level > 0 { batteryWarningBanner }
            if bluetooth.state.isReady, let warning = bluetooth.motionWarning {
                VStack(alignment: .leading, spacing: 5) {
                    Label(warning.title, systemImage: "pause.circle.fill").font(.headline)
                    Text(warning.message).font(.subheadline)
                    Text(warning.reason).font(.caption.monospaced()).textSelection(.enabled)
                }
                .frame(maxWidth: .infinity, alignment: .leading).padding(12)
                .foregroundStyle(.white).background(Color(red: 0.32, green: 0.20, blue: 0.05))
                .accessibilityIdentifier("motionPauseBanner")
            }
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
                                String(format: bluetooth.target.isSimulator ? "가상 전압 ≈ %.1f V" : "안정 전압 ≈ %.1f V", Double($0) / 1000) + (stale ? " · 이전" : "")
                            } ?? "배터리 — V")
                                .font(.caption.monospacedDigit())
                                .foregroundStyle(stale ? .secondary : .primary)
                                .accessibilityHint("정지 후 안정된 세 번의 서보 전압 측정값으로 갱신")
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

                if bluetooth.supportsIMURecovery {
                    Section("IMU 복구") {
                        Button(action: bluetooth.recoverIMU) {
                            Label(bluetooth.imuRecoveryPending ? "복구 중…" : "IMU 복구", systemImage: "arrow.clockwise")
                        }
                        .disabled(!bluetooth.canRecoverIMU)
                        Text(bluetooth.imuRecoveryMessage).font(.caption).foregroundStyle(.secondary)
                    }
                }

                Section("보행 방식") {
                    Picker("보행 방식", selection: $bluetooth.parameterWalking) {
                        Text("모델 보행").tag(false)
                        Text("직접 설정 보행").tag(true)
                    }
                    .pickerStyle(.segmented)
                    .disabled(!bluetooth.canSelectWalkingMode)
                }

                if bluetooth.runtimeState.capabilities.contains("headinghold") {
                    Section("직진 보정") {
                        Toggle("IMU 직진 방향 유지", isOn: Binding(
                            get: { bluetooth.runtimeState.heading == "on" },
                            set: { bluetooth.send(.headingHold($0)) }))
                            .disabled(!bluetooth.state.isReady || bluetooth.motionControlsLocked)
                        Text("전진 시 시작 방향을 유지합니다. 회전 입력은 우선하며, 설정 변경 시 정지합니다.")
                            .font(.caption).foregroundStyle(Color(white: 0.78))
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
                                .font(.caption).foregroundStyle(Color(white: 0.78))
                        }
                        if !bluetooth.parameterWalking {
                        ForEach(SimulatorGaitProfile.allCases.filter { !$0.simulatorOnly || bluetooth.target.isSimulator }, id: \.self) { profile in
                            let selected = bluetooth.runtimeState.simulationProfile == profile.rawValue
                            Button {
                                bluetooth.send(.simulatorProfile(profile))
                            } label: {
                                HStack(spacing: 12) {
                                    Image(systemName: selected ? "checkmark.circle.fill" : "circle")
                                        .foregroundStyle(selected ? Color.accentColor : .secondary)
                                    Text(profile.titleWithSpeed)
                                        .foregroundStyle(.primary)
                                        .multilineTextAlignment(.leading)
                                    Spacer(minLength: 0)
                                }
                                .frame(maxWidth: .infinity, alignment: .leading)
                                .contentShape(Rectangle())
                            }
                            .accessibilityAddTraits(selected ? .isSelected : [])
                            .disabled(!bluetooth.state.isReady || bluetooth.motionControlsLocked || !profile.isSupported(capabilities: bluetooth.runtimeState.capabilities))
                        }
                        Text("V1–V6 계열은 S 자세에서 출발합니다. 괄호 속 속도는 시뮬레이션 최대 전진 기준입니다. 모델을 바꾸면 먼저 정지합니다. 빠른 트롯·하이 스텝은 후진을 60%로 제한합니다. 미끄러운 바닥에서는 방향이 틀어질 수 있습니다.")
                            .font(.caption).foregroundStyle(Color(white: 0.78))
                        }
                    }
                }

            if bluetooth.parameterWalking {
            Section("직접 설정 보행 · V6.2.5") {
                Text("전진 전용 · 저장과 로봇 적용은 별도 · 설정 시간 또는 조이스틱 해제로 정지").font(.caption)
                Button("설정 저장") { bluetooth.saveProbe(probeDraft) }
                Button("저장값 불러오기") { probeDraft=bluetooth.savedProbe }
                Stepper("들림 목표: \(probeDraft.lift) mm", value:$probeDraft.lift,in:12...40)
                Stepper("전진 입력: \(probeDraft.linear) / 1000", value:$probeDraft.linear,in:1...1000)
                Stepper("실행 시간: \(probeDraft.duration) ms", value:$probeDraft.duration,in:500...30000,step:500)
                Picker("대상 다리",selection:$probeDraft.legs) {
                    Text("전체").tag("all");Text("왼쪽 뒤 RL").tag("rl");Text("오른쪽 뒤 RR").tag("rr")
                }
                Stepper("좌우 간격: \(probeDraft.width ?? 0) mm / 한쪽",value:Binding(get:{probeDraft.width ?? 0},set:{probeDraft.width=$0}),in:-40...20)
                    .disabled(!bluetooth.supportsProbeWidth)
                Toggle("출발 시 FR만 추가 오므림",isOn:$probeDraft.frExtra)
                    .disabled(!bluetooth.supportsProbeWidth)
                Text("해제: 좌우 동일. 체크: 안쪽 간격에서 첫걸음 FR J1 변화량 2배, 다음 걸음에 공통 간격으로 복귀.").font(.caption)
                Text("수직 0 · 안쪽 − · 바깥 +. −20mm는 양쪽 합계 40mm 좁힘. 기존 정상 기준은 약 −38mm입니다.")
                    .font(.caption).foregroundStyle(Color(white: 0.78))
                Button("V6.2.5 선택") { bluetooth.send(.simulatorProfile(.s_native_v6_2_5)) }
                    .disabled(!bluetooth.canConfigureProbe)
                if let reason = bluetooth.probeConfigurationBlockReason {
                    Text(reason).font(.caption).foregroundStyle(.orange)
                }
                Button("설정 적용 + 조회") { var value=probeDraft;value.width=bluetooth.supportsProbeWidth ? (probeDraft.width ?? 0) : nil;bluetooth.configureProbe(value) }
                    .disabled(!bluetooth.canConfigureProbe)
                Button("설정 조회") { bluetooth.configureProbe(nil) }
                    .disabled(!bluetooth.canConfigureProbe)
                Text(bluetooth.probeConfig.map { "적용값: " + $0.summary } ?? "설정 조회 필요")
                    .font(.caption).foregroundStyle(Color(white: 0.78))
                Button("시험 시작") { bluetooth.startProbe() }.disabled(!bluetooth.canStartProbe)
                Button("시험 정지",role:.destructive) { bluetooth.stopWalkingOrHold() }.disabled(!bluetooth.state.isReady)
                Text(bluetooth.supportsProbe ? "V6.2.5 선택 → Stand → 설정 적용 → 시험 시작. 직접 설정 보행의 조이스틱 전진에도 적용됩니다(전체 다리 선택). 몸체를 지지하고 시험하세요." : "실기 V77-T1-param 펌웨어가 필요합니다.")
                    .font(.caption).foregroundStyle(Color(white: 0.78))
            }

            }

            Section("조이스틱 사용법") {
                Text(bluetooth.parameterWalking ? "위로 밀면 직접 설정한 값으로 전진합니다. 설정 시간이 끝나거나 손을 떼면 정지합니다." : "위·아래는 전진·후진, 대각선은 이동과 회전, 좌우는 제자리 회전입니다. 중심에서 멀수록 빨라지고 손을 떼면 정지합니다.")
                    .font(.caption).foregroundStyle(Color(white: 0.78))
            }
            Section("안전 자세") {
                postureButton("Landing", pose: "landing", command: .landing)
                if supportsStow {
                    postureButton("Stow · 수납", pose: "stow", command: .stow)
                    Text("설계 검토용 · 12초 동시 접기 / Landing으로 펼치기")
                        .font(.caption).foregroundStyle(Color(white: 0.78))
                }
                postureButton("Stand", pose: "stand", command: .stand)
                postureButton("Stand11", pose: "stand11", command: .stand11)
                Button("Stop") { bluetooth.send(.hold) }.tint(.red)
                Button("Recover") { bluetooth.send(.recover) }.disabled(bluetooth.motionControlsLocked)
                Button("Relax", role: .destructive) { showRelaxConfirmation = true }.disabled(bluetooth.motionControlsLocked)
                Text("Stand11은 다리를 곧게 펴는 캘리브레이션 확인 자세입니다. 몸체를 지지한 상태에서 사용하십시오.")
                    .font(.caption)
                    .foregroundStyle(Color(white: 0.78))
            }
            .disabled(!bluetooth.state.isReady)

            if !bluetooth.parameterWalking {
            Section("보행") {
                Button("개선 전진 · 3회 · 844 ms") {
                    bluetooth.send(.trot5(cycles: 3, periodMilliseconds: 844))
                }
                .disabled(!bluetooth.runtimeState.supportsTrot5)
                Text(bluetooth.runtimeState.supportsTrot5 ?
                     "3S 배터리 모델에서 개선한 전진 보행입니다. 준비 자세로 천천히 전환한 뒤 실행하며, IMU 보정은 상단 설정에 따릅니다." :
                     "개선 전진은 로봇 V13 업데이트 후 사용할 수 있습니다.")
                    .font(.caption)
                    .foregroundStyle(Color(white: 0.78))
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
                    .foregroundStyle(Color(white: 0.78))
            }
            .disabled(!bluetooth.state.isReady || bluetooth.motionControlsLocked)

            }

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
        .onAppear { probeDraft = bluetooth.savedProbe }
        .toolbar {
            ToolbarItemGroup(placement: .keyboard) {
                Spacer()
                Button("완료") { consoleInputFocused = false }
            }
        }
        .onChange(of: bluetooth.batteryWarning.level) { oldLevel, newLevel in
            if newLevel == 0 { acknowledgedBatteryLevel = 0 }
            if bluetooth.target == .robot && bluetooth.state.isReady && newLevel > oldLevel {
                AudioServicesPlayAlertSound(1005)
                UINotificationFeedbackGenerator().notificationOccurred(.error)
                UIAccessibility.post(notification: .announcement, argument: bluetooth.batteryWarning.title)
            }
        }
        .sheet(isPresented: $showFootLiftSettings) {
            FootLiftSettingsView()
        }
        .confirmationDialog("모든 서보의 토크를 해제할까요?", isPresented: $showRelaxConfirmation,
                            titleVisibility: .visible) {
            Button("Relax", role: .destructive) { bluetooth.send(.relax) }
            Button("취소", role: .cancel) {}
        } message: {
            Text("몸체가 쓰러지지 않도록 먼저 로봇을 지지하십시오.")
        }
    }

    private var batteryWarningBanner: some View {
        let warning = bluetooth.batteryWarning
        return VStack(alignment: .leading, spacing: 8) {
            Label("실제 로봇 · " + warning.title,
                  systemImage: "battery.0percent")
                .font(.headline.bold())
            if acknowledgedBatteryLevel < warning.level {
                Text(warning.message).font(.subheadline)
                HStack {
                    Button("보행 정지") { bluetooth.stopDrive(reason: "배터리 충전 필요") }
                        .disabled(!bluetooth.state.isReady)
                    Spacer()
                    Button("확인") { acknowledgedBatteryLevel = warning.level }
                }
                .buttonStyle(.bordered).tint(.white)
            }
        }
        .foregroundStyle(.white)
        .padding(14)
        .frame(maxWidth: .infinity, alignment: .leading)
        .background(Color(red: 0.68, green: 0.06, blue: 0.08))
        .accessibilityElement(children: .contain)
        .accessibilityIdentifier("batteryWarningBanner")
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
            HStack {
                Button(bluetooth.joystickAutoReturn ? "자동 복귀 ON" : "자동 복귀 OFF") {
                    bluetooth.joystickAutoReturn.toggle()
                    if bluetooth.joystickAutoReturn { bluetooth.stopDrive(reason:"auto-return-enabled") }
                }
                Spacer()
                Button { showFootLiftSettings = true } label: {
                    Label("발 높이 설정", systemImage: "slider.horizontal.3")
                }.accessibilityIdentifier("footLiftShortcut")
            }.font(.caption).buttonStyle(.bordered)
            GeometryReader { area in
                let diameter = max(72, min(250, area.size.width, area.size.height))
                VirtualJoystick(enabled: bluetooth.joystickEnabled && (!bluetooth.probeRunning || bluetooth.parameterWalking), autoReturn:bluetooth.joystickAutoReturn, resetToken:bluetooth.joystickResetToken) { x, y in
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
                .font(.system(size: 12)).foregroundStyle(Color(white: 0.78))
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
        bluetooth.runtimeState.capabilities.contains("stow") || (bluetooth.target.isSimulator && bluetooth.runtimeState.capabilities.contains("simstow"))
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
                    bluetooth.stopWalkingOrHold()
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
            Text(title).font(.system(size: 12, weight: .medium)).lineLimit(1)
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
                    .font(.system(size: 13, weight: .bold, design: .monospaced))
                Spacer()
                Button("CLEAR") { bluetooth.clearConsole() }
                    .font(.system(size: 12, weight: .bold, design: .monospaced))
            }
            .foregroundStyle(terminalGreen)
            .padding(.horizontal, 12)
            .frame(height: 30)
            .background(Color(white: 0.06))

            ScrollViewReader { proxy in
                ScrollView {
                    Text(coloredTerminalText)
                        .font(.system(size: 13, design: .monospaced))
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
                          prompt: Text("COMMAND").foregroundStyle(terminalGreen.opacity(0.8)))
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

    private var coloredTerminalText: AttributedString {
        let output = bluetooth.consoleText.isEmpty
            ? "READY. WAITING FOR SPOTOMG-BRIDGE..." : bluetooth.consoleText
        var result = AttributedString()
        let lines = output.components(separatedBy: "\n")
        for (index, line) in lines.enumerated() {
            var part = AttributedString(line + (index < lines.count - 1 ? "\n" : ""))
            let trimmed = line.trimmingCharacters(in: .whitespaces).lowercased()
            // Numeric position error in SPOTSTATE is telemetry, not a fault.
            let error = !trimmed.hasPrefix(">") && (
                trimmed.hasPrefix("error") || trimmed.hasPrefix("[error]") ||
                trimmed.contains("오류") || trimmed.contains("실패") ||
                trimmed.contains("보행명령거부") ||
                (trimmed.hasPrefix("pose ") && ["no-progress", "obstruction-suspected",
                    "feedback-lost", "write-failed", "encoder-invalid", "low-voltage",
                    "servo-fault", "imu-unavailable", "unstable", "hold-failed"]
                    .contains(where: { trimmed.contains("reason=" + $0) })))
            part.foregroundColor = error ? Color.red : terminalGreen
            result.append(part)
        }
        return result
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


struct FootLiftSettingsView: View {
    @EnvironmentObject private var bluetooth: RobotBluetoothManager
    @Environment(\.dismiss) private var dismiss
    @State private var footLiftDraft = [0,0,0,0]
    @State private var footLiftBaseline: [Int]?
    private func loadFootLift() {
        footLiftBaseline = bluetooth.footLiftApplied
        footLiftDraft = footLiftBaseline ?? [0,0,0,0]
    }
    private func footLiftField(_ index: Int,_ name: String) -> some View {
        VStack {
            Text(name).font(.subheadline.weight(.medium))
            HStack(spacing: 4) {
                TextField("mm",value:$footLiftDraft[index],format:.number)
                    .keyboardType(.numberPad).textFieldStyle(.roundedBorder)
                    .font(.body.monospacedDigit()).disabled(bluetooth.footLiftPending)
                    .accessibilityLabel("\(name) 추가 발 높이, 밀리미터")
                    .onChange(of:footLiftDraft[index]) { _,value in footLiftDraft[index]=min(2147483647,max(0,value)) }
                Text("mm").font(.caption).fixedSize().foregroundStyle(Color(white: 0.78))
            }
            Stepper("\(name) 발 높이",value:$footLiftDraft[index],in:0...2147483647)
                .labelsHidden().disabled(bluetooth.footLiftPending)
        }
    }
    private var footLiftEditor: some View {
                    VStack(spacing: 12) {
                        Text("↑ 로봇 앞쪽").font(.caption)
                        HStack(spacing: 24) {
                            footLiftField(0,"FL · 앞 왼쪽")
                            footLiftField(1,"FR · 앞 오른쪽")
                        }
                        SpotRobotTopView()
                            .frame(height: 290)
                            .frame(maxWidth: .infinity)
                        HStack(spacing: 24) {
                            footLiftField(2,"RL · 뒤 왼쪽")
                            footLiftField(3,"RR · 뒤 오른쪽")
                        }
                        Text("0 = 기본 보행 · 스윙 중에만 추가 들림 · 로봇 기준 좌우").font(.caption).foregroundStyle(Color(white: 0.78))
                        if bluetooth.state.isReady, let applied=bluetooth.footLiftApplied {
                            Text(zip(["FL","FR","RL","RR"],applied).map { "\($0.0) \($0.1)mm" }.joined(separator:" / ")).font(.caption.monospacedDigit())
                        }
                        Text(bluetooth.supportsPersistentFootLift ? bluetooth.footLiftMessage : "로봇 영구 저장 지원 펌웨어(V90-R2)가 필요합니다.").font(.caption)
                        HStack {
                            Button("저장·반영") { bluetooth.configureFootLift(footLiftDraft) }.disabled(!bluetooth.canConfigureFootLift)
                            Button("로봇 값 불러오기") { loadFootLift(); bluetooth.refreshFootLift() }.disabled(bluetooth.footLiftPending)
                            Button("모두 0") { footLiftDraft=[0,0,0,0] }.disabled(bluetooth.footLiftPending)
                        }.buttonStyle(.bordered)
                        if let actual = bluetooth.footLiftApplied, actual != footLiftDraft, !bluetooth.footLiftPending {
                            Text("아직 반영되지 않은 입력값입니다. 저장·반영을 눌러 주세요.").foregroundStyle(.orange).font(.subheadline)
                        }
                        Text("STM32에 영구 저장됩니다. Windows·Mac·iPhone이 같은 로봇의 설정을 공유합니다.").font(.caption).foregroundStyle(Color(white: 0.78))
                    }.onAppear { loadFootLift(); bluetooth.refreshFootLift() }
                    .onChange(of: bluetooth.footLiftApplied) { _, actual in
                        if footLiftBaseline == nil || footLiftDraft == footLiftBaseline { loadFootLift() }
                    }
                    .onChange(of: bluetooth.footLiftPending) { old, pending in
                        if old && !pending && bluetooth.footLiftMessage == "로봇 저장·반영 완료" { loadFootLift() }
                    }
    }

    var body: some View {
        NavigationStack {
            Form { Section("발 추가 들림 · 모든 보행") { footLiftEditor } }
                .navigationTitle("발 높이 설정")
                .toolbar { ToolbarItem(placement: .confirmationAction) { Button("완료") { dismiss() } } }
        }
        .preferredColorScheme(.dark).tint(.mint).toggleStyle(.switch)
    }
}


/// Orthographic render of the existing MuJoCo CAD model in its standing pose.
private struct SpotRobotTopView: View {
    var body: some View {
        Image("RobotTopView")
            .resizable()
            .scaledToFit()
            .accessibilityLabel("MuJoCo 실제 CAD 로봇 윗모습. 위쪽이 전방이며 왼쪽 앞발 FL, 오른쪽 앞발 FR입니다.")
    }
}
