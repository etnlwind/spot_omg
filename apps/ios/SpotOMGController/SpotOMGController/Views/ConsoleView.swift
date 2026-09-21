import SwiftUI

struct ConsoleView: View {
    @EnvironmentObject private var bluetooth: RobotBluetoothManager
    @State private var command = ""

    var body: some View {
        VStack(spacing: 0) {
            ScrollViewReader { proxy in
                ScrollView {
                    Text(bluetooth.consoleText.isEmpty ? "명령 결과와 오류가 표시됩니다." : bluetooth.consoleText)
                        .font(.system(.caption, design: .monospaced))
                        .frame(maxWidth: .infinity, alignment: .leading)
                        .textSelection(.enabled)
                        .padding()
                    Color.clear.frame(height: 1).id("bottom")
                }
                .onChange(of: bluetooth.consoleText) { _, _ in
                    proxy.scrollTo("bottom", anchor: .bottom)
                }
            }
            Divider()
            HStack {
                TextField("STM32 명령", text: $command)
                    .textFieldStyle(.roundedBorder)
                    .autocorrectionDisabled()
                    .textInputAutocapitalization(.never)
                    .onSubmit(send)
                Button("전송", action: send)
                    .buttonStyle(.borderedProminent)
                    .disabled(!bluetooth.state.isReady || command.trimmingCharacters(in: .whitespaces).isEmpty)
            }
            .padding()
        }
        .navigationTitle("콘솔")
        .toolbar {
            Button("지우기") { bluetooth.clearConsole() }
            Button("진단 로그 준비") { bluetooth.exportDiagnostics() }
                .disabled(bluetooth.exportingDiagnostics)
            if let url = bluetooth.diagnosticExportURL {
                ShareLink("로그 저장·공유", item: url)
            }
        }
    }

    private func send() {
        bluetooth.send(.raw(command))
        command = ""
    }
}
