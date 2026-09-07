import SwiftUI

struct ConsoleView: View {
    @EnvironmentObject private var bluetooth: RobotBluetoothManager
    @State private var command = ""

    var body: some View {
        VStack(spacing: 0) {
            ScrollViewReader { proxy in
                ScrollView {
                    Text(bluetooth.consoleText.isEmpty ? "BLE console output" : bluetooth.consoleText)
                        .font(.system(.caption, design: .monospaced))
                        .frame(maxWidth: .infinity, alignment: .leading)
                        .textSelection(.enabled)
                        .padding()
                    Color.clear.frame(height: 1).id("bottom")
                }
                .onChange(of: bluetooth.consoleText) { _, _ in
                    withAnimation { proxy.scrollTo("bottom", anchor: .bottom) }
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
        .toolbar { Button("지우기") { bluetooth.clearConsole() } }
    }

    private func send() {
        bluetooth.send(.raw(command))
        command = ""
    }
}
