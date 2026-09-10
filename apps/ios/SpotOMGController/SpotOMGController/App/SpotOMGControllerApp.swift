import SwiftUI

@main
struct SpotOMGControllerApp: App {
    @StateObject private var bluetooth = RobotBluetoothManager()
    @Environment(\.scenePhase) private var scenePhase

    var body: some Scene {
        WindowGroup {
            ContentView()
                .environmentObject(bluetooth)
                .onOpenURL { url in
                    if url.scheme == "spotomg", url.host == "remote-control" { bluetooth.pollRemoteControl() }
                }
        }
        .onChange(of: scenePhase) { _, phase in
            if phase != .active {
                bluetooth.requestSafeStand()
            }
        }
    }
}
