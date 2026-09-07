import SwiftUI

@main
struct SpotOMGControllerApp: App {
    @StateObject private var bluetooth = RobotBluetoothManager()
    @Environment(\.scenePhase) private var scenePhase

    var body: some Scene {
        WindowGroup {
            ContentView()
                .environmentObject(bluetooth)
        }
        .onChange(of: scenePhase) { _, phase in
            if phase != .active {
                bluetooth.requestSafeStand()
            }
        }
    }
}
