import SwiftUI

struct ContentView: View {
    @EnvironmentObject private var bluetooth: RobotBluetoothManager

    var body: some View {
        NavigationStack { ControlView() }
            .toolbar(.hidden, for: .navigationBar)
    }
}
