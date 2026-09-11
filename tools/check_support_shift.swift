import Foundation

@main struct CheckSupportShift {
    static func main() {
        var sent = [String]()
        let previousTarget = UserDefaults.standard.object(forKey: "robotTarget")
        defer { UserDefaults.standard.set(previousTarget, forKey: "robotTarget") }
        UserDefaults.standard.set("robot", forKey: "robotTarget")
        let hardware = RobotBluetoothManager(commandWriter: {
            sent.append(String(decoding: $0, as: UTF8.self))
        })
        hardware.send(.simulatorProfile(.cushion_support_shift))
        hardware.send(.simulatorProfile(.cushion_support_shift_v2))
        hardware.send(.simulatorProfile(.cushion_v2_push))
        hardware.send(.simulatorProfile(.cushion_diagonal_sync_wide80))
        precondition(sent.isEmpty, "Experimental policy must not reach real firmware")
        UserDefaults.standard.set("simulator", forKey: "robotTarget")
        let manager = RobotBluetoothManager(commandWriter: {
            sent.append(String(decoding: $0, as: UTF8.self))
        })
        manager.receiveConsoleText("$SPOTSTATE pose=stand torque=on safety=ok caps=simprofiles\r\n")
        manager.send(.simulatorProfile(.cushion_support_shift))
        precondition(sent.last == "simprofile cushion_support_shift\n")
        manager.receiveConsoleText("OK profile=cushion_support_shift\r\n# ")
        manager.send(.simulatorProfile(.cushion_support_shift_v2))
        precondition(sent.last == "simprofile cushion_support_shift_v2\n")
        manager.receiveConsoleText("OK profile=cushion_support_shift_v2\r\n# ")
        manager.send(.simulatorProfile(.cushion_v2_push))
        precondition(sent.last == "simprofile cushion_v2_push\n")
        manager.receiveConsoleText("OK profile=cushion_v2_push\r\n# ")
        manager.send(.simulatorProfile(.cushion_diagonal_sync_wide80))
        precondition(sent.last == "simprofile cushion_diagonal_sync_wide80\n")
        manager.receiveConsoleText("OK profile=cushion_diagonal_sync_wide80\r\n# ")
        precondition(SimulatorGaitProfile.cushion_diagonal_sync_wide80.titleWithSpeed == "대각선 · 넓은 보폭 80mm (검증실패)")
        manager.send(.simulatorProfile(.cushion_forward))
        precondition(sent.last == "simprofile cushion_forward\n")
        manager.receiveConsoleText("OK profile=cushion_forward\r\n# ")
        manager.updateDrive(x: 0, y: 1)
        manager.stopDrive(reason: "support-shift-check")
        precondition(sent.last?.hasPrefix("@S ") == true)
        print("Experimental selection, policy switch, Stop and hardware exclusion passed")
    }
}
