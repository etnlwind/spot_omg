import Foundation
@main struct CheckSpeedLabels {
    static func main() {
        precondition(SimulatorGaitProfile.speedSuffix(0.144) == "(0.144m/s)")
        precondition(SimulatorGaitProfile.speedSuffix(0) == "(0.000m/s)")
        for invalid: Double? in [nil, .nan, .infinity, -1] {
            precondition(SimulatorGaitProfile.speedSuffix(invalid) == "(검증실패)")
        }
        for profile in SimulatorGaitProfile.allCases {
            precondition(profile.titleWithSpeed.hasPrefix(profile.title + " ("))
            print(profile.titleWithSpeed)
        }
    }
}
