import Foundation
@main struct CheckDriveReversal {
 static func main() {
  var sent=[String]()
  let m=RobotBluetoothManager(commandWriter:{sent.append(String(decoding:$0,as:UTF8.self))})
  m.updateDrive(x:0,y:1)
  m.updateDrive(x:0,y:0)
  assert(sent.filter{$0.hasPrefix("drive ")}.count == 1)
  assert(sent.contains{$0.hasPrefix("@D ") && $0.contains(" 0 0")})
  assert(!sent.contains{$0.hasPrefix("@S ")})
  m.updateDrive(x:0,y:-1)
  RunLoop.main.run(until:Date().addingTimeInterval(0.25))
  assert(sent.contains{$0.hasPrefix("@D ") && $0.contains("-1000")})
  assert(sent.filter{$0.hasPrefix("drive ")}.count == 1)
  m.stopDrive(reason:"gesture-ended")
  assert(sent.last!.hasPrefix("@S "))
  m.receiveConsoleText("$SPOTDRIVE stopped reason=drive watchdog expired elapsed=1000ms\r\nERROR: drive watchdog expired\r\n# ")
  assert(m.driveStatus != "보행 명령 거부")
  print("Forward-center-reverse remains one session; release stops immediately")
 }
}
