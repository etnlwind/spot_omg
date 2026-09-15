import Foundation
@main struct CheckBatteryWarning {
 static func main() {
  var b=RobotBatteryWarning()
  b.observe(10900,at:0);assert(b.level==1)
  b.observe(10300,at:1);assert(b.level==2)
  for (t,mv) in [(2.0,10900),(3.0,11000),(4.0,11300),(5.0,11400),(6.0,11400)] {b.observe(mv,at:t);assert(b.level==2)}
  b.observe(11400,at:10);assert(b.level==0)
  b.observe(11000,at:11);assert(b.level==1)
  b.observe(10500,at:12);assert(b.level==2)
  b.observe(0,at:13);b.observe(65535,at:13);b.observe(12000,at:14,historical:true);assert(b.level==2)
  b.observe(12000,at:15)
  for _ in 0..<10 {b.observe(12000,at:15)}
  b.observe(12000,at:20);assert(b.level==2)
  b.observe(12000,at:21);assert(b.level==0)
  for s in ["ID 1 voltage=0mV", "$BATTERY mv=65535", "$BATTERY mv=10500oops", "ID 2 voltage=10000mV", "old $BATTERY mv=10000"] { assert(RobotBatteryWarning.reading(in:s)==nil) }
  b.observe(10500,at:30);b.observe(12000,at:31);b.observe(12000,at:33)
  b.observe(12000,at:60);assert(b.level==2)
  b.observe(12000,at:62);b.observe(12000,at:65);assert(b.level==0)
  let line="$BATTERY mv=10300\r\n"
  for split in 0...line.count {
   var sent=[String]()
   let m=RobotBluetoothManager(commandWriter:{sent.append(String(decoding:$0,as:UTF8.self))})
   m.updateDrive(x:0,y:1);sent=[]
   m.receiveConsoleText(String(line.prefix(split)));m.receiveConsoleText(String(line.dropFirst(split)))
   assert(m.batteryWarning.level==2 && m.supplyVoltageMillivolts==10300)
   assert(sent.isEmpty)
   m.pollBattery(now:Date().addingTimeInterval(10));assert(sent.isEmpty)
   m.disconnect();assert(m.batteryWarning.level==0 && m.lastVoltageRead==nil)
  }
  var sent=[String]()
  let m=RobotBluetoothManager(commandWriter:{sent.append(String(decoding:$0,as:UTF8.self))})
  m.receiveConsoleText("$SPOTSTATE pose=custom\r\n# ID 1 voltage=11500mV\r\n# ")
  m.receiveConsoleText("Gait diagnostics: samples=2828 min_voltage=10300mV lag=121\r\n")
  assert(m.batteryWarning.level==2 && m.supplyVoltageMillivolts==11500)
  sent=[];m.pollBattery(now:Date().addingTimeInterval(6));assert(sent==["read 1\n"])
  m.pollBattery(now:Date().addingTimeInterval(12));assert(sent.count==1)
  m.disconnect()
  print("PASS: Swift/iPhone/Mac battery boundaries, droop, recovery, fragmented packets, idle-only polling; no robot connected")
 }
}
