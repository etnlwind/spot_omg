import Foundation
@main struct CheckRemote {
 static func main() {
  var sent=[String]()
  let m=RobotBluetoothManager(commandWriter: { sent.append(String(decoding:$0,as:UTF8.self)) })
  m.send(.relax)
  assert(sent == ["landing\n"])
  m.receiveConsoleText("ERROR: motion aborted\r\n# ")
  assert(!sent.contains("relax\n"))
  m.send(.relax)
  m.receiveConsoleText("OK landing\r\n# ")
  assert(sent.last == "relax\n")
  sent.removeAll()
  m.send(.stand)
  let now=Date().timeIntervalSince1970
  m.beginRemoteRequest(AppRemoteRequest(id:UUID().uuidString,action:"disconnect",issuedAt:now,expiresAt:now+30,target:nil))
  assert(m.state.isReady && m.remotePending != nil)
  assert(sent.last == "\u{03}")
  m.send(.stand11)
  assert(sent.last == "\u{03}")
  m.receiveConsoleText("ERROR: motion aborted\r\n# ")
  m.advanceRemoteRequest()
  assert(!m.state.isReady && m.remotePending == nil)
  print("Remote disconnect and Landing-before-Relax checks passed")
 }
}
