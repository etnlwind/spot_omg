import Foundation

/// Local diagnostics only: bounded, asynchronous, and never uploaded by the app.
/// The two JSONL files can be retrieved from the developer app container.
final class RobotConnectionTrace {
    private let queue = DispatchQueue(label: "com.etnlwind.spotomg.connection-trace")
    private let folder: URL

    init() {
        folder = FileManager.default.urls(for: .cachesDirectory, in: .userDomainMask)[0]
            .appendingPathComponent("RobotConnection", isDirectory: true)
        record("app-start", "joystick-link-v2")
    }

    func record(_ event: String, _ detail: String) {
        let row: [String: Any] = [
            "epoch_ms": Int64(Date().timeIntervalSince1970 * 1000),
            "uptime_ms": Int64(ProcessInfo.processInfo.systemUptime * 1000),
            "event": event, "detail": detail
        ]
        guard var data = try? JSONSerialization.data(withJSONObject: row) else { return }
        data.append(0x0A)
        let encoded = data
        queue.async { [folder] in
            do {
                let files = FileManager.default
                try files.createDirectory(at: folder, withIntermediateDirectories: true)
                let current = folder.appendingPathComponent("current.jsonl")
                let previous = folder.appendingPathComponent("previous.jsonl")
                let size = (try? files.attributesOfItem(atPath: current.path)[.size] as? NSNumber)?.intValue ?? 0
                if size + encoded.count > 512 * 1024 {
                    if files.fileExists(atPath: previous.path) { try files.removeItem(at: previous) }
                    if files.fileExists(atPath: current.path) { try files.moveItem(at: current, to: previous) }
                }
                if !files.fileExists(atPath: current.path) {
                    files.createFile(atPath: current.path, contents: nil)
                }
                let handle = try FileHandle(forWritingTo: current)
                defer { try? handle.close() }
                try handle.seekToEnd()
                try handle.write(contentsOf: encoded)
            } catch {
                // Disk diagnostics must never block or stop control.
            }
        }
    }
}
