import SwiftUI
import UIKit

struct ContentView: View {
    @EnvironmentObject private var bluetooth: RobotBluetoothManager

    var body: some View {
        NavigationStack { ControlView() }
            .toolbar(.hidden, for: .navigationBar)
            .background(FirstScreenAppearance { bluetooth.start() })
    }
}

/// onAppear/task can run before rendering. Wait for a display cycle after this
/// view joins the window so the control screen gets a frame before services start.
private struct FirstScreenAppearance: UIViewRepresentable {
    let action: () -> Void

    func makeUIView(context: Context) -> AppearanceView {
        AppearanceView(action: action)
    }

    func updateUIView(_ view: AppearanceView, context: Context) {}

    static func dismantleUIView(_ view: AppearanceView, coordinator: ()) {
        view.cancelDisplayLink()
    }

    final class AppearanceView: UIView {
        private var action: (() -> Void)?
        private var displayLink: CADisplayLink?
        private var displayTicks = 0

        init(action: @escaping () -> Void) {
            self.action = action
            super.init(frame: .zero)
            isUserInteractionEnabled = false
            backgroundColor = .clear
        }

        required init?(coder: NSCoder) { fatalError("init(coder:) has not been implemented") }

        func cancelDisplayLink() {
            displayLink?.invalidate()
            displayLink = nil
            displayTicks = 0
        }

        override func didMoveToWindow() {
            super.didMoveToWindow()
            cancelDisplayLink()
            guard window != nil, action != nil else { return }
            let link = CADisplayLink(target: self, selector: #selector(displayFrame))
            displayLink = link
            link.add(to: .main, forMode: .common)
        }

        @objc private func displayFrame() {
            displayTicks += 1
            // The first callback precedes rendering. The second follows a frame.
            guard displayTicks >= 2 else { return }
            cancelDisplayLink()
            DispatchQueue.main.async { [weak self] in
                guard let self, self.window != nil, let action = self.action else { return }
                self.action = nil
                action()
            }
        }
    }
}
