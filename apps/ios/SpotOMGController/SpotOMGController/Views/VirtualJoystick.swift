import SwiftUI

struct VirtualJoystick: View {
    let enabled: Bool
    let onChange: (Double, Double) -> Void
    let onRelease: (String) -> Void

    @State private var knobOffset: CGSize = .zero
    @GestureState private var gestureActive = false
    @State private var touchActive = false

    var body: some View {
        GeometryReader { geometry in
            let diameter = min(geometry.size.width, geometry.size.height)
            let travel = diameter * 0.32

            ZStack {
                Circle()
                    .fill(.black.opacity(0.82))
                    .overlay(Circle().stroke(.green.opacity(0.55), lineWidth: 2))
                Circle()
                    .stroke(.green.opacity(0.16), lineWidth: 1)
                    .frame(width: diameter * 0.52, height: diameter * 0.52)
                Rectangle().fill(.green.opacity(0.12)).frame(width: 1)
                Rectangle().fill(.green.opacity(0.12)).frame(height: 1)
                Text("FWD")
                    .font(.system(size: 9, weight: .bold, design: .monospaced))
                    .foregroundStyle(.green.opacity(0.65))
                    .offset(y: -diameter * 0.39)
                Circle()
                    .fill(enabled ? Color.green : Color.gray)
                    .frame(width: diameter * 0.30, height: diameter * 0.30)
                    .shadow(color: .green.opacity(0.4), radius: 8)
                    .offset(knobOffset)
            }
            .frame(width: diameter, height: diameter)
            .contentShape(Circle())
            .highPriorityGesture(
                DragGesture(minimumDistance: 0)
                    .updating($gestureActive) { _, active, _ in active = true }
                    .onChanged { value in
                        guard enabled else { return }
                        touchActive = true
                        let dx = value.location.x - diameter / 2
                        let dy = value.location.y - diameter / 2
                        let distance = hypot(dx, dy)
                        let scale = distance > travel ? travel / distance : 1
                        knobOffset = CGSize(width: dx * scale, height: dy * scale)
                        onChange(Double(dx * scale / travel),
                                 Double(-dy * scale / travel))
                    }
                    .onEnded { _ in
                        resetToCenter(reason: "gesture-ended")
                    })
            .opacity(enabled ? 1 : 0.45)
            .frame(maxWidth: .infinity, maxHeight: .infinity)
        }
        .aspectRatio(1, contentMode: .fit)
        .onChange(of: enabled) { _, isEnabled in
            if !isEnabled { resetToCenter(reason: "control-disabled") }
        }
        .onDisappear { resetToCenter(reason: "view-disappeared") }
        .onChange(of: gestureActive) { _, active in
            // SwiftUI also resets GestureState when a parent cancels a drag;
            // onEnded alone does not cover that case.
            if !active { resetToCenter(reason: "gesture-ended-or-cancelled") }
        }
    }

    private func resetToCenter(reason: String) {
        guard touchActive else { return }
        touchActive = false
        withAnimation(.spring(response: 0.22, dampingFraction: 0.72)) {
            knobOffset = .zero
        }
        onRelease(reason)
    }
}
