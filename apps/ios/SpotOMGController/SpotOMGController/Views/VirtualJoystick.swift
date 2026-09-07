import SwiftUI

struct VirtualJoystick: View {
    let enabled: Bool
    let onChange: (Double, Double) -> Void
    let onRelease: () -> Void

    @State private var knobOffset: CGSize = .zero

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
                    .onChanged { value in
                        guard enabled else { return }
                        let dx = value.location.x - diameter / 2
                        let dy = value.location.y - diameter / 2
                        let distance = hypot(dx, dy)
                        let scale = distance > travel ? travel / distance : 1
                        knobOffset = CGSize(width: dx * scale, height: dy * scale)
                        onChange(Double(dx * scale / travel),
                                 Double(-dy * scale / travel))
                    }
                    .onEnded { _ in
                        resetToCenter()
                    })
            .opacity(enabled ? 1 : 0.45)
            .frame(maxWidth: .infinity, maxHeight: .infinity)
        }
        .aspectRatio(1, contentMode: .fit)
        .onChange(of: enabled) { _, isEnabled in
            if !isEnabled { resetToCenter() }
        }
        .onDisappear { resetToCenter() }
    }

    private func resetToCenter() {
        withAnimation(.spring(response: 0.22, dampingFraction: 0.72)) {
            knobOffset = .zero
        }
        onRelease()
    }
}
