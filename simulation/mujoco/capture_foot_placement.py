"""Optional, bounded pendulum-inspired lateral touchdown damping.

For an upright COM height h rotating about a fixed support, y≈-h*roll and
vy≈-h*roll_rate. The capture-point approximation y+vy/sqrt(g/h) supplies a
heuristic touchdown offset. Torso roll is NOT a measured lateral position;
the approximation may fail during translation or diagonal contact dynamics.
Only delivered IMU packets enter this module. It does not inspect plant truth.
"""
import numpy as np
from support_shift import OFFSETS, smooth


class CaptureFootPlacement:
    def __init__(self):
        self.reset()

    def reset(self):
        self.stance = np.ones(4, dtype=bool)
        self.offset = np.zeros(4)
        self.start = np.zeros(4)
        self.end = np.zeros(4)
        self.diagnostic = {}

    def update(self, phase, duty, attitude, height, config=None):
        """Return four additive CAD +Y offsets in metres.

        Endpoints are latched once at liftoff, interpolated during swing,
        then held throughout stance. Sensor loss keeps the latched motion
        finite and suppresses fresh correction; external safety is unchanged.
        """
        cfg = config or {}
        if not np.isfinite([phase, duty, height]).all() or not .5 < duty < 1.:
            raise ValueError('Invalid capture-foot gait phase/duty/height')
        if not .05 <= height <= .5:
            raise ValueError('Capture-foot height must be COM height, in metres')
        gain = float(cfg.get('gain', .25))
        position_gain = float(cfg.get('position_gain', 1.))
        rate_gain = float(cfg.get('rate_gain', 1.))
        limit = float(cfg.get('limit_m', .01))
        prediction = float(cfg.get('prediction_s', 0.))
        if not np.isfinite([gain, position_gain, rate_gain, limit, prediction]).all():
            raise ValueError('Nonfinite capture-foot setting')
        if not 0 <= gain <= 1. or not 0 <= position_gain <= 2. or not 0 <= rate_gain <= 2.:
            raise ValueError('Invalid capture-foot gain')
        if not 0 <= limit <= .01 or not 0 <= prediction <= .08:
            raise ValueError('Invalid capture-foot limit/prediction')
        valid = attitude is not None and not getattr(attitude, 'failures', 0)
        roll = rate = 0.
        if valid:
            packet = getattr(attitude, 'previous', attitude.filtered) if cfg.get('use_latest_imu', True) else attitude.filtered
            angles = np.asarray(packet, dtype=float)
            rates = np.asarray(getattr(attitude, 'rate', [0., 0.]), dtype=float)
            valid = angles.shape == (2,) and rates.shape == (2,)
            valid = valid and np.isfinite(np.r_[angles, rates]).all()
            if valid:
                roll = float(np.radians(angles[0] / 10.))
                rate = float(np.radians(rates[0] / 10.))
        omega = np.sqrt(9.81 / height)
        raw = -gain * height * (position_gain * (roll + prediction * rate) + rate_gain * rate / omega)
        bounded = float(np.clip(raw, -limit, limit))
        enabled = bool(cfg.get('enabled', True))
        local = (float(phase) + OFFSETS) % 1.
        stance = local < duty
        events = []
        for pair in ((0, 3), (1, 2)):
            ids = np.asarray(pair)
            if self.stance[pair[0]] and not stance[pair[0]]:
                self.start[ids] = self.offset[ids]
                # Missing measurements retain the last landed placement;
                # explicit disabling returns to zero on the next touchdown.
                endpoint = bounded if valid and enabled else (0. if not enabled else self.offset[pair[0]])
                self.end[ids] = endpoint
                events.append(dict(pair=list(pair), target_y_m=float(endpoint)))
            if not stance[pair[0]]:
                u = float((local[pair[0]] - duty) / (1. - duty))
                self.offset[ids] = self.start[ids] + smooth(u) * (self.end[ids] - self.start[ids])
            elif not self.stance[pair[0]]:
                self.offset[ids] = self.end[ids]
        self.stance = stance
        self.diagnostic = dict(policy='bounded-pendulum-touchdown-damping',
            source='delayed-imu-pendulum-hypothesis-not-lateral-position-sensor',
            sensor_valid=bool(valid), enabled=enabled, com_height_m=float(height),
            roll_deg=float(np.degrees(roll)), rate_deg_s=float(np.degrees(rate)),
            raw_capture_offset_m=float(raw), limited_capture_offset_m=bounded,
            clipped=bool(abs(raw) > limit), placement_offset_m=self.offset.tolist(),
            scheduled_stance=stance.tolist(), touchdown_events=events)
        return self.offset.copy()
