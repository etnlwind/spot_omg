"""Anticipatory diagonal touchdown placement; no plant state is observed.

X/Z are preserved exactly. A pair's landing Y is latched before swing and
remains fixed in the body frame throughout stance. World-frame slip must
be evaluated independently; this does not anchor the feet in the world.
This geometry planner does not establish dynamic/contact feasibility.
"""
import numpy as np
from support_shift import OFFSETS, smooth


class FootholdTransfer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.initialized = False
        self.stance = np.ones(4, dtype=bool)
        self.current_y = np.zeros(4)
        self.start_y = np.zeros(4)
        self.end_y = np.zeros(4)
        self.previous_targets = None
        self.diagnostic = {}

    def update(self, targets, goals, com, phase, duty, amplitude, config=None):
        cfg = config or {}
        targets = np.asarray(targets, dtype=float)
        goals = np.asarray(goals, dtype=float)
        com = np.asarray(com, dtype=float)
        if targets.shape != (4, 3) or goals.shape != (4, 3) or com.shape != (3,):
            raise ValueError('Expected four CAD targets/goals and a three-dimensional COM')
        if not np.isfinite(np.r_[targets.ravel(), goals.ravel(), com, phase, duty, amplitude]).all():
            raise ValueError('Nonfinite foothold transfer input')
        if not .5 < duty < 1. or not 0 <= amplitude <= 1.:
            raise ValueError('Expected trot duty greater than .5 and amplitude within 0..1')
        limit = float(cfg.get('max_foothold_y_m', .02))
        gain = float(cfg.get('transfer_gain', 1.))
        if not np.isfinite([limit, gain]).all() or not 0 <= limit <= .025 or not 0 <= gain <= 1.:
            raise ValueError('Invalid foothold transfer bound/gain')
        touchdown_x = cfg.get('touchdown_x_offset_m')
        if touchdown_x is not None:
            touchdown_x = np.broadcast_to(np.asarray(touchdown_x, dtype=float), (4,))
            if not np.isfinite(touchdown_x).all() or np.max(abs(touchdown_x)) > .12:
                raise ValueError('Invalid predicted touchdown X offset')
        planning_amplitude = 1. if cfg.get('anticipate_full_amplitude', False) else amplitude
        local = (float(phase) + OFFSETS) % 1.
        stance = local < duty
        if not self.initialized:
            self.current_y = targets[:, 1].copy()
            self.start_y = self.current_y.copy()
            self.end_y = self.current_y.copy()
            self.initialized = True
        landing = []
        for pair in ((0, 3), (1, 2)):
            ids = np.asarray(pair)
            if not stance[pair[0]] and self.stance[pair[0]]:
                # Predict the next support line before leaving the preceding
                # stance. Optional explicit endpoints handle startup amplitude
                # ramps; otherwise symmetric X travel about each J2 goal is
                # inferred from the final stance sample, without changing X.
                predicted = goals[ids, :2].copy()
                if touchdown_x is None:
                    source = targets if self.previous_targets is None else self.previous_targets
                    predicted[:, 0] = 2 * goals[ids, 0] - source[ids, 0]
                else:
                    predicted[:, 0] += planning_amplitude * touchdown_x[ids]
                line = predicted[1] - predicted[0]
                length = float(np.linalg.norm(line))
                normal = np.array([-line[1], line[0]]) / max(length, 1e-9)
                distance = float(normal @ (com[:2] - predicted[0]))
                valid = length > .01 and abs(normal[1]) > .1
                raw = gain * distance / normal[1] if valid else 0.
                # Fade static CAD COM asymmetry at zero command. Endpoint
                # inference already contains current stride amplitude.
                if touchdown_x is None:
                    raw *= planning_amplitude
                shift = float(np.clip(raw, -limit, limit))
                self.start_y[ids] = self.current_y[ids]
                self.end_y[ids] = goals[ids, 1] + shift
                projected = predicted.copy(); projected[:, 1] += shift
                landing.append(dict(pair=list(pair), predicted_touchdown_xy_m=predicted.tolist(),
                    requested_y_offset_m=raw, applied_y_offset_m=shift,
                    support_line_distance_before_m=distance,
                    support_line_distance_after_m=float(normal @ (com[:2] - projected[0])),
                    clipped=abs(raw) > limit, valid=bool(valid)))
            if not stance[pair[0]]:
                u = float((local[pair[0]] - duty) / (1 - duty))
                self.current_y[ids] = self.start_y[ids] + smooth(u) * (self.end_y[ids] - self.start_y[ids])
            elif not self.stance[pair[0]]:
                self.current_y[ids] = self.end_y[ids]
        result = targets.copy()
        result[:, 1] = self.current_y
        self.stance = stance
        self.previous_targets = targets.copy()
        self.diagnostic = dict(policy='planned-diagonal-foothold-transfer',
            source='nominal-cad-geometry-no-contact-oracle', scheduled_stance=stance.tolist(),
            y_offset_m=(result[:, 1] - goals[:, 1]).tolist(),
            maximum_y_offset_m=limit, planning_amplitude=planning_amplitude,
            touchdown_events=landing, xz_preserved=bool(np.array_equal(result[:, [0, 2]], targets[:, [0, 2]])))
        return result
