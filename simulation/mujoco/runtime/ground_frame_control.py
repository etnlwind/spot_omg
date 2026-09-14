"""One CAD foot solve for support posture and ground-frame swing clearance.

Only delivered encoder/IMU packets are inputs. Contact and ground estimates
are scheduled flat-floor hypotheses, not measurements of contact force.
This is a position-servo kinematic controller, not torque WBC.
"""

# Support direct execution from any working directory.
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[3]))
from collections import deque
import numpy as np
import mujoco
from simulation.mujoco.runtime.support_shift import SupportShift, OFFSETS, smooth


class GroundFrameControl:
    def __init__(self, model):
        self.kin = SupportShift(model)
        self.reset()

    def reset(self):
        self.correction = np.zeros(12)
        self.ground_offset = 0.
        self.history = deque(maxlen=3)
        self.tracking = np.zeros(12)
        self.previous_velocity = None
        self.nominal_history = deque(maxlen=5)
        self.diagnostic = {}

    def apply(self, nominal, encoders, attitude, phase, duty, config=None, enabled=True):
        cfg = config or {}
        nominal = np.asarray(nominal, dtype=float).reshape(12)
        low = np.tile([-30., -45., 0.], 4)
        high = np.tile([30., 100., 150.], 4)
        if not np.isfinite(nominal).all():
            raise ValueError('Nonfinite nominal ground-frame target')
        valid = enabled and encoders is not None and attitude is not None
        valid = valid and not getattr(attitude, 'failures', 0)
        if valid:
            encoders = np.asarray(encoders, dtype=float)
            packet = getattr(attitude, 'previous', attitude.filtered) if cfg.get('use_latest_imu', False) else attitude.filtered
            angle = np.radians(np.asarray(packet, dtype=float) / 10.)
            rate = np.radians(np.asarray(getattr(attitude, 'rate', [0., 0.]), dtype=float) / 10.)
            valid = encoders.shape == (12,) and angle.shape == (2,) and rate.shape == (2,)
            valid = valid and np.isfinite(np.r_[encoders, angle, rate, phase, duty]).all()
        if not valid:
            self.reset()
            self.diagnostic = dict(enabled=False, reason='disabled-or-missing-sensor')
            return np.clip(nominal, low, high)
        if not 0 < duty < 1:
            raise ValueError('Duty must be between zero and one')
        bounds = dict(stance_gain=(.5, 0., 2.), stance_damping_s=(0., 0., .1),
                      swing_gain=(1., 0., 1.), max_rotation_rad=(.08, 0., .2),
                      max_correction_deg=(12., .1, 25.), max_step_deg=(1., .05, 5.),
                      height_limit_m=(.01, 0., .03), ground_spread_m=(.03, .001, .08),
                      imu_prediction_s=(0., 0., .06),
                      tracking_gain=(0., 0., .5), tracking_limit_deg=(3., 0., 6.))
        values = {}
        for key, (default, minimum, maximum) in bounds.items():
            values[key] = float(cfg.get(key, default))
            if not np.isfinite(values[key]) or not minimum <= values[key] <= maximum:
                raise ValueError('Invalid ground-frame setting: ' + key)
        iterations = int(cfg.get('iterations', 12))
        if not 1 <= iterations <= 40:
            raise ValueError('Invalid ground-frame iterations')
        alignment_frames = int(cfg.get('ground_alignment_frames', 0))
        if not 0 <= alignment_frames <= 4:
            raise ValueError('Invalid ground alignment delay')
        angle = np.clip(angle + values['imu_prediction_s'] * rate, -.35, .35)
        k = self.kin; m = k.model; d = k.data
        local = (phase + OFFSETS) % 1
        stance = local < duty
        weights = np.zeros(4)
        for i in np.flatnonzero(~stance):
            u = (local[i] - duty) / (1 - duty)
            weights[i] = smooth(u / .15) * smooth((1 - u) / .15)
        cr, sr = np.cos(angle[0] / 2), np.sin(angle[0] / 2)
        cp, sp = np.cos(angle[1] / 2), np.sin(angle[1] / 2)
        quat = np.array([cp * cr, cp * sr, sp * cr, -sp * sr])
        matrix = np.empty(9); mujoco.mju_quat2Mat(matrix, quat); matrix = matrix.reshape(3, 3)

        def pose(q, tilted=False):
            k.set_angles(q)
            if tilted:
                d.qpos[3:7] = quat
                mujoco.mj_kinematics(m, d); mujoco.mj_comPos(m, d)

        # Flat nominal clearance is common ground-relative geometry, not a
        # rotation of SupportShift's hybrid center-XY/bottom-Z point.
        pose(nominal)
        centers = np.array([d.geom_xpos[f].copy() for f in k.feet])
        nominal_bottom = np.array([k.foot(i)[2] for i in range(4)])
        center = np.average(d.xipos, axis=0, weights=m.body_mass)
        reference_ground = float(np.median(nominal_bottom[stance])) if np.any(stance) else float(nominal_bottom.min())
        clearance = nominal_bottom - reference_ground
        pose(nominal, True)
        tilted_nominal_bottom = np.array([k.foot(i)[2] for i in range(4)])
        nominal_ground_tilted = float(np.median(tilted_nominal_bottom[stance])) if np.any(stance) else float(tilted_nominal_bottom.min())
        rotation = np.r_[values['stance_gain'] * angle + values['stance_damping_s'] * rate, 0.]
        rotation = np.clip(rotation, -values['max_rotation_rad'], values['max_rotation_rad'])
        support_delta = np.cross(rotation, centers - center)
        support_xy = centers[:, :2] + support_delta[:, :2]
        support_z = tilted_nominal_bottom + (support_delta @ matrix.T)[:, 2]
        pose(encoders, True)
        observed_bottom = np.array([k.foot(i)[2] for i in range(4)])
        estimate_stance = stance
        estimate_reference = nominal_ground_tilted
        if alignment_frames and len(self.nominal_history) >= alignment_frames:
            old_nominal, old_phase = self.nominal_history[-alignment_frames]
            estimate_stance = ((old_phase + OFFSETS) % 1) < duty
            pose(old_nominal, True)
            old_bottom = np.array([k.foot(i)[2] for i in range(4)])
            if np.any(estimate_stance):
                estimate_reference = float(np.median(old_bottom[estimate_stance]))
        support_heights = observed_bottom[estimate_stance]
        ground_valid = len(support_heights) >= 2 and np.ptp(support_heights) <= values['ground_spread_m']
        if ground_valid:
            requested_offset = float(np.median(support_heights) - estimate_reference)
            bounded_offset = float(np.clip(requested_offset, -values['height_limit_m'], values['height_limit_m']))
            self.ground_offset += .35 * (bounded_offset - self.ground_offset)
        else:
            self.ground_offset *= .9
        ground = nominal_ground_tilted + self.ground_offset
        swing_z = tilted_nominal_bottom + values['swing_gain'] * (ground + clearance - tilted_nominal_bottom)
        target_xy = (1 - weights[:, None]) * support_xy + weights[:, None] * centers[:, :2]
        target_z = (1 - weights) * support_z + weights * swing_z

        def errors_and_jacobians(q):
            pose(q, True)
            errors = []; jacobians = []
            for i, f in enumerate(k.feet):
                point, jac = k.foot(i, True)  # Lowest vertex reselected AFTER rotation.
                jp = np.zeros((3, m.nv)); jr = np.zeros_like(jp)
                mujoco.mj_jacGeom(m, d, jp, jr, f)
                block = jp[:, k.v[i * 3:i * 3 + 3]]
                body_center = matrix.T @ d.geom_xpos[f]
                errors.append(np.r_[target_xy[i] - body_center[:2], target_z[i] - point[2]])
                jacobians.append(np.vstack([matrix[:, 0] @ block, matrix[:, 1] @ block, jac[2]]))
            return np.asarray(errors), jacobians

        q = nominal + self.correction
        for _ in range(iterations):
            errors, jacobians = errors_and_jacobians(q)
            if np.max(np.linalg.norm(errors, axis=1)) < .0001:
                break
            for i, jac in enumerate(jacobians):
                dq = np.linalg.solve(jac.T @ jac + np.eye(3) * 1e-7, jac.T @ errors[i])
                q[i * 3:i * 3 + 3] += np.clip(np.degrees(dq), -4., 4.)
            q = np.clip(q, low, high)
        raw_correction = q - nominal
        desired_correction = np.clip(raw_correction, -values['max_correction_deg'], values['max_correction_deg'])
        self.correction += np.clip(desired_correction - self.correction, -values['max_step_deg'], values['max_step_deg'])
        # Optional load/lag estimate compares the delivered encoder with a
        # command from the same 40ms epoch, never today's advancing target.
        aligned = self.history[-2] if len(self.history) >= 2 else None
        desired_tracking = np.zeros(12) if aligned is None else np.clip(values['tracking_gain'] * (aligned - encoders), -values['tracking_limit_deg'], values['tracking_limit_deg'])
        self.tracking += np.clip(desired_tracking - self.tracking, -.25, .25)
        result = np.clip(nominal + self.correction + self.tracking, low, high)
        velocity = np.zeros(12) if not self.history else np.radians(result - self.history[-1]) / .02
        acceleration = np.zeros(12) if self.previous_velocity is None else (velocity - self.previous_velocity) / .02
        self.previous_velocity = velocity.copy()
        self.history.append(result.copy())
        self.nominal_history.append((nominal.copy(), float(phase)))
        errors, _ = errors_and_jacobians(result)
        self.diagnostic = dict(enabled=True, sensor_source='delayed-imu-quantized-encoders',
            contact_source='scheduled-flat-floor-hypothesis', ground_valid=bool(ground_valid),
            ground_offset_m=self.ground_offset, scheduled_stance=stance.tolist(), swing_weights=weights.tolist(),
            ground_alignment_frames=alignment_frames, observed_roll_pitch_deg=np.degrees(angle).tolist(),
            target_world_z_m=target_z.tolist(), nominal_clearance_m=clearance.tolist(),
            applied_task_residual_m=np.linalg.norm(errors, axis=1).tolist(),
            j1_correction_deg=self.correction[::3].tolist(), correction_deg=self.correction.tolist(),
            correction_clipped=bool(np.any(abs(raw_correction) > values['max_correction_deg'])),
            correction_slew_limited=bool(np.any(abs(desired_correction - self.correction) > 1e-6)),
            tracking_correction_deg=self.tracking.tolist(),
            target_velocity_rad_s=velocity.tolist(), target_acceleration_rad_s2=acceleration.tolist(),
            joint_limit_reached=bool(np.any((result <= low + 1e-7) | (result >= high - 1e-7))))
        return result
