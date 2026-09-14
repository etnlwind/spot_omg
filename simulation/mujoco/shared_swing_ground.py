"""Experimental equal diagonal swing clearance in the gravity frame.

Inputs are delivered IMU and 40ms/4096-count encoder samples. The controller
owns private CAD data; no physical-state or contact-force oracle is read.
Scheduled opposite stance feet estimate a flat floor. Their commands remain
unchanged. This is bounded position IK, not a contact or torque controller.
"""
import numpy as np
import mujoco
from scipy.optimize import least_squares
from support_shift import SupportShift, OFFSETS, smooth


class SharedSwingGround:
    def __init__(self, model):
        self.kin = SupportShift(model)
        self.low = np.tile([-30., -45., 0.], 4)
        self.high = np.tile([30., 100., 150.], 4)
        self.reset()

    def reset(self):
        self.correction = np.zeros(12)
        self.diagnostic = {}
        self.last_nominal = None
        self.last_phase = None
        self.fallback_active = None
        self.last_output = None

    def fallback(self, nominal, reason, phase=None, duty=.7):
        """Release a correction gradually, even if stance is now scheduled."""
        requested=nominal.copy()
        # A sensor failure cannot validate a simultaneous incoming nominal
        # change. Freeze the last accepted nominal while releasing correction.
        nominal=self.last_nominal.copy() if self.last_nominal is not None else nominal.copy()
        before=self.correction.copy()
        prior=nominal+before
        if np.any(prior < self.low-1e-8) or np.any(prior > self.high+1e-8):
            raise ValueError('Retained fallback state is outside joint limits')
        self.correction+=np.clip(-self.correction,-.4,.4)
        result=nominal+self.correction
        output_before=prior if self.last_output is None else self.last_output
        self.fallback_active=reason if np.max(abs(self.correction))>1e-9 else None
        self.last_nominal=nominal.copy()
        self.last_phase=float(phase) if phase is not None and np.isfinite(phase) else None
        stance=np.ones(4,dtype=bool) if self.last_phase is None else ((phase+OFFSETS)%1)<duty
        self.diagnostic=dict(enabled=False,reason=reason,fallback_releasing=True,
            correction_deg=self.correction.tolist(),max_correction_step_deg=float(abs(self.correction-before).max()),
            max_output_step_deg=float(abs(result-output_before).max()),
            deferred_nominal_delta_deg=(requested-nominal).tolist(),
            stance_residual_correction_deg=self.correction.reshape(4,3)[stance].tolist(),
            stance_preserved=bool(np.max(abs(self.correction.reshape(4,3)[stance]),initial=0.)<1e-9),
            fallback_slew_limit_deg=.4)
        self.last_output=result.copy()
        return result

    def apply(self, nominal, encoders, attitude, phase, duty, period_s,
              amplitude, gain=.25, enabled=True):
        nominal = np.asarray(nominal, dtype=float)
        if nominal.shape != (12,) or not np.isfinite(nominal).all():
            if self.last_nominal is not None:
                return self.fallback(self.last_nominal,'nonfinite-nominal',phase,duty)
            raise ValueError('Expected 12 finite nominal joint angles')
        if np.any(nominal < self.low) or np.any(nominal > self.high):
            raise ValueError('Nominal angles exceed existing gait limits')
        if not np.isfinite([phase, duty, period_s, amplitude, gain]).all():
            if np.isfinite([duty,period_s,amplitude,gain]).all():
                return self.fallback(nominal,'nonfinite-phase',None,duty)
            raise ValueError('Nonfinite shared swing configuration')
        if not (.5 < duty < 1 and period_s > .1 and 0 <= amplitude <= 1 and 0 <= gain <= 1):
            raise ValueError('Invalid shared swing configuration')
        valid = enabled and encoders is not None and attitude is not None
        if valid:
            encoders = np.asarray(encoders, dtype=float)
            packet = np.asarray(getattr(attitude, 'filtered', []), dtype=float)
            valid = (encoders.shape == (12,) and packet.shape == (2,)
                     and np.isfinite(encoders).all() and np.isfinite(packet).all()
                     and not getattr(attitude, 'failures', 0))
        if not valid:
            return self.fallback(nominal,'missing-or-invalid-delivered-sensor',phase,duty)
        if self.last_nominal is not None and np.max(abs(nominal-self.last_nominal))>12.:
            return self.fallback(self.last_nominal,'nominal-discontinuity',phase,duty)
        if self.fallback_active:
            return self.fallback(nominal,self.fallback_active,phase,duty)
        if self.last_phase is not None:
            phase_step=(phase-self.last_phase+.5)%1-.5
            if phase_step < -1e-7 or phase_step > 1.5*.02/period_s+1e-7:
                return self.fallback(nominal,'phase-discontinuity',phase,duty)
        self.last_nominal=nominal.copy();self.last_phase=float(phase)
        angle = np.radians(packet / 10.)
        cr, sr = np.cos(angle[0]/2), np.sin(angle[0]/2)
        cp, sp = np.cos(angle[1]/2), np.sin(angle[1]/2)
        quat = np.array([cp*cr, cp*sr, sp*cr, -sp*sr])
        matrix = np.empty(9)
        mujoco.mju_quat2Mat(matrix, quat)
        matrix = matrix.reshape(3, 3)
        k, m, d = self.kin, self.kin.model, self.kin.data

        def pose(q):
            k.set_angles(q)
            d.qpos[3:7] = quat
            mujoco.mj_kinematics(m, d)
            mujoco.mj_comPos(m, d)

        local = (phase + OFFSETS) % 1
        stance = local < duty
        delayed_stance = ((phase - .04/period_s + OFFSETS) % 1) < duty
        support = stance & delayed_stance
        swing = ~stance
        if not np.any(swing):
            if np.max(abs(self.correction))>.400001:
                return self.fallback(nominal,'unexpected-stance-boundary-residual',phase,duty)
            # Expected zero because the previous swing is tapered to zero
            # before its boundary. Never spend stance joint commands.
            before = self.correction.copy()
            self.correction[:] = 0
            self.diagnostic = dict(enabled=True, all_stance=True,
                stance_preserved=True, correction_deg=self.correction.tolist(),
                boundary_reset_step_deg=float(abs(before).max()))
            self.last_output=nominal.copy()
            return nominal.copy()
        pose(encoders)
        observed_bottom = np.array([k.foot(i)[2] for i in range(4)])
        support_heights = observed_bottom[support]
        ground_valid = len(support_heights) >= 2 and np.ptp(support_heights) <= .025
        if not ground_valid:
            result=self.fallback(nominal,'scheduled-floor-reference-invalid',phase,duty)
            self.diagnostic.update(support_count=int(support.sum()),support_heights_m=support_heights.tolist())
            return result
        ground = float(np.mean(support_heights))
        pose(nominal)
        target_xy = np.array([(matrix.T @ d.geom_xpos[f])[:2] for f in k.feet])
        nominal_z = np.array([k.foot(i)[2] for i in range(4)])
        clearance = np.zeros(4)
        blend = np.zeros(4)
        envelope = np.zeros(4)
        for i in np.flatnonzero(swing):
            u = (local[i]-duty)/(1-duty)
            # EXACT existing full-forward rise/hold/fall scalar waveform.
            lift = smooth(u/.4) if u < .4 else smooth((1-u)/.3) if u > .7 else 1.
            clearance[i] = .032*amplitude*lift
            blend[i] = smooth(u/.15)*smooth((1-u)/.15)
            # Reserve one tick to reach zero before a stance boundary. This
            # tightens only the correction envelope, never stance commands.
            ticks_remaining = max(0., (1-local[i])*period_s/.02-1.)
            envelope[i] = min(6., .4*ticks_remaining)
        full_world_z = ground + clearance
        target_z = nominal_z + gain*blend*(full_world_z-nominal_z)
        result = nominal.copy()
        previous = self.correction.copy()
        bounded = False
        infeasible_step = False
        for i in np.flatnonzero(swing):
            section = slice(i*3,i*3+3)
            cap = envelope[i]
            if cap < 1e-10:
                continue
            lo = np.maximum(np.maximum(nominal[section]-cap,self.low[section]),nominal[section]+previous[section]-.4)
            hi = np.minimum(np.minimum(nominal[section]+cap,self.high[section]),nominal[section]+previous[section]+.4)
            if np.any(hi-lo < -1e-8):
                # A discontinuous phase jump can make the old correction
                # incompatible with the required boundary. Report it.
                return self.fallback(nominal,'correction-boundary-infeasible',phase,duty)
            weight = np.array([20.,20.,1.])
            def task(qleg, jacobian=False):
                candidate=nominal.copy();candidate[section]=qleg
                pose(candidate)
                f = k.feet[i]
                point, bottom_jac = k.foot(i, True)
                jp = np.zeros((3, m.nv)); jr = np.zeros_like(jp)
                mujoco.mj_jacGeom(m, d, jp, jr, f)
                block = jp[:, k.v[i*3:i*3+3]]
                body_center = matrix.T @ d.geom_xpos[f]
                error = np.r_[body_center[:2]-target_xy[i], point[2]-target_z[i]]
                jac = np.vstack([matrix[:, 0]@block, matrix[:, 1]@block, bottom_jac[2]])
                return weight[:,None]*jac*(np.pi/180) if jacobian else weight*error
            # The shrinking tail envelope can fix a coordinate to exactly one
            # value (previous correction minus one slew step). Optimize only
            # the remaining free coordinates; equal bounds are not infeasible.
            free=hi-lo>1e-8
            fixed=(lo+hi)/2
            def unpack(values):
                qleg=fixed.copy();qleg[free]=values
                return qleg
            if np.any(free):
                seed=np.clip(nominal[section][free]+previous[section][free],lo[free]+1e-11,hi[free]-1e-11)
                solved=least_squares(lambda v:task(unpack(v)),seed,
                    jac=lambda v:task(unpack(v),True)[:,free],bounds=(lo[free],hi[free]),
                    max_nfev=20,ftol=1e-8,xtol=1e-8,gtol=1e-10)
                qleg=unpack(solved.x)
            else:
                qleg=fixed
            result[section]=qleg
            bounded |= bool(np.any(abs(qleg-lo)<1e-5) or np.any(abs(qleg-hi)<1e-5))
        self.correction = result-nominal
        pose(result)
        applied_z = np.array([k.foot(i)[2] for i in range(4)])
        applied_xy = np.array([(matrix.T@d.geom_xpos[f])[:2] for f in k.feet])
        self.diagnostic = dict(enabled=True, all_stance=False,
            sensor_source='filtered-delayed-imu-and-40ms-4096-count-encoders',
            ground_source='opposite-scheduled-support-pair-flat-floor-hypothesis',
            support= support.tolist(), ground_m=ground,
            support_height_spread_m=float(np.ptp(support_heights)),
            equal_scalar_clearance_m=clearance.tolist(), blend=blend.tolist(),
            full_shared_target_world_z_m=full_world_z.tolist(),
            blended_target_world_z_m=target_z.tolist(), applied_world_z_m=applied_z.tolist(),
            world_z_residual_m=(target_z-applied_z).tolist(),
            body_xy_residual_m=np.linalg.norm(applied_xy-target_xy,axis=1).tolist(),
            correction_deg=self.correction.tolist(),
            max_correction_step_deg=float(abs(self.correction-previous).max()),
            correction_limited=bounded,
            phase_jump_step_infeasible=infeasible_step,
            stance_preserved=bool(np.array_equal(result.reshape(4,3)[stance],nominal.reshape(4,3)[stance])))
        self.last_output=result.copy()
        return result


def install(robot, gain=.25):
    """Preview-only adapter after the existing FF and legacy balance paths."""
    from position_wbc import EncoderChannel
    control = SharedSwingGround(robot.plant.model)
    channel = EncoderChannel(delay_frames=2)
    old_apply = robot.balance.apply
    def apply(nominal, attitude, valid, permitted):
        command = old_apply(nominal, attitude, valid, permitted)
        # This is the sensor acquisition boundary. Raw qpos goes only into the
        # explicit quantized/delayed channel and never directly into control.
        delivered = channel.read(np.degrees(robot.plant.data.qpos[robot.plant.q]))
        if not robot.motion:
            inherited=control.correction.copy()
            control.reset()
            control.diagnostic=dict(enabled=False,reason='not-moving',
                transition_active=robot.transition is not None,
                previous_correction_deg=inherited.tolist())
            return command
        phase = getattr(robot, 'ground_evaluated_phase', robot.phase)
        amplitude = smooth(min(1., robot.elapsed/4.))*min(1., abs(robot.linear))
        enabled = bool(valid and permitted and robot.imu_reading
            and robot.imu_reading['age_ms'] <= 100 and robot.safety == 'ok')
        return control.apply(command, delivered, attitude, phase, .7, 3.2, amplitude, gain, enabled)
    robot.balance.apply = apply
    robot.shared_swing_ground = control
    return control
