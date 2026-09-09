#ifndef GAIT_POLICY_H
#define GAIT_POLICY_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * HAL-independent diagonal-trot policy shared by STM32 and the MuJoCo host.
 *
 * Robot coordinates match the simulator: X forward, Y left, Z up. Canonical
 * joint angles are degrees. Leg order is FL, FR, RL, RR and the support mask
 * uses bit (1 << leg_index).
 */

#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define GAIT_POLICY_LEG_COUNT 4U
#define GAIT_POLICY_SIM_TROT_PERIOD_MS 800U
#define GAIT_POLICY_SIM_TROT_CONTROL_HZ 50U
#define GAIT_POLICY_SIM_TROT_DUTY 0.50f
#define GAIT_POLICY_SIM_TROT_STANCE_J1_DEG 4.0f
#define GAIT_POLICY_SIM_TROT_STANCE_J2_DEG 45.0f
#define GAIT_POLICY_SIM_TROT_STANCE_J3_DEG 90.0f
#define GAIT_POLICY_SIM_TROT_HIP_AMPLITUDE_DEG 8.8f
#define GAIT_POLICY_SIM_TROT_LIFT_AMPLITUDE_DEG 30.0f

#define GAIT_POLICY_TROT2_PERIOD_MS 800U
#define GAIT_POLICY_TROT2_DUTY 0.50f
#define GAIT_POLICY_TROT2_FOLD_J2_DEG 78.0f
#define GAIT_POLICY_TROT2_FOLD_J3_DEG 108.0f

/*
 * Slow real-hardware starts need a finite support polygon.  With duty=0.5 a
 * diagonal pair lifts at phase zero and leaves only a two-point support line;
 * duty=0.65 gives 30% of each cycle with all four feet scheduled in stance.
 * This belongs to the hardware-independent trot3 geometry, not the actuator
 * capability layer.
 */
#define GAIT_POLICY_TROT3_PERIOD_MS 2200U
#define GAIT_POLICY_TROT3_MAX_PERIOD_MS 2400U
#define GAIT_POLICY_TROT3_DUTY 0.65f
#define GAIT_POLICY_TROT3_WEIGHT_SHIFT_DEG 1.5f
#define GAIT_POLICY_TROT3_FOLD_J2_DEG 78.0f
#define GAIT_POLICY_TROT3_FOLD_J3_DEG 100.0f

/*
 * Trot4 is the conservative posture experiment derived from loaded trot3
 * diagnostics.  It trades some four-foot overlap for a longer swing, uses a
 * smaller circular toe path, and gives the phase boundary zero acceleration.
 */
#define GAIT_POLICY_TROT4_PERIOD_MS 1600U
#define GAIT_POLICY_TROT4_MAX_PERIOD_MS 2400U
#define GAIT_POLICY_TROT4_DUTY 0.60f
#define GAIT_POLICY_TROT4_PATH_SCALE 0.70f
#define GAIT_POLICY_TROT4_WEIGHT_SHIFT_DEG 1.0f
#define GAIT_POLICY_TROT4_FOLD_J2_DEG 70.0f
#define GAIT_POLICY_TROT4_FOLD_J3_DEG 95.0f
#define GAIT_POLICY_TROT4_FR_J1_BIAS_DEG (-2.0f)

#define GAIT_POLICY_TURN_PERIOD_MS 2200U
#define GAIT_POLICY_TURN_MIN_PERIOD_MS 1800U
#define GAIT_POLICY_TURN_MAX_PERIOD_MS 2400U

#define GAIT_POLICY_CRAB_PERIOD_MS 4000U
#define GAIT_POLICY_CRAB_MIN_PERIOD_MS 3000U
#define GAIT_POLICY_CRAB_MAX_PERIOD_MS 5000U
#define GAIT_POLICY_CRAB_DUTY 0.80f
#define GAIT_POLICY_CRAB_J1_AMPLITUDE_DEG 2.0f
#define GAIT_POLICY_CRAB_LIFT_HEIGHT 0.14f

#define GAIT_POLICY_JUMP_PERIOD_MS 1200U
#define GAIT_POLICY_JUMP_CONTROL_HZ 50U
#define GAIT_POLICY_JUMP_J1_DEG 4.0f
#define GAIT_POLICY_JUMP_FORWARD_LIMIT 0.30f

#define GAIT_POLICY_PI 3.14159265358979323846f

/* CAD 300mm-frame search candidate 159, revalidated at 11.1V 3S.
 * Geometry is simulator-derived, not a hardware calibration. */
#define GAIT_POLICY_TROT5_PERIOD_MS 844U
#define GAIT_POLICY_TROT5_MAX_PERIOD_MS 2400U
#define GAIT_POLICY_TROT5_DUTY 0.5979488437760033f
#define GAIT_POLICY_TROT5_STRIDE_M 0.06280689761866355f
#define GAIT_POLICY_TROT5_LIFT_M 0.012f
#define GAIT_POLICY_TROT5_HEIGHT_M 0.20175228283900074f
#define GAIT_POLICY_TROT5_FORWARD_M (-0.035f)
#define GAIT_POLICY_TROT5_SPREAD_DEG 0.7411947428343884f

/*
 * Which way a foot travels along the body while it carries weight: rearward,
 * so the body is pushed forward.
 *
 * Not a per-leg parameter.  It used to be one -- callers passed an array of
 * forward signs -- on the assumption that the front and rear mechanisms were
 * mirror images and therefore needed opposite directions.  They are not; the
 * four legs are identical, so they all push the same way, and the array only
 * ever encoded how a leg was mounted.
 *
 * That mattered beyond tidiness.  A mounting fact in the policy's argument
 * list is a mounting fact the simulator has to override to match its own URDF,
 * which is exactly what walk.py did, and it left the simulator unable to
 * answer questions about the very thing it was being consulted for.  Mounting
 * now lives entirely in the calibration, where centre and direction already
 * describe each servo.
 */
#define GAIT_POLICY_STANCE_TRAVEL (-1.0f)

typedef struct
{
    float j1_deg;
    float j2_deg;
    float j3_deg;
    bool stance;
} GaitPolicyLegTarget;

typedef struct
{
    float roll;
    float pitch;
    float roll_rate;
    float pitch_rate;
} GaitPolicyImuSample;

typedef struct
{
    float kp;
    float kd;
    float leg_length_limit;
    float j1_gain_deg;
    float j1_limit_deg;
    float foot_placement_gain;
    float foot_placement_limit;
    bool contact_aware;
} GaitPolicyBalanceConfig;

typedef struct
{
    float roll_control;
    float pitch_control;
    float j1_correction_deg[GAIT_POLICY_LEG_COUNT];
    float down_correction[GAIT_POLICY_LEG_COUNT];
    float forward_correction[GAIT_POLICY_LEG_COUNT];
} GaitPolicyBalancePreview;

static inline float gait_policy_clampf(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static inline float gait_policy_wrap_phase(float phase)
{
    phase = fmodf(phase, 1.0f);
    return phase < 0.0f ? phase + 1.0f : phase;
}

static inline float gait_policy_cosine_ease(float progress)
{
    progress = gait_policy_clampf(progress, 0.0f, 1.0f);
    return 0.5f - 0.5f * cosf(GAIT_POLICY_PI * progress);
}

static inline float gait_policy_smootherstep(float progress)
{
    const float x = gait_policy_clampf(progress, 0.0f, 1.0f);
    if (x <= 0.0f) {
        return 0.0f;
    }
    if (x >= 1.0f) {
        return 1.0f;
    }
    if (x <= 0.5f) {
        return x * x * x * (x * (x * 6.0f - 15.0f) + 10.0f);
    }
    const float mirrored = 1.0f - x;
    const float tail = mirrored * mirrored * mirrored *
        (mirrored * (mirrored * 6.0f - 15.0f) + 10.0f);
    return 1.0f - tail;
}

static inline void gait_policy_leg_forward_kinematics(float upper_deg,
                                                       float knee_deg,
                                                       float *forward,
                                                       float *down)
{
    const float upper = upper_deg * GAIT_POLICY_PI / 180.0f;
    const float lower = (upper_deg - knee_deg) * GAIT_POLICY_PI / 180.0f;
    if (forward != NULL) {
        *forward = sinf(upper) + sinf(lower);
    }
    if (down != NULL) {
        *down = cosf(upper) + cosf(lower);
    }
}

static inline bool gait_policy_leg_inverse_kinematics(float forward,
                                                       float down,
                                                       float *upper_deg,
                                                       float *knee_deg)
{
    const float radius_squared = forward * forward + down * down;
    const float cosine_knee = (radius_squared - 2.0f) / 2.0f;
    if (cosine_knee < -1.0001f || cosine_knee > 1.0001f ||
        upper_deg == NULL || knee_deg == NULL) {
        return false;
    }
    const float knee = acosf(gait_policy_clampf(cosine_knee, -1.0f, 1.0f));
    const float upper = atan2f(forward, down) +
                        atan2f(sinf(knee), 1.0f + cosf(knee));
    *upper_deg = upper * 180.0f / GAIT_POLICY_PI;
    *knee_deg = knee * 180.0f / GAIT_POLICY_PI;
    return isfinite(*upper_deg) && isfinite(*knee_deg);
}

static inline bool gait_policy_trot_targets(
    float global_phase,
    float amplitude_scale,
    float travel_scale,
    GaitPolicyLegTarget targets[GAIT_POLICY_LEG_COUNT])
{
    static const float phase_offsets[GAIT_POLICY_LEG_COUNT] = {
        0.0f, 0.5f, 0.5f, 0.0f
    };
    float base_forward = 0.0f;
    float base_down = 0.0f;
    float lifted_down = 0.0f;

    if (targets == NULL ||
        !isfinite(global_phase) || !isfinite(amplitude_scale) ||
        !isfinite(travel_scale) ||
        amplitude_scale < 0.0f || amplitude_scale > 1.0f ||
        travel_scale < -1.0f || travel_scale > 1.0f) {
        return false;
    }

    gait_policy_leg_forward_kinematics(
        GAIT_POLICY_SIM_TROT_STANCE_J2_DEG,
        GAIT_POLICY_SIM_TROT_STANCE_J3_DEG,
        &base_forward,
        &base_down);
    gait_policy_leg_forward_kinematics(
        GAIT_POLICY_SIM_TROT_STANCE_J2_DEG,
        GAIT_POLICY_SIM_TROT_STANCE_J3_DEG +
            GAIT_POLICY_SIM_TROT_LIFT_AMPLITUDE_DEG * amplitude_scale,
        NULL,
        &lifted_down);

    const float stride = base_down * sinf(
        GAIT_POLICY_SIM_TROT_HIP_AMPLITUDE_DEG * GAIT_POLICY_PI / 180.0f);
    const float lift_height = fmaxf(0.0f, base_down - lifted_down);

    for (uint8_t leg = 0U; leg < GAIT_POLICY_LEG_COUNT; ++leg) {
        const float local_phase = gait_policy_wrap_phase(
            global_phase + phase_offsets[leg]);
        float forward_wave = 0.0f;
        float lift_wave = 0.0f;
        const bool stance = local_phase < GAIT_POLICY_SIM_TROT_DUTY;

        if (stance) {
            const float progress = local_phase / GAIT_POLICY_SIM_TROT_DUTY;
            forward_wave = 1.0f -
                           2.0f * gait_policy_cosine_ease(progress);
        } else {
            const float progress =
                (local_phase - GAIT_POLICY_SIM_TROT_DUTY) /
                (1.0f - GAIT_POLICY_SIM_TROT_DUTY);
            if (progress < 0.20f) {
                forward_wave = -1.0f;
                lift_wave = gait_policy_cosine_ease(progress / 0.20f);
            } else if (progress < 0.80f) {
                forward_wave = -1.0f + 2.0f * gait_policy_cosine_ease(
                    (progress - 0.20f) / 0.60f);
                lift_wave = 1.0f;
            } else {
                forward_wave = 1.0f;
                lift_wave = 1.0f - gait_policy_cosine_ease(
                    (progress - 0.80f) / 0.20f);
            }
        }

        const float foot_forward = base_forward +
            GAIT_POLICY_STANCE_TRAVEL * stride * forward_wave *
            amplitude_scale * travel_scale;
        const float foot_down = base_down - lift_height * lift_wave;
        float upper = 0.0f;
        float knee = 0.0f;
        if (!gait_policy_leg_inverse_kinematics(
                foot_forward, foot_down, &upper, &knee)) {
            return false;
        }

        targets[leg].j1_deg = GAIT_POLICY_SIM_TROT_STANCE_J1_DEG;
        targets[leg].j2_deg = upper;
        targets[leg].j3_deg = knee;
        targets[leg].stance = stance;
    }
    return true;
}

static inline bool gait_policy_sim_trot_targets(
    float global_phase,
    float amplitude_scale,
    GaitPolicyLegTarget targets[GAIT_POLICY_LEG_COUNT])
{
    return gait_policy_trot_targets(
        global_phase, amplitude_scale, 1.0f, targets);
}

/*
 * Circular-foot diagonal trot.
 *
 * During stance the toe moves front-to-rear along the ground. During swing
 * it follows the upper semicircle from rear toe-off to front touchdown. The
 * circle apex is derived from the requested folded J2/J3 pose, so the upper
 * link approaches body-horizontal and both joints remain coupled by IK.
 */
static inline bool gait_policy_circular_trot_targets(
    float global_phase,
    float amplitude_scale,
    float fold_j2_deg,
    float fold_j3_deg,
    float duty_factor,
    GaitPolicyLegTarget targets[GAIT_POLICY_LEG_COUNT])
{
    static const float phase_offsets[GAIT_POLICY_LEG_COUNT] = {
        0.0f, 0.5f, 0.5f, 0.0f
    };
    float base_forward = 0.0f;
    float ground_down = 0.0f;
    float folded_forward = 0.0f;
    float folded_down = 0.0f;

    if (targets == NULL ||
        !isfinite(global_phase) || !isfinite(amplitude_scale) ||
        !isfinite(fold_j2_deg) || !isfinite(fold_j3_deg) ||
        !isfinite(duty_factor) ||
        amplitude_scale < 0.0f || amplitude_scale > 1.0f ||
        fold_j2_deg < 60.0f || fold_j2_deg > 95.0f ||
        fold_j3_deg < 80.0f || fold_j3_deg > 145.0f ||
        duty_factor < 0.50f || duty_factor > 0.80f) {
        return false;
    }

    gait_policy_leg_forward_kinematics(
        GAIT_POLICY_SIM_TROT_STANCE_J2_DEG,
        GAIT_POLICY_SIM_TROT_STANCE_J3_DEG,
        &base_forward,
        &ground_down);
    gait_policy_leg_forward_kinematics(
        fold_j2_deg,
        fold_j3_deg,
        &folded_forward,
        &folded_down);
    const float radius = ground_down - folded_down;
    if (!isfinite(radius) || radius <= 0.0f) {
        return false;
    }

    for (uint8_t leg = 0U; leg < GAIT_POLICY_LEG_COUNT; ++leg) {
        const float local_phase = gait_policy_wrap_phase(
            global_phase + phase_offsets[leg]);
        const bool stance = local_phase < duty_factor;
        const float progress = stance ?
            local_phase / duty_factor :
            (local_phase - duty_factor) / (1.0f - duty_factor);
        const float angle = GAIT_POLICY_PI *
                            gait_policy_cosine_ease(progress);
        const float direction = GAIT_POLICY_STANCE_TRAVEL;
        const float circle_center =
            (folded_forward - base_forward) / direction;
        float travel = circle_center;
        float curve_down = ground_down;
        if (stance) {
            travel += radius * cosf(angle);
        } else {
            travel -= radius * cosf(angle);
            curve_down -= radius * sinf(angle);
        }

        const float curve_forward = base_forward + direction * travel;
        const float foot_forward = base_forward + amplitude_scale *
            (curve_forward - base_forward);
        const float foot_down = ground_down + amplitude_scale *
            (curve_down - ground_down);
        if (!gait_policy_leg_inverse_kinematics(
                foot_forward,
                foot_down,
                &targets[leg].j2_deg,
                &targets[leg].j3_deg)) {
            return false;
        }
        targets[leg].j1_deg = GAIT_POLICY_SIM_TROT_STANCE_J1_DEG;
        targets[leg].stance = stance;
    }
    return true;
}

/* Trot4 variant: smootherstep makes velocity and acceleration both meet the
 * neighbouring segment at zero, unlike the cosine-shaped phase used by the
 * original circular gait. */
static inline bool gait_policy_smooth_circular_trot_targets(
    float global_phase,
    float amplitude_scale,
    float fold_j2_deg,
    float fold_j3_deg,
    float duty_factor,
    GaitPolicyLegTarget targets[GAIT_POLICY_LEG_COUNT])
{
    static const float phase_offsets[GAIT_POLICY_LEG_COUNT] = {
        0.0f, 0.5f, 0.5f, 0.0f
    };
    float base_forward = 0.0f;
    float ground_down = 0.0f;
    float folded_forward = 0.0f;
    float folded_down = 0.0f;

    if (targets == NULL ||
        !isfinite(global_phase) || !isfinite(amplitude_scale) ||
        !isfinite(fold_j2_deg) || !isfinite(fold_j3_deg) ||
        !isfinite(duty_factor) ||
        amplitude_scale < 0.0f || amplitude_scale > 1.0f ||
        fold_j2_deg < 60.0f || fold_j2_deg > 95.0f ||
        fold_j3_deg < 80.0f || fold_j3_deg > 145.0f ||
        duty_factor < 0.50f || duty_factor > 0.80f) {
        return false;
    }

    gait_policy_leg_forward_kinematics(
        GAIT_POLICY_SIM_TROT_STANCE_J2_DEG,
        GAIT_POLICY_SIM_TROT_STANCE_J3_DEG,
        &base_forward,
        &ground_down);
    gait_policy_leg_forward_kinematics(
        fold_j2_deg, fold_j3_deg, &folded_forward, &folded_down);
    const float radius = ground_down - folded_down;
    if (!isfinite(radius) || radius <= 0.0f) {
        return false;
    }

    for (uint8_t leg = 0U; leg < GAIT_POLICY_LEG_COUNT; ++leg) {
        const float local_phase = gait_policy_wrap_phase(
            global_phase + phase_offsets[leg]);
        const bool stance = local_phase < duty_factor;
        const float progress = stance ?
            local_phase / duty_factor :
            (local_phase - duty_factor) / (1.0f - duty_factor);
        const float angle = GAIT_POLICY_PI * gait_policy_smootherstep(progress);
        const float direction = GAIT_POLICY_STANCE_TRAVEL;
        const float circle_center =
            (folded_forward - base_forward) / direction;
        float travel = circle_center;
        float curve_down = ground_down;
        if (stance) {
            travel += radius * cosf(angle);
        } else {
            travel -= radius * cosf(angle);
            curve_down -= radius * sinf(angle);
        }

        const float curve_forward = base_forward + direction * travel;
        const float foot_forward = base_forward + amplitude_scale *
            (curve_forward - base_forward);
        const float foot_down = ground_down + amplitude_scale *
            (curve_down - ground_down);
        if (!gait_policy_leg_inverse_kinematics(
                foot_forward,
                foot_down,
                &targets[leg].j2_deg,
                &targets[leg].j3_deg)) {
            return false;
        }
        targets[leg].j1_deg = GAIT_POLICY_SIM_TROT_STANCE_J1_DEG;
        targets[leg].stance = stance;
    }
    return true;
}

static inline bool gait_policy_trot2_targets(
    float global_phase,
    float amplitude_scale,
    float fold_j2_deg,
    float fold_j3_deg,
    GaitPolicyLegTarget targets[GAIT_POLICY_LEG_COUNT])
{
    return gait_policy_circular_trot_targets(
        global_phase,
        amplitude_scale,
        fold_j2_deg,
        fold_j3_deg,
        GAIT_POLICY_TROT2_DUTY,
        targets);
}

/* Circular-foot gait with four-foot overlap for slow physical starts. */
static inline bool gait_policy_trot3_targets(
    float global_phase,
    float amplitude_scale,
    float fold_j2_deg,
    float fold_j3_deg,
    GaitPolicyLegTarget targets[GAIT_POLICY_LEG_COUNT])
{
    if (!gait_policy_circular_trot_targets(
        global_phase,
        amplitude_scale,
        fold_j2_deg,
        fold_j3_deg,
        GAIT_POLICY_TROT3_DUTY,
        targets)) {
        return false;
    }

    /*
     * Move load toward the next support diagonal while all four feet are
     * scheduled down.  Positive selects FL+RR; negative selects FR+RL.  The
     * transition occupies only the duty overlap, then remains constant while
     * the opposite diagonal swings.  This is canonical body geometry, not a
     * servo mounting correction.
     */
    static const int8_t diagonal_signs[GAIT_POLICY_LEG_COUNT] = {
        1, -1, -1, 1
    };
    const float phase = gait_policy_wrap_phase(global_phase);
    const float overlap = GAIT_POLICY_TROT3_DUTY - 0.5f;
    float transfer = -1.0f;
    if (phase < overlap) {
        transfer = -1.0f + 2.0f * gait_policy_cosine_ease(phase / overlap);
    } else if (phase < 0.5f) {
        transfer = 1.0f;
    } else if (phase < 0.5f + overlap) {
        transfer = 1.0f - 2.0f * gait_policy_cosine_ease(
            (phase - 0.5f) / overlap);
    }

    for (uint8_t leg = 0U; leg < GAIT_POLICY_LEG_COUNT; ++leg) {
        targets[leg].j1_deg +=
            (float)diagonal_signs[leg] *
            GAIT_POLICY_TROT3_WEIGHT_SHIFT_DEG * transfer * amplitude_scale;
    }
    return true;
}

static inline bool gait_policy_trot5_targets(
    float global_phase,
    float amplitude_scale,
    GaitPolicyLegTarget targets[GAIT_POLICY_LEG_COUNT])
{
    if (targets == NULL || !isfinite(global_phase) ||
        !isfinite(amplitude_scale) || amplitude_scale < 0.0f ||
        amplitude_scale > 1.0f) {
        return false;
    }
    static const float offsets[GAIT_POLICY_LEG_COUNT] = {0, .5f, .5f, 0};
    for (uint8_t leg = 0; leg < GAIT_POLICY_LEG_COUNT; ++leg) {
        const float phase = gait_policy_wrap_phase(global_phase + offsets[leg]);
        float forward;
        float down = GAIT_POLICY_TROT5_HEIGHT_M;
        const bool stance = phase < GAIT_POLICY_TROT5_DUTY;
        if (stance) {
            forward = GAIT_POLICY_TROT5_STRIDE_M *
                (.5f - phase / GAIT_POLICY_TROT5_DUTY);
        } else {
            const float s = (phase - GAIT_POLICY_TROT5_DUTY) /
                (1.0f - GAIT_POLICY_TROT5_DUTY);
            const float k = (1.0f - GAIT_POLICY_TROT5_DUTY) /
                GAIT_POLICY_TROT5_DUTY;
            forward = GAIT_POLICY_TROT5_STRIDE_M *
                (-.5f + (1.0f + k) * gait_policy_smootherstep(s) - k * s);
            const float u = s * (1.0f - s);
            down -= GAIT_POLICY_TROT5_LIFT_M * 64.0f * u * u * u;
        }
        forward = GAIT_POLICY_TROT5_FORWARD_M + amplitude_scale * forward;
        down = GAIT_POLICY_TROT5_HEIGHT_M +
            amplitude_scale * (down - GAIT_POLICY_TROT5_HEIGHT_M);
        const float l2 = .141f;
        const float l3 = .150f;
        const float cosine = (forward * forward + down * down -
            l2 * l2 - l3 * l3) / (2.0f * l2 * l3);
        if (cosine < -1.0f || cosine > 1.0f) {
            return false;
        }
        const float knee = acosf(cosine);
        const float hip = atan2f(-forward, down) +
            atan2f(l3 * sinf(knee), l2 + l3 * cosf(knee));
        targets[leg].j1_deg = GAIT_POLICY_TROT5_SPREAD_DEG;
        targets[leg].j2_deg = hip * 180.0f / GAIT_POLICY_PI;
        targets[leg].j3_deg = knee * 180.0f / GAIT_POLICY_PI;
        targets[leg].stance = amplitude_scale == 0.0f || stance;
        if (targets[leg].j2_deg < -45.0f || targets[leg].j2_deg > 100.0f ||
            targets[leg].j3_deg < 0.0f || targets[leg].j3_deg > 150.0f) {
            return false;
        }
    }
    return true;
}

static inline bool gait_policy_trot4_targets(
    float global_phase,
    float amplitude_scale,
    GaitPolicyLegTarget targets[GAIT_POLICY_LEG_COUNT])
{
    if (!gait_policy_smooth_circular_trot_targets(
            global_phase,
            amplitude_scale * GAIT_POLICY_TROT4_PATH_SCALE,
            GAIT_POLICY_TROT4_FOLD_J2_DEG,
            GAIT_POLICY_TROT4_FOLD_J3_DEG,
            GAIT_POLICY_TROT4_DUTY,
            targets)) {
        return false;
    }

    static const int8_t diagonal_signs[GAIT_POLICY_LEG_COUNT] = {
        1, -1, -1, 1
    };
    const float phase = gait_policy_wrap_phase(global_phase);
    const float overlap = GAIT_POLICY_TROT4_DUTY - 0.5f;
    float transfer = -1.0f;
    if (phase < overlap) {
        transfer = -1.0f + 2.0f * gait_policy_smootherstep(phase / overlap);
    } else if (phase < 0.5f) {
        transfer = 1.0f;
    } else if (phase < 0.5f + overlap) {
        transfer = 1.0f - 2.0f * gait_policy_smootherstep(
            (phase - 0.5f) / overlap);
    }

    for (uint8_t leg = 0U; leg < GAIT_POLICY_LEG_COUNT; ++leg) {
        /* The circular helper's 4-degree J1 stance belongs to the simulator
         * gait.  On the physical robot it rotates the front and rear hip
         * mechanisms in visibly different directions, even at zero path
         * amplitude.  Trot4 starts from the calibrated straight J1 pose and
         * adds only its deliberately small diagonal load transfer. */
        const float fr_outward_bias = leg == 1U ?
            GAIT_POLICY_TROT4_FR_J1_BIAS_DEG * amplitude_scale : 0.0f;
        targets[leg].j1_deg = fr_outward_bias +
            (float)diagonal_signs[leg] *
            GAIT_POLICY_TROT4_WEIGHT_SHIFT_DEG * transfer * amplitude_scale;
    }
    return true;
}

/* Apply the same actuator-feasible trot4 path in either longitudinal
 * direction. Forward is the established trajectory; backward reflects every
 * sagittal foot target around the calibrated stand point while preserving
 * lift, support timing, J1 load transfer and the FR mounting correction. */
static inline bool gait_policy_trot4_direction_targets(
    float global_phase,
    float amplitude_scale,
    int8_t direction,
    GaitPolicyLegTarget targets[GAIT_POLICY_LEG_COUNT])
{
    float base_forward = 0.0f;

    if ((direction != -1 && direction != 1) ||
        !gait_policy_trot4_targets(global_phase, amplitude_scale, targets)) {
        return false;
    }
    if (direction > 0) {
        return true;
    }

    gait_policy_leg_forward_kinematics(
        GAIT_POLICY_SIM_TROT_STANCE_J2_DEG,
        GAIT_POLICY_SIM_TROT_STANCE_J3_DEG,
        &base_forward,
        NULL);
    for (uint8_t leg = 0U; leg < GAIT_POLICY_LEG_COUNT; ++leg) {
        float foot_forward = 0.0f;
        float foot_down = 0.0f;
        gait_policy_leg_forward_kinematics(
            targets[leg].j2_deg,
            targets[leg].j3_deg,
            &foot_forward,
            &foot_down);
        foot_forward = 2.0f * base_forward - foot_forward;
        if (!gait_policy_leg_inverse_kinematics(
                foot_forward,
                foot_down,
                &targets[leg].j2_deg,
                &targets[leg].j3_deg)) {
            return false;
        }
    }
    return true;
}

/*
 * Differential trot turn. direction=+1 turns left and -1 turns right.
 * The side on the outside of the turn keeps the trot4 fore/aft trajectory;
 * the inside side mirrors it around the calibrated stance point. Opposite
 * longitudinal ground reactions then create yaw without using the J1 lateral
 * sweep that belongs to the experimental crab gait.
 */
static inline bool gait_policy_turn_targets(
    float global_phase,
    float amplitude_scale,
    int8_t direction,
    GaitPolicyLegTarget targets[GAIT_POLICY_LEG_COUNT])
{
    float base_forward = 0.0f;

    if ((direction != -1 && direction != 1) ||
        !gait_policy_trot4_targets(global_phase, amplitude_scale, targets)) {
        return false;
    }
    gait_policy_leg_forward_kinematics(
        GAIT_POLICY_SIM_TROT_STANCE_J2_DEG,
        GAIT_POLICY_SIM_TROT_STANCE_J3_DEG,
        &base_forward,
        NULL);

    for (uint8_t leg = 0U; leg < GAIT_POLICY_LEG_COUNT; ++leg) {
        float foot_forward = 0.0f;
        float foot_down = 0.0f;
        gait_policy_leg_forward_kinematics(
            targets[leg].j2_deg,
            targets[leg].j3_deg,
            &foot_forward,
            &foot_down);
        const bool left_side = leg == 0U || leg == 2U;
        const float side_direction = left_side ?
            -(float)direction : (float)direction;
        foot_forward = base_forward +
            side_direction * (foot_forward - base_forward);
        if (!gait_policy_leg_inverse_kinematics(
                foot_forward,
                foot_down,
                &targets[leg].j2_deg,
                &targets[leg].j3_deg)) {
            return false;
        }
    }
    return true;
}

/*
 * Continuous remote-control gait.  Longitudinal and yaw inputs are normalized
 * to [-1, 1] and share one motion budget, so a diagonal joystick command
 * cannot ask the actuators for two full-amplitude trajectories at once.
 *
 * Both component trajectories use trot4's phase and support schedule.  Their
 * offsets from the calibrated standing pose can therefore be blended without
 * changing support legs or resetting phase when the joystick crosses from a
 * straight walk into a turn.
 */
/* Conservative joystick turn limit, applied BEFORE input slew.  Full-scale
 * forward/backward is unchanged. This reduces turn-to-straight excitation;
 * it is not a contact estimator or a guarantee against falling. */
static inline int16_t gait_policy_drive_yaw_limit(int16_t requested)
{
    if (requested > 500) return 500;
    if (requested < -500) return -500;
    return requested;
}

static inline bool gait_policy_drive_stride_targets(
    float global_phase,
    float startup_scale,
    float linear,
    float yaw,
    float forward_stride,
    GaitPolicyLegTarget targets[GAIT_POLICY_LEG_COUNT])
{
    GaitPolicyLegTarget base[GAIT_POLICY_LEG_COUNT];
    GaitPolicyLegTarget linear_targets[GAIT_POLICY_LEG_COUNT];
    GaitPolicyLegTarget yaw_targets[GAIT_POLICY_LEG_COUNT];

    if (!isfinite(forward_stride) || forward_stride < 1.0f || forward_stride > 2.0f ||
        targets == NULL || !isfinite(global_phase) ||
        !isfinite(startup_scale) || !isfinite(linear) || !isfinite(yaw) ||
        startup_scale < 0.0f || startup_scale > 1.0f ||
        linear < -1.0f || linear > 1.0f || yaw < -1.0f || yaw > 1.0f) {
        return false;
    }

    float linear_weight = fabsf(linear);
    float yaw_weight = fabsf(yaw);
    const float requested = linear_weight + yaw_weight;
    if (requested > 1.0f) {
        linear_weight /= requested;
        yaw_weight /= requested;
    }
    linear_weight *= startup_scale;
    yaw_weight *= startup_scale;

    if (!gait_policy_trot4_targets(global_phase, 0.0f, base) ||
        !gait_policy_trot4_direction_targets(
            global_phase,
            linear_weight,
            linear < 0.0f ? -1 : 1,
            linear_targets) ||
        !gait_policy_turn_targets(
            global_phase,
            yaw_weight,
            yaw < 0.0f ? 1 : -1,
            yaw_targets)) {
        return false;
    }

    /* Extend only the fore/aft excursion of forward travel, about its
     * existing path center. Foot lift, path center, backward and turn stay
     * unchanged. Apply before mixing the longitudinal and yaw components. */
    if (linear > 0.0f && forward_stride != 1.0f) {
        float bx, fx;
        gait_policy_leg_forward_kinematics(
            GAIT_POLICY_SIM_TROT_STANCE_J2_DEG,
            GAIT_POLICY_SIM_TROT_STANCE_J3_DEG, &bx, NULL);
        gait_policy_leg_forward_kinematics(
            GAIT_POLICY_TROT4_FOLD_J2_DEG,
            GAIT_POLICY_TROT4_FOLD_J3_DEG, &fx, NULL);
        const float center = bx + linear_weight * GAIT_POLICY_TROT4_PATH_SCALE * (fx - bx);
        for (uint8_t leg = 0U; leg < GAIT_POLICY_LEG_COUNT; ++leg) {
            float x, z;
            gait_policy_leg_forward_kinematics(
                linear_targets[leg].j2_deg, linear_targets[leg].j3_deg, &x, &z);
            if (!gait_policy_leg_inverse_kinematics(
                    center + forward_stride * (x - center), z,
                    &linear_targets[leg].j2_deg, &linear_targets[leg].j3_deg)) return false;
        }
    }

    for (uint8_t leg = 0U; leg < GAIT_POLICY_LEG_COUNT; ++leg) {
        targets[leg].j1_deg = base[leg].j1_deg +
            (linear_targets[leg].j1_deg - base[leg].j1_deg) +
            (yaw_targets[leg].j1_deg - base[leg].j1_deg);
        targets[leg].j2_deg = base[leg].j2_deg +
            (linear_targets[leg].j2_deg - base[leg].j2_deg) +
            (yaw_targets[leg].j2_deg - base[leg].j2_deg);
        targets[leg].j3_deg = base[leg].j3_deg +
            (linear_targets[leg].j3_deg - base[leg].j3_deg) +
            (yaw_targets[leg].j3_deg - base[leg].j3_deg);
        targets[leg].stance = base[leg].stance;
    }
    return true;
}

/* Selected in STEP dynamics comparisons; physical validation still required. */
#define GAIT_POLICY_DRIVE_FORWARD_STRIDE 1.6f
static inline bool gait_policy_drive_targets(
    float phase, float startup, float linear, float yaw,
    GaitPolicyLegTarget targets[GAIT_POLICY_LEG_COUNT])
{
    return gait_policy_drive_stride_targets(phase, startup, linear, yaw,
                                           GAIT_POLICY_DRIVE_FORWARD_STRIDE, targets);
}

/* Keep one body height for an entire joystick session, including steering
 * and reverse, so a direction change cannot abruptly change leg extension. */
static inline bool gait_policy_drive_walk_targets(
    float phase, float startup, float linear, float yaw,
    GaitPolicyLegTarget targets[GAIT_POLICY_LEG_COUNT])
{
    if (!gait_policy_drive_targets(phase, startup, linear, yaw, targets)) return false;
    const float height_delta = 2.0f * (cosf(40.0f * GAIT_POLICY_PI / 180.0f) -
                                     cosf(45.0f * GAIT_POLICY_PI / 180.0f));
    for (uint8_t leg = 0; leg < GAIT_POLICY_LEG_COUNT; ++leg) {
        float x, z;
        gait_policy_leg_forward_kinematics(targets[leg].j2_deg, targets[leg].j3_deg, &x, &z);
        if (!gait_policy_leg_inverse_kinematics(x, z + height_delta,
                &targets[leg].j2_deg, &targets[leg].j3_deg) ||
            targets[leg].j2_deg < -45.0f || targets[leg].j2_deg > 100.0f ||
            targets[leg].j3_deg < 0.0f || targets[leg].j3_deg > 150.0f) return false;
    }
    return true;
}

/*
 * Sideways four-beat crawl. direction=+1 moves the body left and -1 right.
 * The end signs map canonical J1 angles to a common world-sideways foot
 * direction on the mirrored physical hip mechanisms.  The offsets order the
 * swing legs FL, RR, FR, RL with a four-foot overlap between them, so three
 * legs always support the body.  During stance all feet travel opposite the
 * requested body direction; the single swing leg lifts vertically and
 * returns without adding fore/aft motion.
 */
static inline bool gait_policy_crab_targets(
    float global_phase,
    float amplitude_scale,
    int8_t direction,
    GaitPolicyLegTarget targets[GAIT_POLICY_LEG_COUNT])
{
    static const float phase_offsets[GAIT_POLICY_LEG_COUNT] = {
        0.80f, 0.30f, 0.05f, 0.55f
    };
    static const int8_t lateral_signs[GAIT_POLICY_LEG_COUNT] = {
        1, 1, -1, -1
    };
    float base_forward = 0.0f;
    float base_down = 0.0f;

    if (targets == NULL || !isfinite(global_phase) ||
        !isfinite(amplitude_scale) || amplitude_scale < 0.0f ||
        amplitude_scale > 1.0f || (direction != -1 && direction != 1)) {
        return false;
    }
    gait_policy_leg_forward_kinematics(
        GAIT_POLICY_SIM_TROT_STANCE_J2_DEG,
        GAIT_POLICY_SIM_TROT_STANCE_J3_DEG,
        &base_forward,
        &base_down);

    for (uint8_t leg = 0U; leg < GAIT_POLICY_LEG_COUNT; ++leg) {
        const float local_phase = gait_policy_wrap_phase(
            global_phase + phase_offsets[leg]);
        const bool stance = local_phase < GAIT_POLICY_CRAB_DUTY;
        const float progress = stance ?
            local_phase / GAIT_POLICY_CRAB_DUTY :
            (local_phase - GAIT_POLICY_CRAB_DUTY) /
                (1.0f - GAIT_POLICY_CRAB_DUTY);
        const float lateral_wave = stance ?
            1.0f - 2.0f * gait_policy_smootherstep(progress) :
            -1.0f + 2.0f * gait_policy_smootherstep(progress);
        float lift_wave = 0.0f;
        if (!stance) {
            lift_wave = progress < 0.5f ?
                gait_policy_smootherstep(progress * 2.0f) :
                gait_policy_smootherstep((1.0f - progress) * 2.0f);
        }
        const float foot_down = base_down - amplitude_scale *
            GAIT_POLICY_CRAB_LIFT_HEIGHT * lift_wave;
        if (!gait_policy_leg_inverse_kinematics(
                base_forward,
                foot_down,
                &targets[leg].j2_deg,
                &targets[leg].j3_deg)) {
            return false;
        }
        targets[leg].j1_deg =
            (float)(direction * lateral_signs[leg]) *
            GAIT_POLICY_CRAB_J1_AMPLITUDE_DEG * lateral_wave *
            amplitude_scale;
        targets[leg].stance = stance;
    }
    return true;
}

static inline float gait_policy_lerp(float start, float end, float progress)
{
    return start + (end - start) * gait_policy_smootherstep(progress);
}

/*
 * Repeating jump trajectory in normalized two-link leg coordinates.
 *
 * The in-place STM32 command passes forward_travel=0.  A later forward-jump
 * command can reuse the same vertical timing by supplying a small positive
 * travel value; the trajectory is body-forward for every leg, since each
 * mirrored physical leg.
 *
 * Phase waypoints:
 *   0.00 stand
 *   0.30 crouch/compress
 *   0.42 full extension/takeoff
 *   0.55 airborne tuck
 *   0.68 landing extension
 *   0.82 impact absorption
 *   1.00 stand
 */
static inline bool gait_policy_jump_targets(
    float global_phase,
    float forward_travel,
    GaitPolicyLegTarget targets[GAIT_POLICY_LEG_COUNT])
{
    static const float waypoint_phase[7] = {
        0.00f, 0.30f, 0.42f, 0.55f, 0.68f, 0.82f, 1.00f
    };
    static const float waypoint_j2_deg[7] = {
        45.0f, 40.0f, 22.0f, 40.0f, 35.0f, 40.0f, 45.0f
    };
    static const float waypoint_j3_deg[7] = {
        90.0f, 130.0f, 44.0f, 130.0f, 100.0f, 125.0f, 90.0f
    };
    static const float waypoint_forward_scale[7] = {
        0.0f, 0.2f, -1.0f, 1.0f, 1.0f, 0.5f, 0.0f
    };

    if (targets == NULL ||
        !isfinite(global_phase) || !isfinite(forward_travel) ||
        fabsf(forward_travel) > GAIT_POLICY_JUMP_FORWARD_LIMIT) {
        return false;
    }

    const float phase = gait_policy_wrap_phase(global_phase);
    uint8_t segment = 0U;
    while (segment < 5U && phase >= waypoint_phase[segment + 1U]) {
        ++segment;
    }
    const float segment_start = waypoint_phase[segment];
    const float segment_end = waypoint_phase[segment + 1U];
    const float progress = (phase - segment_start) /
                           (segment_end - segment_start);

    float start_forward = 0.0f;
    float start_down = 0.0f;
    float end_forward = 0.0f;
    float end_down = 0.0f;
    gait_policy_leg_forward_kinematics(
        waypoint_j2_deg[segment], waypoint_j3_deg[segment],
        &start_forward, &start_down);
    gait_policy_leg_forward_kinematics(
        waypoint_j2_deg[segment + 1U], waypoint_j3_deg[segment + 1U],
        &end_forward, &end_down);

    const float base_forward = gait_policy_lerp(
        start_forward, end_forward, progress);
    const float forward_offset = gait_policy_lerp(
        waypoint_forward_scale[segment] * forward_travel,
        waypoint_forward_scale[segment + 1U] * forward_travel,
        progress);
    const float down = gait_policy_lerp(start_down, end_down, progress);

    float j1_scale = 1.0f;
    if (phase < waypoint_phase[1]) {
        j1_scale = gait_policy_smootherstep(phase / waypoint_phase[1]);
    } else if (phase >= waypoint_phase[5]) {
        j1_scale = 1.0f - gait_policy_smootherstep(
            (phase - waypoint_phase[5]) /
            (waypoint_phase[6] - waypoint_phase[5]));
    }

    for (uint8_t leg = 0U; leg < GAIT_POLICY_LEG_COUNT; ++leg) {
        if (!gait_policy_leg_inverse_kinematics(
                base_forward + GAIT_POLICY_STANCE_TRAVEL * forward_offset,
                down,
                &targets[leg].j2_deg,
                &targets[leg].j3_deg)) {
            return false;
        }
        targets[leg].j1_deg = GAIT_POLICY_JUMP_J1_DEG * j1_scale;
        targets[leg].stance = phase < waypoint_phase[2] ||
                              phase >= waypoint_phase[4];
    }
    return true;
}

static inline uint8_t gait_policy_support_mask(
    const GaitPolicyLegTarget targets[GAIT_POLICY_LEG_COUNT])
{
    uint8_t mask = 0U;
    if (targets == NULL) {
        return mask;
    }
    for (uint8_t leg = 0U; leg < GAIT_POLICY_LEG_COUNT; ++leg) {
        if (targets[leg].stance) {
            mask |= (uint8_t)(1U << leg);
        }
    }
    return mask;
}

static inline bool gait_policy_balance_preview(
    const GaitPolicyImuSample *sample,
    const GaitPolicyBalanceConfig *config,
    uint8_t support_mask,
    GaitPolicyBalancePreview *preview)
{
    static const int8_t side_signs[GAIT_POLICY_LEG_COUNT] = {1, -1, 1, -1};
    static const int8_t end_signs[GAIT_POLICY_LEG_COUNT] = {1, 1, -1, -1};
    if (sample == NULL || config == NULL ||
        preview == NULL || !isfinite(sample->roll) ||
        !isfinite(sample->pitch) || !isfinite(sample->roll_rate) ||
        !isfinite(sample->pitch_rate)) {
        return false;
    }

    preview->roll_control =
        config->kp * sample->roll + config->kd * sample->roll_rate;
    preview->pitch_control =
        config->kp * sample->pitch + config->kd * sample->pitch_rate;

    for (uint8_t leg = 0U; leg < GAIT_POLICY_LEG_COUNT; ++leg) {
        const bool support = !config->contact_aware ||
            (support_mask & (uint8_t)(1U << leg)) != 0U;
        float j1_correction = 0.0f;
        float forward_correction = 0.0f;
        if (config->contact_aware) {
            float j1_effort =
                (float)side_signs[leg] * preview->roll_control;
            if (!support) {
                j1_effort = -j1_effort;
            }
            j1_correction = gait_policy_clampf(
                config->j1_gain_deg * j1_effort,
                -config->j1_limit_deg,
                config->j1_limit_deg);

            if (!support) {
                const float placement = gait_policy_clampf(
                    config->foot_placement_gain * preview->pitch_control,
                    -config->foot_placement_limit,
                    config->foot_placement_limit);
                /* Positive pitch means the nose is low. Place the swing foot
                 * farther forward in body coordinates to catch that fall.
                 * This correction is body-relative and therefore has the
                 * same sign during forward and backward travel. */
                forward_correction = placement;
            }
        }

        preview->j1_correction_deg[leg] = j1_correction;
        preview->forward_correction[leg] = forward_correction;
        preview->down_correction[leg] = gait_policy_clampf(
            -(float)side_signs[leg] * preview->roll_control +
             (float)end_signs[leg] * preview->pitch_control,
            -config->leg_length_limit,
            config->leg_length_limit);
    }
    return true;
}

static inline bool gait_policy_balance_targets(
    const GaitPolicyImuSample *sample,
    const GaitPolicyBalanceConfig *config,
    uint8_t support_mask,
    GaitPolicyLegTarget targets[GAIT_POLICY_LEG_COUNT])
{
    GaitPolicyBalancePreview preview;
    if (targets == NULL || !gait_policy_balance_preview(
            sample, config, support_mask, &preview)) {
        return false;
    }

    for (uint8_t leg = 0U; leg < GAIT_POLICY_LEG_COUNT; ++leg) {
        float forward = 0.0f;
        float down = 0.0f;
        gait_policy_leg_forward_kinematics(
            targets[leg].j2_deg, targets[leg].j3_deg, &forward, &down);
        targets[leg].j1_deg += preview.j1_correction_deg[leg];
        if (!gait_policy_leg_inverse_kinematics(
                forward + preview.forward_correction[leg],
                down + preview.down_correction[leg],
                &targets[leg].j2_deg,
                &targets[leg].j3_deg)) {
            return false;
        }
    }
    return true;
}

/* HAL-independent 20 ms drive command shaping. Keep the simulator and
 * embedded controller on the same integer rounding and acceleration limit. */
static inline int16_t gait_policy_drive_slew(int16_t current, int16_t target)
{
    const int32_t difference = (int32_t)target - current;
    if (difference > 40) return (int16_t)(current + 40);
    if (difference < -40) return (int16_t)(current - 40);
    return target;
}

static inline uint16_t gait_policy_drive_period_ms(int16_t linear, int16_t yaw)
{
    uint32_t magnitude = (uint32_t)(linear < 0 ? -(int32_t)linear : linear) +
        (uint32_t)(yaw < 0 ? -(int32_t)yaw : yaw);
    if (magnitude > 1000U) magnitude = 1000U;
    return (uint16_t)(2400U - (600U * magnitude) / 1000U);
}

#ifdef __cplusplus
}
#endif

#endif
