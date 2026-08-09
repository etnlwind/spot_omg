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
#define GAIT_POLICY_TROT3_PERIOD_MS 1400U
#define GAIT_POLICY_TROT3_MAX_PERIOD_MS 1800U
#define GAIT_POLICY_TROT3_DUTY 0.65f
#define GAIT_POLICY_TROT3_WEIGHT_SHIFT_DEG 1.5f

#define GAIT_POLICY_JUMP_PERIOD_MS 1200U
#define GAIT_POLICY_JUMP_CONTROL_HZ 50U
#define GAIT_POLICY_JUMP_J1_DEG 4.0f
#define GAIT_POLICY_JUMP_FORWARD_LIMIT 0.30f

#define GAIT_POLICY_PI 3.14159265358979323846f

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

static inline bool gait_policy_balance_targets(
    const GaitPolicyImuSample *sample,
    const GaitPolicyBalanceConfig *config,
    uint8_t support_mask,
    GaitPolicyLegTarget targets[GAIT_POLICY_LEG_COUNT])
{
    static const int8_t side_signs[GAIT_POLICY_LEG_COUNT] = {1, -1, 1, -1};
    static const int8_t end_signs[GAIT_POLICY_LEG_COUNT] = {1, 1, -1, -1};
    if (sample == NULL || config == NULL ||
        targets == NULL || !isfinite(sample->roll) ||
        !isfinite(sample->pitch) || !isfinite(sample->roll_rate) ||
        !isfinite(sample->pitch_rate)) {
        return false;
    }

    const float roll_control =
        config->kp * sample->roll + config->kd * sample->roll_rate;
    const float pitch_control =
        config->kp * sample->pitch + config->kd * sample->pitch_rate;

    for (uint8_t leg = 0U; leg < GAIT_POLICY_LEG_COUNT; ++leg) {
        float forward = 0.0f;
        float down = 0.0f;
        gait_policy_leg_forward_kinematics(
            targets[leg].j2_deg, targets[leg].j3_deg, &forward, &down);

        const bool support = !config->contact_aware ||
            (support_mask & (uint8_t)(1U << leg)) != 0U;
        if (config->contact_aware) {
            float j1_effort = (float)side_signs[leg] * roll_control;
            if (!support) {
                j1_effort = -j1_effort;
            }
            targets[leg].j1_deg += gait_policy_clampf(
                config->j1_gain_deg * j1_effort,
                -config->j1_limit_deg,
                config->j1_limit_deg);

            if (!support) {
                const float placement = gait_policy_clampf(
                    config->foot_placement_gain * pitch_control,
                    -config->foot_placement_limit,
                    config->foot_placement_limit);
                forward += GAIT_POLICY_STANCE_TRAVEL * placement;
            }
        }

        const float down_correction = gait_policy_clampf(
            -(float)side_signs[leg] * roll_control +
             (float)end_signs[leg] * pitch_control,
            -config->leg_length_limit,
            config->leg_length_limit);
        if (!gait_policy_leg_inverse_kinematics(
                forward,
                down + down_correction,
                &targets[leg].j2_deg,
                &targets[leg].j3_deg)) {
            return false;
        }
    }
    return true;
}

#ifdef __cplusplus
}
#endif

#endif
