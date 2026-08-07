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

#define GAIT_POLICY_PI 3.14159265358979323846f

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

static inline bool gait_policy_sim_trot_targets(
    float global_phase,
    float amplitude_scale,
    const int8_t forward_signs[GAIT_POLICY_LEG_COUNT],
    GaitPolicyLegTarget targets[GAIT_POLICY_LEG_COUNT])
{
    static const float phase_offsets[GAIT_POLICY_LEG_COUNT] = {
        0.0f, 0.5f, 0.5f, 0.0f
    };
    float base_forward = 0.0f;
    float base_down = 0.0f;
    float lifted_down = 0.0f;

    if (forward_signs == NULL || targets == NULL ||
        !isfinite(global_phase) || !isfinite(amplitude_scale) ||
        amplitude_scale < 0.0f || amplitude_scale > 1.0f) {
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
        if (forward_signs[leg] != -1 && forward_signs[leg] != 1) {
            return false;
        }

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
            (float)forward_signs[leg] * stride * forward_wave * amplitude_scale;
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
    const int8_t forward_signs[GAIT_POLICY_LEG_COUNT],
    GaitPolicyLegTarget targets[GAIT_POLICY_LEG_COUNT])
{
    static const int8_t side_signs[GAIT_POLICY_LEG_COUNT] = {1, -1, 1, -1};
    static const int8_t end_signs[GAIT_POLICY_LEG_COUNT] = {1, 1, -1, -1};
    if (sample == NULL || config == NULL || forward_signs == NULL ||
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
                forward += (float)forward_signs[leg] * placement;
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
