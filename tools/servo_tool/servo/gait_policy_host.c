#include "gait_policy.h"

#if defined(_WIN32)
#define SPOT_GAIT_EXPORT __declspec(dllexport)
#else
#define SPOT_GAIT_EXPORT __attribute__((visibility("default")))
#endif

static void unpack_targets(const float values[12],
                           GaitPolicyLegTarget targets[GAIT_POLICY_LEG_COUNT])
{
    for (uint8_t leg = 0U; leg < GAIT_POLICY_LEG_COUNT; ++leg) {
        targets[leg].j1_deg = values[leg * 3U];
        targets[leg].j2_deg = values[leg * 3U + 1U];
        targets[leg].j3_deg = values[leg * 3U + 2U];
        targets[leg].stance = false;
    }
}

static void pack_targets(const GaitPolicyLegTarget targets[GAIT_POLICY_LEG_COUNT],
                         float values[12])
{
    for (uint8_t leg = 0U; leg < GAIT_POLICY_LEG_COUNT; ++leg) {
        values[leg * 3U] = targets[leg].j1_deg;
        values[leg * 3U + 1U] = targets[leg].j2_deg;
        values[leg * 3U + 2U] = targets[leg].j3_deg;
    }
}

SPOT_GAIT_EXPORT float spot_gait_smootherstep(float progress)
{
    return gait_policy_smootherstep(progress);
}

SPOT_GAIT_EXPORT int spot_gait_trot_targets(
    float phase,
    float amplitude_scale,
    float travel_scale,
    float values[12],
    uint8_t *support_mask)
{
    GaitPolicyLegTarget targets[GAIT_POLICY_LEG_COUNT];
    if (values == NULL || support_mask == NULL ||
        !gait_policy_trot_targets(
            phase, amplitude_scale, travel_scale, targets)) {
        return 0;
    }
    pack_targets(targets, values);
    *support_mask = gait_policy_support_mask(targets);
    return 1;
}

SPOT_GAIT_EXPORT int spot_gait_sim_trot_targets(
    float phase,
    float amplitude_scale,
    float values[12],
    uint8_t *support_mask)
{
    GaitPolicyLegTarget targets[GAIT_POLICY_LEG_COUNT];
    if (values == NULL || support_mask == NULL ||
        !gait_policy_sim_trot_targets(
            phase, amplitude_scale, targets)) {
        return 0;
    }
    pack_targets(targets, values);
    *support_mask = gait_policy_support_mask(targets);
    return 1;
}

SPOT_GAIT_EXPORT int spot_gait_trot2_targets(
    float phase,
    float amplitude_scale,
    float fold_j2_deg,
    float fold_j3_deg,
    float values[12],
    uint8_t *support_mask)
{
    GaitPolicyLegTarget targets[GAIT_POLICY_LEG_COUNT];
    if (values == NULL || support_mask == NULL ||
        !gait_policy_trot2_targets(
            phase,
            amplitude_scale,
            fold_j2_deg,
            fold_j3_deg,
            targets)) {
        return 0;
    }
    pack_targets(targets, values);
    *support_mask = gait_policy_support_mask(targets);
    return 1;
}

SPOT_GAIT_EXPORT int spot_gait_jump_targets(
    float phase,
    float forward_travel,
    float values[12],
    uint8_t *support_mask)
{
    GaitPolicyLegTarget targets[GAIT_POLICY_LEG_COUNT];
    if (values == NULL || support_mask == NULL ||
        !gait_policy_jump_targets(
            phase, forward_travel, targets)) {
        return 0;
    }
    pack_targets(targets, values);
    *support_mask = gait_policy_support_mask(targets);
    return 1;
}

SPOT_GAIT_EXPORT int spot_gait_balance_targets(
    const float imu_values[4],
    const float balance_values[7],
    int contact_aware,
    uint8_t support_mask,
    float values[12])
{
    GaitPolicyLegTarget targets[GAIT_POLICY_LEG_COUNT];
    if (imu_values == NULL || balance_values == NULL || values == NULL) {
        return 0;
    }
    unpack_targets(values, targets);
    const GaitPolicyImuSample sample = {
        imu_values[0], imu_values[1], imu_values[2], imu_values[3]
    };
    const GaitPolicyBalanceConfig config = {
        balance_values[0], balance_values[1], balance_values[2],
        balance_values[3], balance_values[4], balance_values[5],
        balance_values[6], contact_aware != 0
    };
    if (!gait_policy_balance_targets(
            &sample, &config, support_mask, targets)) {
        return 0;
    }
    pack_targets(targets, values);
    return 1;
}
