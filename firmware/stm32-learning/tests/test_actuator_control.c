#include "actuator_control.h"
#include "gait_policy.h"
#include "robot_config.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static void test_rate_limiter_contract(void)
{
    ActuatorRateLimiter limiter;
    ActuatorCommandFrame command;
    float requested[ACTUATOR_CONTROL_JOINT_COUNT] = {0.0f};
    float previous_velocity[ACTUATOR_CONTROL_JOINT_COUNT] = {0.0f};

    actuator_rate_limiter_init(&limiter);
    assert(actuator_rate_limiter_reset(&limiter, requested));

    for (size_t index = 0U; index < ACTUATOR_CONTROL_JOINT_COUNT; ++index) {
        requested[index] = index % 2U == 0U ? 90.0f : -90.0f;
    }

    for (unsigned int frame = 0U; frame < 100U; ++frame) {
        assert(actuator_rate_limiter_apply(
            &limiter, requested, 0.020f, &command));
        for (size_t index = 0U; index < ACTUATOR_CONTROL_JOINT_COUNT; ++index) {
            assert(fabsf(command.velocity_deg_s[index]) <=
                   MOTOR_STS3215_COMMAND_VELOCITY_LIMIT_DEG_S + 0.001f);
            assert(fabsf(command.acceleration_deg_s2[index]) <=
                   MOTOR_STS3215_COMMAND_ACCELERATION_LIMIT_DEG_S2 + 0.01f);
            assert(fabsf(command.velocity_deg_s[index] -
                         previous_velocity[index]) <=
                   MOTOR_STS3215_COMMAND_ACCELERATION_LIMIT_DEG_S2 * 0.020f +
                   0.001f);
            previous_velocity[index] = command.velocity_deg_s[index];
        }
    }
}

static void test_trot3_rejects_an_inner_profile_below_its_outer_limit(void)
{
    assert(!actuator_profile_supports_trot3(800U, 80U));
    assert(!actuator_profile_supports_trot3(3399U, 254U));
    assert(!actuator_profile_supports_trot3(3400U, 253U));
    assert(actuator_profile_supports_trot3(3400U, 254U));
}

static void test_landing_targets_use_the_canonical_pose(void)
{
    uint16_t targets[ROBOT_JOINT_COUNT];
    assert(robot_landing_targets(targets));

    for (size_t index = 0U; index < ROBOT_JOINT_COUNT; ++index) {
        const uint8_t joint = g_robot_joints[index].joint_index;
        const int16_t angle = joint == 2U ? 40 : (joint == 3U ? 130 : 0);
        uint16_t expected = 0U;
        assert(robot_angle_to_position(index, angle, &expected));
        assert(targets[index] == expected);
    }
}

static ActuatorTrackingSample tracking_sample(uint16_t command,
                                               uint16_t measured,
                                               uint16_t voltage_mv)
{
    const ActuatorTrackingSample sample = {
        3U, 0U, 3U, command, measured,
        (int16_t)((int32_t)command - measured),
        45, 120, -300, voltage_mv, 650U
    };
    return sample;
}

static void test_tracking_and_power_diagnostics(void)
{
    ActuatorDiagnostics diagnostics;
    actuator_diagnostics_reset(&diagnostics);

    ActuatorTrackingSample sample = tracking_sample(2200U, 2160U, 12000U);
    assert(actuator_diagnostics_update(&diagnostics, 2U, &sample));
    assert(!actuator_diagnostics_derate_recommended(&diagnostics));
    assert(diagnostics.minimum_voltage_mv == 12000U);

    sample = tracking_sample(2300U, 2180U, 10900U);
    assert(actuator_diagnostics_update(&diagnostics, 2U, &sample));
    assert(!actuator_diagnostics_derate_recommended(&diagnostics));
    assert(diagnostics.lag_with_voltage_droop_samples == 1U);

    sample = tracking_sample(2350U, 2200U, 10800U);
    assert(actuator_diagnostics_update(&diagnostics, 2U, &sample));
    assert(actuator_diagnostics_derate_recommended(&diagnostics));
    assert(diagnostics.joints[2].peak_position_error == 150U);
    assert(diagnostics.joints[2].peak_error_phase == 650U);
    assert(diagnostics.joints[2].absolute_error_sum_ticks == 310U);
    assert(diagnostics.minimum_voltage_mv == 10800U);
    assert(diagnostics.lag_with_voltage_droop_samples == 2U);
}

static int16_t degrees_to_tenths(float degrees)
{
    const float scaled = degrees * 10.0f;
    return (int16_t)(scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f);
}

static uint16_t smootherstep_per_mille(uint16_t progress)
{
    const float normalized = progress >= 1000U ?
        1.0f : (float)progress / 1000.0f;
    const float scaled = gait_policy_smootherstep(normalized) * 1000.0f;
    const uint32_t rounded = (uint32_t)(scaled + 0.5f);
    return (uint16_t)(rounded > 1000U ? 1000U : rounded);
}

static uint16_t trot3_amplitude_scale(unsigned int frame,
                                      unsigned int total_frames)
{
    unsigned int ramp_frames = 700U / 20U;
    if (ramp_frames > total_frames / 2U) {
        ramp_frames = total_frames / 2U;
    }
    if (ramp_frames == 0U) {
        return 1000U;
    }
    uint16_t scale = 1000U;
    if (frame < ramp_frames) {
        scale = smootherstep_per_mille(
            (uint16_t)((frame * 1000U) / ramp_frames));
    }
    const unsigned int remaining = total_frames - frame;
    if (remaining < ramp_frames) {
        const uint16_t ending = smootherstep_per_mille(
            (uint16_t)((remaining * 1000U) / ramp_frames));
        if (ending < scale) {
            scale = ending;
        }
    }
    return scale;
}

static void test_open_loop_trot3_does_not_need_limiting(uint16_t period_ms)
{
    ActuatorRateLimiter limiter;
    ActuatorCommandFrame command;
    GaitPolicyLegTarget legs[GAIT_POLICY_LEG_COUNT];
    float requested[ACTUATOR_CONTROL_JOINT_COUNT];
    const unsigned int frames = period_ms / 20U;
    unsigned int limited_frames = 0U;
    uint16_t limited_joints = 0U;

    actuator_rate_limiter_init(&limiter);
    for (unsigned int frame = 0U; frame <= frames; ++frame) {
        const float phase = (float)(frame % frames) / (float)frames;
        const float amplitude =
            (float)trot3_amplitude_scale(frame, frames) / 1000.0f;
        assert(gait_policy_trot3_targets(
            phase,
            amplitude,
            GAIT_POLICY_TROT2_FOLD_J2_DEG,
            GAIT_POLICY_TROT2_FOLD_J3_DEG,
            legs));
        for (size_t index = 0U; index < ROBOT_JOINT_COUNT; ++index) {
            const RobotJointConfig *joint = &g_robot_joints[index];
            requested[index] = legs[joint->leg_index].j1_deg;
            if (joint->joint_index == 2U) {
                requested[index] = legs[joint->leg_index].j2_deg;
            } else if (joint->joint_index == 3U) {
                requested[index] = legs[joint->leg_index].j3_deg;
            }
        }
        assert(actuator_rate_limiter_apply(
            &limiter, requested, 0.020f, &command));
        if (command.limited_joint_mask != 0U) {
            ++limited_frames;
            limited_joints |= command.limited_joint_mask;
        }
    }
    assert(limited_frames == 0U);
    assert(limited_joints == 0U);
}

static void test_trot3_pipeline_stays_feasible_and_inside_joint_limits(void)
{
    ActuatorRateLimiter limiter;
    ActuatorCommandFrame command;
    GaitPolicyLegTarget legs[GAIT_POLICY_LEG_COUNT];
    float requested[ACTUATOR_CONTROL_JOINT_COUNT];
    unsigned int limited_frames = 0U;

    actuator_rate_limiter_init(&limiter);
    for (unsigned int frame = 0U; frame < 5U * 40U; ++frame) {
        const float phase = (float)(frame % 40U) / 40.0f;
        assert(gait_policy_trot3_targets(
            phase,
            1.0f,
            GAIT_POLICY_TROT2_FOLD_J2_DEG,
            GAIT_POLICY_TROT2_FOLD_J3_DEG,
            legs));

        for (size_t index = 0U; index < ROBOT_JOINT_COUNT; ++index) {
            const RobotJointConfig *joint = &g_robot_joints[index];
            requested[index] = legs[joint->leg_index].j1_deg;
            if (joint->joint_index == 2U) {
                requested[index] = legs[joint->leg_index].j2_deg;
            } else if (joint->joint_index == 3U) {
                requested[index] = legs[joint->leg_index].j3_deg;
            }
        }

        assert(actuator_rate_limiter_apply(
            &limiter, requested, 0.020f, &command));
        if (command.limited_joint_mask != 0U) {
            ++limited_frames;
        }
        for (size_t index = 0U; index < ROBOT_JOINT_COUNT; ++index) {
            uint16_t raw = 0U;
            assert(fabsf(command.velocity_deg_s[index]) <=
                   MOTOR_STS3215_COMMAND_VELOCITY_LIMIT_DEG_S + 0.001f);
            assert(fabsf(command.acceleration_deg_s2[index]) <=
                   MOTOR_STS3215_COMMAND_ACCELERATION_LIMIT_DEG_S2 + 0.01f);
            assert(robot_angle_tenths_to_position(
                index, degrees_to_tenths(command.position_deg[index]), &raw));
            assert(raw >= g_robot_joints[index].minimum);
            assert(raw <= g_robot_joints[index].maximum);
        }
    }
    assert(limited_frames > 0U);
}

int main(void)
{
    test_rate_limiter_contract();
    test_trot3_rejects_an_inner_profile_below_its_outer_limit();
    test_landing_targets_use_the_canonical_pose();
    test_tracking_and_power_diagnostics();
    test_open_loop_trot3_does_not_need_limiting(1400U);
    test_open_loop_trot3_does_not_need_limiting(1800U);
    test_trot3_pipeline_stays_feasible_and_inside_joint_limits();
    puts("actuator control tests passed");
    return 0;
}
