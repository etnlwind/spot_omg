#include "actuator_control.h"

#include <math.h>
#include <string.h>

static float clampf(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static uint16_t magnitude_i16(int16_t value)
{
    const int32_t wide = value;
    return (uint16_t)(wide < 0 ? -wide : wide);
}

/* Canonical actuator arrays are ordered J1,J2,J3 for each of four legs. */
static bool joint_uses_sts3250(size_t joint_array_index)
{
    return (joint_array_index % 3U) == 1U;
}

static float command_velocity_limit(size_t joint_array_index)
{
    return joint_uses_sts3250(joint_array_index) ?
        MOTOR_STS3250_COMMAND_VELOCITY_LIMIT_DEG_S :
        MOTOR_STS3215_COMMAND_VELOCITY_LIMIT_DEG_S;
}

static float command_acceleration_limit(size_t joint_array_index)
{
    return joint_uses_sts3250(joint_array_index) ?
        MOTOR_STS3250_COMMAND_ACCELERATION_LIMIT_DEG_S2 :
        MOTOR_STS3215_COMMAND_ACCELERATION_LIMIT_DEG_S2;
}

void actuator_rate_limiter_init(ActuatorRateLimiter *limiter)
{
    if (limiter != NULL) {
        memset(limiter, 0, sizeof(*limiter));
    }
}

bool actuator_profile_supports_trot3(uint16_t profile_speed,
                                     uint8_t profile_acceleration)
{
    return profile_speed >= MOTOR_STS3215_TROT3_REQUIRED_PROFILE_SPEED &&
           profile_acceleration >=
               MOTOR_STS3215_TROT3_REQUIRED_PROFILE_ACCELERATION;
}

bool actuator_rate_limiter_reset(
    ActuatorRateLimiter *limiter,
    const float positions_deg[ACTUATOR_CONTROL_JOINT_COUNT])
{
    if (limiter == NULL || positions_deg == NULL) {
        return false;
    }
    for (size_t index = 0U; index < ACTUATOR_CONTROL_JOINT_COUNT; ++index) {
        if (!isfinite(positions_deg[index])) {
            return false;
        }
        limiter->position_deg[index] = positions_deg[index];
        limiter->velocity_deg_s[index] = 0.0f;
    }
    limiter->initialized = true;
    return true;
}

bool actuator_rate_limiter_apply(
    ActuatorRateLimiter *limiter,
    const float requested_deg[ACTUATOR_CONTROL_JOINT_COUNT],
    float dt_seconds,
    ActuatorCommandFrame *command)
{
    if (limiter == NULL || requested_deg == NULL || command == NULL ||
        !isfinite(dt_seconds) || dt_seconds <= 0.0f) {
        return false;
    }
    if (!limiter->initialized &&
        !actuator_rate_limiter_reset(limiter, requested_deg)) {
        return false;
    }

    /* Reject the whole frame before mutating any joint state. */
    for (size_t index = 0U; index < ACTUATOR_CONTROL_JOINT_COUNT; ++index) {
        if (!isfinite(requested_deg[index])) {
            return false;
        }
    }

    command->limited_joint_mask = 0U;
    for (size_t index = 0U; index < ACTUATOR_CONTROL_JOINT_COUNT; ++index) {
        const float velocity_limit = command_velocity_limit(index);
        const float acceleration_limit = command_acceleration_limit(index);
        const float maximum_delta_velocity =
            acceleration_limit * dt_seconds;
        const float previous_position = limiter->position_deg[index];
        const float previous_velocity = limiter->velocity_deg_s[index];
        const float error = requested_deg[index] - previous_position;
        const float direction = error < 0.0f ? -1.0f : 1.0f;

        /* Begin braking before the target instead of relying on a position
         * snap, because snapping would violate the acceleration contract. */
        const float stopping_velocity = sqrtf(
            2.0f * acceleration_limit * fabsf(error));
        float desired_velocity = error / dt_seconds;
        desired_velocity = clampf(
            desired_velocity,
            -velocity_limit,
            velocity_limit);
        if (fabsf(desired_velocity) > stopping_velocity) {
            desired_velocity = direction * stopping_velocity;
        }

        const float velocity_change = clampf(
            desired_velocity - previous_velocity,
            -maximum_delta_velocity,
            maximum_delta_velocity);
        const float next_velocity = clampf(
            previous_velocity + velocity_change,
            -velocity_limit,
            velocity_limit);
        const float next_position =
            previous_position + next_velocity * dt_seconds;

        command->position_deg[index] = next_position;
        command->velocity_deg_s[index] = next_velocity;
        command->acceleration_deg_s2[index] =
            (next_velocity - previous_velocity) / dt_seconds;
        if (fabsf(next_position - requested_deg[index]) > 0.0001f) {
            command->limited_joint_mask |= (uint16_t)(1U << index);
        }

        limiter->position_deg[index] = next_position;
        limiter->velocity_deg_s[index] = next_velocity;
    }
    return true;
}

void actuator_diagnostics_reset(ActuatorDiagnostics *diagnostics)
{
    if (diagnostics != NULL) {
        memset(diagnostics, 0, sizeof(*diagnostics));
    }
}

bool actuator_diagnostics_update(ActuatorDiagnostics *diagnostics,
                                 size_t joint_array_index,
                                 const ActuatorTrackingSample *sample)
{
    if (diagnostics == NULL || sample == NULL ||
        joint_array_index >= ACTUATOR_CONTROL_JOINT_COUNT) {
        return false;
    }

    ActuatorJointDiagnostics *joint =
        &diagnostics->joints[joint_array_index];
    const uint16_t error = magnitude_i16(sample->position_error);
    const uint16_t current = magnitude_i16(sample->current);
    const uint16_t load = magnitude_i16(sample->load);

    ++joint->sample_count;
    ++diagnostics->total_samples;
    if (UINT32_MAX - joint->absolute_error_sum_ticks >= error) {
        joint->absolute_error_sum_ticks += error;
    } else {
        joint->absolute_error_sum_ticks = UINT32_MAX;
    }
    joint->latest = *sample;

    if (error > joint->peak_position_error) {
        joint->peak_position_error = error;
        joint->peak_error_phase = sample->gait_phase;
        joint->peak_error_sample = *sample;
    }
    if (current > joint->peak_current_magnitude) {
        joint->peak_current_magnitude = current;
    }
    if (load > joint->peak_load_magnitude) {
        joint->peak_load_magnitude = load;
    }
    if (joint->minimum_voltage_mv == 0U ||
        sample->voltage_mv < joint->minimum_voltage_mv) {
        joint->minimum_voltage_mv = sample->voltage_mv;
    }
    if (diagnostics->minimum_voltage_mv == 0U ||
        sample->voltage_mv < diagnostics->minimum_voltage_mv) {
        diagnostics->minimum_voltage_mv = sample->voltage_mv;
    }

    if (error >= MOTOR_STS3215_TRACKING_LAG_THRESHOLD_TICKS) {
        ++joint->lag_samples;
        ++diagnostics->lag_samples;
        if (joint->consecutive_lag_samples < UINT8_MAX) {
            ++joint->consecutive_lag_samples;
        }
        if (sample->voltage_mv <=
            MOTOR_STS3215_VOLTAGE_DROOP_THRESHOLD_MV) {
            ++diagnostics->lag_with_voltage_droop_samples;
        }
        if (joint->consecutive_lag_samples >=
            MOTOR_STS3215_TRACKING_LAG_SAMPLES_FOR_DERATE) {
            diagnostics->derate_recommended = true;
        }
    } else {
        joint->consecutive_lag_samples = 0U;
    }

    return true;
}
