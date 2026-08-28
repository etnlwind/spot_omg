#ifndef ACTUATOR_CONTROL_H
#define ACTUATOR_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "motor_capability.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define ACTUATOR_CONTROL_JOINT_COUNT 12U

typedef struct
{
    bool initialized;
    float position_deg[ACTUATOR_CONTROL_JOINT_COUNT];
    float velocity_deg_s[ACTUATOR_CONTROL_JOINT_COUNT];
} ActuatorRateLimiter;

typedef struct
{
    float position_deg[ACTUATOR_CONTROL_JOINT_COUNT];
    float velocity_deg_s[ACTUATOR_CONTROL_JOINT_COUNT];
    float acceleration_deg_s2[ACTUATOR_CONTROL_JOINT_COUNT];
    uint16_t limited_joint_mask;
} ActuatorCommandFrame;

void actuator_rate_limiter_init(ActuatorRateLimiter *limiter);

bool actuator_profile_supports_trot3(uint16_t profile_speed,
                                     uint8_t profile_acceleration);

bool actuator_rate_limiter_reset(
    ActuatorRateLimiter *limiter,
    const float positions_deg[ACTUATOR_CONTROL_JOINT_COUNT]);

/*
 * Apply the motor-side velocity/acceleration bounds to canonical joint angles.
 * No calibration, servo IDs or gait geometry is known here.
 */
bool actuator_rate_limiter_apply(
    ActuatorRateLimiter *limiter,
    const float requested_deg[ACTUATOR_CONTROL_JOINT_COUNT],
    float dt_seconds,
    ActuatorCommandFrame *command);

typedef struct
{
    uint8_t servo_id;
    uint8_t leg_index;
    uint8_t joint_index;
    uint16_t commanded_position;
    uint16_t measured_position;
    int16_t position_error;
    int16_t measured_speed;
    int16_t current;
    int16_t load;
    uint16_t voltage_mv;
    uint16_t gait_phase;
    int16_t commanded_velocity_deg_s;
    int16_t commanded_acceleration_deg_s2;
    uint8_t matched_target_age_frames;
    int16_t matched_target_error;
    bool stance;
} ActuatorTrackingSample;

typedef struct
{
    uint32_t sample_count;
    uint32_t absolute_error_sum_ticks;
    uint16_t peak_position_error;
    uint16_t peak_error_phase;
    uint16_t peak_current_magnitude;
    uint16_t peak_load_magnitude;
    uint16_t minimum_voltage_mv;
    uint16_t lag_samples;
    uint8_t consecutive_lag_samples;
    ActuatorTrackingSample latest;
    ActuatorTrackingSample peak_error_sample;
} ActuatorJointDiagnostics;

typedef struct
{
    ActuatorJointDiagnostics joints[ACTUATOR_CONTROL_JOINT_COUNT];
    uint32_t total_samples;
    uint16_t minimum_voltage_mv;
    uint16_t lag_samples;
    uint16_t lag_with_voltage_droop_samples;
    bool derate_recommended;
} ActuatorDiagnostics;

void actuator_diagnostics_reset(ActuatorDiagnostics *diagnostics);

/* Update statistics from a state read the gait safety sampler already made. */
bool actuator_diagnostics_update(ActuatorDiagnostics *diagnostics,
                                 size_t joint_array_index,
                                 const ActuatorTrackingSample *sample);

static inline bool actuator_diagnostics_derate_recommended(
    const ActuatorDiagnostics *diagnostics)
{
    return diagnostics != NULL && diagnostics->derate_recommended;
}

#ifdef __cplusplus
}
#endif

#endif
