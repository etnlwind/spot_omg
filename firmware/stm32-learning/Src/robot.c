#include "robot.h"

#include "feetech_protocol.h"
#include "sts3215.h"

#include <stdbool.h>

#define ROBOT_PROFILE_SPEED_DEFAULT          3400U
#define ROBOT_PROFILE_ACCELERATION_DEFAULT   254U
#define ROBOT_VERIFY_TOLERANCE       120U
#define ROBOT_VERIFY_TIMEOUT_MS      2000U
#define ROBOT_VERIFY_POLL_MS         100U
#define ROBOT_TROT_FRAME_MS          20U
#define ROBOT_TROT_HIP_DEGREES       10
#define ROBOT_TROT_LIFT_DEGREES      16
#define ROBOT_TROT_DUTY_PER_MILLE    600U
#define ROBOT_TROT_RAMP_MS            200U
#define ROBOT_TROT_SWING_LIFT_END     200U
#define ROBOT_TROT_SWING_LOWER_START  800U
#define ROBOT_BALANCE_DEADBAND_TENTHS 5
#define ROBOT_BALANCE_ERROR_LIMIT     300
#define ROBOT_BALANCE_KNEE_LIMIT      50
#define ROBOT_BALANCE_IMU_FAILURES    3U

static RobotResult bus_failure(RobotController *robot,
                               uint8_t servo_id,
                               ServoBusResult result)
{
    robot->last_failed_servo_id = servo_id;
    robot->last_bus_result = result;
    return ROBOT_BUS_ERROR;
}

static const RobotJointConfig *config_for_servo(uint8_t servo_id)
{
    for (size_t index = 0U; index < ROBOT_JOINT_COUNT; ++index) {
        if (g_robot_servo_ids[index] == servo_id) {
            return &g_robot_joints[index];
        }
    }
    return NULL;
}

void robot_init(RobotController *robot, ServoBus *bus)
{
    if (robot == NULL) {
        return;
    }

    robot->bus = bus;
    robot->last_bus_result = SERVO_BUS_OK;
    robot->last_failed_servo_id = 0U;
    robot->profile_speed = ROBOT_PROFILE_SPEED_DEFAULT;
    robot->profile_acceleration = ROBOT_PROFILE_ACCELERATION_DEFAULT;
    robot->attitude_reader = NULL;
    robot->attitude_context = NULL;
    robot->balance_enabled = false;
    robot->balance_reference_valid = false;
    robot->balance_reference_roll_tenths = 0;
    robot->balance_reference_pitch_tenths = 0;
}

bool robot_set_profile(RobotController *robot,
                       uint16_t speed,
                       uint8_t acceleration)
{
    if (robot == NULL || speed == 0U || speed > 3400U ||
        acceleration > 254U) {
        return false;
    }

    robot->profile_speed = speed;
    robot->profile_acceleration = acceleration;
    return true;
}

void robot_set_attitude_reader(RobotController *robot,
                               RobotAttitudeReader reader,
                               void *context)
{
    if (robot == NULL) {
        return;
    }

    robot->attitude_reader = reader;
    robot->attitude_context = context;
    robot->balance_enabled = false;
    robot->balance_reference_valid = false;
}

bool robot_set_balance_enabled(RobotController *robot, bool enabled)
{
    if (robot == NULL || (enabled && robot->attitude_reader == NULL)) {
        return false;
    }

    robot->balance_enabled = enabled;
    robot->balance_reference_valid = false;
    return true;
}

RobotResult robot_require_all(RobotController *robot)
{
    if (robot == NULL || robot->bus == NULL) {
        return ROBOT_INVALID_ARGUMENT;
    }
    if (!robot_config_valid()) {
        return ROBOT_CONFIG_ERROR;
    }

    for (size_t index = 0U; index < ROBOT_JOINT_COUNT; ++index) {
        const uint8_t id = g_robot_servo_ids[index];
        ServoBusResult result = sts3215_ping(robot->bus, id);
        if (result != SERVO_BUS_OK) {
            robot->last_failed_servo_id = id;
            robot->last_bus_result = result;
            return result == SERVO_BUS_TIMEOUT ? ROBOT_MISSING_SERVO
                                               : ROBOT_BUS_ERROR;
        }
    }

    return ROBOT_OK;
}

RobotResult robot_read_positions(
    RobotController *robot,
    uint16_t positions[ROBOT_JOINT_COUNT])
{
    if (robot == NULL || robot->bus == NULL || positions == NULL) {
        return ROBOT_INVALID_ARGUMENT;
    }

    for (size_t index = 0U; index < ROBOT_JOINT_COUNT; ++index) {
        const uint8_t id = g_robot_servo_ids[index];
        ServoBusResult result = sts3215_read_position(robot->bus,
                                                      id,
                                                      &positions[index]);
        if (result != SERVO_BUS_OK) {
            return bus_failure(robot, id, result);
        }
        if (positions[index] < g_robot_joints[index].minimum ||
            positions[index] > g_robot_joints[index].maximum) {
            robot->last_failed_servo_id = id;
            return ROBOT_POSITION_LIMIT;
        }
    }

    return ROBOT_OK;
}

RobotResult robot_relax(RobotController *robot)
{
    if (robot == NULL || robot->bus == NULL) {
        return ROBOT_INVALID_ARGUMENT;
    }

    RobotResult final_result = ROBOT_OK;
    for (size_t index = 0U; index < ROBOT_JOINT_COUNT; ++index) {
        const uint8_t id = g_robot_servo_ids[index];
        ServoBusResult result = sts3215_set_torque(robot->bus, id, false);
        if (result != SERVO_BUS_OK && final_result == ROBOT_OK) {
            final_result = bus_failure(robot, id, result);
        }
    }
    return final_result;
}

RobotResult robot_hold(RobotController *robot)
{
    uint16_t current[ROBOT_JOINT_COUNT];

    RobotResult result = robot_require_all(robot);
    if (result != ROBOT_OK) {
        return result;
    }

    result = robot_read_positions(robot, current);
    if (result != ROBOT_OK) {
        return result;
    }

    ServoBusResult bus_result = sts3215_sync_move(
        robot->bus,
        g_robot_servo_ids,
        current,
        ROBOT_JOINT_COUNT,
        robot->profile_speed,
        robot->profile_acceleration);
    if (bus_result != SERVO_BUS_OK) {
        return bus_failure(robot, FEETECH_BROADCAST_ID, bus_result);
    }

    size_t enabled = 0U;
    for (; enabled < ROBOT_JOINT_COUNT; ++enabled) {
        const uint8_t id = g_robot_servo_ids[enabled];
        bus_result = sts3215_set_torque(robot->bus, id, true);
        if (bus_result != SERVO_BUS_OK) {
            for (size_t rollback = 0U; rollback < enabled; ++rollback) {
                (void)sts3215_set_torque(robot->bus,
                                         g_robot_servo_ids[rollback],
                                         false);
            }
            return bus_failure(robot, id, bus_result);
        }
    }

    return ROBOT_OK;
}

RobotResult robot_stand(RobotController *robot)
{
    uint16_t target[ROBOT_JOINT_COUNT];
    uint16_t measured_positions[ROBOT_JOINT_COUNT];

    if (robot == NULL || robot->bus == NULL) {
        return ROBOT_INVALID_ARGUMENT;
    }
    if (!robot_stand_targets(target)) {
        return ROBOT_CONFIG_ERROR;
    }

    RobotResult result = robot_hold(robot);
    if (result != ROBOT_OK) {
        return result;
    }

    ServoBusResult bus_result = sts3215_sync_positions(robot->bus,
                                                       g_robot_servo_ids,
                                                       target,
                                                       ROBOT_JOINT_COUNT);
    if (bus_result != SERVO_BUS_OK) {
        return bus_failure(robot, FEETECH_BROADCAST_ID, bus_result);
    }

    const uint32_t verify_started_at = HAL_GetTick();
    for (;;) {
        result = robot_read_positions(robot, measured_positions);
        if (result != ROBOT_OK) {
            return result;
        }

        bool all_within_tolerance = true;
        for (size_t joint = 0U; joint < ROBOT_JOINT_COUNT; ++joint) {
            int32_t error = (int32_t)measured_positions[joint] -
                            (int32_t)target[joint];
            if (error < 0) {
                error = -error;
            }
            if ((uint32_t)error > ROBOT_VERIFY_TOLERANCE) {
                if (all_within_tolerance) {
                    robot->last_failed_servo_id = g_robot_servo_ids[joint];
                }
                all_within_tolerance = false;
            }
        }

        if (all_within_tolerance) {
            return ROBOT_OK;
        }
        if ((uint32_t)(HAL_GetTick() - verify_started_at) >=
            ROBOT_VERIFY_TIMEOUT_MS) {
            return ROBOT_VERIFY_ERROR;
        }
        HAL_Delay(ROBOT_VERIFY_POLL_MS);
    }
}

static uint16_t trot_amplitude_scale(uint32_t frame,
                                     uint32_t total_frames)
{
    uint32_t ramp_frames = ROBOT_TROT_RAMP_MS / ROBOT_TROT_FRAME_MS;
    if (ramp_frames > total_frames / 2U) {
        ramp_frames = total_frames / 2U;
    }
    if (ramp_frames == 0U) {
        return 1000U;
    }

    uint32_t scale = 1000U;
    if (frame < ramp_frames) {
        scale = (frame * 1000U) / ramp_frames;
    }
    const uint32_t remaining = total_frames - frame;
    if (remaining < ramp_frames) {
        const uint32_t ending_scale = (remaining * 1000U) / ramp_frames;
        if (ending_scale < scale) {
            scale = ending_scale;
        }
    }
    return (uint16_t)scale;
}

static int16_t trot_forward_offset(uint16_t phase)
{
    if (phase < ROBOT_TROT_DUTY_PER_MILLE) {
        return (int16_t)(ROBOT_TROT_HIP_DEGREES -
            (2L * ROBOT_TROT_HIP_DEGREES * phase) /
                ROBOT_TROT_DUTY_PER_MILLE);
    }

    const uint16_t swing_phase = (uint16_t)(
        ((uint32_t)(phase - ROBOT_TROT_DUTY_PER_MILLE) * 1000U) /
        (1000U - ROBOT_TROT_DUTY_PER_MILLE));
    if (swing_phase < ROBOT_TROT_SWING_LIFT_END) {
        return -ROBOT_TROT_HIP_DEGREES;
    }
    if (swing_phase < ROBOT_TROT_SWING_LOWER_START) {
        return (int16_t)(-ROBOT_TROT_HIP_DEGREES +
            (2L * ROBOT_TROT_HIP_DEGREES *
             (swing_phase - ROBOT_TROT_SWING_LIFT_END)) /
                (ROBOT_TROT_SWING_LOWER_START -
                 ROBOT_TROT_SWING_LIFT_END));
    }
    return ROBOT_TROT_HIP_DEGREES;
}

static int16_t trot_lift_offset(uint16_t phase)
{
    if (phase < ROBOT_TROT_DUTY_PER_MILLE) {
        return 0;
    }

    const uint16_t swing_phase = (uint16_t)(
        ((uint32_t)(phase - ROBOT_TROT_DUTY_PER_MILLE) * 1000U) /
        (1000U - ROBOT_TROT_DUTY_PER_MILLE));
    if (swing_phase < ROBOT_TROT_SWING_LIFT_END) {
        return (int16_t)((ROBOT_TROT_LIFT_DEGREES * swing_phase) /
                         ROBOT_TROT_SWING_LIFT_END);
    }
    if (swing_phase < ROBOT_TROT_SWING_LOWER_START) {
        return ROBOT_TROT_LIFT_DEGREES;
    }
    return (int16_t)((ROBOT_TROT_LIFT_DEGREES *
                      (1000U - swing_phase)) /
                     (1000U - ROBOT_TROT_SWING_LOWER_START));
}

static int16_t clamp_i16(int32_t value, int16_t limit)
{
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return (int16_t)-limit;
    }
    return (int16_t)value;
}

static int16_t balance_deadband(int16_t error)
{
    if (error > ROBOT_BALANCE_DEADBAND_TENTHS) {
        return (int16_t)(error - ROBOT_BALANCE_DEADBAND_TENTHS);
    }
    if (error < -ROBOT_BALANCE_DEADBAND_TENTHS) {
        return (int16_t)(error + ROBOT_BALANCE_DEADBAND_TENTHS);
    }
    return 0;
}

static int16_t balance_knee_correction(uint8_t leg_index,
                                       int16_t roll_error,
                                       int16_t pitch_error)
{
    const int16_t side_sign =
        (leg_index == 0U || leg_index == 2U) ? 1 : -1;
    const int16_t end_sign = leg_index < 2U ? 1 : -1;
    const int32_t correction =
        ((int32_t)side_sign * balance_deadband(roll_error)) -
        ((int32_t)end_sign * balance_deadband(pitch_error));
    return clamp_i16(correction, ROBOT_BALANCE_KNEE_LIMIT);
}

static void return_to_stand_best_effort(RobotController *robot)
{
    uint16_t stand_targets[ROBOT_JOINT_COUNT];
    if (robot != NULL && robot->bus != NULL &&
        robot_stand_targets(stand_targets)) {
        (void)sts3215_sync_positions(robot->bus,
                                     g_robot_servo_ids,
                                     stand_targets,
                                     ROBOT_JOINT_COUNT);
    }
}

RobotResult robot_trot(RobotController *robot,
                       uint8_t cycles,
                       uint16_t period_ms)
{
    uint16_t targets[ROBOT_JOINT_COUNT];
    int16_t filtered_roll_error = 0;
    int16_t filtered_pitch_error = 0;
    uint8_t consecutive_imu_failures = 0U;

    if (robot == NULL || robot->bus == NULL || cycles == 0U || cycles > 10U ||
        period_ms < 600U || period_ms > 5000U) {
        return ROBOT_INVALID_ARGUMENT;
    }

    RobotResult result = robot_stand(robot);
    if (result != ROBOT_OK) {
        return result;
    }
    HAL_Delay(300U);

    robot->balance_reference_valid = false;
    if (robot->balance_enabled) {
        if (robot->attitude_reader == NULL ||
            !robot->attitude_reader(
                robot->attitude_context,
                &robot->balance_reference_roll_tenths,
                &robot->balance_reference_pitch_tenths)) {
            return ROBOT_IMU_ERROR;
        }
        robot->balance_reference_valid = true;
    }

    const uint32_t frames_per_cycle = period_ms / ROBOT_TROT_FRAME_MS;
    const uint32_t total_frames = frames_per_cycle * cycles;
    const uint32_t started_at = HAL_GetTick();

    for (uint32_t frame = 0U; frame <= total_frames; ++frame) {
        const uint16_t global_phase = (uint16_t)(
            ((frame % frames_per_cycle) * 1000U) / frames_per_cycle);
        const uint16_t amplitude_scale =
            trot_amplitude_scale(frame, total_frames);

        if (robot->balance_enabled && frame > 0U && frame < total_frames) {
            int16_t roll_tenths = 0;
            int16_t pitch_tenths = 0;
            if (robot->attitude_reader(
                    robot->attitude_context,
                    &roll_tenths,
                    &pitch_tenths)) {
                const int16_t roll_error = clamp_i16(
                    (int32_t)roll_tenths -
                    robot->balance_reference_roll_tenths,
                    ROBOT_BALANCE_ERROR_LIMIT);
                const int16_t pitch_error = clamp_i16(
                    (int32_t)pitch_tenths -
                    robot->balance_reference_pitch_tenths,
                    ROBOT_BALANCE_ERROR_LIMIT);
                filtered_roll_error = (int16_t)(
                    ((3L * filtered_roll_error) + roll_error) / 4L);
                filtered_pitch_error = (int16_t)(
                    ((3L * filtered_pitch_error) + pitch_error) / 4L);
                consecutive_imu_failures = 0U;
            } else {
                ++consecutive_imu_failures;
                if (consecutive_imu_failures >=
                    ROBOT_BALANCE_IMU_FAILURES) {
                    return_to_stand_best_effort(robot);
                    return ROBOT_IMU_ERROR;
                }
            }
        } else if (frame == total_frames) {
            filtered_roll_error = 0;
            filtered_pitch_error = 0;
        }

        for (size_t index = 0U; index < ROBOT_JOINT_COUNT; ++index) {
            const RobotJointConfig *joint = &g_robot_joints[index];
            const bool second_diagonal = joint->leg_index == 1U ||
                                         joint->leg_index == 2U;
            const uint16_t leg_phase = (uint16_t)(
                (global_phase + (second_diagonal ? 500U : 0U)) % 1000U);
            const int16_t balance_knee = robot->balance_enabled ?
                balance_knee_correction(joint->leg_index,
                                        filtered_roll_error,
                                        filtered_pitch_error) : 0;
            int16_t angle_tenths = 0;

            if (joint->joint_index == 2U) {
                const int16_t forward_sign = joint->leg_index < 2U ? -1 : 1;
                const int16_t forward = trot_forward_offset(leg_phase);
                const int16_t lift = trot_lift_offset(leg_phase);
                /*
                 * J3 is a relative knee angle. Adding half of its lift to
                 * J2 keeps the two-link leg symmetric about vertical, so the
                 * foot rises instead of being pulled backward along the floor.
                 */
                angle_tenths = (int16_t)(450 +
                    (((2L * forward_sign * forward) + lift) *
                     (int32_t)amplitude_scale) / 200L +
                    balance_knee / 2);
            } else if (joint->joint_index == 3U) {
                const int16_t lift = trot_lift_offset(leg_phase);
                angle_tenths = (int16_t)(900 +
                    (lift * (int32_t)amplitude_scale) / 100L +
                    balance_knee);
            }

            if (!robot_angle_tenths_to_position(index,
                                                angle_tenths,
                                                &targets[index])) {
                return_to_stand_best_effort(robot);
                return ROBOT_CONFIG_ERROR;
            }
        }

        ServoBusResult bus_result = sts3215_sync_positions(robot->bus,
                                                           g_robot_servo_ids,
                                                           targets,
                                                           ROBOT_JOINT_COUNT);
        if (bus_result != SERVO_BUS_OK) {
            return bus_failure(robot, FEETECH_BROADCAST_ID, bus_result);
        }

        if (frame < total_frames) {
            const uint32_t deadline = started_at +
                ((frame + 1U) * period_ms) / frames_per_cycle;
            const uint32_t now = HAL_GetTick();
            if ((int32_t)(deadline - now) > 0) {
                HAL_Delay(deadline - now);
            }
        }
    }
    return ROBOT_OK;
}

RobotResult robot_move_single_safe(RobotController *robot,
                                   uint8_t servo_id,
                                   uint16_t target_position,
                                   uint16_t maximum_delta)
{
    uint16_t current = 0U;
    const RobotJointConfig *joint = config_for_servo(servo_id);

    if (robot == NULL || robot->bus == NULL ||
        joint == NULL || target_position < joint->minimum ||
        target_position > joint->maximum || maximum_delta == 0U) {
        return ROBOT_INVALID_ARGUMENT;
    }

    ServoBusResult bus_result = sts3215_ping(robot->bus, servo_id);
    if (bus_result != SERVO_BUS_OK) {
        return bus_failure(robot, servo_id, bus_result);
    }
    bus_result = sts3215_read_position(robot->bus, servo_id, &current);
    if (bus_result != SERVO_BUS_OK) {
        return bus_failure(robot, servo_id, bus_result);
    }
    if (current < joint->minimum || current > joint->maximum) {
        robot->last_failed_servo_id = servo_id;
        return ROBOT_POSITION_LIMIT;
    }

    const uint16_t delta = target_position > current
                               ? (uint16_t)(target_position - current)
                               : (uint16_t)(current - target_position);
    if (delta > maximum_delta) {
        return ROBOT_MOVE_TOO_LARGE;
    }

    bus_result = sts3215_write_position(robot->bus,
                                        servo_id,
                                        current,
                                        robot->profile_speed,
                                        robot->profile_acceleration);
    if (bus_result != SERVO_BUS_OK) {
        return bus_failure(robot, servo_id, bus_result);
    }
    bus_result = sts3215_set_torque(robot->bus, servo_id, true);
    if (bus_result != SERVO_BUS_OK) {
        return bus_failure(robot, servo_id, bus_result);
    }
    HAL_Delay(50U);

    bus_result = sts3215_write_position(robot->bus,
                                        servo_id,
                                        target_position,
                                        robot->profile_speed,
                                        robot->profile_acceleration);
    if (bus_result != SERVO_BUS_OK) {
        (void)sts3215_set_torque(robot->bus, servo_id, false);
        return bus_failure(robot, servo_id, bus_result);
    }

    return ROBOT_OK;
}

const char *robot_result_string(RobotResult result)
{
    switch (result) {
    case ROBOT_OK:
        return "ok";
    case ROBOT_INVALID_ARGUMENT:
        return "invalid argument";
    case ROBOT_CONFIG_ERROR:
        return "configuration error";
    case ROBOT_BUS_ERROR:
        return "servo bus error";
    case ROBOT_MISSING_SERVO:
        return "missing servo";
    case ROBOT_POSITION_LIMIT:
        return "position limit";
    case ROBOT_MOVE_TOO_LARGE:
        return "move exceeds safe delta";
    case ROBOT_VERIFY_ERROR:
        return "final position verification failed";
    case ROBOT_IMU_ERROR:
        return "IMU balance error";
    default:
        return "unknown";
    }
}
