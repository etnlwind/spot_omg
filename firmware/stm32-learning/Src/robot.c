#include "robot.h"

#include "feetech_protocol.h"
#include "sts3215.h"

#include <stdbool.h>

#define ROBOT_PROFILE_SPEED_DEFAULT          3400U
#define ROBOT_PROFILE_ACCELERATION_DEFAULT   254U
#define ROBOT_STAND_FRAMES           100U
#define ROBOT_STAND_FRAME_MS         20U
#define ROBOT_VERIFY_TOLERANCE       120U
#define ROBOT_VERIFY_TIMEOUT_MS      2000U
#define ROBOT_VERIFY_POLL_MS         100U

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
    uint16_t start[ROBOT_JOINT_COUNT];
    uint16_t target[ROBOT_JOINT_COUNT];
    uint16_t frame_positions[ROBOT_JOINT_COUNT];

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
    result = robot_read_positions(robot, start);
    if (result != ROBOT_OK) {
        return result;
    }

    for (uint32_t frame = 1U; frame <= ROBOT_STAND_FRAMES; ++frame) {
        for (size_t joint = 0U; joint < ROBOT_JOINT_COUNT; ++joint) {
            const int32_t delta = (int32_t)target[joint] -
                                  (int32_t)start[joint];
            const int32_t interpolated = (int32_t)start[joint] +
                (delta * (int32_t)frame) / (int32_t)ROBOT_STAND_FRAMES;
            frame_positions[joint] = (uint16_t)interpolated;
        }

        ServoBusResult bus_result = sts3215_sync_positions(
            robot->bus,
            g_robot_servo_ids,
            frame_positions,
            ROBOT_JOINT_COUNT);
        if (bus_result != SERVO_BUS_OK) {
            return bus_failure(robot, FEETECH_BROADCAST_ID, bus_result);
        }
        HAL_Delay(ROBOT_STAND_FRAME_MS);
    }

    const uint32_t verify_started_at = HAL_GetTick();
    for (;;) {
        result = robot_read_positions(robot, frame_positions);
        if (result != ROBOT_OK) {
            return result;
        }

        bool all_within_tolerance = true;
        for (size_t joint = 0U; joint < ROBOT_JOINT_COUNT; ++joint) {
            int32_t error = (int32_t)frame_positions[joint] -
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
    default:
        return "unknown";
    }
}
