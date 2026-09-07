#include "robot.h"
#include "sts3215.h"

#include <math.h>
#include <string.h>

#define FORWARD_FRAME_MS 20U
#define FORWARD_DURATION_MS 24000U
#define FORWARD_TILT_TENTHS 150
/* Match the existing stand arrival tolerance, not the host trial's 16 ticks. */
#define FORWARD_TOLERANCE 120U
#define FORWARD_SETTLE_MS 2500U
#define FORWARD_ENCODER_MARGIN 128U

static uint16_t distance(uint16_t a, uint16_t b)
{
    return a > b ? a - b : b - a;
}

static RobotResult attitude(RobotController *robot)
{
    int16_t roll, pitch;
    if (robot->attitude_reader == NULL ||
        !robot->attitude_reader(robot->attitude_context, &roll, &pitch)) {
        return ROBOT_IMU_ERROR;
    }
    const int32_t roll_error = (int32_t)roll - ROBOT_IMU_LEVEL_ROLL_TENTHS;
    const int32_t pitch_error = (int32_t)pitch - ROBOT_IMU_LEVEL_PITCH_TENTHS;
    if (roll_error > FORWARD_TILT_TENTHS || roll_error < -FORWARD_TILT_TENTHS ||
        pitch_error > FORWARD_TILT_TENTHS || pitch_error < -FORWARD_TILT_TENTHS) {
        return ROBOT_TILT_LIMIT;
    }
    return ROBOT_OK;
}

static bool is_stand(const uint16_t positions[ROBOT_JOINT_COUNT])
{
    uint16_t stand[ROBOT_JOINT_COUNT];
    if (!robot_stand_targets(stand)) {
        return false;
    }
    for (size_t i = 0U; i < ROBOT_JOINT_COUNT; ++i) {
        if (distance(positions[i], stand[i]) > FORWARD_TOLERANCE) {
            return false;
        }
    }
    return true;
}

static RobotResult sample(RobotController *robot, size_t index,
                          const uint16_t target[ROBOT_JOINT_COUNT],
                          uint16_t *position)
{
    const RobotJointConfig *joint = &g_robot_joints[index];
    Sts3215State state;
    ServoBusResult bus = sts3215_read_state(robot->bus, joint->servo_id, &state);
    if (bus != SERVO_BUS_OK) {
        robot->last_failed_servo_id = joint->servo_id;
        robot->last_bus_result = bus;
        return ROBOT_BUS_ERROR;
    }
    if (state.position < FORWARD_ENCODER_MARGIN ||
        state.position > STS3215_MAX_POSITION - FORWARD_ENCODER_MARGIN) {
        robot->last_failed_servo_id = joint->servo_id;
        return ROBOT_POSITION_LIMIT;
    }
    const SafetySample value = {
        joint->servo_id, joint->leg_index, joint->joint_index,
        target[index], state.position, state.load, state.current,
        state.temperature_c, state.hardware_error
    };
    if (safety_update(&robot->safety, &value, HAL_GetTick(), 0U)) {
        robot->last_failed_servo_id = joint->servo_id;
        return ROBOT_SAFETY_FAULT;
    }
    if (position != NULL) {
        *position = state.position;
    }
    return ROBOT_OK;
}

static RobotResult ramp(RobotController *robot,
                        const uint16_t from[ROBOT_JOINT_COUNT],
                        const uint16_t to[ROBOT_JOINT_COUNT], uint32_t duration)
{
    uint16_t target[ROBOT_JOINT_COUNT];
    /* Progress by at most one frame after a delayed bus read. Never catch up
     * with a large position jump. Every frame is one 12-servo Sync Write. */
    for (uint32_t elapsed = FORWARD_FRAME_MS; elapsed <= duration;
         elapsed += FORWARD_FRAME_MS) {
        const uint32_t frame_start = HAL_GetTick();
        if (robot->motion_abort_requested) {
            return ROBOT_MOTION_ABORTED;
        }
        const RobotResult orientation = attitude(robot);
        if (orientation != ROBOT_OK) {
            return orientation;
        }
        const float t = (float)elapsed / (float)duration;
        const float blend = t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
        for (size_t i = 0U; i < ROBOT_JOINT_COUNT; ++i) {
            /* Signed linear coordinates. No modulo, angle wrapping, or
             * choice of an equivalent target on another revolution. */
            target[i] = (uint16_t)lroundf((float)from[i] +
                ((float)to[i] - (float)from[i]) * blend);
        }
        const ServoBusResult bus = sts3215_sync_positions(
            robot->bus, g_robot_servo_ids, target, ROBOT_JOINT_COUNT);
        if (bus != SERVO_BUS_OK) {
            robot->last_failed_servo_id = 254U;
            robot->last_bus_result = bus;
            return ROBOT_BUS_ERROR;
        }
        size_t index = robot->safety_scan_index;
        uint8_t watched_id = 0U;
        if (safety_watching(&robot->safety, &watched_id)) {
            for (size_t i = 0U; i < ROBOT_JOINT_COUNT; ++i) {
                if (g_robot_servo_ids[i] == watched_id) {
                    index = i;
                    break;
                }
            }
        } else {
            robot->safety_scan_index = (uint8_t)((index + 1U) % ROBOT_JOINT_COUNT);
        }
        const RobotResult result = sample(robot, index, target, NULL);
        if (result != ROBOT_OK) {
            return result;
        }
        const uint32_t spent = HAL_GetTick() - frame_start;
        if (spent < FORWARD_FRAME_MS) {
            HAL_Delay(FORWARD_FRAME_MS - spent);
        }
    }
    return ROBOT_OK;
}

static RobotResult settle(RobotController *robot,
                          const uint16_t target[ROBOT_JOINT_COUNT])
{
    const uint32_t started = HAL_GetTick();
    for (;;) {
        uint16_t worst = 0U;
        for (size_t i = 0U; i < ROBOT_JOINT_COUNT; ++i) {
            if (robot->motion_abort_requested) {
                return ROBOT_MOTION_ABORTED;
            }
            const RobotResult orientation = attitude(robot);
            if (orientation != ROBOT_OK) {
                return orientation;
            }
            uint16_t actual;
            const RobotResult result = sample(robot, i, target, &actual);
            if (result != ROBOT_OK) {
                return result;
            }
            const uint16_t error = distance(actual, target[i]);
            if (error > worst) {
                worst = error;
                robot->last_failed_servo_id = g_robot_servo_ids[i];
            }
        }
        if (worst <= FORWARD_TOLERANCE) {
            return ROBOT_OK;
        }
        if (HAL_GetTick() - started >= FORWARD_SETTLE_MS) {
            return ROBOT_VERIFY_ERROR;
        }
        HAL_Delay(FORWARD_FRAME_MS);
    }
}

RobotResult robot_forward_straight(RobotController *robot)
{
    if (robot == NULL || robot->bus == NULL || robot->drive_active) {
        return ROBOT_INVALID_ARGUMENT;
    }
    if (safety_is_faulted(&robot->safety)) {
        return ROBOT_SAFETY_FAULT;
    }
    robot->motion_abort_requested = false;
    uint16_t from[ROBOT_JOINT_COUNT];
    uint16_t forward[ROBOT_JOINT_COUNT];
    RobotResult result = robot_read_positions(robot, from);
    if (result != ROBOT_OK) {
        return result;
    }
    if (!is_stand(from)) {
        return ROBOT_STAND_REQUIRED;
    }
    result = attitude(robot);
    if (result != ROBOT_OK) {
        return result;
    }
    if (robot->motion_abort_requested) {
        return ROBOT_MOTION_ABORTED;
    }
    result = robot_hold(robot);
    if (result != ROBOT_OK) {
        goto stop;
    }
    /* Re-read after holding, so torque-off/manual movement cannot leave a
     * stale start target. The hold itself seeds measured positions first. */
    result = robot_read_positions(robot, from);
    if (result != ROBOT_OK) {
        goto stop;
    }
    if (!is_stand(from)) {
        result = ROBOT_STAND_REQUIRED;
        goto stop;
    }
    memcpy(forward, from, sizeof(forward));
    for (size_t i = 0U; i < ROBOT_JOINT_COUNT; ++i) {
        const uint8_t joint = g_robot_joints[i].joint_index;
        if (joint == 2U) {
            /* Physical direction confirmed by the operator: +90 points
             * rearward on this robot. From stand (+45), go toward -90
             * directly in the existing encoder frame, never via +270. */
            if (!robot_angle_to_position(i, -90, &forward[i])) {
                result = ROBOT_POSITION_LIMIT;
                goto stop;
            }
        } else if (joint == 3U &&
                   !robot_angle_to_position(i, 0, &forward[i])) {
            result = ROBOT_POSITION_LIMIT;
            goto stop;
        }
        if (from[i] < FORWARD_ENCODER_MARGIN ||
            from[i] > STS3215_MAX_POSITION - FORWARD_ENCODER_MARGIN ||
            forward[i] < FORWARD_ENCODER_MARGIN ||
            forward[i] > STS3215_MAX_POSITION - FORWARD_ENCODER_MARGIN ||
            distance(from[i], forward[i]) >= STS3215_STEPS_PER_REVOLUTION / 2U) {
            result = ROBOT_POSITION_LIMIT;
            goto stop;
        }
    }
    robot->safety_scan_index = 0U;
    /* Hips and knees move together. Retaining the v11 knee angle until J2
     * reached -90 would swing the lower links above the hip. */
    result = ramp(robot, from, forward, FORWARD_DURATION_MS);
    if (result == ROBOT_OK) {
        result = settle(robot, forward);
    }
    if (result == ROBOT_OK) {
        return ROBOT_OK;
    }
stop:
    /* Never return to stand automatically from a partially folded posture. */
    if ((result == ROBOT_TILT_LIMIT || result == ROBOT_IMU_ERROR ||
         result == ROBOT_MOTION_ABORTED) && robot_hold(robot) == ROBOT_OK) {
        return result;
    }
    (void)robot_relax(robot);
    return result;
}
