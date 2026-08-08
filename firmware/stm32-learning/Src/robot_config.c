#include "robot_config.h"

#include "sts3215.h"

/*
 * Generated from tools/servo_tool/config/joints.json.
 * Leg index: 0=FL, 1=FR, 2=RL, 3=RR.
 * Joint index: 1=abduction, 2=upper leg, 3=knee.
 */
const RobotJointConfig g_robot_joints[ROBOT_JOINT_COUNT] = {
    {1U,  0U, 1U, 2091U, 0U, 4095U,  1},
    {2U,  0U, 2U, 2235U, 0U, 4095U, -1},
    {3U,  0U, 3U, 2060U, 0U, 4095U, -1},
    {4U,  1U, 1U, 2087U, 0U, 4095U, -1},
    {5U,  1U, 2U, 1999U, 0U, 4095U,  1},
    {6U,  1U, 3U, 1977U, 0U, 4095U,  1},
    {7U,  2U, 1U, 2103U, 0U, 4095U, -1},
    {8U,  2U, 2U, 2142U, 0U, 4095U, -1},
    {9U,  2U, 3U, 1996U, 0U, 4095U,  1},
    {10U, 3U, 1U, 1958U, 0U, 4095U,  1},
    {11U, 3U, 2U, 2040U, 0U, 4095U,  1},
    {12U, 3U, 3U, 2047U, 0U, 4095U, -1},
};

const uint8_t g_robot_servo_ids[ROBOT_JOINT_COUNT] = {
    1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 9U, 10U, 11U, 12U
};

/*
 * Physical forward direction observed on the STM32 robot.
 * Leg order: FL, FR, RL, RR.
 *
 * These were {1, 1, -1, -1} on the assumption that the front and rear planar
 * mechanisms are mirror images, so opposite signs would produce the same
 * body-forward foot motion.  On the bench the rear legs looked right and the
 * front legs did not: a diagonal pair shares a phase offset here, but with
 * opposite signs the two feet of that pair travel in opposite directions at
 * the same instant, which reads as the front leg stepping at the wrong time.
 *
 * The simulator had already reached the same conclusion from the other side.
 * walk.py overrides all four to -1 for the sim-trot and trot2 presets, and
 * that is the mapping the validated 10-cycle MuJoCo run used, so the firmware
 * and the simulator had been running different gaits.
 *
 * Whether forward is -1 or +1 for all four is a separate question from the
 * front/rear relationship; the rear sign is kept because the rear legs were
 * the ones that looked correct.  If the robot now walks backwards, negate all
 * four rather than reintroducing a front/rear split.
 */
const int8_t g_robot_gait_forward_signs[4] = {-1, -1, -1, -1};

bool robot_config_valid(void)
{
    bool seen[254] = {false};
    bool joint_seen[4][3] = {{false}};

    for (size_t index = 0U; index < ROBOT_JOINT_COUNT; ++index) {
        const RobotJointConfig *joint = &g_robot_joints[index];
        if (joint->servo_id == 0U || joint->servo_id > 253U ||
            g_robot_servo_ids[index] != joint->servo_id ||
            seen[joint->servo_id] || joint->leg_index > 3U ||
            joint->joint_index < 1U || joint->joint_index > 3U ||
            joint_seen[joint->leg_index][joint->joint_index - 1U] ||
            joint->minimum > joint->center ||
            joint->center > joint->maximum ||
            joint->maximum > STS3215_MAX_POSITION ||
            (joint->direction != -1 && joint->direction != 1)) {
            return false;
        }
        seen[joint->servo_id] = true;
        joint_seen[joint->leg_index][joint->joint_index - 1U] = true;
    }

    for (size_t leg = 0U; leg < 4U; ++leg) {
        if (g_robot_gait_forward_signs[leg] != -1 &&
            g_robot_gait_forward_signs[leg] != 1) {
            return false;
        }
    }

    return true;
}

bool robot_angle_to_position(size_t joint_array_index,
                             int16_t angle_degrees,
                             uint16_t *position)
{
    if (angle_degrees < -3276 || angle_degrees > 3276) {
        return false;
    }
    return robot_angle_tenths_to_position(
        joint_array_index, (int16_t)(angle_degrees * 10), position);
}

bool robot_angle_tenths_to_position(size_t joint_array_index,
                                    int16_t angle_tenths,
                                    uint16_t *position)
{
    if (joint_array_index >= ROBOT_JOINT_COUNT || position == NULL) {
        return false;
    }

    const RobotJointConfig *joint = &g_robot_joints[joint_array_index];
    int32_t scaled = (int32_t)angle_tenths *
                     (int32_t)STS3215_STEPS_PER_REVOLUTION;
    scaled += scaled >= 0 ? 1800 : -1800;
    const int32_t ticks = scaled / 3600;
    const int32_t raw = (int32_t)joint->center +
                        (int32_t)joint->direction * ticks;

    if (raw < (int32_t)joint->minimum || raw > (int32_t)joint->maximum) {
        return false;
    }

    *position = (uint16_t)raw;
    return true;
}

bool robot_stand_targets(uint16_t targets[ROBOT_JOINT_COUNT])
{
    if (targets == NULL || !robot_config_valid()) {
        return false;
    }

    for (size_t index = 0U; index < ROBOT_JOINT_COUNT; ++index) {
        int16_t angle = 0;
        if (g_robot_joints[index].joint_index == 2U) {
            angle = 45;
        } else if (g_robot_joints[index].joint_index == 3U) {
            angle = 90;
        }

        if (!robot_angle_to_position(index, angle, &targets[index])) {
            return false;
        }
    }

    return true;
}
