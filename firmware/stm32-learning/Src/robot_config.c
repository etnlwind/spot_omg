#include "robot_config.h"

#include "sts3215.h"

/*
 * Generated from tools/servo_tool/config/joints.json.
 * Leg index: 0=FL, 1=FR, 2=RL, 3=RR.
 * Joint index: 1=abduction, 2=upper leg, 3=knee.
 *
 * The front knee servos are installed with their horns 180 degrees round, and
 * the two entries for J3 on legs 0 and 1 say so: centre shifted by 2048 ticks
 * and direction reversed, which is the same mapping their rear counterparts
 * use once that half turn is taken out.  The legs themselves are identical
 * front to rear, so nothing else about them differs.
 *
 * The stand targets are unchanged by this -- 1036 for servo 3 and 3001 for
 * servo 6, exactly as before -- because a half turn plus a reversal is the
 * identity at the pose it was calibrated from.  What changes is everything
 * either side of it: the knees now travel the same way as the rear ones.
 */
const RobotJointConfig g_robot_joints[ROBOT_JOINT_COUNT] = {
    {1U,  0U, 1U, 2091U, 0U, 4095U,  1},
    {2U,  0U, 2U, 2235U, 0U, 4095U, -1},
    {3U,  0U, 3U,   12U, 0U, 4095U,  1},
    {4U,  1U, 1U, 2087U, 0U, 4095U, -1},
    {5U,  1U, 2U, 1999U, 0U, 4095U,  1},
    {6U,  1U, 3U, 4025U, 0U, 4095U, -1},
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
 * All four alike, because this robot's front and rear legs are not mirror
 * images: both knees point the same way, as the unified rearward-knee URDF
 * already assumed.  Earlier values here encoded a mirror that is not there.
 *
 * The old {1, 1, -1, -1} was self-contradictory, which the policy output shows
 * plainly.  Under it FL and RL came out at identical joint angles while the
 * policy marked one in stance and the other in swing -- impossible, since equal
 * angles put both feet at the same height.  With uniform signs the diagonal
 * partners agree instead: FL and RR match to 0.00 degrees and share a stance
 * phase, which is what a trot is.
 *
 * test_firmware_c.py asserts that invariant against this array, so a future
 * edit that breaks the pairing fails in the suite rather than on the robot.
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
