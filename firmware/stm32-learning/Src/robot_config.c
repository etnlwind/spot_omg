#include "robot_config.h"

#include "sts3215.h"

/*
 * Generated from tools/servo_tool/config/joints.json.
 * Leg index: 0=FL, 1=FR, 2=RL, 3=RR.
 * Joint index: 1=abduction, 2=upper leg, 3=knee.
 *
 * These values come from calibrating the physical robot and are not derived
 * here.  On 2026-08-09 the mounting rule was confirmed: front and rear on the
 * same side use the same motor direction, while left and right are mirrored.
 * ID 3 and ID 6 were physically exchanged on 2026-08-09, restoring the
 * standard mapping: ID 3 is FL J3 and ID 6 is FR J3.  Centres remain attached
 * to their measured physical IDs and must be rechecked after remounting.
 */
const RobotJointConfig g_robot_joints[ROBOT_JOINT_COUNT] = {
    {1U,  0U, 1U, 2091U, 0U, 4095U, -1},
    {2U,  0U, 2U, 2235U, 0U, 4095U, -1},
    {3U,  0U, 3U, 2060U, 0U, 4095U,  1},
    {4U,  1U, 1U, 2087U, 0U, 4095U,  1},
    {5U,  1U, 2U, 1999U, 0U, 4095U,  1},
    {6U,  1U, 3U, 1977U, 0U, 4095U, -1},
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

/* Build a whole-robot pose from one canonical angle per joint number. */
static bool robot_pose_targets(int16_t upper_deg,
                               int16_t knee_deg,
                               uint16_t targets[ROBOT_JOINT_COUNT])
{
    if (targets == NULL || !robot_config_valid()) {
        return false;
    }

    for (size_t index = 0U; index < ROBOT_JOINT_COUNT; ++index) {
        int16_t angle = 0;
        if (g_robot_joints[index].joint_index == 2U) {
            angle = upper_deg;
        } else if (g_robot_joints[index].joint_index == 3U) {
            angle = knee_deg;
        }

        if (!robot_angle_to_position(index, angle, &targets[index])) {
            return false;
        }
    }

    return true;
}

bool robot_stand_targets(uint16_t targets[ROBOT_JOINT_COUNT])
{
    return robot_pose_targets(45, 90, targets);
}

bool robot_straight_targets(uint16_t targets[ROBOT_JOINT_COUNT])
{
    /*
     * Every joint at canonical zero, which puts both links of each leg in
     * line and the foot straight below the hip.  Unlike the stand pose this
     * is a physically unambiguous reference: a leg either looks straight or
     * it does not, so it is the pose to check a calibration against.
     */
    return robot_pose_targets(0, 0, targets);
}

size_t robot_pose_clearance(const uint16_t targets[ROBOT_JOINT_COUNT],
                            uint16_t margin_ticks,
                            uint16_t *clearance)
{
    if (targets == NULL) {
        return ROBOT_JOINT_COUNT;
    }
    for (size_t index = 0U; index < ROBOT_JOINT_COUNT; ++index) {
        const uint16_t low = targets[index] - g_robot_joints[index].minimum;
        const uint16_t high = g_robot_joints[index].maximum - targets[index];
        const uint16_t nearest = low < high ? low : high;
        if (nearest < margin_ticks) {
            if (clearance != NULL) {
                *clearance = nearest;
            }
            return index;
        }
    }
    return ROBOT_JOINT_COUNT;
}
