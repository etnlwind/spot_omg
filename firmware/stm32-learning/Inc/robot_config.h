#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define ROBOT_JOINT_COUNT 12U

/* BNO055 values that represent a physically level robot body (0.1 degree). */
#define ROBOT_IMU_LEVEL_ROLL_TENTHS   0
#define ROBOT_IMU_LEVEL_PITCH_TENTHS  0

typedef struct
{
    uint8_t servo_id;
    uint8_t leg_index;
    uint8_t joint_index;
    uint16_t center;
    uint16_t minimum;
    uint16_t maximum;
    int8_t direction;
} RobotJointConfig;

extern const RobotJointConfig g_robot_joints[ROBOT_JOINT_COUNT];
extern const uint8_t g_robot_servo_ids[ROBOT_JOINT_COUNT];

bool robot_config_valid(void);

bool robot_angle_to_position(size_t joint_array_index,
                             int16_t angle_degrees,
                             uint16_t *position);
bool robot_angle_tenths_to_position(size_t joint_array_index,
                                    int16_t angle_tenths,
                                    uint16_t *position);

bool robot_stand_targets(uint16_t targets[ROBOT_JOINT_COUNT]);

/* Canonical landing stance: J1=0, J2=40, J3=130 degrees. */
bool robot_landing_targets(uint16_t targets[ROBOT_JOINT_COUNT]);

/* Canonical zero on every joint: both links in line, foot below the hip. */
bool robot_straight_targets(uint16_t targets[ROBOT_JOINT_COUNT]);

/*
 * Index of the first joint whose target sits closer than margin_ticks to a
 * configured limit, or ROBOT_JOINT_COUNT when every joint has room.  A pose
 * that parks a joint against its stop drives it into the hard stop instead of
 * reaching the angle, which is worth refusing rather than discovering.
 */
size_t robot_pose_clearance(const uint16_t targets[ROBOT_JOINT_COUNT],
                            uint16_t margin_ticks,
                            uint16_t *clearance);

#ifdef __cplusplus
}
#endif

#endif
