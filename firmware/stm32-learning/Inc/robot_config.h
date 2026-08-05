#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define ROBOT_JOINT_COUNT 12U

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

bool robot_stand_targets(uint16_t targets[ROBOT_JOINT_COUNT]);

#ifdef __cplusplus
}
#endif

#endif
