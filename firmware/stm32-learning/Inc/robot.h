#ifndef ROBOT_H
#define ROBOT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "robot_config.h"
#include "servo_bus.h"

#include <stdint.h>

typedef enum
{
    ROBOT_OK = 0,
    ROBOT_INVALID_ARGUMENT,
    ROBOT_CONFIG_ERROR,
    ROBOT_BUS_ERROR,
    ROBOT_MISSING_SERVO,
    ROBOT_POSITION_LIMIT,
    ROBOT_MOVE_TOO_LARGE,
    ROBOT_VERIFY_ERROR
} RobotResult;

typedef struct
{
    ServoBus *bus;
    ServoBusResult last_bus_result;
    uint8_t last_failed_servo_id;
} RobotController;

void robot_init(RobotController *robot, ServoBus *bus);

RobotResult robot_require_all(RobotController *robot);

RobotResult robot_read_positions(
    RobotController *robot,
    uint16_t positions[ROBOT_JOINT_COUNT]);

RobotResult robot_hold(RobotController *robot);
RobotResult robot_relax(RobotController *robot);
RobotResult robot_stand(RobotController *robot);

RobotResult robot_move_single_safe(RobotController *robot,
                                   uint8_t servo_id,
                                   uint16_t target_position,
                                   uint16_t maximum_delta);

const char *robot_result_string(RobotResult result);

#ifdef __cplusplus
}
#endif

#endif
