#ifndef STS3215_H
#define STS3215_H

#ifdef __cplusplus
extern "C" {
#endif

#include "servo_bus.h"
#include "motor_capability.h"

#include <stdbool.h>
#include <stdint.h>

#define STS3215_MIN_POSITION              MOTOR_STS3215_MIN_POSITION
#define STS3215_MAX_POSITION              MOTOR_STS3215_MAX_POSITION
#define STS3215_STEPS_PER_REVOLUTION      MOTOR_STS3215_STEPS_PER_REVOLUTION

#define STS3215_ADDR_ID                   5U
#define STS3215_ADDR_TORQUE_ENABLE        40U
#define STS3215_ADDR_ACCELERATION         41U
#define STS3215_ADDR_GOAL_POSITION        42U
#define STS3215_ADDR_PRESENT_POSITION     56U
#define STS3215_ADDR_PRESENT_CURRENT      69U

typedef struct
{
    uint16_t position;
    int16_t speed;
    int16_t load;
    uint16_t voltage_mv;
    uint8_t temperature_c;
    uint8_t hardware_error;
    bool moving;
    int16_t current;
} Sts3215State;

ServoBusResult sts3215_ping(ServoBus *bus, uint8_t servo_id);

ServoBusResult sts3215_set_torque(ServoBus *bus,
                                  uint8_t servo_id,
                                  bool enabled);

ServoBusResult sts3215_read_position(ServoBus *bus,
                                     uint8_t servo_id,
                                     uint16_t *position);

ServoBusResult sts3215_read_state(ServoBus *bus,
                                  uint8_t servo_id,
                                  Sts3215State *state);

ServoBusResult sts3215_write_position(ServoBus *bus,
                                      uint8_t servo_id,
                                      uint16_t position,
                                      uint16_t speed,
                                      uint8_t acceleration);

ServoBusResult sts3215_sync_move(ServoBus *bus,
                                 const uint8_t *servo_ids,
                                 const uint16_t *positions,
                                 size_t servo_count,
                                 uint16_t speed,
                                 uint8_t acceleration);

ServoBusResult sts3215_sync_positions(ServoBus *bus,
                                      const uint8_t *servo_ids,
                                      const uint16_t *positions,
                                      size_t servo_count);

#ifdef __cplusplus
}
#endif

#endif
