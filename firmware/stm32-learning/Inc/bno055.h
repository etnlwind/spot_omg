#ifndef BNO055_H
#define BNO055_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#include <stdbool.h>
#include <stdint.h>

/*
 * BNO055 attitude source over I2C1.
 *
 * This was the original IMU, replaced by a BNO086 on SPI1 that never answered
 * -- see HARDWARE_TEST_LOG.md.  It is back as a module rather than the loose
 * functions it used to be inside main.c, so the two sensors present the same
 * shape and main.c can pick whichever is actually attached.
 *
 * Wiring on the NUCLEO-F446RE Arduino header:
 *
 *   VIN  3V3   NUCLEO 3V3
 *   GND        NUCLEO GND
 *   SCL  D15   PB8   I2C1_SCL
 *   SDA  D14   PB9   I2C1_SDA
 *   COM3       GND   selects address 0x28
 *
 * The Euler output is 16 LSB per degree and the mounting matches the robot
 * frame, so no axis remap is applied.  Unlike the BNO086's game rotation
 * vector, yaw here is an absolute 0..359.9 heading.
 */

typedef struct
{
    I2C_HandleTypeDef *i2c;
    uint16_t address;     /* 8-bit form, as HAL wants it; 0 when absent */
    bool present;
} Bno055;

/*
 * Probe both possible addresses, verify the chip ID and enter NDOF fusion.
 * Returns false when the part does not answer, leaving the caller free to try
 * another sensor or run open loop.
 */
bool bno055_init(Bno055 *imu, I2C_HandleTypeDef *i2c);

/*
 * RobotAttitudeReader implementation.  Reads over I2C on the spot: the part
 * has no interrupt line here, so there is nothing to cache from.
 */
bool bno055_read_attitude(void *context,
                          int16_t *roll_tenths,
                          int16_t *pitch_tenths);

/*
 * Walk the 7-bit address space and report what answers.  This is the I2C
 * counterpart of busprobe on the servo bus: it separates "nothing on the bus"
 * from "something is there but it is not a BNO055", which bno055_init()
 * collapses into a single false.
 *
 * Writes up to `capacity` 7-bit addresses into `found` and returns how many
 * devices answered.
 */
uint8_t bno055_scan(I2C_HandleTypeDef *i2c, uint8_t *found, uint8_t capacity);

/* Roll, pitch and yaw in tenths of a degree.  False when the read fails. */
bool bno055_read_euler(Bno055 *imu,
                       int16_t *yaw_tenths,
                       int16_t *roll_tenths,
                       int16_t *pitch_tenths);

#ifdef __cplusplus
}
#endif

#endif
