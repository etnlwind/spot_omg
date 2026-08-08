#ifndef BNO086_H
#define BNO086_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#include <stdbool.h>
#include <stdint.h>

/*
 * BNO086 attitude source over SPI, replacing the BNO055 on I2C1.
 *
 * The sensor computes the fusion itself and pulls H_INTN low when a report is
 * ready.  Reports are drained in the main loop and cached, so the balance loop
 * reads attitude from RAM instead of blocking on a bus transfer mid-gait as
 * the BNO055 path did.
 *
 * Wiring on the NUCLEO-F446RE Arduino header:
 *
 *   SCK  D13 PA5   SPI1_SCK    (also drives the LD2 user LED)
 *   SO   D12 PA6   SPI1_MISO
 *   SI   D11 PA7   SPI1_MOSI
 *   CS   D10 PB6   GPIO output, active low
 *   INT  D7  PA8   GPIO EXTI, active low
 *   RST  D3  PB3   GPIO output, active low
 *   WAK  D4  PB5   GPIO output, active low  (PS0 in SPI mode)
 *
 * D2/PA10 must stay clear: it is USART1_RX from the URT-2.
 */

typedef enum
{
    BNO086_OK = 0,
    BNO086_NOT_PRESENT,   /* reset produced no advertisement */
    BNO086_TIMEOUT,       /* H_INTN never asserted */
    BNO086_SPI_ERROR,
    BNO086_PROTOCOL_ERROR
} Bno086Result;

typedef struct
{
    SPI_HandleTypeDef spi;

    /* Latest Game Rotation Vector, Q14 fixed point as the sensor reports it. */
    int16_t quat_i;
    int16_t quat_j;
    int16_t quat_k;
    int16_t quat_real;

    int16_t roll_tenths;
    int16_t pitch_tenths;
    int16_t yaw_tenths;

    uint32_t report_count;
    uint32_t last_report_tick;
    uint32_t protocol_errors;

    uint8_t sequence[6];  /* outgoing SHTP sequence number per channel */
    bool present;
    bool has_attitude;
} Bno086;

/*
 * Reset the sensor, bring up SPI1 and subscribe to the Game Rotation Vector at
 * the requested interval.  Returns BNO086_NOT_PRESENT when the part does not
 * answer, which leaves the caller free to run open loop.
 */
Bno086Result bno086_init(Bno086 *imu, uint32_t report_interval_us);

/*
 * Drain every SHTP packet the sensor has queued.  Call from the main loop; it
 * returns immediately when H_INTN is high.  Safe to call at any rate.
 *
 * H_INTN is polled rather than wired to EXTI.  Both this and the attitude
 * reader test the pin level, so an ISR would add shared mutable state without
 * shortening the path to a fresh sample.  Move to EXTI if the control loop
 * ever becomes timer driven and has to know precisely when a report landed.
 */
void bno086_service(Bno086 *imu);

/*
 * RobotAttitudeReader implementation.  Reads the cached angles, so it never
 * blocks and never touches the SPI bus.
 */
bool bno086_read_attitude(void *context,
                          int16_t *roll_tenths,
                          int16_t *pitch_tenths);

const char *bno086_result_string(Bno086Result result);

#ifdef __cplusplus
}
#endif

#endif
