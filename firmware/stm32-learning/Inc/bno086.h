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
 * SPI1 and the four control pins are declared in stm32-learning.ioc and set
 * up by the CubeMX code, the same way servo_bus borrows huart1.  The pin
 * labels come from main.h:
 *
 *   SCK  D13 PA5   SPI1_SCK    (also drives the LD2 user LED)
 *   SO   D12 PA6   SPI1_MISO
 *   SI   D11 PA7   SPI1_MOSI
 *   CS   D10 PB6   IMU_CS      output, active low
 *   INT  D7  PA8   IMU_INT     input, active low
 *   RST  D3  PB3   IMU_RST     output, active low
 *   WAK  D4  PB5   IMU_WAKE    output, active low (PS0 in SPI mode)
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
    /* Owned by CubeMX as hspi1; this module only borrows it. */
    SPI_HandleTypeDef *spi;

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
Bno086Result bno086_init(Bno086 *imu,
                         SPI_HandleTypeDef *spi,
                         uint32_t report_interval_us);

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
 * Result of one bring-up probe, for the "imuprobe" console command.
 *
 * bno086_init() collapses every bring-up failure into NOT_PRESENT, which does
 * not say whether the part is silent, mis-wired or answering on the wrong
 * interface.  This is the IMU counterpart of busprobe/linestate on the servo
 * bus: it separates those cases without an oscilloscope.
 */
typedef struct
{
    uint32_t int_float_percent;     /* H_INTN sampled with no pull */
    uint32_t int_pulldown_percent;  /* ... and against an internal pull-down */
    /*
     * H_INTN against a pull-down while RST is held low.  The sensor tri-states
     * its outputs in reset, so this separates a pin the sensor drives from one
     * a board pull-up holds high: only the former follows the pull-down here.
     */
    uint32_t int_in_reset_percent;
    bool int_asserted;              /* did H_INTN go low after a reset? */
    uint32_t assert_delay_ms;
    bool header_read;
    uint8_t header[4];              /* raw SHTP header bytes, if any */

    /*
     * Header read without waiting for H_INTN at all.  This is what decides
     * between a dead SPI link and a dead interrupt wire: a sensor that boots
     * into SPI has an advertisement queued, so the bytes come back non-blank
     * even if H_INTN never reaches the MCU.
     */
    bool blind_data_seen;
    uint32_t blind_attempts;
    uint8_t blind_header[4];

    /*
     * The same four bytes clocked with CS left high.  A live sensor releases
     * MISO when deselected, so these should differ from the CS-low read.  If
     * both come back identical, nothing is driving MISO and the wire, not the
     * protocol, is the fault.
     */
    uint8_t deselected_header[4];
} Bno086Probe;

/*
 * Pulse reset and report what the sensor did.  Safe to call whether or not
 * bno086_init() succeeded, and it moves no servos.
 */
void bno086_probe(Bno086 *imu, Bno086Probe *probe);

/*
 * Loopback SPI1 with D11/PA7 MOSI jumpered to D12/PA6 MISO, the sensor
 * disconnected.  This is the SPI counterpart of uarttest: it proves the
 * peripheral, the three signal pins and the clock settings independently of
 * the BNO086, so a silent sensor can be blamed on one side or the other.
 *
 * The jumper is checked as plain DC first.  A pattern mismatch means nothing
 * until the two pins are known to be connected -- an unseated jumper fails
 * the SPI pattern exactly like a broken peripheral would, and reading that as
 * a peripheral fault sends the search to the wrong side of the link.
 */
typedef struct
{
    bool continuity;   /* PA6 followed PA7 driven as plain GPIO */
    bool miso_driven;  /* PA6 beat an internal pull-down: something drives it */
    int failed_byte;   /* -1 when the SPI pattern matched end to end */
    uint8_t sent;
    uint8_t received;
} Bno086Loopback;

void bno086_loopback_test(Bno086 *imu, Bno086Loopback *result);

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
