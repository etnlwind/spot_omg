#ifndef SERVO_RESPONSE_PROBE_H
#define SERVO_RESPONSE_PROBE_H
#include "robot.h"
typedef struct {
    uint8_t stage, failed_id;
    bool restored, hold_sent;
    uint8_t original[7], applied[7], final[7];
    int16_t peak_roll_tenths, peak_pitch_tenths;
    uint16_t temperature_suspects, temperature_recovered;
} ServoResponseProbeReport;
/* RR J3 only, positive raw delta = knee flexion. Never changes torque or J2.
 * Normal exit restores the exact goal/profile bytes; interrupted runs hold
 * only the measured RR J3 position and retain the other eleven goals. */
RobotResult robot_probe_rr_j3(RobotController *r, uint8_t acceleration,
                              uint16_t delta_ticks, ServoResponseProbeReport *report);
#endif
