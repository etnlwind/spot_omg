#ifndef S_NATIVE_SERVO_H
#define S_NATIVE_SERVO_H
#include "locomotion_servo.h"

/* S-native IK returns CAD joint angles, not calibrated motor angles.
 * FR ID4 increasing ticks was observed to adduct on 2026-09-10; CAD J1
 * decreases to adduct. FL is its mirrored front assembly. Rear J1 follows
 * CAD (user observation 2026-09-15). Keep the measured motor calibration
 * and legacy/Stow commands unchanged; convert only at the native boundary.
 * See docs/FR-J1-TURN-CLEARANCE-2026-09-10.md.
 */
static inline bool s_native_servo_targets(const GaitPolicyLegTarget cad[4],
                                           uint16_t ticks[12]) {
    if(!cad || !ticks)return false;
    GaitPolicyLegTarget motor[4];
    for(int leg=0;leg<4;leg++) {
        motor[leg]=cad[leg];
        if(leg<2)motor[leg].j1_deg=-cad[leg].j1_deg;
    }
    return locomotion_servo_targets(motor,ticks);
}
#endif
