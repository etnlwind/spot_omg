#ifndef SAFETY_H
#define SAFETY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * Stall detection for the walking gaits.
 *
 * The servo bus runs from a 12 V 80 W PD supply, about 6.7 A, and one STS3215
 * stalls at roughly 2.7 A.  A leg caught on an obstacle therefore takes the
 * whole rail down within a few hundred milliseconds: every servo browns out at
 * once, which is worse than any single joint failing, because the robot drops
 * with all twelve unpowered.  This detector exists to cut torque deliberately
 * before the supply does it for us.
 *
 * What separates a stall from normal walking is not how large the load is.
 * A foot landing under body weight reads a high load while still *reaching its
 * target*.  A caught leg reads a high load while its position error refuses to
 * shrink.  So position error is the primary signal here and load or current
 * only corroborates it -- which also means no gait-phase load baseline is
 * needed, and nothing goes stale when the gait is retuned.
 *
 * This module is deliberately free of any bus or HAL dependency so the whole
 * decision path can be exercised on the host, the way gait_policy.h is.
 */

#define SAFETY_JOINT_COUNT 12U

typedef enum
{
    SAFETY_OK = 0,
    SAFETY_FAULT_STALL,        /* tracking error held with high effort */
    SAFETY_FAULT_HARDWARE,     /* servo raised its own hardware error bits */
    SAFETY_FAULT_OVERHEAT
} SafetyFaultKind;

typedef struct
{
    /*
     * Position error, in encoder ticks, that a joint must exceed before it is
     * a stall candidate.  The step barrier already treats 48 ticks as "caught
     * up", so this sits well above normal tracking lag.
     */
    uint16_t position_error_ticks;

    /*
     * Effort that must accompany the error.  Either signal alone is normal:
     * a swinging leg can lag without load, and a landing leg loads up without
     * lagging.  Together and sustained, they are a stall.
     */
    uint16_t load_magnitude;
    uint16_t current_magnitude;

    /*
     * How long the pair must hold, in milliseconds.  Landing shock and the
     * load spike at the start of a swing are both far shorter than this; the
     * point of the window is that no single sample can trip the fault.
     */
    uint16_t sustain_ms;

    uint8_t temperature_limit_c;

    /*
     * Readings at or above this are treated as a corrupt sample and ignored
     * rather than believed.  An STS3215 protects itself long before its die
     * reaches such a value, so a number up here says the byte is wrong, not
     * that the servo is that hot.
     */
    uint8_t temperature_implausible_c;

    /*
     * A servo's own hardware-error bits and its temperature still have to be
     * seen twice before they count.  They need no sustained window the way a
     * stall does, but one corrupt frame must not be able to stop the robot.
     */
    uint16_t confirm_ms;
} SafetyLimits;

typedef struct
{
    uint8_t servo_id;
    uint8_t leg_index;      /* 0=FL 1=FR 2=RL 3=RR */
    uint8_t joint_index;    /* 1=abduction 2=upper 3=knee */
    uint16_t target;
    uint16_t actual;
    uint16_t position_error;
    int16_t load;
    int16_t current;
    uint8_t temperature_c;
    uint8_t hardware_error;
    uint16_t duration_ms;
    uint16_t gait_phase;    /* 0..999, as the gait loop counts it */
} SafetyFaultRecord;

typedef struct
{
    SafetyLimits limits;

    /*
     * One candidate at a time.  A stall is a physical event at one joint, and
     * tracking every joint separately would buy nothing while making the
     * "how long has this been going on" question ambiguous.
     */
    uint8_t candidate_servo_id;
    SafetyFaultKind candidate_kind;
    uint32_t candidate_since_ms;
    uint32_t candidate_last_seen_ms;

    SafetyFaultKind fault;
    SafetyFaultRecord record;

    uint16_t stall_candidates_seen;  /* candidates that never reached the fault */
    uint16_t peak_position_error;
    uint16_t implausible_samples;    /* readings discarded as corrupt */
} SafetyMonitor;

/* One joint's state as sampled from the bus, plus what it was commanded. */
typedef struct
{
    uint8_t servo_id;
    uint8_t leg_index;
    uint8_t joint_index;
    uint16_t target;
    uint16_t actual;
    int16_t load;
    int16_t current;
    uint8_t temperature_c;
    uint8_t hardware_error;
} SafetySample;

/* Defaults chosen against the measured gait; see safety.c for the reasoning. */
void safety_limits_default(SafetyLimits *limits);

void safety_init(SafetyMonitor *monitor, const SafetyLimits *limits);

/* Drop the latched fault and every candidate.  Only an operator asks for this. */
void safety_clear(SafetyMonitor *monitor);

static inline bool safety_is_faulted(const SafetyMonitor *monitor)
{
    return monitor != NULL && monitor->fault != SAFETY_OK;
}

/*
 * Feed one sampled joint.  Returns true when this sample latched a fault, so
 * the caller can stop commanding immediately rather than at the next check.
 *
 * now_ms is the caller's millisecond clock; gait_phase is recorded for the
 * fault report only.
 */
bool safety_update(SafetyMonitor *monitor,
                   const SafetySample *sample,
                   uint32_t now_ms,
                   uint16_t gait_phase);

/*
 * True while the joint is a stall candidate but has not yet held long enough.
 * The gait loop uses this to keep sampling the same joint instead of carrying
 * on round-robin, so confirmation takes one frame rather than a full sweep.
 */
bool safety_watching(const SafetyMonitor *monitor, uint8_t *servo_id);

const char *safety_fault_string(SafetyFaultKind fault);

#ifdef __cplusplus
}
#endif

#endif
