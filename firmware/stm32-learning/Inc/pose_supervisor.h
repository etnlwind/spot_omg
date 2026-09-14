#ifndef POSE_SUPERVISOR_H
#define POSE_SUPERVISOR_H
#include <stdint.h>
#define POSE_TRACE_COUNT 24U
typedef enum {
 POSE_IDLE, POSE_MOVING, POSE_WAITING, POSE_COMPLETE, POSE_NO_PROGRESS,
 POSE_OBSTRUCTION_SUSPECTED, POSE_FEEDBACK_LOST, POSE_WRITE_FAILED,
 POSE_ENCODER_INVALID, POSE_LOW_VOLTAGE, POSE_SERVO_FAULT,
 POSE_ATTITUDE_UNAVAILABLE, POSE_UNSTABLE, POSE_CANCELLED, POSE_HOLD_FAILED,
 POSE_COMPLETE_RESIDUAL, POSE_LOAD_SUPPORTED
} PoseDecision;
typedef struct {
 uint32_t ms;
 uint16_t actual,target,error,voltage_mv;
 int16_t load,current,roll,pitch;
 uint8_t id,reason;
} PoseObservation;
typedef struct {
 PoseDecision reason;
 uint32_t elapsed_ms,nominal_ms;
 uint16_t retries,waits,health_recovered,effort_probes,probe_mask;
 uint8_t suspect_id,suspect_temperature,suspect_hardware;
 uint8_t count,next;
 uint8_t hold_verified; /* 0: not attempted, 1: position target sent, 2: verified, 3: failed */
 PoseObservation trace[POSE_TRACE_COUNT];
} PoseDiagnostics;
const char *pose_decision_name(PoseDecision reason);
#endif
