#include "robot.h"
#include "sts3215.h"
#include <string.h>

void robot_latch_locomotion_fault(RobotController *r, RobotResult reason)
{
    if (!r) return;
    r->locomotion_fault = true;
    r->locomotion_fault_reason = reason;
    r->shared_idle = false;
    r->drive_target_linear = r->drive_target_yaw = 0;
    r->drive_stop_requested = true;
}

/* Called only for a fresh foreground command, never a heartbeat or idle tick.
 * Read-only preflight: do not resume targets or enable torque during recovery.
 * IMU/tilt checks remain in the gait; posture commands can attempt repositioning. */
RobotResult robot_prepare_new_command(RobotController *r)
{
    if (!r || r->drive_active) return ROBOT_INVALID_ARGUMENT;
    if (!r->locomotion_fault && !safety_is_faulted(&r->safety)) return ROBOT_OK;
    r->shared_idle = false;
    r->drive_target_linear = r->drive_target_yaw = 0;
    r->drive_stop_requested = true;
    for (unsigned i = 0; i < ROBOT_JOINT_COUNT; ++i) {
        Sts3215State state = {0};
        uint8_t id = g_robot_servo_ids[i];
        ServoBusResult bus = sts3215_read_state(r->bus, id, &state);
        if (bus != SERVO_BUS_OK) {
            r->last_failed_servo_id = id;
            r->last_bus_result = bus;
            robot_latch_locomotion_fault(r, ROBOT_BUS_ERROR);
            return ROBOT_BUS_ERROR;
        }
        if (state.hardware_error || state.temperature_c >= r->safety.limits.temperature_limit_c ||
            state.position > STS3215_MAX_POSITION) {
            r->last_failed_servo_id = id;
            robot_latch_locomotion_fault(r, ROBOT_SAFETY_FAULT);
            return ROBOT_SAFETY_FAULT;
        }
    }
    safety_clear(&r->safety);
    r->locomotion_fault = false;
    r->locomotion_fault_reason = ROBOT_OK;
    r->motion_abort_requested = false;
    memset(&r->drive_control, 0, sizeof(r->drive_control));
    memset(&r->shared_attitude, 0, sizeof(r->shared_attitude));
    memset(&r->shared_balance, 0, sizeof(r->shared_balance));
    return ROBOT_OK;
}
