#ifndef ROBOT_H
#define ROBOT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "robot_config.h"
#include "safety.h"
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
    ROBOT_VERIFY_ERROR,
    ROBOT_STEP_SYNC_ERROR,
    ROBOT_IMU_ERROR,
    ROBOT_MOTION_ABORTED,
    ROBOT_TILT_LIMIT,
    ROBOT_SAFETY_FAULT,      /* stall detector cut torque; latched */
    ROBOT_SERVO_POWER_LOST   /* nothing on the bus answers a ping */
} RobotResult;

typedef bool (*RobotAttitudeReader)(void *context,
                                    int16_t *roll_tenths,
                                    int16_t *pitch_tenths);

typedef enum
{
    ROBOT_BALANCE_NORMAL = 0,
    ROBOT_BALANCE_FULL
} RobotBalanceMode;

/*
 * Build-time defaults. They may be overridden with compiler -D options.
 * Balance is enabled only after a valid IMU reader has been registered.
 */
#ifndef ROBOT_IMU_BALANCE_DEFAULT_ENABLED
#define ROBOT_IMU_BALANCE_DEFAULT_ENABLED 1
#endif

#ifndef ROBOT_IMU_BALANCE_DEFAULT_MODE
#define ROBOT_IMU_BALANCE_DEFAULT_MODE ROBOT_BALANCE_FULL
#endif

/* Normalized jump travel: 0=in place; start forward experiments near 0.02. */
#ifndef ROBOT_JUMP_FORWARD_TRAVEL
#define ROBOT_JUMP_FORWARD_TRAVEL 0.0f
#endif

/* MuJoCo-tuned travel that cancels passive rearward drift during spot trot. */
#ifndef ROBOT_TROT_IN_PLACE_TRAVEL_SCALE
#define ROBOT_TROT_IN_PLACE_TRAVEL_SCALE 0.39f
#endif

typedef struct
{
    ServoBus *bus;
    ServoBusResult last_bus_result;
    uint8_t last_failed_servo_id;
    uint16_t profile_speed;
    uint8_t profile_acceleration;
    RobotAttitudeReader attitude_reader;
    void *attitude_context;
    bool balance_enabled;
    bool balance_required;
    RobotBalanceMode balance_mode;
    bool balance_reference_valid;
    int16_t balance_reference_roll_tenths;
    int16_t balance_reference_pitch_tenths;
    int16_t balance_last_roll_error_tenths;
    int16_t balance_last_pitch_error_tenths;
    int16_t balance_peak_roll_error_tenths;
    int16_t balance_peak_pitch_error_tenths;
    int16_t balance_peak_j1_correction_tenths;
    int16_t balance_peak_knee_correction_tenths;
    uint16_t balance_late_frames;
    uint16_t trot_step_sync_count;
    uint16_t trot_step_sync_wait_ms;
    uint16_t trot_step_sync_peak_error_ticks;
    volatile bool motion_abort_requested;

    /*
     * Stall detection.  The fault latches here rather than in the monitor's
     * caller so every motion entry point can refuse to start while it is set,
     * and only robot_recover() clears it.
     */
    SafetyMonitor safety;
    uint8_t safety_scan_index;   /* round-robin cursor over the twelve joints */
} RobotController;

void robot_init(RobotController *robot, ServoBus *bus);

bool robot_set_profile(RobotController *robot,
                       uint16_t speed,
                       uint8_t acceleration);

void robot_set_attitude_reader(RobotController *robot,
                               RobotAttitudeReader reader,
                               void *context);
bool robot_set_balance_enabled(RobotController *robot, bool enabled);
bool robot_set_balance_mode(RobotController *robot, RobotBalanceMode mode);
const char *robot_balance_mode_string(RobotBalanceMode mode);
void robot_request_motion_abort(RobotController *robot);

RobotResult robot_require_all(RobotController *robot);

RobotResult robot_read_positions(
    RobotController *robot,
    uint16_t positions[ROBOT_JOINT_COUNT]);

/*
 * Bring the robot back after a latched safety fault.
 *
 * Pings first, because a stall and a browned-out supply look nothing alike
 * from here and want opposite handling: with the rail down there is nobody to
 * send a torque command to, and reporting that plainly beats a bus timeout.
 * When the servos do answer, this holds them where they physically are rather
 * than at the gait target they were chasing when the fault hit -- otherwise
 * enabling torque would fling the joint at the position that caused it.
 */
RobotResult robot_recover(RobotController *robot);

RobotResult robot_hold(RobotController *robot);
RobotResult robot_relax(RobotController *robot);
RobotResult robot_stand(RobotController *robot);
RobotResult robot_stand_straight(RobotController *robot);
RobotResult robot_trot(RobotController *robot,
                       uint8_t cycles,
                       uint16_t period_ms);
RobotResult robot_trot_in_place(RobotController *robot,
                                uint8_t cycles,
                                uint16_t period_ms);
RobotResult robot_trot2(RobotController *robot,
                        uint8_t cycles,
                        uint16_t period_ms);
RobotResult robot_jump(RobotController *robot,
                       uint8_t cycles,
                       uint16_t period_ms);

RobotResult robot_move_single_safe(RobotController *robot,
                                   uint8_t servo_id,
                                   uint16_t target_position,
                                   uint16_t maximum_delta);

const char *robot_result_string(RobotResult result);

#ifdef __cplusplus
}
#endif

#endif
