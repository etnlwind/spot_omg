#include "robot.h"

#include "actuator_control.h"
#include "safety.h"

#include "feetech_protocol.h"
#include "gait_policy.h"
#include "sts3215.h"

#include <stdbool.h>
#include <string.h>

#define ROBOT_PROFILE_SPEED_DEFAULT          3400U
#define ROBOT_PROFILE_ACCELERATION_DEFAULT   254U
#define ROBOT_VERIFY_TOLERANCE       120U
#define ROBOT_VERIFY_TIMEOUT_MS      2000U
#define ROBOT_VERIFY_POLL_MS         100U
#define ROBOT_TROT_FRAME_MS          20U

/*
 * How long the straighten pose is given to arrive before its torque is cut.
 * Generous next to the move itself, and short next to how long a servo can
 * lean on a hard stop.
 */
#define ROBOT_STRAIGHTEN_TIMEOUT_MS 2500U
#define ROBOT_TROT_RAMP_MS            500U
#define ROBOT_TROT3_RAMP_MS           700U
#define ROBOT_BALANCE_ERROR_LIMIT     300
#define ROBOT_BALANCE_RATE_LIMIT      1200
#define ROBOT_BALANCE_IMU_FAILURES    3U
#define ROBOT_TROT_TILT_LIMIT         120
#define ROBOT_TROT_TILT_FRAMES          2U
#define ROBOT_TROT_STEP_SYNC_TOLERANCE 48U
#define ROBOT_JUMP_FRAME_MS             (1000U / GAIT_POLICY_JUMP_CONTROL_HZ)
#define ROBOT_JUMP_TILT_LIMIT           300
#define ROBOT_JUMP_IMU_FAILURES           3U
#define ROBOT_JUMP_MAX_CYCLES            20U

static RobotResult bus_failure(RobotController *robot,
                               uint8_t servo_id,
                               ServoBusResult result)
{
    robot->last_failed_servo_id = servo_id;
    robot->last_bus_result = result;
    return ROBOT_BUS_ERROR;
}

static const RobotJointConfig *config_for_servo(uint8_t servo_id)
{
    for (size_t index = 0U; index < ROBOT_JOINT_COUNT; ++index) {
        if (g_robot_servo_ids[index] == servo_id) {
            return &g_robot_joints[index];
        }
    }
    return NULL;
}

void robot_init(RobotController *robot, ServoBus *bus)
{
    if (robot == NULL) {
        return;
    }

    robot->bus = bus;
    robot->last_bus_result = SERVO_BUS_OK;
    robot->last_failed_servo_id = 0U;
    robot->profile_speed = ROBOT_PROFILE_SPEED_DEFAULT;
    robot->profile_acceleration = ROBOT_PROFILE_ACCELERATION_DEFAULT;
    robot->attitude_reader = NULL;
    robot->attitude_context = NULL;
    robot->balance_enabled = false;
#if ROBOT_IMU_BALANCE_DEFAULT_ENABLED
    robot->balance_required = true;
#else
    robot->balance_required = false;
#endif
    robot->balance_mode = ROBOT_IMU_BALANCE_DEFAULT_MODE;
    robot->balance_reference_valid = false;
    robot->balance_reference_roll_tenths = 0;
    robot->balance_reference_pitch_tenths = 0;
    robot->balance_last_roll_error_tenths = 0;
    robot->balance_last_pitch_error_tenths = 0;
    robot->balance_peak_roll_error_tenths = 0;
    robot->balance_peak_pitch_error_tenths = 0;
    robot->balance_peak_j1_correction_tenths = 0;
    robot->balance_peak_knee_correction_tenths = 0;
    robot->balance_late_frames = 0U;
    robot->trot_step_sync_count = 0U;
    robot->trot_step_sync_miss_count = 0U;
    robot->trot_step_sync_wait_ms = 0U;
    robot->trot_step_sync_max_wait_ms = 0U;
    robot->trot_step_sync_peak_error_ticks = 0U;
    robot->balance_max_update_gap_ms = 0U;
    robot->gait_nominal_duration_ms = 0U;
    robot->gait_elapsed_ms = 0U;
    robot->gait_balance_was_enabled = false;
    memset(&robot->balance_trace, 0, sizeof(robot->balance_trace));
    memset(&robot->tilt_snapshot, 0, sizeof(robot->tilt_snapshot));
    robot->motion_abort_requested = false;
    robot->safety_scan_index = 0U;
    safety_init(&robot->safety, NULL);
    actuator_diagnostics_reset(&robot->gait_diagnostics);
    robot->gait_diagnostics_active = false;
    actuator_rate_limiter_init(&robot->trot3_limiter);
    robot->trot3_limited_frames = 0U;
    memset(&robot->limiter_diagnostics,
           0,
           sizeof(robot->limiter_diagnostics));
}

bool robot_set_profile(RobotController *robot,
                       uint16_t speed,
                       uint8_t acceleration)
{
    if (robot == NULL || speed == 0U || speed > 3400U ||
        acceleration > 254U) {
        return false;
    }

    robot->profile_speed = speed;
    robot->profile_acceleration = acceleration;
    return true;
}

void robot_set_attitude_reader(RobotController *robot,
                               RobotAttitudeReader reader,
                               void *context)
{
    if (robot == NULL) {
        return;
    }

    robot->attitude_reader = reader;
    robot->attitude_context = context;
#if ROBOT_IMU_BALANCE_DEFAULT_ENABLED
    robot->balance_enabled = reader != NULL;
    robot->balance_required = true;
#else
    robot->balance_enabled = false;
    robot->balance_required = false;
#endif
    robot->balance_reference_valid = false;
}

bool robot_set_balance_enabled(RobotController *robot, bool enabled)
{
    if (robot == NULL || (enabled && robot->attitude_reader == NULL)) {
        return false;
    }

    robot->balance_enabled = enabled;
    robot->balance_required = enabled;
    robot->balance_reference_valid = false;
    return true;
}

bool robot_set_balance_mode(RobotController *robot, RobotBalanceMode mode)
{
    if (robot == NULL || robot->attitude_reader == NULL ||
        (mode != ROBOT_BALANCE_NORMAL && mode != ROBOT_BALANCE_FULL)) {
        return false;
    }

    robot->balance_mode = mode;
    robot->balance_enabled = true;
    robot->balance_required = true;
    robot->balance_reference_valid = false;
    return true;
}

const char *robot_balance_mode_string(RobotBalanceMode mode)
{
    return mode == ROBOT_BALANCE_FULL ? "full" : "normal";
}

void robot_request_motion_abort(RobotController *robot)
{
    if (robot != NULL) {
        robot->motion_abort_requested = true;
    }
}

RobotResult robot_require_all(RobotController *robot)
{
    if (robot == NULL || robot->bus == NULL) {
        return ROBOT_INVALID_ARGUMENT;
    }
    if (!robot_config_valid()) {
        return ROBOT_CONFIG_ERROR;
    }

    for (size_t index = 0U; index < ROBOT_JOINT_COUNT; ++index) {
        const uint8_t id = g_robot_servo_ids[index];
        ServoBusResult result = sts3215_ping(robot->bus, id);
        if (result != SERVO_BUS_OK) {
            robot->last_failed_servo_id = id;
            robot->last_bus_result = result;
            return result == SERVO_BUS_TIMEOUT ? ROBOT_MISSING_SERVO
                                               : ROBOT_BUS_ERROR;
        }
    }

    return ROBOT_OK;
}

RobotResult robot_read_positions(
    RobotController *robot,
    uint16_t positions[ROBOT_JOINT_COUNT])
{
    if (robot == NULL || robot->bus == NULL || positions == NULL) {
        return ROBOT_INVALID_ARGUMENT;
    }

    for (size_t index = 0U; index < ROBOT_JOINT_COUNT; ++index) {
        const uint8_t id = g_robot_servo_ids[index];
        ServoBusResult result = sts3215_read_position(robot->bus,
                                                      id,
                                                      &positions[index]);
        if (result != SERVO_BUS_OK) {
            return bus_failure(robot, id, result);
        }
        if (positions[index] < g_robot_joints[index].minimum ||
            positions[index] > g_robot_joints[index].maximum) {
            robot->last_failed_servo_id = id;
            return ROBOT_POSITION_LIMIT;
        }
    }

    return ROBOT_OK;
}

RobotResult robot_relax(RobotController *robot)
{
    if (robot == NULL || robot->bus == NULL) {
        return ROBOT_INVALID_ARGUMENT;
    }

    RobotResult final_result = ROBOT_OK;
    for (size_t index = 0U; index < ROBOT_JOINT_COUNT; ++index) {
        const uint8_t id = g_robot_servo_ids[index];
        ServoBusResult result = sts3215_set_torque(robot->bus, id, false);
        if (result != SERVO_BUS_OK && final_result == ROBOT_OK) {
            final_result = bus_failure(robot, id, result);
        }
    }
    return final_result;
}

RobotResult robot_relax_servo(RobotController *robot, uint8_t servo_id)
{
    if (robot == NULL || robot->bus == NULL ||
        config_for_servo(servo_id) == NULL) {
        return ROBOT_INVALID_ARGUMENT;
    }

    const ServoBusResult result = sts3215_set_torque(robot->bus,
                                                      servo_id,
                                                      false);
    return result == SERVO_BUS_OK
        ? ROBOT_OK
        : bus_failure(robot, servo_id, result);
}

RobotResult robot_recover(RobotController *robot)
{
    if (robot == NULL || robot->bus == NULL) {
        return ROBOT_INVALID_ARGUMENT;
    }

    /*
     * Ping before anything else.  A latched stall and a supply that tripped
     * look identical from the console but want opposite handling: with the
     * rail down there is nothing to command, and a caller who is told
     * ROBOT_SERVO_POWER_LOST knows to restore power rather than to keep
     * retrying.  robot_require_all() already pings all twelve.
     */
    RobotResult result = robot_require_all(robot);
    if (result != ROBOT_OK) {
        return result == ROBOT_MISSING_SERVO || result == ROBOT_BUS_ERROR
            ? ROBOT_SERVO_POWER_LOST
            : result;
    }

    /*
     * Servos answer, so clear the latch and hold them where they physically
     * are.  robot_hold() reads the present positions and commands those before
     * enabling torque, which is exactly what must happen here: coming back at
     * the gait target the joint was chasing when it caught would drive it into
     * the obstacle again the moment torque returns.
     */
    safety_clear(&robot->safety);
    robot->safety_scan_index = 0U;
    robot->motion_abort_requested = false;
    return robot_hold(robot);
}

RobotResult robot_hold(RobotController *robot)
{
    uint16_t current[ROBOT_JOINT_COUNT];

    if (robot != NULL && safety_is_faulted(&robot->safety)) {
        return ROBOT_SAFETY_FAULT;
    }

    RobotResult result = robot_require_all(robot);
    if (result != ROBOT_OK) {
        return result;
    }

    result = robot_read_positions(robot, current);
    if (result != ROBOT_OK) {
        return result;
    }

    ServoBusResult bus_result = sts3215_sync_move(
        robot->bus,
        g_robot_servo_ids,
        current,
        ROBOT_JOINT_COUNT,
        robot->profile_speed,
        robot->profile_acceleration);
    if (bus_result != SERVO_BUS_OK) {
        return bus_failure(robot, FEETECH_BROADCAST_ID, bus_result);
    }

    size_t enabled = 0U;
    for (; enabled < ROBOT_JOINT_COUNT; ++enabled) {
        const uint8_t id = g_robot_servo_ids[enabled];
        bus_result = sts3215_set_torque(robot->bus, id, true);
        if (bus_result != SERVO_BUS_OK) {
            for (size_t rollback = 0U; rollback < enabled; ++rollback) {
                (void)sts3215_set_torque(robot->bus,
                                         g_robot_servo_ids[rollback],
                                         false);
            }
            return bus_failure(robot, id, bus_result);
        }
    }

    return ROBOT_OK;
}

/* Defined below with the gait helpers; used by the straighten pose. */
static RobotResult sample_joint(RobotController *robot,
                                size_t index,
                                const uint16_t targets[ROBOT_JOINT_COUNT],
                                uint16_t gait_phase,
                                uint16_t *position,
                                bool *tripped);

/*
 * Straighten every leg: both links in line, foot below the hip.
 *
 * This commands whatever the calibration currently calls zero, which is the
 * point -- seeing where the robot actually goes is how a wrong zero becomes
 * visible.  It does not refuse a pose that sits near a limit, because that
 * pose is exactly the one worth looking at.
 *
 * It does watch.  A joint whose zero is past its mechanical stop cannot arrive,
 * and will sit there pushing at stall current until the supply gives out, so
 * every joint is polled through the stall detector and torque is cut on all
 * twelve the moment one of them is straining without progress.  Joints that
 * simply run out of time are reported too, with torque off, since not arriving
 * means still pushing.
 */
RobotResult robot_stand_straight(RobotController *robot)
{
    uint16_t target[ROBOT_JOINT_COUNT];
    uint16_t position = 0U;

    if (robot == NULL || robot->bus == NULL) {
        return ROBOT_INVALID_ARGUMENT;
    }
    if (safety_is_faulted(&robot->safety)) {
        return ROBOT_SAFETY_FAULT;
    }
    if (!robot_straight_targets(target)) {
        return ROBOT_CONFIG_ERROR;
    }

    RobotResult result = robot_hold(robot);
    if (result != ROBOT_OK) {
        return result;
    }

    const ServoBusResult bus_result = sts3215_sync_positions(
        robot->bus, g_robot_servo_ids, target, ROBOT_JOINT_COUNT);
    if (bus_result != SERVO_BUS_OK) {
        return bus_failure(robot, FEETECH_BROADCAST_ID, bus_result);
    }

    const uint32_t started_at = HAL_GetTick();
    for (;;) {
        uint16_t worst_error = 0U;
        uint8_t worst_servo_id = 0U;

        for (size_t index = 0U; index < ROBOT_JOINT_COUNT; ++index) {
            bool tripped = false;
            result = sample_joint(robot, index, target, 0U, &position,
                                  &tripped);
            if (result != ROBOT_OK) {
                return result;
            }
            int32_t error = (int32_t)position - (int32_t)target[index];
            if (error < 0) {
                error = -error;
            }
            if ((uint32_t)error > worst_error) {
                worst_error = (uint16_t)error;
                worst_servo_id = g_robot_servo_ids[index];
            }
        }

        if (worst_error <= ROBOT_VERIFY_TOLERANCE) {
            return ROBOT_OK;
        }
        if ((uint32_t)(HAL_GetTick() - started_at) >=
            ROBOT_STRAIGHTEN_TIMEOUT_MS) {
            /* Still short of the pose, so still pushing: stop pushing. */
            (void)robot_relax(robot);
            robot->last_failed_servo_id = worst_servo_id;
            return ROBOT_VERIFY_ERROR;
        }
    }
}

typedef bool (*RobotPoseTargets)(uint16_t targets[ROBOT_JOINT_COUNT]);

static RobotResult robot_move_to_pose(RobotController *robot,
                                      RobotPoseTargets build_targets)
{
    /*
     * Gated because it commands positions.  robot_relax() deliberately is not:
     * cutting torque is the safe direction and must work in any state.
     * robot_recover() clears the latch before it holds, so it is unaffected.
     */
    if (robot != NULL && safety_is_faulted(&robot->safety)) {
        return ROBOT_SAFETY_FAULT;
    }
    uint16_t target[ROBOT_JOINT_COUNT];
    uint16_t measured_positions[ROBOT_JOINT_COUNT];

    if (robot == NULL || robot->bus == NULL || build_targets == NULL) {
        return ROBOT_INVALID_ARGUMENT;
    }
    if (!build_targets(target)) {
        return ROBOT_CONFIG_ERROR;
    }

    RobotResult result = robot_hold(robot);
    if (result != ROBOT_OK) {
        return result;
    }

    ServoBusResult bus_result = sts3215_sync_positions(robot->bus,
                                                       g_robot_servo_ids,
                                                       target,
                                                       ROBOT_JOINT_COUNT);
    if (bus_result != SERVO_BUS_OK) {
        return bus_failure(robot, FEETECH_BROADCAST_ID, bus_result);
    }

    const uint32_t verify_started_at = HAL_GetTick();
    for (;;) {
        result = robot_read_positions(robot, measured_positions);
        if (result != ROBOT_OK) {
            return result;
        }

        bool all_within_tolerance = true;
        for (size_t joint = 0U; joint < ROBOT_JOINT_COUNT; ++joint) {
            int32_t error = (int32_t)measured_positions[joint] -
                            (int32_t)target[joint];
            if (error < 0) {
                error = -error;
            }
            if ((uint32_t)error > ROBOT_VERIFY_TOLERANCE) {
                if (all_within_tolerance) {
                    robot->last_failed_servo_id = g_robot_servo_ids[joint];
                }
                all_within_tolerance = false;
            }
        }

        if (all_within_tolerance) {
            return ROBOT_OK;
        }
        if ((uint32_t)(HAL_GetTick() - verify_started_at) >=
            ROBOT_VERIFY_TIMEOUT_MS) {
            return ROBOT_VERIFY_ERROR;
        }
        HAL_Delay(ROBOT_VERIFY_POLL_MS);
    }
}

RobotResult robot_stand(RobotController *robot)
{
    return robot_move_to_pose(robot, robot_stand_targets);
}

RobotResult robot_landing(RobotController *robot)
{
    return robot_move_to_pose(robot, robot_landing_targets);
}

static uint16_t smootherstep_per_mille(uint16_t progress)
{
    const float normalized = progress >= 1000U ?
        1.0f : (float)progress / 1000.0f;
    const float scaled = gait_policy_smootherstep(normalized) * 1000.0f;
    const uint32_t rounded = (uint32_t)(scaled + 0.5f);
    return (uint16_t)(rounded > 1000U ? 1000U : rounded);
}

static uint16_t trot_amplitude_scale(uint32_t frame,
                                     uint32_t total_frames,
                                     uint32_t requested_ramp_ms)
{
    uint32_t ramp_frames = requested_ramp_ms / ROBOT_TROT_FRAME_MS;
    if (ramp_frames > total_frames / 2U) {
        ramp_frames = total_frames / 2U;
    }
    if (ramp_frames == 0U) {
        return 1000U;
    }

    uint32_t scale = 1000U;
    if (frame < ramp_frames) {
        scale = smootherstep_per_mille(
            (uint16_t)((frame * 1000U) / ramp_frames));
    }
    const uint32_t remaining = total_frames - frame;
    if (remaining < ramp_frames) {
        const uint32_t ending_scale = smootherstep_per_mille(
            (uint16_t)((remaining * 1000U) / ramp_frames));
        if (ending_scale < scale) {
            scale = ending_scale;
        }
    }
    return (uint16_t)scale;
}

static int16_t clamp_i16(int32_t value, int16_t limit)
{
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return (int16_t)-limit;
    }
    return (int16_t)value;
}

static int16_t degrees_to_tenths(float degrees)
{
    const float scaled = degrees * 10.0f;
    return (int16_t)(scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f);
}

static int16_t balance_rate_tenths_s(int16_t current, int16_t previous)
{
    const int32_t delta = (int32_t)current - (int32_t)previous;

    /*
     * ROBOT_TROT_FRAME_MS ultimately comes from an unsigned control-rate
     * constant.  Dividing a negative delta by it directly promotes the whole
     * expression to unsigned on Cortex-M, turning a small negative rate into
     * a huge positive value that then clamps at +120deg/s.  Keep the entire
     * derivative signed: decreasing Roll/Pitch must produce a negative D term.
     */
    return clamp_i16(
        delta * INT32_C(1000) / (int32_t)ROBOT_TROT_FRAME_MS,
        ROBOT_BALANCE_RATE_LIMIT);
}

static GaitPolicyBalanceConfig shared_balance_config(RobotBalanceMode mode)
{
    const bool full = mode == ROBOT_BALANCE_FULL;
    const GaitPolicyBalanceConfig config = {
        full ? 1.0f : 0.6f,
        0.04f,
        full ? 0.08f : 0.10f,
        full ? 15.0f : 5.0f,
        5.0f,
        0.0f,
        0.08f,
        true
    };
    return config;
}

static int16_t absolute_i16(int16_t value)
{
    return value < 0 ? (int16_t)-value : value;
}

static void update_peak(int16_t value, int16_t *peak)
{
    const int16_t magnitude = absolute_i16(value);
    if (peak != NULL && magnitude > *peak) {
        *peak = magnitude;
    }
}

static int16_t signed_larger_magnitude(int16_t current, int16_t candidate)
{
    return absolute_i16(candidate) > absolute_i16(current) ? candidate : current;
}

static int16_t float_to_i16_scaled(float value, float scale)
{
    const float scaled = value * scale;
    if (scaled >= (float)INT16_MAX) {
        return INT16_MAX;
    }
    if (scaled <= (float)INT16_MIN) {
        return INT16_MIN;
    }
    return (int16_t)(scaled >= 0.0f ? scaled + 0.5f : scaled - 0.5f);
}

static uint16_t float_magnitude_to_u16(float value, float scale)
{
    const float magnitude = fabsf(value) * scale;
    return magnitude >= (float)UINT16_MAX ?
        UINT16_MAX : (uint16_t)(magnitude + 0.5f);
}

static void balance_trace_reset(RobotController *robot)
{
    if (robot != NULL) {
        memset(&robot->balance_trace, 0, sizeof(robot->balance_trace));
        memset(&robot->tilt_snapshot, 0, sizeof(robot->tilt_snapshot));
    }
}

static void balance_trace_push(RobotController *robot,
                               const RobotBalanceTraceFrame *frame)
{
    if (robot == NULL || frame == NULL) {
        return;
    }
    robot->balance_trace.frames[robot->balance_trace.write_index] = *frame;
    robot->balance_trace.write_index = (uint8_t)(
        (robot->balance_trace.write_index + 1U) %
        ROBOT_BALANCE_TRACE_CAPACITY);
    if (robot->balance_trace.count < ROBOT_BALANCE_TRACE_CAPACITY) {
        ++robot->balance_trace.count;
    }
}

static float canonical_joint_angle(
    const float angles[ROBOT_JOINT_COUNT],
    uint8_t leg_index,
    uint8_t joint_index)
{
    for (size_t index = 0U; index < ROBOT_JOINT_COUNT; ++index) {
        if (g_robot_joints[index].leg_index == leg_index &&
            g_robot_joints[index].joint_index == joint_index) {
            return angles[index];
        }
    }
    return 0.0f;
}

static void canonical_foot_position(
    const float angles[ROBOT_JOINT_COUNT],
    uint8_t leg_index,
    float *forward,
    float *lateral,
    float *vertical)
{
    const float j1 = canonical_joint_angle(angles, leg_index, 1U) *
        GAIT_POLICY_PI / 180.0f;
    const float j2 = canonical_joint_angle(angles, leg_index, 2U);
    const float j3 = canonical_joint_angle(angles, leg_index, 3U);
    float down = 0.0f;
    gait_policy_leg_forward_kinematics(j2, j3, forward, &down);
    *lateral = down * sinf(j1);
    *vertical = down * cosf(j1);
}

static void limiter_diagnostics_update(
    RobotController *robot,
    const float requested[ROBOT_JOINT_COUNT],
    const ActuatorCommandFrame *command)
{
    if (robot == NULL || requested == NULL || command == NULL) {
        return;
    }
    RobotLimiterDiagnostics *diagnostics = &robot->limiter_diagnostics;
    if (diagnostics->frame_count < UINT32_MAX) {
        ++diagnostics->frame_count;
    }
    for (size_t index = 0U; index < ROBOT_JOINT_COUNT; ++index) {
        const uint16_t error_millideg = float_magnitude_to_u16(
            requested[index] - command->position_deg[index], 1000.0f);
        if (error_millideg > diagnostics->joint_peak_error_millideg[index]) {
            diagnostics->joint_peak_error_millideg[index] = error_millideg;
        }
        if (UINT32_MAX - diagnostics->joint_error_sum_millideg[index] >=
            error_millideg) {
            diagnostics->joint_error_sum_millideg[index] += error_millideg;
        } else {
            diagnostics->joint_error_sum_millideg[index] = UINT32_MAX;
        }
    }
    for (uint8_t leg = 0U; leg < GAIT_POLICY_LEG_COUNT; ++leg) {
        float requested_forward = 0.0f;
        float requested_lateral = 0.0f;
        float requested_vertical = 0.0f;
        float limited_forward = 0.0f;
        float limited_lateral = 0.0f;
        float limited_vertical = 0.0f;
        canonical_foot_position(requested,
                                leg,
                                &requested_forward,
                                &requested_lateral,
                                &requested_vertical);
        canonical_foot_position(command->position_deg,
                                leg,
                                &limited_forward,
                                &limited_lateral,
                                &limited_vertical);
        const float df = requested_forward - limited_forward;
        const float dl = requested_lateral - limited_lateral;
        const float dv = requested_vertical - limited_vertical;
        const uint16_t error_milli = float_magnitude_to_u16(
            sqrtf(df * df + dl * dl + dv * dv), 1000.0f);
        if (error_milli > diagnostics->foot_peak_error_milli[leg]) {
            diagnostics->foot_peak_error_milli[leg] = error_milli;
        }
        if (UINT32_MAX - diagnostics->foot_error_sum_milli[leg] >=
            error_milli) {
            diagnostics->foot_error_sum_milli[leg] += error_milli;
        } else {
            diagnostics->foot_error_sum_milli[leg] = UINT32_MAX;
        }
    }
}

static void tilt_snapshot_store(RobotController *robot,
                                const RobotBalanceTraceFrame *frame)
{
    if (robot == NULL || frame == NULL) {
        return;
    }
    robot->tilt_snapshot.valid = true;
    robot->tilt_snapshot.frame = *frame;
    robot->tilt_snapshot.minimum_voltage_mv =
        robot->gait_diagnostics.minimum_voltage_mv;
    uint16_t worst_magnitude = 0U;
    for (size_t index = 0U; index < ROBOT_JOINT_COUNT; ++index) {
        const ActuatorTrackingSample *sample =
            &robot->gait_diagnostics.joints[index].latest;
        const uint16_t magnitude = absolute_i16(sample->position_error);
        if (magnitude > worst_magnitude) {
            worst_magnitude = magnitude;
            robot->tilt_snapshot.worst_servo_id = sample->servo_id;
            robot->tilt_snapshot.worst_position_error_ticks =
                sample->position_error;
        }
    }
}

static void return_to_stand_best_effort(RobotController *robot)
{
    uint16_t stand_targets[ROBOT_JOINT_COUNT];
    /*
     * Deliberately does nothing once a safety fault is latched.  Every other
     * failure here wants the legs parked, but a stall wants the opposite:
     * torque is already off, and sending a stand target would re-command the
     * caught joint and put the current straight back.
     */
    if (robot != NULL && safety_is_faulted(&robot->safety)) {
        return;
    }
    if (robot != NULL && robot->bus != NULL &&
        robot_stand_targets(stand_targets)) {
        (void)sts3215_sync_positions(robot->bus,
                                     g_robot_servo_ids,
                                     stand_targets,
                                     ROBOT_JOINT_COUNT);
    }
}

static bool phase_starts_swing(uint16_t current_global_phase,
                               uint16_t next_global_phase,
                               uint16_t diagonal_offset,
                               uint16_t duty_phase)
{
    const uint16_t current_leg_phase = (uint16_t)(
        (current_global_phase + diagonal_offset) % 1000U);
    const uint16_t next_leg_phase = (uint16_t)(
        (next_global_phase + diagonal_offset) % 1000U);
    return current_leg_phase < duty_phase && next_leg_phase >= duty_phase;
}

/*
 * Cut power to every joint and latch the fault.
 *
 * All twelve, not just the offending one: a single leg going limp mid-stride
 * while the other three keep driving is a worse failure than stopping, because
 * the robot then falls under power instead of settling.
 *
 * Nothing here commands a position.  The whole point is to stop asking a
 * caught joint to move, so this must not be routed through the stand recovery
 * that the other failure paths use.
 */
static void safety_trip(RobotController *robot)
{
    for (size_t index = 0U; index < ROBOT_JOINT_COUNT; ++index) {
        (void)sts3215_set_torque(robot->bus, g_robot_servo_ids[index], false);
    }
    robot->last_failed_servo_id = robot->safety.record.servo_id;
}

/*
 * Read one joint and hand it to the detector.  Full state costs the same as
 * position alone here -- both pay the 10 ms bus settle in servo_bus_request,
 * and the extra thirteen bytes are 0.13 ms at 1 Mbps -- so there is no reason
 * to read less.
 *
 * *tripped is set when this sample latched the fault; torque is already off by
 * the time it returns.
 */
static RobotResult sample_joint(RobotController *robot,
                                size_t index,
                                const uint16_t targets[ROBOT_JOINT_COUNT],
                                uint16_t gait_phase,
                                uint16_t *position,
                                bool *tripped)
{
    Sts3215State state;
    const uint8_t id = g_robot_servo_ids[index];

    if (tripped != NULL) {
        *tripped = false;
    }

    ServoBusResult bus_result = sts3215_read_state(robot->bus, id, &state);
    if (bus_result != SERVO_BUS_OK) {
        return bus_failure(robot, id, bus_result);
    }
    if (position != NULL) {
        *position = state.position;
    }

    if (robot->gait_diagnostics_active) {
        const int32_t signed_error =
            (int32_t)targets[index] - (int32_t)state.position;
        const ActuatorTrackingSample tracking = {
            id,
            g_robot_joints[index].leg_index,
            g_robot_joints[index].joint_index,
            targets[index],
            state.position,
            (int16_t)signed_error,
            state.speed,
            state.current,
            state.load,
            state.voltage_mv,
            gait_phase
        };
        (void)actuator_diagnostics_update(
            &robot->gait_diagnostics, index, &tracking);
    }

    const SafetySample sample = {
        id,
        g_robot_joints[index].leg_index,
        g_robot_joints[index].joint_index,
        targets[index],
        state.position,
        state.load,
        state.current,
        state.temperature_c,
        state.hardware_error,
    };
    if (safety_update(&robot->safety, &sample, HAL_GetTick(), gait_phase)) {
        safety_trip(robot);
        if (tripped != NULL) {
            *tripped = true;
        }
        return ROBOT_SAFETY_FAULT;
    }
    return ROBOT_OK;
}

/*
 * Sample one joint per control frame.
 *
 * Reading all twelve takes about 123 ms, so it cannot happen inside a 20 ms
 * frame; this walks them instead, a sweep every 240 ms.  When a joint is
 * already a stall candidate it stays selected rather than advancing, which is
 * what keeps confirmation inside the sustain window instead of waiting for the
 * cursor to come round again.
 */
static RobotResult sample_next_joint(RobotController *robot,
                                     const uint16_t targets[ROBOT_JOINT_COUNT],
                                     uint16_t gait_phase)
{
    uint8_t watched_id = 0U;
    size_t index = robot->safety_scan_index % ROBOT_JOINT_COUNT;

    if (safety_watching(&robot->safety, &watched_id)) {
        for (size_t candidate = 0U; candidate < ROBOT_JOINT_COUNT;
             ++candidate) {
            if (g_robot_servo_ids[candidate] == watched_id) {
                index = candidate;
                break;
            }
        }
    } else {
        robot->safety_scan_index =
            (uint8_t)((index + 1U) % ROBOT_JOINT_COUNT);
    }

    return sample_joint(robot, index, targets, gait_phase, NULL, NULL);
}

static void observe_step_sync(
    RobotController *robot,
    const uint16_t targets[ROBOT_JOINT_COUNT])
{
    if (robot->trot_step_sync_count < UINT16_MAX) {
        ++robot->trot_step_sync_count;
    }

    uint16_t maximum_error = 0U;
    for (size_t index = 0U; index < ROBOT_JOINT_COUNT; ++index) {
        const ActuatorJointDiagnostics *joint =
            &robot->gait_diagnostics.joints[index];
        if (joint->sample_count == 0U) {
            continue;
        }
        int32_t error = (int32_t)joint->latest.measured_position -
                        (int32_t)targets[index];
        if (error < 0) {
            error = -error;
        }
        if ((uint32_t)error > maximum_error) {
            maximum_error = (uint16_t)error;
        }
    }
    if (maximum_error > robot->trot_step_sync_peak_error_ticks) {
        robot->trot_step_sync_peak_error_ticks = maximum_error;
    }
    if (maximum_error > ROBOT_TROT_STEP_SYNC_TOLERANCE &&
        robot->trot_step_sync_miss_count < UINT16_MAX) {
        ++robot->trot_step_sync_miss_count;
    }
}

static bool gait_policy_to_servo_targets(
    const GaitPolicyLegTarget leg_targets[GAIT_POLICY_LEG_COUNT],
    uint16_t servo_targets[ROBOT_JOINT_COUNT])
{
    if (leg_targets == NULL || servo_targets == NULL) {
        return false;
    }
    for (size_t index = 0U; index < ROBOT_JOINT_COUNT; ++index) {
        const RobotJointConfig *joint = &g_robot_joints[index];
        const GaitPolicyLegTarget *leg = &leg_targets[joint->leg_index];
        float angle = leg->j1_deg;
        if (joint->joint_index == 2U) {
            angle = leg->j2_deg;
        } else if (joint->joint_index == 3U) {
            angle = leg->j3_deg;
        }
        if (!robot_angle_tenths_to_position(
                index, degrees_to_tenths(angle), &servo_targets[index])) {
            return false;
        }
    }
    return true;
}

static bool gait_policy_to_canonical_angles(
    const GaitPolicyLegTarget leg_targets[GAIT_POLICY_LEG_COUNT],
    float angles_deg[ROBOT_JOINT_COUNT])
{
    if (leg_targets == NULL || angles_deg == NULL) {
        return false;
    }
    for (size_t index = 0U; index < ROBOT_JOINT_COUNT; ++index) {
        const RobotJointConfig *joint = &g_robot_joints[index];
        const GaitPolicyLegTarget *leg = &leg_targets[joint->leg_index];
        angles_deg[index] = leg->j1_deg;
        if (joint->joint_index == 2U) {
            angles_deg[index] = leg->j2_deg;
        } else if (joint->joint_index == 3U) {
            angles_deg[index] = leg->j3_deg;
        }
    }
    return true;
}

static bool canonical_angles_to_servo_targets(
    const float angles_deg[ROBOT_JOINT_COUNT],
    uint16_t servo_targets[ROBOT_JOINT_COUNT])
{
    if (angles_deg == NULL || servo_targets == NULL) {
        return false;
    }
    for (size_t index = 0U; index < ROBOT_JOINT_COUNT; ++index) {
        if (!isfinite(angles_deg[index]) ||
            !robot_angle_tenths_to_position(
                index,
                degrees_to_tenths(angles_deg[index]),
                &servo_targets[index])) {
            return false;
        }
    }
    return true;
}

typedef enum
{
    ROBOT_TROT_POLICY_LINEAR = 0,
    ROBOT_TROT_POLICY_CIRCULAR,
    ROBOT_TROT_POLICY_CIRCULAR_OVERLAP
} RobotTrotPolicy;

static bool robot_trot_policy_targets(
    RobotTrotPolicy policy,
    float phase,
    float amplitude_scale,
    float travel_scale,
    GaitPolicyLegTarget targets[GAIT_POLICY_LEG_COUNT])
{
    if (policy == ROBOT_TROT_POLICY_CIRCULAR_OVERLAP) {
        return gait_policy_trot3_targets(
            phase,
            amplitude_scale,
            GAIT_POLICY_TROT2_FOLD_J2_DEG,
            GAIT_POLICY_TROT2_FOLD_J3_DEG,
            targets);
    }
    if (policy == ROBOT_TROT_POLICY_CIRCULAR) {
        return gait_policy_trot2_targets(
            phase,
            amplitude_scale,
            GAIT_POLICY_TROT2_FOLD_J2_DEG,
            GAIT_POLICY_TROT2_FOLD_J3_DEG,
            targets);
    }
    return gait_policy_trot_targets(
        phase,
        amplitude_scale,
        travel_scale,
        targets);
}

static RobotResult robot_trot_scaled(RobotController *robot,
                                     uint8_t cycles,
                                     uint16_t period_ms,
                                     float travel_scale,
                                     RobotTrotPolicy policy,
                                     bool actuator_limited)
{
    uint16_t targets[ROBOT_JOINT_COUNT];
    float canonical_angles[ROBOT_JOINT_COUNT];
    int16_t filtered_roll_error = 0;
    int16_t filtered_pitch_error = 0;
    int16_t filtered_roll_rate = 0;
    int16_t filtered_pitch_rate = 0;
    int16_t previous_roll_error = 0;
    int16_t previous_pitch_error = 0;
    uint8_t consecutive_imu_failures = 0U;
    uint8_t consecutive_tilt_frames = 0U;
    GaitPolicyLegTarget leg_targets[GAIT_POLICY_LEG_COUNT];

    if (robot == NULL || robot->bus == NULL || cycles == 0U || cycles > 10U ||
        period_ms < 600U || period_ms > 5000U ||
        !isfinite(travel_scale) ||
        travel_scale < -1.0f || travel_scale > 1.0f) {
        return ROBOT_INVALID_ARGUMENT;
    }
    if (actuator_limited &&
        !actuator_profile_supports_trot3(
            robot->profile_speed, robot->profile_acceleration)) {
        return ROBOT_ACTUATOR_PROFILE_ERROR;
    }
    if (actuator_limited && period_ms > GAIT_POLICY_TROT3_MAX_PERIOD_MS) {
        return ROBOT_TROT3_PERIOD_ERROR;
    }
    if (robot->balance_required &&
        (!robot->balance_enabled || robot->attitude_reader == NULL)) {
        return ROBOT_IMU_ERROR;
    }

    if (safety_is_faulted(&robot->safety)) {
        return ROBOT_SAFETY_FAULT;
    }

    robot->motion_abort_requested = false;

    RobotResult result = robot_stand(robot);
    if (result != ROBOT_OK) {
        return result;
    }
    if (robot->motion_abort_requested) {
        return_to_stand_best_effort(robot);
        return ROBOT_MOTION_ABORTED;
    }

    /* Reference stance, so the unswapped table: this is what the legs start
     * from regardless of the diagnostic pairing. */
    if (!robot_trot_policy_targets(
            policy,
            0.0f,
            0.0f,
            travel_scale,
            leg_targets) ||
        !gait_policy_to_servo_targets(leg_targets, targets)) {
        return ROBOT_CONFIG_ERROR;
    }
    ServoBusResult bus_result = sts3215_sync_positions(
        robot->bus, g_robot_servo_ids, targets, ROBOT_JOINT_COUNT);
    if (bus_result != SERVO_BUS_OK) {
        return bus_failure(robot, FEETECH_BROADCAST_ID, bus_result);
    }
    HAL_Delay(300U);

    robot->balance_reference_valid = false;
    robot->balance_last_roll_error_tenths = 0;
    robot->balance_last_pitch_error_tenths = 0;
    robot->balance_peak_roll_error_tenths = 0;
    robot->balance_peak_pitch_error_tenths = 0;
    robot->balance_peak_j1_correction_tenths = 0;
    robot->balance_peak_knee_correction_tenths = 0;
    robot->balance_late_frames = 0U;
    robot->trot_step_sync_count = 0U;
    robot->trot_step_sync_miss_count = 0U;
    robot->trot_step_sync_wait_ms = 0U;
    robot->trot_step_sync_max_wait_ms = 0U;
    robot->trot_step_sync_peak_error_ticks = 0U;
    robot->balance_max_update_gap_ms = 0U;
    robot->gait_nominal_duration_ms = (uint32_t)period_ms * cycles;
    robot->gait_elapsed_ms = 0U;
    robot->gait_balance_was_enabled = robot->balance_enabled;
    balance_trace_reset(robot);
    actuator_diagnostics_reset(&robot->gait_diagnostics);
    robot->gait_diagnostics_active = true;
    actuator_rate_limiter_init(&robot->trot3_limiter);
    robot->trot3_limited_frames = 0U;
    memset(&robot->limiter_diagnostics,
           0,
           sizeof(robot->limiter_diagnostics));
    if (robot->attitude_reader != NULL) {
        int16_t initial_roll_tenths = 0;
        int16_t initial_pitch_tenths = 0;
        if (robot->attitude_reader == NULL ||
            !robot->attitude_reader(
                robot->attitude_context,
                &initial_roll_tenths,
                &initial_pitch_tenths)) {
            robot->gait_diagnostics_active = false;
            return ROBOT_IMU_ERROR;
        }
        if (robot->balance_mode == ROBOT_BALANCE_FULL) {
            robot->balance_reference_roll_tenths =
                ROBOT_IMU_LEVEL_ROLL_TENTHS;
            robot->balance_reference_pitch_tenths =
                ROBOT_IMU_LEVEL_PITCH_TENTHS;
        } else {
            robot->balance_reference_roll_tenths = initial_roll_tenths;
            robot->balance_reference_pitch_tenths = initial_pitch_tenths;
        }
        previous_roll_error = clamp_i16(
            (int32_t)initial_roll_tenths -
                robot->balance_reference_roll_tenths,
            ROBOT_BALANCE_ERROR_LIMIT);
        previous_pitch_error = clamp_i16(
            (int32_t)initial_pitch_tenths -
                robot->balance_reference_pitch_tenths,
            ROBOT_BALANCE_ERROR_LIMIT);
        robot->balance_reference_valid = true;
    }

    const uint32_t frames_per_cycle = period_ms / ROBOT_TROT_FRAME_MS;
    const uint32_t total_frames = frames_per_cycle * cycles;
    const uint32_t started_at = HAL_GetTick();
    uint32_t last_attitude_at = started_at;
    const GaitPolicyBalanceConfig balance_config =
        shared_balance_config(robot->balance_mode);

    for (uint32_t frame = 0U; frame <= total_frames; ++frame) {
        if (robot->motion_abort_requested) {
            robot->gait_diagnostics_active = false;
            return_to_stand_best_effort(robot);
            return ROBOT_MOTION_ABORTED;
        }
        const uint16_t global_phase = (uint16_t)(
            ((frame % frames_per_cycle) * 1000U) / frames_per_cycle);
        const uint16_t amplitude_scale = trot_amplitude_scale(
            frame,
            total_frames,
            actuator_limited ? ROBOT_TROT3_RAMP_MS : ROBOT_TROT_RAMP_MS);
        bool tilt_limit_pending = false;

        if (!robot_trot_policy_targets(
                policy,
                (float)global_phase / 1000.0f,
                (float)amplitude_scale / 1000.0f,
                travel_scale,
                leg_targets)) {
            robot->gait_diagnostics_active = false;
            return_to_stand_best_effort(robot);
            return ROBOT_CONFIG_ERROR;
        }

        GaitPolicyLegTarget open_loop_targets[GAIT_POLICY_LEG_COUNT];
        for (uint8_t leg = 0U; leg < GAIT_POLICY_LEG_COUNT; ++leg) {
            open_loop_targets[leg] = leg_targets[leg];
        }
        const uint8_t support_mask = gait_policy_support_mask(leg_targets);
        RobotBalanceTraceFrame trace_frame = {
            .phase = global_phase,
            .support_mask = support_mask,
            .balance_applied = robot->balance_enabled
        };

        if (robot->attitude_reader != NULL && frame > 0U) {
            int16_t roll_tenths = 0;
            int16_t pitch_tenths = 0;
            if (robot->attitude_reader(
                    robot->attitude_context,
                    &roll_tenths,
                    &pitch_tenths)) {
                const uint32_t attitude_at = HAL_GetTick();
                const uint32_t update_gap = attitude_at - last_attitude_at;
                if (update_gap > robot->balance_max_update_gap_ms) {
                    robot->balance_max_update_gap_ms =
                        update_gap > UINT16_MAX ?
                            UINT16_MAX : (uint16_t)update_gap;
                }
                last_attitude_at = attitude_at;
                const int16_t roll_error = clamp_i16(
                    (int32_t)roll_tenths -
                    robot->balance_reference_roll_tenths,
                    ROBOT_BALANCE_ERROR_LIMIT);
                const int16_t pitch_error = clamp_i16(
                    (int32_t)pitch_tenths -
                    robot->balance_reference_pitch_tenths,
                    ROBOT_BALANCE_ERROR_LIMIT);
                trace_frame.raw_roll_tenths = roll_error;
                trace_frame.raw_pitch_tenths = pitch_error;
                if (absolute_i16(roll_error) > ROBOT_TROT_TILT_LIMIT ||
                    absolute_i16(pitch_error) > ROBOT_TROT_TILT_LIMIT) {
                    if (consecutive_tilt_frames < UINT8_MAX) {
                        ++consecutive_tilt_frames;
                    }
                    if (consecutive_tilt_frames >= ROBOT_TROT_TILT_FRAMES) {
                        tilt_limit_pending = true;
                    }
                } else {
                    consecutive_tilt_frames = 0U;
                }
                const int16_t roll_rate = balance_rate_tenths_s(
                    roll_error, previous_roll_error);
                const int16_t pitch_rate = balance_rate_tenths_s(
                    pitch_error, previous_pitch_error);
                previous_roll_error = roll_error;
                previous_pitch_error = pitch_error;
                filtered_roll_error = (int16_t)(
                    ((3L * filtered_roll_error) + roll_error) / 4L);
                filtered_pitch_error = (int16_t)(
                    ((3L * filtered_pitch_error) + pitch_error) / 4L);
                filtered_roll_rate = (int16_t)(
                    ((3L * filtered_roll_rate) + roll_rate) / 4L);
                filtered_pitch_rate = (int16_t)(
                    ((3L * filtered_pitch_rate) + pitch_rate) / 4L);
                robot->balance_last_roll_error_tenths =
                    filtered_roll_error;
                robot->balance_last_pitch_error_tenths =
                    filtered_pitch_error;
                update_peak(filtered_roll_error,
                            &robot->balance_peak_roll_error_tenths);
                update_peak(filtered_pitch_error,
                            &robot->balance_peak_pitch_error_tenths);
                consecutive_imu_failures = 0U;
            } else {
                ++consecutive_imu_failures;
                if (consecutive_imu_failures >=
                    ROBOT_BALANCE_IMU_FAILURES) {
                    robot->gait_diagnostics_active = false;
                    return_to_stand_best_effort(robot);
                    return ROBOT_IMU_ERROR;
                }
            }
        }

        const float tenths_degrees_to_radians =
            GAIT_POLICY_PI / 1800.0f;
        const GaitPolicyImuSample sample = {
            (float)filtered_roll_error * tenths_degrees_to_radians,
            (float)filtered_pitch_error * tenths_degrees_to_radians,
            (float)filtered_roll_rate * tenths_degrees_to_radians,
            (float)filtered_pitch_rate * tenths_degrees_to_radians
        };
        const float roll_control = balance_config.kp * sample.roll +
            balance_config.kd * sample.roll_rate;
        const float pitch_control = balance_config.kp * sample.pitch +
            balance_config.kd * sample.pitch_rate;
        trace_frame.roll_tenths = filtered_roll_error;
        trace_frame.pitch_tenths = filtered_pitch_error;
        trace_frame.roll_rate_tenths_s = filtered_roll_rate;
        trace_frame.pitch_rate_tenths_s = filtered_pitch_rate;
        trace_frame.roll_control_millirad =
            robot->balance_enabled ?
                float_to_i16_scaled(roll_control, 1000.0f) : 0;
        trace_frame.pitch_control_millirad =
            robot->balance_enabled ?
                float_to_i16_scaled(pitch_control, 1000.0f) : 0;

        if (robot->balance_enabled) {
            static const int8_t side_signs[GAIT_POLICY_LEG_COUNT] = {
                1, -1, 1, -1
            };
            static const int8_t end_signs[GAIT_POLICY_LEG_COUNT] = {
                1, 1, -1, -1
            };
            for (uint8_t leg = 0U; leg < GAIT_POLICY_LEG_COUNT; ++leg) {
                const bool support =
                    (support_mask & (uint8_t)(1U << leg)) != 0U;
                float j1_effort = (float)side_signs[leg] * roll_control;
                if (balance_config.contact_aware && !support) {
                    j1_effort = -j1_effort;
                }
                const float raw_j1 =
                    balance_config.j1_gain_deg * j1_effort;
                if (fabsf(raw_j1) > balance_config.j1_limit_deg) {
                    trace_frame.saturation_flags |=
                        ROBOT_BALANCE_SATURATION_J1;
                }

                const float raw_length =
                    -(float)side_signs[leg] * roll_control +
                    (float)end_signs[leg] * pitch_control;
                const float length = gait_policy_clampf(
                    raw_length,
                    -balance_config.leg_length_limit,
                    balance_config.leg_length_limit);
                trace_frame.leg_length_correction_milli =
                    signed_larger_magnitude(
                        trace_frame.leg_length_correction_milli,
                        float_to_i16_scaled(length, 1000.0f));
                if (fabsf(raw_length) > balance_config.leg_length_limit) {
                    trace_frame.saturation_flags |=
                        ROBOT_BALANCE_SATURATION_LEG_LENGTH;
                }

                if (balance_config.contact_aware && !support) {
                    const float raw_placement =
                        balance_config.foot_placement_gain * pitch_control;
                    const float placement = gait_policy_clampf(
                        raw_placement,
                        -balance_config.foot_placement_limit,
                        balance_config.foot_placement_limit);
                    trace_frame.foot_placement_correction_milli =
                        signed_larger_magnitude(
                            trace_frame.foot_placement_correction_milli,
                            float_to_i16_scaled(placement, 1000.0f));
                    if (fabsf(raw_placement) >
                        balance_config.foot_placement_limit) {
                        trace_frame.saturation_flags |=
                            ROBOT_BALANCE_SATURATION_FOOT_PLACEMENT;
                    }
                }
            }
        }

        if (robot->balance_enabled) {
            if (!gait_policy_balance_targets(
                    &sample,
                    &balance_config,
                    support_mask,
                    leg_targets)) {
                robot->gait_diagnostics_active = false;
                return_to_stand_best_effort(robot);
                return ROBOT_CONFIG_ERROR;
            }
            for (uint8_t leg = 0U; leg < GAIT_POLICY_LEG_COUNT; ++leg) {
                const int16_t j1_correction = degrees_to_tenths(
                    leg_targets[leg].j1_deg -
                    open_loop_targets[leg].j1_deg);
                const int16_t knee_correction = degrees_to_tenths(
                    leg_targets[leg].j3_deg -
                    open_loop_targets[leg].j3_deg);
                trace_frame.j1_correction_tenths = signed_larger_magnitude(
                    trace_frame.j1_correction_tenths, j1_correction);
                trace_frame.knee_correction_tenths = signed_larger_magnitude(
                    trace_frame.knee_correction_tenths, knee_correction);
                update_peak(j1_correction,
                    &robot->balance_peak_j1_correction_tenths);
                update_peak(knee_correction,
                    &robot->balance_peak_knee_correction_tenths);
            }
        }

        if (tilt_limit_pending) {
            trace_frame.saturation_flags |= ROBOT_BALANCE_SATURATION_TILT;
            trace_frame.limited_joint_mask =
                robot->trot3_last_command.limited_joint_mask;
            trace_frame.tracking_lag_samples =
                robot->gait_diagnostics.lag_samples;
            balance_trace_push(robot, &trace_frame);
            tilt_snapshot_store(robot, &trace_frame);
            robot->gait_elapsed_ms = HAL_GetTick() - started_at;
            robot->gait_diagnostics_active = false;
            return_to_stand_best_effort(robot);
            return ROBOT_TILT_LIMIT;
        }

        if (actuator_limited) {
            if (!gait_policy_to_canonical_angles(
                    leg_targets, canonical_angles) ||
                !actuator_rate_limiter_apply(
                    &robot->trot3_limiter,
                    canonical_angles,
                    (float)period_ms /
                        ((float)frames_per_cycle * 1000.0f),
                    &robot->trot3_last_command) ||
                !canonical_angles_to_servo_targets(
                    robot->trot3_last_command.position_deg, targets)) {
                robot->gait_diagnostics_active = false;
                return_to_stand_best_effort(robot);
                return ROBOT_CONFIG_ERROR;
            }
            limiter_diagnostics_update(
                robot, canonical_angles, &robot->trot3_last_command);
            if (robot->trot3_last_command.limited_joint_mask != 0U &&
                robot->trot3_limited_frames < UINT32_MAX) {
                ++robot->trot3_limited_frames;
            }
        } else if (!gait_policy_to_servo_targets(leg_targets, targets)) {
            robot->gait_diagnostics_active = false;
            return_to_stand_best_effort(robot);
            return ROBOT_CONFIG_ERROR;
        }

        bus_result = sts3215_sync_positions(robot->bus,
                                            g_robot_servo_ids,
                                            targets,
                                            ROBOT_JOINT_COUNT);
        if (bus_result != SERVO_BUS_OK) {
            robot->gait_diagnostics_active = false;
            return_to_stand_best_effort(robot);
            return bus_failure(robot, FEETECH_BROADCAST_ID, bus_result);
        }

        /*
         * Sample inside the frame's own slack: the deadline below is absolute,
         * so a read that fits in the remaining budget does not move when the
         * next command goes out.
         */
        result = sample_next_joint(robot, targets, global_phase);
        if (result != ROBOT_OK) {
            robot->gait_diagnostics_active = false;
            return_to_stand_best_effort(robot);
            return result;
        }
        trace_frame.limited_joint_mask = actuator_limited ?
            robot->trot3_last_command.limited_joint_mask : 0U;
        trace_frame.tracking_lag_samples =
            robot->gait_diagnostics.lag_samples;
        balance_trace_push(robot, &trace_frame);

        if (frame < total_frames) {
            const uint32_t deadline = started_at +
                ((frame + 1U) * period_ms) / frames_per_cycle;
            const uint32_t now = HAL_GetTick();
            if ((int32_t)(deadline - now) > 0) {
                HAL_Delay(deadline - now);
            } else if (robot->balance_late_frames < UINT16_MAX) {
                ++robot->balance_late_frames;
            }

            const uint16_t next_global_phase = (uint16_t)(
                ((((frame + 1U) % frames_per_cycle) * 1000U) /
                 frames_per_cycle));
            const uint16_t duty_phase =
                policy == ROBOT_TROT_POLICY_CIRCULAR_OVERLAP ?
                    (uint16_t)(GAIT_POLICY_TROT3_DUTY * 1000.0f + 0.5f) :
                    500U;
            const bool step_starts =
                phase_starts_swing(
                    global_phase, next_global_phase, 0U, duty_phase) ||
                phase_starts_swing(
                    global_phase, next_global_phase, 500U, duty_phase);
            if (step_starts) {
                observe_step_sync(robot, targets);
            }
        }
    }

    robot->gait_diagnostics_active = false;
    robot->gait_elapsed_ms = HAL_GetTick() - started_at;
    /* The final phase already has zero gait amplitude and is therefore the
     * stand geometry. Keep its last balance/limiter output instead of issuing
     * an unbalanced raw stand command that removes up to 0.08 leg-length
     * correction in one packet while the body is still rolling. */
    return ROBOT_OK;
}

RobotResult robot_trot(RobotController *robot,
                       uint8_t cycles,
                       uint16_t period_ms)
{
    return robot_trot_scaled(
        robot, cycles, period_ms, 1.0f, ROBOT_TROT_POLICY_LINEAR, false);
}

RobotResult robot_trot_in_place(RobotController *robot,
                                uint8_t cycles,
                                uint16_t period_ms)
{
    return robot_trot_scaled(
        robot,
        cycles,
        period_ms,
        ROBOT_TROT_IN_PLACE_TRAVEL_SCALE,
        ROBOT_TROT_POLICY_LINEAR,
        false);
}

RobotResult robot_trot2(RobotController *robot,
                        uint8_t cycles,
                        uint16_t period_ms)
{
    return robot_trot_scaled(
        robot, cycles, period_ms, 1.0f, ROBOT_TROT_POLICY_CIRCULAR, false);
}

RobotResult robot_trot3(RobotController *robot,
                        uint8_t cycles,
                        uint16_t period_ms)
{
    return robot_trot_scaled(
        robot,
        cycles,
        period_ms,
        1.0f,
        ROBOT_TROT_POLICY_CIRCULAR_OVERLAP,
        true);
}

const ActuatorDiagnostics *robot_gait_diagnostics(
    const RobotController *robot)
{
    return robot == NULL ? NULL : &robot->gait_diagnostics;
}

RobotResult robot_jump(RobotController *robot,
                       uint8_t cycles,
                       uint16_t period_ms)
{
    uint16_t targets[ROBOT_JOINT_COUNT];
    GaitPolicyLegTarget leg_targets[GAIT_POLICY_LEG_COUNT];
    uint8_t consecutive_imu_failures = 0U;

    if (robot == NULL || robot->bus == NULL ||
        cycles > ROBOT_JUMP_MAX_CYCLES ||
        period_ms < 800U || period_ms > 5000U) {
        return ROBOT_INVALID_ARGUMENT;
    }
    if (robot->balance_required &&
        (!robot->balance_enabled || robot->attitude_reader == NULL)) {
        return ROBOT_IMU_ERROR;
    }

    if (safety_is_faulted(&robot->safety)) {
        return ROBOT_SAFETY_FAULT;
    }

    /*
     * No per-frame stall sampling here, unlike the trot gaits.  A jump is
     * ballistic: the legs are commanded past where the servos can follow, so
     * large tracking error under high load is the normal case and the stall
     * test would fire on every takeoff.  Separating a caught leg from an
     * intended one needs a jump-aware expectation this does not have yet, so
     * a jump only refuses to start while a fault is already latched.
     */
    robot->motion_abort_requested = false;
    RobotResult result = robot_stand(robot);
    if (result != ROBOT_OK) {
        return result;
    }
    if (robot->motion_abort_requested) {
        return_to_stand_best_effort(robot);
        return ROBOT_MOTION_ABORTED;
    }

    robot->balance_reference_valid = false;
    robot->balance_last_roll_error_tenths = 0;
    robot->balance_last_pitch_error_tenths = 0;
    robot->balance_peak_roll_error_tenths = 0;
    robot->balance_peak_pitch_error_tenths = 0;
    robot->balance_peak_j1_correction_tenths = 0;
    robot->balance_peak_knee_correction_tenths = 0;
    robot->balance_late_frames = 0U;
    robot->trot_step_sync_count = 0U;
    robot->trot_step_sync_miss_count = 0U;
    robot->trot_step_sync_wait_ms = 0U;
    robot->trot_step_sync_peak_error_ticks = 0U;
    if (robot->balance_enabled) {
        int16_t initial_roll_tenths = 0;
        int16_t initial_pitch_tenths = 0;
        if (robot->attitude_reader == NULL ||
            !robot->attitude_reader(robot->attitude_context,
                                    &initial_roll_tenths,
                                    &initial_pitch_tenths)) {
            return ROBOT_IMU_ERROR;
        }
        if (robot->balance_mode == ROBOT_BALANCE_FULL) {
            robot->balance_reference_roll_tenths =
                ROBOT_IMU_LEVEL_ROLL_TENTHS;
            robot->balance_reference_pitch_tenths =
                ROBOT_IMU_LEVEL_PITCH_TENTHS;
        } else {
            robot->balance_reference_roll_tenths = initial_roll_tenths;
            robot->balance_reference_pitch_tenths = initial_pitch_tenths;
        }
        robot->balance_reference_valid = true;
    }

    const uint32_t frames_per_cycle = period_ms / ROBOT_JUMP_FRAME_MS;
    const bool continuous = cycles == 0U;
    const uint32_t total_frames = frames_per_cycle * cycles;
    uint32_t next_deadline = HAL_GetTick();

    for (uint32_t frame = 0U;
         continuous || frame <= total_frames;
         ++frame) {
        if (robot->motion_abort_requested) {
            return_to_stand_best_effort(robot);
            return ROBOT_MOTION_ABORTED;
        }

        if (robot->balance_enabled && frame > 0U) {
            int16_t roll_tenths = 0;
            int16_t pitch_tenths = 0;
            if (robot->attitude_reader(robot->attitude_context,
                                       &roll_tenths,
                                       &pitch_tenths)) {
                const int32_t roll_error = (int32_t)roll_tenths -
                    robot->balance_reference_roll_tenths;
                const int32_t pitch_error = (int32_t)pitch_tenths -
                    robot->balance_reference_pitch_tenths;
                consecutive_imu_failures = 0U;
                if (roll_error < -ROBOT_JUMP_TILT_LIMIT ||
                    roll_error > ROBOT_JUMP_TILT_LIMIT ||
                    pitch_error < -ROBOT_JUMP_TILT_LIMIT ||
                    pitch_error > ROBOT_JUMP_TILT_LIMIT) {
                    return_to_stand_best_effort(robot);
                    return ROBOT_TILT_LIMIT;
                }
                robot->balance_last_roll_error_tenths =
                    (int16_t)roll_error;
                robot->balance_last_pitch_error_tenths =
                    (int16_t)pitch_error;
                update_peak((int16_t)roll_error,
                            &robot->balance_peak_roll_error_tenths);
                update_peak((int16_t)pitch_error,
                            &robot->balance_peak_pitch_error_tenths);
            } else {
                ++consecutive_imu_failures;
                if (consecutive_imu_failures >= ROBOT_JUMP_IMU_FAILURES) {
                    return_to_stand_best_effort(robot);
                    return ROBOT_IMU_ERROR;
                }
            }
        }

        const uint32_t cycle_frame = frame % frames_per_cycle;
        const float phase = (float)cycle_frame / (float)frames_per_cycle;
        if (!gait_policy_jump_targets(
                phase,
                ROBOT_JUMP_FORWARD_TRAVEL,
                leg_targets) ||
            !gait_policy_to_servo_targets(leg_targets, targets)) {
            return_to_stand_best_effort(robot);
            return ROBOT_CONFIG_ERROR;
        }

        ServoBusResult bus_result = sts3215_sync_positions(
            robot->bus, g_robot_servo_ids, targets, ROBOT_JOINT_COUNT);
        if (bus_result != SERVO_BUS_OK) {
            return_to_stand_best_effort(robot);
            return bus_failure(robot, FEETECH_BROADCAST_ID, bus_result);
        }

        if (!continuous && frame == total_frames) {
            break;
        }
        const uint32_t next_cycle_frame = cycle_frame + 1U;
        next_deadline +=
            (next_cycle_frame * period_ms) / frames_per_cycle -
            (cycle_frame * period_ms) / frames_per_cycle;
        const uint32_t now = HAL_GetTick();
        if ((int32_t)(next_deadline - now) > 0) {
            HAL_Delay(next_deadline - now);
        }
    }

    return_to_stand_best_effort(robot);
    return ROBOT_OK;
}

RobotResult robot_move_single_safe(RobotController *robot,
                                   uint8_t servo_id,
                                   uint16_t target_position,
                                   uint16_t maximum_delta)
{
    /*
     * Gated because this enables torque on one servo.  After a fault the rest
     * are off, and bringing a single joint back under power is the asymmetric
     * state the shutdown deliberately avoids.  Recovery goes through
     * robot_recover(), which brings all twelve back together.
     */
    if (robot != NULL && safety_is_faulted(&robot->safety)) {
        return ROBOT_SAFETY_FAULT;
    }
    uint16_t current = 0U;
    const RobotJointConfig *joint = config_for_servo(servo_id);

    if (robot == NULL || robot->bus == NULL ||
        joint == NULL || target_position < joint->minimum ||
        target_position > joint->maximum || maximum_delta == 0U) {
        return ROBOT_INVALID_ARGUMENT;
    }

    ServoBusResult bus_result = sts3215_ping(robot->bus, servo_id);
    if (bus_result != SERVO_BUS_OK) {
        return bus_failure(robot, servo_id, bus_result);
    }
    bus_result = sts3215_read_position(robot->bus, servo_id, &current);
    if (bus_result != SERVO_BUS_OK) {
        return bus_failure(robot, servo_id, bus_result);
    }
    if (current < joint->minimum || current > joint->maximum) {
        robot->last_failed_servo_id = servo_id;
        return ROBOT_POSITION_LIMIT;
    }

    const uint16_t delta = target_position > current
                               ? (uint16_t)(target_position - current)
                               : (uint16_t)(current - target_position);
    if (delta > maximum_delta) {
        return ROBOT_MOVE_TOO_LARGE;
    }

    bus_result = sts3215_write_position(robot->bus,
                                        servo_id,
                                        current,
                                        robot->profile_speed,
                                        robot->profile_acceleration);
    if (bus_result != SERVO_BUS_OK) {
        return bus_failure(robot, servo_id, bus_result);
    }
    bus_result = sts3215_set_torque(robot->bus, servo_id, true);
    if (bus_result != SERVO_BUS_OK) {
        return bus_failure(robot, servo_id, bus_result);
    }
    HAL_Delay(50U);

    bus_result = sts3215_write_position(robot->bus,
                                        servo_id,
                                        target_position,
                                        robot->profile_speed,
                                        robot->profile_acceleration);
    if (bus_result != SERVO_BUS_OK) {
        (void)sts3215_set_torque(robot->bus, servo_id, false);
        return bus_failure(robot, servo_id, bus_result);
    }

    return ROBOT_OK;
}

const char *robot_result_string(RobotResult result)
{
    switch (result) {
    case ROBOT_OK:
        return "ok";
    case ROBOT_INVALID_ARGUMENT:
        return "invalid argument";
    case ROBOT_CONFIG_ERROR:
        return "configuration error";
    case ROBOT_BUS_ERROR:
        return "servo bus error";
    case ROBOT_MISSING_SERVO:
        return "missing servo";
    case ROBOT_POSITION_LIMIT:
        return "position limit";
    case ROBOT_MOVE_TOO_LARGE:
        return "move exceeds safe delta";
    case ROBOT_VERIFY_ERROR:
        return "final position verification failed";
    case ROBOT_STEP_SYNC_ERROR:
        return "step synchronization timeout";
    case ROBOT_ACTUATOR_PROFILE_ERROR:
        return "servo profile too slow for trot3";
    case ROBOT_TROT3_PERIOD_ERROR:
        return "trot3 period is too slow for diagonal stability";
    case ROBOT_IMU_ERROR:
        return "IMU balance error";
    case ROBOT_MOTION_ABORTED:
        return "motion aborted";
    case ROBOT_SAFETY_FAULT:
        return "safety fault; torque off, run recover";
    case ROBOT_SERVO_POWER_LOST:
        return "servo power lost; restore supply then run recover";
    case ROBOT_TILT_LIMIT:
        return "tilt safety limit reached";
    default:
        return "unknown";
    }
}
