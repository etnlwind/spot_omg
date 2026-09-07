#include "robot.h"
#include "sts3215.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

typedef enum {
    NORMAL, STALL, WRITE_FAIL, READ_FAIL, ABORT, STUCK,
    BAD_ENCODER, SLOW_BUS, HOLD_DRIFT, HOT, TILT, IMU_LOST, START_TILT
} Scenario;
static Scenario scenario;
static RobotController robot;
static ServoBus bus;
static uint16_t actual[12], last_target[12], initial[12];
static uint32_t tick, writes, holds, relaxes;
static bool torque;

static bool read_attitude(void *context, int16_t *roll, int16_t *pitch)
{
    (void)context;
    if (scenario == IMU_LOST && writes >= 20U) {
        return false;
    }
    *roll = (scenario == START_TILT || (scenario == TILT && writes >= 20U))
        ? 160 : 0;
    *pitch = 0;
    return true;
}

uint32_t HAL_GetTick(void) { return tick; }
void HAL_Delay(uint32_t ms) { tick += ms; }

RobotResult robot_read_positions(RobotController *r, uint16_t *positions)
{
    assert(r == &robot);
    memcpy(positions, actual, sizeof(actual));
    return ROBOT_OK;
}

RobotResult robot_hold(RobotController *r)
{
    assert(r == &robot);
    ++holds;
    torque = true;
    if (scenario == HOLD_DRIFT) {
        actual[0] += 200U;
    }
    return ROBOT_OK;
}

RobotResult robot_relax(RobotController *r)
{
    assert(r == &robot);
    ++relaxes;
    torque = false;
    return ROBOT_OK;
}

ServoBusResult sts3215_sync_positions(ServoBus *b, const uint8_t *ids,
                                      const uint16_t *positions, size_t count)
{
    assert(b == &bus && torque && count == 12U);
    ++writes;
    if (scenario == WRITE_FAIL) {
        return SERVO_BUS_TIMEOUT;
    }
    for (size_t i = 0; i < count; ++i) {
        assert(ids[i] == i + 1U);
        int delta = (int)positions[i] - last_target[i];
        assert(delta >= -4 && delta <= 4); /* no seam jump, even on a slow bus */
        assert(positions[i] >= 128U && positions[i] <= 3967U);
        const RobotJointConfig *j = &g_robot_joints[i];
        if (j->joint_index == 1U) {
            assert(positions[i] == initial[i]);
        } else if (j->joint_index == 2U) {
            assert(delta * j->direction <= 0); /* operator-confirmed front */
        } else {
            assert(delta * j->direction <= 0);
        }
        last_target[i] = positions[i];
        if (!((scenario == STALL && i == 8U) ||
              (scenario == STUCK && i == 1U))) {
            actual[i] = positions[i];
        }
    }
    if (scenario == ABORT && writes == 20U) {
        robot.motion_abort_requested = true;
    }
    return SERVO_BUS_OK;
}

ServoBusResult sts3215_read_state(ServoBus *b, uint8_t id, Sts3215State *state)
{
    assert(b == &bus && id >= 1U && id <= 12U);
    tick += scenario == SLOW_BUS ? 55U : 10U;
    if (scenario == READ_FAIL) {
        return SERVO_BUS_TIMEOUT;
    }
    memset(state, 0, sizeof(*state));
    state->position = actual[id - 1U];
    state->voltage_mv = 11200U;
    state->temperature_c = scenario == HOT ? 80U : 35U;
    if (scenario == STALL && id == 9U) {
        state->load = 800;
    }
    if (scenario == BAD_ENCODER) {
        state->position = 0x8001U;
    }
    return SERVO_BUS_OK;
}

static void reset(Scenario next)
{
    scenario = next;
    memset(&robot, 0, sizeof(robot));
    robot.bus = &bus;
    robot.profile_speed = 3400U;
    robot.profile_acceleration = 254U;
    robot.attitude_reader = read_attitude;
    safety_init(&robot.safety, NULL);
    assert(robot_stand_targets(actual));
    memcpy(initial, actual, sizeof(actual));
    memcpy(last_target, actual, sizeof(actual));
    writes = holds = relaxes = tick = 0U;
    torque = false;
}

static void success(void)
{
    assert(robot_forward_straight(&robot) == ROBOT_OK);
    assert(writes == 1200U && holds == 1U && relaxes == 0U && torque);
    for (size_t i = 0; i < 12U; ++i) {
        uint16_t expected = initial[i];
        if (g_robot_joints[i].joint_index != 1U) {
            assert(robot_angle_to_position(i,
                g_robot_joints[i].joint_index == 2U ? -90 : 0, &expected));
        }
        assert(actual[i] == expected);
    }
    assert(robot.profile_speed == 3400U && robot.profile_acceleration == 254U);
}

int main(void)
{
    reset(NORMAL);
    success();
    assert(tick >= 24000U);

    reset(NORMAL);
    /* Measured starting offsets are held; no reset to a stale canonical pose. */
    for (size_t i = 0; i < 12U; ++i) {
        actual[i] += i % 2U ? 19U : 11U;
    }
    memcpy(initial, actual, sizeof(actual));
    memcpy(last_target, actual, sizeof(actual));
    success();

    reset(SLOW_BUS);
    success();
    assert(tick > 60000U);

    reset(NORMAL);
    tick = UINT32_MAX - 100U; /* timer rollover */
    success();

    reset(NORMAL);
    assert(robot_landing_targets(actual));
    assert(robot_forward_straight(&robot) == ROBOT_STAND_REQUIRED);
    assert(writes == 0U && holds == 0U && relaxes == 0U);

    reset(NORMAL);
    robot.safety.fault = SAFETY_FAULT_STALL;
    assert(robot_forward_straight(&robot) == ROBOT_SAFETY_FAULT);
    assert(holds == 0U && writes == 0U);

    reset(NORMAL);
    robot.drive_active = true;
    assert(robot_forward_straight(&robot) == ROBOT_INVALID_ARGUMENT);
    assert(holds == 0U && writes == 0U);
    assert(robot_forward_straight(NULL) == ROBOT_INVALID_ARGUMENT);

    reset(START_TILT);
    assert(robot_forward_straight(&robot) == ROBOT_TILT_LIMIT);
    assert(holds == 0U && writes == 0U);

    reset(NORMAL);
    robot.attitude_reader = NULL;
    assert(robot_forward_straight(&robot) == ROBOT_IMU_ERROR);
    assert(holds == 0U && writes == 0U);

    const Scenario holds_position[] = {ABORT, TILT, IMU_LOST};
    const RobotResult hold_results[] = {ROBOT_MOTION_ABORTED, ROBOT_TILT_LIMIT,
                                       ROBOT_IMU_ERROR};
    for (size_t i = 0; i < 3U; ++i) {
        reset(holds_position[i]);
        assert(robot_forward_straight(&robot) == hold_results[i]);
        assert(torque && holds == 2U && relaxes == 0U && writes == 20U);
    }

    const Scenario cases[] = {STALL, WRITE_FAIL, READ_FAIL, STUCK,
                               BAD_ENCODER, HOLD_DRIFT, HOT};
    const RobotResult expected[] = {ROBOT_SAFETY_FAULT, ROBOT_BUS_ERROR,
        ROBOT_BUS_ERROR, ROBOT_VERIFY_ERROR,
        ROBOT_POSITION_LIMIT, ROBOT_STAND_REQUIRED, ROBOT_SAFETY_FAULT};
    for (size_t i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        reset(cases[i]);
        assert(robot_forward_straight(&robot) == expected[i]);
        assert(!torque && relaxes == 1U);
        assert(writes <= 1200U); /* no auto-return-to-stand or retry */
    }
    puts("forward pose: synchronized negative J2, no wrap, faults and abort OK");
    return 0;
}
