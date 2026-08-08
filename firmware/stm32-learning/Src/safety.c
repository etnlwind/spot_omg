#include "safety.h"

#include <string.h>

/*
 * Default limits, chosen against what the gait actually measures rather than
 * round numbers.
 *
 * position_error_ticks 240
 *   The step barrier calls a joint caught up at 48 ticks, and the worst error
 *   observed across a healthy unloaded run was 314 -- but that was transient,
 *   at a barrier, and it recovered.  240 sits above ordinary tracking lag and
 *   below the excursion a genuinely stuck joint reaches, and it only matters
 *   when it holds for sustain_ms.
 *
 * load_magnitude 500, current_magnitude 700
 *   STS3215 load is 0..1023 and current is reported in units of about 6.5 mA,
 *   so 700 is roughly 4.5 A -- well past the 2.7 A stall figure for one servo
 *   yet still short of what three joints need to trip the 6.7 A supply.
 *   Either signal satisfies the effort test: whichever the servo reports more
 *   honestly under a given fault, one of them will rise.
 *
 * sustain_ms 150
 *   Landing shock and the load step at the start of a swing last a few tens of
 *   milliseconds.  150 ms clears those comfortably and still leaves room
 *   before the supply's protection, which took several hundred milliseconds to
 *   act when a leg caught during trot2.
 */
#define SAFETY_DEFAULT_POSITION_ERROR_TICKS 240U
#define SAFETY_DEFAULT_LOAD_MAGNITUDE        500U
#define SAFETY_DEFAULT_CURRENT_MAGNITUDE     700U
#define SAFETY_DEFAULT_SUSTAIN_MS            150U
#define SAFETY_DEFAULT_TEMPERATURE_LIMIT_C    70U

/*
 * A reading this high is a corrupt byte, not a temperature.  An STS3215 cuts
 * its own torque long before the die could reach it, and the first bench run
 * produced exactly this: temp=150C on a joint sitting 17 ticks from target
 * with zero current, which stopped the robot for nothing.
 */
#define SAFETY_DEFAULT_TEMPERATURE_IMPLAUSIBLE_C 100U

/*
 * The servo's own verdicts -- hardware error bits and temperature -- do not
 * need the stall's sustained window, but they do need to survive one repeat.
 * Two samples of the same joint span 20 ms in the frame loop, so this only
 * rejects the single bad frame.
 */
#define SAFETY_DEFAULT_CONFIRM_MS 10U

/*
 * A candidate is dropped if it goes unseen for longer than this.  Round-robin
 * sampling means an unrelated joint can be looked at in between, and a stall
 * that has actually cleared should not keep its accumulated time.
 */
#define SAFETY_CANDIDATE_STALE_MS 400U

static uint16_t magnitude_u16(int16_t value)
{
    return value < 0 ? (uint16_t)(-(int32_t)value) : (uint16_t)value;
}

void safety_limits_default(SafetyLimits *limits)
{
    if (limits == NULL) {
        return;
    }
    limits->position_error_ticks = SAFETY_DEFAULT_POSITION_ERROR_TICKS;
    limits->load_magnitude = SAFETY_DEFAULT_LOAD_MAGNITUDE;
    limits->current_magnitude = SAFETY_DEFAULT_CURRENT_MAGNITUDE;
    limits->sustain_ms = SAFETY_DEFAULT_SUSTAIN_MS;
    limits->temperature_limit_c = SAFETY_DEFAULT_TEMPERATURE_LIMIT_C;
    limits->temperature_implausible_c =
        SAFETY_DEFAULT_TEMPERATURE_IMPLAUSIBLE_C;
    limits->confirm_ms = SAFETY_DEFAULT_CONFIRM_MS;
}

void safety_init(SafetyMonitor *monitor, const SafetyLimits *limits)
{
    if (monitor == NULL) {
        return;
    }
    memset(monitor, 0, sizeof(*monitor));
    if (limits != NULL) {
        monitor->limits = *limits;
    } else {
        safety_limits_default(&monitor->limits);
    }
}

void safety_clear(SafetyMonitor *monitor)
{
    if (monitor == NULL) {
        return;
    }
    monitor->fault = SAFETY_OK;
    monitor->candidate_servo_id = 0U;
    monitor->candidate_kind = SAFETY_OK;
    monitor->candidate_since_ms = 0U;
    monitor->candidate_last_seen_ms = 0U;
    memset(&monitor->record, 0, sizeof(monitor->record));
}

static void fill_record(SafetyMonitor *monitor,
                        const SafetySample *sample,
                        uint16_t position_error,
                        uint16_t duration_ms,
                        uint16_t gait_phase)
{
    monitor->record.servo_id = sample->servo_id;
    monitor->record.leg_index = sample->leg_index;
    monitor->record.joint_index = sample->joint_index;
    monitor->record.target = sample->target;
    monitor->record.actual = sample->actual;
    monitor->record.position_error = position_error;
    monitor->record.load = sample->load;
    monitor->record.current = sample->current;
    monitor->record.temperature_c = sample->temperature_c;
    monitor->record.hardware_error = sample->hardware_error;
    monitor->record.duration_ms = duration_ms;
    monitor->record.gait_phase = gait_phase;
}

/*
 * Track one candidate per joint-and-reason, and report when it has held long
 * enough.  Every fault kind goes through here: the sustained window differs,
 * but nothing is allowed to latch off a single sample.
 */
static bool candidate_held(SafetyMonitor *monitor,
                           uint8_t servo_id,
                           SafetyFaultKind kind,
                           uint32_t now_ms,
                           uint16_t required_ms,
                           uint32_t *held_ms)
{
    if (monitor->candidate_servo_id != servo_id ||
        monitor->candidate_kind != kind) {
        monitor->candidate_servo_id = servo_id;
        monitor->candidate_kind = kind;
        monitor->candidate_since_ms = now_ms;
        monitor->candidate_last_seen_ms = now_ms;
        *held_ms = 0U;
        return false;
    }

    monitor->candidate_last_seen_ms = now_ms;
    *held_ms = (uint32_t)(now_ms - monitor->candidate_since_ms);
    return *held_ms >= required_ms;
}

static void drop_candidate(SafetyMonitor *monitor, uint8_t servo_id)
{
    if (monitor->candidate_servo_id != servo_id) {
        return;
    }
    if (monitor->candidate_kind == SAFETY_FAULT_STALL &&
        monitor->stall_candidates_seen < UINT16_MAX) {
        ++monitor->stall_candidates_seen;
    }
    monitor->candidate_servo_id = 0U;
    monitor->candidate_kind = SAFETY_OK;
}

bool safety_update(SafetyMonitor *monitor,
                   const SafetySample *sample,
                   uint32_t now_ms,
                   uint16_t gait_phase)
{
    uint32_t held_ms = 0U;

    if (monitor == NULL || sample == NULL) {
        return false;
    }
    /* Once latched, the fault holds until an operator clears it. */
    if (monitor->fault != SAFETY_OK) {
        return false;
    }

    const int32_t signed_error =
        (int32_t)sample->actual - (int32_t)sample->target;
    const uint16_t position_error = signed_error < 0
        ? (uint16_t)(-signed_error)
        : (uint16_t)signed_error;
    if (position_error > monitor->peak_position_error) {
        monitor->peak_position_error = position_error;
    }

    /* Forget a candidate that has gone unseen; see SAFETY_CANDIDATE_STALE_MS. */
    if (monitor->candidate_servo_id != 0U &&
        (uint32_t)(now_ms - monitor->candidate_last_seen_ms) >
            SAFETY_CANDIDATE_STALE_MS) {
        monitor->candidate_servo_id = 0U;
        monitor->candidate_kind = SAFETY_OK;
    }

    /*
     * Discard an impossible temperature instead of acting on it.  Believing
     * one costs a needless stop; ignoring one costs nothing, because a genuine
     * overheat climbs through the plausible range on its way up and is caught
     * by the limit below.
     */
    const bool temperature_usable =
        monitor->limits.temperature_implausible_c == 0U ||
        sample->temperature_c < monitor->limits.temperature_implausible_c;
    if (!temperature_usable && monitor->implausible_samples < UINT16_MAX) {
        ++monitor->implausible_samples;
    }

    if (sample->hardware_error != 0U) {
        if (candidate_held(monitor, sample->servo_id, SAFETY_FAULT_HARDWARE,
                           now_ms, monitor->limits.confirm_ms, &held_ms)) {
            monitor->fault = SAFETY_FAULT_HARDWARE;
            fill_record(monitor, sample, position_error,
                        (uint16_t)held_ms, gait_phase);
            return true;
        }
        return false;
    }

    if (temperature_usable && monitor->limits.temperature_limit_c != 0U &&
        sample->temperature_c >= monitor->limits.temperature_limit_c) {
        if (candidate_held(monitor, sample->servo_id, SAFETY_FAULT_OVERHEAT,
                           now_ms, monitor->limits.confirm_ms, &held_ms)) {
            monitor->fault = SAFETY_FAULT_OVERHEAT;
            fill_record(monitor, sample, position_error,
                        (uint16_t)held_ms, gait_phase);
            return true;
        }
        return false;
    }

    const bool lagging =
        position_error >= monitor->limits.position_error_ticks;
    const bool straining =
        magnitude_u16(sample->load) >= monitor->limits.load_magnitude ||
        magnitude_u16(sample->current) >= monitor->limits.current_magnitude;

    if (!(lagging && straining)) {
        drop_candidate(monitor, sample->servo_id);
        return false;
    }

    if (candidate_held(monitor, sample->servo_id, SAFETY_FAULT_STALL,
                       now_ms, monitor->limits.sustain_ms, &held_ms)) {
        monitor->fault = SAFETY_FAULT_STALL;
        fill_record(monitor, sample, position_error,
                    held_ms > UINT16_MAX ? UINT16_MAX : (uint16_t)held_ms,
                    gait_phase);
        return true;
    }
    return false;
}

bool safety_watching(const SafetyMonitor *monitor, uint8_t *servo_id)
{
    if (monitor == NULL || monitor->fault != SAFETY_OK ||
        monitor->candidate_servo_id == 0U) {
        return false;
    }
    if (servo_id != NULL) {
        *servo_id = monitor->candidate_servo_id;
    }
    return true;
}

const char *safety_fault_string(SafetyFaultKind fault)
{
    switch (fault) {
    case SAFETY_OK:
        return "ok";
    case SAFETY_FAULT_STALL:
        return "stall";
    case SAFETY_FAULT_HARDWARE:
        return "servo hardware error";
    case SAFETY_FAULT_OVERHEAT:
        return "overheat";
    default:
        return "unknown";
    }
}
