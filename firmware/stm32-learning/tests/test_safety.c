#include "safety.h"

#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

/* A joint tracking its target under an ordinary walking load. */
static SafetySample healthy_sample(void)
{
    SafetySample sample;
    memset(&sample, 0, sizeof(sample));
    sample.servo_id = 5U;
    sample.leg_index = 1U;   /* FR */
    sample.joint_index = 2U;
    sample.target = 2310U;
    sample.actual = 2300U;   /* 10 ticks of lag */
    sample.load = 300;
    sample.current = 200;
    sample.temperature_c = 40U;
    return sample;
}

/* The same joint caught on something: far from target and straining. */
static SafetySample stalled_sample(void)
{
    SafetySample sample = healthy_sample();
    sample.actual = 1870U;   /* 440 ticks of error */
    sample.load = 800;
    sample.current = 900;
    return sample;
}

static SafetyMonitor fresh_monitor(void)
{
    SafetyMonitor monitor;
    safety_init(&monitor, NULL);
    return monitor;
}

static void test_healthy_walking_never_faults(void)
{
    SafetyMonitor monitor = fresh_monitor();
    const SafetySample sample = healthy_sample();

    for (uint32_t now = 0U; now < 5000U; now += 20U) {
        assert(!safety_update(&monitor, &sample, now, 0U));
    }
    assert(!safety_is_faulted(&monitor));
}

/*
 * Landing shock: a brief burst of error and load, well inside the sustain
 * window.  This is the case a bare threshold would trip on.
 */
static void test_short_spike_does_not_fault(void)
{
    SafetyMonitor monitor = fresh_monitor();
    const SafetySample healthy = healthy_sample();
    const SafetySample stalled = stalled_sample();

    for (uint32_t now = 0U; now < 100U; now += 20U) {
        assert(!safety_update(&monitor, &stalled, now, 0U));
    }
    /* Recovered before sustain_ms elapsed. */
    for (uint32_t now = 100U; now < 400U; now += 20U) {
        assert(!safety_update(&monitor, &healthy, now, 0U));
    }
    assert(!safety_is_faulted(&monitor));
    assert(monitor.stall_candidates_seen == 1U);
}

static void test_sustained_stall_faults(void)
{
    SafetyMonitor monitor = fresh_monitor();
    const SafetySample stalled = stalled_sample();
    bool latched = false;

    for (uint32_t now = 0U; now <= 300U && !latched; now += 20U) {
        latched = safety_update(&monitor, &stalled, now, 250U);
    }

    assert(latched);
    assert(safety_is_faulted(&monitor));
    assert(monitor.fault == SAFETY_FAULT_STALL);
    assert(monitor.record.servo_id == 5U);
    assert(monitor.record.leg_index == 1U);
    assert(monitor.record.joint_index == 2U);
    assert(monitor.record.target == 2310U);
    assert(monitor.record.actual == 1870U);
    assert(monitor.record.position_error == 440U);
    assert(monitor.record.load == 800);
    assert(monitor.record.current == 900);
    assert(monitor.record.duration_ms >= 150U);
    assert(monitor.record.gait_phase == 250U);
}

/* Either signal on its own is ordinary; only the pair means a stall. */
static void test_each_signal_alone_is_ignored(void)
{
    SafetyMonitor lagging_only = fresh_monitor();
    SafetySample sample = stalled_sample();
    sample.load = 100;
    sample.current = 80;
    for (uint32_t now = 0U; now < 1000U; now += 20U) {
        assert(!safety_update(&lagging_only, &sample, now, 0U));
    }
    assert(!safety_is_faulted(&lagging_only));

    SafetyMonitor loaded_only = fresh_monitor();
    sample = healthy_sample();
    sample.load = 900;
    sample.current = 900;
    for (uint32_t now = 0U; now < 1000U; now += 20U) {
        assert(!safety_update(&loaded_only, &sample, now, 0U));
    }
    assert(!safety_is_faulted(&loaded_only));
}

/* Load can read low while current is honest, so either satisfies the effort test. */
static void test_current_alone_satisfies_effort(void)
{
    SafetyMonitor monitor = fresh_monitor();
    SafetySample sample = stalled_sample();
    sample.load = 0;
    sample.current = 900;
    bool latched = false;

    for (uint32_t now = 0U; now <= 300U && !latched; now += 20U) {
        latched = safety_update(&monitor, &sample, now, 0U);
    }
    assert(latched);
    assert(monitor.fault == SAFETY_FAULT_STALL);
}

/*
 * The servo's own verdicts still need a repeat.  They skip the stall's
 * sustained window, but a single frame must not be able to stop the robot.
 */
static void test_hardware_error_and_overheat_need_confirmation(void)
{
    SafetyMonitor hardware = fresh_monitor();
    SafetySample sample = healthy_sample();
    sample.hardware_error = 0x20U;
    assert(!safety_update(&hardware, &sample, 0U, 0U));
    assert(safety_update(&hardware, &sample, 20U, 0U));
    assert(hardware.fault == SAFETY_FAULT_HARDWARE);
    assert(hardware.record.hardware_error == 0x20U);

    SafetyMonitor hot = fresh_monitor();
    sample = healthy_sample();
    sample.temperature_c = 75U;
    assert(!safety_update(&hot, &sample, 0U, 0U));
    assert(safety_update(&hot, &sample, 20U, 0U));
    assert(hot.fault == SAFETY_FAULT_OVERHEAT);
    assert(hot.record.temperature_c == 75U);
}

/*
 * Regression for the first bench run, which stopped the robot on one frame
 * reporting temp=150C while the joint sat 17 ticks from target drawing no
 * current.  A single bad sample of any kind must not latch anything.
 */
static void test_single_bad_sample_never_faults(void)
{
    SafetyMonitor monitor = fresh_monitor();
    const SafetySample healthy = healthy_sample();
    SafetySample glitch = healthy_sample();
    glitch.temperature_c = 150U;

    assert(!safety_update(&monitor, &healthy, 0U, 0U));
    assert(!safety_update(&monitor, &glitch, 20U, 990U));
    assert(!safety_update(&monitor, &healthy, 40U, 0U));
    assert(!safety_is_faulted(&monitor));
    assert(monitor.implausible_samples == 1U);

    /* Nor should it if the corrupt frame keeps repeating. */
    for (uint32_t now = 60U; now < 2000U; now += 20U) {
        assert(!safety_update(&monitor, &glitch, now, 0U));
    }
    assert(!safety_is_faulted(&monitor));
}

/*
 * A real overheat still trips, because it climbs through the plausible range
 * on its way up rather than jumping straight to a corrupt value.
 */
static void test_plausible_overheat_still_faults(void)
{
    SafetyMonitor monitor = fresh_monitor();
    SafetySample sample = healthy_sample();

    sample.temperature_c = 68U;
    assert(!safety_update(&monitor, &sample, 0U, 0U));
    sample.temperature_c = 72U;
    assert(!safety_update(&monitor, &sample, 20U, 0U));
    assert(safety_update(&monitor, &sample, 40U, 0U));
    assert(monitor.fault == SAFETY_FAULT_OVERHEAT);
}

/*
 * The fault must survive whatever arrives afterwards, including a joint that
 * looks fine again: nothing recovers on its own.
 */
static void test_fault_latches_until_cleared(void)
{
    SafetyMonitor monitor = fresh_monitor();
    const SafetySample stalled = stalled_sample();
    const SafetySample healthy = healthy_sample();

    for (uint32_t now = 0U; now <= 300U; now += 20U) {
        (void)safety_update(&monitor, &stalled, now, 0U);
    }
    assert(monitor.fault == SAFETY_FAULT_STALL);
    const SafetyFaultRecord latched = monitor.record;

    for (uint32_t now = 400U; now < 3000U; now += 20U) {
        assert(!safety_update(&monitor, &healthy, now, 0U));
    }
    assert(monitor.fault == SAFETY_FAULT_STALL);
    assert(memcmp(&latched, &monitor.record, sizeof(latched)) == 0);

    safety_clear(&monitor);
    assert(!safety_is_faulted(&monitor));
    assert(monitor.record.servo_id == 0U);

    for (uint32_t now = 3000U; now < 4000U; now += 20U) {
        assert(!safety_update(&monitor, &healthy, now, 0U));
    }
}

/*
 * Round-robin sampling visits other joints in between, so a candidate that is
 * never seen again must expire rather than accumulate time forever.
 */
static void test_candidate_expires_when_not_seen(void)
{
    SafetyMonitor monitor = fresh_monitor();
    const SafetySample stalled = stalled_sample();

    assert(!safety_update(&monitor, &stalled, 0U, 0U));
    uint8_t watched = 0U;
    assert(safety_watching(&monitor, &watched));
    assert(watched == 5U);

    /* Nothing about servo 5 for well past the staleness window. */
    SafetySample other = healthy_sample();
    other.servo_id = 9U;
    other.joint_index = 3U;
    assert(!safety_update(&monitor, &other, 500U, 0U));

    /* Its clock restarts, so one late sample cannot fault instantly. */
    assert(!safety_update(&monitor, &stalled, 520U, 0U));
    assert(!safety_is_faulted(&monitor));
}

/* A stall moving to another joint restarts the window rather than summing. */
static void test_candidate_switch_restarts_the_window(void)
{
    SafetyMonitor monitor = fresh_monitor();
    SafetySample first = stalled_sample();
    SafetySample second = stalled_sample();
    second.servo_id = 8U;
    second.joint_index = 2U;
    second.leg_index = 2U;

    assert(!safety_update(&monitor, &first, 0U, 0U));
    assert(!safety_update(&monitor, &first, 100U, 0U));
    /* Switching at 120 ms must not inherit the 100 ms already served. */
    assert(!safety_update(&monitor, &second, 120U, 0U));
    assert(!safety_update(&monitor, &second, 200U, 0U));
    assert(!safety_is_faulted(&monitor));

    assert(safety_update(&monitor, &second, 280U, 0U));
    assert(monitor.record.servo_id == 8U);
}

static void test_watching_reports_only_live_candidates(void)
{
    SafetyMonitor monitor = fresh_monitor();
    const SafetySample healthy = healthy_sample();
    const SafetySample stalled = stalled_sample();
    uint8_t watched = 0U;

    assert(!safety_watching(&monitor, &watched));
    assert(!safety_update(&monitor, &stalled, 0U, 0U));
    assert(safety_watching(&monitor, &watched) && watched == 5U);

    assert(!safety_update(&monitor, &healthy, 20U, 0U));
    assert(!safety_watching(&monitor, &watched));
}

static void test_peak_position_error_is_tracked(void)
{
    SafetyMonitor monitor = fresh_monitor();
    SafetySample sample = healthy_sample();

    sample.actual = sample.target - 30U;
    (void)safety_update(&monitor, &sample, 0U, 0U);
    sample.actual = sample.target + 90U;
    (void)safety_update(&monitor, &sample, 20U, 0U);
    sample.actual = sample.target - 15U;
    (void)safety_update(&monitor, &sample, 40U, 0U);

    assert(monitor.peak_position_error == 90U);
}

static void test_null_arguments_are_safe(void)
{
    SafetyMonitor monitor = fresh_monitor();
    const SafetySample sample = healthy_sample();

    assert(!safety_update(NULL, &sample, 0U, 0U));
    assert(!safety_update(&monitor, NULL, 0U, 0U));
    assert(!safety_watching(NULL, NULL));
    assert(!safety_is_faulted(NULL));
    safety_clear(NULL);
    safety_init(NULL, NULL);
    safety_limits_default(NULL);
}

int main(void)
{
    test_healthy_walking_never_faults();
    test_short_spike_does_not_fault();
    test_sustained_stall_faults();
    test_each_signal_alone_is_ignored();
    test_current_alone_satisfies_effort();
    test_hardware_error_and_overheat_need_confirmation();
    test_single_bad_sample_never_faults();
    test_plausible_overheat_still_faults();
    test_fault_latches_until_cleared();
    test_candidate_expires_when_not_seen();
    test_candidate_switch_restarts_the_window();
    test_watching_reports_only_live_candidates();
    test_peak_position_error_is_tracked();
    test_null_arguments_are_safe();
    return 0;
}
