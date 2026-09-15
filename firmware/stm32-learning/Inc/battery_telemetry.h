#ifndef BATTERY_TELEMETRY_H
#define BATTERY_TELEMETRY_H
#include <stdbool.h>
#include <stdint.h>

/* Reuse existing successful servo reads. At most one short record per second;
 * minimum is from the unsent window, never the all-time gait minimum. */
typedef struct { uint32_t sent_ms,event; uint16_t minimum_mv,published_mv; } BatteryTelemetry;
static inline void battery_telemetry_sample(BatteryTelemetry *b, uint16_t mv) {
    if(mv && mv<=60000U && (!b->minimum_mv || mv<b->minimum_mv))b->minimum_mv=mv;
}
static inline uint16_t battery_telemetry_take(BatteryTelemetry *b,uint32_t now) {
    if(!b->minimum_mv || (uint32_t)(now-b->sent_ms)<1000U)return 0;
    uint16_t mv=b->minimum_mv;b->minimum_mv=0;b->sent_ms=now;
    b->published_mv=mv;++b->event;return mv;
}
static inline uint16_t battery_telemetry_poll(BatteryTelemetry *b,uint32_t now,uint32_t *seen) {
    (void)battery_telemetry_take(b,now);
    if(*seen==b->event)return 0;
    *seen=b->event;return b->published_mv;
}
#endif
