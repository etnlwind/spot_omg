#ifndef DRIVE_WATCHDOG_H
#define DRIVE_WATCHDOG_H
#include <stdbool.h>
#include <stdint.h>
/* A received Stop ends the heartbeat obligation while deceleration completes. */
static inline bool drive_watchdog_due(uint32_t now, uint32_t updated,
                                     uint32_t timeout, bool stopping) {
    return !stopping && (uint32_t)(now - updated) > timeout;
}
#endif
