#ifndef FLIGHT_LOG_H
#define FLIGHT_LOG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define FLIGHT_LOG_TEXT_CAPACITY 96U

typedef struct
{
    uint32_t sequence;
    uint32_t boot_id;
    uint32_t uptime_ms;
    uint32_t epoch_seconds;
    uint16_t epoch_millis;
    uint16_t text_length;
    char text[FLIGHT_LOG_TEXT_CAPACITY];
} FlightLogEntry;

/* Scan the persistent sector and start a new boot session. */
void flight_log_init(const char *revision);

/* Append compact events only while the motion loop is not running. */
/* Reserve a whole diagnostic batch before its first record; idle only. */
bool flight_log_prepare_entries(size_t count);
bool flight_log_append(const char *text);
bool flight_log_appendf(const char *format, ...);

/* Host time is Unix epoch milliseconds. Uptime remains authoritative too. */
bool flight_log_set_epoch_ms(uint64_t epoch_ms);
bool flight_log_time_is_synchronized(void);
uint64_t flight_log_resolve_epoch_ms(const FlightLogEntry *entry);

size_t flight_log_count(void);
uint32_t flight_log_boot_id(void);
bool flight_log_get(size_t index, FlightLogEntry *entry);

/* Explicit clear/automatic rotation preserve the IMU calibration prefix. */
bool flight_log_clear(void);

/* BNO055 calibration rewrites sector 7 and intentionally starts a new log. */
void flight_log_on_sector_reformatted(void);

#ifdef __cplusplus
}
#endif

#endif
