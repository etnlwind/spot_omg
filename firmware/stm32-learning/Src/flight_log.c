#include "flight_log.h"

#include "main.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/*
 * Sector 7 is 0x08060000..0x0807ffff. The first KiB is retained for the
 * existing BNO055 calibration record; the rest is an append-only flight log.
 * The immutable OTA bootloader never erases sector 7.
 */
#define FLIGHT_LOG_SECTOR_ADDRESS       0x08060000UL
#define FLIGHT_LOG_FLASH_ADDRESS        0x08060400UL
#define FLIGHT_LOG_FLASH_END            0x08080000UL
#define FLIGHT_LOG_PRESERVED_WORDS      256U
#define FLIGHT_LOG_MAGIC                0x474F4C53UL
#define FLIGHT_LOG_VERSION              1U
#define FLIGHT_LOG_MIN_EPOCH_MS         UINT64_C(1577836800000)
#define FLIGHT_LOG_MAX_EPOCH_MS         UINT64_C(4294967295999)

typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    uint32_t sequence;
    uint32_t boot_id;
    uint32_t uptime_ms;
    uint32_t epoch_seconds;
    uint16_t epoch_millis;
    uint16_t text_length;
    char text[FLIGHT_LOG_TEXT_CAPACITY];
    uint32_t checksum;
} FlightLogRecord;

_Static_assert(sizeof(FlightLogRecord) == 128U,
               "flight log record must remain 128 bytes");

static uint32_t next_address = FLIGHT_LOG_FLASH_ADDRESS;
static uint32_t next_sequence = 1U;
static uint32_t current_boot_id = 1U;
static size_t record_count = 0U;
static bool initialized = false;
static bool time_synchronized = false;
static uint64_t epoch_at_sync_ms = 0U;
static uint32_t uptime_at_sync_ms = 0U;
static uint32_t preserved_prefix[FLIGHT_LOG_PRESERVED_WORDS];

static uint32_t record_checksum(const FlightLogRecord *record)
{
    const uint8_t *bytes = (const uint8_t *)record;
    uint32_t hash = 2166136261UL;
    const size_t length = offsetof(FlightLogRecord, checksum);
    for (size_t index = 0U; index < length; ++index) {
        hash ^= bytes[index];
        hash *= 16777619UL;
    }
    return hash;
}

static bool record_valid(const FlightLogRecord *record)
{
    return record->magic == FLIGHT_LOG_MAGIC &&
           record->version == FLIGHT_LOG_VERSION &&
           record->size == sizeof(*record) &&
           record->text_length < FLIGHT_LOG_TEXT_CAPACITY &&
           record->text[record->text_length] == '\0' &&
           record->checksum == record_checksum(record);
}

static bool address_has_space(uint32_t address)
{
    return address <= FLIGHT_LOG_FLASH_END - sizeof(FlightLogRecord);
}

static bool erase_preserving_calibration(void)
{
    memcpy(preserved_prefix,
           (const void *)FLIGHT_LOG_SECTOR_ADDRESS,
           sizeof(preserved_prefix));

    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t sector_error = 0U;
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Sector = FLASH_SECTOR_7;
    erase.NbSectors = 1U;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    bool ok = HAL_FLASHEx_Erase(&erase, &sector_error) == HAL_OK;

    for (size_t index = 0U; ok && index < FLIGHT_LOG_PRESERVED_WORDS; ++index) {
        if (preserved_prefix[index] == UINT32_MAX) {
            continue;
        }
        ok = HAL_FLASH_Program(
                 FLASH_TYPEPROGRAM_WORD,
                 FLIGHT_LOG_SECTOR_ADDRESS + (uint32_t)(index * 4U),
                 preserved_prefix[index]) == HAL_OK;
    }
    HAL_FLASH_Lock();
    if (ok) {
        next_address = FLIGHT_LOG_FLASH_ADDRESS;
        record_count = 0U;
    }
    return ok;
}

static void copy_entry(const FlightLogRecord *record, FlightLogEntry *entry)
{
    entry->sequence = record->sequence;
    entry->boot_id = record->boot_id;
    entry->uptime_ms = record->uptime_ms;
    entry->epoch_seconds = record->epoch_seconds;
    entry->epoch_millis = record->epoch_millis;
    entry->text_length = record->text_length;
    memcpy(entry->text, record->text, sizeof(entry->text));
}

void flight_log_init(const char *revision)
{
    uint32_t address = FLIGHT_LOG_FLASH_ADDRESS;
    const FlightLogRecord *last = NULL;
    bool incomplete_record = false;
    record_count = 0U;
    while (address_has_space(address)) {
        const FlightLogRecord *record = (const FlightLogRecord *)address;
        if (!record_valid(record)) {
            incomplete_record = record->magic != UINT32_MAX;
            break;
        }
        last = record;
        ++record_count;
        address += sizeof(FlightLogRecord);
    }
    next_address = address;
    next_sequence = last == NULL ? 1U : last->sequence + 1U;
    current_boot_id = last == NULL ? 1U : last->boot_id + 1U;
    initialized = true;
    time_synchronized = false;
    /* A reset can interrupt one word-program sequence. Flash cannot change
     * those partially programmed zero bits back to one, so start a clean
     * generation instead of repeatedly trying to overwrite the bad slot. */
    if (incomplete_record) {
        (void)erase_preserving_calibration();
    }
    (void)flight_log_appendf("BOOT reset_flags=%08lx rev=%s",
                             (unsigned long)RCC->CSR,
                             revision == NULL ? "unknown" : revision);
}

bool flight_log_append(const char *text)
{
    if (!initialized || text == NULL) {
        return false;
    }
    if (!address_has_space(next_address) && !erase_preserving_calibration()) {
        return false;
    }

    FlightLogRecord record;
    memset(&record, 0, sizeof(record));
    record.magic = FLIGHT_LOG_MAGIC;
    record.version = FLIGHT_LOG_VERSION;
    record.size = sizeof(record);
    record.sequence = next_sequence;
    record.boot_id = current_boot_id;
    record.uptime_ms = HAL_GetTick();
    if (time_synchronized) {
        const uint64_t elapsed = (uint32_t)(record.uptime_ms - uptime_at_sync_ms);
        const uint64_t epoch_ms = epoch_at_sync_ms + elapsed;
        record.epoch_seconds = (uint32_t)(epoch_ms / 1000U);
        record.epoch_millis = (uint16_t)(epoch_ms % 1000U);
    }
    size_t length = strnlen(text, FLIGHT_LOG_TEXT_CAPACITY - 1U);
    while (length > 0U && (text[length - 1U] == '\r' || text[length - 1U] == '\n')) {
        --length;
    }
    memcpy(record.text, text, length);
    record.text[length] = '\0';
    record.text_length = (uint16_t)length;
    record.checksum = record_checksum(&record);

    HAL_FLASH_Unlock();
    bool ok = true;
    const uint8_t *bytes = (const uint8_t *)&record;
    for (size_t offset = 0U; ok && offset < sizeof(record); offset += 4U) {
        uint32_t word;
        memcpy(&word, bytes + offset, sizeof(word));
        ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                               next_address + (uint32_t)offset,
                               word) == HAL_OK;
    }
    HAL_FLASH_Lock();
    if (!ok) {
        return false;
    }
    next_address += sizeof(record);
    ++next_sequence;
    ++record_count;
    return true;
}

bool flight_log_appendf(const char *format, ...)
{
    if (format == NULL) {
        return false;
    }
    char text[FLIGHT_LOG_TEXT_CAPACITY];
    va_list arguments;
    va_start(arguments, format);
    (void)vsnprintf(text, sizeof(text), format, arguments);
    va_end(arguments);
    return flight_log_append(text);
}

bool flight_log_set_epoch_ms(uint64_t epoch_ms)
{
    if (epoch_ms < FLIGHT_LOG_MIN_EPOCH_MS || epoch_ms > FLIGHT_LOG_MAX_EPOCH_MS) {
        return false;
    }
    const uint32_t now = HAL_GetTick();
    bool record_sync = !time_synchronized;
    if (time_synchronized) {
        const uint64_t expected = epoch_at_sync_ms +
            (uint32_t)(now - uptime_at_sync_ms);
        const uint64_t difference = expected > epoch_ms ?
            expected - epoch_ms : epoch_ms - expected;
        record_sync = difference >= 5000U;
    }
    epoch_at_sync_ms = epoch_ms;
    uptime_at_sync_ms = now;
    time_synchronized = true;
    return !record_sync ||
           flight_log_appendf("TIME_SYNC epoch=%lu.%03u",
                              (unsigned long)(epoch_ms / 1000U),
                              (unsigned int)(epoch_ms % 1000U));
}

bool flight_log_time_is_synchronized(void)
{
    return time_synchronized;
}

uint64_t flight_log_resolve_epoch_ms(const FlightLogEntry *entry)
{
    if (entry == NULL) {
        return 0U;
    }
    if (entry->epoch_seconds != 0U) {
        return (uint64_t)entry->epoch_seconds * 1000U + entry->epoch_millis;
    }
    if (time_synchronized && entry->boot_id == current_boot_id) {
        const int64_t delta = (int32_t)(entry->uptime_ms - uptime_at_sync_ms);
        if (delta < 0 && (uint64_t)(-delta) > epoch_at_sync_ms) {
            return 0U;
        }
        return (uint64_t)((int64_t)epoch_at_sync_ms + delta);
    }
    return 0U;
}

size_t flight_log_count(void)
{
    return record_count;
}

uint32_t flight_log_boot_id(void)
{
    return current_boot_id;
}

bool flight_log_get(size_t index, FlightLogEntry *entry)
{
    if (entry == NULL || index >= record_count) {
        return false;
    }
    const FlightLogRecord *record = (const FlightLogRecord *)(
        FLIGHT_LOG_FLASH_ADDRESS + index * sizeof(FlightLogRecord));
    if (!record_valid(record)) {
        return false;
    }
    copy_entry(record, entry);
    return true;
}

bool flight_log_clear(void)
{
    return initialized && erase_preserving_calibration();
}

void flight_log_on_sector_reformatted(void)
{
    if (!initialized) {
        return;
    }
    next_address = FLIGHT_LOG_FLASH_ADDRESS;
    record_count = 0U;
}
