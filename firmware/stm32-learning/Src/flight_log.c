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
#define FOOT_LIFT_MAGIC                 0x5446494CUL
#define FOOT_LIFT_SNAPSHOT_OFFSET       256U
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

#define FLIGHT_LOG_SLOT_COUNT ((FLIGHT_LOG_FLASH_END-FLIGHT_LOG_FLASH_ADDRESS)/sizeof(FlightLogRecord))
static uint16_t valid_slots[FLIGHT_LOG_SLOT_COUNT];
static const void *flash_pointer(uint32_t address)
{
#ifdef FLIGHT_LOG_HOST_TEST
    extern const void *flight_log_test_pointer(uint32_t address);
    return flight_log_test_pointer(address);
#else
    return (const void *)(uintptr_t)address;
#endif
}
static bool record_erased(const FlightLogRecord *record)
{
    const uint8_t *bytes=(const uint8_t *)record;
    for(size_t i=0;i<sizeof(*record);i++)if(bytes[i]!=0xff)return false;
    return true;
}

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
    return (record->magic == FLIGHT_LOG_MAGIC || record->magic == FOOT_LIFT_MAGIC) &&
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

static const FlightLogRecord *latest_foot_lift(void)
{
    const FlightLogRecord *last = flash_pointer(FLIGHT_LOG_SECTOR_ADDRESS + FOOT_LIFT_SNAPSHOT_OFFSET);
    if (!record_valid(last) || last->magic != FOOT_LIFT_MAGIC) last = NULL;
    for (uint32_t address=FLIGHT_LOG_FLASH_ADDRESS; address_has_space(address); address+=sizeof(FlightLogRecord)) {
        const FlightLogRecord *record=flash_pointer(address);
        if (record_valid(record) && record->magic == FOOT_LIFT_MAGIC) last=record;
    }
    return last;
}

bool flight_log_load_foot_lift(uint32_t values[4])
{
    const FlightLogRecord *record=latest_foot_lift();
    if (!record) return false;
    /* Payload is four fixed little-endian words in the text area. */
    uint32_t loaded[4];
    memcpy(loaded, record->text, sizeof(loaded));
    for (int i=0;i<4;i++) if (loaded[i]>INT32_MAX) return false;
    memcpy(values, loaded, sizeof(loaded));
    return true;
}

bool flight_log_load_foot_width(int32_t values[4])
{
    const FlightLogRecord *record=latest_foot_lift();
    if(!record)return false;
    memset(values,0,4*sizeof(int32_t));
    if(record->text_length==8*sizeof(uint32_t))memcpy(values,record->text+4*sizeof(uint32_t),4*sizeof(int32_t));
    return true;
}

static bool rewrite_prefix(const void *calibration, size_t size)

{
    memcpy(preserved_prefix,
           flash_pointer(FLIGHT_LOG_SECTOR_ADDRESS),
           sizeof(preserved_prefix));
    const FlightLogRecord *latest=latest_foot_lift();
    if (latest) memcpy((uint8_t *)preserved_prefix + FOOT_LIFT_SNAPSHOT_OFFSET, latest, sizeof(*latest));
    if (calibration) {
        for (size_t i=0;i<FOOT_LIFT_SNAPSHOT_OFFSET/sizeof(uint32_t);i++) preserved_prefix[i]=UINT32_MAX;
        memcpy(preserved_prefix, calibration, size);
    }

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
    ok = ok && memcmp(preserved_prefix, flash_pointer(FLIGHT_LOG_SECTOR_ADDRESS), sizeof(preserved_prefix)) == 0;
    if (ok) {
        next_address = FLIGHT_LOG_FLASH_ADDRESS;
        record_count = 0U;
    }
    return ok;
}

static bool erase_preserving_calibration(void) { return rewrite_prefix(NULL, 0); }

bool flight_log_save_calibration(const void *record, size_t size)
{
    return record && size <= FOOT_LIFT_SNAPSHOT_OFFSET && rewrite_prefix(record, size);
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
    if (record->magic == FOOT_LIFT_MAGIC) {
        uint32_t v[4]; memcpy(v, record->text, sizeof(v));
        entry->text_length=(uint16_t)snprintf(entry->text,sizeof(entry->text),"FOOTLIFT saved %lu %lu %lu %lu",(unsigned long)v[0],(unsigned long)v[1],(unsigned long)v[2],(unsigned long)v[3]);
    }
}

void flight_log_init(const char *revision)
{
    const FlightLogRecord *last = NULL;
    record_count = 0U;
    next_address = FLIGHT_LOG_FLASH_ADDRESS;
    /* Scan the entire sector: interrupted records and erased gaps are consumed
     * slots, not a reason to erase earlier completed diagnostic batches. */
    for(uint32_t address=FLIGHT_LOG_FLASH_ADDRESS;address_has_space(address);
            address+=sizeof(FlightLogRecord)) {
        const FlightLogRecord *record=flash_pointer(address);
        if(!record_erased(record))next_address=address+sizeof(FlightLogRecord);
        if(record_valid(record)) {
            valid_slots[record_count++]=(uint16_t)((address-FLIGHT_LOG_FLASH_ADDRESS)/sizeof(FlightLogRecord));
            last=record;
        }
    }
    next_sequence = last == NULL ? 1U : last->sequence + 1U;
    current_boot_id = last == NULL ? 1U : last->boot_id + 1U;
    initialized = true;
    time_synchronized = false;
    (void)flight_log_appendf("BOOT reset_flags=%08lx rev=%s",
                             (unsigned long)RCC->CSR,
                             revision == NULL ? "unknown" : revision);
}

bool flight_log_prepare_entries(size_t count)
{
    if(!initialized || count==0 || count>FLIGHT_LOG_SLOT_COUNT)return false;
    size_t remaining=(FLIGHT_LOG_FLASH_END-next_address)/sizeof(FlightLogRecord);
    return remaining>=count || erase_preserving_calibration();
}

static bool append_record(const char *text, const uint32_t *foot_lift, const int32_t *foot_width)
{
    if (!initialized || text == NULL) {
        return false;
    }
    if (!address_has_space(next_address) && !erase_preserving_calibration()) {
        return false;
    }

    FlightLogRecord record;
    memset(&record, 0, sizeof(record));
    record.magic = foot_lift ? FOOT_LIFT_MAGIC : FLIGHT_LOG_MAGIC;
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
    if (foot_lift) {
        memcpy(record.text, foot_lift, 4*sizeof(uint32_t));
        record.text_length = 8*sizeof(uint32_t);
        if(foot_width)memcpy(record.text+4*sizeof(uint32_t),foot_width,4*sizeof(int32_t));
        record.text[record.text_length] = 0;
    }
    record.checksum = record_checksum(&record);

    const uint32_t record_address=next_address;
    /* Never retry programming a partially written slot. */
    next_address += sizeof(record);
    HAL_FLASH_Unlock();
    bool ok = true;
    const uint8_t *bytes = (const uint8_t *)&record;
    for (size_t offset = 0U; ok && offset < sizeof(record); offset += 4U) {
        uint32_t word;
        memcpy(&word, bytes + offset, sizeof(word));
        ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                               record_address + (uint32_t)offset,
                               word) == HAL_OK;
    }
    HAL_FLASH_Lock();
    if (!ok || memcmp(flash_pointer(record_address), &record, sizeof(record)) != 0) {
        return false;
    }
    valid_slots[record_count++]=(uint16_t)((record_address-FLIGHT_LOG_FLASH_ADDRESS)/sizeof(record));
    ++next_sequence;
    return true;
}

bool flight_log_append(const char *text) { return append_record(text, NULL, NULL); }

bool flight_log_save_foot_settings(const uint32_t values[4],const int32_t widths[4])
{
    if(!values || !widths || !initialized)return false;
    for(int i=0;i<4;i++)if(values[i]>INT32_MAX)return false;
    uint32_t previous[4];int32_t old_widths[4];
    if(flight_log_load_foot_lift(previous) && flight_log_load_foot_width(old_widths) &&
       !memcmp(previous,values,sizeof(previous)) && !memcmp(old_widths,widths,sizeof(old_widths)))return true;
    if(!append_record("",values,widths))return false;
    return flight_log_load_foot_lift(previous) && flight_log_load_foot_width(old_widths) &&
       !memcmp(previous,values,sizeof(previous)) && !memcmp(old_widths,widths,sizeof(old_widths));
}
bool flight_log_save_foot_lift(const uint32_t values[4])
{
    int32_t widths[4]={0};(void)flight_log_load_foot_width(widths);
    return flight_log_save_foot_settings(values,widths);
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
    const FlightLogRecord *record = flash_pointer(
        FLIGHT_LOG_FLASH_ADDRESS + valid_slots[index] * sizeof(FlightLogRecord));
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
