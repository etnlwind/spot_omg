#include "bno055.h"
#include "bno_imu_config.h"
#include "flight_log.h"

#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#define BNO055_CHIP_ID_ADDR       0x00U
#define BNO055_PAGE_ID_ADDR       0x07U
#define BNO055_EULER_H_LSB_ADDR   0x1AU
#define BNO055_GYRO_X_LSB_ADDR    0x14U
#define BNO055_UNIT_SEL_ADDR      0x3BU
#define BNO055_CALIB_STAT_ADDR     0x35U
#define BNO055_OPR_MODE_ADDR      0x3DU
#define BNO055_PWR_MODE_ADDR      0x3EU
#define BNO055_SYS_TRIGGER_ADDR   0x3FU
#define BNO055_OFFSET_START_ADDR   0x55U

#define BNO055_CHIP_ID            0xA0U
#define BNO055_MODE_CONFIG        0x00U
#define BNO055_MODE_IMUPLUS       0x08U
#define BNO055_POWER_NORMAL       0x00U

#define BNO055_TIMEOUT_MS         100U

/*
 * Sign conventions, matching what the README documents for this robot:
 *
 *   Pitch +  tipping toward the front of the robot
 *   Roll  +  tipping toward the right of the robot
 *
 * Measured on the bench 2026-08-08: roll already agreed, pitch came out
 * negative when the robot was tipped forward, so pitch is inverted here.
 * This is the place to fix it -- the balance loop is written against the
 * convention above, and compensating in its gains would hide the mismatch.
 */
#define BNO055_YAW_SIGN    (+1)

#define BNO055_CAL_MAGIC           0x354F4E42UL
#define BNO055_CAL_VERSION         1U
#define BNO055_CAL_DEVICE_VALID    (1UL << 0)
#define BNO055_CAL_LEVEL_VALID     (1UL << 1)
#define BNO055_CAL_FLASH_ADDRESS   0x08060000UL

typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    uint32_t flags;
    uint8_t device_profile[22];
    int16_t level_roll_tenths;
    int16_t level_pitch_tenths;
    uint16_t reserved;
    uint32_t checksum;
} Bno055CalibrationRecord;

_Static_assert((sizeof(Bno055CalibrationRecord) % 4U) == 0U,
               "calibration record must be word aligned");

/* Euler output is 16 LSB per degree; the robot works in tenths. */
#define BNO055_EULER_TO_TENTHS(raw) ((int16_t)(((int32_t)(raw) * 10) / 16))

static HAL_StatusTypeDef write8(Bno055 *imu, uint8_t reg, uint8_t value)
{
    return HAL_I2C_Mem_Write(imu->i2c,
                             imu->address,
                             reg,
                             I2C_MEMADD_SIZE_8BIT,
                             &value,
                             1U,
                             BNO055_TIMEOUT_MS);
}

/* STM32 HAL has a separate 25ms pre-transfer BUSY wait, regardless of Timeout.
 * This single-master driver rejects an already busy bus before entering HAL.
 * A healthy 100kHz bus needs about 3ms total for metadata and 12-byte payload.
 * The timeout assumes SysTick runs and no concurrent ISR owns/reconfigures I2C.
 * It is not a timing proof for a multi-master arbitration race.
 */
static bool read_body_bytes(Bno055 *imu,uint8_t reg,uint8_t *bytes,uint16_t size)
{
    if (imu->i2c==NULL || HAL_I2C_GetState(imu->i2c)!=HAL_I2C_STATE_READY ||
        __HAL_I2C_GET_FLAG(imu->i2c,I2C_FLAG_BUSY)!=RESET) return false;
    return HAL_I2C_Mem_Read(imu->i2c,imu->address,reg,I2C_MEMADD_SIZE_8BIT,
                           bytes,size,BNO055_BODY_READ_TIMEOUT_MS)==HAL_OK;
}

bool bno055_set_body_frame(Bno055 *imu,const Bno055BodyFrame *frame)
{
    if (imu==NULL || !bno_imu_frame_valid(frame)) return false;
    imu->body_frame=*frame;
    imu->cached_body_imu.valid=false;
    return true;
}

bool bno055_read_body_imu(void *context,BodyImuState *sample)
{
    Bno055 *imu=context;
    if (sample==NULL) return false;
    memset(sample,0,sizeof(*sample));
    if (imu==NULL) return false;
    sample->sequence=imu->body_sequence;
    sample->timestamp_ms=imu->cached_body_imu.timestamp_ms;
    if (!imu->present || imu->i2c==NULL) return false;
    const uint32_t now=HAL_GetTick();
    if (imu->cached_body_imu.valid &&
        (uint32_t)(now-imu->cached_body_imu.timestamp_ms)<BNO055_BODY_CACHE_MS) {
        *sample=imu->cached_body_imu;
        return true;
    }
    imu->cached_body_imu.valid=false;
    imu->body_metadata_valid=false;
    /* Read units and remap on every uncached observation. A sensor reset or
     * changed configuration must not silently reuse stale SI scales/axes.
     * These read-only checks do not change legacy init success or registers.
     */
    if (!read_body_bytes(imu,BNO055_UNIT_SEL_ADDR,imu->body_metadata,8U) ||
        !bno_imu_metadata_valid(imu->body_metadata)) return false;
    imu->body_metadata_valid=true;
    uint8_t bytes[12];
    if (!read_body_bytes(imu,BNO055_GYRO_X_LSB_ADDR,bytes,sizeof(bytes))) return false;
    float yaw_rad=0.f;
    if (!bno_imu_decode(bytes,imu->body_metadata,&imu->body_frame,
                        imu->level_valid ? imu->level_roll_tenths : 0,
                        imu->level_valid ? imu->level_pitch_tenths : 0,
                        HAL_GetTick(),imu->body_sequence+1U,sample,
                        imu->body_gyro_rad_s,&yaw_rad)) {
        sample->sequence=imu->body_sequence;
        sample->timestamp_ms=imu->cached_body_imu.timestamp_ms;
        return false;
    }
    imu->body_sequence=sample->sequence;
    imu->cached_body_imu=*sample;
    /* Keep the existing heading callback usable without a second Euler read.
     * The legacy Euler callback itself remains byte-for-byte unchanged.
     */
    const float yaw_tenths=yaw_rad*(10.f/BNO_IMU_DEG_TO_RAD);
    imu->cached_yaw_tenths=(int16_t)(BNO055_YAW_SIGN*yaw_tenths);
    imu->cached_yaw_at=sample->timestamp_ms;
    imu->cached_yaw_valid=true;
    return true;
}

static uint32_t record_checksum(const Bno055CalibrationRecord *record)
{
    const uint8_t *bytes = (const uint8_t *)record;
    uint32_t hash = 2166136261UL;
    const size_t length = offsetof(Bno055CalibrationRecord, checksum);
    for (size_t index = 0U; index < length; ++index) {
        hash ^= bytes[index];
        hash *= 16777619UL;
    }
    return hash;
}

static bool record_valid(const Bno055CalibrationRecord *record)
{
    return record->magic == BNO055_CAL_MAGIC &&
           record->version == BNO055_CAL_VERSION &&
           record->size == sizeof(*record) &&
           record->checksum == record_checksum(record);
}

static void load_record(Bno055 *imu)
{
    const Bno055CalibrationRecord *stored =
        (const Bno055CalibrationRecord *)BNO055_CAL_FLASH_ADDRESS;
    if (!record_valid(stored)) {
        return;
    }

    if ((stored->flags & BNO055_CAL_DEVICE_VALID) != 0U) {
        memcpy(imu->device_profile, stored->device_profile,
               sizeof(imu->device_profile));
        imu->device_profile_valid = true;
    }
    if ((stored->flags & BNO055_CAL_LEVEL_VALID) != 0U) {
        imu->level_roll_tenths = stored->level_roll_tenths;
        imu->level_pitch_tenths = stored->level_pitch_tenths;
        imu->level_valid = true;
    }
}

static bool save_record(const Bno055 *imu)
{
    Bno055CalibrationRecord record;
    memset(&record, 0, sizeof(record));
    record.magic = BNO055_CAL_MAGIC;
    record.version = BNO055_CAL_VERSION;
    record.size = sizeof(record);
    if (imu->device_profile_valid) {
        record.flags |= BNO055_CAL_DEVICE_VALID;
        memcpy(record.device_profile, imu->device_profile,
               sizeof(record.device_profile));
    }
    if (imu->level_valid) {
        record.flags |= BNO055_CAL_LEVEL_VALID;
        record.level_roll_tenths = imu->level_roll_tenths;
        record.level_pitch_tenths = imu->level_pitch_tenths;
    }
    record.checksum = record_checksum(&record);

    HAL_FLASH_Unlock();
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t sector_error = 0U;
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Sector = FLASH_SECTOR_7;
    erase.NbSectors = 1U;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    bool ok = HAL_FLASHEx_Erase(&erase, &sector_error) == HAL_OK;

    const uint8_t *bytes = (const uint8_t *)&record;
    for (size_t offset = 0U; ok && offset < sizeof(record); offset += 4U) {
        uint32_t word = 0xFFFFFFFFUL;
        memcpy(&word, &bytes[offset],
               sizeof(record) - offset < 4U ? sizeof(record) - offset : 4U);
        ok = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                               BNO055_CAL_FLASH_ADDRESS + offset,
                               word) == HAL_OK;
    }
    HAL_FLASH_Lock();
    if (ok) {
        /* Sector 7 is shared with the flight log. Calibration changes are
         * rare and deliberately begin a fresh log after the sector erase. */
        flight_log_on_sector_reformatted();
    }
    return ok;
}

static uint16_t detect(Bno055 *imu)
{
    /* COM3 low gives 0x28, high gives 0x29; the built robot ties it low. */
    static const uint8_t candidates[] = {0x28U, 0x29U};

    for (size_t index = 0U; index < sizeof(candidates); ++index) {
        const uint16_t address = (uint16_t)(candidates[index] << 1);
        uint8_t chip_id = 0U;

        if (HAL_I2C_IsDeviceReady(imu->i2c, address, 3U,
                                  BNO055_TIMEOUT_MS) != HAL_OK) {
            continue;
        }
        if (HAL_I2C_Mem_Read(imu->i2c, address, BNO055_CHIP_ID_ADDR,
                             I2C_MEMADD_SIZE_8BIT, &chip_id, 1U,
                             BNO055_TIMEOUT_MS) != HAL_OK) {
            continue;
        }
        /*
         * Check the chip ID rather than trusting the address: other parts live
         * at 0x28 too, and answering there is not the same as being a BNO055.
         */
        if (chip_id == BNO055_CHIP_ID) {
            return address;
        }
    }
    return 0U;
}

bool bno055_init(Bno055 *imu, I2C_HandleTypeDef *i2c)
{
    if (imu == NULL || i2c == NULL) {
        return false;
    }

    memset(imu, 0, sizeof(*imu));
    imu->i2c = i2c;
    const Bno055BodyFrame body_frame={
        .gyro_axis={BNO055_BODY_GYRO_X_AXIS,BNO055_BODY_GYRO_Y_AXIS,
                    BNO055_BODY_GYRO_Z_AXIS},
        .expected_axis_config=BNO055_BODY_EXPECT_AXIS_CONFIG,
        .expected_axis_sign=BNO055_BODY_EXPECT_AXIS_SIGN,
        .expected_orientation=BNO055_BODY_EXPECT_ORIENTATION,
        .axis_verified=BNO055_BODY_AXES_VERIFIED!=0
    };
    (void)bno055_set_body_frame(imu,&body_frame);
    load_record(imu);
    imu->address = detect(imu);
    if (imu->address == 0U) {
        return false;
    }

    /* Configuration registers only accept writes outside a fusion mode. */
    if (write8(imu, BNO055_OPR_MODE_ADDR, BNO055_MODE_CONFIG) != HAL_OK) {
        return false;
    }
    HAL_Delay(25);

    if (write8(imu, BNO055_PAGE_ID_ADDR, 0x00U) != HAL_OK ||
        write8(imu, BNO055_PWR_MODE_ADDR, BNO055_POWER_NORMAL) != HAL_OK) {
        return false;
    }
    HAL_Delay(10);

    if (write8(imu, BNO055_SYS_TRIGGER_ADDR, 0x00U) != HAL_OK) {
        return false;
    }
    HAL_Delay(10);

    if (imu->device_profile_valid) {
        if (HAL_I2C_Mem_Write(imu->i2c, imu->address,
                              BNO055_OFFSET_START_ADDR,
                              I2C_MEMADD_SIZE_8BIT,
                              imu->device_profile,
                              sizeof(imu->device_profile),
                              BNO055_TIMEOUT_MS) != HAL_OK) {
            return false;
        }
        imu->device_profile_restored = true;
    }

    if (write8(imu, BNO055_OPR_MODE_ADDR, BNO055_MODE_IMUPLUS) != HAL_OK) {
        return false;
    }
    HAL_Delay(30);

    imu->present = true;
    /* A failed new-path metadata read never changes legacy init success. */
    imu->body_metadata_valid=
        read_body_bytes(imu,BNO055_UNIT_SEL_ADDR,imu->body_metadata,8U) &&
        bno_imu_metadata_valid(imu->body_metadata);
    return true;
}

uint8_t bno055_scan(I2C_HandleTypeDef *i2c, uint8_t *found, uint8_t capacity)
{
    uint8_t count = 0U;

    if (i2c == NULL || found == NULL) {
        return 0U;
    }

    for (uint8_t address = 1U; address < 127U; ++address) {
        if (HAL_I2C_IsDeviceReady(i2c, (uint16_t)(address << 1), 2U, 20U) !=
            HAL_OK) {
            continue;
        }
        if (count < capacity) {
            found[count] = address;
        }
        ++count;
    }
    return count;
}

bool bno055_read_euler(Bno055 *imu,
                       int16_t *yaw_tenths,
                       int16_t *roll_tenths,
                       int16_t *pitch_tenths)
{
    uint8_t data[6];

    if (imu == NULL || !imu->present ||
        yaw_tenths == NULL || roll_tenths == NULL || pitch_tenths == NULL) {
        return false;
    }

    imu->cached_yaw_valid = false;
    if (HAL_I2C_Mem_Read(imu->i2c,
                         imu->address,
                         BNO055_EULER_H_LSB_ADDR,
                         I2C_MEMADD_SIZE_8BIT,
                         data,
                         sizeof(data),
                         BNO055_TIMEOUT_MS) != HAL_OK) {
        return false;
    }

    const int16_t sensor_yaw = BNO055_EULER_TO_TENTHS(
        (int16_t)(((uint16_t)data[1] << 8) | data[0]));
    const int16_t sensor_roll = BNO055_EULER_TO_TENTHS(
        (int16_t)(((uint16_t)data[3] << 8) | data[2]));
    const int16_t sensor_pitch = BNO055_EULER_TO_TENTHS(
        (int16_t)(((uint16_t)data[5] << 8) | data[4]));

    /*
     * The board is mounted 90 degrees from the robot frame.  Bench-verified:
     * sensor Roll grows forward and sensor Pitch grows to robot-right.
     */
    const int16_t mapped_roll = sensor_pitch;
    const int16_t mapped_pitch = sensor_roll;
    *yaw_tenths = (int16_t)(BNO055_YAW_SIGN * sensor_yaw);
    imu->cached_yaw_tenths = *yaw_tenths;
    imu->cached_yaw_at = HAL_GetTick();
    imu->cached_yaw_valid = true;
    *roll_tenths = (int16_t)(mapped_roll -
        (imu->level_valid ? imu->level_roll_tenths : 0));
    *pitch_tenths = (int16_t)(mapped_pitch -
        (imu->level_valid ? imu->level_pitch_tenths : 0));
    return true;
}

bool bno055_get_calibration_status(Bno055 *imu,
                                   Bno055CalibrationStatus *status)
{
    uint8_t value = 0U;
    if (imu == NULL || !imu->present || status == NULL ||
        HAL_I2C_Mem_Read(imu->i2c, imu->address, BNO055_CALIB_STAT_ADDR,
                         I2C_MEMADD_SIZE_8BIT, &value, 1U,
                         BNO055_TIMEOUT_MS) != HAL_OK) {
        return false;
    }
    status->system = (uint8_t)((value >> 6) & 0x03U);
    status->gyro = (uint8_t)((value >> 4) & 0x03U);
    status->accel = (uint8_t)((value >> 2) & 0x03U);
    status->mag = (uint8_t)(value & 0x03U);
    return true;
}

bool bno055_save_device_calibration(Bno055 *imu)
{
    Bno055CalibrationStatus status;
    if (!bno055_get_calibration_status(imu, &status) ||
        status.gyro != 3U || status.accel != 3U) {
        return false;
    }

    if (write8(imu, BNO055_OPR_MODE_ADDR, BNO055_MODE_CONFIG) != HAL_OK) {
        return false;
    }
    HAL_Delay(25U);
    const bool read_ok =
        HAL_I2C_Mem_Read(imu->i2c, imu->address,
                         BNO055_OFFSET_START_ADDR,
                         I2C_MEMADD_SIZE_8BIT,
                         imu->device_profile,
                         sizeof(imu->device_profile),
                         BNO055_TIMEOUT_MS) == HAL_OK;
    const bool mode_ok =
        write8(imu, BNO055_OPR_MODE_ADDR, BNO055_MODE_IMUPLUS) == HAL_OK;
    HAL_Delay(30U);
    if (!read_ok || !mode_ok) {
        return false;
    }

    imu->device_profile_valid = true;
    imu->device_profile_restored = true;
    return save_record(imu);
}

bool bno055_save_level_calibration(Bno055 *imu, uint16_t samples)
{
    if (imu == NULL || !imu->present || samples == 0U) {
        return false;
    }

    const bool had_level = imu->level_valid;
    const int16_t old_roll = imu->level_roll_tenths;
    const int16_t old_pitch = imu->level_pitch_tenths;
    imu->level_valid = false;

    int32_t roll_sum = 0;
    int32_t pitch_sum = 0;
    for (uint16_t sample = 0U; sample < samples; ++sample) {
        int16_t yaw = 0;
        int16_t roll = 0;
        int16_t pitch = 0;
        if (!bno055_read_euler(imu, &yaw, &roll, &pitch)) {
            imu->level_valid = had_level;
            imu->level_roll_tenths = old_roll;
            imu->level_pitch_tenths = old_pitch;
            return false;
        }
        roll_sum += roll;
        pitch_sum += pitch;
        HAL_Delay(10U);
    }

    imu->level_roll_tenths = (int16_t)(roll_sum / samples);
    imu->level_pitch_tenths = (int16_t)(pitch_sum / samples);
    imu->level_valid = true;
    imu->cached_body_imu.valid = false;
    return save_record(imu);
}

bool bno055_clear_calibration(Bno055 *imu)
{
    if (imu == NULL) {
        return false;
    }
    imu->device_profile_valid = false;
    imu->device_profile_restored = false;
    imu->level_valid = false;
    imu->level_roll_tenths = 0;
    imu->level_pitch_tenths = 0;
    imu->cached_body_imu.valid = false;
    memset(imu->device_profile, 0, sizeof(imu->device_profile));
    return save_record(imu);
}

bool bno055_read_attitude(void *context,
                          int16_t *roll_tenths,
                          int16_t *pitch_tenths)
{
    Bno055 *imu = (Bno055 *)context;
    int16_t yaw_tenths = 0;

    /* Yaw is deliberately unused: balance only corrects roll and pitch. */
    return bno055_read_euler(imu, &yaw_tenths, roll_tenths, pitch_tenths);
}

/* Reuse the same six-byte Euler transaction as roll/pitch, not another read. */
bool bno055_read_heading(void *context, int16_t *yaw_tenths)
{
    Bno055 *imu=context;
    if(!imu || !yaw_tenths || !imu->present || !imu->cached_yaw_valid ||
       (uint32_t)(HAL_GetTick()-imu->cached_yaw_at)>100U) return false;
    *yaw_tenths=imu->cached_yaw_tenths;
    return true;
}
