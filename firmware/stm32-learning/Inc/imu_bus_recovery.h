#ifndef IMU_BUS_RECOVERY_H
#define IMU_BUS_RECOVERY_H
#include "bno055.h"

/* Manual, foreground-only bus recovery. No sensor configuration, calibration,
 * servo command, fault clearing or automatic motion restart occurs here. */
typedef struct {
    HAL_StatusTypeDef deinit_status, init_status;
    uint8_t chip, mode, samples;
    bool chip_ok, mode_ok, body_ok;
} ImuBusRecovery;

static inline bool imu_bus_recover(Bno055 *imu, bool moving, ImuBusRecovery *result)
{
    *result=(ImuBusRecovery){.deinit_status=HAL_ERROR,.init_status=HAL_ERROR};
    if(moving || !imu || !imu->i2c || !imu->present ||
       (imu->address!=0x50U && imu->address!=0x52U))return false;
    imu->cached_yaw_valid=false;
    imu->cached_body_imu.valid=false;
    imu->body_metadata_valid=false;
    result->deinit_status=HAL_I2C_DeInit(imu->i2c);
    if(result->deinit_status!=HAL_OK)return false;
    result->init_status=HAL_I2C_Init(imu->i2c);
    if(result->init_status!=HAL_OK)return false;
    result->chip_ok=HAL_I2C_Mem_Read(imu->i2c,imu->address,0x00U,
        I2C_MEMADD_SIZE_8BIT,&result->chip,1U,20U)==HAL_OK && result->chip==0xa0U;
    if(!result->chip_ok)return false;
    result->mode_ok=HAL_I2C_Mem_Read(imu->i2c,imu->address,0x3dU,
        I2C_MEMADD_SIZE_8BIT,&result->mode,1U,20U)==HAL_OK && result->mode==0x08U;
    if(!result->mode_ok)return false;
    for(unsigned i=0;i<3;i++) {
        int16_t roll,pitch;
        BodyImuState body;
        imu->cached_body_imu.valid=false;
        result->body_ok=bno055_read_body_imu(imu,&body);
        if(!result->body_ok || !bno055_read_attitude(imu,&roll,&pitch))return false;
        result->samples++;
        if(i<2)HAL_Delay(20U);
    }
    return true;
}
#endif
