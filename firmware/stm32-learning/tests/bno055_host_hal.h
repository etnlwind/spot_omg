#ifndef BNO055_HOST_HAL_H
#define BNO055_HOST_HAL_H
#define __MAIN_H
#include <stdint.h>
typedef enum { HAL_OK,HAL_ERROR,HAL_BUSY,HAL_TIMEOUT } HAL_StatusTypeDef;
typedef enum { HAL_I2C_STATE_READY,HAL_I2C_STATE_BUSY_RX } HAL_I2C_StateTypeDef;
typedef struct { HAL_I2C_StateTypeDef state; unsigned busy; } I2C_HandleTypeDef;
typedef struct { uint32_t TypeErase,Sector,NbSectors,VoltageRange; } FLASH_EraseInitTypeDef;
#define I2C_FLAG_BUSY 1U
#define RESET 0U
#define I2C_MEMADD_SIZE_8BIT 1U
#define FLASH_TYPEERASE_SECTORS 0U
#define FLASH_SECTOR_7 7U
#define FLASH_VOLTAGE_RANGE_3 3U
#define FLASH_TYPEPROGRAM_WORD 0U
#define __HAL_I2C_GET_FLAG(i2c,flag) ((void)(flag),(i2c)->busy)
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t ms);
HAL_I2C_StateTypeDef HAL_I2C_GetState(I2C_HandleTypeDef *i2c);
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *,uint16_t,uint16_t,uint16_t,uint8_t *,uint16_t,uint32_t);
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *,uint16_t,uint16_t,uint16_t,uint8_t *,uint16_t,uint32_t);
HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *,uint16_t,uint32_t,uint32_t);
void HAL_FLASH_Unlock(void);
void HAL_FLASH_Lock(void);
HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *,uint32_t *);
HAL_StatusTypeDef HAL_FLASH_Program(uint32_t,uint32_t,uint64_t);
#endif
