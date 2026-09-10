#ifndef FLIGHT_LOG_TEST_HAL_H
#define FLIGHT_LOG_TEST_HAL_H
#define __MAIN_H
#include <stdint.h>
typedef struct {uint32_t CSR;} TestRcc;
extern TestRcc test_rcc;
#define RCC (&test_rcc)
typedef struct {uint32_t TypeErase,Sector,NbSectors,VoltageRange;} FLASH_EraseInitTypeDef;
#define FLASH_TYPEERASE_SECTORS 0
#define FLASH_SECTOR_7 7
#define FLASH_VOLTAGE_RANGE_3 3
#define FLASH_TYPEPROGRAM_WORD 0
#define HAL_OK 0
#define HAL_ERROR 1
uint32_t HAL_GetTick(void);
int HAL_FLASH_Unlock(void);
int HAL_FLASH_Lock(void);
int HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *erase,uint32_t *error);
int HAL_FLASH_Program(uint32_t type,uint32_t address,uint64_t word);
#endif
