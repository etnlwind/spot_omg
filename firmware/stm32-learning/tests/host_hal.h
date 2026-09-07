#ifndef SPOT_TEST_HOST_HAL_H
#define SPOT_TEST_HOST_HAL_H
/* Suppress the MCU-only main.h for the host execution of forward_pose.c. */
#define __MAIN_H
#include <stdint.h>
typedef struct { void *Instance; } UART_HandleTypeDef;
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t ms);
#endif
