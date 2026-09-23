#ifndef SERVO_BUS_HOST_HAL_H
#define SERVO_BUS_HOST_HAL_H
#define __MAIN_H
#define SERVO_BUS_HOST_TEST
#include <stdint.h>
typedef struct { volatile uint32_t SR,DR,CR1,CR3; } USART_TypeDef;
typedef struct { USART_TypeDef *Instance; } UART_HandleTypeDef;
typedef enum { HAL_OK,HAL_ERROR,HAL_BUSY,HAL_TIMEOUT } HAL_StatusTypeDef;
#define USART_SR_PE (1U<<0)
#define USART_SR_FE (1U<<1)
#define USART_SR_NE (1U<<2)
#define USART_SR_ORE (1U<<3)
#define USART_SR_RXNE (1U<<5)
#define USART_SR_TC (1U<<6)
#define USART_SR_TXE (1U<<7)
#define USART_CR1_RE (1U<<2)
#define USART_CR1_RXNEIE (1U<<5)
#define USART_CR1_TCIE (1U<<6)
#define USART_CR1_TXEIE (1U<<7)
#define USART_CR3_EIE 1U
#define SET_BIT(r,b) ((r)|=(b))
#define CLEAR_BIT(r,b) ((r)&=~(b))
#define RESET 0
#define UART_FLAG_RXNE USART_SR_RXNE
#define __DMB() ((void)0)
#define __HAL_UART_GET_FLAG(u,f) ((u)->Instance->SR&(f))
#define __HAL_UART_CLEAR_OREFLAG(u) ((u)->Instance->SR&=~(USART_SR_RXNE|15U))
extern uint32_t SystemCoreClock;
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t);
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *,uint8_t *,uint16_t,uint32_t);
#endif
