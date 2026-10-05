#ifndef __UART_H
#define _UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h723xx.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_uart.h"
#include "stdio.h"

void MX_USART1_UART_Init(void);
void MX_USART2_UART_Init(void);
void uart_printf(const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif /* __UART_H */
