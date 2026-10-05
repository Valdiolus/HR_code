#ifndef __GPIO_H
#define __GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h723xx.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_gpio.h"


void MX_GPIO_Init(void);

#define LED_Pin GPIO_PIN_7
#define LED_GPIO_Port GPIOG

#ifdef __cplusplus
}
#endif

#endif /* __GPIO_H */
