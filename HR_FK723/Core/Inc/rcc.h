#ifndef __RCC_H
#define __RCC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h723xx.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_rcc.h"


void SystemClock_Config(void);
void DWT_ENABLE(void);

#ifdef __cplusplus
}
#endif

#endif /* __RCC_H */
