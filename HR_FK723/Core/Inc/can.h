#ifndef __CAN_H
#define __CAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h723xx.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_fdcan.h"


void MX_FDCAN1_Init(void);
void MX_FDCAN2_Init(void);
void MX_FDCAN3_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H */
