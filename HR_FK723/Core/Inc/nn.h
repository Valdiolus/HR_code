#ifndef __NN_H
#define __NN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h723xx.h"
#include "stm32h7xx_hal.h"
#include "network.h"
#include "network_data.h"
#include "ai_platform.h"

void AI_init(void);
void AI_RunInference(void);


#ifdef __cplusplus
}
#endif

#endif /* __NN_H */
