#ifndef __MOTORS_H
#define __MOTORS_H

#include "SteadyWin.h"
#include "Robstride.h"
#include "can.h"
#include "uart.h"

#ifdef __cplusplus
extern "C" {
#endif

void RightLeg_RobStride_CAN_Init(void);
void LeftLeg_RobStride_CAN_Init(void);
void RightLeg_SteadyWin_CAN_Init(void);
void LeftLeg_SteadyWin_CAN_Init(void);
void CaptureMotorBusRxFrame(volatile uint8_t *bus_rx_data, volatile uint32_t *bus_rx_id, volatile uint8_t *bus_rx_dlc, 
  volatile uint8_t *bus_rx_new, const FDCAN_RxHeaderTypeDef *rxHdr, const uint8_t *rxData);
uint8_t FDCAN_DLCToBytes(uint32_t dlc_code);

float Return_current_SteadyWin_Speed(SteadyWin_Motor motor);
float Return_current_SteadyWin_Angle(SteadyWin_Motor motor);
float Return_current_SteadyWin_current(SteadyWin_Motor motor);
float Return_current_SteadyWin_temp(SteadyWin_Motor motor);
float Return_current_RobStride_Angle(RobStride_Motor motor);
float Return_current_RobStride_Speed(RobStride_Motor motor);
float Return_current_RobStride_torque(RobStride_Motor motor);
float Return_current_RobStride_temp(RobStride_Motor motor);

// SteadyWin motor count and index defines
#define SW_MOTOR_COUNT     5
#define SW_IDX_HIP_PITCH   0
#define SW_IDX_HIP_ROLL    1
#define SW_IDX_HIP_YAW     2
#define SW_IDX_ANKLE_TOP   3
#define SW_IDX_ANKLE_BOT   4

#define LOOP_S_SW_READ   0
#define LOOP_S_RS_READ   1
#define LOOP_S_AI        2
#define LOOP_S_SW_WRITE  3
#define LOOP_S_RS_WRITE  4
#define LOOP_TIMEOUT_MS  20U   /* no state change for this long -> main restarts the loop */

#ifdef __cplusplus
}
#endif

#endif /* __MOTORS_H */
