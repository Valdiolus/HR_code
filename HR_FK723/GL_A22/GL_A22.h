#ifndef GL_A22_H
#define GL_A22_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GL_A22_DEFAULT_ADDRESS 0x01U
#define GL_A22_REG_PROCESSED_DISTANCE 0x0100U
#define GL_A22_REG_REALTIME_DISTANCE  0x0101U
#define GL_A22_REG_TEMPERATURE        0x0102U

#define GL_A22_PHASE_IDLE        0U
#define GL_A22_PHASE_FLUSH       1U
#define GL_A22_PHASE_TX          2U
#define GL_A22_PHASE_RX          3U
#define GL_A22_PHASE_VALIDATE    4U
#define GL_A22_PHASE_DONE        5U

typedef struct
{
  UART_HandleTypeDef *uart;
  uint8_t address;
  uint32_t timeout_ms;
  uint32_t last_duration_ms;
  uint16_t last_reg_addr;
  uint8_t last_phase;
  uint8_t last_status;
  uint8_t last_flush_bytes;
  uint8_t last_rx_count;
  uint8_t last_rx_bytes[7];
} GL_A22_Handle;

void GL_A22_Init(GL_A22_Handle *sensor, UART_HandleTypeDef *uart, uint8_t address);
HAL_StatusTypeDef GL_A22_ReadRegisterAddressed(GL_A22_Handle *sensor, uint8_t request_address, uint16_t reg_addr, uint8_t *response_address, uint16_t *value);
HAL_StatusTypeDef GL_A22_ReadRegister(GL_A22_Handle *sensor, uint16_t reg_addr, uint16_t *value);
HAL_StatusTypeDef GL_A22_ReadProcessedDistanceMm(GL_A22_Handle *sensor, uint16_t *distance_mm);
HAL_StatusTypeDef GL_A22_ReadRealtimeDistanceMm(GL_A22_Handle *sensor, uint16_t *distance_mm);

#ifdef __cplusplus
}
#endif

#endif
