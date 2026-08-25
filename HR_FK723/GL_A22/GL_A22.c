#include "GL_A22.h"

#define GL_A22_MODBUS_READ_FN         0x03U
#define GL_A22_REQUEST_LEN            8U
#define GL_A22_RESPONSE_LEN           7U
#define GL_A22_RESPONSE_DATA_BYTES    2U
#define GL_A22_UART_TIMEOUT_MS        150U
#define GL_A22_FLUSH_TIMEOUT_MS       2U
#define GL_A22_TX_TO_RX_DELAY_MS      15U
#define GL_A22_RX_BYTE_TIMEOUT_MS     20U

static uint16_t GL_A22_Crc16(const uint8_t *data, uint16_t length)
{
  uint16_t crc = 0xFFFFU;

  for (uint16_t i = 0; i < length; ++i)
  {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8U; ++bit)
    {
      if ((crc & 0x0001U) != 0U)
      {
        crc = (crc >> 1U) ^ 0xA001U;
      }
      else
      {
        crc >>= 1U;
      }
    }
  }

  return crc;
}

static uint8_t GL_A22_FlushRx(GL_A22_Handle *sensor)
{
  uint8_t byte = 0;
  uint8_t flushed = 0;

  while (HAL_UART_Receive(sensor->uart, &byte, 1U, GL_A22_FLUSH_TIMEOUT_MS) == HAL_OK)
  {
    if (flushed < 0xFFU)
    {
      flushed++;
    }
  }

  return flushed;
}

static HAL_StatusTypeDef GL_A22_ReadResponse(GL_A22_Handle *sensor, uint8_t *response, uint8_t response_len)
{
  uint32_t start_tick = HAL_GetTick();

  sensor->last_rx_count = 0U;
  for (uint8_t index = 0; index < GL_A22_RESPONSE_LEN; ++index)
  {
    sensor->last_rx_bytes[index] = 0U;
  }

  while (sensor->last_rx_count < response_len)
  {
    HAL_StatusTypeDef status = HAL_UART_Receive(sensor->uart,
                                                &response[sensor->last_rx_count],
                                                1U,
                                                GL_A22_RX_BYTE_TIMEOUT_MS);
    if (status == HAL_OK)
    {
      sensor->last_rx_bytes[sensor->last_rx_count] = response[sensor->last_rx_count];
      sensor->last_rx_count++;
      continue;
    }

    sensor->last_status = (uint8_t)((sensor->last_rx_count > 0U) ? HAL_ERROR : status);
    sensor->last_duration_ms = HAL_GetTick() - start_tick;
    return (sensor->last_rx_count > 0U) ? HAL_ERROR : status;
  }

  sensor->last_duration_ms = HAL_GetTick() - start_tick;
  return HAL_OK;
}

void GL_A22_Init(GL_A22_Handle *sensor, UART_HandleTypeDef *uart, uint8_t address)
{
  HAL_Delay(800);
  if ((sensor == NULL) || (uart == NULL))
  {
    return;
  }

  sensor->uart = uart;
  sensor->address = address;
  sensor->timeout_ms = GL_A22_UART_TIMEOUT_MS;
  sensor->last_duration_ms = 0U;
  sensor->last_reg_addr = 0U;
  sensor->last_phase = GL_A22_PHASE_IDLE;
  sensor->last_status = (uint8_t)HAL_OK;
  sensor->last_flush_bytes = 0U;
  sensor->last_rx_count = 0U;
  for (uint8_t index = 0; index < GL_A22_RESPONSE_LEN; ++index)
  {
    sensor->last_rx_bytes[index] = 0U;
  }
}

HAL_StatusTypeDef GL_A22_ReadRegisterAddressed(GL_A22_Handle *sensor, uint8_t request_address, uint16_t reg_addr, uint8_t *response_address, uint16_t *value)
{
  uint8_t request[GL_A22_REQUEST_LEN] = {0};
  uint8_t response[GL_A22_RESPONSE_LEN] = {0};
  uint16_t crc = 0;
  uint32_t start_tick = HAL_GetTick();
  HAL_StatusTypeDef status = HAL_ERROR;

  if ((sensor == NULL) || (sensor->uart == NULL) || (value == NULL))
  {
    return HAL_ERROR;
  }

  sensor->last_reg_addr = reg_addr;
  sensor->last_duration_ms = 0U;
  sensor->last_phase = GL_A22_PHASE_IDLE;
  sensor->last_status = (uint8_t)HAL_ERROR;
  sensor->last_flush_bytes = 0U;
  sensor->last_rx_count = 0U;
  for (uint8_t index = 0; index < GL_A22_RESPONSE_LEN; ++index)
  {
    sensor->last_rx_bytes[index] = 0U;
  }

  request[0] = request_address;
  request[1] = GL_A22_MODBUS_READ_FN;
  request[2] = (uint8_t)(reg_addr >> 8);
  request[3] = (uint8_t)(reg_addr & 0xFFU);
  request[4] = 0x00U;
  request[5] = 0x01U;

  crc = GL_A22_Crc16(request, GL_A22_REQUEST_LEN - 2U);
  request[6] = (uint8_t)(crc & 0xFFU);
  request[7] = (uint8_t)(crc >> 8);

  sensor->last_phase = GL_A22_PHASE_FLUSH;
  sensor->last_flush_bytes = GL_A22_FlushRx(sensor);

  sensor->last_phase = GL_A22_PHASE_TX;
  status = HAL_UART_Transmit(sensor->uart, request, GL_A22_REQUEST_LEN, sensor->timeout_ms);
  if (status != HAL_OK)
  {
    sensor->last_status = (uint8_t)status;
    sensor->last_duration_ms = HAL_GetTick() - start_tick;
    return status;
  }

  HAL_Delay(GL_A22_TX_TO_RX_DELAY_MS);

  sensor->last_phase = GL_A22_PHASE_RX;
  status = GL_A22_ReadResponse(sensor, response, GL_A22_RESPONSE_LEN);
  if (status != HAL_OK)
  {
    sensor->last_status = (uint8_t)status;
    if (sensor->last_duration_ms == 0U)
    {
      sensor->last_duration_ms = HAL_GetTick() - start_tick;
    }
    return status;
  }

  sensor->last_phase = GL_A22_PHASE_VALIDATE;
  if ((response[1] != GL_A22_MODBUS_READ_FN) ||
      (response[2] != GL_A22_RESPONSE_DATA_BYTES))
  {
    sensor->last_status = (uint8_t)HAL_ERROR;
    sensor->last_duration_ms = HAL_GetTick() - start_tick;
    return HAL_ERROR;
  }

  crc = GL_A22_Crc16(response, GL_A22_RESPONSE_LEN - 2U);
  if ((response[5] != (uint8_t)(crc & 0xFFU)) ||
      (response[6] != (uint8_t)(crc >> 8)))
  {
    sensor->last_status = (uint8_t)HAL_ERROR;
    sensor->last_duration_ms = HAL_GetTick() - start_tick;
    return HAL_ERROR;
  }

  if (response_address != NULL)
  {
    *response_address = response[0];
  }

  *value = (uint16_t)(((uint16_t)response[3] << 8) | response[4]);
  sensor->last_phase = GL_A22_PHASE_DONE;
  sensor->last_status = (uint8_t)HAL_OK;
  sensor->last_duration_ms = HAL_GetTick() - start_tick;
  return HAL_OK;
}

HAL_StatusTypeDef GL_A22_ReadRegister(GL_A22_Handle *sensor, uint16_t reg_addr, uint16_t *value)
{
  return GL_A22_ReadRegisterAddressed(sensor, sensor->address, reg_addr, NULL, value);
}

HAL_StatusTypeDef GL_A22_ReadProcessedDistanceMm(GL_A22_Handle *sensor, uint16_t *distance_mm)
{
  return GL_A22_ReadRegister(sensor, GL_A22_REG_PROCESSED_DISTANCE, distance_mm);
}

HAL_StatusTypeDef GL_A22_ReadRealtimeDistanceMm(GL_A22_Handle *sensor, uint16_t *distance_mm)
{
  return GL_A22_ReadRegister(sensor, GL_A22_REG_REALTIME_DISTANCE, distance_mm);
}
