#include "uart.h"
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

#define UART1_TX_FIFO_SIZE 4096U
#define UART_MSG_MAX       320U

static uint8_t uart1_tx_fifo[UART1_TX_FIFO_SIZE];
static volatile uint16_t uart1_head;    /* written by uart_printf only */
static volatile uint16_t uart1_tail;    /* written by TX complete ISR / kick */
static volatile uint16_t uart1_tx_len;  /* bytes in the transfer in flight, 0 = idle */

/* Must be called with the USART1 IRQ masked or from the USART1 IRQ context */
static void uart1_tx_kick(void)
{
  if (uart1_tx_len != 0U)
    return;

  uint16_t head = uart1_head;
  uint16_t tail = uart1_tail;
  if (head == tail)
    return;

  uint16_t n = (head > tail) ? (uint16_t)(head - tail) : (uint16_t)(UART1_TX_FIFO_SIZE - tail);
  if (HAL_UART_Transmit_IT(&huart1, &uart1_tx_fifo[tail], n) == HAL_OK)
    uart1_tx_len = n;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance != USART1)
    return;

  uart1_tail = (uint16_t)((uart1_tail + uart1_tx_len) % UART1_TX_FIFO_SIZE);
  uart1_tx_len = 0U;
  uart1_tx_kick();
}

/* Non-blocking: formats into the TX FIFO; the whole message is dropped if it does not fit */
void uart_printf(const char *fmt, ...)
{
  char msg[UART_MSG_MAX];
  va_list ap;
  va_start(ap, fmt);
  int len = vsnprintf(msg, sizeof(msg), fmt, ap);
  va_end(ap);
  if (len <= 0)
    return;
  if ((size_t)len >= sizeof(msg))
    len = (int)sizeof(msg) - 1;

  if (huart1.Instance == NULL)
    return;

  HAL_NVIC_DisableIRQ(USART1_IRQn);

  uint16_t head = uart1_head;
  uint16_t tail = uart1_tail;
  uint16_t used = (uint16_t)((head + UART1_TX_FIFO_SIZE - tail) % UART1_TX_FIFO_SIZE);
  uint16_t room = (uint16_t)(UART1_TX_FIFO_SIZE - 1U - used);

  if ((uint16_t)len <= room)
  {
    for (int i = 0; i < len; i++)
    {
      uart1_tx_fifo[head] = (uint8_t)msg[i];
      head = (uint16_t)((head + 1U) % UART1_TX_FIFO_SIZE);
    }
    uart1_head = head;
    uart1_tx_kick();
  }

  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&huart1);
  HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8);
  HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8);
  HAL_UARTEx_DisableFifoMode(&huart1);
}

void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 921600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&huart2);
  HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8);
  HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8);
  HAL_UARTEx_DisableFifoMode(&huart2);
}