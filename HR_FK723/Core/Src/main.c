/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include "network.h"
#include "network_data.h"
#include "GL_A22.h"
#include "Robstride.h"
#include "SteadyWin.h"
#include "wit_c_sdk.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Buffers allocated by X-CUBE-AI
//AI_ALIGNED(4) ai_i8 activations[AI_NETWORK_DATA_ACTIVATIONS_SIZE];
//AI_ALIGNED(4) ai_float in_data[AI_NETWORK_IN_1_SIZE];
//AI_ALIGNED(4) ai_float out_data[AI_NETWORK_OUT_1_SIZE];

// Force the intermediate activations into DTCM
__attribute__((section(".dtcmram"))) AI_ALIGNED(4) ai_i8 activations[AI_NETWORK_DATA_ACTIVATIONS_SIZE];

// Force your input (sensor states) and output (motor commands) into DTCM
__attribute__((section(".dtcmram"))) AI_ALIGNED(4) ai_i8 in_data[AI_NETWORK_IN_1_SIZE];
__attribute__((section(".dtcmram"))) AI_ALIGNED(4) ai_i8 out_data[AI_NETWORK_OUT_1_SIZE];

// AI Handle
ai_handle network;

/* SteadyWin motor count and index defines */
#define SW_MOTOR_COUNT     5
#define SW_IDX_HIP_PITCH   0
#define SW_IDX_HIP_ROLL    1
#define SW_IDX_HIP_YAW     2
#define SW_IDX_ANKLE_TOP   3
#define SW_IDX_ANKLE_BOT   4

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan2;
FDCAN_HandleTypeDef hfdcan3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
static uint32_t usart_last_tick = 0;
static uint32_t ultrasonic_last_tick = 0;
static char uart_msg[320];
static uint32_t led_last_tick = 0;
static const uint32_t led_toggle_interval = 500U;
static GL_A22_Handle gl_a22_sensor;
static uint32_t loop_debug_last_tick = 0;
static uint32_t loop_debug_counter = 0;
static uint8_t gl_a22_scan_done = 0;

/* RobStride motor instances */
static RobStride_Motor right_knee_motor;
static RobStride_Motor left_knee_motor;

/*
Hip pitch: forward - negative, backward - positive
Hip roll: right(outward) - positive, left(inward) - negative
Hip yaw: inward - positive, outward - negative
Knee pitch: forward - negative, backward - positive
Ankle top (outward, RSU scheme) rod goes up - positive, rod goes down - negative
Ankle bot (inward, RSU scheme) rod goes down - positive, rod goes up - negative
*/

/* From zero state:
Hip pitch max angles: forward 120°, backward 45°
Hip roll max angles: right(outward) 45°, left(inward) 20°
Hip yaw max angles: inward 45°, outward 45°
Knee max angles: forward 120°, backward 5°
Ankles max angles: for now just avoid hitting mechanical limits, e.g. 30° each direction
*/

/* ---- Leg initial positions template ----------------------------------------
 * Run once with PrintInitialAngles() and copy the printed counts here.
 * SW units: angle_a4 encoder counts (1 count = 360/16384 deg ≈ 0.022°)
 *           use with SteadyWin_AbsPosControl() cast to int32_t
 * RS units: radians (Pos_Info.Angle)
 * --------------------------------------------------------------------------- */
static const uint16_t right_leg_sw_init_pos[SW_MOTOR_COUNT] = {
    /* hip_pitch  (0x02) */  16082,
    /* hip_roll   (0x03) */  8155,
    /* hip_yaw    (0x04) */  11998,
    /* ankle_top  (0x05) */  13090,
    /* ankle_bot  (0x06) */  11216,
};

static const uint16_t left_leg_sw_init_pos[SW_MOTOR_COUNT] = {
    /* hip_pitch  (0x0A) */  1246,
    /* hip_roll   (0x0B) */  11532,
    /* hip_yaw    (0x0C) */  3998,
    /* ankle_top  (0x0D) */  15782,
    /* ankle_bot  (0x0E) */  12499,
};

static const float right_knee_init_pos = 1.0424f;
static const float left_knee_init_pos = 4.3837f;

static const uint16_t init_sw_max_speed[SW_MOTOR_COUNT] = {30, 10, 20, 60, 60};
static const uint8_t init_sw_accel[SW_MOTOR_COUNT] = {3, 3, 3, 3, 3};
static const uint16_t normal_sw_max_speed[SW_MOTOR_COUNT] = {300, 100, 200, 600, 600};
static const uint8_t normal_sw_accel[SW_MOTOR_COUNT] = {30, 30, 30, 30, 30};

/* SteadyWin motor instances (0=hip_pitch, 1=hip_roll, 2=hip_yaw, 3=ankle_top, 4=ankle_bot) */
static SteadyWin_Motor right_leg_sw_motors[SW_MOTOR_COUNT];
static SteadyWin_Motor left_leg_sw_motors[SW_MOTOR_COUNT];
static const uint8_t right_leg_sw_motor_addrs[SW_MOTOR_COUNT] = {0x02, 0x03, 0x04, 0x05, 0x06};
static const uint8_t left_leg_sw_motor_addrs[SW_MOTOR_COUNT] = {0x0A, 0x0B, 0x0C, 0x0D, 0x0E};
static const char * const leg_sw_joint_names[SW_MOTOR_COUNT] = {
  "hip_pitch", "hip_roll", "hip_yaw", "ankle_top", "ankle_bot"};

/* RobStride knee motor IDs */
static const uint8_t right_knee_motor_addr = 0x7F;
static const uint8_t left_knee_motor_addr = 0x7F; //not 0x3E

/* Last CAN frame received per motor bus via interrupt */
volatile uint8_t  right_leg_can_rx_data[8];
volatile uint32_t right_leg_can_rx_id;
volatile uint8_t  right_leg_can_rx_dlc;
volatile uint8_t  right_leg_can_rx_new;
volatile uint8_t  left_leg_can_rx_data[8];
volatile uint32_t left_leg_can_rx_id;
volatile uint8_t  left_leg_can_rx_dlc;
volatile uint8_t  left_leg_can_rx_new;

// Inference timing (DWT cycle counts)
static uint32_t inf_min_cycles = UINT32_MAX;
static uint32_t inf_max_cycles = 0;
static uint32_t inf_count = 0;

/* ---- CAN motor loop state machine ---- */
/*
 * 4-step ping-pong loop in CAN RX interrupt (both motors on one bus):
 *   S0: TX SW ReadMulti(0xA4)  → RX temp+current+speed+angle
 *   S1: TX SW SpeedCtrl(0xC1)  → RX speed feedback
 *   S2: TX RS MotorRequest     → RX angle+speed+torque+temp
 *   S3: TX RS SetSpeed(0x700A) → RX ack
 *   → measure loop time, restart at S0
 *
 * Timing: 4 CAN round-trips per loop at 1 Mbps ≈ 130 bits/frame.
 * For N motors: extend states per motor (read+speed per motor).
 */
#define SW_LOOP_SPEED_CMD  2000      /* 20 RPM in 0.01 RPM units */
#define RS_LOOP_SPEED_RAD  2.094f   /* 20 RPM in rad/s */
#define LOOP_S_SW_READ   0
#define LOOP_S_SW_SPEED  1
#define LOOP_S_RS_READ   2
#define LOOP_S_RS_SPEED  3

/* ---- WIT IMU data-ready flags ---- */
#define WIT_ACC_UPDATE    0x01
#define WIT_GYRO_UPDATE   0x02
#define WIT_ANGLE_UPDATE  0x04

// ---- Loop state machine variables ----
static volatile uint8_t  loop_active = 0;
static volatile uint8_t  loop_state  = LOOP_S_SW_READ;
static volatile uint32_t loop_count  = 0;
static volatile uint32_t loop_cyc_start = 0;
static volatile uint32_t loop_cyc_min = UINT32_MAX;
static volatile uint32_t loop_cyc_max = 0;

/* ---- WIT IMU state ---- */
static volatile uint8_t  wit_data_update = 0;
static uint32_t          wit_print_last_tick = 0;

/* ---- WIT debug counters (updated in ISR, read in main loop) ---- */
static volatile uint32_t wit_rx_total   = 0;  /* all frames received on FDCAN2        */
static volatile uint32_t wit_rx_valid   = 0;  /* frames that passed 0x55 + DLC>=8     */
static volatile uint32_t wit_angle_updates = 0; /* angle packets ~= one IMU sample      */
static volatile uint32_t wit_last_id    = 0;
static volatile uint8_t  wit_last_id_ext = 0; /* 1 = extended, 0 = standard           */
static volatile uint8_t  wit_last_dlc_code = 0;
static volatile uint8_t  wit_last_dlc   = 0;
static volatile uint8_t  wit_last_frame[8];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_FDCAN2_Init(void);
static void MX_FDCAN3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch)
{
  uint8_t byte = (uint8_t)ch;
  HAL_UART_Transmit(&huart1, &byte, 1, HAL_MAX_DELAY);
  return ch;
}

void uart_printf(const char *fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  int len = vsnprintf(uart_msg, sizeof(uart_msg), fmt, ap);
  va_end(ap);
  if (len > 0)
    HAL_UART_Transmit(&huart1, (uint8_t *)uart_msg, (uint16_t)len, HAL_MAX_DELAY);
}

/* GL-A22 Ultrasonic Sensor Module code

static const char *GL_A22_PhaseName(uint8_t phase)
{
  switch (phase)
  {
    case GL_A22_PHASE_IDLE:     return "idle";
    case GL_A22_PHASE_FLUSH:    return "flush";
    case GL_A22_PHASE_TX:       return "tx";
    case GL_A22_PHASE_RX:       return "rx";
    case GL_A22_PHASE_VALIDATE: return "validate";
    case GL_A22_PHASE_DONE:     return "done";
    default:                    return "unknown";
  }
}

static HAL_StatusTypeDef GL_A22_SetUartBaud(UART_HandleTypeDef *uart, uint32_t baud_rate)
{
  uart->Init.BaudRate = baud_rate;
  return HAL_UART_Init(uart);
}

static void GL_A22_PrintLastRx(const GL_A22_Handle *sensor)
{
  uart_printf("[GL-A22] rx_count=%u data=%02X %02X %02X %02X %02X %02X %02X\r\n",
              (unsigned int)sensor->last_rx_count,
              (unsigned int)sensor->last_rx_bytes[0],
              (unsigned int)sensor->last_rx_bytes[1],
              (unsigned int)sensor->last_rx_bytes[2],
              (unsigned int)sensor->last_rx_bytes[3],
              (unsigned int)sensor->last_rx_bytes[4],
              (unsigned int)sensor->last_rx_bytes[5],
              (unsigned int)sensor->last_rx_bytes[6]);
}

static void GL_A22_DebugScan(void)
{
  static const uint32_t baud_rates[] = {921600U, 460800U, 230400U, 115200U, 9600U, 19200U, 38400U, 57600U, 76800U, 4800U, 2400U, 14400U};
  const size_t baud_count = sizeof(baud_rates) / sizeof(baud_rates[0]);
  uint32_t original_baud = huart2.Init.BaudRate;
  uint16_t value = 0U;
  uint8_t response_address = 0U;

  uart_printf("[GL-A22] scan start\r\n");

  for (size_t index = 0; index < baud_count; ++index)
  {
    HAL_StatusTypeDef status;

    if (GL_A22_SetUartBaud(&huart2, baud_rates[index]) != HAL_OK)
    {
      uart_printf("[GL-A22] scan baud=%lu init failed\r\n", (unsigned long)baud_rates[index]);
      continue;
    }

    GL_A22_Init(&gl_a22_sensor, &huart2, GL_A22_DEFAULT_ADDRESS);
    uart_printf("[GL-A22] scan try baud=%lu\r\n", (unsigned long)baud_rates[index]);

    status = GL_A22_ReadRegisterAddressed(&gl_a22_sensor, 0xFFU, 0x0200U, &response_address, &value);
    if (status == HAL_OK)
    {
      uart_printf("[GL-A22] scan hit baud=%lu sensor_addr=0x%02X\r\n",
                  (unsigned long)baud_rates[index],
                  (unsigned int)response_address);
      gl_a22_sensor.address = response_address;
      gl_a22_scan_done = 1U;
      return;
    }

    uart_printf("[GL-A22] scan miss baud=%lu status=%d phase=%s elapsed=%lu ms\r\n",
                (unsigned long)baud_rates[index],
                (int)status,
                GL_A22_PhaseName(gl_a22_sensor.last_phase),
                (unsigned long)gl_a22_sensor.last_duration_ms);
    GL_A22_PrintLastRx(&gl_a22_sensor);
  }

  if (GL_A22_SetUartBaud(&huart2, original_baud) == HAL_OK)
  {
    GL_A22_Init(&gl_a22_sensor, &huart2, GL_A22_DEFAULT_ADDRESS);
  }

  uart_printf("[GL-A22] scan no response\r\n");
}
*/

/* WIT HWT901B-CAN IMU support

// Delay callback required by the WIT SDK
static void WIT_Delayms(uint16_t ucMs)
{
  HAL_Delay(ucMs);
}

// CAN transmit adapter: sends a standard-ID frame on FDCAN2
static void WIT_CAN_Send(uint8_t ucStdId, uint8_t *p_ucData, uint32_t uiLen)
{
  FDCAN_TxHeaderTypeDef txHdr;
  txHdr.Identifier          = ucStdId;
  txHdr.IdType              = FDCAN_STANDARD_ID;
  txHdr.TxFrameType         = FDCAN_DATA_FRAME;
  txHdr.DataLength          = (uint32_t)uiLen << 16; // FDCAN DLC encoding
  txHdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  txHdr.BitRateSwitch       = FDCAN_BRS_OFF;
  txHdr.FDFormat            = FDCAN_CLASSIC_CAN;
  txHdr.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
  txHdr.MessageMarker       = 0;
  HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &txHdr, p_ucData);
}

// Called by the WIT SDK whenever a register batch is updated
static void WIT_SensorDataUpdate(uint32_t uiReg, uint32_t uiRegNum)
{
  uint32_t i;
  uint8_t angle_updated = 0;
  wit_rx_valid++;   // SDK accepted this frame (passed 0x55 + DLC check)
  for (i = 0; i < uiRegNum; i++)
  {
    switch (uiReg)
    {
      case AZ:    wit_data_update |= WIT_ACC_UPDATE;   break;
      case GZ:    wit_data_update |= WIT_GYRO_UPDATE;  break;
      case Yaw:
        wit_data_update |= WIT_ANGLE_UPDATE;
        angle_updated = 1;
        break;
      default: break;
    }
    uiReg++;
  }

  if (angle_updated)
    wit_angle_updates++;
}

// Initialise FDCAN2 filters/start and the WIT SDK 
void WIT_IMU_Init(void)
{
  uart_printf("\r\n======== WIT IMU Init ========\r\n");

  // Accept all standard-ID frames into FIFO0 
  FDCAN_FilterTypeDef filter;
  filter.IdType       = FDCAN_STANDARD_ID;
  filter.FilterIndex  = 0;
  filter.FilterType   = FDCAN_FILTER_MASK;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter.FilterID1    = 0;
  filter.FilterID2    = 0;
  HAL_FDCAN_ConfigFilter(&hfdcan2, &filter);

  HAL_FDCAN_ConfigGlobalFilter(&hfdcan2,
                               FDCAN_ACCEPT_IN_RX_FIFO0,
                               FDCAN_ACCEPT_IN_RX_FIFO0,
                               FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
  HAL_FDCAN_Start(&hfdcan2);
  HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

  // WIT SDK setup: CAN protocol, device address 0x50 
  WitInit(WIT_PROTOCOL_CAN, 0x50);
  WitRegisterCallBack(WIT_SensorDataUpdate);
  WitCanWriteRegister(WIT_CAN_Send);
  WitDelayMsRegister(WIT_Delayms);

  uart_printf("[WIT] FDCAN2 started, SDK initialised (addr=0x50)\r\n");
  uart_printf("==============================\r\n\r\n");
}

// Call from the main loop: prints accel/gyro/angle + parsed WIT update rate once per second 
void WIT_IMU_PrintIfDue(void)
{
  static uint32_t wit_prev_angle_updates = 0;
  uint32_t now = HAL_GetTick();
  if ((now - wit_print_last_tick) < 1000U)
    return;
  wit_print_last_tick = now;

  // --- always send a poll request so the device responds even if auto-send is off --- 
  WitReadReg(AX,   3);  // accel
  HAL_Delay(3);
  WitReadReg(GX,   3);  // gyro
  HAL_Delay(3);
  WitReadReg(Roll, 3);  // angles
  HAL_Delay(3);

  uint32_t angle_updates = wit_angle_updates;
  uint32_t rate = angle_updates - wit_prev_angle_updates;
  wit_prev_angle_updates = angle_updates;

  // --- converted values --- 
  float acc[3], gyro[3], angle[3];
  for (int i = 0; i < 3; i++)
  {
    acc[i]   = sReg[AX   + i] / 32768.0f * 16.0f;
    gyro[i]  = sReg[GX   + i] / 32768.0f * 2000.0f;
    angle[i] = sReg[Roll + i] / 32768.0f * 180.0f;
  }
  //uart_printf("[WIT] Rate: %lu Hz  |  Acc: %.3f %.3f %.3f g  |  Gyro: %.2f %.2f %.2f deg/s  |  Angle(RPY): %.2f %.2f %.2f deg\r\n",
  //  (unsigned long)rate,
  //  (double)acc[0], (double)acc[1], (double)acc[2],
  //  (double)gyro[0], (double)gyro[1], (double)gyro[2],
  //  (double)angle[0], (double)angle[1], (double)angle[2]);
  wit_data_update = 0;
}
*/

static uint8_t FDCAN_DLCToBytes(uint32_t dlc_code)
{
  switch (dlc_code)
  {
    case FDCAN_DLC_BYTES_0:  return 0;
    case FDCAN_DLC_BYTES_1:  return 1;
    case FDCAN_DLC_BYTES_2:  return 2;
    case FDCAN_DLC_BYTES_3:  return 3;
    case FDCAN_DLC_BYTES_4:  return 4;
    case FDCAN_DLC_BYTES_5:  return 5;
    case FDCAN_DLC_BYTES_6:  return 6;
    case FDCAN_DLC_BYTES_7:  return 7;
    case FDCAN_DLC_BYTES_8:  return 8;
    case FDCAN_DLC_BYTES_12: return 12;
    case FDCAN_DLC_BYTES_16: return 16;
    case FDCAN_DLC_BYTES_20: return 20;
    case FDCAN_DLC_BYTES_24: return 24;
    case FDCAN_DLC_BYTES_32: return 32;
    case FDCAN_DLC_BYTES_48: return 48;
    case FDCAN_DLC_BYTES_64: return 64;
    default:                 return 0;
  }
}

static void CaptureMotorBusRxFrame(volatile uint8_t *bus_rx_data,
                                   volatile uint32_t *bus_rx_id,
                                   volatile uint8_t *bus_rx_dlc,
                                   volatile uint8_t *bus_rx_new,
                                   const FDCAN_RxHeaderTypeDef *rxHdr,
                                   const uint8_t *rxData)
{
  for (int index = 0; index < 8; ++index)
    bus_rx_data[index] = rxData[index];

  *bus_rx_id = rxHdr->Identifier;
  *bus_rx_dlc = FDCAN_DLCToBytes(rxHdr->DataLength);
  *bus_rx_new = 1;
}

static void ApplySteadyWinMotionProfile(SteadyWin_Motor *leg_sw_motors,
                                        const uint16_t *max_speeds,
                                        const uint8_t *accels)
{
  for (int index = 0; index < SW_MOTOR_COUNT; ++index) {
    SteadyWin_SetMaxSpeed(&leg_sw_motors[index], max_speeds[index]);
    HAL_Delay(10);
  }

  for (int index = 0; index < SW_MOTOR_COUNT; ++index) {
    SteadyWin_SetAccel(&leg_sw_motors[index], accels[index]);
    HAL_Delay(10);
  }
}

/*static void RefreshLeftLegInitPose(void)
{
  for (int index = 0; index < SW_MOTOR_COUNT; ++index) {
    left_leg_can_rx_new = 0;
    SteadyWin_ReadMulti(&left_leg_sw_motors[index]);
    HAL_Delay(50);

    if (!left_leg_can_rx_new || left_leg_can_rx_data[0] != SW_CMD_READ_MULTI)
      continue;

    left_leg_sw_init_pos[index] = (uint16_t)left_leg_can_rx_data[6] |
                                  ((uint16_t)left_leg_can_rx_data[7] << 8);
    left_leg_sw_motors[index].fb.angle_a4 = left_leg_sw_init_pos[index];
  }

  left_leg_can_rx_new = 0;
  RobStride_MotorRequest(&left_knee_motor);
  HAL_Delay(30);
  if (left_leg_can_rx_new)
    left_knee_init_pos = left_knee_motor.Pos_Info.Angle;
}*/

void PrintInitialAngles(void) {
  uart_printf("\r\n---- Right Leg Initial Angles ----\r\n");

  /* Poll each SteadyWin motor — parse directly from can_rx_data (same method
   * as version check in init) so we are independent of SteadyWin_Analysis
   * ID matching.  ReadMulti response layout:
   *   [0]=0xA4  [1]=temp  [2-3]=current  [4-5]=speed  [6-7]=angle(LE) */
  for (int i = 0; i < SW_MOTOR_COUNT; i++) {
    right_leg_can_rx_new = 0;
    SteadyWin_ReadMulti(&right_leg_sw_motors[i]);
    HAL_Delay(50);

    if (!right_leg_can_rx_new) {
      uart_printf("[R-SW 0x%02X %-9s] NO RESPONSE\r\n",
        right_leg_sw_motors[i].dev_addr, leg_sw_joint_names[i]);
    } else if (right_leg_can_rx_data[0] != SW_CMD_READ_MULTI) {
      uart_printf("[R-SW 0x%02X %-9s] unexpected cmd=0x%02X [resp ID=0x%03lX]\r\n",
        right_leg_sw_motors[i].dev_addr, leg_sw_joint_names[i],
        right_leg_can_rx_data[0], (unsigned long)right_leg_can_rx_id);
    } else {
      /* Parse angle from raw bytes and sync struct */
      uint16_t raw = (uint16_t)right_leg_can_rx_data[6] | ((uint16_t)right_leg_can_rx_data[7] << 8);
      right_leg_sw_motors[i].fb.angle_a4 = raw;
      float deg = (float)raw * (360.0f / 16384.0f);
      uart_printf("[R-SW 0x%02X %-9s] %.2f deg  (%u counts)  [resp ID=0x%03lX]\r\n",
        right_leg_sw_motors[i].dev_addr, leg_sw_joint_names[i],
        (double)deg, (unsigned)raw, (unsigned long)right_leg_can_rx_id);
    }
    right_leg_can_rx_new = 0;
  }

  /* Poll RobStride */
  right_leg_can_rx_new = 0;
  RobStride_MotorRequest(&right_knee_motor);
  HAL_Delay(30);
  uart_printf("[R-RS 0x%02X right_knee] %.4f rad  (%.2f deg)\r\n",
    right_knee_motor.CAN_ID, (double)right_knee_motor.Pos_Info.Angle,
    (double)(right_knee_motor.Pos_Info.Angle * (180.0f / 3.14159265f)));

  uart_printf("\r\n---- Left Leg Initial Angles ----\r\n");

  for (int i = 0; i < SW_MOTOR_COUNT; i++) {
    left_leg_can_rx_new = 0;
    SteadyWin_ReadMulti(&left_leg_sw_motors[i]);
    HAL_Delay(50);

    if (!left_leg_can_rx_new) {
      uart_printf("[L-SW 0x%02X %-9s] NO RESPONSE\r\n",
        left_leg_sw_motors[i].dev_addr, leg_sw_joint_names[i]);
    } else if (left_leg_can_rx_data[0] != SW_CMD_READ_MULTI) {
      uart_printf("[L-SW 0x%02X %-9s] unexpected cmd=0x%02X [resp ID=0x%03lX]\r\n",
        left_leg_sw_motors[i].dev_addr, leg_sw_joint_names[i],
        left_leg_can_rx_data[0], (unsigned long)left_leg_can_rx_id);
    } else {
      uint16_t raw = (uint16_t)left_leg_can_rx_data[6] | ((uint16_t)left_leg_can_rx_data[7] << 8);
      left_leg_sw_motors[i].fb.angle_a4 = raw;
      float deg = (float)raw * (360.0f / 16384.0f);
      uart_printf("[L-SW 0x%02X %-9s] %.2f deg  (%u counts)  [resp ID=0x%03lX]\r\n",
        left_leg_sw_motors[i].dev_addr, leg_sw_joint_names[i],
        (double)deg, (unsigned)raw, (unsigned long)left_leg_can_rx_id);
    }
    left_leg_can_rx_new = 0;
  }

  left_leg_can_rx_new = 0;
  RobStride_MotorRequest(&left_knee_motor);
  HAL_Delay(30);
  if (left_leg_can_rx_new) {
    uart_printf("[L-RS 0x%02X left_knee ] %.4f rad  (%.2f deg)\r\n",
      left_knee_motor.CAN_ID, (double)left_knee_motor.Pos_Info.Angle,
      (double)(left_knee_motor.Pos_Info.Angle * (180.0f / 3.14159265f)));
  } else {
    uart_printf("[L-RS 0x%02X left_knee ] NO RESPONSE\r\n",
      left_knee_motor.CAN_ID);
  }

  uart_printf("Copy measured counts/rad into the leg init position constants in main.c\r\n"
    "------------------------------\r\n\r\n");
}

void RightLeg_RobStride_CAN_Init(void) {
  uart_printf("\r\n======== Right RobStride Init ========\r\n");

  /* Extended ID filter — accept all */
  FDCAN_FilterTypeDef extFilter;
  extFilter.IdType       = FDCAN_EXTENDED_ID;
  extFilter.FilterIndex  = 0;
  extFilter.FilterType   = FDCAN_FILTER_MASK;
  extFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  extFilter.FilterID1    = 0;
  extFilter.FilterID2    = 0;
  HAL_FDCAN_ConfigFilter(&hfdcan3, &extFilter);

  /* Standard ID filter — accept all (needed for MIT mode responses) */
  FDCAN_FilterTypeDef stdFilter;
  stdFilter.IdType       = FDCAN_STANDARD_ID;
  stdFilter.FilterIndex  = 0;
  stdFilter.FilterType   = FDCAN_FILTER_MASK;
  stdFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  stdFilter.FilterID1    = 0;
  stdFilter.FilterID2    = 0;
  HAL_FDCAN_ConfigFilter(&hfdcan3, &stdFilter);

  /* Accept non-matching frames into FIFO0 as well */
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_ACCEPT_IN_RX_FIFO0,
                                FDCAN_ACCEPT_IN_RX_FIFO0,
                                FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
  HAL_FDCAN_Start(&hfdcan3);
  HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

  /* Init right knee motor (CAN ID 0x7F, private protocol) */
  RobStride_Init(&right_knee_motor, right_knee_motor_addr, 0, &hfdcan3);
  HAL_Delay(50);

  /* Disable first to clear any previous state */
  RobStride_Disable(&right_knee_motor, 1);
  HAL_Delay(50);

  /* Set speed control mode */
  RobStride_SetParameter(&right_knee_motor, 0x7005, Speed_control_mode, Set_mode);
  HAL_Delay(20);

  /* Enable the motor */
  RobStride_Enable(&right_knee_motor);
  HAL_Delay(20);

  /* Set current limit 5 A */
  //RobStride_SetParameter(&right_knee_motor, 0x7018, 5.0f, Set_parameter);
  //HAL_Delay(20);

  /* Set acceleration */
  //RobStride_SetParameter(&right_knee_motor, 0x7022, 10.0f, Set_parameter);
  //HAL_Delay(20);

  /* Set speed reference: 20 RPM ≈ 2.094 rad/s */
  //RobStride_SetParameter(&right_knee_motor, 0x700A, RS_LOOP_SPEED_RAD, Set_parameter);
  HAL_Delay(20);

  /* Disable proactive reporting — we poll via MotorRequest in the loop */
  RobStride_ProactiveEscalationSet(&right_knee_motor, 0x00);
  HAL_Delay(20);

  /* Verify communication */
  right_leg_can_rx_new = 0;
  RobStride_MotorRequest(&right_knee_motor);
  HAL_Delay(50);
  if (right_leg_can_rx_new) {
    right_leg_can_rx_new = 0;
    uart_printf("[Right RobStride] Using CAN ID 0x%02X on FDCAN3 - Communication OK\r\n", right_knee_motor.CAN_ID);
  } else {
    uart_printf("[Right RobStride] Using CAN ID 0x%02X on FDCAN3 - NO RESPONSE!\r\n", right_knee_motor.CAN_ID);
  }
  uart_printf("\r\n");
}

void RightLeg_SteadyWin_CAN_Init(void) {
  uart_printf("======== Right Leg SteadyWin Init ========\r\n");

  /* NOTE: FDCAN3 filters, start, and notifications already done by RightLeg_RobStride_CAN_Init */

  for (int i = 0; i < SW_MOTOR_COUNT; i++) {
    SteadyWin_Init(&right_leg_sw_motors[i], right_leg_sw_motor_addrs[i], &hfdcan3);
    HAL_Delay(20);

    /* Read driver version (0xA0) to check communication */
    right_leg_can_rx_new = 0;
    SteadyWin_ReadVersion(&right_leg_sw_motors[i]);
    HAL_Delay(50);

    if (right_leg_can_rx_new) {
      right_leg_can_rx_new = 0;
      uint16_t boot_ver = (uint16_t)right_leg_can_rx_data[1] | ((uint16_t)right_leg_can_rx_data[2] << 8);
      uint16_t app_ver  = (uint16_t)right_leg_can_rx_data[3] | ((uint16_t)right_leg_can_rx_data[4] << 8);
      uint16_t hw_ver   = (uint16_t)right_leg_can_rx_data[5] | ((uint16_t)right_leg_can_rx_data[6] << 8);
      uint8_t  can_ver  = right_leg_can_rx_data[7];
      uart_printf("[R-SW 0x%02X %-9s] OK  (Boot=%u App=%u HW=%u CAN=%u)\r\n",
        right_leg_sw_motors[i].dev_addr, leg_sw_joint_names[i],
        boot_ver, app_ver, hw_ver, can_ver);
    } else {
      uart_printf("[R-SW 0x%02X %-9s] NO RESPONSE\r\n",
        right_leg_sw_motors[i].dev_addr, leg_sw_joint_names[i]);
    }

    /* Clear any faults */
    SteadyWin_ClearFault(&right_leg_sw_motors[i]);
    HAL_Delay(20);
  }

  uart_printf("=============================\r\n\r\n");
}

void LeftLeg_RobStride_CAN_Init(void) {
  uart_printf("======== Left RobStride Init ========\r\n");

  /* Extended ID filter — accept all */
  FDCAN_FilterTypeDef extFilter;
  extFilter.IdType       = FDCAN_EXTENDED_ID;
  extFilter.FilterIndex  = 0;
  extFilter.FilterType   = FDCAN_FILTER_MASK;
  extFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  extFilter.FilterID1    = 0;
  extFilter.FilterID2    = 0;
  HAL_FDCAN_ConfigFilter(&hfdcan1, &extFilter);

  /* Standard ID filter — accept all (needed for MIT mode responses) */
  FDCAN_FilterTypeDef stdFilter;
  stdFilter.IdType       = FDCAN_STANDARD_ID;
  stdFilter.FilterIndex  = 0;
  stdFilter.FilterType   = FDCAN_FILTER_MASK;
  stdFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  stdFilter.FilterID1    = 0;
  stdFilter.FilterID2    = 0;
  HAL_FDCAN_ConfigFilter(&hfdcan1, &stdFilter);

  HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_ACCEPT_IN_RX_FIFO0,
                                FDCAN_ACCEPT_IN_RX_FIFO0,
                                FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
  HAL_FDCAN_Start(&hfdcan1);
  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

  RobStride_Init(&left_knee_motor, left_knee_motor_addr, 0, &hfdcan1);
  HAL_Delay(50);

  RobStride_Disable(&left_knee_motor, 1);
  HAL_Delay(50);

  RobStride_SetParameter(&left_knee_motor, 0x7005, Speed_control_mode, Set_mode);
  HAL_Delay(20);

  RobStride_Enable(&left_knee_motor);
  HAL_Delay(20);

  RobStride_ProactiveEscalationSet(&left_knee_motor, 0x00);
  HAL_Delay(20);

  left_leg_can_rx_new = 0;
  RobStride_MotorRequest(&left_knee_motor);
  HAL_Delay(50);
  if (left_leg_can_rx_new) {
    left_leg_can_rx_new = 0;
    uart_printf("[Left RobStride] Using CAN ID 0x%02X on FDCAN1 - Communication OK\r\n", left_knee_motor.CAN_ID);
  } else {
    uart_printf("[Left RobStride] Using CAN ID 0x%02X on FDCAN1 - NO RESPONSE!\r\n", left_knee_motor.CAN_ID);
  }
  uart_printf("\r\n");
}

void LeftLeg_SteadyWin_CAN_Init(void) {
  uart_printf("======== Left Leg SteadyWin Init ========\r\n");

  /* NOTE: FDCAN1 filters, start, and notifications already done by LeftLeg_RobStride_CAN_Init */

  for (int i = 0; i < SW_MOTOR_COUNT; i++) {
    SteadyWin_Init(&left_leg_sw_motors[i], left_leg_sw_motor_addrs[i], &hfdcan1);
    HAL_Delay(20);

    left_leg_can_rx_new = 0;
    SteadyWin_ReadVersion(&left_leg_sw_motors[i]);
    HAL_Delay(50);

    if (left_leg_can_rx_new) {
      left_leg_can_rx_new = 0;
      uint16_t boot_ver = (uint16_t)left_leg_can_rx_data[1] | ((uint16_t)left_leg_can_rx_data[2] << 8);
      uint16_t app_ver  = (uint16_t)left_leg_can_rx_data[3] | ((uint16_t)left_leg_can_rx_data[4] << 8);
      uint16_t hw_ver   = (uint16_t)left_leg_can_rx_data[5] | ((uint16_t)left_leg_can_rx_data[6] << 8);
      uint8_t  can_ver  = left_leg_can_rx_data[7];
      uart_printf("[L-SW 0x%02X %-9s] OK  (Boot=%u App=%u HW=%u CAN=%u)\r\n",
        left_leg_sw_motors[i].dev_addr, leg_sw_joint_names[i],
        boot_ver, app_ver, hw_ver, can_ver);
    } else {
      uart_printf("[L-SW 0x%02X %-9s] NO RESPONSE\r\n",
        left_leg_sw_motors[i].dev_addr, leg_sw_joint_names[i]);
    }

    SteadyWin_ClearFault(&left_leg_sw_motors[i]);
    HAL_Delay(20);
  }

  uart_printf("============================\r\n\r\n");
}

// Called from FDCAN RX FIFO0 interrupt for FDCAN2 (WIT IMU), FDCAN1 (left leg), and FDCAN3 (right leg)
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)
  {
    FDCAN_RxHeaderTypeDef rxHdr;
    uint8_t rxData[8];
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHdr, rxData) == HAL_OK)
    {
      /* ---- FDCAN2: WIT IMU sensor ---- */
      /* NOTE: returns here — loop_active has NO effect on FDCAN2/WIT */
      if (hfdcan->Instance == FDCAN2)
      {
        uint8_t dlc = FDCAN_DLCToBytes(rxHdr.DataLength);

        /* Capture raw frame for diagnostics */
        wit_rx_total++;
        wit_last_id     = rxHdr.Identifier;
        wit_last_id_ext = (rxHdr.IdType == FDCAN_EXTENDED_ID) ? 1u : 0u;
        wit_last_dlc_code = (uint8_t)rxHdr.DataLength;
        wit_last_dlc    = dlc;
        for (int i = 0; i < 8; i++) wit_last_frame[i] = rxData[i];

        WitCanDataIn(rxData, dlc);
        return;  /* early return — FDCAN3 motor code below is never reached */
      }

      /* ---- FDCAN1: left leg SteadyWin bus ---- */
      if (hfdcan->Instance == FDCAN1)
      {
        CaptureMotorBusRxFrame(left_leg_can_rx_data, &left_leg_can_rx_id,
                               &left_leg_can_rx_dlc, &left_leg_can_rx_new,
                               &rxHdr, rxData);

        if (rxHdr.IdType == FDCAN_EXTENDED_ID)
          RobStride_Analysis(&left_knee_motor, rxData, rxHdr.Identifier);
        else {
          for (int i = 0; i < SW_MOTOR_COUNT; i++)
            SteadyWin_Analysis(&left_leg_sw_motors[i], rxData, left_leg_can_rx_dlc, rxHdr.Identifier);
        }
        return;
      }

      /* ---- FDCAN3: right leg motor bus (loop_active gates only this section) ---- */
      CaptureMotorBusRxFrame(right_leg_can_rx_data, &right_leg_can_rx_id,
                             &right_leg_can_rx_dlc, &right_leg_can_rx_new,
                             &rxHdr, rxData);

      /* Dispatch to protocol parsers (always, regardless of loop state) */
      if (rxHdr.IdType == FDCAN_EXTENDED_ID)
        RobStride_Analysis(&right_knee_motor, rxData, rxHdr.Identifier);
      else {
        for (int i = 0; i < SW_MOTOR_COUNT; i++)
          SteadyWin_Analysis(&right_leg_sw_motors[i], rxData, right_leg_can_rx_dlc, rxHdr.Identifier);
      }

      if (!loop_active) return;

      /* ---- 4-state CAN ping-pong loop ---- */

      /* SteadyWin response: standard ID matching hip-pitch motor (loop motor).
       * Accept both bare dev_addr and 0x100|dev_addr response formats. */
      uint32_t sw_id = right_leg_sw_motors[SW_IDX_HIP_PITCH].dev_addr;
      if (rxHdr.IdType == FDCAN_STANDARD_ID &&
          (rxHdr.Identifier == sw_id || rxHdr.Identifier == (0x100U | sw_id)))
      {
        uint8_t cmd = rxData[0];
        if (loop_state == LOOP_S_SW_READ && cmd == SW_CMD_READ_MULTI)
        {
          /* S0→S1: got read data, send speed command */
          loop_state = LOOP_S_SW_SPEED;
          SteadyWin_SpeedControl(&right_leg_sw_motors[SW_IDX_HIP_PITCH], SW_LOOP_SPEED_CMD);
        }
        else if (loop_state == LOOP_S_SW_SPEED && cmd == SW_CMD_SPEED_CTRL)
        {
          /* S1→S2: SW done, now read RobStride */
          loop_state = LOOP_S_RS_READ;
          RobStride_MotorRequest(&right_knee_motor);
        }
      }

      /* RobStride response: extended ID with our motor CAN_ID as source */
      if (rxHdr.IdType == FDCAN_EXTENDED_ID &&
          ((rxHdr.Identifier >> 8) & 0xFF) == right_knee_motor.CAN_ID)
      {
        if (loop_state == LOOP_S_RS_READ)
        {
          /* S2→S3: got RS feedback, send speed command */
          loop_state = LOOP_S_RS_SPEED;
          RobStride_SetParameter(&right_knee_motor, 0x700A, RS_LOOP_SPEED_RAD, Set_parameter);
        }
        else if (loop_state == LOOP_S_RS_SPEED)
        {
          /* S3→S0: loop complete, measure and restart */
          uint32_t now = DWT->CYCCNT;
          uint32_t elapsed = now - loop_cyc_start;
          if (elapsed < loop_cyc_min) loop_cyc_min = elapsed;
          if (elapsed > loop_cyc_max) loop_cyc_max = elapsed;
          loop_count++;
          loop_cyc_start = DWT->CYCCNT;
          loop_state = LOOP_S_SW_READ;
          SteadyWin_ReadMulti(&right_leg_sw_motors[SW_IDX_HIP_PITCH]);
        }
      }
    }
  }
}

// Initial small acceleration and speed for smooth movement to init position
void Motors_speed_accel_init_settings(void) {
  /* Move RobStride to init position */
  RobStride_Disable(&right_knee_motor, 0);
  HAL_Delay(50);
  right_knee_motor.drw.run_mode.data = 0;
  right_knee_motor.Motor_Set_All.set_limit_speed = 0.1f;
  right_knee_motor.Motor_Set_All.set_acceleration = 0.5f;
  RobStride_Disable(&left_knee_motor, 0);
  HAL_Delay(50);
  left_knee_motor.drw.run_mode.data = 0;
  left_knee_motor.Motor_Set_All.set_limit_speed = 0.1f;
  left_knee_motor.Motor_Set_All.set_acceleration = 0.5f;
  HAL_Delay(10);

  /* ---- Set SteadyWin max speed & accel for slow, smooth movement ---- */
  ApplySteadyWinMotionProfile(right_leg_sw_motors, init_sw_max_speed, init_sw_accel);
  ApplySteadyWinMotionProfile(left_leg_sw_motors, init_sw_max_speed, init_sw_accel);
}

// Normal operation acceleration and speed settings
void Motors_speed_accel_normal_settings(void) {
  /* ---- Prepare RobStride: switch to position mode ---- */
  RobStride_Disable(&right_knee_motor, 0);
  HAL_Delay(50);
  right_knee_motor.drw.run_mode.data = 0;
  right_knee_motor.Motor_Set_All.set_limit_speed  = 0.5f;
  right_knee_motor.Motor_Set_All.set_acceleration = 1.0f;
  RobStride_Disable(&left_knee_motor, 0);
  HAL_Delay(50);
  left_knee_motor.drw.run_mode.data = 0;
  left_knee_motor.Motor_Set_All.set_limit_speed  = 0.5f;
  left_knee_motor.Motor_Set_All.set_acceleration = 1.0f;
  HAL_Delay(10);

  /* ---- Set SteadyWin max speed & accel for normal operation ---- */
  ApplySteadyWinMotionProfile(right_leg_sw_motors, normal_sw_max_speed, normal_sw_accel);
  ApplySteadyWinMotionProfile(left_leg_sw_motors, normal_sw_max_speed, normal_sw_accel);
}

// Move all motors to their initial positions (blocking)
void Move_to_init_pos(void) {
  uart_printf("[BOOT] Move_to_init_pos: start\r\n");

  /* ---- Set SteadyWin max speed & accel for slow, smooth movement ---- */
  Motors_speed_accel_init_settings();

  //RefreshLeftLegInitPose();

  /* Move SteadyWin motors to init position with blocking calls */
  for (int i = 0; i < SW_MOTOR_COUNT; i++) {
    SteadyWin_AbsPosControl(&right_leg_sw_motors[i], (int32_t)right_leg_sw_init_pos[i]);
    HAL_Delay(20);

    SteadyWin_AbsPosControl(&left_leg_sw_motors[i], (int32_t)left_leg_sw_init_pos[i]);
    HAL_Delay(20);
  }

  /* Move RobStride to init position */
  RobStride_PosControl(&right_knee_motor, 0.1f, right_knee_init_pos);
  RobStride_PosControl(&left_knee_motor, 0.1f, left_knee_init_pos);
  HAL_Delay(2000);

  uart_printf("[BOOT] Move_to_init_pos: done\r\n");
}

// Simple movement program for testing motors (blocking)
void Simple_movement_program(void) {
  Motors_speed_accel_normal_settings();

  RefreshLeftLegInitPose();

  uint32_t hip_pitch_target_angle = right_leg_sw_init_pos[SW_IDX_HIP_PITCH] - (16384.0f / 360.0f) * 60.0f;
  uint32_t hip_roll_target_angle  = right_leg_sw_init_pos[SW_IDX_HIP_ROLL]  - (16384.0f / 360.0f) * 20.0f;
  uint32_t hip_yaw_target_angle   = right_leg_sw_init_pos[SW_IDX_HIP_YAW]   + (16384.0f / 360.0f) * 35.0f;
  float knee_pitch_target_angle = right_knee_init_pos + (3.14159265f / 180.0f) * 50.0f;
  uint32_t ankle_top_target_angle_first = right_leg_sw_init_pos[SW_IDX_ANKLE_TOP] + (16384.0f / 360.0f) * 30.0f;
  uint32_t ankle_bot_target_angle_first = right_leg_sw_init_pos[SW_IDX_ANKLE_BOT] - (16384.0f / 360.0f) * 30.0f;
  uint32_t ankle_top_target_angle_second = right_leg_sw_init_pos[SW_IDX_ANKLE_TOP] - (16384.0f / 360.0f) * 60.0f;
  uint32_t ankle_bot_target_angle_second = right_leg_sw_init_pos[SW_IDX_ANKLE_BOT] + (16384.0f / 360.0f) * 60.0f;
  uint32_t left_hip_pitch_target_angle = left_leg_sw_init_pos[SW_IDX_HIP_PITCH] - (16384.0f / 360.0f) * 60.0f;
  uint32_t left_hip_roll_target_angle  = left_leg_sw_init_pos[SW_IDX_HIP_ROLL]  - (16384.0f / 360.0f) * 20.0f;
  uint32_t left_hip_yaw_target_angle   = left_leg_sw_init_pos[SW_IDX_HIP_YAW]   + (16384.0f / 360.0f) * 35.0f;
  float left_knee_pitch_target_angle = left_knee_init_pos + (3.14159265f / 180.0f) * 50.0f;
  uint32_t left_ankle_top_target_angle_first = left_leg_sw_init_pos[SW_IDX_ANKLE_TOP] + (16384.0f / 360.0f) * 30.0f;
  uint32_t left_ankle_bot_target_angle_first = left_leg_sw_init_pos[SW_IDX_ANKLE_BOT] - (16384.0f / 360.0f) * 30.0f;
  uint32_t left_ankle_top_target_angle_second = left_leg_sw_init_pos[SW_IDX_ANKLE_TOP] - (16384.0f / 360.0f) * 60.0f;
  uint32_t left_ankle_bot_target_angle_second = left_leg_sw_init_pos[SW_IDX_ANKLE_BOT] + (16384.0f / 360.0f) * 60.0f;

  //knee/hip test
  SteadyWin_AbsPosControl(&right_leg_sw_motors[SW_IDX_HIP_PITCH], hip_pitch_target_angle);
  uart_printf("hip pitch angle: %u\r\n", hip_pitch_target_angle);
  SteadyWin_AbsPosControl(&right_leg_sw_motors[SW_IDX_HIP_ROLL], hip_roll_target_angle);
  uart_printf("hip roll angle: %u\r\n", hip_roll_target_angle);
  SteadyWin_AbsPosControl(&right_leg_sw_motors[SW_IDX_HIP_YAW],  hip_yaw_target_angle);
  uart_printf("hip yaw angle: %u\r\n", hip_yaw_target_angle);
  SteadyWin_AbsPosControl(&left_leg_sw_motors[SW_IDX_HIP_PITCH], left_hip_pitch_target_angle);
  uart_printf("left hip pitch angle: %u\r\n", left_hip_pitch_target_angle);
  SteadyWin_AbsPosControl(&left_leg_sw_motors[SW_IDX_HIP_ROLL], left_hip_roll_target_angle);
  uart_printf("left hip roll angle: %u\r\n", left_hip_roll_target_angle);
  SteadyWin_AbsPosControl(&left_leg_sw_motors[SW_IDX_HIP_YAW],  left_hip_yaw_target_angle);
  uart_printf("left hip yaw angle: %u\r\n", left_hip_yaw_target_angle);
  RobStride_PosControl(&right_knee_motor, 0.2f, knee_pitch_target_angle);
  uart_printf("Knee angle: %f\r\n", knee_pitch_target_angle);
  RobStride_PosControl(&left_knee_motor, 0.2f, left_knee_pitch_target_angle);
  uart_printf("Left knee angle: %f\r\n", left_knee_pitch_target_angle);
  HAL_Delay(4000);

  //ankle test
  SteadyWin_AbsPosControl(&right_leg_sw_motors[SW_IDX_ANKLE_TOP], ankle_top_target_angle_first);
  uart_printf("ankle top angle: %u\r\n", ankle_top_target_angle_first);
  SteadyWin_AbsPosControl(&right_leg_sw_motors[SW_IDX_ANKLE_BOT], ankle_bot_target_angle_first);
  uart_printf("ankle bot angle: %u\r\n", ankle_bot_target_angle_first);
  SteadyWin_AbsPosControl(&left_leg_sw_motors[SW_IDX_ANKLE_TOP], left_ankle_top_target_angle_first);
  uart_printf("left ankle top angle: %u\r\n", left_ankle_top_target_angle_first);
  SteadyWin_AbsPosControl(&left_leg_sw_motors[SW_IDX_ANKLE_BOT], left_ankle_bot_target_angle_first);
  uart_printf("left ankle bot angle: %u\r\n", left_ankle_bot_target_angle_first);
  HAL_Delay(2000);
  SteadyWin_AbsPosControl(&right_leg_sw_motors[SW_IDX_ANKLE_TOP], ankle_top_target_angle_second);
  uart_printf("ankle top angle: %u\r\n", ankle_top_target_angle_second);
  SteadyWin_AbsPosControl(&right_leg_sw_motors[SW_IDX_ANKLE_BOT], ankle_bot_target_angle_second);
  uart_printf("ankle bot angle: %u\r\n", ankle_bot_target_angle_second);
  SteadyWin_AbsPosControl(&left_leg_sw_motors[SW_IDX_ANKLE_TOP], left_ankle_top_target_angle_second);
  uart_printf("left ankle top angle: %u\r\n", left_ankle_top_target_angle_second);
  SteadyWin_AbsPosControl(&left_leg_sw_motors[SW_IDX_ANKLE_BOT], left_ankle_bot_target_angle_second);
  uart_printf("left ankle bot angle: %u\r\n", left_ankle_bot_target_angle_second);
  HAL_Delay(4000);


  Move_to_init_pos();
}


// with DWT
void AI_RunInference(void) {
    // 1. Fill input with test data (INT8: range -128..127)
    for (int i = 0; i < AI_NETWORK_IN_1_SIZE; i++) {
      in_data[i] = 1;
    }

    // 2. Get the buffer descriptors from the runtime
    ai_buffer *ai_input = ai_network_inputs_get(network, NULL);
    ai_buffer *ai_output = ai_network_outputs_get(network, NULL);

    // 3. Point the buffers to our data arrays
    ai_input[0].data = AI_HANDLE_PTR(in_data);
    ai_output[0].data = AI_HANDLE_PTR(out_data);

    // 4. Run the MLP policy and measure inference time
    uint32_t cyc_start = DWT->CYCCNT;
    ai_network_run(network, &ai_input[0], &ai_output[0]);
    uint32_t cyc_elapsed = DWT->CYCCNT - cyc_start;

    if (cyc_elapsed < inf_min_cycles) inf_min_cycles = cyc_elapsed;
    if (cyc_elapsed > inf_max_cycles) inf_max_cycles = cyc_elapsed;
    inf_count++;
    //Results: 
    // FP32 ONNX 40_256_256_12 - 1ms
    // INT8 ONNX 40_256_256_12 - 437us
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* Enable the CPU Cache */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_FDCAN1_Init();
  // CAN1 for Left leg, PD0 - RX, PD1 - TX
  MX_FDCAN2_Init();
  // CAN2 for WIT, PB12 - RX, PB13 - TX
  MX_FDCAN3_Init();
  // CAN3 for Right leg, PD12 - RX, PD13 - TX
  /* USER CODE BEGIN 2 */
  // Enable DWT cycle counter for precise timing
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  // AI init
  ai_network_create(&network, AI_NETWORK_DATA_CONFIG);
  ai_network_params params = 
  {
      AI_NETWORK_DATA_WEIGHTS(ai_network_data_weights_get()),
      AI_NETWORK_DATA_ACTIVATIONS(activations)
  };
  ai_network_init(network, &params);

  RightLeg_RobStride_CAN_Init();   /* sets up FDCAN3 filters, starts bus, inits right RobStride motor */
  RightLeg_SteadyWin_CAN_Init();   /* inits right-leg SteadyWin motors on FDCAN3 */
  LeftLeg_RobStride_CAN_Init();    /* sets up FDCAN1 filters, starts bus, inits left RobStride motor */
  LeftLeg_SteadyWin_CAN_Init();    /* inits left-leg SteadyWin motors on FDCAN1 */
  //WIT_IMU_Init();              /* FDCAN2: WIT HWT901B-CAN IMU */
  //GL_A22_Init(&gl_a22_sensor, &huart2, GL_A22_DEFAULT_ADDRESS);
  //GL_A22_DebugScan();

  //Move_to_init_pos();
  
  /* Read and print current angles of all motors */
  PrintInitialAngles();

  /* Kick off the 4-state CAN loop */
  loop_active = 0;
  loop_cyc_start = DWT->CYCCNT;
  loop_state = LOOP_S_SW_READ;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t current_tick = HAL_GetTick();
    loop_debug_counter++;

    if ((current_tick - loop_debug_last_tick) >= 1000U)
    {
      //uart_printf("[LOOP] alive tick=%lu count=%lu\r\n",
      //            (unsigned long)current_tick,
      //            (unsigned long)loop_debug_counter);
      loop_debug_last_tick = current_tick;
      loop_debug_counter = 0;
    }

    // ---- 1-second stats and feedback printout ----
    if ((current_tick - usart_last_tick) >= 1000U)
    {
      usart_last_tick = current_tick;
      uint32_t seconds = current_tick / 1000U;
      uint32_t milliseconds = current_tick % 1000U;
      uint32_t cpu_mhz = SystemCoreClock / 1000000U;

      /* ---- Snapshot loop stats (atomic-ish read from ISR counters) ---- */
      uint32_t cnt  = loop_count;
      uint32_t cmin = loop_cyc_min;
      uint32_t cmax = loop_cyc_max;
      /* Reset for next 1-second window */
      loop_count   = 0;
      loop_cyc_min = UINT32_MAX;
      loop_cyc_max = 0;

      uint32_t loop_min_us = (cpu_mhz > 0 && cmin != UINT32_MAX) ? cmin / cpu_mhz : 0;
      uint32_t loop_max_us = (cpu_mhz > 0 && cmax > 0) ? cmax / cpu_mhz : 0;

      /* SteadyWin feedback: 0.01 RPM units, angle raw*(360/16384) deg */
      float sw_spd = right_leg_sw_motors[SW_IDX_HIP_PITCH].fb.speed_a4 * 0.01f;
      float sw_ang = right_leg_sw_motors[SW_IDX_HIP_PITCH].fb.angle_a4 * (360.0f / 16384.0f);
      /* RobStride feedback: rad/s and rad */
      float rs_spd = right_knee_motor.Pos_Info.Speed;
      float rs_ang = right_knee_motor.Pos_Info.Angle;

      /*uart_printf("%lu.%03lus | Loop: %lu Hz, %lu-%lu us | SW: %.1f RPM %.1f° | RS: %.2f rad/s %.2f rad\r\n",
        seconds, milliseconds, cnt,
        loop_min_us, loop_max_us,
        (double)sw_spd, (double)sw_ang,
        (double)rs_spd, (double)rs_ang);*/
      //uart_printf("[TICK] %lu.%03lu s\r\n", (unsigned long)seconds, (unsigned long)milliseconds);
    }

    /*
    if ((current_tick - ultrasonic_last_tick) >= 1000U)
    {
      uint16_t distance_mm = 0;
      HAL_StatusTypeDef sensor_status;
      uint32_t sensor_start_tick = HAL_GetTick();

      ultrasonic_last_tick = current_tick;
      if (gl_a22_scan_done == 0U)
      {
        uart_printf("[GL-A22] skipped poll, scan found no sensor\r\n");
        continue;
      }

      uart_printf("[GL-A22] poll start reg=0x%04X\r\n", GL_A22_REG_REALTIME_DISTANCE);
      sensor_status = GL_A22_ReadRealtimeDistanceMm(&gl_a22_sensor, &distance_mm);
      if (sensor_status == HAL_OK)
      {
        uart_printf("[GL-A22] ok dist=%u mm elapsed=%lu ms flush=%u phase=%s\r\n",
                    (unsigned int)distance_mm,
                    (unsigned long)(HAL_GetTick() - sensor_start_tick),
                    (unsigned int)gl_a22_sensor.last_flush_bytes,
                    GL_A22_PhaseName(gl_a22_sensor.last_phase));
      }
      else
      {
        uart_printf("[GL-A22] fail status=%d phase=%s elapsed=%lu ms flush=%u reg=0x%04X\r\n",
                    (int)sensor_status,
                    GL_A22_PhaseName(gl_a22_sensor.last_phase),
                    (unsigned long)gl_a22_sensor.last_duration_ms,
                    (unsigned int)gl_a22_sensor.last_flush_bytes,
                    (unsigned int)gl_a22_sensor.last_reg_addr);
        GL_A22_PrintLastRx(&gl_a22_sensor);
      }

      uart_printf("[GL-A22] poll end\r\n");
    }
      */

    if ((current_tick - led_last_tick) >= led_toggle_interval)
    {
      led_last_tick = current_tick;
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    }

    //WIT_IMU_PrintIfDue();

    //AI_RunInference();

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 44;
  RCC_OscInitStruct.PLL.PLLP = 1;
  RCC_OscInitStruct.PLL.PLLQ = 5;   /* PLL1Q = 550/5 = 110 MHz → FDCAN clock */
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  /* PLL1Q = 110 MHz → prescaler 5, (1+17+4)=22 TQ, 110/(5*22) = 1 Mbps, SP 81.8% */
  hfdcan1.Init.NominalPrescaler = 5;
  hfdcan1.Init.NominalSyncJumpWidth = 4;
  hfdcan1.Init.NominalTimeSeg1 = 17;
  hfdcan1.Init.NominalTimeSeg2 = 4;
  hfdcan1.Init.DataPrescaler = 5;
  hfdcan1.Init.DataSyncJumpWidth = 4;
  hfdcan1.Init.DataTimeSeg1 = 17;
  hfdcan1.Init.DataTimeSeg2 = 4;
  /* FDCAN3 uses offset 0, FDCAN2 uses 68, so FDCAN1 starts after both */
  hfdcan1.Init.MessageRAMOffset = 136;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 1;
  hfdcan1.Init.RxFifo0ElmtsNbr = 8;
  hfdcan1.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxFifo1ElmtsNbr = 0;
  hfdcan1.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.RxBuffersNbr = 0;
  hfdcan1.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan1.Init.TxEventsNbr = 0;
  hfdcan1.Init.TxBuffersNbr = 0;
  hfdcan1.Init.TxFifoQueueElmtsNbr = 8;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan1.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief FDCAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN2_Init(void)
{

  /* USER CODE BEGIN FDCAN2_Init 0 */

  /* USER CODE END FDCAN2_Init 0 */

  /* USER CODE BEGIN FDCAN2_Init 1 */

  /* USER CODE END FDCAN2_Init 1 */
  hfdcan2.Instance = FDCAN2;
  hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan2.Init.AutoRetransmission = DISABLE;
  hfdcan2.Init.TransmitPause = DISABLE;
  hfdcan2.Init.ProtocolException = DISABLE;
  /* PLL1Q = 110 MHz → prescaler 5, (1+17+4)=22 TQ, 110/(5*22) = 1 Mbps, SP 81.8% */
  hfdcan2.Init.NominalPrescaler = 5;
  hfdcan2.Init.NominalSyncJumpWidth = 4;
  hfdcan2.Init.NominalTimeSeg1 = 17;
  hfdcan2.Init.NominalTimeSeg2 = 4;
  hfdcan2.Init.DataPrescaler = 5;
  hfdcan2.Init.DataSyncJumpWidth = 4;
  hfdcan2.Init.DataTimeSeg1 = 17;
  hfdcan2.Init.DataTimeSeg2 = 4;
  /* FDCAN3 uses offset 0 and occupies 68 words; FDCAN2 starts after */
  hfdcan2.Init.MessageRAMOffset = 68;
  hfdcan2.Init.StdFiltersNbr = 1;
  hfdcan2.Init.ExtFiltersNbr = 1;
  hfdcan2.Init.RxFifo0ElmtsNbr = 8;
  hfdcan2.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxFifo1ElmtsNbr = 0;
  hfdcan2.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.RxBuffersNbr = 0;
  hfdcan2.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan2.Init.TxEventsNbr = 0;
  hfdcan2.Init.TxBuffersNbr = 0;
  hfdcan2.Init.TxFifoQueueElmtsNbr = 8;
  hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan2.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN2_Init 2 */

  /* USER CODE END FDCAN2_Init 2 */

}

/**
  * @brief FDCAN3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN3_Init(void)
{

  /* USER CODE BEGIN FDCAN3_Init 0 */

  /* USER CODE END FDCAN3_Init 0 */

  /* USER CODE BEGIN FDCAN3_Init 1 */

  /* USER CODE END FDCAN3_Init 1 */
  hfdcan3.Instance = FDCAN3;
  hfdcan3.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan3.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan3.Init.AutoRetransmission = DISABLE;
  hfdcan3.Init.TransmitPause = DISABLE;
  hfdcan3.Init.ProtocolException = DISABLE;
  /* PLL1Q = 110 MHz → prescaler 5, (1+17+4)=22 TQ, 110/(5*22) = 1 Mbps, SP 81.8% */
  hfdcan3.Init.NominalPrescaler = 5;
  hfdcan3.Init.NominalSyncJumpWidth = 4;
  hfdcan3.Init.NominalTimeSeg1 = 17;
  hfdcan3.Init.NominalTimeSeg2 = 4;
  hfdcan3.Init.DataPrescaler = 5;
  hfdcan3.Init.DataSyncJumpWidth = 4;
  hfdcan3.Init.DataTimeSeg1 = 17;
  hfdcan3.Init.DataTimeSeg2 = 4;
  hfdcan3.Init.MessageRAMOffset = 0;
  hfdcan3.Init.StdFiltersNbr = 1;
  hfdcan3.Init.ExtFiltersNbr = 1;
  hfdcan3.Init.RxFifo0ElmtsNbr = 8;
  hfdcan3.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan3.Init.RxFifo1ElmtsNbr = 0;
  hfdcan3.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
  hfdcan3.Init.RxBuffersNbr = 0;
  hfdcan3.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
  hfdcan3.Init.TxEventsNbr = 0;
  hfdcan3.Init.TxBuffersNbr = 0;
  hfdcan3.Init.TxFifoQueueElmtsNbr = 8;
  hfdcan3.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  hfdcan3.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
  if (HAL_FDCAN_Init(&hfdcan3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN3_Init 2 */

  /* USER CODE END FDCAN3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 230400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : PF6 PF7 PF8 PF9 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OCTOSPIM_P1;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PF10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_OCTOSPIM_P1;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : PG6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_OCTOSPIM_P1;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
