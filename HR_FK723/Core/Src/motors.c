#include "motors.h"

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

// ---- Loop state ----
extern uint8_t  loop_active = 0;
extern uint8_t  loop_state  = LOOP_S_SW_READ;
extern uint32_t loop_count  = 0;
extern uint32_t loop_cyc_start = 0;
extern uint32_t loop_cyc_min = UINT32_MAX;
extern uint32_t loop_cyc_max = 0;

static float init_pos_tolerance_deg = 2.0f;
static float init_pos_tolerance_rad = 2.0f * (3.14159265f / 180.0f);

/* RobStride knee motor IDs */
extern uint8_t right_knee_motor_addr = 0x7F;
extern uint8_t left_knee_motor_addr = 0x7F; //not 0x3E

RobStride_Motor right_knee_motor;
RobStride_Motor left_knee_motor;

SteadyWin_Motor right_leg_sw_motors[SW_MOTOR_COUNT];
SteadyWin_Motor left_leg_sw_motors[SW_MOTOR_COUNT];

// SteadyWin motor instances (0=hip_pitch, 1=hip_roll, 2=hip_yaw, 3=ankle_top, 4=ankle_bot) 
static uint8_t right_leg_sw_motor_addrs[SW_MOTOR_COUNT] = {0x02, 0x03, 0x04, 0x05, 0x06};
static uint8_t left_leg_sw_motor_addrs[SW_MOTOR_COUNT] = {0x0A, 0x0B, 0x0C, 0x0D, 0x0E};
static char * leg_sw_joint_names[SW_MOTOR_COUNT] = {
  "hip_pitch", "hip_roll", "hip_yaw", "ankle_top", "ankle_bot"};



static uint16_t init_sw_max_speed[SW_MOTOR_COUNT] = {30, 10, 20, 60, 60};
static uint8_t init_sw_accel[SW_MOTOR_COUNT] = {3, 3, 3, 3, 3};
static uint16_t normal_sw_max_speed[SW_MOTOR_COUNT] = {300, 100, 200, 600, 600};
static uint8_t normal_sw_accel[SW_MOTOR_COUNT] = {30, 30, 30, 30, 30};

volatile uint8_t  right_leg_can_rx_data[8];
volatile uint8_t  right_leg_can_rx_new;
volatile uint32_t right_leg_can_rx_id;
volatile uint8_t  right_leg_can_rx_dlc;

volatile uint8_t  left_leg_can_rx_data[8];
volatile uint8_t  left_leg_can_rx_new;
volatile uint32_t left_leg_can_rx_id;
volatile uint8_t  left_leg_can_rx_dlc;

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

/* ---- Loop over both legs: index 0 = right (FDCAN3), 1 = left (FDCAN1) ---- */
SteadyWin_Motor *const sw_legs[2] = { right_leg_sw_motors, left_leg_sw_motors };
RobStride_Motor *const rs_legs[2] = { &right_knee_motor, &left_knee_motor };

#define LOOP_SW_MASK  ((1U << (2U * SW_MOTOR_COUNT)) - 1U)   /* one bit per SW motor, both legs */
#define LOOP_RS_MASK  0x3U                                   /* one bit per knee */
volatile uint32_t loop_rx_mask = 0;

volatile uint32_t loop_last_tick = 0;          /* tick of the last state change */
volatile uint32_t loop_timeouts = 0;           /* cumulative, counted in main */
volatile uint32_t loop_timeout_missing = 0;    /* reply mask that was missing at the last timeout */
volatile uint8_t  loop_timeout_state = 0;
volatile uint32_t loop_dead_sw = 0;            /* SW motors not waited for (timed out); cleared when they reply */
volatile uint32_t loop_dead_rs = 0;            /* same for the knees */

// Called from FDCAN RX FIFO0 interrupt for FDCAN2 (WIT IMU), FDCAN1 (left leg), and FDCAN3 (right leg)
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE)
  {
    FDCAN_RxHeaderTypeDef rxHdr;
    uint8_t rxData[8];
    // Several motors reply back-to-back, so drain the whole FIFO per interrupt
    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0 &&
           HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHdr, rxData) == HAL_OK)
    {
      //FDCAN2: WIT IMU sensor
      if (hfdcan->Instance == FDCAN2)
      {
        /*
        uint8_t dlc = FDCAN_DLCToBytes(rxHdr.DataLength);

        // Capture raw frame for diagnostics
        wit_rx_total++;
        wit_last_id     = rxHdr.Identifier;
        wit_last_id_ext = (rxHdr.IdType == FDCAN_EXTENDED_ID) ? 1u : 0u;
        wit_last_dlc_code = (uint8_t)rxHdr.DataLength;
        wit_last_dlc    = dlc;
        for (int i = 0; i < 8; i++) wit_last_frame[i] = rxData[i];

        WitCanDataIn(rxData, dlc); */
        return;
      }

      int leg = (hfdcan->Instance == FDCAN1) ? 1 : 0;
      volatile uint8_t *leg_can_rx_data = (leg == 1) ? left_leg_can_rx_data : right_leg_can_rx_data;
      volatile uint32_t *leg_can_rx_id = (leg == 1) ? &left_leg_can_rx_id : &right_leg_can_rx_id;
      volatile uint8_t *leg_can_rx_dlc = (leg == 1) ? &left_leg_can_rx_dlc : &right_leg_can_rx_dlc;
      volatile uint8_t *leg_can_rx_new = (leg == 1) ? &left_leg_can_rx_new : &right_leg_can_rx_new;
      SteadyWin_Motor *leg_sw_motors = sw_legs[leg];
      RobStride_Motor *leg_rs_motor = rs_legs[leg];

      CaptureMotorBusRxFrame(leg_can_rx_data, leg_can_rx_id,
                             leg_can_rx_dlc, leg_can_rx_new,
                             &rxHdr, rxData);

      if (rxHdr.IdType == FDCAN_EXTENDED_ID)
      {
        RobStride_Analysis(leg_rs_motor, rxData, rxHdr.Identifier);
      }
      else
      {
        for (int i = 0; i < SW_MOTOR_COUNT; i++)
          SteadyWin_Analysis(&leg_sw_motors[i], rxData, *leg_can_rx_dlc, rxHdr.Identifier);
      }

      if (!loop_active) continue;

      // 5-state loop: SW read, RS read, AI in main, SW write, RS write
      if (loop_state == LOOP_S_SW_READ || loop_state == LOOP_S_SW_WRITE)
      {
        // SteadyWin reply: standard ID, bare dev_addr or 0x100|dev_addr
        uint8_t expect = (loop_state == LOOP_S_SW_READ) ? SW_CMD_READ_MULTI : SW_CMD_SPEED_CTRL;
        if (rxHdr.IdType == FDCAN_STANDARD_ID && rxData[0] == expect)
        {
          for (int i = 0; i < SW_MOTOR_COUNT; i++)
          {
            uint32_t addr = leg_sw_motors[i].dev_addr;
            if (rxHdr.Identifier == addr || rxHdr.Identifier == (0x100U | addr))
            {
              loop_rx_mask |= 1U << (leg * SW_MOTOR_COUNT + i);
              loop_dead_sw &= ~(1U << (leg * SW_MOTOR_COUNT + i));
            }
          }
        }
      }
      else if ((loop_state == LOOP_S_RS_READ || loop_state == LOOP_S_RS_WRITE) &&
               rxHdr.IdType == FDCAN_EXTENDED_ID &&
               ((rxHdr.Identifier >> 8) & 0xFF) == leg_rs_motor->CAN_ID)
      {
        // RobStride reply: extended ID with the knee CAN_ID as source
        loop_rx_mask |= 1U << leg;
        loop_dead_rs &= ~(1U << leg);
      }

      // S0→S1 / S1→S2 / S2→S3 / S3→S0: advance once every motor of the state has replied
      switch (loop_state)
      {
        case LOOP_S_SW_READ:
          if ((loop_rx_mask | loop_dead_sw) == LOOP_SW_MASK)
          {
            // all SW read, now read both knees
            loop_rx_mask = 0;
            loop_last_tick = HAL_GetTick();
            loop_state = LOOP_S_RS_READ;
            for (int l = 0; l < 2; l++)
              RobStride_MotorRequest(rs_legs[l]);
          }
          break;

        case LOOP_S_RS_READ:
          if ((loop_rx_mask | loop_dead_rs) == LOOP_RS_MASK)
          {
            // all reads complete, let main run inference before issuing writes
            loop_rx_mask = 0;
            loop_last_tick = HAL_GetTick();
            loop_state = LOOP_S_AI;
          }
          break;

        case LOOP_S_SW_WRITE:
          if ((loop_rx_mask | loop_dead_sw) == LOOP_SW_MASK)
          {
            // all SW write acks received, now write both knees
            loop_rx_mask = 0;
            loop_last_tick = HAL_GetTick();
            loop_state = LOOP_S_RS_WRITE;
            for (int l = 0; l < 2; l++)
              RobStride_SetParameter(rs_legs[l], 0x700A, 0.0f, Set_parameter);
          }
          break;

        case LOOP_S_RS_WRITE:
          if ((loop_rx_mask | loop_dead_rs) == LOOP_RS_MASK)
          {
            // loop complete, measure and restart at SW read
            loop_rx_mask = 0;
            loop_last_tick = HAL_GetTick();
            {
              uint32_t now = DWT->CYCCNT;
              uint32_t elapsed = now - loop_cyc_start;
              if (elapsed < loop_cyc_min) loop_cyc_min = elapsed;
              if (elapsed > loop_cyc_max) loop_cyc_max = elapsed;
              loop_count++;
              loop_cyc_start = now;
              loop_state = LOOP_S_SW_READ;
              for (int l = 0; l < 2; l++)
                for (int i = 0; i < SW_MOTOR_COUNT; i++)
                  SteadyWin_ReadMulti(&sw_legs[l][i]);
            }
          }
          break;

        case LOOP_S_AI:
        default:
          break;
      }
    }
  }
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

float SteadyWinCountsToDegrees(uint16_t counts)
{
  return (float)counts * 0.02197265625f; // 360 / 16384
}

float RadiansToDegrees(float angle_rad)
{
  return angle_rad * 57.2957795056; // 180 / pi
}

float Return_current_SteadyWin_Speed(SteadyWin_Motor motor)
{
  return motor.fb.speed_a4;
}

float Return_current_SteadyWin_Angle(SteadyWin_Motor motor)
{
  return SteadyWinCountsToDegrees(motor.fb.angle_a4);
}

float Return_current_SteadyWin_current(SteadyWin_Motor motor)
{
  return motor.fb.current_a4;
}

float Return_current_SteadyWin_temp(SteadyWin_Motor motor)
{
  return motor.fb.temp_a4;
}

float Return_current_RobStride_Angle(RobStride_Motor motor)
{
  return motor.Pos_Info.Angle;
}

float Return_current_RobStride_Speed(RobStride_Motor motor)
{
  return motor.Pos_Info.Speed;
}

float Return_current_RobStride_torque(RobStride_Motor motor)
{
  return motor.Pos_Info.Torque;
}

float Return_current_RobStride_temp(RobStride_Motor motor)
{
  return motor.Pos_Info.Temp;
}

uint8_t FDCAN_DLCToBytes(uint32_t dlc_code)
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

static int32_t SteadyWinShortestDeltaCounts(uint16_t current_counts, uint16_t ideal_counts)
{
  int32_t delta_counts = (int32_t)current_counts - (int32_t)ideal_counts;

  if (delta_counts > 8192)
    delta_counts -= 16384;
  else if (delta_counts < -8192)
    delta_counts += 16384;

  return delta_counts;
}

static uint8_t SteadyWinWithinInitTolerance(uint16_t current_counts, uint16_t ideal_counts)
{
  int32_t delta_counts = SteadyWinShortestDeltaCounts(current_counts, ideal_counts);
  float delta_deg = fabsf((float)delta_counts) * (360.0f / 16384.0f);
  return (delta_deg <= init_pos_tolerance_deg) ? 1U : 0U;
}

static uint8_t RobStrideWithinInitTolerance(float current_angle, float ideal_angle)
{
  return (fabsf(current_angle - ideal_angle) <= init_pos_tolerance_rad) ? 1U : 0U;
}

void CaptureMotorBusRxFrame(volatile uint8_t *bus_rx_data, volatile uint32_t *bus_rx_id, volatile uint8_t *bus_rx_dlc, 
  volatile uint8_t *bus_rx_new, const FDCAN_RxHeaderTypeDef *rxHdr, const uint8_t *rxData)
{
  for (int index = 0; index < 8; ++index)
    bus_rx_data[index] = rxData[index];

  *bus_rx_id = rxHdr->Identifier;
  *bus_rx_dlc = FDCAN_DLCToBytes(rxHdr->DataLength);
  *bus_rx_new = 1;
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
      //Read status
      SteadyWin_ReadStatus(&right_leg_sw_motors[i]);
      HAL_Delay(20);
      /*right_leg_can_rx_new = 0;
      uint16_t boot_ver = (uint16_t)right_leg_can_rx_data[1] | ((uint16_t)right_leg_can_rx_data[2] << 8);
      uint16_t app_ver  = (uint16_t)right_leg_can_rx_data[3] | ((uint16_t)right_leg_can_rx_data[4] << 8);
      uint16_t hw_ver   = (uint16_t)right_leg_can_rx_data[5] | ((uint16_t)right_leg_can_rx_data[6] << 8);
      uint8_t  can_ver  = right_leg_can_rx_data[7];
      uart_printf("[R-SW 0x%02X %-9s] OK  (Boot=%u App=%u HW=%u CAN=%u)\r\n",
        right_leg_sw_motors[i].dev_addr, leg_sw_joint_names[i],
        boot_ver, app_ver, hw_ver, can_ver);*/
    //Print status
    uart_printf("[R-SW 0x%02X %-9s] OK Status: BusV=%u BusI=%u Temp=%u Mode=%u Fault=%u\r\n",
      right_leg_sw_motors[i].dev_addr, leg_sw_joint_names[i],
      right_leg_sw_motors[i].fb.bus_voltage_raw/100,
      right_leg_sw_motors[i].fb.bus_current_raw/100,
      right_leg_sw_motors[i].fb.temperature,
      right_leg_sw_motors[i].fb.run_mode,
      right_leg_sw_motors[i].fb.fault_code);
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
      //Read status
      SteadyWin_ReadStatus(&left_leg_sw_motors[i]);
      HAL_Delay(20);
      left_leg_can_rx_new = 0;
      /*uint16_t boot_ver = (uint16_t)left_leg_can_rx_data[1] | ((uint16_t)left_leg_can_rx_data[2] << 8);
      uint16_t app_ver  = (uint16_t)left_leg_can_rx_data[3] | ((uint16_t)left_leg_can_rx_data[4] << 8);
      uint16_t hw_ver   = (uint16_t)left_leg_can_rx_data[5] | ((uint16_t)left_leg_can_rx_data[6] << 8);
      uint8_t  can_ver  = left_leg_can_rx_data[7];
      uart_printf("[L-SW 0x%02X %-9s] OK  (Boot=%u App=%u HW=%u CAN=%u)\r\n",
        left_leg_sw_motors[i].dev_addr, leg_sw_joint_names[i],
        boot_ver, app_ver, hw_ver, can_ver);*/
      //Print status
      uart_printf("[L-SW 0x%02X %-9s] OK Status: BusV=%u BusI=%u Temp=%u Mode=%u Fault=%u\r\n",
        left_leg_sw_motors[i].dev_addr, leg_sw_joint_names[i],
        left_leg_sw_motors[i].fb.bus_voltage_raw/100,
        left_leg_sw_motors[i].fb.bus_current_raw/100,
        left_leg_sw_motors[i].fb.temperature,
        left_leg_sw_motors[i].fb.run_mode,
        left_leg_sw_motors[i].fb.fault_code);
    } else {
      uart_printf("[L-SW 0x%02X %-9s] NO RESPONSE\r\n",
        left_leg_sw_motors[i].dev_addr, leg_sw_joint_names[i]);
    }





    SteadyWin_ClearFault(&left_leg_sw_motors[i]);
    HAL_Delay(20);
  }

  uart_printf("============================\r\n\r\n");
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

// Initial small acceleration and speed for smooth movement to init position
void Motors_speed_accel_init_settings(void) {
  /* Move RobStride to init position */
  RobStride_Disable(&right_knee_motor, 0);
  HAL_Delay(50);
  right_knee_motor.drw.run_mode.data = 0;
  right_knee_motor.Motor_Set_All.set_limit_speed = 0.25f;
  right_knee_motor.Motor_Set_All.set_acceleration = 0.5f;
  RobStride_Disable(&left_knee_motor, 0);
  HAL_Delay(50);
  left_knee_motor.drw.run_mode.data = 0;
  left_knee_motor.Motor_Set_All.set_limit_speed = 0.25f;
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