#include "main.h"

#include "rcc.h"
#include "mpu.h"
#include "gpio.h"
#include "uart.h"
#include "can.h"
#include "nn.h"
#include "motors.h"
//#include "MCL.h"
//#include "sensors.h"
//#include "GL_A22.h"
#include "Robstride.h"
#include "SteadyWin.h"
//#include "wit_c_sdk.h"

extern RobStride_Motor right_knee_motor;
extern RobStride_Motor left_knee_motor;

extern SteadyWin_Motor right_leg_sw_motors[SW_MOTOR_COUNT];
extern SteadyWin_Motor left_leg_sw_motors[SW_MOTOR_COUNT];

extern SteadyWin_Motor *const sw_legs[2];   /* 0 = right, 1 = left */
extern RobStride_Motor *const rs_legs[2];

extern uint8_t  loop_active;
extern uint8_t  loop_state;
extern uint32_t loop_count;
extern uint32_t loop_cyc_start;
extern uint32_t loop_cyc_min;
extern uint32_t loop_cyc_max;
extern volatile uint32_t loop_timeouts;
extern volatile uint32_t loop_timeout_missing;
extern volatile uint8_t  loop_timeout_state;
extern volatile uint32_t loop_last_tick;
extern volatile uint32_t loop_rx_mask;
extern volatile uint32_t loop_dead_sw;
extern volatile uint32_t loop_dead_rs;

/* USER CODE BEGIN PV */
uint32_t usart_last_tick = 0;
uint32_t led_last_tick = 0;

uint32_t led_toggle_interval = 500U;
uint32_t loop_debug_last_tick = 0;
uint32_t loop_debug_counter = 0;


/* ---- CAN motor loop state machine ---- */
/*
 * 5-stage loop across ISR and main:
 *   S0: TX SW ReadMulti(0xA4) to 5+5 motors   → RX temp+current+speed+angle
 *   S1: TX RS MotorRequest to both knees      → RX angle+speed+torque+temp
 *   S2: Run AI inference in main using the latest feedback snapshot
 *   S3: TX SW SpeedCtrl(0xC1, 0) to 5+5       → RX speed feedback
 *   S4: TX RS SetSpeed(0x700A, 0) to both     → RX ack
 *   → measure loop time, restart at S0
 */

/*
Standart 5 steps Loop without AI - 400Hz
With AI 1ms - 250Hz
*/

int main(void) //CHECK ALL FUNCTIONS HERE!!!!
{
  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();// Configure the Memory Protection Unit (MPU)

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();// Enable the instruction cache

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();// Enable the data cache

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();// Initialize the Hardware Abstraction Layer (HAL)

  /* Configure the system clock */
  SystemClock_Config();// Configure the system clock

  /* Initialize all configured peripherals */
  MX_GPIO_Init();// Initialize GPIO

  MX_USART1_UART_Init();// Initialize USART1 for UART communication
  MX_USART2_UART_Init();// Initialize USART2 for UART communication

  // CAN1 for Left leg, PD0 - RX, PD1 - TX
  MX_FDCAN1_Init();//
  // CAN2 for WIT, PB12 - RX, PB13 - TX
  //MX_FDCAN2_Init();//
  // CAN3 for Right leg, PD12 - RX, PD13 - TX
  MX_FDCAN3_Init();//

  // Enable DWT cycle counter for precise timing
  DWT_ENABLE();

  // AI init
  AI_init();// Initialize AI module

  HAL_Delay(2000);

  RightLeg_RobStride_CAN_Init();   /* sets up FDCAN3 filters, starts bus, inits right RobStride motor */
  RightLeg_SteadyWin_CAN_Init();   /* inits right-leg SteadyWin motors on FDCAN3 */
  LeftLeg_RobStride_CAN_Init();    /* sets up FDCAN1 filters, starts bus, inits left RobStride motor */
  LeftLeg_SteadyWin_CAN_Init();    /* inits left-leg SteadyWin motors on FDCAN1 */

  //WIT_UART_Init();               // already in MX_USART2_UART_Init?

  //WIT_IMU_Init();              /* FDCAN2: WIT HWT901B-CAN IMU */
  //GL_A22_Init(&gl_a22_sensor, &huart2, GL_A22_DEFAULT_ADDRESS);
  //GL_A22_DebugScan();
  
  /* Read and print current angles of all motors */
  //PrintInitialAngles();

  //Move_to_init_pos();

  // Run a simple movement program to test motors
  //Simple_movement_program();

  uint32_t cpu_mhz = SystemCoreClock / 1000000U;

  /* The loop only advances on CAN replies, so the first requests are sent here. */
  loop_active = 1;
  loop_cyc_start = DWT->CYCCNT;
  loop_state = LOOP_S_SW_READ;
  loop_last_tick = HAL_GetTick();
  for (int leg = 0; leg < 2; leg++)
    for (int i = 0; i < SW_MOTOR_COUNT; i++)
      SteadyWin_ReadMulti(&sw_legs[leg][i]);

  const char *leg_tag[2] = { "R", "L" };
  const char *sw_names[SW_MOTOR_COUNT] = { "hip_pitch", "hip_roll", "hip_yaw", "ankle_top", "ankle_bot" };

  while (1)
  {
    uint32_t current_tick = HAL_GetTick();
    loop_debug_counter++;

    // ---- Loop timeout: a motor stayed silent, record who and restart from S0 ----
    if (loop_active && loop_state != LOOP_S_AI && (current_tick - loop_last_tick) > LOOP_TIMEOUT_MS)
    {
      __disable_irq();
      if (loop_state != LOOP_S_AI && (HAL_GetTick() - loop_last_tick) > LOOP_TIMEOUT_MS)   // re-check: ISR may have advanced meanwhile
      {
        loop_timeouts++;
        loop_timeout_state = loop_state;
        if (loop_state == LOOP_S_SW_READ || loop_state == LOOP_S_SW_WRITE)
        {
          loop_timeout_missing = 0x3FFU & ~(loop_rx_mask | loop_dead_sw);
          loop_dead_sw |= loop_timeout_missing;   // stop waiting for silent motors
        }
        else if (loop_state == LOOP_S_RS_READ || loop_state == LOOP_S_RS_WRITE)
        {
          loop_timeout_missing = 0x3U & ~(loop_rx_mask | loop_dead_rs);
          loop_dead_rs |= loop_timeout_missing;
        }
        else
        {
          loop_timeout_missing = 0;
        }
        loop_rx_mask = 0;
        loop_state = LOOP_S_SW_READ;
        loop_last_tick = HAL_GetTick();
        for (int leg = 0; leg < 2; leg++)
          for (int i = 0; i < SW_MOTOR_COUNT; i++)
            SteadyWin_ReadMulti(&sw_legs[leg][i]);
      }
      __enable_irq();
    }

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
      

      /* ---- Snapshot loop stats (atomic-ish read from ISR counters) ---- */
      uint32_t cnt  = loop_count;
      uint32_t cmin = loop_cyc_min;
      uint32_t cmax = loop_cyc_max;

      uint32_t loop_min_us = (cpu_mhz > 0 && cmin != UINT32_MAX) ? cmin / cpu_mhz : 0;
      uint32_t loop_max_us = (cpu_mhz > 0 && cmax > 0) ? cmax / cpu_mhz : 0;

      uart_printf("%lu.%03lus | Loop: %lu Hz, %.2f-%.2f ms | state S%u | timeouts=%lu (last: S%u, missing 0x%03lX) | dead SW=0x%03lX RS=0x%lX | speed 0 written to all\r\n",
        seconds, milliseconds, cnt, (double)loop_min_us / 1000.0, (double)loop_max_us / 1000.0, loop_state,
        (unsigned long)loop_timeouts, loop_timeout_state, (unsigned long)loop_timeout_missing,
        (unsigned long)loop_dead_sw, (unsigned long)loop_dead_rs);

      for (int leg = 0; leg < 2; leg++)
      {
        /* SteadyWin feedback: 0.01 RPM units, angle raw*(360/16384) deg */
        for (int i = 0; i < SW_MOTOR_COUNT; i++)
        {
          float sw_spd  = Return_current_SteadyWin_Speed(sw_legs[leg][i])/100;
          float sw_ang  = Return_current_SteadyWin_Angle(sw_legs[leg][i]);
          float sw_cur  = Return_current_SteadyWin_current(sw_legs[leg][i])/1000;
          float sw_temp = Return_current_SteadyWin_temp(sw_legs[leg][i]);
          uart_printf("  %s %-9s: %.1f RPM %.1f° %.2fA %.1f°C\r\n",
            leg_tag[leg], sw_names[i],
            (double)sw_spd, (double)sw_ang, (double)sw_cur, (double)sw_temp);
        }

        /* RobStride feedback: rad/s and rad */
        float rs_spd  = Return_current_RobStride_Speed(*rs_legs[leg]);
        float rs_ang  = Return_current_RobStride_Angle(*rs_legs[leg]);
        float rs_trq  = Return_current_RobStride_torque(*rs_legs[leg]);
        float rs_temp = Return_current_RobStride_temp(*rs_legs[leg]);
        uart_printf("  %s %-9s: %.2f rad/s %.2f rad %.2f Nm %.1f°C\r\n",
          leg_tag[leg], "knee", (double)rs_spd, (double)rs_ang, (double)rs_trq, (double)rs_temp);
      }
      //uart_printf("[TICK] %lu.%03lu s\r\n", (unsigned long)seconds, (unsigned long)milliseconds);

      /* Reset for next 1-second window */
      loop_count   = 0;
      loop_cyc_min = UINT32_MAX;
      loop_cyc_max = 0;
    }

    if ((current_tick - led_last_tick) >= led_toggle_interval)
    {
      led_last_tick = current_tick;
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    }

    //WIT_UART_AccelerationTask();
    //WIT_IMU_PrintIfDue();

    if (loop_active && loop_state == LOOP_S_AI)
    {
      HAL_Delay(1);  // simulate AI inference time of 1 ms
      loop_rx_mask = 0;
      loop_last_tick = HAL_GetTick();
      loop_state = LOOP_S_SW_WRITE;
      for (int leg = 0; leg < 2; leg++)
        for (int i = 0; i < SW_MOTOR_COUNT; i++)
          SteadyWin_SpeedControl(&sw_legs[leg][i], 0);
    }

  }
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
