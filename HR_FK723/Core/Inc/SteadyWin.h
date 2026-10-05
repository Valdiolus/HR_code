#ifndef __STEADYWIN_H__
#define __STEADYWIN_H__

#include "main.h"
#include "stm32h723xx.h"
#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_fdcan.h"
#include <string.h>

/*
READ PARAMETERS (8 total)
These are parameters you can query from the motor:
Code	Parameter	Returns
0xA0	Version Info	Boot, Software, Hardware, Protocol versions
0xA1	Current	Q-axis current (0.001A units)
0xA2	Speed	Rotation speed (0.01 RPM units)
0xA3	Angles	Single-turn & multi-turn absolute angles
0xA4	Multi-Read	Temperature, Current, Speed, Angle (combined)
0xAE	Status	Bus voltage, current, temp, mode, fault code ⭐ MOST COMPLETE
0xB0	Motor Info	Pole pairs, torque constant, gear ratio
0xF1	MIT Mode	Position, speed, torque (MIT protocol mode)
0xB6-0xB9	PID Gains	Position/Velocity loop Kp & Ki gains (read/write)
*/

/*
WRITE PARAMETERS (13 total control/config)
Configuration Parameters (Survive Reboot: 0xB1, 0xF0 only):
Code	Parameter	Format
0xB1	Set Origin	Mark current position as home
0xB2	Max Speed	uint32_t (0.01 RPM) - Position mode limit
0xB3	Max Current	uint32_t (0.001A) - Current limit
0xB4	Current Slope	uint32_t (0.001A/s) - Ramp rate
0xB5	Acceleration	uint32_t (0.01 RPM/s) - Speed ramp
0xB6-0xB9	PID Gains	uint32_t - Proportional/Integral gains
0xF0	MIT Config	Pos_Max, Vel_Max, T_Max limits
*/

/*
Control Commands (Real-time, lost on power cycle):
Code	Parameter	Format
0x00	Reboot	Restart controller
0xC0	Current	int32_t (0.001A) - Torque command
0xC1	Speed	int32_t (0.01 RPM) - Speed setpoint
0xC2	Abs Position	int32_t - Move to absolute position
0xC3	Rel Position	int32_t - Relative movement
0xC4	Go to Origin	Return home (≤180°)
0xCE	Brake	Holding brake control
0xCF	Disable	Motor free-spin mode
*/

/* ---- Command codes ---- */
#define SW_CMD_REBOOT           0x00
#define SW_CMD_READ_VERSION     0xA0
#define SW_CMD_READ_CURRENT     0xA1
#define SW_CMD_READ_SPEED       0xA2
#define SW_CMD_READ_ANGLE       0xA3
#define SW_CMD_READ_MULTI       0xA4
#define SW_CMD_READ_STATUS      0xAE
#define SW_CMD_CLEAR_FAULT      0xAF

#define SW_CMD_READ_MOTOR_INFO  0xB0
#define SW_CMD_SET_ZERO         0xB1
#define SW_CMD_SET_MAX_SPEED    0xB2
#define SW_CMD_SET_MAX_CURRENT  0xB3
#define SW_CMD_SET_CURRENT_SLOPE 0xB4
#define SW_CMD_SET_ACCEL        0xB5
#define SW_CMD_POS_KP           0xB6
#define SW_CMD_POS_KI           0xB7
#define SW_CMD_VEL_KP           0xB8
#define SW_CMD_VEL_KI           0xB9

#define SW_CMD_CURRENT_CTRL     0xC0
#define SW_CMD_SPEED_CTRL       0xC1
#define SW_CMD_ABS_POS_CTRL     0xC2
#define SW_CMD_REL_POS_CTRL     0xC3
#define SW_CMD_GOTO_ORIGIN      0xC4
#define SW_CMD_BRAKE_CTRL       0xCE
#define SW_CMD_DISABLE          0xCF

#define SW_CMD_MIT_CONFIG       0xF0
#define SW_CMD_MIT_READ         0xF1

/* Run modes reported in 0xAE byte[6] */
#define SW_MODE_OFF             0
#define SW_MODE_VOLTAGE         1
#define SW_MODE_CURRENT         2
#define SW_MODE_SPEED           3
#define SW_MODE_POSITION        4

/* Motor feedback */
typedef struct {
    /* From 0xAE */
    uint16_t bus_voltage_raw;   /* 0.01V units */
    uint16_t bus_current_raw;   /* 0.01A units */
    uint8_t  temperature;       /* degC */
    uint8_t  run_mode;
    uint8_t  fault_code;

    /* From 0xA1 / 0xC0 */
    int32_t  q_current_raw;     /* 0.001A units */

    /* From 0xA2 / 0xC1 */
    int32_t  speed_raw;         /* 0.01 RPM units */

    /* From 0xA3 / 0xC2 / 0xC3 */
    uint16_t single_angle_raw;  /* Angle = value*(360/16384) */
    int32_t  multi_angle_raw;   /* Total Angle = value*(360/16384) */

    /* From 0xA4 */
    uint8_t  temp_a4;
    int16_t  current_a4;        /* 0.001A */
    int16_t  speed_a4;          /* 0.01 RPM */
    uint16_t angle_a4;          /* value*(360/16384) */
} SteadyWin_Feedback;

/* Motor instance */
typedef struct {
    uint8_t  dev_addr;          /* Device address (1-254) */
    uint8_t  gear_ratio;        /* From 0xB0, for output shaft calc */
    FDCAN_HandleTypeDef *hfdcan;
    SteadyWin_Feedback fb;
} SteadyWin_Motor;

/* Encoder helper: degrees from raw feedback values */
float SteadyWin_SingleAngleDeg(const SteadyWin_Motor *m);
float SteadyWin_MultiAngleDeg(const SteadyWin_Motor *m);
float SteadyWin_OutputShaftDeg(const SteadyWin_Motor *m);

/* ---- API ---- */
void SteadyWin_Init(SteadyWin_Motor *m, uint8_t dev_addr,
                    FDCAN_HandleTypeDef *hfdcan);

/* RX analysis: call from CAN RX callback */
void SteadyWin_Analysis(SteadyWin_Motor *m, uint8_t *data, uint8_t dlc,
                        uint32_t std_id);

/* System commands */
void SteadyWin_Reboot(SteadyWin_Motor *m);
void SteadyWin_ReadVersion(SteadyWin_Motor *m);
void SteadyWin_ReadStatus(SteadyWin_Motor *m);
void SteadyWin_ClearFault(SteadyWin_Motor *m);
void SteadyWin_Disable(SteadyWin_Motor *m);

/* Read commands */
void SteadyWin_ReadCurrent(SteadyWin_Motor *m);
void SteadyWin_ReadSpeed(SteadyWin_Motor *m);
void SteadyWin_ReadAngle(SteadyWin_Motor *m);
void SteadyWin_ReadMulti(SteadyWin_Motor *m);
void SteadyWin_ReadMotorInfo(SteadyWin_Motor *m);

/* Parameter commands */
void SteadyWin_SetZero(SteadyWin_Motor *m);
void SteadyWin_SetMaxSpeed(SteadyWin_Motor *m, uint32_t speed_001rpm);
void SteadyWin_SetMaxCurrent(SteadyWin_Motor *m, uint32_t current_001a);
void SteadyWin_SetCurrentSlope(SteadyWin_Motor *m, uint32_t slope_001a_s);
void SteadyWin_SetAccel(SteadyWin_Motor *m, uint32_t accel_001rpm_s);

/* Control commands */
void SteadyWin_CurrentControl(SteadyWin_Motor *m, int32_t current_001a);
void SteadyWin_SpeedControl(SteadyWin_Motor *m, int32_t speed_001rpm);
void SteadyWin_AbsPosControl(SteadyWin_Motor *m, int32_t position_count);
void SteadyWin_RelPosControl(SteadyWin_Motor *m, int32_t position_count);
void SteadyWin_GotoOrigin(SteadyWin_Motor *m);
void SteadyWin_BrakeControl(SteadyWin_Motor *m, uint8_t action);

#endif /* __STEADYWIN_H__ */
