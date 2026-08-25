#ifndef __ROBSTRIDE_H__
#define __ROBSTRIDE_H__

#include "main.h"

/* Control mode characters */
#define Set_mode          'j'
#define Set_parameter     'p'

/* Motor control modes */
#define move_control_mode   0
#define Pos_control_mode    1
#define Speed_control_mode  2
#define Elect_control_mode  3
#define Set_Zero_mode       4
#define CSP_control_mode    5

/* Communication type addresses */
#define Communication_Type_Get_ID                   0x00
#define Communication_Type_MotionControl            0x01
#define Communication_Type_MotorRequest             0x02
#define Communication_Type_MotorEnable              0x03
#define Communication_Type_MotorStop                0x04
#define Communication_Type_SetPosZero               0x06
#define Communication_Type_Can_ID                   0x07
#define Communication_Type_Control_Mode             0x12
#define Communication_Type_GetSingleParameter       0x11
#define Communication_Type_SetSingleParameter       0x12
#define Communication_Type_ErrorFeedback            0x15
#define Communication_Type_MotorDataSave            0x16
#define Communication_Type_BaudRateChange           0x17
#define Communication_Type_ProactiveEscalationSet   0x18
#define Communication_Type_MotorModeSet             0x19

/* Parameter read/write entry */
typedef struct {
    uint16_t index;
    float data;
} DataReadWriteOne;

/* Parameter table */
typedef struct {
    DataReadWriteOne run_mode;
    DataReadWriteOne iq_ref;
    DataReadWriteOne spd_ref;
    DataReadWriteOne imit_torque;
    DataReadWriteOne cur_kp;
    DataReadWriteOne cur_ki;
    DataReadWriteOne cur_filt_gain;
    DataReadWriteOne loc_ref;
    DataReadWriteOne limit_spd;
    DataReadWriteOne limit_cur;
    /* Read-only */
    DataReadWriteOne mechPos;
    DataReadWriteOne iqf;
    DataReadWriteOne mechVel;
    DataReadWriteOne VBUS;
    DataReadWriteOne rotation;
} DataReadWrite;

/* Motor feedback info */
typedef struct {
    float Angle;
    float Speed;
    float Torque;
    float Temp;
    int   pattern;
} Motor_Pos_RobStride_Info;

/* Motor set-point values */
typedef struct {
    int   set_motor_mode;
    float set_current;
    float set_speed;
    float set_acceleration;
    float set_Torque;
    float set_angle;
    float set_limit_cur;
    float set_limit_speed;
    float set_Kp;
    float set_Ki;
    float set_Kd;
} Motor_Set;

/* MIT mode type */
typedef enum {
    operationControl = 0,
    positionControl  = 1,
    speedControl     = 2
} MIT_TYPE;

/* RobStride motor instance (C struct) */
typedef struct {
    uint8_t   CAN_ID;
    uint64_t  Unique_ID;
    uint16_t  Master_CAN_ID;

    Motor_Set Motor_Set_All;
    uint8_t   error_code;

    int       MIT_Mode;     /* 0 = private protocol, 1 = MIT */
    MIT_TYPE  MIT_Type;

    float     output;
    int       Can_Motor;
    Motor_Pos_RobStride_Info Pos_Info;
    DataReadWrite drw;

    FDCAN_HandleTypeDef *hfdcan;   /* FDCAN peripheral handle */
} RobStride_Motor;

/* ---- API ---- */
void RobStride_Init(RobStride_Motor *m, uint8_t CAN_Id, int MIT_mode,
                    FDCAN_HandleTypeDef *hfdcan);
void RobStride_InitDrw(DataReadWrite *drw);

void RobStride_Analysis(RobStride_Motor *m, uint8_t *DataFrame, uint32_t ID_ExtId);

void RobStride_GetCAN_ID(RobStride_Motor *m);
void RobStride_MotorRequest(RobStride_Motor *m);
void RobStride_SetParameter(RobStride_Motor *m, uint16_t Index, float Value, char Value_mode);
void RobStride_GetParameter(RobStride_Motor *m, uint16_t Index);

void RobStride_MoveControl(RobStride_Motor *m, float Torque, float Angle, float Speed, float Kp, float Kd);
void RobStride_PosControl(RobStride_Motor *m, float Speed, float Angle);
void RobStride_CSPControl(RobStride_Motor *m, float Angle, float limit_spd);
void RobStride_SpeedControl(RobStride_Motor *m, float Speed, float limit_cur);
void RobStride_CurrentControl(RobStride_Motor *m, float current);
void RobStride_SetZeroControl(RobStride_Motor *m);

void RobStride_Enable(RobStride_Motor *m);
void RobStride_Disable(RobStride_Motor *m, uint8_t clear_error);
void RobStride_SetCAN_ID_Val(RobStride_Motor *m, uint8_t new_id);
void RobStride_SetZeroPos(RobStride_Motor *m);
void RobStride_MotorDataSave(RobStride_Motor *m);
void RobStride_BaudRateChange(RobStride_Motor *m, uint8_t F_CMD);
void RobStride_ProactiveEscalationSet(RobStride_Motor *m, uint8_t F_CMD);
void RobStride_MotorModeSet(RobStride_Motor *m, uint8_t F_CMD);

/* MIT-specific */
void RobStride_MIT_Enable(RobStride_Motor *m);
void RobStride_MIT_Disable(RobStride_Motor *m);
void RobStride_MIT_Control(RobStride_Motor *m, float Angle, float Speed, float Kp, float Kd, float Torque);
void RobStride_MIT_PositionControl(RobStride_Motor *m, float position_rad, float speed_rad_per_s);
void RobStride_MIT_SpeedControl(RobStride_Motor *m, float speed_rad_per_s, float current_limit);
void RobStride_MIT_SetZeroPos(RobStride_Motor *m);
void RobStride_MIT_ClearOrCheckError(RobStride_Motor *m, uint8_t F_CMD);
void RobStride_MIT_SetMotorType(RobStride_Motor *m, uint8_t F_CMD);
void RobStride_MIT_SetMotorId(RobStride_Motor *m, uint8_t F_CMD);
void RobStride_MIT_MotorModeSet(RobStride_Motor *m, uint8_t F_CMD);

#endif /* __ROBSTRIDE_H__ */
