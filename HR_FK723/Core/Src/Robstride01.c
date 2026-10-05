#include "Robstride.h"
#include <string.h>
#include <stdint.h>

#define P_MIN  -12.5f
#define P_MAX   12.5f
#define V_MIN  -44.0f
#define V_MAX   44.0f
#define KP_MIN   0.0f
#define KP_MAX 500.0f
#define KD_MIN   0.0f
#define KD_MAX   5.0f
#define T_MIN  -17.0f
#define T_MAX   17.0f

/* RobStride knee motor IDs */
extern uint8_t right_knee_motor_addr;
extern uint8_t left_knee_motor_addr; //not 0x3E

static const uint16_t Index_List[] = {
    0x7005, 0x7006, 0x700A, 0x700B, 0x7010, 0x7011, 0x7014,
    0x7016, 0x7017, 0x7018, 0x7019, 0x701A, 0x701B, 0x701C, 0x701D
};

/* ---------- helpers ---------- */

static float uint16_to_float(uint16_t x, float x_min, float x_max, int bits)
{
    uint32_t span = (1 << bits) - 1;
    x &= span;
    float offset = x_max - x_min;
    return offset * x / span + x_min;
}

static int float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    if (x > x_max) x = x_max;
    else if (x < x_min) x = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

static float Byte_to_float(uint8_t *bytedata)
{
    uint32_t data = (uint32_t)bytedata[7] << 24 | (uint32_t)bytedata[6] << 16 |
                    (uint32_t)bytedata[5] << 8  | (uint32_t)bytedata[4];
    float data_float;
    memcpy(&data_float, &data, sizeof(float));
    return data_float;
}

static uint8_t mapFaults(uint16_t fault16)
{
    uint8_t fault8 = 0;
    if (fault16 & (1 << 14)) fault8 |= (1 << 4);
    if (fault16 & (1 <<  7)) fault8 |= (1 << 5);
    if (fault16 & (1 <<  3)) fault8 |= (1 << 3);
    if (fault16 & (1 <<  2)) fault8 |= (1 << 0);
    if (fault16 & (1 <<  1)) fault8 |= (1 << 1);
    if (fault16 & (1 <<  0)) fault8 |= (1 << 2);
    return fault8;
}

/* ---------- FDCAN transmit helpers ---------- */

static void fdcan_send_ext(RobStride_Motor *m, uint32_t ext_id, uint8_t *data)
{
    FDCAN_TxHeaderTypeDef hdr;
    hdr.Identifier          = ext_id;
    hdr.IdType              = FDCAN_EXTENDED_ID;
    hdr.TxFrameType         = FDCAN_DATA_FRAME;
    hdr.DataLength          = FDCAN_DLC_BYTES_8;
    hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    hdr.BitRateSwitch       = FDCAN_BRS_OFF;
    hdr.FDFormat            = FDCAN_CLASSIC_CAN;
    hdr.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    hdr.MessageMarker       = 0;
    HAL_FDCAN_AddMessageToTxFifoQ(m->hfdcan, &hdr, data);
}

static void fdcan_send_std(RobStride_Motor *m, uint32_t std_id, uint8_t *data)
{
    FDCAN_TxHeaderTypeDef hdr;
    hdr.Identifier          = std_id;
    hdr.IdType              = FDCAN_STANDARD_ID;
    hdr.TxFrameType         = FDCAN_DATA_FRAME;
    hdr.DataLength          = FDCAN_DLC_BYTES_8;
    hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    hdr.BitRateSwitch       = FDCAN_BRS_OFF;
    hdr.FDFormat            = FDCAN_CLASSIC_CAN;
    hdr.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    hdr.MessageMarker       = 0;
    HAL_FDCAN_AddMessageToTxFifoQ(m->hfdcan, &hdr, data);
}

/* ---------- Init ---------- */

void RobStride_InitDrw(DataReadWrite *drw)
{
    drw->run_mode.index     = Index_List[0];
    drw->iq_ref.index       = Index_List[1];
    drw->spd_ref.index      = Index_List[2];
    drw->imit_torque.index  = Index_List[3];
    drw->cur_kp.index       = Index_List[4];
    drw->cur_ki.index       = Index_List[5];
    drw->cur_filt_gain.index= Index_List[6];
    drw->loc_ref.index      = Index_List[7];
    drw->limit_spd.index    = Index_List[8];
    drw->limit_cur.index    = Index_List[9];
    drw->mechPos.index      = Index_List[10];
    drw->iqf.index          = Index_List[11];
    drw->mechVel.index      = Index_List[12];
    drw->VBUS.index         = Index_List[13];
    drw->rotation.index     = Index_List[14];
}

void RobStride_Init(RobStride_Motor *m, uint8_t CAN_Id, int MIT_mode,
                    FDCAN_HandleTypeDef *hfdcan)
{
    memset(m, 0, sizeof(*m));
    m->CAN_ID        = CAN_Id;
    m->Master_CAN_ID = 0xFD;
    m->Motor_Set_All.set_motor_mode = move_control_mode;
    m->MIT_Mode      = MIT_mode;
    m->MIT_Type      = operationControl;
    m->hfdcan        = hfdcan;
    RobStride_InitDrw(&m->drw);
}

/* ---------- Analysis (RX) ---------- */

void RobStride_Analysis(RobStride_Motor *m, uint8_t *DataFrame, uint32_t ID_ExtId)
{
    if (m->MIT_Mode)
    {
        if ((ID_ExtId & 0xFF) == 0xFD)
        {
            if (DataFrame[3] == 0x00 && DataFrame[4] == 0x00 &&
                DataFrame[5] == 0x00 && DataFrame[6] == 0x00 && DataFrame[7] == 0x00)
            {
                uint16_t fault16 = 0;
                memcpy(&fault16, &DataFrame[1], 2);
                m->error_code = mapFaults(fault16);
            }
            else
            {
                m->Pos_Info.Angle  = uint16_to_float((DataFrame[1] << 8) | DataFrame[2], P_MIN, P_MAX, 16);
                m->Pos_Info.Speed  = uint16_to_float((DataFrame[3] << 4) | (DataFrame[4] >> 4), V_MIN, V_MAX, 12);
                m->Pos_Info.Torque = uint16_to_float((DataFrame[4] << 8) | DataFrame[5], T_MIN, T_MAX, 12);
                m->Pos_Info.Temp   = ((DataFrame[6] << 8) | DataFrame[7]) * 0.1f;
            }
        }
        else
        {
            memcpy(&m->Unique_ID, DataFrame, 8);
        }
    }
    else
    {
        if ((uint8_t)((ID_ExtId & 0xFF00) >> 8) == m->CAN_ID)
        {
            int comm_type = (int)((ID_ExtId & 0x3F000000) >> 24);
            if (comm_type == 2 || comm_type == 0x18)
            {
                m->Pos_Info.Angle   = uint16_to_float(DataFrame[0] << 8 | DataFrame[1], P_MIN, P_MAX, 16);
                m->Pos_Info.Speed   = uint16_to_float(DataFrame[2] << 8 | DataFrame[3], V_MIN, V_MAX, 16);
                m->Pos_Info.Torque  = uint16_to_float(DataFrame[4] << 8 | DataFrame[5], T_MIN, T_MAX, 16);
                m->Pos_Info.Temp    = (DataFrame[6] << 8 | DataFrame[7]) * 0.1f;
                m->error_code       = (uint8_t)((ID_ExtId & 0x3F0000) >> 16);
                m->Pos_Info.pattern = (uint8_t)((ID_ExtId & 0xC00000) >> 22);
            }
            else if (comm_type == 17)
            {
                int idx;
                for (idx = 0; idx <= 13; idx++)
                {
                    if ((DataFrame[1] << 8 | DataFrame[0]) == Index_List[idx])
                    {
                        switch (idx)
                        {
                            case 0:  m->drw.run_mode.data     = (uint8_t)DataFrame[4]; break;
                            case 1:  m->drw.iq_ref.data       = Byte_to_float(DataFrame); break;
                            case 2:  m->drw.spd_ref.data      = Byte_to_float(DataFrame); break;
                            case 3:  m->drw.imit_torque.data  = Byte_to_float(DataFrame); break;
                            case 4:  m->drw.cur_kp.data       = Byte_to_float(DataFrame); break;
                            case 5:  m->drw.cur_ki.data       = Byte_to_float(DataFrame); break;
                            case 6:  m->drw.cur_filt_gain.data= Byte_to_float(DataFrame); break;
                            case 7:  m->drw.loc_ref.data      = Byte_to_float(DataFrame); break;
                            case 8:  m->drw.limit_spd.data    = Byte_to_float(DataFrame); break;
                            case 9:  m->drw.limit_cur.data    = Byte_to_float(DataFrame); break;
                            case 10: m->drw.mechPos.data      = Byte_to_float(DataFrame); break;
                            case 11: m->drw.iqf.data          = Byte_to_float(DataFrame); break;
                            case 12: m->drw.mechVel.data      = Byte_to_float(DataFrame); break;
                            case 13: m->drw.VBUS.data         = Byte_to_float(DataFrame); break;
                        }
                    }
                }
            }
            else if ((uint8_t)(ID_ExtId & 0xFF) == 0xFE)
            {
                m->CAN_ID = (uint8_t)((ID_ExtId & 0xFF00) >> 8);
                memcpy(&m->Unique_ID, DataFrame, 8);
            }
        }
    }
}

/* ---------- Get device ID (comm type 0) ---------- */

void RobStride_GetCAN_ID(RobStride_Motor *m)
{
    uint8_t txdata[8] = {0};
    uint32_t ext_id = (uint32_t)Communication_Type_Get_ID << 24 |
                      (uint32_t)m->Master_CAN_ID << 8 | m->CAN_ID;
    fdcan_send_ext(m, ext_id, txdata);
}

/* ---------- Motion control (comm type 1) ---------- */

void RobStride_MoveControl(RobStride_Motor *m, float Torque, float Angle, float Speed, float Kp, float Kd)
{
    uint8_t txdata[8] = {0};
    m->Motor_Set_All.set_Torque = Torque;
    m->Motor_Set_All.set_angle  = Angle;
    m->Motor_Set_All.set_speed  = Speed;
    m->Motor_Set_All.set_Kp     = Kp;
    m->Motor_Set_All.set_Kd     = Kd;

    if (m->drw.run_mode.data != 0)
    {
        RobStride_SetParameter(m, 0x7005, move_control_mode, Set_mode);
        RobStride_GetParameter(m, 0x7005);
        RobStride_Enable(m);
        m->Motor_Set_All.set_motor_mode = move_control_mode;
    }
    if (m->Pos_Info.pattern != 2)
    {
        RobStride_Enable(m);
    }

    uint32_t ext_id = (uint32_t)Communication_Type_MotionControl << 24 |
                      (uint32_t)float_to_uint(m->Motor_Set_All.set_Torque, T_MIN, T_MAX, 16) << 8 |
                      m->CAN_ID;
    txdata[0] = float_to_uint(m->Motor_Set_All.set_angle, P_MIN, P_MAX, 16) >> 8;
    txdata[1] = float_to_uint(m->Motor_Set_All.set_angle, P_MIN, P_MAX, 16);
    txdata[2] = float_to_uint(m->Motor_Set_All.set_speed, V_MIN, V_MAX, 16) >> 8;
    txdata[3] = float_to_uint(m->Motor_Set_All.set_speed, V_MIN, V_MAX, 16);
    txdata[4] = float_to_uint(m->Motor_Set_All.set_Kp, KP_MIN, KP_MAX, 16) >> 8;
    txdata[5] = float_to_uint(m->Motor_Set_All.set_Kp, KP_MIN, KP_MAX, 16);
    txdata[6] = float_to_uint(m->Motor_Set_All.set_Kd, KD_MIN, KD_MAX, 16) >> 8;
    txdata[7] = float_to_uint(m->Motor_Set_All.set_Kd, KD_MIN, KD_MAX, 16);
    fdcan_send_ext(m, ext_id, txdata);
}

/* ---------- MIT Enable ---------- */

void RobStride_MIT_Enable(RobStride_Motor *m)
{
    uint8_t txdata[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    fdcan_send_std(m, m->CAN_ID, txdata);
}

/* ---------- MIT Disable ---------- */

void RobStride_MIT_Disable(RobStride_Motor *m)
{
    uint8_t txdata[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    fdcan_send_std(m, m->CAN_ID, txdata);
}

/* ---------- MIT Clear/Check Error ---------- */

void RobStride_MIT_ClearOrCheckError(RobStride_Motor *m, uint8_t F_CMD)
{
    uint8_t txdata[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, F_CMD, 0xFB};
    fdcan_send_std(m, m->CAN_ID, txdata);
}

/* ---------- MIT Set Motor Type ---------- */

void RobStride_MIT_SetMotorType(RobStride_Motor *m, uint8_t F_CMD)
{
    uint8_t txdata[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, F_CMD, 0xFC};
    fdcan_send_std(m, m->CAN_ID, txdata);
}

/* ---------- MIT Set Motor Id ---------- */

void RobStride_MIT_SetMotorId(RobStride_Motor *m, uint8_t F_CMD)
{
    uint8_t txdata[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, F_CMD, 0x01};
    fdcan_send_std(m, m->CAN_ID, txdata);
}

/* ---------- MIT Control ---------- */

void RobStride_MIT_Control(RobStride_Motor *m, float Angle, float Speed, float Kp, float Kd, float Torque)
{
    uint8_t txdata[8] = {0};
    txdata[0] = float_to_uint(Angle, P_MIN, P_MAX, 16) >> 8;
    txdata[1] = float_to_uint(Angle, P_MIN, P_MAX, 16);
    txdata[2] = float_to_uint(Speed, V_MIN, V_MAX, 12) >> 4;
    txdata[3] = (float_to_uint(Speed, V_MIN, V_MAX, 12) << 4) |
                (float_to_uint(Kp, KP_MIN, KP_MAX, 12) >> 8);
    txdata[4] = float_to_uint(Kp, KP_MIN, KP_MAX, 12);
    txdata[5] = float_to_uint(Kd, KD_MIN, KD_MAX, 12) >> 4;
    txdata[6] = (float_to_uint(Kd, KD_MIN, KD_MAX, 12) << 4) |
                (float_to_uint(Torque, T_MIN, T_MAX, 12) >> 8);
    txdata[7] = float_to_uint(Torque, T_MIN, T_MAX, 12);
    fdcan_send_std(m, m->CAN_ID, txdata);
}

/* ---------- MIT Position Control ---------- */

void RobStride_MIT_PositionControl(RobStride_Motor *m, float position_rad, float speed_rad_per_s)
{
    uint8_t txdata[8] = {0};
    memcpy(&txdata[0], &position_rad, 4);
    memcpy(&txdata[4], &speed_rad_per_s, 4);
    fdcan_send_std(m, (1 << 8) | m->CAN_ID, txdata);
}

/* ---------- MIT Speed Control ---------- */

void RobStride_MIT_SpeedControl(RobStride_Motor *m, float speed_rad_per_s, float current_limit)
{
    uint8_t txdata[8] = {0};
    memcpy(&txdata[0], &speed_rad_per_s, 4);
    memcpy(&txdata[4], &current_limit, 4);
    fdcan_send_std(m, (2 << 8) | m->CAN_ID, txdata);
}

/* ---------- MIT Set Zero Position ---------- */

void RobStride_MIT_SetZeroPos(RobStride_Motor *m)
{
    uint8_t txdata[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
    fdcan_send_std(m, m->CAN_ID, txdata);
}

/* ---------- Position control (PP mode) ---------- */

void RobStride_PosControl(RobStride_Motor *m, float Speed, float Angle)
{
    m->Motor_Set_All.set_speed = Speed;
    m->Motor_Set_All.set_angle = Angle;

    if (m->drw.run_mode.data != 1)
    {
        RobStride_SetParameter(m, 0x7005, Pos_control_mode, Set_mode);
        RobStride_GetParameter(m, 0x7005);
        m->Motor_Set_All.set_motor_mode = Pos_control_mode;
        RobStride_Enable(m);
        RobStride_SetParameter(m, 0x7024, m->Motor_Set_All.set_limit_speed, Set_parameter);
        RobStride_SetParameter(m, 0x7025, m->Motor_Set_All.set_acceleration, Set_parameter);
    }
    HAL_Delay(1);
    RobStride_SetParameter(m, 0x7016, m->Motor_Set_All.set_angle, Set_parameter);
}

/* ---------- CSP Position control ---------- */

void RobStride_CSPControl(RobStride_Motor *m, float Angle, float limit_spd)
{
    if (m->MIT_Mode)
    {
        RobStride_MIT_PositionControl(m, Angle, limit_spd);
    }
    else
    {
        m->Motor_Set_All.set_angle = Angle;
        m->Motor_Set_All.set_limit_speed = limit_spd;
        if (m->drw.run_mode.data != 1)
        {
            RobStride_SetParameter(m, 0x7005, CSP_control_mode, Set_mode);
            RobStride_GetParameter(m, 0x7005);
            RobStride_Enable(m);
            RobStride_SetParameter(m, 0x7017, m->Motor_Set_All.set_limit_speed, Set_parameter);
        }
        HAL_Delay(1);
        RobStride_SetParameter(m, 0x7016, m->Motor_Set_All.set_angle, Set_parameter);
    }
}

/* ---------- Speed control ---------- */

void RobStride_SpeedControl(RobStride_Motor *m, float Speed, float limit_cur)
{
    m->Motor_Set_All.set_speed     = Speed;
    m->Motor_Set_All.set_limit_cur = limit_cur;

    if (m->drw.run_mode.data != 2)
    {
        RobStride_SetParameter(m, 0x7005, Speed_control_mode, Set_mode);
        RobStride_GetParameter(m, 0x7005);
        RobStride_Enable(m);
        m->Motor_Set_All.set_motor_mode = Speed_control_mode;
        RobStride_SetParameter(m, 0x7018, m->Motor_Set_All.set_limit_cur, Set_parameter);
        RobStride_SetParameter(m, 0x7022, 10, Set_parameter);
    }
    RobStride_SetParameter(m, 0x700A, m->Motor_Set_All.set_speed, Set_parameter);
}

/* ---------- Current control ---------- */

void RobStride_CurrentControl(RobStride_Motor *m, float current)
{
    m->Motor_Set_All.set_current = current;
    m->output = m->Motor_Set_All.set_current;

    if (m->Motor_Set_All.set_motor_mode != 3)
    {
        RobStride_SetParameter(m, 0x7005, Elect_control_mode, Set_mode);
        RobStride_GetParameter(m, 0x7005);
        m->Motor_Set_All.set_motor_mode = Elect_control_mode;
        RobStride_Enable(m);
    }
    RobStride_SetParameter(m, 0x7006, m->Motor_Set_All.set_current, Set_parameter);
}

/* ---------- Set-zero mode ---------- */

void RobStride_SetZeroControl(RobStride_Motor *m)
{
    RobStride_SetParameter(m, 0x7005, Set_Zero_mode, Set_mode);
}

/* ---------- Enable (comm type 3) ---------- */

void RobStride_Enable(RobStride_Motor *m)
{
    if (m->MIT_Mode)
    {
        RobStride_MIT_Enable(m);
    }
    else
    {
        uint8_t txdata[8] = {0};
        uint32_t ext_id = (uint32_t)Communication_Type_MotorEnable << 24 |
                          (uint32_t)m->Master_CAN_ID << 8 | m->CAN_ID;
        fdcan_send_ext(m, ext_id, txdata);
    }
}

/* ---------- Disable (comm type 4) ---------- */

void RobStride_Disable(RobStride_Motor *m, uint8_t clear_error)
{
    if (m->MIT_Mode)
    {
        RobStride_MIT_Disable(m);
    }
    else
    {
        uint8_t txdata[8] = {0};
        txdata[0] = clear_error;
        uint32_t ext_id = (uint32_t)Communication_Type_MotorStop << 24 |
                          (uint32_t)m->Master_CAN_ID << 8 | m->CAN_ID;
        fdcan_send_ext(m, ext_id, txdata);
        RobStride_SetParameter(m, 0x7005, move_control_mode, Set_mode);
    }
}

/* ---------- Set parameter (comm type 18) ---------- */

void RobStride_SetParameter(RobStride_Motor *m, uint16_t Index, float Value, char Value_mode)
{
    uint8_t txdata[8] = {0};
    uint32_t ext_id = (uint32_t)Communication_Type_SetSingleParameter << 24 |
                      (uint32_t)m->Master_CAN_ID << 8 | m->CAN_ID;
    txdata[0] = (uint8_t)(Index);
    txdata[1] = (uint8_t)(Index >> 8);
    txdata[2] = 0x00;
    txdata[3] = 0x00;

    if (Value_mode == 'p')
    {
        memcpy(&txdata[4], &Value, 4);
    }
    else if (Value_mode == 'j')
    {
        m->Motor_Set_All.set_motor_mode = (int)Value;
        txdata[4] = (uint8_t)Value;
        txdata[5] = 0x00;
        txdata[6] = 0x00;
        txdata[7] = 0x00;
    }
    fdcan_send_ext(m, ext_id, txdata);
}

/* ---------- Get parameter (comm type 17) ---------- */

void RobStride_GetParameter(RobStride_Motor *m, uint16_t Index)
{
    uint8_t txdata[8] = {0};
    txdata[0] = (uint8_t)(Index);
    txdata[1] = (uint8_t)(Index >> 8);
    uint32_t ext_id = (uint32_t)Communication_Type_GetSingleParameter << 24 |
                      (uint32_t)m->Master_CAN_ID << 8 | m->CAN_ID;
    fdcan_send_ext(m, ext_id, txdata);
}

/* ---------- Motor Request (comm type 2) — request feedback ---------- */

void RobStride_MotorRequest(RobStride_Motor *m)
{
    uint8_t txdata[8] = {0};
    uint32_t ext_id = (uint32_t)Communication_Type_MotorRequest << 24 |
                      (uint32_t)m->Master_CAN_ID << 8 | m->CAN_ID;
    fdcan_send_ext(m, ext_id, txdata);
}

/* ---------- Set CAN_ID (comm type 7) ---------- */

void RobStride_SetCAN_ID_Val(RobStride_Motor *m, uint8_t new_id)
{
    RobStride_Disable(m, 0);
    uint8_t txdata[8] = {0};
    uint32_t ext_id = (uint32_t)Communication_Type_Can_ID << 24 |
                      (uint32_t)new_id << 16 |
                      (uint32_t)m->Master_CAN_ID << 8 | m->CAN_ID;
    fdcan_send_ext(m, ext_id, txdata);
}

/* ---------- Set zero position (comm type 6) ---------- */

void RobStride_SetZeroPos(RobStride_Motor *m)
{
    RobStride_Disable(m, 0);
    uint8_t txdata[8] = {0};
    txdata[0] = 1;
    uint32_t ext_id = (uint32_t)Communication_Type_SetPosZero << 24 |
                      (uint32_t)m->Master_CAN_ID << 8 | m->CAN_ID;
    fdcan_send_ext(m, ext_id, txdata);
    RobStride_Enable(m);
}

/* ---------- Motor data save (comm type 22) ---------- */

void RobStride_MotorDataSave(RobStride_Motor *m)
{
    uint8_t txdata[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    uint32_t ext_id = (uint32_t)Communication_Type_MotorDataSave << 24 |
                      (uint32_t)m->Master_CAN_ID << 8 | m->CAN_ID;
    fdcan_send_ext(m, ext_id, txdata);
}

/* ---------- Baud rate change (comm type 23) ---------- */

void RobStride_BaudRateChange(RobStride_Motor *m, uint8_t F_CMD)
{
    uint8_t txdata[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, F_CMD, 0x08};
    uint32_t ext_id = (uint32_t)Communication_Type_BaudRateChange << 24 |
                      (uint32_t)m->Master_CAN_ID << 8 | m->CAN_ID;
    fdcan_send_ext(m, ext_id, txdata);
}

/* ---------- Proactive escalation set (comm type 24) ---------- */

void RobStride_ProactiveEscalationSet(RobStride_Motor *m, uint8_t F_CMD)
{
    uint8_t txdata[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, F_CMD, 0x08};
    uint32_t ext_id = (uint32_t)Communication_Type_ProactiveEscalationSet << 24 |
                      (uint32_t)m->Master_CAN_ID << 8 | m->CAN_ID;
    fdcan_send_ext(m, ext_id, txdata);
}

/* ---------- MIT Motor mode set (comm type 25) ---------- */

void RobStride_MIT_MotorModeSet(RobStride_Motor *m, uint8_t F_CMD)
{
    uint8_t txdata[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, F_CMD, 0xFD};
    fdcan_send_std(m, m->CAN_ID, txdata);
}

/* ---------- Motor mode set (private protocol, comm type 25) ---------- */

void RobStride_MotorModeSet(RobStride_Motor *m, uint8_t F_CMD)
{
    uint8_t txdata[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, F_CMD, 0x08};
    uint32_t ext_id = (uint32_t)Communication_Type_MotorModeSet << 24 |
                      (uint32_t)m->Master_CAN_ID << 8 | m->CAN_ID;
    fdcan_send_ext(m, ext_id, txdata);
}
