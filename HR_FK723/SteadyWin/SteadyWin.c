#include "SteadyWin.h"
#include <string.h>

/* ---------- FDCAN transmit helper (Standard ID) ---------- */

static void sw_send_std(SteadyWin_Motor *m, uint32_t std_id,
                        uint8_t *data, uint32_t dlc_code)
{
    FDCAN_TxHeaderTypeDef hdr;
    hdr.Identifier          = std_id;
    hdr.IdType              = FDCAN_STANDARD_ID;
    hdr.TxFrameType         = FDCAN_DATA_FRAME;
    hdr.DataLength          = dlc_code;
    hdr.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    hdr.BitRateSwitch       = FDCAN_BRS_OFF;
    hdr.FDFormat            = FDCAN_CLASSIC_CAN;
    hdr.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    hdr.MessageMarker       = 0;
    HAL_FDCAN_AddMessageToTxFifoQ(m->hfdcan, &hdr, data);
}

/* Map byte count to FDCAN DLC code */
static uint32_t sw_dlc(uint8_t bytes)
{
    switch (bytes) {
        case 0: return FDCAN_DLC_BYTES_0;
        case 1: return FDCAN_DLC_BYTES_1;
        case 2: return FDCAN_DLC_BYTES_2;
        case 3: return FDCAN_DLC_BYTES_3;
        case 4: return FDCAN_DLC_BYTES_4;
        case 5: return FDCAN_DLC_BYTES_5;
        case 6: return FDCAN_DLC_BYTES_6;
        case 7: return FDCAN_DLC_BYTES_7;
        default: return FDCAN_DLC_BYTES_8;
    }
}

/* Send command with only the command code byte */
static void sw_send_cmd1(SteadyWin_Motor *m, uint8_t cmd)
{
    uint8_t txdata[8] = {0};
    txdata[0] = cmd;
    sw_send_std(m, 0x100 | m->dev_addr, txdata, sw_dlc(1));
}

/* Send command with cmd + 4 bytes of data (DLC=5, little-endian) */
static void sw_send_cmd5_u32(SteadyWin_Motor *m, uint8_t cmd, uint32_t value)
{
    uint8_t txdata[8] = {0};
    txdata[0] = cmd;
    txdata[1] = (uint8_t)(value);
    txdata[2] = (uint8_t)(value >> 8);
    txdata[3] = (uint8_t)(value >> 16);
    txdata[4] = (uint8_t)(value >> 24);
    sw_send_std(m, 0x100 | m->dev_addr, txdata, sw_dlc(5));
}

/* Send command with cmd + 4 bytes of signed data (DLC=5, little-endian) */
static void sw_send_cmd5_s32(SteadyWin_Motor *m, uint8_t cmd, int32_t value)
{
    uint32_t v;
    memcpy(&v, &value, 4);
    sw_send_cmd5_u32(m, cmd, v);
}

/* ---------- Init ---------- */

void SteadyWin_Init(SteadyWin_Motor *m, uint8_t dev_addr,
                    FDCAN_HandleTypeDef *hfdcan)
{
    memset(m, 0, sizeof(*m));
    m->dev_addr = dev_addr;
    m->hfdcan   = hfdcan;
}

/* ---------- RX Analysis ---------- */

void SteadyWin_Analysis(SteadyWin_Motor *m, uint8_t *data, uint8_t dlc,
                        uint32_t std_id)
{
    /* Only process frames from our device address.
     * Some firmware versions echo back the request ID (0x100|dev_addr),
     * others reply with bare dev_addr — accept both. */
    if (std_id != m->dev_addr && std_id != (uint32_t)(0x100U | m->dev_addr))
        return;
    if (dlc < 1)
        return;

    uint8_t cmd = data[0];

    switch (cmd) {
    case SW_CMD_READ_CURRENT:   /* 0xA1 */
    case SW_CMD_CURRENT_CTRL:   /* 0xC0 - same format */
        if (dlc >= 5) {
            m->fb.q_current_raw = (int32_t)((uint32_t)data[1] |
                                  ((uint32_t)data[2] << 8) |
                                  ((uint32_t)data[3] << 16) |
                                  ((uint32_t)data[4] << 24));
        }
        break;

    case SW_CMD_READ_SPEED:     /* 0xA2 */
    case SW_CMD_SPEED_CTRL:     /* 0xC1 - same format */
        if (dlc >= 5) {
            m->fb.speed_raw = (int32_t)((uint32_t)data[1] |
                              ((uint32_t)data[2] << 8) |
                              ((uint32_t)data[3] << 16) |
                              ((uint32_t)data[4] << 24));
        }
        break;

    case SW_CMD_READ_ANGLE:     /* 0xA3 */
    case SW_CMD_ABS_POS_CTRL:   /* 0xC2 - same format */
    case SW_CMD_REL_POS_CTRL:   /* 0xC3 - same format */
    case SW_CMD_GOTO_ORIGIN:    /* 0xC4 - same format */
        if (dlc >= 3) {
            m->fb.single_angle_raw = (uint16_t)data[1] |
                                     ((uint16_t)data[2] << 8);
        }
        if (dlc >= 7) {
            m->fb.multi_angle_raw = (int32_t)((uint32_t)data[3] |
                                    ((uint32_t)data[4] << 8) |
                                    ((uint32_t)data[5] << 16) |
                                    ((uint32_t)data[6] << 24));
        }
        break;

    case SW_CMD_READ_MULTI:     /* 0xA4 */
        if (dlc >= 8) {
            m->fb.temp_a4    = data[1];
            m->fb.current_a4 = (int16_t)((uint16_t)data[2] | ((uint16_t)data[3] << 8));
            m->fb.speed_a4   = (int16_t)((uint16_t)data[4] | ((uint16_t)data[5] << 8));
            m->fb.angle_a4   = (uint16_t)data[6] | ((uint16_t)data[7] << 8);
        }
        break;

    case SW_CMD_READ_STATUS:    /* 0xAE */
    case SW_CMD_DISABLE:        /* 0xCF - same format */
        if (dlc >= 8) {
            m->fb.bus_voltage_raw = (uint16_t)data[1] | ((uint16_t)data[2] << 8);
            m->fb.bus_current_raw = (uint16_t)data[3] | ((uint16_t)data[4] << 8);
            m->fb.temperature     = data[5];
            m->fb.run_mode        = data[6];
            m->fb.fault_code      = data[7];
        }
        break;

    case SW_CMD_CLEAR_FAULT:    /* 0xAF */
        if (dlc >= 2) {
            m->fb.fault_code = data[1];
        }
        break;

    default:
        break;
    }
}

/* ---------- System commands ---------- */

void SteadyWin_Reboot(SteadyWin_Motor *m)
{
    uint8_t txdata[8] = {0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF, 0x00, 0xFF};
    sw_send_std(m, 0x100 | m->dev_addr, txdata, sw_dlc(8));
}

void SteadyWin_ReadVersion(SteadyWin_Motor *m)
{
    sw_send_cmd1(m, SW_CMD_READ_VERSION);
}

void SteadyWin_ReadStatus(SteadyWin_Motor *m)
{
    sw_send_cmd1(m, SW_CMD_READ_STATUS);
}

void SteadyWin_ClearFault(SteadyWin_Motor *m)
{
    sw_send_cmd1(m, SW_CMD_CLEAR_FAULT);
}

void SteadyWin_Disable(SteadyWin_Motor *m)
{
    sw_send_cmd1(m, SW_CMD_DISABLE);
}

/* ---------- Read commands ---------- */

void SteadyWin_ReadCurrent(SteadyWin_Motor *m)
{
    sw_send_cmd1(m, SW_CMD_READ_CURRENT);
}

void SteadyWin_ReadSpeed(SteadyWin_Motor *m)
{
    sw_send_cmd1(m, SW_CMD_READ_SPEED);
}

void SteadyWin_ReadAngle(SteadyWin_Motor *m)
{
    sw_send_cmd1(m, SW_CMD_READ_ANGLE);
}

void SteadyWin_ReadMulti(SteadyWin_Motor *m)
{
    sw_send_cmd1(m, SW_CMD_READ_MULTI);
}

void SteadyWin_ReadMotorInfo(SteadyWin_Motor *m)
{
    sw_send_cmd1(m, SW_CMD_READ_MOTOR_INFO);
}

/* ---------- Parameter commands ---------- */

void SteadyWin_SetZero(SteadyWin_Motor *m)
{
    sw_send_cmd1(m, SW_CMD_SET_ZERO);
}

void SteadyWin_SetMaxSpeed(SteadyWin_Motor *m, uint32_t speed_001rpm)
{
    sw_send_cmd5_u32(m, SW_CMD_SET_MAX_SPEED, speed_001rpm);
}

void SteadyWin_SetMaxCurrent(SteadyWin_Motor *m, uint32_t current_001a)
{
    sw_send_cmd5_u32(m, SW_CMD_SET_MAX_CURRENT, current_001a);
}

void SteadyWin_SetCurrentSlope(SteadyWin_Motor *m, uint32_t slope_001a_s)
{
    sw_send_cmd5_u32(m, SW_CMD_SET_CURRENT_SLOPE, slope_001a_s);
}

void SteadyWin_SetAccel(SteadyWin_Motor *m, uint32_t accel_001rpm_s)
{
    sw_send_cmd5_u32(m, SW_CMD_SET_ACCEL, accel_001rpm_s);
}

/* ---------- Control commands ---------- */

void SteadyWin_CurrentControl(SteadyWin_Motor *m, int32_t current_001a)
{
    sw_send_cmd5_s32(m, SW_CMD_CURRENT_CTRL, current_001a);
}

void SteadyWin_SpeedControl(SteadyWin_Motor *m, int32_t speed_001rpm)
{
    sw_send_cmd5_s32(m, SW_CMD_SPEED_CTRL, speed_001rpm);
}

void SteadyWin_AbsPosControl(SteadyWin_Motor *m, int32_t position_count)
{
    sw_send_cmd5_s32(m, SW_CMD_ABS_POS_CTRL, position_count);
}

void SteadyWin_RelPosControl(SteadyWin_Motor *m, int32_t position_count)
{
    sw_send_cmd5_s32(m, SW_CMD_REL_POS_CTRL, position_count);
}

void SteadyWin_GotoOrigin(SteadyWin_Motor *m)
{
    sw_send_cmd1(m, SW_CMD_GOTO_ORIGIN);
}

void SteadyWin_BrakeControl(SteadyWin_Motor *m, uint8_t action)
{
    uint8_t txdata[8] = {0};
    txdata[0] = SW_CMD_BRAKE_CTRL;
    txdata[1] = action;
    sw_send_std(m, 0x100 | m->dev_addr, txdata, sw_dlc(2));
}

/* ---------- Encoder helpers ---------- */

/* Motor encoder: single-turn absolute angle in degrees (0xA3 [1]-[2]) */
float SteadyWin_SingleAngleDeg(const SteadyWin_Motor *m)
{
    return (float)m->fb.single_angle_raw * (360.0f / 16384.0f);
}

/* Multi-turn accumulated angle in degrees (0xA3 [3]-[6]) */
float SteadyWin_MultiAngleDeg(const SteadyWin_Motor *m)
{
    return (float)m->fb.multi_angle_raw * (360.0f / 16384.0f);
}

/* Output shaft angle in degrees = multi-turn / gear ratio */
float SteadyWin_OutputShaftDeg(const SteadyWin_Motor *m)
{
    if (m->gear_ratio == 0) return 0.0f;
    return SteadyWin_MultiAngleDeg(m) / (float)m->gear_ratio;
}
