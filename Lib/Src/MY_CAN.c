#include "MY_CAN.h"
#include "fdcan.h"
#include "stm32g4xx_hal_fdcan.h"
#include "Control.h"
#include "FOC.h"

FDCAN_TxHeaderTypeDef TxHeader;
FDCAN_RxHeaderTypeDef RxHeader;
uint8_t CAN_TxData[8] = {108,111,118,101,102,103,104,105};
uint8_t CAN_RxData[8];

#define CAN_POS_SCALE_RAD     1.0f
#define CAN_VEL_SCALE_RPM     10.0f
#define CAN_IQ_SCALE_A        1000.0f
#define CAN_CMD_POS_MIN_RAD   -100.0f
#define CAN_CMD_POS_MAX_RAD   100.0f
#define CAN_CMD_VEL_MIN_RAD_S -65.0f
#define CAN_CMD_VEL_MAX_RAD_S 65.0f
#define CAN_CMD_TOR_MIN_NM    -5.0f
#define CAN_CMD_TOR_MAX_NM    5.0f
#define CAN_CMD_KP_MIN        0.0f
#define CAN_CMD_KP_MAX        500.0f
#define CAN_CMD_KD_MIN        0.0f
#define CAN_CMD_KD_MAX        5.0f

static int16_t CAN_FloatToInt16(float value, float scale)
{
    float scaled = value * scale;

    if (scaled > 32767.0f)
    {
        return 32767;
    }
    if (scaled < -32768.0f)
    {
        return -32768;
    }

    if (scaled >= 0.0f)
    {
        return (int16_t)(scaled + 0.5f);
    }

    return (int16_t)(scaled - 0.5f);
}

static void CAN_PackInt16BE(uint8_t *data, uint8_t index, int16_t value)
{
    uint16_t raw = (uint16_t)value;

    data[index] = (uint8_t)(raw >> 8);
    data[index + 1U] = (uint8_t)(raw & 0x00FFU);
}

static float CAN_UintToFloat(uint16_t value, float min, float max, uint8_t bits)
{
    float span = max - min;
    float raw_max = (float)((1UL << bits) - 1UL);

    return ((float)value * span / raw_max) + min;
}

static void CAN_UnpackMITCommand(const uint8_t *data)
{
    uint16_t kp_raw;
    uint16_t kd_raw;
    uint16_t pos_raw;
    uint16_t vel_raw;
    uint16_t torque_raw;

    kp_raw = ((uint16_t)data[0] << 4) | ((uint16_t)data[1] >> 4);
    kd_raw = (((uint16_t)data[1] & 0x000FU) << 8) | (uint16_t)data[2];
    pos_raw = ((uint16_t)data[3] << 8) | (uint16_t)data[4];
    vel_raw = ((uint16_t)data[5] << 4) | ((uint16_t)data[6] >> 4);
    torque_raw = (((uint16_t)data[6] & 0x000FU) << 8) | (uint16_t)data[7];

    MITMode.kp = CAN_UintToFloat(kp_raw, CAN_CMD_KP_MIN, CAN_CMD_KP_MAX, 12U);
    MITMode.kd = CAN_UintToFloat(kd_raw, CAN_CMD_KD_MIN, CAN_CMD_KD_MAX, 12U);
    MITMode.pos_ref = CAN_UintToFloat(pos_raw, CAN_CMD_POS_MIN_RAD, CAN_CMD_POS_MAX_RAD, 16U);
    MITMode.vel_ref = CAN_UintToFloat(vel_raw, CAN_CMD_VEL_MIN_RAD_S, CAN_CMD_VEL_MAX_RAD_S, 12U);
    MITMode.torque_ref = CAN_UintToFloat(torque_raw, CAN_CMD_TOR_MIN_NM, CAN_CMD_TOR_MAX_NM, 12U);
}



void FDCAN2_Config(void)
{
  FDCAN_FilterTypeDef sFilterConfig;
  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0x011;
  sFilterConfig.FilterID2 = 0x7FF;
  if(HAL_FDCAN_ConfigFilter(&hfdcan2, &sFilterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2,
                                 FDCAN_REJECT,
                                 FDCAN_REJECT,
                                 FDCAN_REJECT_REMOTE,
                                 FDCAN_REJECT_REMOTE) != HAL_OK)
    {
    Error_Handler();
    }
  if(HAL_FDCAN_Start(&hfdcan2) != HAL_OK)
  {
    Error_Handler();
  }
  if(HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
  {
    Error_Handler();
  }

  TxHeader.Identifier = 0x101;
  TxHeader.IdType = FDCAN_STANDARD_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;
  TxHeader.DataLength = FDCAN_DLC_BYTES_8;
  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0;
}


void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    int16_t pos;
    int16_t vel;
    int16_t iq;

    if (hfdcan->Instance != FDCAN2)
    {
        return;
    }

    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U)
    {
        return;
    }

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, CAN_RxData) != HAL_OK)
    {
        return;
    }

    CAN_UnpackMITCommand(CAN_RxData);

    pos = CAN_FloatToInt16(Real.Mech_Pos, CAN_POS_SCALE_RAD);
    vel = CAN_FloatToInt16(Real.Mech_Vel_RPM, CAN_VEL_SCALE_RPM);
    iq = CAN_FloatToInt16(FOC_State.Iq, CAN_IQ_SCALE_A);

    CAN_PackInt16BE(CAN_TxData, 0U, pos);
    CAN_PackInt16BE(CAN_TxData, 2U, vel);
    CAN_PackInt16BE(CAN_TxData, 4U, iq);
    CAN_TxData[6] = 0U;
    CAN_TxData[7] = 0U;

    HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, CAN_TxData);
}


