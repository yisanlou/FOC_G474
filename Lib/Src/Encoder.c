#include "Encoder.h"
#include "Control.h"
#include "spi.h"

#include "FOC.h"
#include "MotorCofig.h"
#include "arm_math.h"


uint8_t ContinusRead[2] = {0xA0, 0x03};
uint8_t SPI_Data[4];
uint8_t Speed_CNT = 0;
uint32_t Speed_Raw[2];
volatile uint8_t Angle_Ready = 0;

Encoder_t Encoder_Value = 
{
    .Offset = 281047 
};


void Start_Angle_Read(void)
{
    if (hspi3.State != HAL_SPI_STATE_READY) {
        return;
    }

    HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_RESET);

    if (HAL_SPI_Transmit_DMA(&hspi3, ContinusRead, 2) != HAL_OK) {
        HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);
    }
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI3) {
        if (HAL_SPI_Receive_DMA(&hspi3, SPI_Data, 4) != HAL_OK) {
            HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);
        }
    }
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    if (hspi->Instance == SPI3) {
        HAL_GPIO_WritePin(SPI3_CS_GPIO_Port, SPI3_CS_Pin, GPIO_PIN_SET);
        /* 这里解析 Encoder_Data */
        Encoder_Value.Raw_Num =((uint32_t)SPI_Data[0] << 13) | ((uint32_t)SPI_Data[1] << 5)  | ((uint32_t)SPI_Data[2] >> 3);
        Encoder_GetElectricalRad(Encoder_Value.Raw_Num - Encoder_Value.Offset );

        Angle_Ready = 1;
    }
}

void Encoder_GetElectricalRad(uint32_t encoder_raw)
{
    uint32_t mech_raw;


    mech_raw = encoder_raw & ENC_MASK;

    /*
     * 机械角度 × 极对数
     * 然后对 2^21 取模，相当于电角度限制在 0~2π
     */
    Encoder_Value.elec_raw = ((uint64_t)mech_raw * Motor->Pole_pairs) & ENC_MASK;


    Encoder_Value.Elec_Angle_pi = (Encoder_Value.elec_raw * TWO_PI )/ (float)ENC_RES ;

    FOC_State.Etheta = Encoder_Value.Elec_Angle_pi ;
    FOC_State.cosVal = arm_cos_f32(FOC_State.Etheta);
    FOC_State.sinVal = arm_sin_f32(FOC_State.Etheta);
}


void Encoder_CalcSpeed(float Ts)
{
    static uint32_t last_raw = 0;
    static uint8_t init = 0;
    static float vel_rpm_filt = 0.0f;
    uint32_t raw_now = Encoder_Value.Raw_Num;
    int32_t delta;
    float vel_rpm_raw;

    if (Ts <= 0.0f)
    {
        vel_rpm_filt = 0.0f;
        Real.Mech_Vel_RPM = 0.0f;
        return;
    }

    if (init == 0U)
    {
        last_raw = raw_now;
        init = 1U;
        vel_rpm_filt = 0.0f;
        Real.Mech_Vel_RPM = 0.0f;
        return;
    }

    delta = (int32_t)(raw_now - last_raw);

    if (delta > (int32_t)(ENC_RES / 2U))
    {
        delta -= (int32_t)ENC_RES;
    }
    else if (delta < -(int32_t)(ENC_RES / 2U))
    {
        delta += (int32_t)ENC_RES;
    }

    last_raw = raw_now;

    vel_rpm_raw = ((float)delta * 60.0f) / ((float)ENC_RES * Ts);
    vel_rpm_filt += 0.9f * (vel_rpm_raw - vel_rpm_filt);
    Real.Mech_Vel_RPM = vel_rpm_filt;
}
