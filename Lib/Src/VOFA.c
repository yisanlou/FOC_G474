#include "VOFA.h"
#include "stm32g4xx_hal_uart.h"
#include "usart.h"
#include <string.h>

#include "FOC.h"
#include "Control.h"
#include "MY_ADC.h"
#include "Encoder.h"


uint8_t UART_GetData[2];
volatile float Expt_Spd_Now = 0.0f;
volatile float Spd_Step = 0.0f;
volatile float Spd_Ramp_Rate_RPM_S = 0.0f;

#define SPD_RAMP_TIME_S 0.005f

void Vofa_JustFloat(float *_data, uint8_t _num)
{
    static uint8_t tempData[100];
    uint8_t temp_end[4] = {0x00, 0x00, 0x80, 0x7F};

    // 参数检查，防止空指针
    if (_data == NULL)
    {
        return;
    }

    // tempData 一共 100 字节
    // 每个 float 4 字节，帧尾 4 字节
    // 所以最多发送 24 个 float
    if (_num > 24)
    {
        return;
    }

    if (huart2.gState != HAL_UART_STATE_READY)
    {
        return;
    }

    // 复制 float 数据到字节数组
    memcpy(tempData, (uint8_t *)_data, sizeof(float) * _num);

    // 添加 VOFA+ JustFloat 帧尾：00 00 80 7F
    memcpy(&tempData[_num * 4], temp_end, 4);

    // 通过串口 DMA 发送
    (void)HAL_UART_Transmit_DMA(&huart2, tempData, (_num + 1) * 4);
}

void Vofa_Send(uint8_t num)
{
    static float Vofa_Buffer[8];

    Vofa_Buffer[0] = FOC_State.Id;
    Vofa_Buffer[1] = FOC_State.Iq;
    Vofa_Buffer[2] = Real.Mech_Vel_RPM;
    Vofa_Buffer[3] = Expt.Mech_Vel_RPM;
    Vofa_Buffer[4] = Expt.Iq;
    Vofa_Buffer[5] = ADC_Value.Curr_A + ADC_Value.Curr_B + ADC_Value.Curr_C;
    Vofa_Buffer[6] = ADC_Value.Curr_A;
    Vofa_Buffer[7] = ADC_Value.Curr_B;

    if (num > 8)
    {
        num = 8;
    }

    Vofa_JustFloat(Vofa_Buffer, num);

}

void UART_Send(const char *str)
{
    if (str == NULL)
    {
        return;
    }

    HAL_UART_Transmit(&huart2, (uint8_t *)str, strlen(str), 10);
}

void UART_Receive(void)
{
    HAL_UART_Receive_IT(&huart2, UART_GetData, 2);

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart-> Instance== USART2)
    {
        float expt_spd_last = Expt.Mech_Vel_RPM;
        int16_t spd_cmd;

        spd_cmd = (int16_t)(((uint16_t)UART_GetData[0] << 8) | UART_GetData[1]);
        Expt_Spd_Now = (float)spd_cmd;
        Expt.Mech_Vel_RPM = Expt_Spd_Now;
        Spd_Step = (Expt_Spd_Now - expt_spd_last) * SpdLoop.Ts / SPD_RAMP_TIME_S;
        Spd_Ramp_Rate_RPM_S = (Expt_Spd_Now - expt_spd_last) / SPD_RAMP_TIME_S;

        HAL_UART_Receive_IT(&huart2, UART_GetData, 2);

    }
}
