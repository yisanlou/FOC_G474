#include "VOFA.h"
#include "usart.h"
#include <string.h>

#include "FOC.h"

#include "MY_ADC.h"

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
    static float Vofa_Buffer[5];
    Vofa_Buffer[0] = ADC_Value.Curr_A + ADC_Value.Curr_B + ADC_Value.Curr_C;
    Vofa_Buffer[1] = ADC_Value.Curr_B;
    Vofa_Buffer[2] = ADC_Value.Curr_C;
    Vofa_Buffer[3] = 0;
    Vofa_Buffer[4] = 0;

    Vofa_JustFloat(Vofa_Buffer, num);

}
