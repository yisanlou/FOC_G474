#include "FOC.h"

FOC_State_t FOC_State =
{
    .Ialpha = 0.0f,
    .Ibeta = 0.0f,
    .Etheta = 0.0f,
    .sinVal = 0.0f,
    .cosVal = 0.0f,
    .Id = 0.0f,
    .Iq = 0.0f,

};


uint16_t CMPU = 0;
uint16_t CMPV = 0;
uint16_t CMPW = 0;

static inline uint16_t SVPWM_ClampCompare(float compare)
{
    if (compare != compare)
    {
        return PWM_ARR / 2U;
    }

    if (compare <= 0.0f)
    {
        return 0;
    }

    if (compare >= (float)PWM_ARR)
    {
        return PWM_ARR;
    }

    return (uint16_t)(compare + 0.5f);
}

static inline void SVPWM_SetNeutralCompare(void)
{
    CMPU = PWM_ARR / 2U;
    CMPV = PWM_ARR / 2U;
    CMPW = PWM_ARR / 2U;
}

void SVPWM(float Valpha, float Vbeta, float Vdc)
{
    float Tfirst;
    float Tsecond;
    float Ta;
    float Tb;
    float Tc;
    uint8_t SectorN = 0U;
    const float Vref2 = TWO_BY_SQRT3 * Valpha - 0.5f * Vbeta;
    const float Vref3 = -TWO_BY_SQRT3 * Valpha - 0.5f * Vbeta;
    float K;

    if (!(Vdc > FOC_VDC_MIN))
    {
        SVPWM_SetNeutralCompare();
        return;
    }

    if(Vbeta > 0.0f) SectorN |= 1U;
    if(Vref2 > 0.0f) SectorN |= 2U;
    if(Vref3 > 0.0f) SectorN |= 4U;

    K = SQRT3 * SVPWM_PERIOD / Vdc;

    switch(SectorN)
    {
    case 1:
        Tfirst = K * (Vref3 + Vbeta);
        Tsecond = K * (Vref2 + Vbeta);
        break;
    case 2:
        Tfirst = K * (Vref2 + Vbeta);
        Tsecond = -K * Vbeta;
        break;
    case 3:
        Tfirst = -K * (Vref3 + Vbeta);
        Tsecond = K * Vbeta;
        break;
    case 4:
        Tfirst = -K * Vbeta;
        Tsecond = K * (Vref3 + Vbeta);
        break;
    case 5:
        Tfirst = K * Vbeta;
        Tsecond = -K * (Vref2 + Vbeta);
        break;
    case 6:
        Tfirst = -K * (Vref2 + Vbeta);
        Tsecond = -K * (Vref3 + Vbeta);
        break;
    default:
        SVPWM_SetNeutralCompare();
        return;
    }

    float Tsum = Tfirst + Tsecond;
    if(Tsum > SVPWM_PERIOD)
    {
        float AntiTall = 1.0f / Tsum;
        Tfirst = Tfirst * SVPWM_PERIOD * AntiTall;
        Tsecond = Tsecond * SVPWM_PERIOD * AntiTall;
    }

    Ta = (SVPWM_PERIOD - Tfirst - Tsecond) * 0.25f;
    Tb = Ta + Tfirst * 0.5f;
    Tc = Tb + Tsecond * 0.5f;

    switch(SectorN)
    {
    case 1:
        CMPU = SVPWM_ClampCompare(Tb);
        CMPV = SVPWM_ClampCompare(Ta);
        CMPW = SVPWM_ClampCompare(Tc);
        break;
    case 2:
        CMPU = SVPWM_ClampCompare(Ta);
        CMPV = SVPWM_ClampCompare(Tc);
        CMPW = SVPWM_ClampCompare(Tb);
        break;
    case 3:
        CMPU = SVPWM_ClampCompare(Ta);
        CMPV = SVPWM_ClampCompare(Tb);
        CMPW = SVPWM_ClampCompare(Tc);
        break;
    case 4:
        CMPU = SVPWM_ClampCompare(Tc);
        CMPV = SVPWM_ClampCompare(Tb);
        CMPW = SVPWM_ClampCompare(Ta);
        break;
    case 5:
        CMPU = SVPWM_ClampCompare(Tc);
        CMPV = SVPWM_ClampCompare(Ta);
        CMPW = SVPWM_ClampCompare(Tb);
        break;
    case 6:
        CMPU = SVPWM_ClampCompare(Tb);
        CMPV = SVPWM_ClampCompare(Tc);
        CMPW = SVPWM_ClampCompare(Ta);
        break;
    default:
        SVPWM_SetNeutralCompare();
        break;
    }

}
