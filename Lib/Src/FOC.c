#include "FOC.h"
#include "hrtim.h"
#include "MotorCofig.h"
#include "MY_ADC.h"

#define FOC_DEFAULT_VBUS 48.0f

FOC_State_t FOC_State =
{
    .Ialpha = 0.0f,
    .Ibeta = 0.0f,
    .Ualpha = 0.0f,
    .Ubeta = 0.0f,
    .Etheta = 0.0f,
    .sinVal = 0.0f,
    .cosVal = 0.0f,
    .Vol_Q = 0.0f,
    .Vol_D = 0.0f,
    .Id = 0.0f,
    .Iq = 0.0f,
    .Vel = 0.0f

};


uint16_t CMPU = 0;
uint16_t CMPV = 0;
uint16_t CMPW = 0;

uint8_t  Sector = 0;

static inline uint16_t SVPWM_ClampCompare(float compare)
{
    uint16_t cmp;

    if (compare != compare)
    {
        return PWM_ARR / 2U;
    }

    if (compare <= 0.0f)
    {
        cmp = 0;
    }
    else if (compare >= (float)PWM_ARR)
    {
        cmp = PWM_ARR;
    }
    else
    {
        cmp = (uint16_t)(compare + 0.5f);
    }

    return PWM_ARR - cmp;
}

static inline void SVPWM_SetNeutralCompare(void)
{
    CMPU = PWM_ARR / 2U;
    CMPV = PWM_ARR / 2U;
    CMPW = PWM_ARR / 2U;
}

float FOC_GetVbus(void)
{
    if (ADC_Value.Vol_Bus > FOC_VDC_MIN)
    {
        return ADC_Value.Vol_Bus;
    }

    return FOC_DEFAULT_VBUS;
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
    Sector = SectorN;

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

void FOC_UpdatePwmCompare(void)
{
#if (PHASE_SEQUENCE == PHASE_POSITIVE)
    HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].CMP1xR = CMPU;
    HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR = CMPV;
    HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_E].CMP1xR = CMPW;
#elif (PHASE_SEQUENCE == PHASE_NEGATIVE)
    HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].CMP1xR = CMPU;
    HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR = CMPW;
    HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_E].CMP1xR = CMPV;
#else
#error "Invalid PHASE_SEQUENCE"
#endif
}
