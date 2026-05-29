#include "FOC.h"
#include "MY_ADC.h"
#include "arm_math.h"
#include "hrtim.h"
#include "main.h"

#define OPENLOOP_DEFAULT_VBUS 48.0f
#define OPENLOOP_TWO_PI       6.28318530718f

float Theta = 0.0f;

static inline float Openloop_GetVbus(void)
{
    if (ADC_Value.Vol_Bus > FOC_VDC_MIN)
    {
        return ADC_Value.Vol_Bus;
    }

    return OPENLOOP_DEFAULT_VBUS;
}

void OpenloopVol(float Volq, float Spd, float Poles)
{
    float Valpha;
    float Vbeta;
    float theta_step;

    if (Poles <= 0.0f)
    {
        SVPWM(0.0f, 0.0f, Openloop_GetVbus());
        return;
    }

    theta_step = Spd * Poles / (float)PWM_FREQ;
    Theta = Theta + theta_step;

    while (Theta >= OPENLOOP_TWO_PI)
    {
        Theta = Theta - OPENLOOP_TWO_PI;
    }

    while (Theta < 0.0f)
    {
        Theta = Theta + OPENLOOP_TWO_PI;
    }

    FOC_State.Etheta = Theta;
    FOC_State.cosVal = arm_cos_f32(Theta);
    FOC_State.sinVal = arm_sin_f32(Theta);

    INVERSEPARK(Volq, 0.0f, FOC_State.cosVal, FOC_State.sinVal, &Valpha, &Vbeta);
    SVPWM(Valpha, Vbeta, Openloop_GetVbus());

    HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR = CMPU;
    HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].CMP1xR = CMPV;
    HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_E].CMP1xR = CMPW;
}

