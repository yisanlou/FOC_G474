#ifndef _FOC_H_
#define _FOC_H_

#include <stdint.h>

#define SQRT3         1.73205080757f
#define ONE_BY_SQRT3  0.57735026919f
#define TWO_BY_SQRT3  0.86602540378f

#define PWM_ARR       17000U
#define SVPWM_PERIOD  (2.0f * (float)PWM_ARR)
#define FOC_VDC_MIN    1.0f



typedef struct
{
    float Ialpha;
    float Ibeta;
    float Etheta;
    float sinVal;
    float cosVal;
    float Id;
    float Iq;
}FOC_State_t;



extern FOC_State_t FOC_State;
extern uint16_t CMPU;
extern uint16_t CMPV;
extern uint16_t CMPW;


static inline float OutputLimitation(float max, float min, float value)
{
    if (value > max) return max;
    if (value < min) return min;
    return value;
}

static inline void INVERSEPARK(float VolQ, float VolD, float cosVal, float sinVal, float *pValpha, float *pVbeta)
{
    *pValpha = VolD * cosVal - VolQ * sinVal;
    *pVbeta  = VolD * sinVal + VolQ * cosVal;
}

static inline void CLARKE(float Ia, float Ib, float Ic, float *pIalpha, float *pIbeta)
{
    *pIalpha = Ia;
    *pIbeta  = ONE_BY_SQRT3 * (Ib - Ic);
}

static inline void ParkTransform(float Ialpha, float Ibeta,float sinVal, float cosVal, float *pId, float *pIq)
{
    *pId = Ialpha * cosVal + Ibeta * sinVal;
    *pIq = -Ialpha * sinVal + Ibeta * cosVal;
}

void SVPWM(float Valpha, float Vbeta, float Vdc);


#endif
