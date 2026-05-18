#ifndef _CONTROL_H_
#define _CONTROL_H_

#include <stdint.h>

typedef struct
{
    float kp;
    float ki;
    float kd;

    float ref;
    float fdb;
    float err;
    float last_err;

    float integral;
    float derivative;

    float integral_max;
    float integral_min;

    float out;
    float out_max;
    float out_min;

    float Ts;

    uint8_t enable;


}Ctrl_t;

extern Ctrl_t IqLoop;
extern Ctrl_t IdLoop;
extern Ctrl_t SpdLoop;
extern Ctrl_t PosLoop;

typedef struct
{
    float pos_ref;
    float vel_ref;
    float kp;
    float kd;
    float torque_ref;

    float pos_fdb;
    float vel_fdb;

    float torque_out;
    float torque_max;
    float torque_min;

    uint8_t enable;


}MITMode_t;

extern MITMode_t MITMode;

void CurrLoop_Reset(void);
void CurrLoop_Run(float RealQ, float RealD, float ExptQ, float ExptD, float *Volq, float *Vold);
void SpdLoop_Run(void);
void PosLoop_Run(void);
void MITModeLoop_Run(void);

#endif
