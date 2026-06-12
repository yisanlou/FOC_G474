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


typedef struct
{
    float Iq;
    float Id;
    float Mech_Vel_RPM;
    float Mech_Vel_Rad;
    float Mech_Pos;

}Ctrl_Value_t;

extern Ctrl_t IqLoop;
extern Ctrl_t IdLoop;
extern Ctrl_t SpdLoop;
extern Ctrl_t PosLoop;
extern MITMode_t MITMode;
extern Ctrl_Value_t Expt;
extern Ctrl_Value_t Real;

void CurrLoop_Reset(void);
void CurrLoop_Run(float RealQ, float RealD, float ExptQ, float ExptD, float *Volq, float *Vold);
void SpdLoop_Run(float RealVel, float ExptVel, float *ExptQ);
void PosLoop_Run(float RealPos, float ExptPos, float *ExptVel);
void MITModeLoop_Run(float ExptPos, float ExptVel, float ExptTorque, float Kp, float Kd);

#endif
