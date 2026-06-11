#ifndef _VOFA_H_
#define _VOFA_H_

#include "main.h"

extern uint8_t UART_GetData[2];
extern volatile float Expt_Spd_Now;
extern volatile float Spd_Step;
extern volatile float Spd_Ramp_Rate_RPM_S;

void Vofa_Send(uint8_t num);
void UART_Send(const char *str);
void UART_Receive(void);


#endif
