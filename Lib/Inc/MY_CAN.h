#ifndef _MY_CAN_H_
#define _MY_CAN_H_

#include "main.h"
#include "fdcan.h"
#include "stm32g4xx_hal_fdcan.h"



extern FDCAN_TxHeaderTypeDef TxHeader;
extern FDCAN_RxHeaderTypeDef RxHeader;
extern uint8_t CAN_TxData[8];
extern uint8_t CAN_RxData[8];

void FDCAN2_Config(void);



#endif