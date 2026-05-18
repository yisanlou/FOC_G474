/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PWM_FREQ 20000
#define HRTIMR_ARR MASTER_ARR/2
#define MASTER_ARR 680000000/PWM_FREQ
#define ADC_TRIG MASTER_ARR/2
#define LED_0_Pin GPIO_PIN_14
#define LED_0_GPIO_Port GPIOC
#define LED_1_Pin GPIO_PIN_15
#define LED_1_GPIO_Port GPIOC
#define CURR_U_Pin GPIO_PIN_0
#define CURR_U_GPIO_Port GPIOB
#define CURR_V_Pin GPIO_PIN_1
#define CURR_V_GPIO_Port GPIOB
#define CURR_W_Pin GPIO_PIN_2
#define CURR_W_GPIO_Port GPIOB
#define CURR_BUS_Pin GPIO_PIN_14
#define CURR_BUS_GPIO_Port GPIOB
#define VOL_SAMP_Pin GPIO_PIN_15
#define VOL_SAMP_GPIO_Port GPIOB
#define PWM_CL_Pin GPIO_PIN_8
#define PWM_CL_GPIO_Port GPIOC
#define PWM_CH_Pin GPIO_PIN_9
#define PWM_CH_GPIO_Port GPIOC
#define PWM_BL_Pin GPIO_PIN_8
#define PWM_BL_GPIO_Port GPIOA
#define PWM_BH_Pin GPIO_PIN_9
#define PWM_BH_GPIO_Port GPIOA
#define PWM_AL_Pin GPIO_PIN_10
#define PWM_AL_GPIO_Port GPIOA
#define PWM_AH_Pin GPIO_PIN_11
#define PWM_AH_GPIO_Port GPIOA
#define SPI3_CS_Pin GPIO_PIN_15
#define SPI3_CS_GPIO_Port GPIOA
#define RS485_EN_Pin GPIO_PIN_7
#define RS485_EN_GPIO_Port GPIOB
#define RS485_RX_Pin GPIO_PIN_8
#define RS485_RX_GPIO_Port GPIOB
#define RS485_TX_Pin GPIO_PIN_9
#define RS485_TX_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
