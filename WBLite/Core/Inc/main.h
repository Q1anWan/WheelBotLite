/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

#include "stm32h7xx_ll_dma.h"
#include "stm32h7xx_ll_opamp.h"
#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_crs.h"
#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_system.h"
#include "stm32h7xx_ll_exti.h"
#include "stm32h7xx_ll_cortex.h"
#include "stm32h7xx_ll_utils.h"
#include "stm32h7xx_ll_pwr.h"
#include "stm32h7xx_ll_tim.h"
#include "stm32h7xx_ll_usart.h"
#include "stm32h7xx_ll_gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "DL_H750.h"
#include "cmsis_compiler.h"
#include "tx_thread_sleep_until.h"
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
#define USART2_DE_Pin LL_GPIO_PIN_1
#define USART2_DE_GPIO_Port GPIOA
#define EN_5V_Pin LL_GPIO_PIN_10
#define EN_5V_GPIO_Port GPIOE
#define KEYT_Pin LL_GPIO_PIN_14
#define KEYT_GPIO_Port GPIOE
#define USART3_DE_Pin LL_GPIO_PIN_12
#define USART3_DE_GPIO_Port GPIOD
#define LED_G_Pin LL_GPIO_PIN_14
#define LED_G_GPIO_Port GPIOD
#define LED_B_Pin LL_GPIO_PIN_15
#define LED_B_GPIO_Port GPIOD
#define BUZZER_Pin LL_GPIO_PIN_6
#define BUZZER_GPIO_Port GPIOC
#define LED_R_Pin LL_GPIO_PIN_7
#define LED_R_GPIO_Port GPIOC
#define IMU_CS_GYRO_Pin LL_GPIO_PIN_9
#define IMU_CS_GYRO_GPIO_Port GPIOB
#define IMU_INT_GYRO_Pin LL_GPIO_PIN_0
#define IMU_INT_GYRO_GPIO_Port GPIOE
#define IMU_INT_GYRO_EXTI_IRQn EXTI0_IRQn
#define IMU_INT_ACCEL_Pin LL_GPIO_PIN_1
#define IMU_INT_ACCEL_GPIO_Port GPIOE
#define IMU_INT_ACCEL_EXTI_IRQn EXTI1_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
