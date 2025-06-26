/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define acc_cs_Pin GPIO_PIN_3
#define acc_cs_GPIO_Port GPIOE
#define servo_pwm_Pin GPIO_PIN_5
#define servo_pwm_GPIO_Port GPIOE
#define espcam_miso_Pin GPIO_PIN_2
#define espcam_miso_GPIO_Port GPIOC
#define espcam_mosi_Pin GPIO_PIN_3
#define espcam_mosi_GPIO_Port GPIOC
#define enc4_a_Pin GPIO_PIN_0
#define enc4_a_GPIO_Port GPIOA
#define enc4_b_Pin GPIO_PIN_1
#define enc4_b_GPIO_Port GPIOA
#define esp_uart_tx_Pin GPIO_PIN_2
#define esp_uart_tx_GPIO_Port GPIOA
#define esp_uart_rx_Pin GPIO_PIN_3
#define esp_uart_rx_GPIO_Port GPIOA
#define gyro_sck_Pin GPIO_PIN_5
#define gyro_sck_GPIO_Port GPIOA
#define gyro_miso_Pin GPIO_PIN_6
#define gyro_miso_GPIO_Port GPIOA
#define gyro_mosi_Pin GPIO_PIN_7
#define gyro_mosi_GPIO_Port GPIOA
#define stdby1_Pin GPIO_PIN_0
#define stdby1_GPIO_Port GPIOB
#define stdby2_Pin GPIO_PIN_1
#define stdby2_GPIO_Port GPIOB
#define pwm1_Pin GPIO_PIN_9
#define pwm1_GPIO_Port GPIOE
#define pwm2_Pin GPIO_PIN_11
#define pwm2_GPIO_Port GPIOE
#define pwm3_Pin GPIO_PIN_13
#define pwm3_GPIO_Port GPIOE
#define pwm4_Pin GPIO_PIN_14
#define pwm4_GPIO_Port GPIOE
#define espcam_nss_Pin GPIO_PIN_12
#define espcam_nss_GPIO_Port GPIOB
#define espcam_sck_Pin GPIO_PIN_13
#define espcam_sck_GPIO_Port GPIOB
#define motor1_in1_Pin GPIO_PIN_14
#define motor1_in1_GPIO_Port GPIOB
#define motor1_in2_Pin GPIO_PIN_15
#define motor1_in2_GPIO_Port GPIOB
#define motor2_in1_Pin GPIO_PIN_8
#define motor2_in1_GPIO_Port GPIOD
#define motor2_in2_Pin GPIO_PIN_9
#define motor2_in2_GPIO_Port GPIOD
#define motor3_in1_Pin GPIO_PIN_10
#define motor3_in1_GPIO_Port GPIOD
#define motor3_in2_Pin GPIO_PIN_11
#define motor3_in2_GPIO_Port GPIOD
#define enc3_a_Pin GPIO_PIN_12
#define enc3_a_GPIO_Port GPIOD
#define enc3_b_Pin GPIO_PIN_13
#define enc3_b_GPIO_Port GPIOD
#define motor4_in1_Pin GPIO_PIN_14
#define motor4_in1_GPIO_Port GPIOD
#define motor4_in2_Pin GPIO_PIN_15
#define motor4_in2_GPIO_Port GPIOD
#define enc1_a_Pin GPIO_PIN_15
#define enc1_a_GPIO_Port GPIOA
#define enc1_b_Pin GPIO_PIN_3
#define enc1_b_GPIO_Port GPIOB
#define enc2_a_Pin GPIO_PIN_4
#define enc2_a_GPIO_Port GPIOB
#define enc2_b_Pin GPIO_PIN_5
#define enc2_b_GPIO_Port GPIOB
#define acc_scl_Pin GPIO_PIN_6
#define acc_scl_GPIO_Port GPIOB
#define acc_sda_Pin GPIO_PIN_7
#define acc_sda_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
