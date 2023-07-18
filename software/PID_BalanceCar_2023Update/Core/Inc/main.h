/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sys.h"
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
#define encoder_A_Pin GPIO_PIN_0
#define encoder_A_GPIO_Port GPIOA
#define encoder_B_Pin GPIO_PIN_1
#define encoder_B_GPIO_Port GPIOA
#define LED_Pin GPIO_PIN_4
#define LED_GPIO_Port GPIOA
#define KEY_Pin GPIO_PIN_5
#define KEY_GPIO_Port GPIOA
#define ADC_Pin GPIO_PIN_6
#define ADC_GPIO_Port GPIOA
#define ultrasonic_cap_Pin GPIO_PIN_0
#define ultrasonic_cap_GPIO_Port GPIOB
#define ultrasonic_Pin GPIO_PIN_1
#define ultrasonic_GPIO_Port GPIOB
#define usart3_tx_Pin GPIO_PIN_10
#define usart3_tx_GPIO_Port GPIOB
#define usart3_rx_Pin GPIO_PIN_11
#define usart3_rx_GPIO_Port GPIOB
#define BIN2_Pin GPIO_PIN_12
#define BIN2_GPIO_Port GPIOB
#define BIN1_Pin GPIO_PIN_13
#define BIN1_GPIO_Port GPIOB
#define AIN1_Pin GPIO_PIN_14
#define AIN1_GPIO_Port GPIOB
#define AIN2_Pin GPIO_PIN_15
#define AIN2_GPIO_Port GPIOB
#define PWMA_Pin GPIO_PIN_8
#define PWMA_GPIO_Port GPIOA
#define usart1_tx_Pin GPIO_PIN_9
#define usart1_tx_GPIO_Port GPIOA
#define usart1_rx_Pin GPIO_PIN_10
#define usart1_rx_GPIO_Port GPIOA
#define PWMB_Pin GPIO_PIN_11
#define PWMB_GPIO_Port GPIOA
#define MPU6050_EXTI_Pin GPIO_PIN_12
#define MPU6050_EXTI_GPIO_Port GPIOA
#define MPU6050_EXTI_EXTI_IRQn EXTI15_10_IRQn
#define OLED_PA15_Pin GPIO_PIN_15
#define OLED_PA15_GPIO_Port GPIOA
#define OLED_PB3_Pin GPIO_PIN_3
#define OLED_PB3_GPIO_Port GPIOB
#define OLED_PB4_Pin GPIO_PIN_4
#define OLED_PB4_GPIO_Port GPIOB
#define OLED_PB5_Pin GPIO_PIN_5
#define OLED_PB5_GPIO_Port GPIOB
#define encoder_C_Pin GPIO_PIN_6
#define encoder_C_GPIO_Port GPIOB
#define encoder_D_Pin GPIO_PIN_7
#define encoder_D_GPIO_Port GPIOB
#define IIC_SCL_Pin GPIO_PIN_8
#define IIC_SCL_GPIO_Port GPIOB
#define IIC_SDA_Pin GPIO_PIN_9
#define IIC_SDA_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
