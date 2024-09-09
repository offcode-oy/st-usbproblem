/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32h7rsxx_hal.h"

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
#define USB_SW_HS_SEL_Pin GPIO_PIN_3
#define USB_SW_HS_SEL_GPIO_Port GPIOM
#define DISP1_SCK_Pin GPIO_PIN_3
#define DISP1_SCK_GPIO_Port GPIOD
#define LED2_Pin GPIO_PIN_14
#define LED2_GPIO_Port GPIOE
#define SDA_Pin GPIO_PIN_0
#define SDA_GPIO_Port GPIOF
#define USB_SW_EN_N_Pin GPIO_PIN_2
#define USB_SW_EN_N_GPIO_Port GPIOM
#define LED1_Pin GPIO_PIN_13
#define LED1_GPIO_Port GPIOE
#define DISP1_CS_Pin GPIO_PIN_9
#define DISP1_CS_GPIO_Port GPIOB
#define SCL_Pin GPIO_PIN_1
#define SCL_GPIO_Port GPIOF
#define DISP_BL_PWM_Pin GPIO_PIN_11
#define DISP_BL_PWM_GPIO_Port GPIOE
#define DISP1_DC_Pin GPIO_PIN_0
#define DISP1_DC_GPIO_Port GPIOC
#define DISP1_MOSI_Pin GPIO_PIN_1
#define DISP1_MOSI_GPIO_Port GPIOC
#define DISP1_RESET_Pin GPIO_PIN_2
#define DISP1_RESET_GPIO_Port GPIOC
#define BUTTON3_Pin GPIO_PIN_12
#define BUTTON3_GPIO_Port GPIOB
#define QSPI_IO2_Pin GPIO_PIN_2
#define QSPI_IO2_GPIO_Port GPIOP
#define QSPI_IO3_Pin GPIO_PIN_3
#define QSPI_IO3_GPIO_Port GPIOP
#define QSPI_MOSI_IO0_Pin GPIO_PIN_0
#define QSPI_MOSI_IO0_GPIO_Port GPIOP
#define DISP2_CS_Pin GPIO_PIN_0
#define DISP2_CS_GPIO_Port GPIOO
#define BUTTON2_Pin GPIO_PIN_11
#define BUTTON2_GPIO_Port GPIOB
#define BUTTON4_Pin GPIO_PIN_13
#define BUTTON4_GPIO_Port GPIOB
#define QSPI_IO1_Pin GPIO_PIN_1
#define QSPI_IO1_GPIO_Port GPIOP
#define QSPI_SCLK_Pin GPIO_PIN_4
#define QSPI_SCLK_GPIO_Port GPIOO

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
