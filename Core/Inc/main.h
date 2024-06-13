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
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "CB_status_error_flags.h"

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
#define SDC_IN_Pin GPIO_PIN_0
#define SDC_IN_GPIO_Port GPIOA
#define V_FB_AIR_negative_Pin GPIO_PIN_1
#define V_FB_AIR_negative_GPIO_Port GPIOA
#define VCP_TX_Pin GPIO_PIN_2
#define VCP_TX_GPIO_Port GPIOA
#define V_FB_AIR_positive_Pin GPIO_PIN_3
#define V_FB_AIR_positive_GPIO_Port GPIOA
#define V_FB_PC_Relay_Pin GPIO_PIN_4
#define V_FB_PC_Relay_GPIO_Port GPIOA
#define ISO_SPI_SCLK_Pin GPIO_PIN_5
#define ISO_SPI_SCLK_GPIO_Port GPIOA
#define ISO_SPI_MISO_Pin GPIO_PIN_6
#define ISO_SPI_MISO_GPIO_Port GPIOA
#define ISO_SPI_MOSI_Pin GPIO_PIN_7
#define ISO_SPI_MOSI_GPIO_Port GPIOA
#define ISO_SPI_CS2_Pin GPIO_PIN_0
#define ISO_SPI_CS2_GPIO_Port GPIOB
#define ISO_SPI_CS1_Pin GPIO_PIN_1
#define ISO_SPI_CS1_GPIO_Port GPIOB
#define SDC_Out_Pin GPIO_PIN_8
#define SDC_Out_GPIO_Port GPIOA
#define Charge_EN_Pin GPIO_PIN_9
#define Charge_EN_GPIO_Port GPIOA
#define Charger_Con_Pin GPIO_PIN_10
#define Charger_Con_GPIO_Port GPIOA
#define RXCAN_Pin GPIO_PIN_11
#define RXCAN_GPIO_Port GPIOA
#define TXCAN_Pin GPIO_PIN_12
#define TXCAN_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define VCP_RX_Pin GPIO_PIN_15
#define VCP_RX_GPIO_Port GPIOA
#define User_LED_Pin GPIO_PIN_3
#define User_LED_GPIO_Port GPIOB
#define Drive_AIR_positive_Pin GPIO_PIN_4
#define Drive_AIR_positive_GPIO_Port GPIOB
#define Drive_AIR_negative_Pin GPIO_PIN_5
#define Drive_AIR_negative_GPIO_Port GPIOB
#define Drive_Precharge_Relay_Pin GPIO_PIN_6
#define Drive_Precharge_Relay_GPIO_Port GPIOB
#define Precharge_EN_Pin GPIO_PIN_7
#define Precharge_EN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
