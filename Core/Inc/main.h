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
#include "stm32h7xx_hal.h"

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
#define SPINDLE_TIMER htim12
#define AXE_Z_TIMER htim15
#define SPEED_TIMER htim16
#define AXE_A_TIMER htim24
#define SYNC_TIMER htim23
#define AXE_Y_TIMER htim5
#define ENC_Z_TIMER htim8
#define AXE_X_TIMER htim2
#define ENC_X_TIMER htim3
#define MASTER_TIMER htim1
#define ENC_Y_TIMER htim4
#define AXE_B_TIMER htim8
#define SPEED_PERIOD 10
#define configRUNTIME_TIMER htim7
#define MASTER_FREQ 150000000
#define panel0_Pin GPIO_PIN_2
#define panel0_GPIO_Port GPIOE
#define limZ_Pin GPIO_PIN_3
#define limZ_GPIO_Port GPIOE
#define dirZ_Pin GPIO_PIN_4
#define dirZ_GPIO_Port GPIOE
#define axe_A_E1_Pin GPIO_PIN_6
#define axe_A_E1_GPIO_Port GPIOF
#define axe__A_E2_Pin GPIO_PIN_7
#define axe__A_E2_GPIO_Port GPIOF
#define dirX_Pin GPIO_PIN_4
#define dirX_GPIO_Port GPIOA
#define limX_Pin GPIO_PIN_6
#define limX_GPIO_Port GPIOA
#define limY_Pin GPIO_PIN_0
#define limY_GPIO_Port GPIOB
#define dirY_Pin GPIO_PIN_1
#define dirY_GPIO_Port GPIOB
#define ETH_RESET_Pin GPIO_PIN_2
#define ETH_RESET_GPIO_Port GPIOB
#define limA_Pin GPIO_PIN_14
#define limA_GPIO_Port GPIOF
#define dirA_Pin GPIO_PIN_15
#define dirA_GPIO_Port GPIOF
#define TEST8_Pin GPIO_PIN_7
#define TEST8_GPIO_Port GPIOE
#define TEST7_Pin GPIO_PIN_8
#define TEST7_GPIO_Port GPIOE
#define TEST6_Pin GPIO_PIN_9
#define TEST6_GPIO_Port GPIOE
#define TEST5_Pin GPIO_PIN_10
#define TEST5_GPIO_Port GPIOE
#define TEST4_Pin GPIO_PIN_11
#define TEST4_GPIO_Port GPIOE
#define TEST3_Pin GPIO_PIN_12
#define TEST3_GPIO_Port GPIOE
#define TEST2_Pin GPIO_PIN_13
#define TEST2_GPIO_Port GPIOE
#define TEST1_Pin GPIO_PIN_14
#define TEST1_GPIO_Port GPIOE
#define SW1_Pin GPIO_PIN_15
#define SW1_GPIO_Port GPIOE
#define SW2_Pin GPIO_PIN_10
#define SW2_GPIO_Port GPIOB
#define MotorDir_Pin GPIO_PIN_15
#define MotorDir_GPIO_Port GPIOB
#define MotorEna_Pin GPIO_PIN_8
#define MotorEna_GPIO_Port GPIOD
#define MotorSense_Pin GPIO_PIN_9
#define MotorSense_GPIO_Port GPIOD
#define usbSense_Pin GPIO_PIN_10
#define usbSense_GPIO_Port GPIOD
#define Emerge_Pin GPIO_PIN_11
#define Emerge_GPIO_Port GPIOD
#define relay_Pin GPIO_PIN_14
#define relay_GPIO_Port GPIOD
#define relay2_Pin GPIO_PIN_15
#define relay2_GPIO_Port GPIOD
#define relay3_Pin GPIO_PIN_6
#define relay3_GPIO_Port GPIOG
#define dirB_Pin GPIO_PIN_7
#define dirB_GPIO_Port GPIOG
#define limB_Pin GPIO_PIN_8
#define limB_GPIO_Port GPIOG
#define encZbis1_Pin GPIO_PIN_8
#define encZbis1_GPIO_Port GPIOC
#define encZbis2_Pin GPIO_PIN_9
#define encZbis2_GPIO_Port GPIOC
#define encB1_Pin GPIO_PIN_9
#define encB1_GPIO_Port GPIOA
#define encB2_Pin GPIO_PIN_10
#define encB2_GPIO_Port GPIOA
#define LCD_RS_Pin GPIO_PIN_10
#define LCD_RS_GPIO_Port GPIOC
#define LCD_RW_Pin GPIO_PIN_11
#define LCD_RW_GPIO_Port GPIOC
#define LCD_E_Pin GPIO_PIN_12
#define LCD_E_GPIO_Port GPIOC
#define LCD_D4_Pin GPIO_PIN_0
#define LCD_D4_GPIO_Port GPIOD
#define LCD_D5_Pin GPIO_PIN_1
#define LCD_D5_GPIO_Port GPIOD
#define LCD_D6_Pin GPIO_PIN_2
#define LCD_D6_GPIO_Port GPIOD
#define LCD_D7_Pin GPIO_PIN_3
#define LCD_D7_GPIO_Port GPIOD
#define LCD_BL_Pin GPIO_PIN_4
#define LCD_BL_GPIO_Port GPIOD
#define PROBE_Pin GPIO_PIN_5
#define PROBE_GPIO_Port GPIOD
#define LED1_Pin GPIO_PIN_6
#define LED1_GPIO_Port GPIOD
#define LED2_Pin GPIO_PIN_7
#define LED2_GPIO_Port GPIOD
#define EMERGENCY_Pin GPIO_PIN_9
#define EMERGENCY_GPIO_Port GPIOG
#define panel4_Pin GPIO_PIN_8
#define panel4_GPIO_Port GPIOB
#define panel3_Pin GPIO_PIN_9
#define panel3_GPIO_Port GPIOB
#define panel2_Pin GPIO_PIN_0
#define panel2_GPIO_Port GPIOE
#define panel1_Pin GPIO_PIN_1
#define panel1_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
