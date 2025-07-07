#ifndef COMMON_DEF_H
#define COMMON_DEF_H

#include "stm32h7xx.h"
#include "stm32h7xx_hal.h"

#include "GeneralConfig.h"

#define AXE_MASTER_Timer  TIM1  
#define AXE_SYNC_Timer    TIM23   
#define AXE_SYNC_IRQ      TIM23_IRQn
#define AXE_SPEED_Timer   TIM16
#define AXE_SPEED_IRQ     TIM16_IRQn



#define TEST1_PORT GPIOE
#define TEST1_PIN GPIO_PIN_14
#define TEST2_PORT GPIOE
#define TEST2_PIN GPIO_PIN_13
#define TEST3_PORT GPIOE
#define TEST3_PIN GPIO_PIN_12
#define TEST4_PORT GPIOE
#define TEST4_PIN GPIO_PIN_11
#define TEST5_PORT GPIOE
#define TEST5_PIN GPIO_PIN_10
#define TEST6_PORT GPIOE
#define TEST6_PIN GPIO_PIN_9
#define TEST7_PORT GPIOE
#define TEST7_PIN GPIO_PIN_8
#define TEST8_PORT GPIOE
#define TEST8_PIN GPIO_PIN_7




#ifdef __cplusplus
 extern "C" {
#endif

void Error_Handler(void);

#ifdef __cplusplus
}
#endif


#endif