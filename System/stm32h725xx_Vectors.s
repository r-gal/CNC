/*********************************************************************
*                    SEGGER Microcontroller GmbH                     *
*                        The Embedded Experts                        *
**********************************************************************
*                                                                    *
*            (c) 2014 - 2024 SEGGER Microcontroller GmbH             *
*                                                                    *
*       www.segger.com     Support: support@segger.com               *
*                                                                    *
**********************************************************************
*                                                                    *
* All rights reserved.                                               *
*                                                                    *
* Redistribution and use in source and binary forms, with or         *
* without modification, are permitted provided that the following    *
* condition is met:                                                  *
*                                                                    *
* - Redistributions of source code must retain the above copyright   *
*   notice, this condition and the following disclaimer.             *
*                                                                    *
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND             *
* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,        *
* INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF           *
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE           *
* DISCLAIMED. IN NO EVENT SHALL SEGGER Microcontroller BE LIABLE FOR *
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR           *
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT  *
* OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;    *
* OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF      *
* LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT          *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE  *
* USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH   *
* DAMAGE.                                                            *
*                                                                    *
**********************************************************************

-------------------------- END-OF-HEADER -----------------------------

File      : stm32h725xx_Vectors.s
Purpose   : Exception and interrupt vectors for stm32h725xx devices.

Additional information:
  Preprocessor Definitions
    __NO_EXTERNAL_INTERRUPTS
      If defined,
        the vector table will contain only the internal exceptions
        and interrupts.
    __VECTORS_IN_RAM
      If defined,
        an area of RAM, large enough to store the vector table,
        will be reserved.

    __OPTIMIZATION_SMALL
      If defined,
        all weak definitions of interrupt handlers will share the
        same implementation.
      If not defined,
        all weak definitions of interrupt handlers will be defined
        with their own implementation.
*/
        .syntax unified

/*********************************************************************
*
*       Macros
*
**********************************************************************
*/

//
// Directly place a vector (word) in the vector table
//
.macro VECTOR Name=
        .section .vectors, "ax"
        .code 16
        .word \Name
.endm

//
// Declare an exception handler with a weak definition
//
.macro EXC_HANDLER Name=
        //
        // Insert vector in vector table
        //
        .section .vectors, "ax"
        .word \Name
        //
        // Insert dummy handler in init section
        //
        .section .init.\Name, "ax"
        .thumb_func
        .weak \Name
        .balign 2
\Name:
        1: b 1b   // Endless loop
.endm

//
// Declare an interrupt handler with a weak definition
//
.macro ISR_HANDLER Name=
        //
        // Insert vector in vector table
        //
        .section .vectors, "ax"
        .word \Name
        //
        // Insert dummy handler in init section
        //
#if defined(__OPTIMIZATION_SMALL)
        .section .init, "ax"
        .weak \Name
        .thumb_set \Name,Dummy_Handler
#else
        .section .init.\Name, "ax"
        .thumb_func
        .weak \Name
        .balign 2
\Name:
        1: b 1b   // Endless loop
#endif
.endm

//
// Place a reserved vector in vector table
//
.macro ISR_RESERVED
        .section .vectors, "ax"
        .word 0
.endm

//
// Place a reserved vector in vector table
//
.macro ISR_RESERVED_DUMMY
        .section .vectors, "ax"
        .word Dummy_Handler
.endm

/*********************************************************************
*
*       Externals
*
**********************************************************************
*/
        .extern __stack_end__
        .extern Reset_Handler
        .extern HardFault_Handler

/*********************************************************************
*
*       Global functions
*
**********************************************************************
*/

/*********************************************************************
*
*  Setup of the vector table and weak definition of interrupt handlers
*
*/
        .section .vectors, "ax"
        .code 16
        .balign 1024
        .global _vectors
_vectors:
        //
        // Internal exceptions and interrupts
        //
        VECTOR __stack_end__
        VECTOR Reset_Handler
        EXC_HANDLER NMI_Handler
        VECTOR HardFault_Handler
#ifdef __ARM_ARCH_6M__
        ISR_RESERVED
        ISR_RESERVED
        ISR_RESERVED
#else
        EXC_HANDLER MemManage_Handler
        EXC_HANDLER BusFault_Handler
        EXC_HANDLER UsageFault_Handler
#endif
        ISR_RESERVED
        ISR_RESERVED
        ISR_RESERVED
        ISR_RESERVED
        EXC_HANDLER SVC_Handler
#ifdef __ARM_ARCH_6M__
        ISR_RESERVED
#else
        EXC_HANDLER DebugMon_Handler
#endif
        ISR_RESERVED
        EXC_HANDLER PendSV_Handler
        EXC_HANDLER SysTick_Handler
        //
        // External interrupts
        //
#ifndef __NO_EXTERNAL_INTERRUPTS
        ISR_HANDLER WWDG_IRQHandler
        ISR_HANDLER PVD_AVD_IRQHandler
        ISR_HANDLER TAMP_STAMP_IRQHandler
        ISR_HANDLER RTC_WKUP_IRQHandler
        ISR_HANDLER FLASH_IRQHandler
        ISR_HANDLER RCC_IRQHandler
        ISR_HANDLER EXTI0_IRQHandler
        ISR_HANDLER EXTI1_IRQHandler
        ISR_HANDLER EXTI2_IRQHandler
        ISR_HANDLER EXTI3_IRQHandler
        ISR_HANDLER EXTI4_IRQHandler
        ISR_HANDLER DMA1_Stream0_IRQHandler
        ISR_HANDLER DMA1_Stream1_IRQHandler
        ISR_HANDLER DMA1_Stream2_IRQHandler
        ISR_HANDLER DMA1_Stream3_IRQHandler
        ISR_HANDLER DMA1_Stream4_IRQHandler
        ISR_HANDLER DMA1_Stream5_IRQHandler
        ISR_HANDLER DMA1_Stream6_IRQHandler
        ISR_HANDLER ADC_IRQHandler
        ISR_HANDLER FDCAN1_IT0_IRQHandler
        ISR_HANDLER FDCAN2_IT0_IRQHandler
        ISR_HANDLER FDCAN1_IT1_IRQHandler
        ISR_HANDLER FDCAN2_IT1_IRQHandler
        ISR_HANDLER EXTI9_5_IRQHandler
        ISR_HANDLER TIM1_BRK_IRQHandler
        ISR_HANDLER TIM1_UP_IRQHandler
        ISR_HANDLER TIM1_TRG_COM_IRQHandler
        ISR_HANDLER TIM1_CC_IRQHandler
        ISR_HANDLER TIM2_IRQHandler
        ISR_HANDLER TIM3_IRQHandler
        ISR_HANDLER TIM4_IRQHandler
        ISR_HANDLER I2C1_EV_IRQHandler
        ISR_HANDLER I2C1_ER_IRQHandler
        ISR_HANDLER I2C2_EV_IRQHandler
        ISR_HANDLER I2C2_ER_IRQHandler
        ISR_HANDLER SPI1_IRQHandler
        ISR_HANDLER SPI2_IRQHandler
        ISR_HANDLER USART1_IRQHandler
        ISR_HANDLER USART2_IRQHandler
        ISR_HANDLER USART3_IRQHandler
        ISR_HANDLER EXTI15_10_IRQHandler
        ISR_HANDLER RTC_Alarm_IRQHandler
        ISR_RESERVED
        ISR_HANDLER TIM8_BRK_TIM12_IRQHandler
        ISR_HANDLER TIM8_UP_TIM13_IRQHandler
        ISR_HANDLER TIM8_TRG_COM_TIM14_IRQHandler
        ISR_HANDLER TIM8_CC_IRQHandler
        ISR_HANDLER DMA1_Stream7_IRQHandler
        ISR_HANDLER FMC_IRQHandler
        ISR_HANDLER SDMMC1_IRQHandler
        ISR_HANDLER TIM5_IRQHandler
        ISR_HANDLER SPI3_IRQHandler
        ISR_HANDLER UART4_IRQHandler
        ISR_HANDLER UART5_IRQHandler
        ISR_HANDLER TIM6_DAC_IRQHandler
        ISR_HANDLER TIM7_IRQHandler
        ISR_HANDLER DMA2_Stream0_IRQHandler
        ISR_HANDLER DMA2_Stream1_IRQHandler
        ISR_HANDLER DMA2_Stream2_IRQHandler
        ISR_HANDLER DMA2_Stream3_IRQHandler
        ISR_HANDLER DMA2_Stream4_IRQHandler
        ISR_HANDLER ETH_IRQHandler
        ISR_HANDLER ETH_WKUP_IRQHandler
        ISR_HANDLER FDCAN_CAL_IRQHandler
        ISR_RESERVED
        ISR_RESERVED
        ISR_RESERVED
        ISR_RESERVED
        ISR_HANDLER DMA2_Stream5_IRQHandler
        ISR_HANDLER DMA2_Stream6_IRQHandler
        ISR_HANDLER DMA2_Stream7_IRQHandler
        ISR_HANDLER USART6_IRQHandler
        ISR_HANDLER I2C3_EV_IRQHandler
        ISR_HANDLER I2C3_ER_IRQHandler
        ISR_HANDLER OTG_HS_EP1_OUT_IRQHandler
        ISR_HANDLER OTG_HS_EP1_IN_IRQHandler
        ISR_HANDLER OTG_HS_WKUP_IRQHandler
        ISR_HANDLER OTG_HS_IRQHandler
        ISR_HANDLER DCMI_PSSI_IRQHandler
        ISR_RESERVED
        ISR_HANDLER RNG_IRQHandler
        ISR_HANDLER FPU_IRQHandler
        ISR_HANDLER UART7_IRQHandler
        ISR_HANDLER UART8_IRQHandler
        ISR_HANDLER SPI4_IRQHandler
        ISR_HANDLER SPI5_IRQHandler
        ISR_HANDLER SPI6_IRQHandler
        ISR_HANDLER SAI1_IRQHandler
        ISR_HANDLER LTDC_IRQHandler
        ISR_HANDLER LTDC_ER_IRQHandler
        ISR_HANDLER DMA2D_IRQHandler
        ISR_RESERVED
        ISR_HANDLER OCTOSPI1_IRQHandler
        ISR_HANDLER LPTIM1_IRQHandler
        ISR_HANDLER CEC_IRQHandler
        ISR_HANDLER I2C4_EV_IRQHandler
        ISR_HANDLER I2C4_ER_IRQHandler
        ISR_HANDLER SPDIF_RX_IRQHandler
        ISR_RESERVED
        ISR_RESERVED
        ISR_RESERVED
        ISR_RESERVED
        ISR_HANDLER DMAMUX1_OVR_IRQHandler
        ISR_RESERVED
        ISR_RESERVED
        ISR_RESERVED
        ISR_RESERVED
        ISR_RESERVED
        ISR_RESERVED
        ISR_RESERVED
        ISR_HANDLER DFSDM1_FLT0_IRQHandler
        ISR_HANDLER DFSDM1_FLT1_IRQHandler
        ISR_HANDLER DFSDM1_FLT2_IRQHandler
        ISR_HANDLER DFSDM1_FLT3_IRQHandler
        ISR_RESERVED
        ISR_HANDLER SWPMI1_IRQHandler
        ISR_HANDLER TIM15_IRQHandler
        ISR_HANDLER TIM16_IRQHandler
        ISR_HANDLER TIM17_IRQHandler
        ISR_HANDLER MDIOS_WKUP_IRQHandler
        ISR_HANDLER MDIOS_IRQHandler
        ISR_RESERVED
        ISR_HANDLER MDMA_IRQHandler
        ISR_RESERVED
        ISR_HANDLER SDMMC2_IRQHandler
        ISR_HANDLER HSEM1_IRQHandler
        ISR_RESERVED
        ISR_HANDLER ADC3_IRQHandler
        ISR_HANDLER DMAMUX2_OVR_IRQHandler
        ISR_HANDLER BDMA_Channel0_IRQHandler
        ISR_HANDLER BDMA_Channel1_IRQHandler
        ISR_HANDLER BDMA_Channel2_IRQHandler
        ISR_HANDLER BDMA_Channel3_IRQHandler
        ISR_HANDLER BDMA_Channel4_IRQHandler
        ISR_HANDLER BDMA_Channel5_IRQHandler
        ISR_HANDLER BDMA_Channel6_IRQHandler
        ISR_HANDLER BDMA_Channel7_IRQHandler
        ISR_HANDLER COMP1_IRQHandler
        ISR_HANDLER LPTIM2_IRQHandler
        ISR_HANDLER LPTIM3_IRQHandler
        ISR_HANDLER LPTIM4_IRQHandler
        ISR_HANDLER LPTIM5_IRQHandler
        ISR_HANDLER LPUART1_IRQHandler
        ISR_RESERVED
        ISR_HANDLER CRS_IRQHandler
        ISR_HANDLER ECC_IRQHandler
        ISR_HANDLER SAI4_IRQHandler
        ISR_HANDLER DTS_IRQHandler
        ISR_RESERVED
        ISR_HANDLER WAKEUP_PIN_IRQHandler
        ISR_HANDLER OCTOSPI2_IRQHandler
        ISR_RESERVED
        ISR_RESERVED
        ISR_HANDLER FMAC_IRQHandler
        ISR_HANDLER CORDIC_IRQHandler
        ISR_HANDLER UART9_IRQHandler
        ISR_HANDLER USART10_IRQHandler
        ISR_HANDLER I2C5_EV_IRQHandler
        ISR_HANDLER I2C5_ER_IRQHandler
        ISR_HANDLER FDCAN3_IT0_IRQHandler
        ISR_HANDLER FDCAN3_IT1_IRQHandler
        ISR_HANDLER TIM23_IRQHandler
        ISR_HANDLER TIM24_IRQHandler
#endif
        //
        .section .vectors, "ax"
_vectors_end:

#ifdef __VECTORS_IN_RAM
        //
        // Reserve space with the size of the vector table
        // in the designated RAM section.
        //
        .section .vectors_ram, "ax"
        .balign 1024
        .global _vectors_ram

_vectors_ram:
        .space _vectors_end - _vectors, 0
#endif

/*********************************************************************
*
*  Dummy handler to be used for reserved interrupt vectors
*  and weak implementation of interrupts.
*
*/
        .section .init.Dummy_Handler, "ax"
        .thumb_func
        .weak Dummy_Handler
        .balign 2
Dummy_Handler:
        1: b 1b   // Endless loop


/*************************** End of file ****************************/
