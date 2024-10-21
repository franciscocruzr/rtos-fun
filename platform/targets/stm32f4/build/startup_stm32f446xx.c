/********************************************************************************
 * @file   startup_stm32f446xx.c
 *
 * @brief  STM32F446xx system startup for GCC compiler.
 ********************************************************************************/

#include "main.h"
#include "pal_gpio.h"
#include "stm32f4xx.h"

#include <stdint.h>

/* Forward declaration of functions. */
extern void SystemInit(void);
extern void __libc_init_array(void);
extern int main(void);

/* Core vectors. */
void __attribute__((weak)) Reset_Handler(void);
void __attribute__((weak)) NMI_Handler(void);
void __attribute__((weak)) HardFault_Handler(void);
void __attribute__((weak)) MemManage_Handler(void);
void __attribute__((weak)) BusFault_Handler(void);
void __attribute__((weak)) UsageFault_Handler(void);
void __attribute__((weak)) SVC_Handler(void);
void __attribute__((weak)) DebugMon_Handler(void);
void __attribute__((weak)) PendSV_Handler(void);
void __attribute__((weak)) SysTick_Handler(void);
void __attribute__((weak)) WWDG_IRQHandler(void);
void __attribute__((weak)) PVD_IRQHandler(void);
void __attribute__((weak)) TAMP_STAMP_IRQHandler(void);
void __attribute__((weak)) RTC_WKUP_IRQHandler(void);
void __attribute__((weak)) FLASH_IRQHandler(void);
void __attribute__((weak)) RCC_IRQHandler(void);
void __attribute__((weak)) EXTI0_IRQHandler(void);
void __attribute__((weak)) EXTI1_IRQHandler(void);
void __attribute__((weak)) EXTI2_IRQHandler(void);
void __attribute__((weak)) EXTI3_IRQHandler(void);
void __attribute__((weak)) EXTI4_IRQHandler(void);
void __attribute__((weak)) DMA1_Stream0_IRQHandler(void);
void __attribute__((weak)) DMA1_Stream1_IRQHandler(void);
void __attribute__((weak)) DMA1_Stream2_IRQHandler(void);
void __attribute__((weak)) DMA1_Stream3_IRQHandler(void);
void __attribute__((weak)) DMA1_Stream4_IRQHandler(void);
void __attribute__((weak)) DMA1_Stream5_IRQHandler(void);
void __attribute__((weak)) DMA1_Stream6_IRQHandler(void);
void __attribute__((weak)) ADC_IRQHandler(void);
void __attribute__((weak)) CAN1_TX_IRQHandler(void);
void __attribute__((weak)) CAN1_RX0_IRQHandler(void);
void __attribute__((weak)) CAN1_RX1_IRQHandler(void);
void __attribute__((weak)) CAN1_SCE_IRQHandler(void);
void __attribute__((weak)) EXTI9_5_IRQHandler(void);
void __attribute__((weak)) TIM1_BRK_TIM9_IRQHandler(void);
void __attribute__((weak)) TIM1_UP_TIM10_IRQHandler(void);
void __attribute__((weak)) TIM1_TRG_COM_TIM11_IRQHandler(void);
void __attribute__((weak)) TIM1_CC_IRQHandler(void);
void __attribute__((weak)) TIM2_IRQHandler(void);
void __attribute__((weak)) TIM3_IRQHandler(void);
void __attribute__((weak)) TIM4_IRQHandler(void);
void __attribute__((weak)) I2C1_EV_IRQHandler(void);
void __attribute__((weak)) I2C1_ER_IRQHandler(void);
void __attribute__((weak)) I2C2_EV_IRQHandler(void);
void __attribute__((weak)) I2C2_ER_IRQHandler(void);
void __attribute__((weak)) SPI1_IRQHandler(void);
void __attribute__((weak)) SPI2_IRQHandler(void);
void __attribute__((weak)) USART1_IRQHandler(void);
void __attribute__((weak)) USART2_IRQHandler(void);
void __attribute__((weak)) USART3_IRQHandler(void);
void __attribute__((weak)) EXTI15_10_IRQHandler(void);
void __attribute__((weak)) RTC_Alarm_IRQHandler(void);
void __attribute__((weak)) OTG_FS_WKUP_IRQHandler(void);
void __attribute__((weak)) TIM8_BRK_TIM12_IRQHandler(void);
void __attribute__((weak)) TIM8_UP_TIM13_IRQHandler(void);
void __attribute__((weak)) TIM8_TRG_COM_TIM14_IRQHandler(void);
void __attribute__((weak)) TIM8_CC_IRQHandler(void);
void __attribute__((weak)) DMA1_Stream7_IRQHandler(void);
void __attribute__((weak)) FMC_IRQHandler(void);
void __attribute__((weak)) SDIO_IRQHandler(void);
void __attribute__((weak)) TIM5_IRQHandler(void);
void __attribute__((weak)) SPI3_IRQHandler(void);
void __attribute__((weak)) UART4_IRQHandler(void);
void __attribute__((weak)) UART5_IRQHandler(void);
void __attribute__((weak)) TIM6_DAC_IRQHandler(void);
void __attribute__((weak)) TIM7_IRQHandler(void);
void __attribute__((weak)) DMA2_Stream0_IRQHandler(void);
void __attribute__((weak)) DMA2_Stream1_IRQHandler(void);
void __attribute__((weak)) DMA2_Stream2_IRQHandler(void);
void __attribute__((weak)) DMA2_Stream3_IRQHandler(void);
void __attribute__((weak)) DMA2_Stream4_IRQHandler(void);
void __attribute__((weak)) CAN2_TX_IRQHandler(void);
void __attribute__((weak)) CAN2_RX0_IRQHandler(void);
void __attribute__((weak)) CAN2_RX1_IRQHandler(void);
void __attribute__((weak)) CAN2_SCE_IRQHandler(void);
void __attribute__((weak)) OTG_FS_IRQHandler(void);
void __attribute__((weak)) DMA2_Stream5_IRQHandler(void);
void __attribute__((weak)) DMA2_Stream6_IRQHandler(void);
void __attribute__((weak)) DMA2_Stream7_IRQHandler(void);
void __attribute__((weak)) USART6_IRQHandler(void);
void __attribute__((weak)) I2C3_EV_IRQHandler(void);
void __attribute__((weak)) I2C3_ER_IRQHandler(void);
void __attribute__((weak)) OTG_HS_EP1_OUT_IRQHandler(void);
void __attribute__((weak)) OTG_HS_EP1_IN_IRQHandler(void);
void __attribute__((weak)) OTG_HS_WKUP_IRQHandler(void);
void __attribute__((weak)) OTG_HS_IRQHandler(void);
void __attribute__((weak)) DCMI_IRQHandler(void);
void __attribute__((weak)) FPU_IRQHandler(void);
void __attribute__((weak)) SPI4_IRQHandler(void);
void __attribute__((weak)) SAI1_IRQHandler(void);
void __attribute__((weak)) SAI2_IRQHandler(void);
void __attribute__((weak)) QUADSPI_IRQHandler(void);
void __attribute__((weak)) CEC_IRQHandler(void);
void __attribute__((weak)) SPDIF_RX_IRQHandler(void);
void __attribute__((weak)) FMPI2C1_EV_IRQHandler(void);
void __attribute__((weak)) FMPI2C1_ER_IRQHandler(void);

/* Assign default weak references. */
#pragma weak NMI_Handler                   = Default_Handler
#pragma weak MemManage_Handler             = Default_Handler
#pragma weak BusFault_Handler              = Default_Handler
#pragma weak UsageFault_Handler            = Default_Handler
#pragma weak SVC_Handler                   = Default_Handler
#pragma weak DebugMon_Handler              = Default_Handler
#pragma weak PendSV_Handler                = Default_Handler
#pragma weak SysTick_Handler               = Default_Handler
#pragma weak WWDG_IRQHandler               = Default_Handler
#pragma weak PVD_IRQHandler                = Default_Handler
#pragma weak TAMP_STAMP_IRQHandler         = Default_Handler
#pragma weak RTC_WKUP_IRQHandler           = Default_Handler
#pragma weak FLASH_IRQHandler              = Default_Handler
#pragma weak RCC_IRQHandler                = Default_Handler
#pragma weak EXTI0_IRQHandler              = Default_Handler
#pragma weak EXTI1_IRQHandler              = Default_Handler
#pragma weak EXTI2_IRQHandler              = Default_Handler
#pragma weak EXTI3_IRQHandler              = Default_Handler
#pragma weak EXTI4_IRQHandler              = Default_Handler
#pragma weak DMA1_Stream0_IRQHandler       = Default_Handler
#pragma weak DMA1_Stream1_IRQHandler       = Default_Handler
#pragma weak DMA1_Stream2_IRQHandler       = Default_Handler
#pragma weak DMA1_Stream3_IRQHandler       = Default_Handler
#pragma weak DMA1_Stream4_IRQHandler       = Default_Handler
#pragma weak DMA1_Stream5_IRQHandler       = Default_Handler
#pragma weak DMA1_Stream6_IRQHandler       = Default_Handler
#pragma weak ADC_IRQHandler                = Default_Handler
#pragma weak CAN1_TX_IRQHandler            = Default_Handler
#pragma weak CAN1_RX0_IRQHandler           = Default_Handler
#pragma weak CAN1_RX1_IRQHandler           = Default_Handler
#pragma weak CAN1_SCE_IRQHandler           = Default_Handler
#pragma weak EXTI9_5_IRQHandler            = Default_Handler
#pragma weak TIM1_BRK_TIM9_IRQHandler      = Default_Handler
#pragma weak TIM1_UP_TIM10_IRQHandler      = Default_Handler
#pragma weak TIM1_TRG_COM_TIM11_IRQHandler = Default_Handler
#pragma weak TIM1_CC_IRQHandler            = Default_Handler
#pragma weak TIM2_IRQHandler               = Default_Handler
#pragma weak TIM3_IRQHandler               = Default_Handler
#pragma weak TIM4_IRQHandler               = Default_Handler
#pragma weak I2C1_EV_IRQHandler            = Default_Handler
#pragma weak I2C1_ER_IRQHandler            = Default_Handler
#pragma weak I2C2_EV_IRQHandler            = Default_Handler
#pragma weak I2C2_ER_IRQHandler            = Default_Handler
#pragma weak SPI1_IRQHandler               = Default_Handler
#pragma weak SPI2_IRQHandler               = Default_Handler
#pragma weak USART1_IRQHandler             = Default_Handler
#pragma weak USART2_IRQHandler             = Default_Handler
#pragma weak USART3_IRQHandler             = Default_Handler
#pragma weak EXTI15_10_IRQHandler          = Default_Handler
#pragma weak RTC_Alarm_IRQHandler          = Default_Handler
#pragma weak OTG_FS_WKUP_IRQHandler        = Default_Handler
#pragma weak TIM8_BRK_TIM12_IRQHandler     = Default_Handler
#pragma weak TIM8_UP_TIM13_IRQHandler      = Default_Handler
#pragma weak TIM8_TRG_COM_TIM14_IRQHandler = Default_Handler
#pragma weak TIM8_CC_IRQHandler            = Default_Handler
#pragma weak DMA1_Stream7_IRQHandler       = Default_Handler
#pragma weak FMC_IRQHandler                = Default_Handler
#pragma weak SDIO_IRQHandler               = Default_Handler
#pragma weak TIM5_IRQHandler               = Default_Handler
#pragma weak SPI3_IRQHandler               = Default_Handler
#pragma weak UART4_IRQHandler              = Default_Handler
#pragma weak UART5_IRQHandler              = Default_Handler
#pragma weak TIM6_DAC_IRQHandler           = Default_Handler
#pragma weak TIM7_IRQHandler               = Default_Handler
#pragma weak DMA2_Stream0_IRQHandler       = Default_Handler
#pragma weak DMA2_Stream1_IRQHandler       = Default_Handler
#pragma weak DMA2_Stream2_IRQHandler       = Default_Handler
#pragma weak DMA2_Stream3_IRQHandler       = Default_Handler
#pragma weak DMA2_Stream4_IRQHandler       = Default_Handler
#pragma weak CAN2_TX_IRQHandler            = Default_Handler
#pragma weak CAN2_RX0_IRQHandler           = Default_Handler
#pragma weak CAN2_RX1_IRQHandler           = Default_Handler
#pragma weak CAN2_SCE_IRQHandler           = Default_Handler
#pragma weak OTG_FS_IRQHandler             = Default_Handler
#pragma weak DMA2_Stream5_IRQHandler       = Default_Handler
#pragma weak DMA2_Stream6_IRQHandler       = Default_Handler
#pragma weak DMA2_Stream7_IRQHandler       = Default_Handler
#pragma weak USART6_IRQHandler             = Default_Handler
#pragma weak I2C3_EV_IRQHandler            = Default_Handler
#pragma weak I2C3_ER_IRQHandler            = Default_Handler
#pragma weak OTG_HS_EP1_OUT_IRQHandler     = Default_Handler
#pragma weak OTG_HS_EP1_IN_IRQHandler      = Default_Handler
#pragma weak OTG_HS_WKUP_IRQHandler        = Default_Handler
#pragma weak OTG_HS_IRQHandler             = Default_Handler
#pragma weak DCMI_IRQHandler               = Default_Handler
#pragma weak FPU_IRQHandler                = Default_Handler
#pragma weak SPI4_IRQHandler               = Default_Handler
#pragma weak SAI1_IRQHandler               = Default_Handler
#pragma weak SAI2_IRQHandler               = Default_Handler
#pragma weak QUADSPI_IRQHandler            = Default_Handler
#pragma weak CEC_IRQHandler                = Default_Handler
#pragma weak SPDIF_RX_IRQHandler           = Default_Handler
#pragma weak FMPI2C1_EV_IRQHandler         = Default_Handler
#pragma weak FMPI2C1_ER_IRQHandler         = Default_Handler


/* Defined by linker. */
extern uint32_t _sidata; /* Start address for data segment in Flash. */
extern uint32_t _sdata;  /* Start address for data segment in SRAM. */
extern uint32_t _edata;  /* End address for data segment in SRAM. */
extern uint32_t _sbss;   /* Start address for .bss segment in SRAM. */
extern uint32_t _ebss;   /* End address for .bss segment in SRAM. */
extern uint32_t _estack; /* End of stack address. */

typedef void (*ISR_Handler)(void);

/** @brief  Core vector table. */
ISR_Handler const g_pfnVectors[] __attribute__((section(".isr_vector"))) =
{
  (ISR_Handler)(uintptr_t)(&_estack),  /**< Initial stack pointer. */
  Reset_Handler,                       /**< Reset handler.         */
  NMI_Handler,                         /**< NMI handler.           */
  HardFault_Handler,                   /**< Hard fault handler.    */
  MemManage_Handler,                   /**< Memory manage handler. */
  BusFault_Handler,                    /**< Bus fault handler.     */
  UsageFault_Handler,                  /**< Usage fault handler.   */
  0,                                   /**< Reserved.              */
  0,                                   /**< Reserved.              */
  0,                                   /**< Reserved.              */
  0,                                   /**< Reserved.              */
  SVC_Handler,                         /**< SVC handler.           */
  DebugMon_Handler,                    /**< Debug monitor handler. */
  0,                                   /**< Reserved.              */
  PendSV_Handler,                      /**< PendSV handler.        */
  SysTick_Handler,                     /**< SysTick handler.       */

  /* External Interrupts */
  WWDG_IRQHandler,                     /**< Window WatchDog.                    */
  PVD_IRQHandler,                      /**< PVD through EXTI Line detection.    */
  TAMP_STAMP_IRQHandler,               /**< Tamper and TimeStamps through EXTI. */
  RTC_WKUP_IRQHandler,                 /**< RTC Wakeup through the EXTI line    */
  FLASH_IRQHandler,                    /**< FLASH.                              */
  RCC_IRQHandler,                      /**< RCC.                                */
  EXTI0_IRQHandler,                    /**< EXTI Line0.                         */
  EXTI1_IRQHandler,                    /**< EXTI Line1.                         */
  EXTI2_IRQHandler,                    /**< EXTI Line2.                         */
  EXTI3_IRQHandler,                    /**< EXTI Line3.                         */
  EXTI4_IRQHandler,                    /**< EXTI Line4.                         */
  DMA1_Stream0_IRQHandler,             /**< DMA1 Stream 0.                      */
  DMA1_Stream1_IRQHandler,             /**< DMA1 Stream 1.                      */
  DMA1_Stream2_IRQHandler,             /**< DMA1 Stream 2.                      */
  DMA1_Stream3_IRQHandler,             /**< DMA1 Stream 3.                      */
  DMA1_Stream4_IRQHandler,             /**< DMA1 Stream 4.                      */
  DMA1_Stream5_IRQHandler,             /**< DMA1 Stream 5.                      */
  DMA1_Stream6_IRQHandler,             /**< DMA1 Stream 6.                      */
  ADC_IRQHandler,                      /**< ADC1, ADC2 and ADC3s.               */
  CAN1_TX_IRQHandler,                  /**< CAN1 TX.                            */
  CAN1_RX0_IRQHandler,                 /**< CAN1 RX0.                           */
  CAN1_RX1_IRQHandler,                 /**< CAN1 RX1.                           */
  CAN1_SCE_IRQHandler,                 /**< CAN1 SCE.                           */
  EXTI9_5_IRQHandler,                  /**< External Line[9:5]s.                */
  TIM1_BRK_TIM9_IRQHandler,            /**< TIM1 Break and TIM9.                */
  TIM1_UP_TIM10_IRQHandler,            /**< TIM1 Update and TIM10.              */
  TIM1_TRG_COM_TIM11_IRQHandler,       /**< TIM1 Trigger/Commutation and TIM11. */
  TIM1_CC_IRQHandler,                  /**< TIM1 Capture Compare.               */
  TIM2_IRQHandler,                     /**< TIM2.                               */
  TIM3_IRQHandler,                     /**< TIM3.                               */
  TIM4_IRQHandler,                     /**< TIM4.                               */
  I2C1_EV_IRQHandler,                  /**< I2C1 Event.                         */
  I2C1_ER_IRQHandler,                  /**< I2C1 Error.                         */
  I2C2_EV_IRQHandler,                  /**< I2C2 Event.                         */
  I2C2_ER_IRQHandler,                  /**< I2C2 Error.                         */
  SPI1_IRQHandler,                     /**< SPI1.                               */
  SPI2_IRQHandler,                     /**< SPI2.                               */
  USART1_IRQHandler,                   /**< USART1.                             */
  USART2_IRQHandler,                   /**< USART2.                             */
  USART3_IRQHandler,                   /**< USART3.                             */
  EXTI15_10_IRQHandler,                /**< External Line[15:10]s.              */
  RTC_Alarm_IRQHandler,                /**< RTC Alarm (A and B) through EXTI.   */
  OTG_FS_WKUP_IRQHandler,              /**< USB OTG FS Wakeup through EXTI.     */
  TIM8_BRK_TIM12_IRQHandler,           /**< TIM8 Break and TIM12.               */
  TIM8_UP_TIM13_IRQHandler,            /**< TIM8 Update and TIM13.              */
  TIM8_TRG_COM_TIM14_IRQHandler,       /**< TIM8 Trigger/Commutation and TIM14. */
  TIM8_CC_IRQHandler,                  /**< TIM8 Capture Compare.               */
  DMA1_Stream7_IRQHandler,             /**< DMA1 Stream7.                       */
  FMC_IRQHandler,                      /**< FMC.                                */
  SDIO_IRQHandler,                     /**< SDIO.                               */
  TIM5_IRQHandler,                     /**< TIM5.                               */
  SPI3_IRQHandler,                     /**< SPI3.                               */
  UART4_IRQHandler,                    /**< UART4.                              */
  UART5_IRQHandler,                    /**< UART5.                              */
  TIM6_DAC_IRQHandler,                 /**< TIM6 and DAC1&2 underrun errors.    */
  TIM7_IRQHandler,                     /**< TIM7.                               */
  DMA2_Stream0_IRQHandler,             /**< DMA2 Stream 0.                      */
  DMA2_Stream1_IRQHandler,             /**< DMA2 Stream 1.                      */
  DMA2_Stream2_IRQHandler,             /**< DMA2 Stream 2.                      */
  DMA2_Stream3_IRQHandler,             /**< DMA2 Stream 3.                      */
  DMA2_Stream4_IRQHandler,             /**< DMA2 Stream 4.                      */
  0,                                   /**< Reserved.                           */
  0,                                   /**< Reserved.                           */
  CAN2_TX_IRQHandler,                  /**< CAN2 TX.                            */
  CAN2_RX0_IRQHandler,                 /**< CAN2 RX0.                           */
  CAN2_RX1_IRQHandler,                 /**< CAN2 RX1.                           */
  CAN2_SCE_IRQHandler,                 /**< CAN2 SCE.                           */
  OTG_FS_IRQHandler,                   /**< USB OTG FS.                         */
  DMA2_Stream5_IRQHandler,             /**< DMA2 Stream 5.                      */
  DMA2_Stream6_IRQHandler,             /**< DMA2 Stream 6.                      */
  DMA2_Stream7_IRQHandler,             /**< DMA2 Stream 7.                      */
  USART6_IRQHandler,                   /**< USART6.                             */
  I2C3_EV_IRQHandler,                  /**< I2C3 event.                         */
  I2C3_ER_IRQHandler,                  /**< I2C3 error.                         */
  OTG_HS_EP1_OUT_IRQHandler,           /**< USB OTG HS End Point 1 Out.         */
  OTG_HS_EP1_IN_IRQHandler,            /**< USB OTG HS End Point 1 In.          */
  OTG_HS_WKUP_IRQHandler,              /**< USB OTG HS Wakeup through EXTI.     */
  OTG_HS_IRQHandler,                   /**< USB OTG HS.                         */
  DCMI_IRQHandler,                     /**< DCMI.                               */
  0,                                   /**< Reserved.                           */
  0,                                   /**< Reserved.                           */
  FPU_IRQHandler,                      /**< FPU.                                */
  0,                                   /**< Reserved.                           */
  0,                                   /**< Reserved.                           */
  SPI4_IRQHandler,                     /**< SPI4.                               */
  0,                                   /**< Reserved.                           */
  0,                                   /**< Reserved.                           */
  SAI1_IRQHandler,                     /**< SAI1.                               */
  0,                                   /**< Reserved.                           */
  0,                                   /**< Reserved.                           */
  0,                                   /**< Reserved.                           */
  SAI2_IRQHandler,                     /**< SAI2.                               */
  QUADSPI_IRQHandler,                  /**< QuadSPI.                            */
  CEC_IRQHandler,                      /**< CEC.                                */
  SPDIF_RX_IRQHandler,                 /**< SPDIF RX.                           */
  FMPI2C1_EV_IRQHandler,               /**< FMPI2C 1 Event.                     */
  FMPI2C1_ER_IRQHandler,               /**< FMPI2C 1 Error.                     */
};

/******************************************************************************
 * @brief  STM32F446xx reset handler.
 *****************************************************************************/
void Reset_Handler(void)
{
  /* Set the stack pointer. */
  __asm volatile ("ldr sp, =_estack");

  /* Call the clock system initialization function
   * Setup VTOR address with address of the vector table. */
  SystemInit();

  /* Copy the data segment initializers from Flash to SRAM. */
  uint32_t *pSrc = &_sidata;
  uint32_t *pDest = &_sdata;
  while (pDest < &_edata)
  {
    *pDest++ = *pSrc++;
  }

  /* Zero fill the .bss segment. */
  uint32_t *pBss = &_sbss;
  while (pBss < &_ebss)
  {
    *pBss++ = 0;
  }

  /* Call static constructors (if any). */
  __libc_init_array();

  /* Call the application's entry point. */
  main();

  /* Loop indefinitely if main returns. */
  while (1);
}

/******************************************************************************
 * @brief  STM32F446xx hardware fault handler.
 *****************************************************************************/
void HardFault_Handler(void)
{
  palGpioLedOn(PAL_GPIO_LED_ID_ERROR);

  /* Local variable used to get the MSP value. */
  volatile uint32_t msp = (uint32_t)&msp;
  volatile unsigned int forever = 1;
  volatile uint32_t faultPC = 0;

  /* Verify if this is a precise or imprecise HW Fault. If the imprecise bit is set,
   * then the fault address (PC) cannot be determined acurately. */
  volatile uint32_t imprecise = (SCB->CFSR & SCB_CFSR_IMPRECISERR_Msk) >> SCB_CFSR_IMPRECISERR_Pos;

  if (!imprecise)
  {
    /* Determine the address (PC) of the instruction next to the one that caused the HW Fault
     * (bare-metal only). Valid only for precise HW Fault. */
    faultPC = *((uint32_t*)(msp + 0x2C));
    /* Clear bit 0 (Thumb mode). */
    faultPC &= ~1;
  }

  while (forever);

  (void)faultPC;
}

/******************************************************************************
 * @brief  STM32F446xx default handler.
 *****************************************************************************/
void Default_Handler(void)
{
  while (1);
}