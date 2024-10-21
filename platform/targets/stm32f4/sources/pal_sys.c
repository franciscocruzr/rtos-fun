/********************************************************************************
 * @file   pal_sys.c
 *
 * @brief  STM32F4 PAL SYS general use functions file.
 ********************************************************************************/

#include "pal_sys.h"
#include "stm32f4xx_hal.h"

#define PAL_TICKS_PER_SEC 1000

/******************************************************************************
 * @brief  STM32F4 PAL SysTick initialization function.
 *****************************************************************************/
void palSysInit(void)
{
  /* Initialize HAL module. */
  HAL_Init();

  /* Initialize SysTick. */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / PAL_TICKS_PER_SEC);

  /* Initialize Systick interrupt priority. */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}