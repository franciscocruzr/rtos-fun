/********************************************************************************
 * @file   pal.c
 *
 * @brief  STM32F4 Main PAL functions file.
 ********************************************************************************/

#include "pal.h"
#include "pal_sys.h"
#include "pal_gpio.h"
#include "pal_uart.h"
#include "stm32f4xx_hal.h"

/******************************************************************************
 * @brief  STM32F4 main PAL initialization function. Initializes SysTick, GPIOs and
 *         NVIC.
 *
 * @note   Needs to be called before entering the main scheduler loop.
 *****************************************************************************/
void palInit(void)
{
  /* Initialize HAL module. */
  HAL_Init();

  /* Initialize the rest of the peripherals. */
  palSysInit();
  palInitGpio();
  palInitUart();
}