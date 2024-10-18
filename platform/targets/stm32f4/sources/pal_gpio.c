/********************************************************************************
 * @file   pal_gpio.c
 *
 * @brief  STM32F4 PAL GPIO general use functions file.
 ********************************************************************************/

#include "pal_gpio.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"

/******************************************************************************
 * @brief  STM32F4 PAL GPIO initialization function.
 *****************************************************************************/
void palInitGpio(void)
{
  /* Enable GPIO A clock. */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* Set parameters. */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  /* Initialize GPIO A.*/
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}