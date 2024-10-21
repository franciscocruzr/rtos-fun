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
void palGpioInit(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

/******************************************************************************
 * @brief  STM32F4 PAL GPIO set LED on.
 *
 * @param  ledId      LED Identifier.
 *****************************************************************************/
void palGpioLedOn(uint8_t ledId)
{
  switch(ledId)
  {
    case PAL_GPIO_LED_ID_ERROR:
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
      break;
    default:
      break;
  }
}

/******************************************************************************
 * @brief  STM32F4 PAL GPIO set LED off.
 *
 * @param  ledId      LED Identifier.
 *****************************************************************************/
void palGpioLedOff(uint8_t ledId)
{
  switch(ledId)
  {
    case PAL_GPIO_LED_ID_ERROR:
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
      break;
    default:
      break;
  }
}