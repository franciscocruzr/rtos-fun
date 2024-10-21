/********************************************************************************
 * @file   pal_usart.c
 *
 * @brief  PAL USART general use functions file.
 ********************************************************************************/

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_usart.h"
#include "pal_usart.h"

#define PAL_USART_BAUD_RATE   115200

/******************************************************************************
 * @brief  STM32F4 PAL USART initialization function.
 *****************************************************************************/
void palUsartInit(void)
{
  /* Initialize NVIC for USART1. */
  const uint32_t preemptPrio = 1;
  const uint32_t subPrio = 0;
  HAL_NVIC_SetPriority(USART1_IRQn, preemptPrio, subPrio);
  HAL_NVIC_EnableIRQ(USART1_IRQn);

  /* Enable USART 1 clock. */
  __HAL_RCC_USART1_CLK_ENABLE();

  /* Set parameters. */
  USART_HandleTypeDef husart1;
  husart1.Init.BaudRate = PAL_USART_BAUD_RATE;
  husart1.Init.WordLength = USART_WORDLENGTH_8B;
  husart1.Init.StopBits = USART_STOPBITS_1;
  husart1.Init.Parity = USART_PARITY_NONE;
  husart1.Init.Mode = USART_MODE_TX_RX;
  husart1.Init.CLKPolarity = USART_POLARITY_LOW;
  husart1.Init.CLKPhase = USART_PHASE_1EDGE;
  husart1.Init.CLKLastBit = USART_LASTBIT_DISABLE;

  /* Initialize USART 1.*/
  if (HAL_USART_Init(&husart1) != HAL_OK)
  {
    /* TODO: Set error handling to an ASSERT. */
  }
}