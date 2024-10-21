/********************************************************************************
 * @file   pal_gpio.h
 *
 * @brief  PAL GPIO general use functions header.
 ********************************************************************************/

#ifndef PAL_GPIO_H
#define PAL_GPIO_H

#include <stdint.h>

/** @brief  LED Identifiers. */
typedef enum
{
  PAL_GPIO_LED_ID_ERROR,        /**< General purpose led identifier. */
} palGpioLedId_t;

/******************************************************************************
 * @brief  PAL GPIO initialization function.
 *****************************************************************************/
void palGpioInit(void);

/******************************************************************************
 * @brief  STM32F4 PAL GPIO set LED on.
 *
 * @param  ledId      LED Identifier.
 *****************************************************************************/
void palGpioLedOn(uint8_t ledId);

/******************************************************************************
 * @brief  STM32F4 PAL GPIO set LED off.
 *
 * @param  ledId      LED Identifier.
 *****************************************************************************/
void palGpioLedOff(uint8_t ledId);

#endif