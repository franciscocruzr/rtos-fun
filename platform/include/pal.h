/********************************************************************************
 * @file   pal.h
 *
 * @brief  Main PAL functions header.
 ********************************************************************************/

#ifndef PAL_H
#define PAL_H

/******************************************************************************
 * @brief  Main PAL initialization function. Initializes SysTick timer, GPIOs
 *         and NVIC.
 *
 * @note   Needs to be called before entering the main scheduler loop.
 *****************************************************************************/
void palInit(void);

#endif