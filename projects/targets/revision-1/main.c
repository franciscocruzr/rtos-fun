/******************************************************************************
 * @file   main.c
 *
 * @brief  Revision-1 main file.
 *****************************************************************************/

#include "main.h"
#include "os.h"
#include "pal_sys.h"
#include "pal_gpio.h"
#include "pal_usart.h"

/******************************************************************************
 * @brief  Main Peripheral Abstraction Layer initialization.
 *
 * @note   Must be called befor entering OS main loop.
 *****************************************************************************/
static void mainPalInit(void)
{
  palSysInit();
  palGpioInit();
  palUsartInit();
}

/******************************************************************************
 * @brief  Main function.
 *
 * @return Does not return.
 *
 * @note   Gets called after startup.
 *****************************************************************************/
int main(void)
{
  /* Initialize peripheral abstraction layer. */
  mainPalInit();

  while (1)
  {
    palGpioLedOn(PAL_GPIO_LED_ID_ERROR);
    palGpioLedOff(PAL_GPIO_LED_ID_ERROR);
  }

  osMainLoop();

  return 0;
}