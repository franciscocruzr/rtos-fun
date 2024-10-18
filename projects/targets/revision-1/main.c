/******************************************************************************
 * @file   main.c
 *
 * @brief  Revision-1 main file.
 *****************************************************************************/

#include "main.h"
#include "os.h"
#include "pal.h"

#include <stdbool.h>

/******************************************************************************
 * @brief  Main function.
 *
 * @return Does not return.
 *
 * @note   Gets called after startup.
 *****************************************************************************/
int main(void)
{
  palInit();

  osMainLoop();

  return 0;
}