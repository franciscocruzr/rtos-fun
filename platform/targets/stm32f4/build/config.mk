####################################################################################################
# @file   config.mk
#
# @brief  stm32f4 platform makefile configuration.
####################################################################################################

# Libraries
BSP_DIR         := $(ROOT_DIR)/thirdparty/stm32f4

# Board
CFG_DEV 				+= STM32F446xx

# Peripherals
CFG_DEV					+= USE_HAL_DRIVER
CFG_DEV					+= HAL_MODULE_ENABLED
#CFG_DEV					+= HAL_ADC_MODULE_ENABLED
#CFG_DEV					+= HAL_CAN_MODULE_ENABLED
#CFG_DEV					+= HAL_CAN_LEGACY_MODULE_ENABLED
#CFG_DEV					+= HAL_CRC_MODULE_ENABLED
#CFG_DEV					+= HAL_CEC_MODULE_ENABLED
#CFG_DEV					+= HAL_CRYP_MODULE_ENABLED
#CFG_DEV					+= HAL_DAC_MODULE_ENABLED
#CFG_DEV					+= HAL_DCMI_MODULE_ENABLED
CFG_DEV					+= HAL_DMA_MODULE_ENABLED
#CFG_DEV					+= HAL_DMA2D_MODULE_ENABLED
#CFG_DEV					+= HAL_ETH_MODULE_ENABLED
#CFG_DEV					+= HAL_ETH_LEGACY_MODULE_ENABLED
CFG_DEV					+= HAL_FLASH_MODULE_ENABLED
#CFG_DEV					+= HAL_NAND_MODULE_ENABLED
#CFG_DEV					+= HAL_NOR_MODULE_ENABLED
#CFG_DEV					+= HAL_PCCARD_MODULE_ENABLED
#CFG_DEV					+= HAL_SRAM_MODULE_ENABLED
#CFG_DEV					+= HAL_SDRAM_MODULE_ENABLED
#CFG_DEV					+= HAL_HASH_MODULE_ENABLED
CFG_DEV					+= HAL_GPIO_MODULE_ENABLED
#CFG_DEV					+= HAL_EXTI_MODULE_ENABLED
#CFG_DEV					+= HAL_I2C_MODULE_ENABLED
#CFG_DEV					+= HAL_SMBUS_MODULE_ENABLED
#CFG_DEV					+= HAL_I2S_MODULE_ENABLED
#CFG_DEV					+= HAL_IWDG_MODULE_ENABLED
#CFG_DEV					+= HAL_LTDC_MODULE_ENABLED
#CFG_DEV					+= HAL_DSI_MODULE_ENABLED
#CFG_DEV					+= HAL_PWR_MODULE_ENABLED
#CFG_DEV					+= HAL_QSPI_MODULE_ENABLED
CFG_DEV					+= HAL_RCC_MODULE_ENABLED
#CFG_DEV					+= HAL_RNG_MODULE_ENABLED
#CFG_DEV					+= HAL_RTC_MODULE_ENABLED
#CFG_DEV					+= HAL_SAI_MODULE_ENABLED
#CFG_DEV					+= HAL_SD_MODULE_ENABLED
#CFG_DEV					+= HAL_SPI_MODULE_ENABLED
#CFG_DEV					+= HAL_TIM_MODULE_ENABLED
CFG_DEV					+= HAL_UART_MODULE_ENABLED
CFG_DEV					+= HAL_USART_MODULE_ENABLED
#CFG_DEV					+= HAL_IRDA_MODULE_ENABLED
#CFG_DEV					+= HAL_SMARTCARD_MODULE_ENABLED
#CFG_DEV					+= HAL_WWDG_MODULE_ENABLED
CFG_DEV					+= HAL_CORTEX_MODULE_ENABLED
#CFG_DEV					+= HAL_PCD_MODULE_ENABLED
#CFG_DEV					+= HAL_HCD_MODULE_ENABLED
#CFG_DEV					+= HAL_FMPI2C_MODULE_ENABLED
#CFG_DEV					+= HAL_FMPSMBUS_MODULE_ENABLED
#CFG_DEV					+= HAL_SPDIFRX_MODULE_ENABLED
#CFG_DEV					+= HAL_DFSDM_MODULE_ENABLED
#CFG_DEV					+= HAL_LPTIM_MODULE_ENABLED
#CFG_DEV					+= HAL_MMC_MODULE_ENABLED

# Board definitions
BOOTLOADER_SIZE := 0x0000

# Sources
LD_FILE         := $(ROOT_DIR)/platform/targets/stm32f4/build/STM32F446RETX_FLASH.ld

# Compiler flags
C_FLAGS         += -mcpu=cortex-m4 -mthumb -mlittle-endian

# Linker flags
LD_FLAGS        += -mthumb -mcpu=cortex-m4
LD_FLAGS        += --specs=nano.specs
LD_FLAGS        += --specs=nosys.specs