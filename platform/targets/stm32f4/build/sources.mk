####################################################################################################
# @file   sources.mk
#
# @brief  stm32f4 platform makefile sources.
####################################################################################################

# Headers
INC_DIRS += $(ROOT_DIR)/platform/targets/stm32f4/include

# Thirdparty headers
INC_DIRS += \
	$(ROOT_DIR)/thirdparty/cmsis/CMSIS/Core/Include \
	$(ROOT_DIR)/thirdparty/cmsis/Device/ARM/ARMCM4/Include \
	$(BSP_DIR)/cmsis/Include \
	$(BSP_DIR)/hal/Inc

# C files
C_FILES += $(sort $(wildcard $(ROOT_DIR)/platform/targets/stm32f4/sources/*.c))

# Build C files
C_FILES += $(ROOT_DIR)/platform/targets/stm32f4/build/retarget_gcc.c
C_FILES += $(ROOT_DIR)/platform/targets/stm32f4/build/startup_stm32f446xx.c

# Thirdparty C files
C_FILES += \
	$(BSP_DIR)/cmsis/Source/Templates/system_stm32f4xx.c \
	$(BSP_DIR)/hal/Src/stm32f4xx_hal.c \
	$(BSP_DIR)/hal/Src/stm32f4xx_hal_dma.c \
	$(BSP_DIR)/hal/Src/stm32f4xx_hal_flash.c \
	$(BSP_DIR)/hal/Src/stm32f4xx_hal_gpio.c \
	$(BSP_DIR)/hal/Src/stm32f4xx_hal_rcc.c \
	$(BSP_DIR)/hal/Src/stm32f4xx_hal_uart.c \
	$(BSP_DIR)/hal/Src/stm32f4xx_hal_usart.c \
	$(BSP_DIR)/hal/Src/stm32f4xx_hal_cortex.c
