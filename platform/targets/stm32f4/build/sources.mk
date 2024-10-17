# Headers
INC_DIRS += $(ROOT_DIR)/platform/targets/stm32f4/include

# Thirdparty headers
INC_DIRS += \
	$(ROOT_DIR)/thirdparty/cmsis/CMSIS/Core/Include \
	$(ROOT_DIR)/thirdparty/cmsis/Device/ARM/ARMCM4/Include \
	$(BSP_DIR)/cmsis/Include \
	$(BSP_DIR)/hal/Inc

# C files
C_FILES += $(sort $(wildcard $(ROOT_DIR)/plaftorm/targets/stm32f4/sources/*.c))

# Build C files
C_FILES += $(ROOT_DIR)/platform/targets/stm32f4/build/retarget_gcc.c

# Thirdparty C files
C_FILES += \
	$(BSP_DIR)/cmsis/Source/Templates/system_stm32f4xx.c \
	$(BSP_DIR)/hal/Src/stm32f4xx_hal.c

A_FILES += 	$(BSP_DIR)/cmsis/Source/Templates/gcc/startup_stm32f446xx.s
