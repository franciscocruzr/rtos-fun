# Root directory
ROOT_DIR	:= ../../../../..

# Output
BIN_FILE	:= revision-1.elf

# Options
DEBUG			:= 1
TRACE			:= 1

# Includes
include $(ROOT_DIR)/projects/build/common/gcc/config.mk
include $(ROOT_DIR)/platform/build/common/gcc/config.mk
include $(ROOT_DIR)/rtos/build/common/gcc/config.mk

# Sources
C_FILES += $(ROOT_DIR)/projects/targets/revision-1/main.c
include $(ROOT_DIR)/projects/build/common/gcc/sources.mk
include $(ROOT_DIR)/platform/build/common/gcc/sources.mk
include $(ROOT_DIR)/rtos/build/common/gcc/sources.mk

# Targets
include $(ROOT_DIR)/projects/build/common/gcc/build.mk
include $(ROOT_DIR)/platform/build/common/gcc/install.mk
