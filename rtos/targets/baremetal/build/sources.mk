####################################################################################################
# @file   sources.mk
#
# @brief  Baremetal RTOS makefile sources.
####################################################################################################

# C files
C_FILES += $(sort $(wildcard $(ROOT_DIR)/rtos/targets/baremetal/sources/*.c))
