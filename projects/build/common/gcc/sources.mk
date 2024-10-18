####################################################################################################
# @file   sources.mk
#
# @brief  Common project makefile sources.
####################################################################################################

# Headers
INC_DIRS += $(ROOT_DIR)/projects/include

# C files
C_FILES += $(sort $(wildcard $(ROOT_DIR)/projects/sources/*.c))