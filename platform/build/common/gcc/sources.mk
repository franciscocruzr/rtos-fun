####################################################################################################
# @file   sources.mk
#
# @brief  Common platform makefile sources.
####################################################################################################

# Use target specific file sources
include $(ROOT_DIR)/platform/targets/$(PLATFORM)/build/sources.mk

# Include general headers
INC_DIRS += $(ROOT_DIR)/platform/include
