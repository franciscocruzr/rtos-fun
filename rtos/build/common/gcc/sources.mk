# Use target specific sources
include $(ROOT_DIR)/rtos/targets/$(RTOS)/build/sources.mk

# Include general headers
INC_DIRS += $(ROOT_DIR)/rtos/include
