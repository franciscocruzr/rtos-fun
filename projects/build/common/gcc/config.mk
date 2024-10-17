# Platform targets
PLATFORM_LIST   := $(sort $(notdir $(wildcard $(ROOT_DIR)/platform/targets/*)))
RTOS_LIST				:= $(sort $(notdir $(wildcard $(ROOT_DIR)/rtos/targets/*)))

# Inputs
ifeq ($(PLATFORM), )
PLATFORM 				:= $(firstword $(PLATFORM_LIST))
endif

ifeq ($(RTOS), )
RTOS						:= $(firstword $(RTOS_LIST))
endif

# Output
INT_DIR					:= obj
BIN_DIR					:= bin
BIN  						:= $(BIN_DIR)/$(BIN_FILE)

# Options
C_STD						:= -std=c17 -pedantic

# Debug configuration
ifeq ($(DEBUG), 0)
CFG_DEV					+= ASSERT_ENABLED=0
else
CFG_DEV					+= ASSERT_ENABLED=1
endif

ifeq ($(TRACE), 1)
CFG_DEV					+= TRACE_ENABLED=1
else
CFG_DEV					+= TRACE_ENABLED=0
endif

# Compilation flags
C_FLAGS					+= $(C_STD)
C_FLAGS         += -Wall -Wextra -Wno-unused-parameter -Wshadow	#-Wno-expansion-to-defined
C_FLAGS         += -fno-common -ffunction-sections -fdata-sections

ifeq ($(DEBUG), 0)
C_FLAGS 				+= -fomit-frame-pointer -Os -g
else
C_FLAGS					+= -O0 -g -DDEBUG
endif

# Archiver flags
AR_FLAGS				:= rcs

# Linker flags
LD_FLAGS        := -Wl,-Map=$(BIN:.elf=.map)
LD_FLAGS        += -Wl,--gc-sections
