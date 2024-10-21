####################################################################################################
# @file   build.mk
#
# @brief  Common project makefile build.
####################################################################################################

# GCC ARM cross toolchain
CROSS_COMPILE   := arm-none-eabi-

# Toolchain
CC              := $(CROSS_COMPILE)gcc
AR              := $(CROSS_COMPILE)gcc-ar
LD              := $(CROSS_COMPILE)gcc
DEP             := $(CROSS_COMPILE)gcc
OBJDUMP         := $(CROSS_COMPILE)objdump
OBJCOPY         := $(CROSS_COMPILE)objcopy
SIZE            := $(CROSS_COMPILE)size

# Add include directories
C_FLAGS         += $(addprefix -I,$(INC_DIRS))

# Add configuration flags
C_FLAGS         += $(addprefix -D,$(CFG_DEV))

# Dependency flags
DEP_FLAGS       := $(C_FLAGS) -MM -MF

# Object files
OBJ_FILES       := $(C_FILES:.c=.o) $(A_FILES:.s=.o)
OBJ_FILES       := $(subst $(ROOT_DIR)/,$(INT_DIR)/,$(OBJ_FILES))
OBJ_FILES       := $(subst ./,$(INT_DIR)/,$(OBJ_FILES))
OBJ_FILES       := $(subst $(BSP_DIR)/,$(INT_DIR)/,$(OBJ_FILES))

# Dependency files
DEP_FILES       := $(OBJ_FILES:.o=.d)

# Make commands
all: $(BIN) show.options
	@echo "+++ Toolchain"
	@$(CC) --version | head -n 1

$(BIN): $(OBJ_FILES) $(LD_FILE)
	@echo "+++ Linking: $@"
	@mkdir -p $(BIN_DIR)
	@$(LD) -o $(BIN) -T$(LD_FILE) $(OBJ_FILES) $(LD_FLAGS)
	@$(OBJDUMP) -t $(BIN) > $(BIN:.elf=.sym)
	@$(OBJCOPY) -O binary $(BIN) $(BIN:.elf=.bin)
	@$(OBJCOPY) -O ihex $(BIN) $(BIN:.elf=.hex)
	@$(OBJDUMP) -t $(BIN) > $(BIN:.elf=.sym)
	@echo "+++ Binary summary: $(BIN)"
	@-$(SIZE) $(BIN)
	@echo "+++ Section summary: $(BIN:.elf=.map)"
	@grep ^.text $(BIN:.elf=.map) | awk '{print "    " $$3 "\t" $$1}'
	@grep ^.bss  $(BIN:.elf=.map) | awk '{print "    " $$3 "\t" $$1}'
	@grep ^.data $(BIN:.elf=.map) | awk '{print "    " $$3 "\t" $$1}'

$(INT_DIR)/%.o: $(ROOT_DIR)/%.c
	@mkdir -p $(dir $@)
	@echo "+++ Compiling: $<";
	@$(CC) $(C_FLAGS) -DMODULE_ID=$(call FILE_HASH,$<) -c -o $@ $<
	@$(if $(DEP),$(DEP) $(DEP_FLAGS) $(subst .o,.d,$@) -MP -MT $@ $<,)

$(INT_DIR)/%.o: $(ROOT_DIR)/%.s
	@mkdir -p $(dir $@)
	@echo "+++ Assembling: $<"
	@$(CC) $(C_FLAGS) -DMODULE_ID=$(call FILE_HASH,$<) -c -o $@ $<
	@$(if $(DEP),$(DEP) $(DEP_FLAGS) $(subst .o,.d,$@) -MP -MT $@ $<,)

$(INT_DIR)/thirdparty/%.o: $(ROOT_DIR)/thirdparty/%.s
	@echo "+++ Assembling thirdparty: $<"
	@mkdir -p $(dir $@)
	@$(CC) $(filter-out $(C_STD) -W%,$(C_FLAGS)) -c -o $@ $<
	@$(if $(DEP),$(DEP) $(DEP_FLAGS) $(subst .o,.d,$@) -MP -MT $@ $<,)

$(INT_DIR)/thirdparty/%.o: $(ROOT_DIR)/thirdparty/%.c
	@echo "+++ Compiling thirdparty: $<"
	@mkdir -p $(dir $@)
	@$(CC) $(filter-out $(C_STD) -W%,$(C_FLAGS)) -c -o $@ $<
	@$(if $(DEP),$(DEP) $(DEP_FLAGS) $(subst .o,.d,$@) -MP -MT $@ $<,)

clean:
	@rm -rf $(INT_DIR)
	@rm -rf $(BIN_DIR)
	@rm -rf $(LIB_DIR)

show.options:
	@echo "+++ Build options"
	@echo "    PLATFORM         = $(PLATFORM)"
	@echo "    BSP_DIR          = $(BSP_DIR)"
	@echo "    RTOS             = $(RTOS)"
	@echo "    DEBUG            = $(DEBUG)"
	@echo "    TRACE            = $(TRACE)"
	@echo "    LD_FILE          = $(LD_FILE)"
	@echo "    CFG_DEV          = $(strip $(CFG_DEV))"

show.config:
	@for s in $(CFG_DEV); \
		do echo $$s; \
	done

show.includes:
	@for f in $(subst $(ROOT_DIR)/,,$(INC_DIRS)); \
		do echo $$f; \
	done

show.headers:
	@for f in $(subst $(ROOT_DIR)/,,$(sort $(shell find $(INC_DIRS) -name *.h -print))); \
		do echo $$f; \
	done

show.sources:
	@for f in $(subst $(ROOT_DIR)/,,$(sort $(C_FILES) $(wildcard $(addsuffix *.h,$(dir $(C_FILES)))))); \
		do echo $$f; \
	done

show.platform:
	@echo $(PLATFORM_LIST)

show.rtos:
	@echo $(RTOS_LIST)

# Include dependency files for next build
-include $(DEP_FILES)

.PHONY: all clean
.PHONY: show.options show.includes show.sources show.platform show.rtos