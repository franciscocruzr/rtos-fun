####################################################################################################
# @file   install.mk
#
# @brief  stm32f4 platform makefile install.
####################################################################################################

# Target specific install
STMPROG 		:= ST-LINK_CLI -c SWD
BASE_ADDR		:= 0x08000000

# Displace address for bootloader
BASE_ADDR   := $(shell printf "0x%X" $$(($(BASE_ADDR) + $(BOOTLOADER_SIZE))))

# Make commands
program: erase
	@$(STMPROG) -P $(BIN:.elf=.bin) $(BASE_ADDR) -V -Rst

erase:
	@$(STMPROG) -ME

reset:
	@$(STMPROG) -Rst

.PHONY: program erase reset