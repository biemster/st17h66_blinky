##############################################################################
BUILD = build
BIN = Blinky
CMSIS_PATH = ../CMSIS_5/
##############################################################################
.PHONY: all directory clean size

CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
SIZE = arm-none-eabi-size

CFLAGS += -W -Wall --std=gnu99 -O3
CFLAGS += -Wno-parentheses
CFLAGS += -fno-diagnostics-show-caret
CFLAGS += -fdata-sections -ffunction-sections
CFLAGS += -funsigned-char -funsigned-bitfields
CFLAGS += -mcpu=cortex-m0 -mthumb -mthumb-interwork
CFLAGS += -MD -MP -MT $(BUILD)/$(*F).o -MF $(BUILD)/$(@F).d

LDFLAGS += -mcpu=cortex-m0 -mthumb -mthumb-interwork
LDFLAGS += -Wl,--gc-sections
LDFLAGS += -Wl,--script=./st17h66.ld
LDFLAGS += -Wl,--just-symbols=./bb_rom_sym_m0.gcc

INCLUDES += \
	-I$(CMSIS_PATH)/Device/ARM/ARMCM0/Include \
	-I$(CMSIS_PATH)/CMSIS/Core/Include

SRC = ./main.c

CFLAGS += $(INCLUDES)

OBJS = $(addprefix $(BUILD)/, $(notdir %/$(subst .c,.o, $(SRC))))
# $(info List of files : [${OBJS}])

all: directory $(BUILD)/$(BIN).elf $(BUILD)/$(BIN).hex $(BUILD)/$(BIN).bin size

$(BUILD)/$(BIN).elf: $(OBJS)
	@echo LD $@
	@$(CC) $(LDFLAGS) $(OBJS) $(LIBS) -o $@

$(BUILD)/$(BIN).hex: $(BUILD)/$(BIN).elf
	@echo OBJCOPY $@
	@$(OBJCOPY) -O ihex $^ $@
#@sed -i "130i $$(head -1 $@)" $@ # The flasher needs to know when .text starts after the jump table and global config

$(BUILD)/$(BIN).bin: $(BUILD)/$(BIN).elf
	@echo OBJCOPY $@
	@$(OBJCOPY) -O binary $^ $@

%.o:
	@echo CC $@
	@$(CC) $(CFLAGS) $(filter %/$(subst .o,.c,$(notdir $@)), $(SRC)) -c -o $@

directory:
	@mkdir -p $(BUILD)

size: $(BUILD)/$(BIN).elf
	@echo size:
	@$(SIZE) -t $^

clean:
	@echo clean
	@-rm -rf $(BUILD)

-include $(wildcard $(BUILD)/*.d)
