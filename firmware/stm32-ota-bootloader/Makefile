PROJECT := spot-ota-bootloader
STM32_ROOT := ../stm32-learning
BUILD := build
PREFIX ?= arm-none-eabi-
CC := $(PREFIX)gcc
OBJCOPY := $(PREFIX)objcopy
SIZE := $(PREFIX)size
CFLAGS := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard \
          -std=c11 -Os -ffunction-sections -fdata-sections -Wall -Wextra \
          -Werror -DSTM32F446xx -I$(STM32_ROOT)/Drivers/CMSIS/Device/ST/STM32F4xx/Include \
          -I$(STM32_ROOT)/Drivers/CMSIS/Include
LDFLAGS := -Tbootloader.ld -nostartfiles --specs=nosys.specs \
           -Wl,--gc-sections -Wl,-Map=$(BUILD)/$(PROJECT).map
SOURCES := bootloader.c $(STM32_ROOT)/Startup/startup_stm32f446retx.s
OBJECTS := $(BUILD)/bootloader.o $(BUILD)/startup.o

all: $(BUILD)/$(PROJECT).elf $(BUILD)/$(PROJECT).bin

factory: all
	python3 make_factory_image.py $(BUILD)/$(PROJECT).bin \
		../stm32-learning/Debug/stm32-learning.bin $(BUILD)/spot-factory.bin

$(BUILD):
	mkdir -p $(BUILD)

$(BUILD)/bootloader.o: bootloader.c | $(BUILD)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/startup.o: $(STM32_ROOT)/Startup/startup_stm32f446retx.s | $(BUILD)
	$(CC) $(CFLAGS) -x assembler-with-cpp -c $< -o $@

$(BUILD)/$(PROJECT).elf: $(OBJECTS) bootloader.ld
	$(CC) $(CFLAGS) $(OBJECTS) $(LDFLAGS) -o $@
	$(SIZE) $@

$(BUILD)/$(PROJECT).bin: $(BUILD)/$(PROJECT).elf
	$(OBJCOPY) -O binary $< $@

clean:
	rm -rf $(BUILD)

.PHONY: all clean factory
