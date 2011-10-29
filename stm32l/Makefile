EXECUTABLE=probot.elf
BIN_IMAGE=probot.bin

CC=arm-none-eabi-gcc
OBJCOPY=arm-none-eabi-objcopy

CFLAGS=-O3 -mlittle-endian -mthumb -Werror
ifeq ($(CONFIG_STM32L_DISCOVERY), 1)
	CFLAGS+=-mcpu=cortex-m3 -DCONFIG_STM32L_DISCOVERY=1
else ifeq ($(CONFIG_STM32VL_DISCOVERY), 1)
	CFLAGS+=-mcpu=cortex-m3 -DCONFIG_STM32VL_DISCOVERY=1
else ifeq ($(CONFIG_STM32F4_DISCOVERY), 1)
	CFLAGS+=-mcpu=cortex-m4 -DCONFIG_STM32F4_DISCOVERY=1	
else
# default to stm32l
	CFLAGS+=-mcpu=cortex-m3 -DCONFIG_STM32L_DISCOVERY=1
endif
CFLAGS+=-ffreestanding -nostdlib -nostdinc

# to run from FLASH
CFLAGS+=-Wl,-T,stm32_flash.ld

# stm32l_discovery lib
LIBSTM32_DIR=$(HOME)/repo/stlink/example/libstm32l_discovery
CFLAGS+=-I$(LIBSTM32_DIR)/inc
CFLAGS+=-I$(LIBSTM32_DIR)/inc/base
CFLAGS+=-I$(LIBSTM32_DIR)/inc/core_support
CFLAGS+=-I$(LIBSTM32_DIR)/inc/device_support

all: $(BIN_IMAGE)

$(BIN_IMAGE): $(EXECUTABLE)
	$(OBJCOPY) -O binary $^ $@

$(EXECUTABLE): main.c system_stm32l1xx.c startup_stm32l1xx_md.s
	$(CC) $(CFLAGS) $^ -o $@  -L$(LIBSTM32_DIR)/build -lstm32l_discovery

clean:
	rm -rf $(EXECUTABLE)
	rm -rf $(BIN_IMAGE)

write: all
	sudo $(HOME)/repo/stlink/flash/flash write ./probot.bin 0x08000000

.PHONY: all clean write
