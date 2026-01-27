# Toolchain
CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
SIZE = arm-none-eabi-size
GDB = arm-none-eabi-gdb

# Project name
TARGET = program

# Source files
ASM_SOURCES = startup.s
C_SOURCES = main.c

# Object files
OBJECTS = $(ASM_SOURCES:.s=.o) $(C_SOURCES:.c=.o)

# Compiler flags
CFLAGS = -Og -g3
CFLAGS += -mthumb -mcpu=cortex-m3
CFLAGS += -Wall
CFLAGS += --specs=nosys.specs -nostdlib

# Linker flags
LDFLAGS = -T linker.inv
LDFLAGS += -Wl,-Map=$(TARGET).map

# OpenOCD configuration
OPENOCD = openocd
OPENOCD_INTERFACE = interface/stlink.cfg
OPENOCD_TARGET = target/stm32f1x.cfg
OPENOCD_EXTRA = -c "set CPUTAPID 0x1ba01477"

# Build rules
all: $(TARGET).elf $(TARGET).bin

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

%.o: %.s
	$(CC) $(CFLAGS) -c $< -o $@

$(TARGET).elf: $(OBJECTS)
	$(CC) $(CFLAGS) $(OBJECTS) $(LDFLAGS) -o $@
	$(SIZE) $@

$(TARGET).bin: $(TARGET).elf
	$(OBJCOPY) -O binary $< $@

# Flash firmware
flash: $(TARGET).elf
	$(OPENOCD) $(OPENOCD_EXTRA) -f $(OPENOCD_INTERFACE) -f $(OPENOCD_TARGET) \
		-c "program $(TARGET).elf verify reset exit"

# Start OpenOCD server for debugging
openocd:
	$(OPENOCD) $(OPENOCD_EXTRA) -f $(OPENOCD_INTERFACE) -f $(OPENOCD_TARGET)

# Clean build files
clean:
	rm -f $(TARGET).elf $(TARGET).bin $(TARGET).map $(OBJECTS)

# Debug with GDB
debug: $(TARGET).elf
	$(GDB) $(TARGET).elf -ex "target remote localhost:3333"

.PHONY: all flash openocd clean debug
