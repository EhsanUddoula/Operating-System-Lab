# Makefile for DUOS Bootloader

# MCU and Tools
MCU = stm32f446re
CC = arm-none-eabi-gcc
MACH = cortex-m4
FPU = -mfloat-abi=softfp -mfpu=fpv4-sp-d16
OBJECT = object
BOOTLOADER = bootloader
MAPDIR = mapfiles
TARGET_DIR = $(BOOTLOADER)

# Bootloader Source Files
BOOT_SOURCES = bootloader.c cm4.c sys_clock.c
BOOT_OBJS = $(addprefix $(OBJECT)/, $(BOOT_SOURCES:.c=.o))

# Paths and Directories
INCLUDE = -I$(BOOTLOADER)

# Flags
CFLAGS = -c -mcpu=$(MACH) -mthumb $(FPU) -std=gnu11 -Wall -O0
LDFLAGS = -nostdlib -nostartfiles -nodefaultlibs -fno-exceptions \
          -mcpu=$(MACH) -mthumb $(FPU) -T $(PWD)/bootloader.ld \
          -Wl,-Map=$(MAPDIR)/bootloader.map -O0

# Targets
.PHONY: all clean load

# Default target to build everything
all: directories bootloader

# Create necessary directories if they don't exist
directories:
	mkdir -p $(OBJECT) $(MAPDIR) $(TARGET_DIR)

# Object file compilation rule with bootloader path explicitly specified
$(OBJECT)/bootloader.o: bootloader.c
	$(CC) $(CFLAGS) $(INCLUDE) $< -o $@

$(OBJECT)/cm4.o: cm4.c
	$(CC) $(CFLAGS) $(INCLUDE) $< -o $@

$(OBJECT)/sys_clock.o: sys_clock.c
	$(CC) $(CFLAGS) $(INCLUDE) $< -o $@


bootloader: $(OBJECT)/bootloader.o \
	$(OBJECT)/cm4.o \
	$(OBJECT)/sys_clock.o

	$(CC) $(LDFLAGS) $^ -o $(TARGET_DIR)/$@

# Linking the bootloader ELF and creating the map file
final.elf:  $(OBJECT)/bootloader.o \
	$(OBJECT)/cm4.o \
	$(OBJECT)/sys_clock.o

	$(CC) $(LDFLAGS) $^ -o build/$@

# Clean the build directories and files
clean:
	rm -rf $(OBJECT)/*.o $(MAPDIR)/*.map $(TARGET_DIR)/*.elf

# OpenOCD command to load the bootloader
load:
	openocd -f /usr/share/openocd/scripts/board/stm32f4discovery.cfg -f /usr/share/openocd/scripts/interface/stlink.cfg -c "program $(TARGET_DIR)/bootloader verify reset exit"
