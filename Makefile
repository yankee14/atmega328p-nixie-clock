########
#### This top section contains variables which may change more often
#### than others. 
########

# See https://gcc.gnu.org/onlinedocs/gcc/AVR-Options.html for the
# list of AVR devices and/or instruction set architectures to
# assign to 'MCU'.
#
# Compiled objects, including the final binary .hex file, will
# be placed in a subdirectory titled build-'MCU' .
MCU = atmega328p

# Clock frequency of the CPU, after prescalers and other clock
# signal conditioning.
CPU_FREQ = 16000000UL

# Where are avr-libc headers such as <avr/io.h> located?
AVR_LIBC_DIR = /usr/lib/avr/include

# Which kind of in-system programmer (ISP) are you using?
AVRDUDE_PROGRAMMER_ID = arduino

# Symbols rate while reading and writing code to avr?
AVRDUDE_BAUD = 115200

# Which optimization setting should the compiler use?
# See: https://gcc.gnu.org/onlinedocs/gcc-4.8.1/gcc/Optimize-Options.html
OPTIMIZATION_LEVEL = fast

###################################################################
###################################################################
#### Edits below this line probably are not needed very often. ####
###################################################################
###################################################################

########
#### If no chip information given
########

ifndef MCU # stop if device or ISA not specified
    $(error No MCU specified.)
endif

ifndef CPU_FREQ # default to 1MHz unless otherwise specified
    CPU_FREQ = 1000000UL
endif

########
#### Avrdude setup
########

include ./Avrdude.mk

########
#### Compiler setup
########

ifndef OPTIMIZATION_LEVEL
    OPTIMIZATION_LEVEL = s
endif

CC = avr-gcc
#CXX = avr-g++ # TODO
OBJCOPY = avr-objcopy # for creating .hex from .elf
OBJDUMP = avr-objdump # for debugging

CFLAGS = -Wall -Wextra -Wpedantic -fdiagnostics-color -std=c99 \
		 -O$(OPTIMIZATION_LEVEL) -DF_CPU=$(CPU_FREQ) -mmcu=$(MCU) \
		 -I$(AVR_LIBC_DIR)

########
#### Files and directories
########

# Store objects and binaries in a subdirectory called "build_$(MCU)",
# e.g. ~/MyProject/src/build_atmega328p/
ifndef BUILD_DIR
    BUILD_DIR = ./build_$(MCU)
endif

# Specify avr-libc headers
AVR_LIBC = /usr/lib/avr/include

# Get list of all c files compiled
SRCS = $(wildcard *.c)

# List of objects to be created or updated
OBJS = $(BUILD_DIR)/$(SRCS:.c=.o)

# Name of built binary, not flashable
ELF = $(BUILD_DIR)/$(MCU).elf

# Name of built binary to be flashed
BINARY = $(BUILD_DIR)/$(MCU).hex

########
#### Build targets
########

# Shell commands used
MKDIR	= mkdir -p
RMDIR 	= $(RM) -r
PRINT   = printf

.DEFAULT_GOAL := all

##
# The final *.hex binary to be flashed,
# depends on the not-flashable *.elf binary.
#
# 1. Deletes previous *.hex, if any.
# 2. Recreates *.hex from *.elf using avr-objcopy.
.PHONY: all
all: $(BINARY)
$(BINARY): $(ELF)
	$(RM) $(BINARY)
	$(OBJCOPY) --only-section .text --only-section .data --output-target ihex $(ELF) $(BINARY)

##
# Link up the final *.elf binary,
# depends on the *.o object files compiled by avr-gcc.
#
# 1. Link compiled objects into *.elf binary.
#
$(ELF): $(OBJS)
	$(CC) $(CFLAGS) -o $(ELF) $(OBJS)

##
# Create *.o objects,
# depends on the *.c files the user has written.
#
# 1. Ensure the build directory exists.
# 2. Create the objects from the sources.
#
$(OBJS): $(SRCS)
	$(MKDIR) $(BUILD_DIR)
	$(CC) $(CFLAGS) -c $< -o $@

##
# Invoke avrdude to upload the *.hex to the avr,
# depends on everything being built.
#
# 1. Upload the hex file to the avr.
#
.PHONY: upload
upload: all
	$(AVRDUDE) $(AVRDUDE_FLAGS) -U flash:w:$(BUILD_DIR)/$(MCU).hex:i

# fuse does not require a binary to flash 
# TODO fuse seems to be broken...
#.PHONY: fuse
#fuse:
#	$(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_FUSE_FLAGS)

##
# Generate avr assembly and place in text files *.asm and *.s,
# depends on the *.elf existing.
#
# 1. Generate *.asm from *.elf using avr-objdump.
# 2. Generate *.s from *.c using avr-gcc.
#
.PHONY: assembly
assembly: $(ELF)
	$(OBJDUMP) --disassemble $(ELF) > $(ELF:.elf=.asm)
	$(CC) $(CFLAGS) -S $(SRCS) -o $(BUILD_DIR)/$(SRCS:.c=.s)

##
# Reset the build directory to default.
#
# 1. Delete the entire build directory.
#
.PHONY: clean
clean:
	$(RMDIR) $(BUILD_DIR)


##
# Print information about valid targets to the prompt.
#
.PHONY: help
help:
	@$(PRINT) "\nAvailable build targets:\n\
 make          - compile only.\n\
 make upload   - invoke avrdude, upload code to board.\n\
 #make fuse     - BROKEN set fuses. See BTLDR chapter in datasheet.\n\
 make assembly - avr assembly for debugging.\n\
 make clean    - delete the build subdirectory.\n\
 make help     - show this help.\n\n"

