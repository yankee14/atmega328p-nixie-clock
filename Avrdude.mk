########
#### Makefile avrdude setup
########

## DETERMINE FUSE SETTINGS
## Attempts to define sane defaults depending on MCU in use.

ifndef AVRDUDE_LOW_FUSE
    ifeq ($(MCU),atmega328p)
        AVRDUDE_LOW_FUSE = lfuse:w:0xFF:m
    endif
endif

ifndef AVRDUDE_HIGH_FUSE
    ifeq ($(MCU),atmega328p) # 
        AVRDUDE_HIGH_FUSE = hfuse:w:0xDE:m
    endif
endif

ifndef AVRDUDE_EXT_FUSE
    ifeq ($(MCU),atmega328p)
        AVRDUDE_EXT_FUSE = efuse:w:0x05:m
    endif
endif

ifndef AVRDUDE # Default 'avrdude' executable
    AVRDUDE = avrdude
endif

ifndef AVRDUDE_BAUD # Default to 9600 baud
    AVRDUDE_BAUD = 9600
endif

ifndef AVRDUDE_PROGRAMMER_ID # Default to "Uno" programmer
    AVRDUDE_PROGRAMMER_ID = arduino
endif

ifndef AVRDUDE_PORT
    AVRDUDE_PORT = /dev/ttyACM0
endif

ifndef AVRDUDE_PARTNO
    AVRDUDE_PARTNO = m328p
endif

AVRDUDE_FLAGS = -v -b $(AVRDUDE_BAUD) -c $(AVRDUDE_PROGRAMMER_ID)\
			   	-p $(AVRDUDE_PARTNO) -P $(AVRDUDE_PORT)
AVRDUDE_FUSE_FLAGS = -U $(AVRDUDE_LOW_FUSE)\
					 -U $(AVRDUDE_HIGH_FUSE)\
					 -U $(AVRDUDE_EXT_FUSE)

