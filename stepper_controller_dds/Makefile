SRC += stepperctrl.c

OUTPUT = stepperctrl

ASF_ROOT = /home/duerrfk/local/asf-standalone-archive-3.29.0.41/xdk-asf-3.29.0

INCLUDES += -I.

INCLUDES += -I$(ASF_ROOT)/common/boards 
INCLUDES += -I$(ASF_ROOT)/common/services/clock 
INCLUDES += -I$(ASF_ROOT)/common/services/gpio 
INCLUDES += -I$(ASF_ROOT)/common/services/ioport 
INCLUDES += -I$(ASF_ROOT)/common/utils 
INCLUDES += -I$(ASF_ROOT)/sam/boards 
INCLUDES += -I$(ASF_ROOT)/sam/boards/arduino_due_x 
INCLUDES += -I$(ASF_ROOT)/sam/drivers/pio 
INCLUDES += -I$(ASF_ROOT)/sam/drivers/pmc 
INCLUDES += -I$(ASF_ROOT)/sam/utils 
INCLUDES += -I$(ASF_ROOT)/sam/utils/cmsis/sam3x/include 
INCLUDES += -I$(ASF_ROOT)/sam/utils/cmsis/sam3x/source/templates 
INCLUDES += -I$(ASF_ROOT)/sam/utils/header_files 
INCLUDES += -I$(ASF_ROOT)/sam/utils/preprocessor
INCLUDES += -I$(ASF_ROOT)/thirdparty/CMSIS/Include 
INCLUDES += -I$(ASF_ROOT)/thirdparty/CMSIS/Lib/GCC

ASF_SRC +=  $(ASF_ROOT)/sam/utils/cmsis/sam3x/source/templates/system_sam3x.c
ASF_SRC +=  $(ASF_ROOT)/sam/utils/cmsis/sam3x/source/templates/gcc/startup_sam3x.c 
ASF_SRC +=  $(ASF_ROOT)/sam/utils/cmsis/sam3x/source/templates/exceptions.c
ASF_SRC +=  $(ASF_ROOT)/sam/drivers/pmc/pmc.c
ASF_SRC +=  $(ASF_ROOT)/sam/drivers/pio/pio.c
ASF_SRC +=  $(ASF_ROOT)/sam/drivers/pio/pio_handler.c
ASF_SRC +=  $(ASF_ROOT)/common/utils/interrupt/interrupt_sam_nvic.c

CROSS = /usr/local/gcc-arm-none-eabi-5_2-2015q4/bin/arm-none-eabi-

OPTIMIZATION = -O0

CC = $(CROSS)gcc
LD = $(CROSS)g++
OBJCOPY = $(CROSS)objcopy

LINKER_SCRIPT = $(ASF_ROOT)/sam/utils/linker_scripts/sam3x/sam3x8/gcc/flash.ld

CFLAGS += --param max-inline-insns-single=500 -mcpu=cortex-m3 -mthumb 
CFLAGS += -fno-strict-aliasing -ffunction-sections -fdata-sections -std=gnu99
CFLAGS += -D BOARD=ARDUINO_DUE_X -D __SAM3X8E__ -D ARM_MATH_CM3=true 
CFLAGS += -D printf=iprintf -D scanf=iscanf
CFLAGS += -Wall $(OPTIMIZATION) $(INCLUDES)

LDFLAGS += -L$(ASF_ROOT)/thirdparty/CMSIS/Lib/GCC
LDFLAGS += -Wl,--entry=Reset_Handler -Wl,--cref  -mcpu=cortex-m3 -mthumb
LDFLAGS += -Wl,"-T$(LINKER_SCRIPT)" -Wl,--gc-sections 
LDFLAGS += -Wl,-Map=$(OUTPUT).map,--cref
LDFLAGS += -Wl,--check-sections -Wl,--unresolved-symbols=report-all 
LDFLAGS += -Wl,--warn-section-align 
LDFLAGS += -Wl,--warn-unresolved-symbols
LDFLAGS += -larm_cortexM3l_math -lm

OBJ = $(SRC:.c=.o)
ASF_OBJ = $(ASF_SRC:.c=.o)

all: $(OUTPUT).bin

$(OUTPUT).elf: $(OBJ) $(ASF_OBJ)
	$(LD) $(LDFLAGS) $(OBJ) $(ASF_OBJ) -o $@

$(OUTPUT).bin: $(OUTPUT).elf
	$(OBJCOPY) -O binary $< $@

.PHONY: clean
clean:
	rm -rf $(OBJ) $(ASF_OBJ) $(OUTPUT).elf $(OUTPUT).bin $(OUTPUT).map

