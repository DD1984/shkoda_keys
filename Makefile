######################################
TOPDIR := $(shell pwd)
######################################
# target
######################################
TARGET = shkoda_keys

######################################
# Posibilities
######################################

######################################
# MCU
######################################
MCU = -mthumb -mcpu=cortex-m3

######################################
# building variables
######################################
# debug build?
DEBUG = 1
# optimization
OPT = -Os

#######################################
# pathes
#############################/src##########
# source path

######################################
# source
######################################
C_SOURCES :=
CPP_SOURCES :=

#hal
C_SOURCES += CMSIS/Device/ST/STM32F1xx/Source/system_stm32f1xx.c
C_SOURCES += hal/Src/stm32f1xx_hal.c
C_SOURCES += hal/Src/stm32f1xx_hal_cortex.c
C_SOURCES += hal/Src/stm32f1xx_hal_rcc.c
C_SOURCES += hal/Src/stm32f1xx_hal_rcc_ex.c
C_SOURCES += hal/Src/stm32f1xx_hal_uart.c
C_SOURCES += hal/Src/stm32f1xx_hal_gpio.c
C_SOURCES += hal/Src/stm32f1xx_hal_gpio_ex.c
C_SOURCES += hal/Src/stm32f1xx_hal_tim.c
C_SOURCES += hal/Src/stm32f1xx_hal_tim_ex.c
C_SOURCES += hal/Src/stm32f1xx_hal_adc.c
C_SOURCES += hal/Src/stm32f1xx_hal_adc_ex.c
C_SOURCES += hal/Src/stm32f1xx_hal_dma.c
C_SOURCES += hal/Src/stm32f1xx_hal_flash.c
C_SOURCES += hal/Src/stm32f1xx_hal_flash_ex.c

C_SOURCES += hal/Src/stm32f1xx_ll_usb.c
C_SOURCES += hal/Src/stm32f1xx_hal_pcd.c
C_SOURCES += hal/Src/stm32f1xx_hal_pcd_ex.c

C_SOURCES += hal/stm32f1xx_hal_msp.c
C_SOURCES += stm32f1xx_it.c
C_SOURCES += syscalls.c

C_SOURCES += STM32_USB_Device_Library/Core/Src/usbd_core.c
C_SOURCES += STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
C_SOURCES += STM32_USB_Device_Library/Core/Src/usbd_ioreq.c
C_SOURCES += STM32_USB_Device_Library/Class/CustomHID/Src/usbd_customhid.c


C_SOURCES += usbd_conf.c
C_SOURCES += usbd_desc.c
C_SOURCES += usbd_custom_hid_if.c

C_SOURCES += main.c

#asm
ASM_SOURCES = CMSIS/Device/ST/STM32F1xx/Source/startup_stm32f103x6.s

#######################################
# binaries
#######################################
CC = arm-none-eabi-gcc
AS = arm-none-eabi-gcc
CP = arm-none-eabi-objcopy
AR = arm-none-eabi-ar
SZ = arm-none-eabi-size
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S
 
#######################################
# CFLAGS
#######################################
# macros for gcc
AS_DEFS = 
C_DEFS = -DSTM32F103x6
# includes for gcc
AS_INCLUDES = 
C_INCLUDES = -I . -I hal -I hal/Inc -I CMSIS/Include -I CMSIS/Device/ST/STM32F1xx/Include -I STM32_USB_Device_Library/Class/CustomHID/Inc

#for usb
C_INCLUDES += -I STM32_USB_Device_Library/Core/Inc
 
# compile gcc flags
ASFLAGS = $(MCU) $(AS_DEFS) $(AS_INCLUDES) $(OPT) -Wall -fdata-sections -ffunction-sections
CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) $(OPT) -fdata-sections -ffunction-sections
ifeq ($(DEBUG), 1)
CFLAGS += -g -gdwarf-2
endif
# Generate dependency information
CFLAGS += -MD -MP -MF .dep/$(@F).d

#######################################
# LDFLAGS
#######################################
# link script
LDSCRIPT = CMSIS/Device/ST/STM32F1xx/Source/STM32F103X6_FLASH.ld
# libraries
LIBS =
LIBDIR =
LDFLAGS = $(MCU) --specs=nano.specs -T$(LDSCRIPT) $(LIBDIR) $(LIBS) -Wl,-Map=$(TARGET).map,--cref -Wl,--gc-sections

# default action: build all
all: $(TARGET).elf $(TARGET).hex $(TARGET).bin

#######################################
# build the application
#######################################
# list of objects
#c
OBJECTS = $(C_SOURCES:.c=.o)
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(ASM_SOURCES:.s=.o)
vpath %.s $(sort $(dir $(ASM_SOURCES)))

%.o: %.c
	$(CC) -c $(CFLAGS) $< -o $@

%.o: %.s
	$(AS) -c $(CFLAGS) $< -o $@

$(TARGET).elf: $(OBJECTS)
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
	$(SZ) $@

%.hex: %.elf
	$(HEX) $< $@
	
%.bin: %.elf
	$(BIN) $< $@	
	
#######################################
# clean up
#######################################
clean:
	-rm -fR .dep $(OBJECTS) $(TARGET).elf $(TARGET).bin $(TARGET).map $(TARGET).hex
  
#######################################
# dependencies
#######################################
-include $(shell mkdir .dep 2>/dev/null) $(wildcard .dep/*)

# *** EOF ***
