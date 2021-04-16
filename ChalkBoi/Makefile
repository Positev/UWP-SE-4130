###
# Purpose: to create a bare-metal project with mbed SDK.

###
# GNU ARM Embedded Toolchain
CC=arm-none-eabi-gcc
CXX=arm-none-eabi-g++
LD=arm-none-eabi-ld
AR=arm-none-eabi-ar
AS=arm-none-eabi-as
CP=arm-none-eabi-objcopy
OD=arm-none-eabi-objdump
NM=arm-none-eabi-nm
SIZE=arm-none-eabi-size
A2L=arm-none-eabi-addr2line

###
# Directory Structure
BINDIR=bin
INCDIR=Core\Inc
SRCDIR=Core\Src

###
# Find source files
ASM_SOURCES =  \
startup_stm32f413xx.s

C_SOURCES =  \
Core/Src/stm32f4xx_it.c \
Core/Src/stm32f4xx_hal_msp.c \
USB_HOST/App/usb_host.c \
USB_HOST/Target/usbh_conf.c \
USB_HOST/Target/usbh_platform.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_hcd.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_usb.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_adc.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dac.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dac_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dfsdm.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_fmpi2c.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_fmpi2c_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_fsmc.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_sram.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2s.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2s_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_qspi.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_ll_sdmmc.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_sd.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c \
Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c \
Core/Src/system_stm32f4xx.c \
Middlewares/ST/STM32_USB_Host_Library/Core/Src/usbh_core.c \
Middlewares/ST/STM32_USB_Host_Library/Core/Src/usbh_ctlreq.c \
Middlewares/ST/STM32_USB_Host_Library/Core/Src/usbh_ioreq.c \
Middlewares/ST/STM32_USB_Host_Library/Core/Src/usbh_pipes.c \
Middlewares/ST/STM32_USB_Host_Library/Class/CDC/Src/usbh_cdc.c


CXXSOURCES= \
Core/Src/main.cpp 

# Find header directories
INCLUDES= \
-IUSB_HOST/App \
-IUSB_HOST/Target \
-ICore/Inc \
-IDrivers/STM32F4xx_HAL_Driver/Inc \
-IDrivers/STM32F4xx_HAL_Driver/Inc/Legacy \
-IMiddlewares/ST/STM32_USB_Host_Library/Core/Inc \
-IMiddlewares/ST/STM32_USB_Host_Library/Class/CDC/Inc \
-IDrivers/CMSIS/Device/ST/STM32F4xx/Include \
-IDrivers/CMSIS/Include

# Find libraries
INCLUDES_LIBS= -lc -lm -lnosys
LINK_LIBS=
# Create object list
OBJECTS = $(addprefix $(BINDIR)/,$(notdir $(ASM_SOURCES:%.s=%.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES))) 

OBJECTS += $(addprefix $(BINDIR)/,$(notdir $(C_SOURCES:%.c=%.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))

OBJECTS += $(addprefix $(BINDIR)/,$(notdir $(CXXSOURCES:%.cpp=%.o)))
vpath %.cpp $(sort $(dir $(CXXSOURCES)))
# Define output files ELF & IHEX
BINELF=outp.elf
BINHEX=outp.hex

###
# MCU FLAGS
MCFLAGS= -mcpu=cortex-m4 -mthumb  -specs=nosys.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard
# COMPILE FLAGS
DEFS=-DUSE_HAL_DRIVER -DSTM32F413xx
CFLAGS=-c -g $(MCFLAGS) $(DEFS) $(INCLUDES)
CXXFLAGS=-c -g $(MCFLAGS) $(DEFS) $(INCLUDES) -std=c++11
# LINKER FLAGS
LDSCRIPT=STM32F413ZHTx_FLASH.ld
LDFLAGS =-T $(LDSCRIPT) $(MCFLAGS)  $(INCLUDES_LIBS) $(LINK_LIBS)

###
# Build Rules
.PHONY: all release release-memopt debug clean

all: release-memopt

release-memopt-blame: CFLAGS+=-g
release-memopt-blame: CXXFLAGS+=-g
release-memopt-blame: LDFLAGS+=-g -Wl,-Map=$(BINDIR)/output.map
release-memopt-blame: release-memopt
release-memopt-blame:
	@echo "Top 10 space consuming symbols from the object code ...\n"
	$(NM) -A -l -C -td --reverse-sort --size-sort $(BINDIR)/$(BINELF) | head -n10 | cat -n # Output legend: man nm
	@echo "\n... and corresponging source files to blame.\n"
	$(NM) --reverse-sort --size-sort -S -tx $(BINDIR)/$(BINELF) | head -10 | cut -d':' -f2 | cut -d' ' -f1 | $(A2L) -e $(BINDIR)/$(BINELF) | cat -n # Output legend: man addr2line

release-memopt: DEFS+=-DCUSTOM_NEW -DNO_EXCEPTIONS
release-memopt: CFLAGS+=-Os -ffunction-sections -fdata-sections -fno-builtin # -flto
release-memopt: CXXFLAGS+=-Os -fno-exceptions -ffunction-sections -fdata-sections -fno-builtin -fno-rtti # -flto
release-memopt: LDFLAGS+=-Os -Wl,-gc-sections --specs=nano.specs # -flto
release-memopt: release

debug: CFLAGS+=-g
debug: CXXFLAGS+=-g
debug: LDFLAGS+=-g
debug: release

release: $(BINDIR)/$(BINHEX)

$(BINDIR)/$(BINHEX): $(BINDIR)/$(BINELF)
	$(CP) -O ihex $< $@
	@echo "Objcopy from ELF to IHEX complete!\n"

##
# C++ linking is used.
#
# Change
#   $(CXX) $(OBJECTS) $(LDFLAGS) -o $@ to 
#   $(CC) $(OBJECTS) $(LDFLAGS) -o $@ if
#   C linker is required.
$(BINDIR)/$(BINELF): $(OBJECTS)
	$(CXX) $(OBJECTS) $(LDFLAGS) -o $@
	@echo "Linking complete!\n"
	$(SIZE) $(BINDIR)/$(BINELF)

$(BINDIR)/%.o: %.cpp
	$(CXX) -c $(CXXFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.cpp=.lst)) $< -o $@
	@echo "Compiled "$<"!\n"

$(BINDIR)/%.o: %.c
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@
	@echo "Compiled "$<"!\n"

$(BINDIR)/%.o: %.s
	$(CC) $(CFLAGS) $< -o $@
	@echo "Assambled "$<"!\n"

clean:
	rm -f $(OBJECTS) $(BINDIR)/$(BINELF) $(BINDIR)/$(BINHEX) $(BINDIR)/output.map

flash: all
	openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program $(BINDIR)/$(BINELF) verify reset exit"

#######################################
# dependencies
#######################################
-include $(wildcard $(BINDIR)/*.d)