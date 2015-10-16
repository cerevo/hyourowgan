TARGET=hyourowgan
DFP_VER=1.35.0

ifeq ($(DFP_VER),)
	DFP_PATH = $(SDK_DIR)TOSHIBA.TZ10xx_DFP
else
	DFP_PATH = $(SDK_DIR)TOSHIBA.TZ10xx_DFP.$(DFP_VER)
endif

#Directories
TOOL_DIR ?= /opt/cross/bin/
SDK_DIR  ?= /opt/TZ10xx_SUPPORT/
BLD_DIR = build/

#Commands
CC      = $(TOOL_DIR)arm-none-eabi-gcc
LD      = $(TOOL_DIR)arm-none-eabi-gcc
OBJCPY  = $(TOOL_DIR)arm-none-eabi-objcopy

#Flags
CFLAGS  = -mcpu=cortex-m4 -mthumb -mthumb-interwork -march=armv7e-m -mfloat-abi=softfp -mfpu=fpv4-sp-d16 --specs=tz10xx.specs -std=c99 -g -O0
LDFLAGS = -mcpu=cortex-m4 -mthumb -mthumb-interwork -march=armv7e-m -mfloat-abi=softfp -mfpu=fpv4-sp-d16 --specs=tz10xx.specs -T $(DFP_PATH)/Device/Source/GCC/gcc_TZ10xx.ld -Wl,-Map=$(BLD_DIR)$(TARGET).map
LIBS    = 

#Include dir, Source files.
INCLUDE = -IRTE -IRTE/Device/TZ1001MBG
SOURCES = src/main.c src/ble.c src/pwm_out.c
include *.mk

#Objects
_SRCS   = $(subst $(SDK_DIR), _SDK/, $(SOURCES))
OBJS    = $(addprefix $(BLD_DIR), $(patsubst %.c,%.o,$(_SRCS)))
BOOTLDR = $(BLD_DIR)bootloader_TZ10xx.o
STARTUP = $(BLD_DIR)startup_TZ10xx.o
#Depends
DEPS    = $(OBJS:%.o=%.d)


$(TARGET).bin: $(OBJS) $(BOOTLDR) $(STARTUP)
	$(LD) $(LDFLAGS) -o $(TARGET).elf $(OBJS) $(BOOTLDR) $(STARTUP) $(LIBS)
	$(OBJCPY) -O binary $(TARGET).elf $(TARGET).bin

$(BOOTLDR): $(DFP_PATH)/Device/Source/GCC/bootloader_TZ10xx.S 
	$(CC) $(CFLAGS) -c -o $@ $<

$(STARTUP): $(DFP_PATH)/Device/Source/GCC/startup_TZ10xx.S 
	$(CC) $(CFLAGS) -c -o $@ $<

$(BLD_DIR)_SDK/%.o: $(SDK_DIR)%.c
	mkdir -p $(dir $@)
	$(CC) -c -MMD -MP $(CFLAGS) $(INCLUDE) -o $@ $<

$(BLD_DIR)%.o: %.c
	mkdir -p $(dir $@)
	$(CC) -c -MMD -MP $(CFLAGS) $(INCLUDE) -o $@ $<

clean:
	@echo $(DFP_PATH)
	rm -rf $(BLD_DIR)
	rm -f $(TARGET).elf $(TARGET).bin

.PHONY: clean

-include $(DEPS)
