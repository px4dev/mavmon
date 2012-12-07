
#
# Application sources
#
SRCS		 = main.cpp \
		   board_fld_v2.c
INCDIRS		 =

#
# Serial device
#
UPLOAD_DEV	 = /dev/cu.usbserial-00001114B

#
# Build this
#
PRODUCT		 = mavmon
SUFFIXES	 = .bin .elf

#
# Toolchain
#
PREFIX		 = arm-none-eabi-
CC		 = $(PREFIX)gcc
AS		 = $(PREFIX)gcc
CXX		 = $(PREFIX)g++
LD		 = $(PREFIX)g++
OBJCOPY		 = $(PREFIX)objcopy
ARCH_FLAGS	 = -mthumb -mcpu=cortex-m3 -msoft-float

EXTRA_CFLAGS	 =
EXTRA_DEFINES	 =

#
# libopencm3
#
LCM3		 = ./libopencm3
LCM3_URL	 = https://github.com/libopencm3/libopencm3.git
ifeq ($(wildcard $(LCM3)),)
R		:= $(shell git clone $(LCM3_URL))
endif
LCM3_LIB	 = opencm3_stm32f1
LCM3_TARGETS	 = stm32/f1
INCDIRS		+= $(LCM3)/include
EXTRA_DEFINES	+= -DSTM32F1


#
# scmRTOS
#
SCMRTOS		 = ./scmRTOS
SRCS		+= $(wildcard $(SCMRTOS)/Common/*.cpp) \
		   $(wildcard $(SCMRTOS)/CortexM3/*.cpp) \
		   $(wildcard $(SCMRTOS)/CortexM3/*.S)
INCDIRS		+= $(SCMRTOS)/Common \
		   $(SCMRTOS)/CortexM3 \
		   $(SCMRTOS)
EXTRA_DEFINES	+= -DSTM32F10X_MD

#
# U8glib
#
U8GLIB		 = ./u8glib
U8GLIB_URL	 = https://code.google.com/p/u8glib/
ifeq ($(wildcard $(U8GLIB)),)
R		:= $(shell hg clone $(U8GLIB_URL))
endif
U8GLIB_EXCLUDE	 = $(U8GLIB)/csrc/chessengine.c \
		   $(U8GLIB)/csrc/u8g_com_api_16% \
		   $(U8GLIB)/csrc/u8g_com_arduino% \
		   $(U8GLIB)/csrc/u8g_com_atmega% \
		   $(U8GLIB)/csrc/u8g_com_i2c% \
		   $(U8GLIB)/csrc/u8g_com_io% \
		   $(U8GLIB)/csrc/u8g_com_null% \
		   $(U8GLIB)/csrc/u8g_delay.c \
		   $(U8GLIB)/csrc/u8g_dev_% \
		   $(U8GLIB)/csrc/u8g_pb14% \
		   $(U8GLIB)/csrc/u8g_pb16% \
		   $(U8GLIB)/csrc/u8g_pb8h% \
		   $(U8GLIB)/csrc/u8g_pb8v2%
U8GLIB_SRCS	 = $(wildcard $(U8GLIB)/csrc/*.c)
SRCS		+= $(filter-out $(U8GLIB_EXCLUDE),$(U8GLIB_SRCS))
SRCS		+= $(wildcard $(U8GLIB)/sfntsrc/*.c)
INCDIRS		+= $(U8GLIB)/csrc
EXTRA_CFLAGS	+= -Wno-unused

#
# m2tklib
#
M2TKLIB		 = ./m2tklib
M2TKLIB_URL	 = https://code.google.com/p/m2tklib/
ifeq ($(wildcard $(M2TKLIB)),)
R		:= $(shell hg clone $(M2TKLIB_URL))
endif

#
# stm32flash
#
STM32FLASH	 = ./stm32flash
STM32FLASH_URL	 = git://gitorious.org/stm32flash/stm32flash.git
ifeq ($(wildcard $(STM32FLASH)),)
R		:= $(shell git clone $(STM32FLASH_URL))
endif
UPLOADER	 = $(STM32FLASH)/stm32flash

#
# Build controls
#
OBJS		 = $(addsuffix .o,$(basename $(SRCS)))
DEPS		 = $(OBJS:.o=.d)
GLOBAL_DEPS	 = $(MAKEFILE_LIST)

CFLAGS		 = $(ARCH_FLAGS) \
		   -Os -g \
		   -Wall -Wextra \
		   -fno-common \
		   -ffunction-sections -fdata-sections \
		   -MD \
		   $(addprefix -I,$(INCDIRS)) \
		   $(EXTRA_DEFINES) \
		   $(EXTRA_CFLAGS)

CXXFLAGS	 = $(ARCH_FLAGS) \
		   -Os -g \
		   -Wall -Wextra \
		   -fno-common \
		   -ffunction-sections -fdata-sections \
		   -fno-exceptions \
		   -fno-rtti \
		   -fno-threadsafe-statics \
		   -funsigned-bitfields \
		   -fshort-enums \
		   -Wpointer-arith \
		   -Wredundant-decls \
		   -Wshadow \
		   -Wcast-qual \
		   -Wcast-align \
		   -MD \
		   $(addprefix -I,$(INCDIRS)) \
		   -DSTM32F1 \
		   $(EXTRA_CXXFLAGS)

ASFLAGS		 = $(ARCH_FLAGS) \
		   -g \
		   $(addprefix -I,$(INCDIRS)) \
		   -D__ASSEMBLER__

LDSCRIPT	 = stm32f103rbt6.ld
LDFLAGS		 = -l $(LCM3_LIB) \
		   --static \
		   -L $(LCM3)/lib \
		   -T $(LDSCRIPT) \
		   -nostartfiles \
		   -nostdlib \
		   -Wl,--start-group \
		   -lnosys -lc -lgcc \
		   -Wl,--end-group \
		   -Wl,--gc-sections \
		   $(ARCH_FLAGS) \
		   -Wl,-Map=$(PRODUCT).map,--cref \
		   -v


# Build debugging
ifeq ($(V),)
Q		 = @
endif

#
# Rules
#
PRODUCTS	 = $(addprefix $(PRODUCT),$(SUFFIXES))
all: $(PRODUCTS)

upload: $(PRODUCT).bin $(UPLOADER)
	@echo UPLOAD $<
	$(Q) $(UPLOADER) -w $< $(UPLOAD_DEV)

$(PRODUCT).elf: $(OBJS) $(LDSCRIPT) $(GLOBAL_DEPS) $(LCM3)
	@echo LD $@
	$(Q) $(LD) -o $@ $(OBJS) $(LDFLAGS)

%.o: %.c $(GLOBAL_DEPS)
	@echo CC $@
	$(Q) $(CC) $(CFLAGS) -o $@ -c $<

%.o: %.cpp $(GLOBAL_DEPS)
	@echo CXX $@
	$(Q) $(CXX) $(CXXFLAGS) -o $@ -c $<

%.o: %.S $(GLOBAL_DEPS)
	@echo AS $@
	$(Q) $(AS) $(ASFLAGS) -o $@ -c $<

%.bin: %.elf
	@echo BIN $@
	$(Q) $(OBJCOPY) -O binary $^ $@

$(SRCS): $(LCM3)

.PHONY: $(LCM3)
$(LCM3):
	$(Q) make -C $(LCM3) TARGETS=$(LCM3_TARGETS) lib

$(UPLOADER):
	$(Q) make -C $(STM32FLASH)

.PHONY: clean
clean:
	$(Q) rm -f $(OBJS) $(DEPS) $(PRODUCTS) $(PRODUCT).map

.PHONY: reallyclean
reallyclean: clean
	$(Q) make -C $(LCM3) TARGETS=$(LCM3_TARGETS) clean

-include $(DEPS)
