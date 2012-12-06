
#
# Sources
#
SRCS		 = main.c \
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

CFLAGS		 = $(ARCH_FLAGS) \
		   -Os -g \
		   -Wall -Wextra \
		   -fno-common \
		   -ffunction-sections -fdata-sections \
		   -MD \
		   $(addprefix -I,$(INCDIRS)) \
		   -DSTM32F1

LDSCRIPT	 = stm32f103rbt6.ld
LDFLAGS		 = $(ARCH_FLAGS) \
		   --static \
		   -nostartfiles \
		   -L $(LCM3)/lib \
		   -L $(LCM3)/lib/stm32/f1 \
		   -l $(LCM3_LIB) \
		   -Wl,--start-group \
		   -lc -lgcc -lnosys \
		   -Wl,--end-group \
		   -Wl,--gc-sections \
		   -T $(LDSCRIPT) \
		   -T $(LCM3_LD)

#
# Toolchain
#
PREFIX		 = arm-none-eabi-
CC		 = $(PREFIX)gcc
OBJCOPY		 = $(PREFIX)objcopy
ARCH_FLAGS	 = -mthumb -mcpu=cortex-m3 -msoft-float
STM32FLASH	 = ../stm32flash/stm32flash

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
LCM3_LD		 = $(LCM3)/lib/libopencm3_stm32f1.ld
INCDIRS		+= $(LCM3)/include

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
CFLAGS		+= -Wno-unused

#
# m2tklib
#
M2TKLIB		 = ./m2tklib
M2TKLIB_URL	 = https://code.google.com/p/m2tklib/
ifeq ($(wildcard $(M2TKLIB)),)
R		:= $(shell hg clone $(M2TKLIB_URL))
endif

#
# Build controls
#
OBJS		 = $(SRCS:.c=.o)
DEPS		 = $(OBJS:.o=.d)
GLOBAL_DEPS	 = $(MAKEFILE_LIST)

# Build debugging
ifeq ($(V),)
Q		 = @
endif

#
# Rules
#
PRODUCTS	 = $(addprefix $(PRODUCT),$(SUFFIXES))
all: $(PRODUCTS)

upload: $(PRODUCT).bin
	@echo UPLOAD $<
	$(Q) $(STM32FLASH) -w $< $(UPLOAD_DEV)

$(PRODUCT).elf: $(OBJS) $(LDSCRIPT) $(GLOBAL_DEPS) $(LCM3)
	@echo LD $@
	$(Q) $(CC) -o $@ $(OBJS) $(LDFLAGS)

%.o: %.c $(GLOBAL_DEPS)
	@echo CC $@
	$(Q) $(CC) $(CFLAGS) -o $@ -c $<

%.bin: %.elf
	@echo BIN $@
	$(Q) $(OBJCOPY) -O binary $^ $@

$(SRCS): $(LCM3)

.PHONY: $(LCM3)
$(LCM3):
	$(Q) make -C $(LCM3) TARGETS=$(LCM3_TARGETS) lib

.PHONY: clean
clean:
	$(Q) rm -f $(OBJS) $(DEPS) $(PRODUCTS)

.PHONY: reallyclean
reallyclean: clean
	$(Q) make -C $(LCM3) TARGETS=$(LCM3_TARGETS) clean

-include $(DEPS)
