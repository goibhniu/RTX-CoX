CC=/opt/arm/bin/arm-none-eabi-gcc
AR=/opt/arm/bin/arm-none-eabi-ar

###########################################

vpath %.c src 

CFLAGS  = -g -O2 -Wall
CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m4 -mthumb-interwork
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
CFLAGS += -ffreestanding -nostdlib
CFLAGS += -Iinc 

#SRCS  = stm32f4_discovery.c
SRCS = xadc.c xcore.c xdebug.c xdma.c xgpio.c xi2c.o xpwm.c \
		xrtc.c xspi.c xsysctl.c xtimer.c xuart.c xwdt.c

OBJS = $(SRCS:.c=.o)

.PHONY: libhal.a

all: libhal.a

%.o : %.c
	$(CC) $(CFLAGS) -c -o $@ $^

libhal.a: $(OBJS)
	$(AR) -r $@ $(OBJS)

clean:
	rm -f $(OBJS) libhal.a
