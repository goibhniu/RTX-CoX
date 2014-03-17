# put your *.o targets here, make should handle the rest!

SRCS = main.c stm32f4xx_it.c system_stm32f4xx.c HAL_CM.c rt_CMSIS.c rt_Event.c rt_System.c rt_Task.c rt_Semaphore.c
SRCS += rt_List.c rt_Mailbox.c rt_MemBox.c rt_Memory.c rt_Mutex.c rt_Robin.c rt_Time.c rt_Timer.c RTX_Conf_CM.c
SRCS += HAL_CM4.s SVC_Table.s 


# all the files will be generated with this name (main.elf, main.bin, main.hex, etc)

PROJ_NAME=main

# that's it, no need to change anything below this line!

###################################################

CC=/opt/arm/bin/arm-none-eabi-gcc
OBJCOPY=/opt/arm/bin/arm-none-eabi-objcopy
AS=/opt/arm/bin/arm-none-eabi-as

CFLAGS  = -g -O2 -Wall -Tstm32_flash.ld -D__CMSIS_RTOS -D__CORTEX_M4F -D__FPU_PRESENT
CFLAGS += -mlittle-endian -mthumb -mcpu=cortex-m4 -mthumb-interwork
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16

###################################################

vpath %.c src
vpath %.a hal
vpath %.s src

ROOT=$(shell pwd)

CFLAGS += -Iinc  -Ihal/inc 

SRCS += startup_stm32f4xx.s # add startup file to build

OBJS = $(SRCS:.c=.o)

###################################################

.PHONY: lib proj

all: lib proj

lib:
	$(MAKE) -C hal

proj: 	$(PROJ_NAME).elf

$(PROJ_NAME).elf: $(SRCS)
	$(CC) $(CFLAGS) $^ -o $@ -Lhal -lhal
	$(OBJCOPY) -O ihex $(PROJ_NAME).elf $(PROJ_NAME).hex
	$(OBJCOPY) -O binary $(PROJ_NAME).elf $(PROJ_NAME).bin

clean:
	$(MAKE) -C lib clean
	$(MAKE) -C hal clean
	rm -f $(PROJ_NAME).elf
	rm -f $(PROJ_NAME).hex
	rm -f $(PROJ_NAME).bin
