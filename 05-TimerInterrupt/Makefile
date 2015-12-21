CROSS_COMPILE ?= arm-none-eabi-
CC := $(CROSS_COMPILE)gcc
export SHELL := /bin/bash
CFLAGS = -g -fno-common -ffreestanding -O0 \
	 -gdwarf-2 -g3 -Wall -Werror \
	 -mcpu=cortex-m3 -mthumb \
	 -Wl,-Thello.ld -nostartfiles \

TARGET = hello.bin
all: $(TARGET)

$(TARGET): hello.c startup.c
	$(CC) $(CFLAGS) $^ -o hello.elf
	$(CROSS_COMPILE)objcopy -Obinary hello.elf hello.bin
	$(CROSS_COMPILE)objdump -S hello.elf > hello.list

qemu: $(TARGET)
	@qemu-system-arm -M ? | grep stm32-p103 >/dev/null || exit
	@echo "Press Ctrl-A and then X to exit QEMU"
	@echo
	qemu-system-arm -M stm32-p103 -nographic -kernel hello.bin

qemudbg: $(TARGET)
	qemu-system-arm -M ? | grep stm32-p103 >/dev/null || exit
	@echo "Press Ctrl-A and then X to exit QEMU"
	@echo
	qemu-system-arm -M stm32-p103 -monitor stdio -gdb tcp::3333 -S -kernel hello.bin 2>&1>/dev/null & \
	echo $$! > qemu_pid && \
	$(CROSS_COMPILE)gdb -x gdbscript && \
	cat qemu_pid | `xargs kill 2>/dev/null || test true` && \
	rm -f qemu_pid

clean:
	rm -f *.o *.bin *.elf *.list
