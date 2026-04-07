
ARMGNU ?= arm-none-eabi

AOPS = --warn --fatal-warnings
COPS = -Wall -Werror -O2 -nostdlib -nostartfiles -ffreestanding

all: kernel.img kernel.srec

clean:
	rm -f *.o
	rm -f *.bin
	rm -f *.hex
	rm -f *.srec
	rm -f *.elf
	rm -f *.list
	rm -f *.img

vectors.o: vectors.s
	$(ARMGNU)-as $(AOPS) vectors.s -o vectors.o

main.o: main.c
	$(ARMGNU)-gcc $(COPS) -c main.c -o main.o

kernel.elf: memmap vectors.o main.o
	$(ARMGNU)-ld vectors.o main.o -T memmap -o kernel.elf
	$(ARMGNU)-objdump -D kernel.elf > kernel.list

kernel.img: kernel.elf
	$(ARMGNU)-objcopy kernel.elf -O binary kernel.img

kernel.srec: kernel.elf
	$(ARMGNU)-objcopy --srec-forceS3 kernel.elf -O srec kernel.srec
