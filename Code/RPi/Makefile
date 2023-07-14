
ARMGNU ?= aarch64-none-elf

#COPS = -Wall -O2 -nostdlib -nostartfiles -ffreestanding 
COPS = -Wall -O2 -nostartfiles 

gcc : baremetal_dls.bin

all : gcc

clean :
	rm -f *.o
	rm -f *.bin
	rm -f *.hex
	rm -f *.elf
	rm -f *.list


vectors.o : vector.s
	$(ARMGNU)-as vector.s -o vectors.o

main.o : main.c
	$(ARMGNU)-gcc $(COPS) -c main.c -o main.o

RPi_DLS7.o : RPi_DLS.c 
	$(ARMGNU)-gcc $(COPS) -c RPi_DLS.c -o RPi_DLS7.o

baremetal_dls.bin : memmap vectors.o RPi_DLS7.o main.o 
	$(ARMGNU)-ld vectors.o RPi_DLS7.o main.o -T memmap -o main.elf
	$(ARMGNU)-objdump -D main.elf > main.list
	$(ARMGNU)-objcopy main.elf -O ihex main.hex
	$(ARMGNU)-objcopy main.elf -O binary main.bin

