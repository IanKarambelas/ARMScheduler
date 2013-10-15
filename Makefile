# Modify as appropriate
STELLARISWARE=~/StellarisWare

CC=~/CodeSourcery/Sourcery_G++_Lite/bin/arm-none-eabi-gcc -Wall -Os -march=armv7-m -mcpu=cortex-m3 -mthumb -mfix-cortex-m3-ldrd -Wl,--gc-sections
	
crsched.elf: crsched.c create.S threads.c
	${CC} -o $@ -I ${STELLARISWARE} -L ${STELLARISWARE}/driverlib/gcc-cm3 -Tlinkscript.x -Wl,-Map,crsched.map -Wl,--entry,ResetISR crsched.c create.S threads.c startup_gcc.c syscalls.c rit128x96x4.c -ldriver-cm3

.PHONY: clean
clean:
	rm -f *.elf *.map

# vim: noexpandtab  
