/* TI's linker script for the 'hello' sample application */
/* Modified to put .ARM.exidx section in FLASH and to define */
/* 'end' label for use by syscalls (sbrk function) */

MEMORY
{
    FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 0x00040000
    SRAM (rwx) : ORIGIN = 0x20000000, LENGTH = 0x00010000
}

SECTIONS
{
    .text :
    {
        _text = .;
        KEEP(*(.isr_vector))
        *(.text*)
        *(.rodata*)
    } > FLASH

    .ARM.exidx :
    {
        *(.ARM.exidx)
        _etext = .;
    } > FLASH

    .data : AT(ADDR(.ARM.exidx) + SIZEOF(.ARM.exidx))
    {
        _data = .;
        *(vtable)
        *(.data*)
        _edata = .;
    } > SRAM
    .bss :
    {
        _bss = .;
        *(.bss*)
        *(COMMON)
        _ebss = .;
        end = .; /* sbrk() syscall expects this */
    } > SRAM
}
