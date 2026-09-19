/* Linker script for the FPU acceptance firmware (STM32F411, 512K flash / 128K RAM). */

ENTRY(main)

MEMORY
{
    FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 512K
    RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 128K
}

_estack = ORIGIN(RAM) + LENGTH(RAM);

SECTIONS
{
    .text : {
        KEEP(*(.vectors))
        *(.text*)
        *(.rodata*)
    } > FLASH

    /* No initialised data is used, so .data stays empty and no C startup is needed. */
    .data : { *(.data*) } > RAM
    .bss  : { *(.bss*) *(COMMON) } > RAM

    /DISCARD/ : { *(.ARM.exidx*) *(.comment) }
}
