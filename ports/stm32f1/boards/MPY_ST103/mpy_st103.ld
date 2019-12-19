/*
    GNU linker script for STM32F103xE with Ext NorFlash & SRAM
*/

/* Specify the memory areas */
MEMORY
{
    FLASH (rx)      : ORIGIN = 0x08000000, LENGTH = 1M  /* entire flash, 1M*/
    FLASH_ISR (rx)  : ORIGIN = 0x08000000, LENGTH = 16K  /* 16K ISR */
    FLASH_FS (rx)   : ORIGIN = 0x08004000, LENGTH = 112K /* 112k fs */
    FLASH_TEXT (rx) : ORIGIN = 0x08020000, LENGTH = 800K
    RAM (xrw)       : ORIGIN = 0x20000000, LENGTH = 96K
}

/* produce a link error if there is not this amount of RAM for these sections */
_minimum_stack_size = 2K;
_minimum_heap_size = 16K;

/* Define the stack.  The stack is full descending so begins just above last byte
   of RAM.  Note that EABI requires the stack to be 8-byte aligned for a call. */
_estack = ORIGIN(RAM) + LENGTH(RAM) - _estack_reserve;
_sstack = _estack - 8K; /* tunable */

/* RAM extents for the garbage collector */
_ram_start = ORIGIN(RAM);
_ram_end = ORIGIN(RAM) + LENGTH(RAM);
_heap_start = _ebss; /* heap starts just after statically allocated memory */
_heap_end = _sstack;
