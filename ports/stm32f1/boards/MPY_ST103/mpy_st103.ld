/*
    GNU linker script for STM32F103xE with Ext NorFlash & SRAM
*/

/* Specify the memory areas */
MEMORY
{
    FLASH (rx)      : ORIGIN = 0x64000000, LENGTH = 16M  /* entire flash, 8M-16bit */
    FLASH_ISR (rx)  : ORIGIN = 0x64000000, LENGTH = 16K  /* 16K ISR */
    FLASH_FS (rx)   : ORIGIN = 0x64004000, LENGTH = 112K /* 112k fs */
    FLASH_TEXT (rx) : ORIGIN = 0x64000000, LENGTH = 16M  /* ext NorFlash S29GL128P */
    RAM (xrw)       : ORIGIN = 0x20000000, LENGTH = 96K
    RAM_APP(xrw)    : ORIGIN = 0x68000000, LENGTH = 1M  /* ext SRAM IS62WV51216 */
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
