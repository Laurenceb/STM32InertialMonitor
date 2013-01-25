/* Linker script for STM32 (by Laurence)
 *
 * Configure target memory and included script
 * according to your application requirements. */

/* Define memory spaces. */
MEMORY
{
  RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 20K
  FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 128K
}

/*use this for the ARM-GCC launchpad toolchain*/
INCLUDE "lanchon-stm32-names.inc"
INCLUDE "sections.ld"

