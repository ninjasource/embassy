MEMORY
{
    FLASH    : ORIGIN = 0x08000000, LENGTH = 1024K
    RAM      : ORIGIN = 0x24000000, LENGTH = 320K
    SDRAM     : ORIGIN = 0x70000000, LENGTH = 16M 
}

_stack_start = ORIGIN(RAM) + LENGTH(RAM);

SECTIONS
{
  .sdram (NOLOAD) : {
    . = ALIGN(4);
    *(.sdram);
    . = ALIGN(4);
  } > SDRAM
}
