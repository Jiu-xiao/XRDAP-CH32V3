ENTRY(_start)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 8K
  RAM (xrw)  : ORIGIN = 0x20000000, LENGTH = 32K
}

SECTIONS
{
  .init :
  {
    . = ALIGN(4);
    KEEP(*(.init))
    KEEP(*(.text*))
    KEEP(*(.rodata*))
    . = ALIGN(4);
  } > FLASH AT > FLASH

  .data :
  {
    . = ALIGN(4);
    *(.data*)
    *(.sdata*)
    . = ALIGN(4);
  } > RAM AT > FLASH

  .bss :
  {
    . = ALIGN(4);
    *(.bss*)
    *(.sbss*)
    *(COMMON)
    . = ALIGN(4);
  } > RAM
}
