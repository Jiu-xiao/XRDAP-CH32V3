/* Stage 1 (stage1.S): code only, at address 0, within its 4 KB sector. */
ENTRY(_start)
MEMORY
{
  FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 4K
}
SECTIONS
{
  .text : { KEEP(*(.text.stage1)) } > FLASH
  /DISCARD/ : { *(.data) *(.data.*) *(.bss) *(.bss.*) *(.riscv.attributes) *(.comment) }
}
