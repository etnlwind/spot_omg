ENTRY(Reset_Handler)

_estack = ORIGIN(RAM) + LENGTH(RAM);
_Min_Heap_Size = 0;
_Min_Stack_Size = 0x400;

MEMORY
{
  RAM (xrw)  : ORIGIN = 0x20000000, LENGTH = 0x1FFF0
  FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 48K
}

SECTIONS
{
  .isr_vector : { KEEP(*(.isr_vector)) } >FLASH
  .text : { *(.text*) *(.rodata*) KEEP(*(.init)) KEEP(*(.fini)) } >FLASH
  .ARM.extab : { *(.ARM.extab*) } >FLASH
  .ARM : { __exidx_start = .; *(.ARM.exidx*) __exidx_end = .; } >FLASH
  _sidata = LOADADDR(.data);
  .data : { _sdata = .; *(.data*) _edata = .; } >RAM AT>FLASH
  .bss : { _sbss = .; __bss_start__ = .; *(.bss*) *(COMMON) _ebss = .; __bss_end__ = .; } >RAM
  ._user_heap_stack : { . = ALIGN(8); . += _Min_Heap_Size; . += _Min_Stack_Size; } >RAM
  /DISCARD/ : { libc.a(*) libm.a(*) libgcc.a(*) }
}
