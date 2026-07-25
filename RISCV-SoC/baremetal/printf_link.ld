ENTRY(_start)
SECTIONS {
  . = 0x0;
  .text : { printf_crt0.o(.text.init) *(.text*) }
  .rodata : { *(.rodata*) *(.srodata*) }
  .data : { *(.data*) *(.sdata*) }
  .bss : { *(.bss*) *(.sbss*) *(COMMON) }
  _end = .;
  . = ALIGN(16);
  . = . + 0x400;          /* 1KB de stack */
  _stack_top = .;
}
