ENTRY(_start)
SECTIONS {
  . = 0x0;
  .text : { crt0.o(.text.init) *(.text*) }
  .rodata : { *(.rodata*) }
  .data : { *(.data*) }
  .bss : { *(.bss*) *(COMMON) }
}
