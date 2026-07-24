/* link.ld — Script de enlace bare-metal para RV32IMC
 *
 * Layout de memoria (espejo exacto de memory_map.h):
 *   RAM_BASE = 0x00000000
 *   RAM_SIZE = 512 KB
 *   STACK    = 64 KB al final de RAM
 *
 * El segmento PT_LOAD del ELF resultante cargara .text, .rodata y .data
 * desde el offset 0 del archivo ELF hacia la direccion 0x00000000 de RAM.
 * El ELF loader en main.cpp copia cada PT_LOAD a memory.data[p_vaddr].
 */

OUTPUT_ARCH(riscv)
ENTRY(_start)

MEMORY {
    RAM (rwx) : ORIGIN = 0x00000000, LENGTH = 512K
}

/* Tamano del stack: 64 KB al final de la RAM */
_stack_size = 64K;

SECTIONS {
    /* ── Codigo y constantes (solo lectura) ──────────────────────────── */
    . = ORIGIN(RAM);

    .text : {
        KEEP(*(.text.startup))   /* crt0.S va primero */
        *(.text*)
        *(.rodata*)
        *(.rodata)
    } > RAM

    . = ALIGN(4);

    /* ── Datos inicializados ─────────────────────────────────────────── */
    .data : {
        __global_pointer$ = . + 0x800;  /* gp apunta aqui para +/-2 KB */
        *(.sdata*)
        *(.sdata)
        *(.data*)
        *(.data)
    } > RAM

    . = ALIGN(4);

    /* ── BSS: variables no inicializadas (crt0.S las zeroes) ────────── */
    _bss_start = .;
    .bss : {
        *(.sbss*)
        *(.sbss)
        *(.bss*)
        *(.bss)
        *(COMMON)
    } > RAM
    _bss_end = .;

    . = ALIGN(16);

    /* ── Heap estatico para _sbrk (newlib, si se usa malloc) ─────────── */
    _heap_start = .;
    . = . + 32K;
    _heap_end   = .;

    /* ── Stack al final de RAM ───────────────────────────────────────── */
    . = ALIGN(16);
    . = ORIGIN(RAM) + LENGTH(RAM) - _stack_size;
    _stack_bottom = .;
    . = . + _stack_size;
    _stack_top    = .;          /* crt0.S carga esto en sp */
}
