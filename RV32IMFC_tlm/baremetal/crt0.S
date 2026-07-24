/* crt0.S — Arranque bare-metal para RV32IMC
 *
 * Flujo:
 *   1. Carga sp con la direccion de _stack_top (definida en link.ld)
 *   2. Inicializa gp (global pointer) para acceso eficiente a variables globales
 *   3. Zeroes BSS (.bss y .sbss entre _bss_start y _bss_end)
 *   4. Llama a main(0, NULL, NULL)
 *   5. Al retornar, llama ECALL SYS_EXIT con el codigo de retorno de main
 *
 * El simulador intercepta ECALL con a7=93 (SYS_EXIT) en processor.h
 * y detiene la simulacion imprimiendo las metricas de IPC.
 */

    .section .text.startup, "ax", @progbits
    .global  _start
    .type    _start, @function

_start:
    /* 1. Stack pointer */
    la   sp, _stack_top

    /* 2. Global pointer (para acceso a .sdata/.sbss en +/-2KB) */
    .option push
    .option norelax
    la   gp, __global_pointer$
    .option pop

    /* 3. Limpiar BSS — loop de 4 bytes */
    la   a0, _bss_start
    la   a1, _bss_end
.bss_loop:
    bge  a0, a1, .bss_done
    sw   zero, 0(a0)
    addi a0, a0, 4
    j    .bss_loop
.bss_done:

    /* 4. Llamar main(argc=0, argv=NULL, envp=NULL) */
    li   a0, 0
    li   a1, 0
    li   a2, 0
    call main

    /* 5. ECALL SYS_EXIT con el codigo que devolvio main (en a0) */
    mv   a0, a0          /* exit code = retorno de main */
    li   a7, 93          /* SYS_EXIT */
    ecall

    /* No deberia llegar aqui — loop infinito de seguridad */
.hang:
    j    .hang

    .size _start, . - _start
