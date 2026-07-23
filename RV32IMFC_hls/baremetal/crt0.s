.section .text.init
.globl _start
_start:
    li   sp, 0xF0         # stack en rango (dmem chica), aunque -O2 no lo usa
    call main
    ecall                 # fin de programa
