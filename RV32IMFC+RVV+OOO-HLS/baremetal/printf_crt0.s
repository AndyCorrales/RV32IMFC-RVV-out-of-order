.section .text.init
.globl _start
_start:
    la   sp, _stack_top
    call main
    li   t0, 0
    csrw mtvec, t0        # sin handler -> el ecall detiene el core
    ecall
