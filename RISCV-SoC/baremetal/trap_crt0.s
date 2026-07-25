.section .text.init
.globl _start
_start:
    li   sp, 0xF0
    call main
    li   t0, 0
    csrw mtvec, t0        # sin handler -> el proximo ecall detiene el core
    ecall
