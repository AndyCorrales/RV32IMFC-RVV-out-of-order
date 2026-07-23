// Demo bare-metal COMPLETO: UART, interrupcion de timer, y U-mode.
//  1. imprime "RV32" por el UART (store a 0x20000000)
//  2. programa el timer y habilita la interrupcion -> el handler la atiende
//  3. baja a U-mode con MRET y ejecuta un ECALL desde ahi (causa 8, no 11)
#define UART ((volatile unsigned int*)0x20000000)
// Resultados en 0x400, FUERA de la imagen del programa (~214 bytes): en
// la pista TLM la memoria es unificada, asi que escribir dentro del
// propio codigo lo auto-modificaria. En HLS no molestaba (imem y dmem
// estan separadas), pero asi el MISMO binario sirve en ambas pistas.
volatile unsigned int* res = (unsigned int*)0x400;

static void putc_(char c) { *UART = c; }

void __attribute__((naked, aligned(4))) handler(void) {
    __asm__ volatile(
        "csrr t0, mcause          \n"
        "li   t1, 0x80000007      \n" // interrupcion de timer?
        "bne  t0, t1, 1f          \n"
        "  li   t2, 0xFFFFFFFF    \n" // desarmar el timer
        "  csrw 0x7C1, t2         \n"
        "  li   t2, 0x11          \n" // marca: atendio la interrupcion
        "  li   t3, 0x404         \n"
        "  sw   t2, 0(t3)         \n"
        "  mret                   \n"
        "1:                       \n" // si no, fue un ECALL
        "  li   t3, 0x408         \n"
        "  sw   t0, 0(t3)         \n" // guarda la causa reportada
        "  csrr t2, mepc          \n"
        "  addi t2, t2, 4         \n"
        "  csrw mepc, t2          \n"
        "  mret                   \n"
        ::: "t0","t1","t2","t3");
}

int main(void) {
    __asm__ volatile("csrw mtvec, %0" :: "r"(handler));
    putc_('R'); putc_('V'); putc_('3'); putc_('2'); putc_('\n');
    res[0] = 1;                                    // 0x400 = 1

    // --- interrupcion de timer ---
    __asm__ volatile("csrw 0x7C1, %0" :: "r"(40));  // mtimecmp = 40 ciclos
    __asm__ volatile("csrs mie, %0"   :: "r"(1<<7)); // habilita MTIE
    __asm__ volatile("csrs mstatus, %0" :: "r"(1<<3)); // habilita MIE global
    for (volatile int i = 0; i < 60; i++) { }       // espera a que dispare
    __asm__ volatile("csrc mstatus, %0" :: "r"(1<<3)); // vuelve a deshabilitar

    // --- bajar a U-mode y hacer un ECALL desde ahi ---
    __asm__ volatile(
        "la   t0, 1f            \n"
        "csrw mepc, t0          \n"
        "li   t1, 0x1800        \n" // limpia mstatus.MPP -> MRET baja a U-mode
        "csrc mstatus, t1       \n"
        "mret                   \n"
        "1: ecall               \n" // ECALL desde U-mode -> causa 8
        ::: "t0","t1");
    return 0;
}
