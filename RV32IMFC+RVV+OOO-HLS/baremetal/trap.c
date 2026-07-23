// Programa bare-metal con MANEJO DE EXCEPCIONES REAL.
// Instala un handler en mtvec, dispara un ECALL, el handler corre,
// incrementa mepc para saltar la instruccion que atrapo, y vuelve con
// MRET. El programa continua despues del ECALL: eso solo es posible con
// excepciones PRECISAS y reanudables.
volatile int* out = (int*)0x80;

void __attribute__((naked, aligned(4))) trap_handler(void) {
    __asm__ volatile(
        "li   t0, 0xBEEF        \n" // marca: el handler corrio
        "sw   t0, 0(%0)         \n"
        "csrr t1, mcause        \n" // guarda la causa reportada
        "sw   t1, 4(%0)         \n"
        "csrr t2, mepc          \n" // mepc apunta al ECALL...
        "addi t2, t2, 4         \n" // ...avanzar para no repetirlo
        "csrw mepc, t2          \n"
        "mret                   \n" // vuelve al programa
        :: "r"(0x90) : "t0","t1","t2");
}

int main(void) {
    __asm__ volatile("csrw mtvec, %0" :: "r"(trap_handler));
    out[0] = 1;              // 0x80 = 1  (antes del trap)
    __asm__ volatile("ecall");
    out[1] = 2;              // 0x84 = 2  (SOLO se ejecuta si el MRET volvio)
    return 0;
}
