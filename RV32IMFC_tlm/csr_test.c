/* csr_test.c — Aísla qué CSR de contador soporta el simulador.
 * Imprime ANTES de cada lectura: donde se detenga, ese CSR es el que falta.
 * Compilar con -O0 para que no se reordenen las lecturas.
 */
extern int printf(const char *fmt, ...);

int main(void) {
    unsigned v = 0;

    printf("A: antes de rdcycle\n");
    __asm__ volatile ("rdcycle %0" : "=r"(v));
    printf("B: rdcycle OK    = %u\n", v);

    printf("C: antes de rdcycleh\n");
    __asm__ volatile ("rdcycleh %0" : "=r"(v));
    printf("D: rdcycleh OK   = %u\n", v);

    printf("E: antes de rdinstret\n");
    __asm__ volatile ("rdinstret %0" : "=r"(v));
    printf("F: rdinstret OK  = %u\n", v);

    printf("G: antes de rdinstreth\n");
    __asm__ volatile ("rdinstreth %0" : "=r"(v));
    printf("H: rdinstreth OK = %u\n", v);

    printf("I: TODOS los CSR de contador funcionan\n");
    return 0;
}
