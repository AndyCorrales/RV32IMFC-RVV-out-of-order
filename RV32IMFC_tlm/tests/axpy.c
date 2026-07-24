/* axpy.c — Baseline escalar FP32: y = a*x + y, n=1024, bare-metal.
 * Autovalida (resultado exacto en FP32) y reporta ciclos/instrucciones
 * del kernel via rdcycle/rdinstret. Compilar con -march=rv32imfc.
 */
#include <stdint.h>
extern int printf(const char *fmt, ...);

#define N 1024

/* volatile => el compilador emite flw/fsw reales y no pliega el kernel */
static volatile float x[N];
static volatile float y[N];

static inline uint64_t rdcycle64(void) {
    uint32_t lo, hi, hi2;
    do { __asm__ volatile("rdcycleh %0":"=r"(hi));
         __asm__ volatile("rdcycle  %0":"=r"(lo));
         __asm__ volatile("rdcycleh %0":"=r"(hi2)); } while (hi != hi2);
    return ((uint64_t)hi << 32) | lo;
}
static inline uint64_t rdinstret64(void) {
    uint32_t lo, hi, hi2;
    do { __asm__ volatile("rdinstreth %0":"=r"(hi));
         __asm__ volatile("rdinstret  %0":"=r"(lo));
         __asm__ volatile("rdinstreth %0":"=r"(hi2)); } while (hi != hi2);
    return ((uint64_t)hi << 32) | lo;
}

int main(void) {
    const float a = 2.0f;

    for (int i = 0; i < N; i++) { x[i] = (float)i; y[i] = 1.0f; }

    uint64_t c0 = rdcycle64(), k0 = rdinstret64();
    for (int i = 0; i < N; i++)
        y[i] = a * x[i] + y[i];                 /* AXPY */
    uint64_t c1 = rdcycle64(), k1 = rdinstret64();

    /* Validacion: y[i] == 2*i + 1, exacto en FP32 para i < 2^23 */
    int err = 0;
    for (int i = 0; i < N; i++) {
        float expected = (float)(2 * i + 1);
        if (y[i] != expected) err++;
    }

    uint64_t cyc = c1 - c0, ins = k1 - k0;
    printf("=== AXPY FP32 (y = a*x + y), n=%d ===\n", N);
    printf("  ciclos        : %llu\n", (unsigned long long)cyc);
    printf("  instrucciones : %llu\n", (unsigned long long)ins);
    printf("  ciclos/elem   : %llu\n", (unsigned long long)(cyc / N));
    printf("  validacion    : %d errores -> %s\n", err, err ? "FAIL" : "PASS");
    return 0;
}
