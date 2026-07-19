/* gemm.c — Baseline escalar FP32: C = A*B, 32x32, bare-metal.
 * Entradas enteras pequenas => C exacto en FP32; se valida contra una
 * referencia entera independiente. Reporta ciclos/instrucciones del kernel.
 * Compilar con -march=rv32imfc.
 */
#include <stdint.h>
extern int printf(const char *fmt, ...);

#define M 32

static volatile float A[M][M];
static volatile float B[M][M];
static volatile float C[M][M];

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
    /* Entradas: valores 0..6 y 0..4 -> suma maxima 32*6*4=768 < 2^24 (exacto) */
    for (int i = 0; i < M; i++)
        for (int j = 0; j < M; j++) {
            A[i][j] = (float)((i + j) % 7);
            B[i][j] = (float)((i * 3 + j) % 5);
        }

    uint64_t c0 = rdcycle64(), k0 = rdinstret64();
    for (int i = 0; i < M; i++)
        for (int j = 0; j < M; j++) {
            float s = 0.0f;
            for (int k = 0; k < M; k++)
                s += A[i][k] * B[k][j];         /* producto punto */
            C[i][j] = s;
        }
    uint64_t c1 = rdcycle64(), k1 = rdinstret64();

    /* Validacion contra referencia entera independiente */
    int err = 0;
    for (int i = 0; i < M; i++)
        for (int j = 0; j < M; j++) {
            int s = 0;
            for (int k = 0; k < M; k++)
                s += ((i + k) % 7) * ((k * 3 + j) % 5);
            if ((int)C[i][j] != s) err++;
        }

    uint64_t cyc = c1 - c0, ins = k1 - k0;
    printf("=== GEMM FP32 (C = A*B), %dx%d ===\n", M, M);
    printf("  ciclos        : %llu\n", (unsigned long long)cyc);
    printf("  instrucciones : %llu\n", (unsigned long long)ins);
    printf("  validacion    : %d errores -> %s\n", err, err ? "FAIL" : "PASS");
    return 0;
}
