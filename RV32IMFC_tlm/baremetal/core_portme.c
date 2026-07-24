/* core_portme.c — VERSION CON MARCADORES DE DIAGNOSTICO.
 * Los printf con >>> nos dicen hasta donde llega CoreMark antes de morir. */
#include "coremark.h"
#include "core_portme.h"
#include <stdint.h>

extern int printf(const char* fmt, ...);

#if VALIDATION_RUN
volatile ee_s32 seed1_volatile = 0x3415;
volatile ee_s32 seed2_volatile = 0x3415;
volatile ee_s32 seed3_volatile = 0x66;
#endif
#if PERFORMANCE_RUN
volatile ee_s32 seed1_volatile = 0x0;
volatile ee_s32 seed2_volatile = 0x0;
volatile ee_s32 seed3_volatile = 0x66;
#endif
#if PROFILE_RUN
volatile ee_s32 seed1_volatile = 0x8;
volatile ee_s32 seed2_volatile = 0x8;
volatile ee_s32 seed3_volatile = 0x8;
#endif
volatile ee_s32 seed4_volatile = ITERATIONS;
volatile ee_s32 seed5_volatile = 0;

#ifndef CPU_HZ
#define CPU_HZ 25000000ULL 
#endif

static inline uint64_t rdcycle64(void) {
    uint32_t lo, hi, hi2;
    do {
        __asm__ volatile ("rdcycleh %0" : "=r"(hi));
        __asm__ volatile ("rdcycle  %0" : "=r"(lo));
        __asm__ volatile ("rdcycleh %0" : "=r"(hi2));
    } while (hi != hi2);
    return ((uint64_t)hi << 32) | lo;
}

static inline uint64_t rdinstret64(void) {
    uint32_t lo, hi, hi2;
    do {
        __asm__ volatile ("rdinstreth %0" : "=r"(hi));
        __asm__ volatile ("rdinstret  %0" : "=r"(lo));
        __asm__ volatile ("rdinstreth %0" : "=r"(hi2));
    } while (hi != hi2);
    return ((uint64_t)hi << 32) | lo;
}

CORE_TICKS start_time_val;
CORE_TICKS stop_time_val;
static uint64_t start_instret;
static uint64_t stop_instret;

void start_time(void) {
    printf(">>> MARCADOR 1: start_time (el init de estructuras TERMINO OK)\n");
    start_instret  = rdinstret64();
    start_time_val = rdcycle64();
}

void stop_time(void) {
    printf(">>> MARCADOR 2: stop_time (iterate() TERMINO OK)\n");
    stop_time_val = rdcycle64();
    stop_instret  = rdinstret64();
    uint64_t ciclos = stop_time_val - start_time_val;
    uint64_t instrs = stop_instret  - start_instret;
    printf("\n[CoreMark Metrics]\n");
    printf("  Ciclos        : %llu\n", (unsigned long long)ciclos);
    printf("  Instrucciones : %llu\n", (unsigned long long)instrs);
    if (ciclos > 0) {
        uint64_t ipc_e = instrs / ciclos;
        uint64_t ipc_f = (instrs * 1000 / ciclos) % 1000;
        printf("  IPC           : %llu.%03llu\n",
               (unsigned long long)ipc_e, (unsigned long long)ipc_f);
    }
}

CORE_TICKS get_time(void) { return stop_time_val - start_time_val; }

secs_ret time_in_secs(CORE_TICKS ticks) {
    return (secs_ret)((double)ticks / (double)CPU_HZ);
}

ee_u32 default_num_contexts = 1;

void portable_init(core_portable *p, int *argc, char *argv[]) {
    (void)argc; (void)argv;
    p->portable_id = 1;
    printf("[portme] CoreMark iniciando en RV32IMFC TLM Simulator\n\n");
}

void portable_fini(core_portable *p) { p->portable_id = 0; }
