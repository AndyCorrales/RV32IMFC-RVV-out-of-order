#ifndef CORE_PORTME_H
#define CORE_PORTME_H

/* stdint.h, stddef.h y stdarg.h son headers FREESTANDING del compilador:
 * el estandar C obliga a que existan aun sin libreria C. stddef.h da NULL
 * y size_t -> arregla todos los errores de 'NULL undeclared'.
 * printf esta definido en syscall_stubs.c e importado con extern. */
#include <stdint.h>
#include <stddef.h>

/* ---- Configuracion de plataforma (coremark.h NO tiene defaults) ---- */
#ifndef HAS_FLOAT
#define HAS_FLOAT 0     /* 0 => reporte final con %d/%u (tu printf ya los soporta).
                           Ponelo en 1 solo si agregas %f a tu printf. */
#endif
#ifndef HAS_TIME_H
#define HAS_TIME_H 0
#endif
#ifndef HAS_STDIO
#define HAS_STDIO 0
#endif
#ifndef HAS_PRINTF
#define HAS_PRINTF 0
#endif

/* Cadenas que CoreMark imprime en el reporte (obligatorias). */
#ifndef COMPILER_VERSION
#ifdef __VERSION__
#define COMPILER_VERSION "GCC " __VERSION__
#else
#define COMPILER_VERSION "riscv64-unknown-elf-gcc"
#endif
#endif
#ifndef COMPILER_FLAGS
#define COMPILER_FLAGS "-march=rv32imc -mabi=ilp32 -O2 -static -nostartfiles"
#endif
#ifndef MEM_LOCATION
#define MEM_LOCATION "STATIC"
#endif

/* ---- Tipos requeridos por CoreMark ---- */
typedef signed short    ee_s16;
typedef unsigned short  ee_u16;
typedef signed int      ee_s32;
typedef float           ee_f32;   /* antes decia ee_f64 (nombre incorrecto) */
typedef unsigned char   ee_u8;
typedef unsigned int    ee_u32;
typedef uint64_t        CORE_TICKS;
typedef unsigned int    ee_ptr_int;   /* ILP32: puntero = 32 bits */
typedef unsigned int    ee_size_t;

/* NO redefinir secs_ret aqui: coremark.h lo hace segun HAS_FLOAT. */

/* Alinea un offset a 32 bits (lo usa el algoritmo de matrices). */
#define align_mem(x) (void *)(4 + (((ee_ptr_int)(x)-1) & ~3))

/* printf casero definido en syscall_stubs.c */
extern int printf(const char* fmt, ...);
#define ee_printf printf

/* ---- Configuracion CoreMark ---- */
#define MULTITHREAD             1
#define USE_PTHREAD             0
#define USE_FORK                0
#define USE_SOCKET              0
#define MAIN_HAS_NOARGC         1   /* bare-metal: main(void), no depende de argv */
#define MAIN_HAS_NORETURN       0

/* Bare-metal SIN argv: hay que usar SEED_VOLATILE (lee seedN_volatile).
 * Con SEED_ARG las iteraciones saldrian de argv[4] => 0 iteraciones. */
#ifndef SEED_METHOD
#define SEED_METHOD SEED_VOLATILE
#endif

/* Contexto unico; debe existir y valer 1 (definido en el .c). */
extern ee_u32 default_num_contexts;

typedef struct { int portable_id; } core_portable;

#ifdef __cplusplus
extern "C" {
#endif
extern CORE_TICKS start_time_val;
extern CORE_TICKS stop_time_val;
void       start_time(void);
void       stop_time(void);
CORE_TICKS get_time(void);
void       portable_init(core_portable *p, int *argc, char *argv[]);
void       portable_fini(core_portable *p);
/* time_in_secs lo declara coremark.h (usa secs_ret) -> NO declararlo aqui. */
#ifdef __cplusplus
}
#endif

#endif /* CORE_PORTME_H */
