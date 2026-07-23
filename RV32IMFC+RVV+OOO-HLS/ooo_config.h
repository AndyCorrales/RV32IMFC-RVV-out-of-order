#ifndef OOO_CONFIG_H
#define OOO_CONFIG_H

// Parametros de la microarquitectura: latencias de las unidades
// funcionales y tamanos de las estructuras. Es el archivo que se toca para
// explorar el espacio de diseno (mas ALUs, ROB mas profundo, etc.).
#include "rv32_ooo.h"
#include "rv32i_defs.h"


// ---- latencias de las unidades funcionales (en ticks) ----
static const int ALU_LAT   = 1;
static const int MUL_LAT   = 3;
static const int DIV_LAT   = 8;
static const int BR_LAT    = 1;
static const int FPU_LAT_ADDMUL = 3; // FADD/FSUB/FMUL
static const int FPU_LAT_DIV    = 8; // FDIV/FSQRT
static const int FPU_LAT_FMA    = 4; // familia FMADD (R4)
static const int FPU_LAT_MISC   = 2; // sgnj/minmax/cmp/cvt/mv/class

static const int ROB_SZ  = 8;
static const int N_ALU   = 2;
static const int VEC_LAT = 3; // vadd.vv/vsub.vv/vmul.vv

// ---- RVV: opcodes/campos verificados contra la especificacion oficial
// (RISC-V "V" Vector Extension v1.0), mismos valores que rv32_vector.cpp.
// No se agregan a rv32i_defs.h a proposito (ese header es compartido con
// rv32_core.cpp/rv32_vector.cpp, no hace falta tocarlo). ----
// Periferico UART mapeado en memoria: cualquier store a esta direccion
// imprime el byte bajo por la consola del simulador. Es lo minimo que
// necesita un runtime C para tener salida (putchar/printf).
// Region MMIO alta, deliberadamente FUERA del rango de dmem (64KB); misma
// direccion que memory_map::UART_BASE en la pista TLM, para que ambas
// pistas ejecuten EXACTAMENTE los mismos binarios para
// no colisionar con el binario ni con sus datos.
static const uint32_t UART_TX_ADDR = 0x20000000;

#endif // OOO_CONFIG_H
