#ifndef RV32_VECTOR_H
#define RV32_VECTOR_H

#include <ap_int.h>

// Primer decoder REAL de instrucciones RVV (Vector Extension v1.0),
// sintetizable con Vitis HLS. Modulo nuevo y separado de rv32_core.cpp /
// rv32_ooo.cpp -- no los toca. Codificacion de bits verificada contra la
// especificacion oficial (RISC-V "V" Vector Extension, Version 1.0,
// seccion 19 "Vector Instruction Listing" y seccion 7 "Vector Loads and
// Stores"), no inventada de memoria.
//
// Alcance deliberadamente minimo, dividido en las dos secciones
// naturales del ISA vectorial:
//
//   Seccion 1 -- Memoria vectorial (unit-stride, sin mascara):
//     vle32.v vd,  (rs1)   -- load:  vd[0..3]  <- mem[rs1 .. rs1+15]
//     vse32.v vs3, (rs1)   -- store: mem[rs1..rs1+15] <- vs3[0..3]
//
//   Seccion 2 -- Aritmetica vectorial vector-vector (OPIVV/OPMVV):
//     vadd.vv vd, vs2, vs1  -- vd[i] = vs2[i] + vs1[i]
//     vsub.vv vd, vs2, vs1  -- vd[i] = vs2[i] - vs1[i]
//     vmul.vv vd, vs2, vs1  -- vd[i] = vs2[i] * vs1[i]
//
// Simplificaciones documentadas (ver LIMITACIONES.md):
//   - SEW=32 fijo, LMUL=1 fijo, VLEN=128 -> exactamente 4 elementos
//     (lanes) por registro vectorial. Sin vtype/vsetvli dinamico.
//   - vm=1 siempre (sin mascara, v0 no se usa como registro de mascara).
//   - nf=1 (sin loads/stores segmentados), mop=00 (solo unit-stride, sin
//     strided/indexed).
//   - Sin reduccion, sin permutacion, sin punto flotante vectorial.
//
// Interfaz: una funcion de UN PASO (mismo estilo que rv32_core_step),
// decode+execute de una instruccion vectorial por llamada. El operando
// escalar rs1 (direccion base) entra ya resuelto como parametro --en una
// integracion real, el decoder del CPU escalar leeria x[rs1] y se lo
// pasaria a esta unidad, igual que se documento como pendiente en
// vector_unit.h de la pista TLM.
//
// Banco de registros vectoriales: 32 registros x 4 lanes de 32 bits,
// aplanado a un arreglo de 128 palabras (indice = vreg*4 + lane) para
// que la interfaz HLS sea simple (un solo puerto BRAM en vez de un
// arreglo 2D).
#define VEC_NUM_VREGS   32
#define VEC_LANES       4   // VLEN(128)/SEW(32)
#define VEC_REGFILE_LEN (VEC_NUM_VREGS * VEC_LANES)
#define VEC_MEM_WORDS   64

void rv32_vector_step(
    ap_uint<32> instr,
    ap_uint<32> rs1_val,                          // direccion base (ya resuelta)
    ap_uint<32> vregs_in[VEC_REGFILE_LEN],
    ap_uint<32> vregs_out[VEC_REGFILE_LEN],
    ap_uint<32>* mem,
    ap_uint<1>&  halted
);

#endif // RV32_VECTOR_H
