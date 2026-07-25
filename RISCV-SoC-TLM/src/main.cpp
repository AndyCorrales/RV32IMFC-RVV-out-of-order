#include <systemc.h>
#include <cstring>
#include <cstdio>
#include <vector>

#include "memory_map.h"
#include "memory.h"
#include "bus.h"
#include "uart.h"
#include "processor_ooo.h"
// definiciones fuera de linea de ProcessorOOO (una por responsabilidad)
#include "processor_vector_unit.h"
#include "processor_tick.h"
#include "processor_dispatch.h"
#include "vector_unit.h"
#include "rv32c_defs.h" // encoders r_type/i_type/s_type/b_type/u_type/j_type

// Binarios ELF reales compilados con gcc. Son EXACTAMENTE los mismos que
// ejecuta la pista HLS (mismo UART en 0x20000000), asi que correrlos en
// ambas y obtener el mismo resultado es la verificacion cruzada TLM<->HLS.
#include "trap_elf.h"    // handler de excepciones + MRET (reanudable)
#include "full_elf.h"    // UART + interrupcion de timer + modos M/U
#include "printf_elf.h"  // printf de la biblioteca C (newlib)

// =====================================================================
// TESTBENCH UNICO del core RV32IMFC + RVV + OOO (pista TLM).
//
// Corre cuatro suites en secuencia sobre la MISMA topologia TLM
// (ProcessorOOO -> Bus -> {Memory, Uart}), reseteando el core y la
// memoria entre cada una:
//
//   A. ISA + RVV  -- programa ensamblado a mano (I+M+F+C y RVV Fases 1-3)
//   B. Excepciones-- ELF real: handler en mtvec, ECALL, MRET, continua
//   C. Sistema    -- ELF real: UART, interrupcion de timer y U-mode
//   D. printf     -- ELF real enlazado contra newlib
//
// A diferencia de la pista HLS, aca el UART es un PERIFERICO TLM de
// verdad colgado del Bus: el procesador solo emite un store y es el Bus
// quien decodifica la direccion y lo rutea, sin saber que del otro lado
// hay algo distinto de una memoria.
// =====================================================================

using namespace rv32c;
namespace F3A = rv32i::Funct3_ALU;
namespace F3M = rv32i::Funct3_MULDIV;
namespace F7F = rv32i::Funct7_FP;

// Codificacion RVV -- mismos campos de bits que la pista HLS
// (rv32_vector.cpp / rv32_ooo.cpp), verificados contra RVV v1.0.
static uint32_t enc_vec_mem(uint32_t opcode, uint32_t vd_or_vs3, uint32_t rs1) {
    // nf=000, mew=0, mop=00, vm=1, lumop/sumop=00000, width=110 (32b)
    return (1u << 25) | (rs1 << 15) | (0b110u << 12) | (vd_or_vs3 << 7) | opcode;
}
// vsetvli rd, rs1, vtypei -- instr[31]=0, zimm[10:0]=instr[30:20],
// rs1=AVL, funct3=111 (OPCFG). Formato de la seccion 5 de la spec.
static uint32_t enc_vsetvli(uint32_t rd, uint32_t rs1, uint32_t vtypei) {
    return ((vtypei & 0x7FF) << 20) | (rs1 << 15) | (0b111u << 12) | (rd << 7) | 0b1010111u;
}
// vtype para SEW=32 (vsew=010 en bits[5:3]) y LMUL=1 (vlmul=000 en bits[2:0])
static const uint32_t VTYPE_E32_M1 = (0b010u << 3) | 0b000u;
static const uint32_t VTYPE_E8_M1  = (0b000u << 3) | 0b000u; // SEW=8  -> VLMAX=16
static const uint32_t VTYPE_E16_M1 = (0b001u << 3) | 0b000u; // SEW=16 -> VLMAX=8
static const uint32_t WIDTH_8  = 0b000; // vle8.v  / vse8.v
static const uint32_t WIDTH_16 = 0b101; // vle16.v / vse16.v
// load vectorial con EEW explicito en el campo width
static uint32_t enc_vec_mem_w(uint32_t opcode, uint32_t vd_or_vs3, uint32_t rs1, uint32_t width) {
    return (1u << 25) | (rs1 << 15) | (width << 12) | (vd_or_vs3 << 7) | opcode;
}

static uint32_t enc_op_v(uint32_t funct6, uint32_t vs2, uint32_t vs1, uint32_t funct3, uint32_t vd) {
    return (funct6 << 26) | (1u << 25) | (vs2 << 20) | (vs1 << 15) | (funct3 << 12) | (vd << 7) | 0b1010111u;
}
// version con control de mascara: vm=0 -> la operacion queda predicada
// por v0 (solo se escriben los elementos con v0.mask[i]=1).
static uint32_t enc_op_v_m(uint32_t funct6, uint32_t vs2, uint32_t vs1,
                           uint32_t funct3, uint32_t vd, uint32_t vm) {
    return (funct6 << 26) | (vm << 25) | (vs2 << 20) | (vs1 << 15) | (funct3 << 12) | (vd << 7) | 0b1010111u;
}
// funct3 de las familias/formas y funct6 de las operaciones (seccion 19)
static const uint32_t F3_OPIVV = 0b000, F3_OPMVV = 0b010;
static const uint32_t F3_OPIVI = 0b011, F3_OPIVX = 0b100;
static const uint32_t F6_VADD = 0b000000, F6_VSUB = 0b000010, F6_VRSUB = 0b000011;
static const uint32_t F6_VAND = 0b001001, F6_VOR = 0b001010, F6_VXOR = 0b001011;
static const uint32_t F6_VMIN = 0b000101, F6_VMAX = 0b000111;
static const uint32_t F6_VSLL = 0b100101, F6_VSRL = 0b101000, F6_VSRA = 0b101001;
static const uint32_t F6_VMUL = 0b100101, F6_VDIV = 0b100001, F6_VREM = 0b100011;
// Fase 3: comparaciones, merge, logica de mascaras y grupos unary
static const uint32_t F6_VMSEQ = 0b011000, F6_VMSLT = 0b011011, F6_VMSGT = 0b011111;
static const uint32_t F6_VMERGE = 0b010111;
static const uint32_t F6_VMAND = 0b011001, F6_VMOR = 0b011010, F6_VMXOR = 0b011011;
static const uint32_t F6_VWXUNARY0 = 0b010000, F6_VMUNARY0 = 0b010100;
static const uint32_t VS1_VCPOP = 0b10000, VS1_VFIRST = 0b10001;
static const uint32_t VS1_VIOTA = 0b10000, VS1_VID = 0b10001;

// direcciones de datos vectoriales (BYTE) -- lejos del codigo y del
// scratch escalar (256/260), sin colision
static const uint32_t VEC_SRC1 = 512;  // v1: {10,20,30,40}
static const uint32_t VEC_SRC2 = 528;  // v2: {1,2,3,4}
static const uint32_t VEC_DST  = 544;  // destino del vse32.v
static const uint32_t VEC_MASK = 560;  // patron de mascara para v0

static std::vector<uint16_t> build_test_program() {
    const uint32_t OP     = rv32i::Opcode::OP;
    const uint32_t OP_IMM = rv32i::Opcode::OP_IMM;
    const uint32_t LOAD   = rv32i::Opcode::LOAD;
    const uint32_t STORE  = rv32i::Opcode::STORE;
    const uint32_t MULDIV = rv32i::Funct7::MULDIV;
    const uint32_t OP_FP  = rv32i::Opcode::OP_FP;

    std::vector<uint16_t> prog;
    auto push32 = [&](uint32_t w) { prog.push_back(w & 0xFFFF); prog.push_back(w >> 16); };
    auto push16 = [&](uint16_t h) { prog.push_back(h); };

    // ---- parte I+M (pcs 0..56) ----
    push32(i_type(OP_IMM, 1, F3A::ADD_SUB, 0, 5));            //   0: addi x1,x0,5      d0
    push32(i_type(OP_IMM, 2, F3A::ADD_SUB, 0, 10));           //   4: addi x2,x0,10     d1
    push32(r_type(OP, 3, F3M::MUL, 1, 2, MULDIV));            //   8: mul  x3,x1,x2     d2  (lat 3)
    push32(i_type(OP_IMM, 4, F3A::ADD_SUB, 0, 7));            //  12: addi x4,x0,7      d3  (OOO vs d2)
    push32(r_type(OP, 5, F3A::ADD_SUB, 3, 4, 0));             //  16: add  x5,x3,x4     d4  = 57
    // Direcciones de scratch (256/260) elegidas MUY lejos del programa a
    // proposito: aca instrucciones y datos comparten la misma RAM (a
    // diferencia de la pista HLS, que tiene imem/dmem fisicamente
    // separados) -- usar una direccion dentro del rango del propio
    // programa auto-modificaria el codigo en ejecucion.
    push32(s_type(STORE, rv32i::Funct3_STORE::SW, 0, 5, 1024)); //  20: sw   x5,1024(x0)   d5
    push32(i_type(LOAD, 6, rv32i::Funct3_LOAD::LW, 0, 1024));   //  24: lw   x6,1024(x0)   d6
    push32(b_type(rv32i::Funct3_BRANCH::BEQ, 1, 1, 8));       //  28: beq  x1,x1,+8     d7  (tomado)
    push32(i_type(OP_IMM, 7, F3A::ADD_SUB, 0, 99));           //  32: addi x7,x0,99     (saltada)
    push32(i_type(OP_IMM, 7, F3A::ADD_SUB, 0, 1));            //  36: addi x7,x0,1      d8
    push32(j_type(8, 8));                                     //  40: jal  x8,+8        d9  (link 44)
    push32(i_type(OP_IMM, 9, F3A::ADD_SUB, 0, 88));           //  44: addi x9,x0,88     (saltada)
    push32(u_type(rv32i::Opcode::LUI, 10, 0x12345));          //  48: lui  x10,0x12345  d10
    push32(r_type(OP, 11, F3M::DIV, 3, 1, MULDIV));           //  52: div  x11,x3,x1    d11 (lat 8)
    push32(r_type(OP, 12, F3A::ADD_SUB, 2, 1, rv32i::Funct7::ALT)); // 56: sub x12,x2,x1 d12 (OOO vs d11)

    // ---- parte F (pcs 60..104) ----
    push32(i_type(OP_IMM, 20, F3A::ADD_SUB, 0, 3));           //  60: addi x20,x0,3     d13
    push32(r_type(OP_FP, 1, 0, 20, rv32i::Rs2_FCVT::W, F7F::FCVT_S_W)); // 64: fcvt.s.w f1,x20 d14 (3.0)
    push32(i_type(OP_IMM, 21, F3A::ADD_SUB, 0, 4));           //  68: addi x21,x0,4     d15
    push32(r_type(OP_FP, 2, 0, 21, rv32i::Rs2_FCVT::W, F7F::FCVT_S_W)); // 72: fcvt.s.w f2,x21 d16 (4.0)
    push32(r_type(OP_FP, 3, 0, 1, 2, F7F::FMUL_S));           //  76: fmul.s f3,f1,f2   d17 (12.0)
    push32(r_type(rv32i::Opcode::FMADD, 5, 0, 1, 2, (3u << 2))); // 80: fmadd.s f5,f1,f2,f3 d18 (24.0)
    push32(s_type(rv32i::Opcode::STORE_FP, rv32i::Funct3_FP_MEM::W, 0, 5, 1028)); // 84: fsw f5,1028(x0) d19
    push32(i_type(rv32i::Opcode::LOAD_FP, 6, rv32i::Funct3_FP_MEM::W, 0, 1028));  // 88: flw f6,1028(x0) d20
    push32(r_type(OP_FP, 22, rv32i::Funct3_FCMP::FEQ, 5, 6, F7F::FCMP_S)); // 92: feq.s x22,f5,f6 d21
    push32(r_type(OP_FP, 23, 0, 3, 0, F7F::FMV_X_W_FCLASS_S)); // 96: fmv.x.w x23,f3    d22
    push32(r_type(OP_FP, 7, 0, 3, 1, F7F::FDIV_S));           // 100: fdiv.s f7,f3,f1   d23 (lat 8)
    push32(i_type(OP_IMM, 24, F3A::ADD_SUB, 0, 2));           // 104: addi x24,x0,2     d24 (OOO vs d23)

    // ---- parte C (pcs 108..114) ----
    push16(0x47A5);                                           // 108: c.li x15,9        d25 (16 bits)
    push32(i_type(OP_IMM, 16, F3A::ADD_SUB, 0, 21));          // 110: addi x16,x0,21    d26 (straddle)
    push16(0x078D);                                           // 114: c.addi x15,3      d27 -> x15=12

    // ---- parte RVV: coprocesamiento (pcs 116..168) ----
    // Todo programa RVV real arranca con vsetvli: al reset vtype tiene
    // vill y vl=0, asi que sin el las vectoriales no procesarian nada.
    push32(enc_vsetvli(/*rd=*/19, /*rs1=*/0, VTYPE_E32_M1));               // 116: vsetvli x19,x0,e32,m1 d28 -> vl=VLMAX=4
    push32(i_type(OP_IMM, 13, F3A::ADD_SUB, 0, VEC_SRC1));                 // 120: addi x13,x0,512  d29
    push32(enc_vec_mem(rv32i::Opcode::LOAD_FP, /*vd=*/1, /*rs1=*/13));     // 124: vle32.v v1,(x13) d30 -> {10,20,30,40}
    push32(i_type(OP_IMM, 14, F3A::ADD_SUB, 0, VEC_SRC2));                 // 128: addi x14,x0,528  d31
    push32(enc_vec_mem(rv32i::Opcode::LOAD_FP, /*vd=*/2, /*rs1=*/14));     // 132: vle32.v v2,(x14) d32 -> {1,2,3,4}
    push32(enc_op_v(0b000000, 2, 1, 0b000, /*vd=*/3));                    // 136: vadd.vv v3,v2,v1 d33 (lat 3) -> {11,22,33,44}
    push32(i_type(OP_IMM, 25, F3A::ADD_SUB, 0, 99));                      // 140: addi x25,x0,99   d34 (OOO vs d33)
    push32(enc_op_v(0b100101, 1, 1, 0b010, /*vd=*/4));                   // 144: vmul.vv v4,v1,v1 d35 (lat 3) -> {100,400,900,1600}
    push32(i_type(OP_IMM, 26, F3A::ADD_SUB, 0, 77));                      // 148: addi x26,x0,77   d36 (OOO vs d35)
    push32(enc_vec_mem(rv32i::Opcode::LOAD_FP, /*vd=*/5, /*rs1=*/13));     // 152: vle32.v v5,(x13) d37 -> centinela {10,20,30,40}
    push32(i_type(OP_IMM, 17, F3A::ADD_SUB, 0, VEC_DST));                 // 156: addi x17,x0,544  d38
    push32(enc_vec_mem(rv32i::Opcode::STORE_FP, /*vs3=*/3, /*rs1=*/17));   // 160: vse32.v v3,(x17) d39 -> mem[544..559]

    // ---- largo vectorial DINAMICO: vl=2 (AVL=2 < VLMAX=4) ----
    // v5 quedara {11,22, 30,40}: los dos primeros calculados con vl=2, los
    // dos ultimos sobrevivientes del centinela (tail-undisturbed).
    push32(i_type(OP_IMM, 27, F3A::ADD_SUB, 0, 2));                       // 164: addi x27,x0,2    d40 (AVL=2)
    push32(enc_vsetvli(/*rd=*/28, /*rs1=*/27, VTYPE_E32_M1));             // 168: vsetvli x28,x27  d41 -> vl=2
    push32(enc_op_v(0b000000, 2, 1, 0b000, /*vd=*/5));                    // 172: vadd.vv v5,v2,v1 d42 -> solo lanes 0..1

    // ---- Fase 1: familias de ops, formas escalares y MASCARA ----
    // (v1={10,20,30,40}, v2={1,2,3,4}, vl vuelve a 4 antes de empezar)
    // OJO: rd debe ser != x0. Con rd=x0 Y rs1=x0 la spec manda CONSERVAR
    // el vl actual (solo cambia vtype) -- aca queremos volver a VLMAX=4.
    push32(enc_vsetvli(/*rd=*/19, /*rs1=*/0, VTYPE_E32_M1));               // 176: vsetvli x19,x0 -> vl=4  d43
    push32(enc_op_v(F6_VAND, 2, 1, F3_OPIVV, /*vd=*/6));                   // 180: vand.vv v6,v2,v1        d44
    push32(enc_op_v(F6_VOR,  2, 1, F3_OPIVV, /*vd=*/7));                   // 184: vor.vv  v7,v2,v1        d45
    push32(enc_op_v(F6_VMAX, 2, 1, F3_OPIVV, /*vd=*/8));                   // 188: vmax.vv v8,v2,v1        d46
    push32(enc_op_v(F6_VSLL, 2, 1, F3_OPIVV, /*vd=*/9));                   // 192: vsll.vv v9,v2,v1        d47
    push32(enc_op_v(F6_VDIV, 1, 2, F3_OPMVV, /*vd=*/10));                  // 196: vdiv.vv v10,v1,v2       d48

    // formas vector-escalar: .vi (inmediato) y .vx (registro entero)
    push32(enc_op_v(F6_VADD, 1, /*imm=*/3, F3_OPIVI, /*vd=*/11));          // 200: vadd.vi v11,v1,3        d49
    push32(i_type(OP_IMM, 29, F3A::ADD_SUB, 0, 100));                      // 204: addi x29,x0,100         d50
    push32(enc_op_v(F6_VADD, 1, /*rs1=*/29, F3_OPIVX, /*vd=*/12));         // 208: vadd.vx v12,v1,x29      d51
    push32(enc_op_v(F6_VRSUB, 1, /*imm=*/0, F3_OPIVI, /*vd=*/13));         // 212: vrsub.vi v13,v1,0       d52 (= -v1)

    // MASCARA: v0 = {1,0,1,0} -> solo los elementos 0 y 2 se escriben.
    // v14 se precarga con el centinela v1 para ver que los inactivos
    // sobreviven (politica mask-undisturbed).
    push32(i_type(OP_IMM, 30, F3A::ADD_SUB, 0, 0b0101));                   // 216: addi x30,x0,0b0101      d53
    push32(i_type(OP_IMM, 31, F3A::ADD_SUB, 0, VEC_MASK));                 // 220: addi x31,x0,560         d54
    push32(s_type(STORE, rv32i::Funct3_STORE::SW, 31, 30, 0));             // 224: sw x30,0(x31)           d55
    push32(enc_vec_mem(rv32i::Opcode::LOAD_FP, /*vd=*/0, /*rs1=*/31));     // 228: vle32.v v0,(x31)        d56 -> v0.mask
    push32(enc_vec_mem(rv32i::Opcode::LOAD_FP, /*vd=*/14, /*rs1=*/13));    // 232: vle32.v v14,(x13)       d57 -> centinela {10,20,30,40}
    push32(enc_op_v_m(F6_VADD, 2, 1, F3_OPIVV, /*vd=*/14, /*vm=*/0));      // 236: vadd.vv v14,v2,v1,v0.t  d58 -> solo lanes 0 y 2

    // ---- Fase 2: ANCHO DE ELEMENTO VARIABLE (EEW 8 y 16) ----
    // Los datos en 512 son las palabras {10,20,30,40}; leidos como BYTES
    // (little-endian) dan 16 elementos: 0A,00,00,00, 14,00,00,00, ...
    push32(enc_vsetvli(/*rd=*/18, /*rs1=*/0, VTYPE_E8_M1));                // 240: vsetvli x18,x0,e8,m1 -> vl=VLMAX=16
    push32(enc_vec_mem_w(rv32i::Opcode::LOAD_FP, /*vd=*/15, /*rs1=*/13, WIDTH_8)); // 244: vle8.v v15,(x13)
    push32(enc_op_v(F6_VADD, 15, /*imm=*/1, F3_OPIVI, /*vd=*/16));         // 248: vadd.vi v16,v15,1 (16 elems de 8 bits)

    // Ahora SEW=16: 8 elementos de media palabra. Se reusa x18 como rd
    // (su valor final, 8, se verifica abajo); que los 16 bytes de v16
    // hayan sido procesados ya prueba que el caso e8 uso vl=16.
    push32(enc_vsetvli(/*rd=*/18, /*rs1=*/0, VTYPE_E16_M1));               // 252: vsetvli x18,x0,e16,m1 -> vl=VLMAX=8
    push32(enc_vec_mem_w(rv32i::Opcode::LOAD_FP, /*vd=*/17, /*rs1=*/13, WIDTH_16)); // 256: vle16.v v17,(x13)
    push32(enc_op_v(F6_VADD, 17, /*imm=*/2, F3_OPIVI, /*vd=*/18));         // 260: vadd.vi v18,v17,2 (8 elems de 16 bits)

    // ---- Fase 3: comparaciones, merge, logica de mascaras, unary ----
    // Volvemos a SEW=32 (vl=4). v1={10,20,30,40}, v2={1,2,3,4}.
    push32(enc_vsetvli(/*rd=*/18, /*rs1=*/0, VTYPE_E32_M1));               // vsetvli x18,x0,e32 -> vl=4

    // comparaciones -> escriben MASCARAS (un bit por elemento)
    push32(enc_op_v(F6_VMSEQ, 1, 1, F3_OPIVV, /*vd=*/19));                 // vmseq.vv v19,v1,v1 -> todos 1 = 0b1111
    push32(enc_op_v(F6_VMSLT, 1, 2, F3_OPIVV, /*vd=*/20));                 // vmslt.vv v20,v1,v2 -> v1<v2? no -> 0b0000
    push32(enc_op_v(F6_VMSGT, 1, /*imm=*/15, F3_OPIVI, /*vd=*/21));        // vmsgt.vi v21,v1,15 -> {10,20,30,40}>15 = 0b1110

    // logica entre mascaras: v22 = v19 AND v21 = 0b1111 & 0b1110 = 0b1110
    push32(enc_op_v(F6_VMAND, 19, 21, F3_OPMVV, /*vd=*/22));               // vmand.mm v22,v19,v21
    // v23 = v20 OR v21 = 0b0000 | 0b1110 = 0b1110
    push32(enc_op_v(F6_VMOR, 20, 21, F3_OPMVV, /*vd=*/23));                // vmor.mm v23,v20,v21

    // vmerge: v0 = 0b0101 (cargado en Fase 1) -> toma v1 donde el bit es 1
    push32(enc_op_v_m(F6_VMERGE, 2, 1, F3_OPIVV, /*vd=*/24, /*vm=*/0));    // vmerge.vvm v24,v2,v1,v0
    // vmv.v.v (mismo funct6 con vm=1): copia v1 a v25
    push32(enc_op_v(F6_VMERGE, 0, 1, F3_OPIVV, /*vd=*/25));                // vmv.v.v v25,v1

    // vid.v -> v26 = {0,1,2,3}
    push32(enc_op_v(F6_VMUNARY0, 0, VS1_VID, F3_OPMVV, /*vd=*/26));        // vid.v v26
    // viota.m sobre v21 (0b1110) -> prefijos {0,0,1,2}
    push32(enc_op_v(F6_VMUNARY0, 21, VS1_VIOTA, F3_OPMVV, /*vd=*/27));     // viota.m v27,v21

    // vcpop.m sobre v21 -> 3 bits en 1 ; vfirst.m -> primer bit = indice 1
    push32(enc_op_v(F6_VWXUNARY0, 21, VS1_VCPOP,  F3_OPMVV, /*vd=*/29));   // vcpop.m x29,v21  -> 3
    push32(enc_op_v(F6_VWXUNARY0, 21, VS1_VFIRST, F3_OPMVV, /*vd=*/30));   // vfirst.m x30,v21 -> 1

    push16(0x0000);                                           // fin de programa

    return prog;
}


// funct6 de la Fase 4 (tabla de la seccion 19 de la spec)
static const uint32_t F6_VREDSUM = 0b000000, F6_VREDAND = 0b000001, F6_VREDMAX = 0b000111;
static const uint32_t F6_VRGATHER = 0b001100, F6_VSLIDEUP = 0b001110, F6_VSLIDEDOWN = 0b001111;
static const uint32_t F6_VCOMPRESS = 0b010111;
static const uint32_t F6_VSADDU = 0b100000, F6_VSSUBU = 0b100010, F6_VAADD = 0b001001;
static const uint32_t F3_OPMVX = 0b110;

// Programa de la Fase 4: reducciones, permutaciones y punto fijo.
// Va en una suite aparte para tener los 32 registros vectoriales libres.
static std::vector<uint16_t> build_fase4_program() {
    const uint32_t OP_IMM = rv32i::Opcode::OP_IMM;
    const uint32_t STORE  = rv32i::Opcode::STORE;
    std::vector<uint16_t> p;
    auto push32 = [&](uint32_t w) { p.push_back(w & 0xFFFF); p.push_back(w >> 16); };
    auto push16 = [&](uint16_t h) { p.push_back(h); };

    push32(enc_vsetvli(1, 0, VTYPE_E32_M1));                      // vsetvli x1,x0,e32 -> vl=4
    push32(i_type(OP_IMM, 2, F3A::ADD_SUB, 0, VEC_SRC1));         // x2 = 512
    push32(enc_vec_mem(rv32i::Opcode::LOAD_FP, 1, 2));            // vle32.v v1,(x2) -> {10,20,30,40}
    push32(i_type(OP_IMM, 3, F3A::ADD_SUB, 0, VEC_SRC2));         // x3 = 528
    push32(enc_vec_mem(rv32i::Opcode::LOAD_FP, 2, 3));            // vle32.v v2,(x3) -> {1,2,3,4}

    // --- 4a: reducciones (acumulador inicial = vs1[0], resultado en vd[0]) ---
    push32(enc_op_v(F6_VREDSUM, 1, 2, F3_OPMVV, 6));              // v6[0] = v2[0] + sum(v1) = 1+100 = 101
    push32(enc_op_v(F6_VREDMAX, 1, 2, F3_OPMVV, 7));              // v7[0] = max(1, 10..40)  = 40
    push32(enc_op_v(F6_VREDAND, 1, 1, F3_OPMVV, 8));              // v8[0] = 10&10&20&30&40  = 0

    // --- 4b: permutaciones ---
    push32(enc_op_v(F6_VMERGE, 0, 2, F3_OPIVV, 9));               // vmv.v.v v9,v2 -> {1,2,3,4}
    push32(enc_op_v(F6_VSLIDEUP, 1, 1, F3_OPIVI, 9));             // vslideup.vi v9,v1,1 -> {1,10,20,30}
    push32(enc_op_v(F6_VSLIDEDOWN, 1, 1, F3_OPIVI, 10));          // vslidedown.vi v10,v1,1 -> {20,30,40,0}
    push32(enc_op_v(F6_VRGATHER, 1, 2, F3_OPIVI, 11));            // vrgather.vi v11,v1,2 -> {30,30,30,30}

    // mascara 0b0101 en v0, para vcompress
    push32(i_type(OP_IMM, 4, F3A::ADD_SUB, 0, 0b0101));           // x4 = 5
    push32(i_type(OP_IMM, 5, F3A::ADD_SUB, 0, VEC_MASK));         // x5 = 560
    push32(s_type(STORE, rv32i::Funct3_STORE::SW, 5, 4, 0));      // sw x4,0(x5)
    push32(enc_vec_mem(rv32i::Opcode::LOAD_FP, 0, 5));            // vle32.v v0,(x5) -> mascara
    push32(enc_op_v(F6_VCOMPRESS, 1, 0, F3_OPMVV, 12));           // vcompress.vm v12,v1,v0 -> {10,30}

    // --- 4c: punto fijo ---
    push32(i_type(OP_IMM, 6, F3A::ADD_SUB, 0, -1));               // x6 = 0xFFFFFFFF
    push32(enc_op_v(F6_VSADDU, 1, 6, F3_OPIVX, 13));              // vsaddu.vx v13,v1,x6 -> satura a 0xFFFFFFFF
    push32(i_type(OP_IMM, 7, F3A::ADD_SUB, 0, 10));               // x7 = 10
    push32(enc_op_v(F6_VSSUBU, 2, 7, F3_OPIVX, 14));              // vssubu.vx v14,v2,x7 -> satura a 0
    push32(enc_op_v(F6_VAADD, 1, 2, F3_OPMVV, 15));               // vaadd.vv v15,v1,v2 -> promedio redondeado

    push16(0x0000);
    return p;
}

// ---------------------------------------------------------------------
// Fase 5: los modos de direccionamiento de memoria.
//
// Codificacion de un load/store vectorial (seccion 7.1 de la spec):
//   nf[31:29] mew[28] mop[27:26] vm[25] lumop/rs2/vs2[24:20]
//   rs1[19:15] width[14:12] vd|vs3[11:7] opcode[6:0]
// ---------------------------------------------------------------------
static uint32_t enc_vmem(uint32_t opcode, uint32_t vd_or_vs3, uint32_t rs1,
                         uint32_t width, uint32_t mop, uint32_t f24_20,
                         uint32_t nf = 0, uint32_t vm = 1) {
    return (nf << 29) | (0u << 28) | (mop << 26) | (vm << 25) | (f24_20 << 20) |
           (rs1 << 15) | (width << 12) | (vd_or_vs3 << 7) | opcode;
}
static const uint32_t MOP_UNIT = 0b00, MOP_IDX_U = 0b01, MOP_STRIDED = 0b10;
static const uint32_t LUMOP_UNIT = 0b00000, LUMOP_WHOLE = 0b01000;
static const uint32_t LUMOP_MASK = 0b01011, LUMOP_FOF = 0b10000;
static const uint32_t WIDTH_32t = 0b110, WIDTH_64t = 0b111;

// Direcciones de datos de la Fase 5 (memoria UNIFICADA: lejos del programa)
static const uint32_t F5_SRC     = 1024; // 8 palabras: 10,20,...,80
static const uint32_t F5_IDX     = 1088; // offsets en bytes: {12,8,4,0}
static const uint32_t F5_SEG     = 1152; // pares intercalados (x,y)
static const uint32_t F5_STRIDED = 1280; // destino del store strided
static const uint32_t F5_SCATTER = 1344; // destino del scatter indexado
static const uint32_t F5_WHOLE   = 1408; // destino del registro completo
static const uint32_t F5_MASK    = 1472; // patron de mascara
static const uint32_t F5_MASKDST = 1536; // destino del vsm.v

static std::vector<uint16_t> build_fase5_program() {
    const uint32_t OP_IMM = rv32i::Opcode::OP_IMM;
    const uint32_t LDFP   = rv32i::Opcode::LOAD_FP;
    const uint32_t STFP   = rv32i::Opcode::STORE_FP;
    std::vector<uint16_t> p;
    auto push32 = [&](uint32_t w) { p.push_back(w & 0xFFFF); p.push_back(w >> 16); };
    auto push16 = [&](uint16_t h) { p.push_back(h); };

    push32(enc_vsetvli(1, 0, VTYPE_E32_M1));                     // vl=VLMAX=4
    push32(i_type(OP_IMM, 20, F3A::ADD_SUB, 0, F5_SRC));         // x20 = base de datos

    // ---- 1. STRIDED: paso de 8 bytes -> toma una palabra SI y una NO ----
    push32(i_type(OP_IMM, 2, F3A::ADD_SUB, 0, 8));               // x2 = paso 8B
    push32(enc_vmem(LDFP, 1, 20, WIDTH_32t, MOP_STRIDED, 2));    // vlse32.v v1,(x20),x2
    // guardar con el mismo paso y releer contiguo: si el store strided
    // funciona, la relectura contigua NO da lo mismo (quedan huecos).
    push32(i_type(OP_IMM, 3, F3A::ADD_SUB, 0, F5_STRIDED));
    push32(enc_vmem(STFP, 1, 3, WIDTH_32t, MOP_STRIDED, 2));     // vsse32.v v1,(x3),x2
    push32(enc_vmem(LDFP, 2, 3, WIDTH_32t, MOP_UNIT, LUMOP_UNIT));

    // ---- 2. INDEXADO: los offsets salen de un registro VECTORIAL ----
    push32(i_type(OP_IMM, 4, F3A::ADD_SUB, 0, F5_IDX));
    push32(enc_vmem(LDFP, 4, 4, WIDTH_32t, MOP_UNIT, LUMOP_UNIT)); // v4 = {12,8,4,0}
    push32(enc_vmem(LDFP, 3, 20, WIDTH_32t, MOP_IDX_U, 4));        // vluxei32.v v3,(x20),v4
    push32(i_type(OP_IMM, 5, F3A::ADD_SUB, 0, F5_SCATTER));
    push32(enc_vmem(STFP, 1, 5, WIDTH_32t, MOP_IDX_U, 4));         // vsuxei32.v v1,(x5),v4
    push32(enc_vmem(LDFP, 5, 5, WIDTH_32t, MOP_UNIT, LUMOP_UNIT));

    // ---- 3. SEGMENTADO: separa un arreglo de pares en dos vectores ----
    push32(i_type(OP_IMM, 6, F3A::ADD_SUB, 0, F5_SEG));
    push32(enc_vmem(LDFP, 6, 6, WIDTH_32t, MOP_UNIT, LUMOP_UNIT, /*nf=*/1));

    // ---- 4. REGISTRO COMPLETO: no mira vl (se prueba con vl=0) ----
    push32(i_type(OP_IMM, 7, F3A::ADD_SUB, 0, F5_WHOLE));
    push32(enc_vmem(STFP, 1, 7, WIDTH_32t, MOP_UNIT, LUMOP_WHOLE)); // vs1r.v v1,(x7)
    push32(i_type(OP_IMM, 8, F3A::ADD_SUB, 0, 0));                  // x8 = 0 -> AVL=0
    push32(enc_vsetvli(9, 8, VTYPE_E32_M1));                        // vl=0
    push32(enc_vmem(LDFP, 8, 7, WIDTH_32t, MOP_UNIT, LUMOP_WHOLE)); // vl1r.v CON vl=0
    push32(enc_vsetvli(9, 0, VTYPE_E32_M1));                        // restaura vl=4

    // ---- 5. MASCARA: vsm.v / vlm.v mueven bits, no elementos ----
    push32(i_type(OP_IMM, 10, F3A::ADD_SUB, 0, 0b1010));
    push32(i_type(OP_IMM, 11, F3A::ADD_SUB, 0, F5_MASK));
    push32(s_type(rv32i::Opcode::STORE, rv32i::Funct3_STORE::SW, 11, 10, 0));
    push32(enc_vmem(LDFP, 12, 11, 0b000, MOP_UNIT, LUMOP_MASK));    // vlm.v
    push32(i_type(OP_IMM, 12, F3A::ADD_SUB, 0, F5_MASKDST));
    push32(enc_vmem(STFP, 12, 12, 0b000, MOP_UNIT, LUMOP_MASK));    // vsm.v
    push32(enc_vmem(LDFP, 13, 12, WIDTH_32t, MOP_UNIT, LUMOP_UNIT));// relee como palabra

    // ---- 6. FAULT-ONLY-FIRST: recorta vl en vez de atrapar ----
    // base = ultimas 2 palabras de la RAM; los elementos 2 y 3 se salen.
    push32(u_type(rv32i::Opcode::LUI, 14, memory_map::RAM_SIZE >> 12));
    push32(i_type(OP_IMM, 14, F3A::ADD_SUB, 14, -8));
    push32(enc_vmem(LDFP, 14, 14, WIDTH_32t, MOP_UNIT, LUMOP_FOF)); // vle32ff.v
    push32(i_type(rv32i::Opcode::SYSTEM, 15, rv32i::Funct3_SYSTEM::CSRRS,
                  0, rv32i::CSR::VL));                              // csrr x15, vl
    push32(enc_vsetvli(16, 0, VTYPE_E32_M1));                       // restaura vl=4
    push32(enc_op_v(F6_VMERGE, 0, 15, F3_OPIVX, 16));               // vmv.v.x v16,x15

    // ---- 7. EEW=64 esta FUERA de Zve32x -> instruccion ilegal ----
    // Sin mtvec instalado el core se detiene: la prueba es que la
    // instruccion que sigue NO llegue a escribir su registro.
    push32(enc_vmem(LDFP, 20, 0, WIDTH_64t, MOP_UNIT, LUMOP_UNIT)); // vle64.v -> ILEGAL
    push32(enc_op_v(F6_VMERGE, 0, 1, F3_OPIVX, 20));                // no deberia ejecutarse
    push16(0x0000);
    return p;
}

static int checks_fase5(ProcessorOOO& cpu) {
    struct VC { int vreg, lane; uint32_t expect; const char* what; };
    const VC cks[] = {
        {1,0,10, "vlse32.v paso=8: v1[0]=mem[0]"},
        {1,1,30, "vlse32.v paso=8: v1[1]=mem[+8], SALTEA una palabra"},
        {1,2,50, "vlse32.v paso=8"},
        {1,3,70, "vlse32.v paso=8"},
        {2,0,10, "vsse32.v: releido contiguo, v2[0]"},
        {2,1,0,  "vsse32.v: el hueco del paso quedo SIN escribir"},
        {2,2,30, "vsse32.v: v2[2] es el segundo elemento guardado"},
        {2,3,0,  "vsse32.v: segundo hueco intacto"},
        {3,0,40, "vluxei32.v offsets {12,8,4,0}: invierte el vector"},
        {3,1,30, "vluxei32.v"},
        {3,2,20, "vluxei32.v"},
        {3,3,10, "vluxei32.v"},
        {5,0,70, "vsuxei32.v: scatter, v1[3]=70 fue al offset 0"},
        {5,1,50, "vsuxei32.v scatter"},
        {5,2,30, "vsuxei32.v scatter"},
        {5,3,10, "vsuxei32.v scatter"},
        {6,0,1,  "vlseg2e32.v: campo 0 -> v6 (los 'x' de los pares)"},
        {6,1,2,  "vlseg2e32.v campo 0"},
        {6,3,4,  "vlseg2e32.v campo 0"},
        {7,0,100,"vlseg2e32.v: campo 1 -> v7 (los 'y'), DESINTERCALADO"},
        {7,1,200,"vlseg2e32.v campo 1"},
        {7,3,400,"vlseg2e32.v campo 1"},
        {8,0,10, "vl1r.v con vl=0: copia el registro COMPLETO igual"},
        {8,3,70, "vl1r.v ignora vl (el elemento 3 tambien se copio)"},
        {12,0,0b1010, "vlm.v: carga la mascara como bytes"},
        {13,0,0b1010, "vsm.v: y la vuelve a guardar identica"},
        {14,0,0xAAAA, "vle32ff.v: el elemento 0 SI se cargo"},
        {14,1,0xBBBB, "vle32ff.v: el elemento 1 tambien"},
        {16,0,2, "vle32ff.v RECORTO vl a 2 en vez de atrapar"},
        {20,0,0, "vle64.v (EEW=64, fuera de Zve32x) atrapo: v20 sin escribir"},
    };
    int fails = 0;
    for (const VC& c : cks) {
        uint32_t got = cpu.vregs[c.vreg * ProcessorOOO::VEC_LANES + c.lane];
        if (got != c.expect) {
            std::printf("FAIL  v%d[%d] = 0x%08x, esperado 0x%08x (%s)\n",
                        c.vreg, c.lane, got, c.expect, c.what);
            fails++;
        } else {
            std::printf("OK    v%d[%d] = 0x%08x (%s)\n", c.vreg, c.lane, got, c.what);
        }
    }
    return fails;
}


// ---------------------------------------------------------------------
// Fase 4d (widening/narrowing) + Fase 6 (vstart).
// Se corre con SEW=16 porque ensanchar exige 2*SEW <= ELEN = 32.
// ---------------------------------------------------------------------
static const uint32_t F6_VWADDU = 0b110000, F6_VWADD = 0b110001;
static const uint32_t F6_VWMULU = 0b111000, F6_VWMACC = 0b111101;
static const uint32_t F6_VNSRL  = 0b101100, F6_VNSRA = 0b101101;
static const uint32_t F6_VNCLIPU= 0b101110;
static const uint32_t F6_VADD_t = 0b000000;

static const uint32_t F4D_SRC1 = 1024; // 8 elementos de 16b: 1..8
static const uint32_t F4D_SRC2 = 1056; // 8 elementos de 16b: 100,200,...
static const uint32_t F4D_WIDE = 1088; // 4 palabras de 32b (fuente narrowing)

static std::vector<uint16_t> build_fase4d_program() {
    const uint32_t OP_IMM = rv32i::Opcode::OP_IMM;
    std::vector<uint16_t> p;
    auto push32 = [&](uint32_t w) { p.push_back(w & 0xFFFF); p.push_back(w >> 16); };
    auto push16 = [&](uint16_t h) { p.push_back(h); };

    // SEW=16 -> VLMAX = 128/16 = 8 elementos
    push32(enc_vsetvli(1, 0, VTYPE_E16_M1));
    push32(i_type(OP_IMM, 2, F3A::ADD_SUB, 0, F4D_SRC1));
    push32(enc_vec_mem_w(rv32i::Opcode::LOAD_FP, 2, 2, WIDTH_16));   // v2 = {1..8}
    push32(i_type(OP_IMM, 3, F3A::ADD_SUB, 0, F4D_SRC2));
    push32(enc_vec_mem_w(rv32i::Opcode::LOAD_FP, 4, 3, WIDTH_16));   // v4 = {100,200,...}

    // ---- 4d widening: el destino ocupa el PAR (v6,v7) ----
    push32(enc_op_v(F6_VWADDU, 2, 4, F3_OPMVV, 6));                  // vwaddu.vv v6,v2,v4
    push32(enc_op_v(F6_VWMULU, 2, 4, F3_OPMVV, 8));                  // vwmulu.vv v8,v2,v4
    push32(i_type(OP_IMM, 5, F3A::ADD_SUB, 0, -1));                  // x5 = 0xFFFFFFFF
    push32(enc_op_v(F6_VWADDU, 2, 5, F3_OPMVX, 10));                 // vwaddu.vx: zero-extiende
    push32(enc_op_v(F6_VWADD,  2, 5, F3_OPMVX, 12));                 // vwadd.vx : CON signo -> -1
    push32(enc_op_v(F6_VWMACC, 2, 4, F3_OPMVV, 8));                  // v8 += v2*v4

    // ---- 4d narrowing: la fuente v16 es ANCHA (2*SEW=32) ----
    push32(i_type(OP_IMM, 6, F3A::ADD_SUB, 0, F4D_WIDE));
    push32(enc_vsetvli(7, 0, VTYPE_E32_M1));                         // vl=4 para cargar 32b
    push32(enc_vec_mem(rv32i::Opcode::LOAD_FP, 16, 6));              // v16 = 4 palabras
    push32(enc_vsetvli(7, 0, VTYPE_E16_M1));                         // vuelve a SEW=16, vl=8
    push32(enc_op_v(F6_VNSRL, 16, 4, F3_OPIVI, 18));                 // vnsrl.wi v18,v16,4
    push32(enc_op_v(F6_VNCLIPU, 16, 0, F3_OPIVI, 19));               // vnclipu.wi 0 -> SATURA
    push32(enc_op_v(F6_VNSRA, 16, 4, F3_OPIVI, 20));                 // vnsra.wi (con signo)

    // ---- Fase 6: vstart ----
    push32(enc_op_v(F6_VMERGE, 0, 2, F3_OPIVV, 21));                 // vmv.v.v v21,v2 (centinela)
    push32(i_type(OP_IMM, 8, F3A::ADD_SUB, 0, 3));                   // x8 = 3
    push32(i_type(rv32i::Opcode::SYSTEM, 0, rv32i::Funct3_SYSTEM::CSRRW,
                  8, rv32i::CSR::VSTART));                           // csrw vstart, x8
    push32(enc_op_v(F6_VADD_t, 2, 4, F3_OPIVV, 21));                 // vadd.vv: solo del 3 en adelante
    push32(enc_op_v(F6_VADD_t, 2, 4, F3_OPIVV, 22));                 // vstart ya volvio a 0
    push32(i_type(rv32i::Opcode::SYSTEM, 9, rv32i::Funct3_SYSTEM::CSRRS,
                  0, rv32i::CSR::VSTART));                           // csrr x9, vstart
    push32(enc_op_v(F6_VMERGE, 0, 9, F3_OPIVX, 23));                 // vmv.v.x v23,x9

    push16(0x0000);
    return p;
}

static int checks_fase4d(ProcessorOOO& cpu) {
    auto e16 = [&](int vreg, int idx) -> uint32_t {
        uint32_t w = cpu.vregs[vreg * ProcessorOOO::VEC_LANES + idx/2];
        return (idx % 2) ? (w >> 16) : (w & 0xFFFF);
    };
    // elemento ANCHO (32b) del par que empieza en `vreg`
    auto e32p = [&](int vreg, int idx) -> uint32_t {
        return cpu.vregs[(vreg + idx/4) * ProcessorOOO::VEC_LANES + (idx % 4)];
    };
    struct { uint32_t got, expect; const char* what; } cks[] = {
        {e32p(6,0), 101u,   "vwaddu.vv: 1+100 en 32b, elemento 0 del par"},
        {e32p(6,3), 404u,   "vwaddu.vv: 4+400"},
        {e32p(6,4), 505u,   "vwaddu.vv: el elemento 4 YA ESTA EN v7 (EMUL=2)"},
        {e32p(6,7), 808u,   "vwaddu.vv: ultimo elemento, en v7"},
        {e32p(8,7), 6400u*2u, "vwmacc: 8*800 (vwmulu) + 8*800 (vwmacc)"},
        {e32p(10,0), 1u + 0xFFFFu, "vwaddu.vx: el escalar se toma de 16b SIN signo"},
        {e32p(12,0), 0u,           "vwadd.vx: el MISMO escalar CON signo es -1 -> 1-1=0"},
        {e16(18,0), 0x1234u, "vnsrl.wi 4: 0x00012345 >> 4, truncado a 16b"},
        {e16(18,1), 0x7FFFu, "vnsrl.wi 4: 0x0007FFFF >> 4 = 0x7FFF"},
        {e16(19,0), 0xFFFFu, "vnclipu.wi 0: 0x12345 NO cabe en 16b -> SATURA"},
        {e16(20,2), 0xF000u, "vnsra.wi 4: 0xFFFF0000 con signo"},
        {e16(21,0), 1u,   "vstart=3: el elemento 0 quedo SIN TOCAR (centinela v2)"},
        {e16(21,2), 3u,   "vstart=3: el elemento 2 tampoco se toco"},
        {e16(21,3), 4u+400u, "vstart=3: el elemento 3 SI se calculo"},
        {e16(21,7), 8u+800u, "vstart=3: y del 3 en adelante tambien"},
        {e16(22,0), 1u+100u, "vstart volvio a 0: el siguiente vadd escribe TODO"},
        {e16(23,0), 0u,   "csrr vstart = 0 tras completar la vectorial"},
    };
    int fails = 0;
    for (const auto& c : cks) {
        if (c.got != c.expect) {
            std::printf("FAIL  0x%08x, esperado 0x%08x (%s)\n", c.got, c.expect, c.what);
            fails++;
        } else {
            std::printf("OK    0x%08x (%s)\n", c.got, c.what);
        }
    }
    return fails;
}


// ---------------------------------------------------------------------
// Suite E: el predictor TAGE del frontend (espejo de la pista HLS).
// Lazo de 32 iteraciones + branch ALTERNANTE (T,N,T,N...): un bimodal
// falla ~50% para siempre; TAGE lo aprende con un bit de historia.
// ---------------------------------------------------------------------
static std::vector<uint16_t> build_tage_program() {
    const uint32_t OP_IMM = rv32i::Opcode::OP_IMM;
    std::vector<uint16_t> p;
    auto push32 = [&](uint32_t w) { p.push_back(w & 0xFFFF); p.push_back(w >> 16); };
    auto push16 = [&](uint16_t h) { p.push_back(h); };
    push32(i_type(OP_IMM, 1, F3A::ADD_SUB, 0, 0));     //  0: contador
    push32(i_type(OP_IMM, 2, F3A::ADD_SUB, 0, 32));    //  4: limite
    push32(i_type(OP_IMM, 4, F3A::ADD_SUB, 0, 0));     //  8: acumulador "par"
    push32(i_type(OP_IMM, 1, F3A::ADD_SUB, 1, 1));     // 12: (loop) x1++
    push32(i_type(OP_IMM, 3, F3A::AND, 1, 1));         // 16: andi x3,x1,1
    push32(b_type(rv32i::Funct3_BRANCH::BNE, 3, 0, 8));// 20: ALTERNA T,N,T,N...
    push32(i_type(OP_IMM, 4, F3A::ADD_SUB, 4, 1));     // 24: solo pares
    push32(b_type(rv32i::Funct3_BRANCH::BLT, 1, 2, -16)); // 28: backward, 31T+1N
    push32(s_type(rv32i::Opcode::STORE, rv32i::Funct3_STORE::SW, 0, 1, 1024));
    push32(s_type(rv32i::Opcode::STORE, rv32i::Funct3_STORE::SW, 0, 4, 1028));
    push16(0x0000);
    return p;
}

// ---------------------------------------------------------------------
// Carga de binarios ELF32 en la RAM (comun a las suites B, C y D)
// ---------------------------------------------------------------------
struct Elf32_Ehdr {
    uint8_t  e_ident[16];
    uint16_t e_type, e_machine;
    uint32_t e_version, e_entry, e_phoff, e_shoff, e_flags;
    uint16_t e_ehsize, e_phentsize, e_phnum, e_shentsize, e_shnum, e_shstrndx;
};
struct Elf32_Phdr {
    uint32_t p_type, p_offset, p_vaddr, p_paddr, p_filesz, p_memsz, p_flags, p_align;
};

// Codifica "jal x0, offset". El inmediato J-type va troceado y reordenado.
static uint32_t enc_jal_x0(int32_t off) {
    uint32_t i = static_cast<uint32_t>(off);
    return (((i >> 20) & 1) << 31) | (((i >> 1) & 0x3FF) << 21) |
           (((i >> 11) & 1) << 20) | (((i >> 12) & 0xFF) << 12) | 0x6Fu;
}

// Carga los segmentos PT_LOAD en la RAM (acceso backdoor previo a correr).
// Aca la memoria es UNIFICADA (instrucciones y datos comparten la RAM),
// asi que no hace falta duplicar el contenido como en la pista HLS.
static bool load_elf(const unsigned char* img, unsigned len, Memory& mem) {
    if (len < sizeof(Elf32_Ehdr)) return false;
    Elf32_Ehdr eh; std::memcpy(&eh, img, sizeof(eh));
    if (!(eh.e_ident[0] == 0x7f && eh.e_ident[1] == 'E')) return false;
    std::fill(mem.data.begin(), mem.data.end(), 0);
    for (int i = 0; i < eh.e_phnum; i++) {
        Elf32_Phdr ph;
        std::memcpy(&ph, img + eh.e_phoff + i * eh.e_phentsize, sizeof(ph));
        if (ph.p_type != 1) continue; // PT_LOAD
        if (ph.p_vaddr + ph.p_filesz > mem.data.size()) return false;
        std::memcpy(&mem.data[ph.p_vaddr], img + ph.p_offset, ph.p_filesz);
    }
    // El core arranca en pc=0 pero el entry del ELF puede estar en otra
    // direccion: se instala un stub de arranque (jal al entry) en 0, lo
    // mismo que hace el vector de reset de un core real.
    if (eh.e_entry != 0) {
        uint32_t j = enc_jal_x0(static_cast<int32_t>(eh.e_entry));
        std::memcpy(&mem.data[0], &j, 4);
    }
    return true;
}

// Lee una palabra de la RAM (backdoor, solo para verificar resultados).
static uint32_t mem_word(Memory& mem, uint32_t addr) {
    uint32_t v = 0; std::memcpy(&v, &mem.data[addr], 4); return v;
}

struct MemCheck { uint32_t addr; uint32_t expect; const char* what; };
static int check_mem(Memory& mem, const MemCheck* cks, int n) {
    int fails = 0;
    for (int i = 0; i < n; i++) {
        uint32_t got = mem_word(mem, cks[i].addr);
        if (got != cks[i].expect) {
            std::printf("FAIL  mem[0x%x] = 0x%x, esperado 0x%x (%s)\n",
                        cks[i].addr, got, cks[i].expect, cks[i].what);
            fails++;
        } else {
            std::printf("OK    mem[0x%x] = 0x%x (%s)\n", cks[i].addr, got, cks[i].what);
        }
    }
    return fails;
}

// Checks de la suite A (estado reconstruido desde el core, sin backdoor)
static int checks_isa_rvv(ProcessorOOO& cpu, Memory& mem) {
        int fails = 0;

        struct XC { int reg; uint32_t expect; const char* what; };
        const XC xchecks[] = {
            {1, 5,           "addi"},
            {2, 10,          "addi"},
            {3, 50,          "mul"},
            {4, 7,           "addi independiente del mul"},
            {5, 57,          "add dependiente del mul (RAW via CDB)"},
            {6, 57,          "lw round-trip del sw"},
            {7, 1,           "beq tomado (99 nunca ejecuto)"},
            {8, 44,          "link del jal"},
            {9, 0,           "instruccion saltada por jal"},
            {10, 0x12345000, "lui"},
            {11, 10,         "div"},
            {12, 5,          "sub independiente del div"},
            {15, 12,         "C.LI + C.ADDI comprimidas"},
            {16, 21,         "addi de 32 bits en pc%4==2 (straddle)"},
            {20, 3,          "addi (fuente de fcvt)"},
            {21, 4,          "addi (fuente de fcvt)"},
            {22, 1,          "feq.s f5==f6 (FP escribe banco entero)"},
            {23, 0x41400000, "fmv.x.w bits de 12.0f"},
            {24, 2,          "addi independiente del fdiv"},
            {13, 512,        "addi (base del primer vle32.v)"},
            {14, 528,        "addi (base del segundo vle32.v)"},
            {25, 99,         "addi independiente de vadd.vv"},
            {26, 77,         "addi independiente de vmul.vv"},
            {17, 544,        "addi (direccion del vse32.v)"},
            {19, 4,          "vsetvli x19,x0 -> vl=VLMAX=4 (rs1=x0 pide el maximo)"},
            {27, 2,          "addi (AVL para el segundo vsetvli)"},
            {28, 2,          "vsetvli x28,x27 -> vl=min(AVL=2,VLMAX=4)=2 (largo dinamico)"},
            {18, 4,          "vsetvli e32 al inicio de Fase 3 -> vl=VLMAX=4"},
            {29, 3,          "vcpop.m  sobre mascara 0b1110 -> 3 bits en 1"},
            {30, 1,          "vfirst.m sobre mascara 0b1110 -> primer bit en indice 1"},
        };
        for (const XC& c : xchecks) {
            if (cpu.regs[c.reg] != c.expect) {
                std::printf("FAIL  x%-2d = 0x%08x, esperado 0x%08x (%s)\n",
                            c.reg, cpu.regs[c.reg], c.expect, c.what);
                fails++;
            } else {
                std::printf("OK    x%-2d = 0x%08x (%s)\n", c.reg, cpu.regs[c.reg], c.what);
            }
        }

        const XC fchecks[] = {
            {1, 0x40400000, "fcvt.s.w 3 -> 3.0"},
            {2, 0x40800000, "fcvt.s.w 4 -> 4.0"},
            {3, 0x41400000, "fmul.s 3*4 = 12.0"},
            {5, 0x41C00000, "fmadd.s 3*4+12 = 24.0 (rs3)"},
            {6, 0x41C00000, "flw round-trip del fsw"},
            {7, 0x40800000, "fdiv.s 12/3 = 4.0"},
        };
        for (const XC& c : fchecks) {
            uint32_t bits = rv32i::float_to_bits(cpu.fregs[c.reg]);
            if (bits != c.expect) {
                std::printf("FAIL  f%-2d = 0x%08x, esperado 0x%08x (%s)\n",
                            c.reg, bits, c.expect, c.what);
                fails++;
            } else {
                std::printf("OK    f%-2d = 0x%08x (%s)\n", c.reg, bits, c.what);
            }
        }

        // Con el frontend especulativo (TAGE) los indices dN ya no son
        // estables (hay dispatches descartados por mispredict): los checks
        // de orden van POR PC. El ULTIMO dispatch de un pc es el valido.
        auto cyc_of_pc = [&](uint32_t pc) -> int {
            int r = -1;
            for (int d = 0; d < 64 && d < cpu.n_disp; d++)
                if (cpu.pc_of_disp[d] == pc) r = cpu.complete_cycle[d];
            return r;
        };
        struct OC { uint32_t later, earlier; const char* what; };
        const OC ooo[] = {
            {12,  8,   "addi(pc12) antes que mul(pc8)"},
            {56,  52,  "sub(pc56) antes que div(pc52)"},
            {104, 100, "addi(pc104) antes que fdiv(pc100) -- OOO cruzando bancos"},
            {140, 136, "addi(pc140) antes que vadd.vv(pc136) -- coprocesamiento"},
            {148, 144, "addi(pc148) antes que vmul.vv(pc144) -- coprocesamiento"},
        };
        for (const OC& c : ooo) {
            int cl = cyc_of_pc(c.later), ce = cyc_of_pc(c.earlier);
            if (cl > 0 && ce > 0 && cl < ce) {
                std::printf("OK    OOO: %s (ciclos %d < %d)\n", c.what, cl, ce);
            } else {
                std::printf("FAIL  OOO: %s NO se cumplio (ciclos %d vs %d)\n", c.what, cl, ce);
                fails++;
            }
        }

        // banco vectorial (acceso directo al miembro publico cpu.vregs --
        // igual que cpu.regs/cpu.fregs, no es backdoor de memoria)
        struct VC { int vreg, lane; uint32_t expect; };
        const VC vchecks[] = {
            {1,0,10},{1,1,20},{1,2,30},{1,3,40},      // v1 (vle32.v)
            {2,0,1}, {2,1,2}, {2,2,3}, {2,3,4},       // v2 (vle32.v)
            {3,0,11},{3,1,22},{3,2,33},{3,3,44},      // v3 (vadd.vv)
            {4,0,100},{4,1,400},{4,2,900},{4,3,1600}, // v4 (vmul.vv)
            // v5: vadd.vv con vl=2 -- lanes 0..1 calculados, lanes 2..3
            // conservan el centinela (tail undisturbed). Evidencia del
            // largo vectorial dinamico.
            {5,0,11},{5,1,22},{5,2,30},{5,3,40},
            // --- Fase 1: familias de operaciones enteras (v1={10,20,30,40}, v2={1,2,3,4}) ---
            {6,0,0},{6,1,0},{6,2,2},{6,3,0},              // vand.vv  v2&v1 (4&40=0)
            {7,0,11},{7,1,22},{7,2,31},{7,3,44},          // vor.vv   v2|v1
            {8,0,10},{8,1,20},{8,2,30},{8,3,40},          // vmax.vv  max(v2,v1)
            // vsll: el shift se enmascara a log2(SEW)=5 bits -> 4<<(40&31)=4<<8=1024
            {9,0,1024},{9,1,2097152},{9,2,3221225472u},{9,3,1024},
            {10,0,10},{10,1,10},{10,2,10},{10,3,10},      // vdiv.vv  v1/v2
            // --- Fase 1: formas vector-escalar ---
            {11,0,13},{11,1,23},{11,2,33},{11,3,43},      // vadd.vi  v1+3
            {12,0,110},{12,1,120},{12,2,130},{12,3,140},  // vadd.vx  v1+x29(100)
            {13,0,(uint32_t)-10},{13,1,(uint32_t)-20},{13,2,(uint32_t)-30},{13,3,(uint32_t)-40}, // vrsub.vi 0-v1
            // --- Fase 1: MASCARA (v0={1,0,1,0}) sobre el centinela {10,20,30,40} ---
            // lanes 0 y 2 calculados (v2+v1), lanes 1 y 3 intactos
            {14,0,11},{14,1,20},{14,2,33},{14,3,40},
            // --- Fase 2: EEW=8 (16 elementos). Fuente en 512 = palabras
            // {10,20,30,40} = bytes 0A,00,00,00, 14,00,00,00, ...
            // vadd.vi +1 sobre los 16 -> cada byte incrementado.
            {15,0,0x0000000A},{15,1,0x00000014},{15,2,0x0000001E},{15,3,0x00000028}, // vle8.v (mismos bits)
            {16,0,0x0101010B},{16,1,0x01010115},{16,2,0x0101011F},{16,3,0x01010129}, // vadd.vi por byte
            // --- Fase 2: EEW=16 (8 elementos), vadd.vi +2 por halfword ---
            {17,0,0x0000000A},{17,1,0x00000014},{17,2,0x0000001E},{17,3,0x00000028}, // vle16.v
            {18,0,0x0002000C},{18,1,0x00020016},{18,2,0x00020020},{18,3,0x0002002A},
            // --- Fase 3: comparaciones -> MASCARAS (un bit por elemento) ---
            {19,0,0b1111},   // vmseq.vv v1,v1 -> todos iguales
            {20,0,0b0000},   // vmslt.vv v1,v2 -> ninguno menor
            {21,0,0b1110},   // vmsgt.vi v1,15 -> {10,20,30,40}>15
            // --- Fase 3: logica entre mascaras ---
            {22,0,0b1110},   // vmand.mm v19,v21
            {23,0,0b1110},   // vmor.mm  v20,v21
            // --- Fase 3: vmerge (v0=0b0101 elige v1) y vmv.v.v ---
            {24,0,10},{24,1,2},{24,2,30},{24,3,4},
            {25,0,10},{25,1,20},{25,2,30},{25,3,40},
            // --- Fase 3: vid.v y viota.m sobre 0b1110 ---
            {26,0,0},{26,1,1},{26,2,2},{26,3,3},
            {27,0,0},{27,1,0},{27,2,1},{27,3,2},
        };
        for (const VC& c : vchecks) {
            uint32_t got = cpu.vregs[c.vreg * ProcessorOOO::VEC_LANES + c.lane];
            if (got != c.expect) {
                std::printf("FAIL  v%d[%d] = 0x%08x, esperado 0x%08x\n", c.vreg, c.lane, got, c.expect);
                fails++;
            } else {
                std::printf("OK    v%d[%d] = 0x%08x\n", c.vreg, c.lane, got);
            }
        }

        // Los COMMITS son arquitectonicos (no dependen del predictor);
        // los dispatches pueden incluir especulativos descartados.
        const int N_EXPECTED = 77;
        if (cpu.n_commit != N_EXPECTED) {
            std::printf("FAIL  commits: %d, esperados %d\n", cpu.n_commit, N_EXPECTED);
            fails++;
        }
        if (cpu.n_disp < cpu.n_commit) {
            std::printf("FAIL  dispatches (%d) < commits (%d)\n", cpu.n_disp, cpu.n_commit);
            fails++;
        }

        // lectura backdoor de la RAM, solo para verificacion del testbench
        uint32_t w64 = 0, w68 = 0;
        std::memcpy(&w64, &mem.data[1024], 4);
        std::memcpy(&w68, &mem.data[1028], 4);
        if (w64 != 57) {
            std::printf("FAIL  mem[1024] = 0x%08x, esperado 57 (sw al commit)\n", w64);
            fails++;
        } else {
            std::printf("OK    mem[1024] = 57 (sw escribio al commit)\n");
        }
        if (w68 != 0x41C00000) {
            std::printf("FAIL  mem[1028] = 0x%08x, esperado 0x41C00000 (fsw al commit)\n", w68);
            fails++;
        } else {
            std::printf("OK    mem[1028] = 0x41C00000 (fsw escribio 24.0f al commit)\n");
        }

        // round-trip del vse32.v: los 4 lanes de v3 escritos en VEC_DST=544
        uint32_t vs[4];
        for (int i = 0; i < 4; i++) std::memcpy(&vs[i], &mem.data[544 + i * 4], 4);
        if (vs[0] == 11 && vs[1] == 22 && vs[2] == 33 && vs[3] == 44) {
            std::printf("OK    mem[544..559] = {11,22,33,44} (vse32.v round-trip, resuelto en cabeza del ROB)\n");
        } else {
            std::printf("FAIL  mem[544..559] = {%u,%u,%u,%u}, esperado {11,22,33,44}\n",
                        vs[0], vs[1], vs[2], vs[3]);
            fails++;
        }

        return fails;
}

// Checks de la suite A2 (Fase 4)
static int checks_fase4(ProcessorOOO& cpu) {
    struct VC { int vreg, lane; uint32_t expect; const char* what; };
    const VC cks[] = {
        // 4a: reducciones -- solo el elemento 0 del destino
        {6,0,101, "vredsum.vs: v2[0]=1 + sum(v1)=100"},
        {7,0,40,  "vredmax.vs: max(1, 10,20,30,40)"},
        {8,0,0,   "vredand.vs: 10 & 10&20&30&40"},
        // 4b: permutaciones
        {9,0,1,  "vslideup.vi: el elemento 0 queda SIN TOCAR"},
        {9,1,10, "vslideup.vi: v9[1] = v1[0]"},
        {9,2,20, "vslideup.vi: v9[2] = v1[1]"},
        {9,3,30, "vslideup.vi: v9[3] = v1[2]"},
        {10,0,20,"vslidedown.vi: v10[0] = v1[1]"},
        {10,1,30,"vslidedown.vi"},
        {10,2,40,"vslidedown.vi"},
        {10,3,0, "vslidedown.vi: mas alla del vector -> 0"},
        {11,0,30,"vrgather.vi: todos toman v1[2]"},
        {11,3,30,"vrgather.vi"},
        {12,0,10,"vcompress.vm: empaqueta v1[0] (mascara 0b0101)"},
        {12,1,30,"vcompress.vm: y v1[2], CONSECUTIVOS"},
        // 4c: punto fijo
        {13,0,0xFFFFFFFF,"vsaddu.vx: 10+0xFFFFFFFF SATURA al maximo"},
        {13,3,0xFFFFFFFF,"vsaddu.vx satura"},
        {14,0,0,  "vssubu.vx: 1-10 SATURA a 0 (no envuelve)"},
        {14,3,0,  "vssubu.vx satura"},
        {15,0,6,  "vaadd.vv: (10+1)/2 con redondeo = 6"},
        {15,1,11, "vaadd.vv: (20+2)/2 = 11"},
        {15,2,17, "vaadd.vv: (30+3)/2 con redondeo = 17"},
        {15,3,22, "vaadd.vv: (40+4)/2 = 22"},
    };
    int fails = 0;
    for (const VC& c : cks) {
        uint32_t got = cpu.vregs[c.vreg * ProcessorOOO::VEC_LANES + c.lane];
        if (got != c.expect) {
            std::printf("FAIL  v%d[%d] = 0x%08x, esperado 0x%08x (%s)\n",
                        c.vreg, c.lane, got, c.expect, c.what);
            fails++;
        } else {
            std::printf("OK    v%d[%d] = 0x%08x (%s)\n", c.vreg, c.lane, got, c.what);
        }
    }
    return fails;
}

// ---------------------------------------------------------------------
SC_MODULE(Testbench) {
    SC_HAS_PROCESS(Testbench);
    ProcessorOOO& cpu;
    Memory&       mem;
    int           total_fails = 0;

    Testbench(sc_module_name n, ProcessorOOO& c, Memory& m)
        : sc_module(n), cpu(c), mem(m) { SC_THREAD(run); }

    // Corre un binario ELF de punta a punta sobre el core ya reseteado.
    bool correr_elf(const unsigned char* img, unsigned len, uint64_t max_ciclos) {
        cpu.reset_state();
        cpu.trace = false;              // las suites de ELF no imprimen traza
        if (!load_elf(img, len, mem)) {
            std::printf("FAIL  no se pudo cargar el ELF\n");
            return false;
        }
        bool ok = cpu.run_until_halt(max_ciclos);
        if (!ok) std::printf("FAIL  el programa no termino en %lu ciclos\n",
                             (unsigned long)max_ciclos);
        else     std::printf("      (termino en %lu ciclos)\n",
                             (unsigned long)cpu.cycle);
        return ok;
    }

    void suite(const char* nombre) {
        std::printf("\n===============================================================\n");
        std::printf("  %s\n", nombre);
        std::printf("===============================================================\n");
    }

    void run() {
        // ---- Suite A: ISA + RVV (programa ensamblado a mano) ----
        suite("A. ISA (I+M+F+C) + RVV Fases 1-3");
        cpu.reset_state();
        cpu.trace = false;
        {
            std::vector<uint16_t> program = build_test_program();
            std::fill(mem.data.begin(), mem.data.end(), 0);
            std::memcpy(mem.data.data(), program.data(), program.size() * sizeof(uint16_t));
            const uint32_t v1[4] = {10, 20, 30, 40};
            const uint32_t v2[4] = {1, 2, 3, 4};
            std::memcpy(&mem.data[VEC_SRC1], v1, sizeof(v1));
            std::memcpy(&mem.data[VEC_SRC2], v2, sizeof(v2));
        }
        cpu.run_until_halt();
        std::printf("      (%d dispatches, %d commits en %lu ciclos)\n",
                    cpu.n_disp, cpu.n_commit, (unsigned long)cpu.cycle);
        total_fails += checks_isa_rvv(cpu, mem);

        // ---- Suite A2: RVV Fase 4 (reducciones, permutaciones, punto fijo) ----
        suite("A2. RVV Fase 4: reducciones, permutaciones y punto fijo");
        cpu.reset_state();
        cpu.trace = false;
        {
            std::vector<uint16_t> pr = build_fase4_program();
            std::fill(mem.data.begin(), mem.data.end(), 0);
            std::memcpy(mem.data.data(), pr.data(), pr.size() * sizeof(uint16_t));
            const uint32_t v1[4] = {10, 20, 30, 40};
            const uint32_t v2[4] = {1, 2, 3, 4};
            std::memcpy(&mem.data[VEC_SRC1], v1, sizeof(v1));
            std::memcpy(&mem.data[VEC_SRC2], v2, sizeof(v2));
        }
        cpu.run_until_halt();
        std::printf("      (%d dispatches en %lu ciclos)\n", cpu.n_disp, (unsigned long)cpu.cycle);
        total_fails += checks_fase4(cpu);

        // ---- Suite A3: RVV Fase 5 (modos de direccionamiento de memoria) ----
        suite("A3. RVV Fase 5: strided, indexado, segmentado, fof, reg. completo");
        cpu.reset_state();
        cpu.trace = false;
        {
            std::vector<uint16_t> pr = build_fase5_program();
            std::fill(mem.data.begin(), mem.data.end(), 0);
            std::memcpy(mem.data.data(), pr.data(), pr.size() * sizeof(uint16_t));
            // 8 palabras consecutivas: 10..80
            uint32_t src[8]; for (int i = 0; i < 8; i++) src[i] = 10 * (i + 1);
            std::memcpy(&mem.data[F5_SRC], src, sizeof(src));
            // offsets en BYTES -> leen el vector AL REVES
            const uint32_t idx[4] = {12, 8, 4, 0};
            std::memcpy(&mem.data[F5_IDX], idx, sizeof(idx));
            // pares intercalados (1,100) (2,200) (3,300) (4,400), como un
            // arreglo de structs que el load segmentado va a desintercalar
            uint32_t seg[8];
            for (int i = 0; i < 4; i++) { seg[2*i] = i + 1; seg[2*i+1] = 100 * (i + 1); }
            std::memcpy(&mem.data[F5_SEG], seg, sizeof(seg));
            // al FINAL de la RAM, para el fault-only-first: solo dos
            // palabras validas; el tercer elemento ya se sale del rango.
            const uint32_t tail[2] = {0xAAAA, 0xBBBB};
            std::memcpy(&mem.data[memory_map::RAM_SIZE - 8], tail, sizeof(tail));
        }
        cpu.run_until_halt();
        std::printf("      (%d dispatches en %lu ciclos)\n", cpu.n_disp, (unsigned long)cpu.cycle);
        total_fails += checks_fase5(cpu);

        // ---- Suite A4: Fase 4d (widening/narrowing) + Fase 6 (vstart) ----
        suite("A4. RVV Fase 4d (widening/narrowing) + Fase 6 (vstart)");
        cpu.reset_state();
        cpu.trace = false;
        {
            std::vector<uint16_t> pr = build_fase4d_program();
            std::fill(mem.data.begin(), mem.data.end(), 0);
            std::memcpy(mem.data.data(), pr.data(), pr.size() * sizeof(uint16_t));
            // 8 elementos de 16 bits: 1,2,...,8
            uint16_t a[8]; for (int i = 0; i < 8; i++) a[i] = uint16_t(i + 1);
            std::memcpy(&mem.data[F4D_SRC1], a, sizeof(a));
            // 8 elementos de 16 bits: 100,200,...,800
            uint16_t b[8]; for (int i = 0; i < 8; i++) b[i] = uint16_t(100 * (i + 1));
            std::memcpy(&mem.data[F4D_SRC2], b, sizeof(b));
            // fuente ANCHA (32b) para narrowing
            const uint32_t w[4] = {0x00012345u, 0x0007FFFFu, 0xFFFF0000u, 0x00000FF0u};
            std::memcpy(&mem.data[F4D_WIDE], w, sizeof(w));
        }
        cpu.run_until_halt();
        std::printf("      (%d dispatches en %lu ciclos)\n", cpu.n_disp, (unsigned long)cpu.cycle);
        total_fails += checks_fase4d(cpu);

        // ---- Suite E: el predictor TAGE ----
        suite("E. Predictor TAGE: lazo + branch alternante");
        cpu.reset_state();
        cpu.trace = false;
        {
            std::vector<uint16_t> pr = build_tage_program();
            std::fill(mem.data.begin(), mem.data.end(), 0);
            std::memcpy(mem.data.data(), pr.data(), pr.size() * sizeof(uint16_t));
        }
        cpu.run_until_halt();
        {
            uint32_t br = cpu.stat_branches, mp = cpu.stat_mispredicts;
            uint32_t cnt, par;
            std::memcpy(&cnt, &mem.data[1024], 4);
            std::memcpy(&par, &mem.data[1028], 4);
            std::printf("      (%lu ciclos; %u branches, %u mispredicts -> %.1f%% acierto)\n",
                        (unsigned long)cpu.cycle, br, mp,
                        br ? 100.0 * (br - mp) / br : 0.0);
            struct { bool cond; const char* what; } cks[] = {
                {cnt == 32, "el lazo ejecuto sus 32 iteraciones BAJO especulacion"},
                {par == 16, "el camino alternante tomo exactamente los 16 pares"},
                {br == 64,  "64 branches condicionales retirados (32+32)"},
                {mp <= 16,  "TAGE aprendio: mispredicts <= 16 de 64 (bimodal: ~32)"},
            };
            for (auto& c : cks) {
                if (c.cond) { std::printf("OK    %s\n", c.what); }
                else        { std::printf("FAIL  %s\n", c.what); total_fails++; }
            }
        }

        // ---- Suite B: excepciones precisas y reanudables ----
        suite("B. Excepciones precisas y reanudables (ELF real)");
        if (!correr_elf(trap_elf, trap_elf_len, 5000)) total_fails++;
        else {
            static const MemCheck cks[] = {
                {0x80, 1,      "out[0]=1 escrito ANTES del ECALL"},
                {0x90, 0xBEEF, "el handler instalado en mtvec REALMENTE corrio"},
                {0x94, 11,     "mcause=11 (environment call from M-mode), por la spec"},
                {0x84, 2,      "out[1]=2 DESPUES del MRET -> excepcion REANUDABLE"},
            };
            total_fails += check_mem(mem, cks, sizeof(cks)/sizeof(cks[0]));
        }

        // ---- Suite C: UART + timer + modos M/U ----
        suite("C. UART + interrupcion de timer + modos M/U");
        std::printf("      salida del programa por el UART (periferico TLM): >>> ");
        bool okc = correr_elf(full_elf, full_elf_len, 50000);
        std::printf(" <<<\n");
        if (!okc) total_fails++;
        else {
            static const MemCheck cks[] = {
                {0x400, 1,    "el programa avanzo tras imprimir por el UART"},
                {0x404, 0x11, "el handler atendio la INTERRUPCION de timer y volvio con MRET"},
                {0x408, 8,    "ECALL desde U-mode reporta causa 8 (no 11): privilegios funcionan"},
            };
            total_fails += check_mem(mem, cks, sizeof(cks)/sizeof(cks[0]));
        }

        // ---- Suite D: printf de la biblioteca C real ----
        suite("D. printf de la biblioteca C real (newlib)");
        std::printf("      salida del programa por el UART (periferico TLM):\n>>> ");
        bool okd = correr_elf(printf_elf, printf_elf_len, 3000000);
        std::printf("<<<\n");
        if (!okd) total_fails++;
        else std::printf("OK    printf de newlib se ejecuto completo sobre el core\n");

        std::printf("\n===============================================================\n");
        if (total_fails == 0) std::printf("  Todos los checks pasaron.\n");
        else                  std::printf("  %d check(s) fallaron.\n", total_fails);
        std::printf("===============================================================\n");
        sc_stop();
    }
};

int sc_main(int, char*[]) {
    Memory       memory("memory");
    Uart         uart("uart");
    Bus          bus("bus");
    ProcessorOOO cpu("cpu_ooo");
    VectorUnit   vu("vector_unit"); // no se ejercita, pero bus.vector_target
                                    // necesita un bind para la elaboracion
    Testbench    tb("testbench", cpu, memory);

    cpu.init_socket.bind(bus.cpu_target);
    vu.init_socket.bind(bus.vector_target);
    bus.mem_initiator.bind(memory.socket);
    bus.uart_initiator.bind(uart.socket);   // periferico UART en el Bus

    sc_start();
    return tb.total_fails == 0 ? 0 : 1;
}
