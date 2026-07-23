#include "rv32_vector.h"
#include <cstdint>

// ---- opcodes/campos RVV verificados contra la especificacion oficial
// (RISC-V "V" Vector Extension v1.0) -- no son namespaces compartidos
// con rv32i_defs.h a proposito, este core no toca esos archivos. ----
namespace rvv {
    constexpr uint32_t OPCODE_LOAD_FP  = 0b0000111;
    constexpr uint32_t OPCODE_STORE_FP = 0b0100111;
    constexpr uint32_t OPCODE_OP_V     = 0b1010111;

    constexpr uint32_t WIDTH_32 = 0b110; // EEW=32 (seccion 7, tabla de 'width')

    constexpr uint32_t FUNCT3_OPIVV = 0b000;
    constexpr uint32_t FUNCT3_OPMVV = 0b010;

    // funct6 (seccion 19, tabla "Vector Instruction Listing")
    constexpr uint32_t FUNCT6_VADD = 0b000000;
    constexpr uint32_t FUNCT6_VSUB = 0b000010;
    constexpr uint32_t FUNCT6_VMUL = 0b100101;
}

// extractores de campos -- mismo patron shift+mascara que rv32i_defs.h,
// pero locales a este archivo (nombres RVV: vd/vs1/vs2/vs3 en vez de
// rd/rs1/rs2, aunque caen en las mismas posiciones de bits que R-type)
static inline uint32_t opcode(ap_uint<32> i) { return i.range(6, 0).to_uint(); }
static inline uint32_t vd(ap_uint<32> i)     { return i.range(11, 7).to_uint(); }
static inline uint32_t width(ap_uint<32> i)  { return i.range(14, 12).to_uint(); }
static inline uint32_t funct3(ap_uint<32> i) { return i.range(14, 12).to_uint(); }
static inline uint32_t vs1(ap_uint<32> i)    { return i.range(19, 15).to_uint(); }
static inline uint32_t vs2(ap_uint<32> i)    { return i.range(24, 20).to_uint(); }
static inline uint32_t lumop(ap_uint<32> i)  { return i.range(24, 20).to_uint(); }
static inline uint32_t mop(ap_uint<32> i)    { return i.range(27, 26).to_uint(); }
static inline uint32_t funct6(ap_uint<32> i) { return i.range(31, 26).to_uint(); }

void rv32_vector_step(
    ap_uint<32> instr,
    ap_uint<32> rs1_val,
    ap_uint<32> vregs_in[VEC_REGFILE_LEN],
    ap_uint<32> vregs_out[VEC_REGFILE_LEN],
    ap_uint<32>* mem,
    ap_uint<1>&  halted)
{
#pragma HLS INTERFACE ap_none port=instr
#pragma HLS INTERFACE ap_none port=rs1_val
#pragma HLS INTERFACE bram port=vregs_in
#pragma HLS INTERFACE bram port=vregs_out
#pragma HLS INTERFACE m_axi port=mem offset=slave bundle=gmem depth=VEC_MEM_WORDS
#pragma HLS INTERFACE ap_none port=halted
#pragma HLS INTERFACE s_axilite port=return bundle=control

    // por defecto, el banco de salida es una copia identica del de
    // entrada -- cada seccion abajo sobreescribe solo lo que le toca
    for (int i = 0; i < VEC_REGFILE_LEN; i++) {
        vregs_out[i] = vregs_in[i];
    }
    halted = 0;

    uint32_t opc = opcode(instr);

    // ---- Seccion 1: memoria vectorial (vle32.v / vse32.v) ----
    if (opc == rvv::OPCODE_LOAD_FP && width(instr) == rvv::WIDTH_32 &&
        mop(instr) == 0 && lumop(instr) == 0) {
        // vle32.v vd, (rs1): 4 palabras consecutivas de memoria -> 4 lanes
        uint32_t d = vd(instr);
        ap_uint<32> word_addr = rs1_val >> 2;
        for (int lane = 0; lane < VEC_LANES; lane++) {
#pragma HLS UNROLL
            vregs_out[d * VEC_LANES + lane] = mem[word_addr + lane];
        }
    } else if (opc == rvv::OPCODE_STORE_FP && width(instr) == rvv::WIDTH_32 &&
               mop(instr) == 0 && lumop(instr) == 0) {
        // vse32.v vs3, (rs1): 4 lanes -> 4 palabras consecutivas de memoria
        // (el campo en la posicion de 'vd' es vs3 para el store, mismo bit range)
        uint32_t s3 = vd(instr);
        ap_uint<32> word_addr = rs1_val >> 2;
        for (int lane = 0; lane < VEC_LANES; lane++) {
#pragma HLS UNROLL
            mem[word_addr + lane] = vregs_in[s3 * VEC_LANES + lane];
        }
    }
    // ---- Seccion 2: aritmetica vectorial vector-vector ----
    else if (opc == rvv::OPCODE_OP_V &&
             (funct3(instr) == rvv::FUNCT3_OPIVV || funct3(instr) == rvv::FUNCT3_OPMVV)) {
        uint32_t d = vd(instr), s1 = vs1(instr), s2 = vs2(instr), f6 = funct6(instr);
        bool is_add = (funct3(instr) == rvv::FUNCT3_OPIVV && f6 == rvv::FUNCT6_VADD);
        bool is_sub = (funct3(instr) == rvv::FUNCT3_OPIVV && f6 == rvv::FUNCT6_VSUB);
        bool is_mul = (funct3(instr) == rvv::FUNCT3_OPMVV && f6 == rvv::FUNCT6_VMUL);
        if (is_add || is_sub || is_mul) {
            for (int lane = 0; lane < VEC_LANES; lane++) {
#pragma HLS UNROLL
                ap_uint<32> a = vregs_in[s2 * VEC_LANES + lane]; // vs2 (primer operando, ver sintaxis "vadd.vv vd,vs2,vs1")
                ap_uint<32> b = vregs_in[s1 * VEC_LANES + lane]; // vs1 (segundo operando)
                ap_uint<32> r;
                if (is_add)      r = a + b;
                else if (is_sub) r = a - b;
                else             r = a * b;
                vregs_out[d * VEC_LANES + lane] = r;
            }
        }
    } else {
        // instruccion no reconocida en este alcance minimo (vsetvli,
        // strided/indexed, reduccion, punto flotante vectorial, etc.)
        halted = 1;
    }
}
