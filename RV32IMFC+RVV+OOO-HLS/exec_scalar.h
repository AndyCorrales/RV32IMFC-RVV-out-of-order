#ifndef EXEC_SCALAR_H
#define EXEC_SCALAR_H

// Unidades de ejecucion ESCALARES y la red de bypass: el CDB, la lectura
// de operandos (con renombrado), la ALU entera, mul/div y la FPU, mas la
// condicion de salto. Son funciones puras salvo cdb_broadcast, que es
// justamente la que despierta a las estaciones de reserva.
#include "rv32_ooo.h"
#include "rv32i_defs.h"
#include "ooo_state.h"
#include "immediates_hls.h"
#include "fp_ops.h"
#include <cmath>

using namespace rv32_hls;

// ================= helpers combinacionales =================

// Despierta a toda RS que esperara `tag` -- el rol del CDB. Se llama una
// vez por cada unidad que completa en el tick (bus de resultado por
// unidad, sin arbitraje de un CDB unico -- simplificacion documentada en
// rv32_ooo.h).
static void cdb_broadcast(ap_uint<3> tag, ap_uint<32> value) {
    for (int i = 0; i < N_ALU; i++) {
#pragma HLS UNROLL
        if (alu_rs[i].busy && !alu_rs[i].s1.ready && alu_rs[i].s1.tag == tag) {
            alu_rs[i].s1.ready = true; alu_rs[i].s1.val = value;
        }
        if (alu_rs[i].busy && !alu_rs[i].s2.ready && alu_rs[i].s2.tag == tag) {
            alu_rs[i].s2.ready = true; alu_rs[i].s2.val = value;
        }
    }
    if (md_rs.busy && !md_rs.s1.ready && md_rs.s1.tag == tag) {
        md_rs.s1.ready = true; md_rs.s1.val = value;
    }
    if (md_rs.busy && !md_rs.s2.ready && md_rs.s2.tag == tag) {
        md_rs.s2.ready = true; md_rs.s2.val = value;
    }
    if (lsu_rs.busy && !lsu_rs.s1.ready && lsu_rs.s1.tag == tag) {
        lsu_rs.s1.ready = true; lsu_rs.s1.val = value;
    }
    if (lsu_rs.busy && !lsu_rs.s2.ready && lsu_rs.s2.tag == tag) {
        lsu_rs.s2.ready = true; lsu_rs.s2.val = value;
    }
    if (br_rs.busy && !br_rs.s1.ready && br_rs.s1.tag == tag) {
        br_rs.s1.ready = true; br_rs.s1.val = value;
    }
    if (br_rs.busy && !br_rs.s2.ready && br_rs.s2.tag == tag) {
        br_rs.s2.ready = true; br_rs.s2.val = value;
    }
    if (fpu_rs.busy && !fpu_rs.s1.ready && fpu_rs.s1.tag == tag) {
        fpu_rs.s1.ready = true; fpu_rs.s1.val = value;
    }
    if (fpu_rs.busy && !fpu_rs.s2.ready && fpu_rs.s2.tag == tag) {
        fpu_rs.s2.ready = true; fpu_rs.s2.val = value;
    }
    if (fpu_rs.busy && !fpu_rs.s3.ready && fpu_rs.s3.tag == tag) {
        fpu_rs.s3.ready = true; fpu_rs.s3.val = value;
    }
    if (vec_rs.busy && !vec_rs.s1.ready && vec_rs.s1.tag == tag) {
        vec_rs.s1.ready = true; vec_rs.s1.val = value;
    }
    if (vec_rs.busy && !vec_rs.s2.ready && vec_rs.s2.tag == tag) {
        vec_rs.s2.ready = true; vec_rs.s2.val = value;
    }
    if (sys_rs.busy && !sys_rs.s1.ready && sys_rs.s1.tag == tag) {
        sys_rs.s1.ready = true; sys_rs.s1.val = value;
    }
}

// Lectura/escritura del banco de CSRs (subset minimo de modo maquina).
// misa/mhartid son de solo lectura (valores fijos); el resto es
static Operand read_operand(ap_uint<5> reg) {
    Operand op;
    if (reg == 0) {
        op.ready = true; op.val = 0; op.tag = 0;
        return op;
    }
    if (!rat[reg].has_tag) {
        op.ready = true; op.val = regfile[reg]; op.tag = 0;
        return op;
    }
    ap_uint<3> t = rat[reg].tag;
    if (rob[t].ready) {
        op.ready = true; op.val = rob[t].value; op.tag = 0;
    } else {
        op.ready = false; op.val = 0; op.tag = t;
    }
    return op;
}

// Version F de read_operand: usa FRAT/fregs. A diferencia de x0, f0 es
// un registro normal (se renombra y escribe como cualquier otro). Los
// tags son del MISMO ROB, asi que el wakeup por CDB es identico.
static Operand read_operand_fp(ap_uint<5> reg) {
    Operand op;
    if (!frat[reg].has_tag) {
        op.ready = true; op.val = fregs[reg]; op.tag = 0;
        return op;
    }
    ap_uint<3> t = frat[reg].tag;
    if (rob[t].ready) {
        op.ready = true; op.val = rob[t].value; op.tag = 0;
    } else {
        op.ready = false; op.val = 0; op.tag = t;
    }
    return op;
}

// ALU entera RV32I -- identica semantica al bloque OP/OP_IMM de
// rv32_core.cpp, factorizada como funcion pura (fase 1 del plan:
// separar decode de execute).
static ap_uint<32> alu_compute(ap_uint<3> f3, bool alt, ap_uint<32> a_u, ap_uint<32> b_u) {
    int32_t a = static_cast<int32_t>(a_u.to_uint());
    int32_t b = static_cast<int32_t>(b_u.to_uint());
    ap_uint<5> sh = b_u.range(4, 0);
    switch (f3.to_uint()) {
        case rv32i::Funct3_ALU::ADD_SUB: return alt ? ap_uint<32>(a - b) : ap_uint<32>(a + b);
        case rv32i::Funct3_ALU::SLL:     return a_u << sh;
        case rv32i::Funct3_ALU::SLT:     return (a < b) ? 1 : 0;
        case rv32i::Funct3_ALU::SLTU:    return (a_u < b_u) ? 1 : 0;
        case rv32i::Funct3_ALU::XOR:     return a_u ^ b_u;
        case rv32i::Funct3_ALU::SRL_SRA: return alt ? ap_uint<32>(a >> sh.to_uint()) : ap_uint<32>(a_u >> sh);
        case rv32i::Funct3_ALU::OR:      return a_u | b_u;
        case rv32i::Funct3_ALU::AND:     return a_u & b_u;
        default:                         return 0;
    }
}

// Extension M completa, con los mismos casos borde del ISA que
// rv32_core.cpp (division por cero, overflow INT32_MIN/-1).
static ap_uint<32> md_compute(ap_uint<3> f3, ap_uint<32> a_u, ap_uint<32> b_u) {
    uint32_t ua = a_u.to_uint(), ub = b_u.to_uint();
    int32_t  sa = static_cast<int32_t>(ua), sb = static_cast<int32_t>(ub);
    int64_t  a64 = sa, b64 = sb;
    uint64_t ua64 = ua, ub64 = ub;
    switch (f3.to_uint()) {
        case rv32i::Funct3_MULDIV::MUL:
            return ua * ub;
        case rv32i::Funct3_MULDIV::MULH:
            return static_cast<uint32_t>(static_cast<uint64_t>(a64 * b64) >> 32);
        case rv32i::Funct3_MULDIV::MULHSU:
            return static_cast<uint32_t>(static_cast<uint64_t>(a64 * static_cast<int64_t>(ub64)) >> 32);
        case rv32i::Funct3_MULDIV::MULHU:
            return static_cast<uint32_t>((ua64 * ub64) >> 32);
        case rv32i::Funct3_MULDIV::DIV:
            if (sb == 0)                          return 0xFFFFFFFF;
            if (ua == 0x80000000u && sb == -1)    return 0x80000000;
            return static_cast<uint32_t>(sa / sb);
        case rv32i::Funct3_MULDIV::DIVU:
            return (ub == 0) ? 0xFFFFFFFF : (ua / ub);
        case rv32i::Funct3_MULDIV::REM:
            if (sb == 0)                          return ua;
            if (ua == 0x80000000u && sb == -1)    return 0;
            return static_cast<uint32_t>(sa % sb);
        case rv32i::Funct3_MULDIV::REMU:
            return (ub == 0) ? ua : (ua % ub);
        default:
            return 0;
    }
}

// Extension F completa -- misma semantica que el bloque OP_FP/R4 de
// rv32_core.cpp (fp_ops.h para reinterpretaciones/saturacion/FCLASS),
// factorizada como funcion pura de la reservation station de la FPU.
// Los operandos entran/salen como bits IEEE-754 crudos.
static ap_uint<32> fpu_compute(const FpuRs& u) {
    float a = rv32i::bits_to_float(u.s1.val.to_uint());
    float b = rv32i::bits_to_float(u.s2.val.to_uint());
    float c = rv32i::bits_to_float(u.s3.val.to_uint());
    switch (u.r4op.to_uint()) {
        case 1: return rv32i::float_to_bits(std::fma(a, b, c));   // FMADD
        case 2: return rv32i::float_to_bits(std::fma(a, b, -c));  // FMSUB
        case 3: return rv32i::float_to_bits(std::fma(-a, b, c));  // FNMSUB
        case 4: return rv32i::float_to_bits(std::fma(-a, b, -c)); // FNMADD
    }
    switch (u.f7.to_uint()) {
        case rv32i::Funct7_FP::FADD_S:  return rv32i::float_to_bits(a + b);
        case rv32i::Funct7_FP::FSUB_S:  return rv32i::float_to_bits(a - b);
        case rv32i::Funct7_FP::FMUL_S:  return rv32i::float_to_bits(a * b);
        case rv32i::Funct7_FP::FDIV_S:  return rv32i::float_to_bits(a / b);
        case rv32i::Funct7_FP::FSQRT_S: return rv32i::float_to_bits(std::sqrt(a));
        case rv32i::Funct7_FP::FSGNJ_S: {
            uint32_t ab = u.s1.val.to_uint();
            uint32_t bb = u.s2.val.to_uint();
            uint32_t sign = 0;
            switch (u.f3.to_uint()) {
                case rv32i::Funct3_FSGNJ::FSGNJ:  sign = bb & 0x80000000u; break;
                case rv32i::Funct3_FSGNJ::FSGNJN: sign = (~bb) & 0x80000000u; break;
                case rv32i::Funct3_FSGNJ::FSGNJX: sign = (ab ^ bb) & 0x80000000u; break;
            }
            return (ab & 0x7FFFFFFFu) | sign;
        }
        case rv32i::Funct7_FP::FMINMAX_S:
            return rv32i::float_to_bits(
                (u.f3 == rv32i::Funct3_FMINMAX::FMIN) ? std::fmin(a, b) : std::fmax(a, b));
        case rv32i::Funct7_FP::FCMP_S:
            switch (u.f3.to_uint()) {
                case rv32i::Funct3_FCMP::FLE: return (a <= b) ? 1 : 0;
                case rv32i::Funct3_FCMP::FLT: return (a <  b) ? 1 : 0;
                case rv32i::Funct3_FCMP::FEQ: return (a == b) ? 1 : 0;
            }
            return 0;
        case rv32i::Funct7_FP::FCVT_W_S:
            return (u.rs2f == rv32i::Rs2_FCVT::WU)
                       ? rv32i::fcvt_wu_s(a)
                       : static_cast<uint32_t>(rv32i::fcvt_w_s(a));
        case rv32i::Funct7_FP::FCVT_S_W:
            // s1 viene del banco ENTERO: es un entero crudo, no bits float
            return (u.rs2f == rv32i::Rs2_FCVT::WU)
                       ? rv32i::float_to_bits(static_cast<float>(u.s1.val.to_uint()))
                       : rv32i::float_to_bits(static_cast<float>(static_cast<int32_t>(u.s1.val.to_uint())));
        case rv32i::Funct7_FP::FMV_X_W_FCLASS_S:
            return (u.f3 == rv32i::Funct3_FMV_FCLASS::FCLASS_S)
                       ? ap_uint<32>(rv32i::fclass_s(a))
                       : u.s1.val; // FMV.X.W: bits crudos, sin conversion
        case rv32i::Funct7_FP::FMV_W_X:
            return u.s1.val;       // FMV.W.X: bits crudos desde el banco entero
        default:
            return 0;
    }
}

// Aritmetica RVV vector-vector -- misma semantica que rv32_vector.cpp,
// pero limitada a los primeros `vl` elementos (el largo vectorial
// dinamico que fijo el ultimo vsetvli). Los elementos con indice >= vl
// son el "tail" y se dejan SIN TOCAR (politica tail-undisturbed, vta=0):
// la spec permite explicitamente que una implementacion simple ignore
// vta y use siempre undisturbed (nota de la seccion 3.4.3).
// ---- acceso a elementos con ancho SEW variable ----
// El registro vectorial son OOO_VEC_LANES palabras de 32 bits. El
// elemento `idx` de ancho `sew_b` bytes vive en el bit idx*sew_b*8; como
// 8/16/32 dividen exacto a 32, nunca cruza el limite de palabra.
static bool branch_taken(ap_uint<3> f3, ap_uint<32> a_u, ap_uint<32> b_u) {
    int32_t a = static_cast<int32_t>(a_u.to_uint());
    int32_t b = static_cast<int32_t>(b_u.to_uint());
    switch (f3.to_uint()) {
        case rv32i::Funct3_BRANCH::BEQ:  return a_u == b_u;
        case rv32i::Funct3_BRANCH::BNE:  return a_u != b_u;
        case rv32i::Funct3_BRANCH::BLT:  return a < b;
        case rv32i::Funct3_BRANCH::BGE:  return a >= b;
        case rv32i::Funct3_BRANCH::BLTU: return a_u < b_u;
        case rv32i::Funct3_BRANCH::BGEU: return a_u >= b_u;
        default:                         return false;
    }
}

// Acceso a memoria de datos direccionada por palabra -- misma tecnica
// (y misma limitacion de no cruzar palabra) que mem_load/mem_store de
// rv32_core.cpp, sobre el arreglo dmem local de esta pista.

#endif // EXEC_SCALAR_H
