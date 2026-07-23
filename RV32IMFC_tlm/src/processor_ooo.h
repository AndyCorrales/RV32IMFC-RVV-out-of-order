#ifndef PROCESSOR_OOO_H
#define PROCESSOR_OOO_H

#include <systemc.h>
#include <tlm.h>
#include <tlm_utils/simple_initiator_socket.h>
#include <array>
#include <cstring>
#include <cmath>
#include <iostream>
#include <vector>

#include "rv32i_defs.h"
#include "immediates.h"
#include "fp_ops.h"
#include "rv32c_defs.h"

// CPU RV32IMFC con ejecucion FUERA DE ORDEN (Tomasulo-lite). Mismo rol de
// initiator TLM-2.0 que Processor (processor.h) -- mismo socket, mismo
// bus_access de 9 pasos, mismo fetch de 16 bits -- pero internamente el
// modelo es de microarquitectura por ciclos:
//
//   - RAT (enteros) + FRAT (flotantes) apuntando a un ROB unificado de 8
//     entradas con retiro en orden.
//   - Unidades funcionales con reservation station propia: 2x ALU (lat 1),
//     1x MUL/DIV (lat 3/8), 1x FPU (F completa, familia FMADD con rs3;
//     lat 3/8/4/2 segun op), 1x LSU, 1x branch, 1x VEC (coprocesamiento
//     vectorial RVV, ver abajo). LUI/AUIPC/JAL resuelven en el dispatch.
//   - CDB: cada unidad difunde (tag, valor) al completar y despierta a
//     las RS en espera. Los valores F viajan como bits IEEE-754 crudos.
//
// Coprocesamiento vectorial (RVV, unidad VecRs): mismas 5 instrucciones
// y misma codificacion de bits verificada contra la especificacion
// oficial RVV v1.0 que la pista HLS (rv32_ooo.cpp) -- vle32.v/vse32.v
// (memoria unit-stride) + vadd.vv/vsub.vv/vmul.vv (aritmetica
// vector-vector), MAS vsetvli/vsetivli/vsetvl con vl DINAMICO
// (vl = min(AVL,VLMAX)) y tail-undisturbed. Unica configuracion
// soportada: SEW=32/LMUL=1/VLEN=128 -> VLMAX=4 (otra activa vill). Sin
// mascara. Se despachan desde el MISMO decoder OOO que el resto del
// programa: un vadd.vv puede estar ejecutando mientras instrucciones
// escalares independientes completan fuera de orden alrededor suyo --
// eso es coprocesamiento con un core OOO, no dos decoders separados. El
// banco vectorial (vregs) no se renombra via el ROB: una sola RS VEC
// serializa las instrucciones vectoriales entre si (como la LSU). La
// distincion con FLW/FSW escalar es el campo width (010=escalar,
// 110=vectorial de 32b). vle32.v/vse32.v se resuelven en la cabeza del
// ROB, con acceso al Bus real (respetando la convencion TLM).
//
// Decisiones de alcance (mismas que la pista HLS, rv32_ooo.h):
//   - SIN especulacion de saltos: fetch detenido hasta resolver
//     BRANCH/JALR (JAL redirige en el dispatch). Sin squash/recovery.
//   - Memoria EN ORDEN: un acceso en vuelo; un load ejecuta solo en la
//     cabeza del ROB; un store escribe por el Bus recien al commit.
//   - Sin CSRs de F (rm=RNE fijo). RVV minimo (ver arriba), sin el resto
//     del ISA vectorial.
//
// Convenciones TLM del proyecto respetadas: sin sc_signal, todo acceso a
// memoria via b_transport bloqueante por el Bus (fetch de 16 bits, loads
// de 1/2/4 bytes -- aca la memoria es de bytes, asi que LB/LH/desalineados
// funcionan nativamente, sin el read-modify-write de la pista HLS). El
// "ciclo" es un tick del bucle de run(): el tiempo simulado avanza 1ns
// nominal por tick MAS las latencias reales que el Bus/Memory anotan en
// b_transport (estilo LT: el tiempo se cobra donde se genera).
//
// Para la evidencia de reordenamiento, el modulo registra en que ciclo
// completo cada instruccion (indexado por orden de dispatch) en
// complete_cycle[], y cuenta dispatches/commits -- el testbench
// (main_ooo.cpp) verifica con eso que una instruccion posterior completo
// antes que una anterior, sin acceso backdoor al estado interno.
SC_MODULE(ProcessorOOO) {
    tlm_utils::simple_initiator_socket<ProcessorOOO, 32> init_socket;

    static const int ROB_SZ = 8;
    static const int N_ALU  = 2;
    static const int ALU_LAT = 1, MUL_LAT = 3, DIV_LAT = 8, BR_LAT = 1;
    static const int FPU_LAT_ADDMUL = 3, FPU_LAT_DIV = 8, FPU_LAT_FMA = 4, FPU_LAT_MISC = 2;
    static const int VEC_LAT = 3; // vadd.vv/vsub.vv/vmul.vv

    // ---- coprocesamiento vectorial (RVV): VLEN=128/SEW=32/LMUL=1 -> 4 lanes,
    // mismos valores y misma codificacion de bits que la pista HLS
    // (rv32_ooo.cpp / rv32_vector.cpp), verificados contra la
    // especificacion oficial RVV v1.0 ----
    static const int VEC_NUM_VREGS = 32;
    static const int VEC_LANES     = 4;
    static const int VEC_REGFILE_LEN = VEC_NUM_VREGS * VEC_LANES;
    // opcodes/campos RVV (no viven en rv32i_defs.h: ese header es
    // compartido con Processor in-order, no hace falta tocarlo)
    static const uint32_t RVV_OPCODE_OP_V = 0b1010111;
    static const uint32_t RVV_WIDTH_32    = 0b110; // distingue vle32.v/vse32.v de FLW/FSW (width=010)
    static const uint32_t RVV_FUNCT3_OPIVV = 0b000;
    static const uint32_t RVV_FUNCT3_OPMVV = 0b010;
    static const uint32_t RVV_FUNCT6_VADD  = 0b000000;
    static const uint32_t RVV_FUNCT6_VSUB  = 0b000010;
    static const uint32_t RVV_FUNCT6_VMUL  = 0b100101;
    static const uint32_t RVV_FUNCT3_OPCFG = 0b111; // vsetvli / vsetivli / vsetvl
    // Campos de vtype (spec 3.4): vlmul[2:0], vsew[5:3], vta[6], vma[7],
    // vill[XLEN-1]. Solo se soporta SEW=32 / LMUL=1 -> VLMAX = 128/32 = 4.
    static const uint32_t RVV_VSEW_32  = 0b010;
    static const uint32_t RVV_VLMUL_1  = 0b000;
    static const uint32_t RVV_VILL_BIT = 31;
    static const uint32_t RVV_VLMAX    = VEC_LANES;

    // ---- estado arquitectonico ----
    std::array<uint32_t, 32> regs{};
    std::array<float, 32> fregs{};
    std::array<uint32_t, VEC_REGFILE_LEN> vregs{}; // banco vectorial (sin renombrar)
    // CSRs vectoriales: los escribe la familia vsetvl*, no csrw.
    uint32_t csr_vtype = (1u << RVV_VILL_BIT); // al reset: vill activo
    uint32_t csr_vl = 0;                       // ...y vl=0 (spec 3.11)
    bool halted = false;
    sc_event finished;

    // ---- observabilidad para el testbench ----
    bool trace = true;
    uint64_t cycle = 0;
    int n_disp = 0, n_commit = 0;
    std::array<int, 64> complete_cycle; // por indice de dispatch

    struct Operand { bool ready; uint32_t val; uint8_t tag; };
    struct RatEntry { bool has_tag; uint8_t tag; };
    struct RobEntry {
        bool valid, ready, is_store, dest_is_fp;
        uint8_t dest;
        uint32_t value;   // para F: bits IEEE-754 crudos
        uint32_t addr, sdata; // solo stores
        uint8_t mem_f3;
        int disp_idx;     // orden de dispatch (para complete_cycle[])
    };
    struct AluRs {
        bool busy, executing; uint8_t remaining;
        uint8_t f3; bool alt; uint8_t rob_tag;
        Operand s1, s2;
    };
    struct MdRs {
        bool busy, executing; uint8_t remaining;
        uint8_t f3; uint8_t rob_tag;
        Operand s1, s2;
    };
    struct FpuRs {
        bool busy, executing; uint8_t remaining;
        uint8_t r4op;  // 0=OP_FP; 1..4 = FMADD/FMSUB/FNMSUB/FNMADD
        uint8_t f7, f3, rs2f, rob_tag;
        Operand s1, s2, s3; // s3: tercer operando de la familia R4
    };
    struct LsuRs {
        bool busy, is_load;
        uint8_t f3, rob_tag;
        int32_t imm;
        Operand s1, s2; // s1: base (banco entero); s2: dato del store
    };
    struct BrRs {
        bool busy, executing; uint8_t remaining;
        bool is_jalr;
        uint8_t f3, rob_tag, size; // size: 2 o 4 (extension C)
        uint32_t br_pc;
        int32_t imm;
        Operand s1, s2;
    };
    // Unidad vectorial (RVV): una sola reservation station, como la LSU --
    // serializa las instrucciones vectoriales ENTRE SI (no hay VRAT que
    // las renombre), pero se solapan libremente con instrucciones
    // escalares independientes que completen fuera de orden alrededor
    // suyo. Eso es coprocesamiento con un core OOO, no "otro decoder".
    struct VecRs {
        bool busy, executing; uint8_t remaining;
        bool is_load, is_store, is_arith;
        uint8_t vd_or_vs3, vs1, vs2; // mismos bits que rd/rs1/rs2
        uint8_t arith_op;            // 0=vadd.vv, 1=vsub.vv, 2=vmul.vv
        uint8_t rob_tag;
        Operand s1;                  // direccion base (rs1, banco entero) -- solo memoria
        uint8_t vl;                  // largo vectorial capturado en el dispatch
    };

    // ---- estado de la microarquitectura ----
    std::array<RatEntry, 32> rat{};
    std::array<RatEntry, 32> frat{};
    std::array<RobEntry, ROB_SZ> rob{};
    uint8_t rob_head = 0, rob_tail = 0, rob_count = 0;
    std::array<AluRs, N_ALU> alu_rs{};
    MdRs  md_rs{};
    FpuRs fpu_rs{};
    LsuRs lsu_rs{};
    BrRs  br_rs{};
    VecRs vec_rs{};
    uint32_t fetch_pc = 0;
    bool fetch_stalled = false, fetch_done = false;

    SC_CTOR(ProcessorOOO) : init_socket("init_socket") {
        complete_cycle.fill(-1);
        SC_THREAD(run);
    }

    // ---- acceso al Bus: identico a Processor (processor.h) ----
    void bus_access(tlm::tlm_command cmd, uint32_t addr, uint8_t* data, unsigned int len) {
        tlm::tlm_generic_payload trans;
        sc_time delay = SC_ZERO_TIME;

        trans.set_command(cmd);
        trans.set_address(addr);
        trans.set_data_ptr(data);
        trans.set_data_length(len);
        trans.set_streaming_width(len);
        trans.set_byte_enable_ptr(nullptr);
        trans.set_dmi_allowed(false);
        trans.set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);

        init_socket->b_transport(trans, delay);
        wait(delay);

        if (trans.get_response_status() != tlm::TLM_OK_RESPONSE) {
            std::cerr << "[ProcessorOOO] Error de bus en direccion 0x" << std::hex << addr << std::dec << std::endl;
            halted = true;
        }
    }

    uint16_t fetch16(uint32_t addr) {
        uint16_t half = 0;
        bus_access(tlm::TLM_READ_COMMAND, addr, reinterpret_cast<uint8_t*>(&half), 2);
        return half;
    }

    uint32_t load(uint32_t addr, unsigned int len) {
        uint32_t value = 0;
        bus_access(tlm::TLM_READ_COMMAND, addr, reinterpret_cast<uint8_t*>(&value), len);
        return value;
    }

    void store(uint32_t addr, uint32_t value, unsigned int len) {
        bus_access(tlm::TLM_WRITE_COMMAND, addr, reinterpret_cast<uint8_t*>(&value), len);
    }

    // ---- helpers combinacionales (mismos que la pista HLS rv32_ooo.cpp) ----

    void cdb_broadcast(uint8_t tag, uint32_t value) {
        for (int i = 0; i < N_ALU; i++) {
            if (alu_rs[i].busy && !alu_rs[i].s1.ready && alu_rs[i].s1.tag == tag) {
                alu_rs[i].s1.ready = true; alu_rs[i].s1.val = value;
            }
            if (alu_rs[i].busy && !alu_rs[i].s2.ready && alu_rs[i].s2.tag == tag) {
                alu_rs[i].s2.ready = true; alu_rs[i].s2.val = value;
            }
        }
        if (md_rs.busy && !md_rs.s1.ready && md_rs.s1.tag == tag) { md_rs.s1.ready = true; md_rs.s1.val = value; }
        if (md_rs.busy && !md_rs.s2.ready && md_rs.s2.tag == tag) { md_rs.s2.ready = true; md_rs.s2.val = value; }
        if (fpu_rs.busy && !fpu_rs.s1.ready && fpu_rs.s1.tag == tag) { fpu_rs.s1.ready = true; fpu_rs.s1.val = value; }
        if (fpu_rs.busy && !fpu_rs.s2.ready && fpu_rs.s2.tag == tag) { fpu_rs.s2.ready = true; fpu_rs.s2.val = value; }
        if (fpu_rs.busy && !fpu_rs.s3.ready && fpu_rs.s3.tag == tag) { fpu_rs.s3.ready = true; fpu_rs.s3.val = value; }
        if (lsu_rs.busy && !lsu_rs.s1.ready && lsu_rs.s1.tag == tag) { lsu_rs.s1.ready = true; lsu_rs.s1.val = value; }
        if (lsu_rs.busy && !lsu_rs.s2.ready && lsu_rs.s2.tag == tag) { lsu_rs.s2.ready = true; lsu_rs.s2.val = value; }
        if (br_rs.busy && !br_rs.s1.ready && br_rs.s1.tag == tag) { br_rs.s1.ready = true; br_rs.s1.val = value; }
        if (br_rs.busy && !br_rs.s2.ready && br_rs.s2.tag == tag) { br_rs.s2.ready = true; br_rs.s2.val = value; }
        if (vec_rs.busy && !vec_rs.s1.ready && vec_rs.s1.tag == tag) { vec_rs.s1.ready = true; vec_rs.s1.val = value; }
    }

    // Aritmetica RVV vector-vector -- misma semantica que rv32_vector.cpp
    // (elemento a elemento sobre los VEC_LANES lanes).
    // Limitada a los primeros `vl` elementos; el resto es el "tail" y se
    // deja sin tocar (politica tail-undisturbed, que la spec permite usar
    // siempre en implementaciones simples).
    void vec_arith_compute(const VecRs& u) {
        for (int lane = 0; lane < VEC_LANES; lane++) {
            if (lane >= u.vl) continue;
            uint32_t a = vregs[u.vs2 * VEC_LANES + lane];
            uint32_t b = vregs[u.vs1 * VEC_LANES + lane];
            uint32_t r;
            if (u.arith_op == 0)      r = a + b; // vadd.vv
            else if (u.arith_op == 1) r = a - b; // vsub.vv
            else                      r = a * b; // vmul.vv
            vregs[u.vd_or_vs3 * VEC_LANES + lane] = r;
        }
    }

    Operand read_operand(uint8_t reg) {
        Operand op;
        if (reg == 0) { op.ready = true; op.val = 0; op.tag = 0; return op; }
        if (!rat[reg].has_tag) { op.ready = true; op.val = regs[reg]; op.tag = 0; return op; }
        uint8_t t = rat[reg].tag;
        if (rob[t].ready) { op.ready = true; op.val = rob[t].value; op.tag = 0; }
        else              { op.ready = false; op.val = 0; op.tag = t; }
        return op;
    }

    // f0 es un registro normal (se renombra y escribe como cualquier otro)
    Operand read_operand_fp(uint8_t reg) {
        Operand op;
        if (!frat[reg].has_tag) {
            op.ready = true; op.val = rv32i::float_to_bits(fregs[reg]); op.tag = 0;
            return op;
        }
        uint8_t t = frat[reg].tag;
        if (rob[t].ready) { op.ready = true; op.val = rob[t].value; op.tag = 0; }
        else              { op.ready = false; op.val = 0; op.tag = t; }
        return op;
    }

    static uint32_t alu_compute(uint8_t f3, bool alt, uint32_t a_u, uint32_t b_u) {
        int32_t a = static_cast<int32_t>(a_u);
        int32_t b = static_cast<int32_t>(b_u);
        uint32_t sh = b_u & 0x1F;
        switch (f3) {
            case rv32i::Funct3_ALU::ADD_SUB: return alt ? static_cast<uint32_t>(a - b) : static_cast<uint32_t>(a + b);
            case rv32i::Funct3_ALU::SLL:     return a_u << sh;
            case rv32i::Funct3_ALU::SLT:     return (a < b) ? 1 : 0;
            case rv32i::Funct3_ALU::SLTU:    return (a_u < b_u) ? 1 : 0;
            case rv32i::Funct3_ALU::XOR:     return a_u ^ b_u;
            case rv32i::Funct3_ALU::SRL_SRA: return alt ? static_cast<uint32_t>(a >> sh) : (a_u >> sh);
            case rv32i::Funct3_ALU::OR:      return a_u | b_u;
            case rv32i::Funct3_ALU::AND:     return a_u & b_u;
            default:                         return 0;
        }
    }

    static uint32_t md_compute(uint8_t f3, uint32_t ua, uint32_t ub) {
        int32_t  sa = static_cast<int32_t>(ua), sb = static_cast<int32_t>(ub);
        int64_t  a64 = sa, b64 = sb;
        uint64_t ua64 = ua, ub64 = ub;
        switch (f3) {
            case rv32i::Funct3_MULDIV::MUL:    return ua * ub;
            case rv32i::Funct3_MULDIV::MULH:   return static_cast<uint32_t>(static_cast<uint64_t>(a64 * b64) >> 32);
            case rv32i::Funct3_MULDIV::MULHSU: return static_cast<uint32_t>(static_cast<uint64_t>(a64 * static_cast<int64_t>(ub64)) >> 32);
            case rv32i::Funct3_MULDIV::MULHU:  return static_cast<uint32_t>((ua64 * ub64) >> 32);
            case rv32i::Funct3_MULDIV::DIV:
                if (sb == 0)                       return 0xFFFFFFFF;
                if (ua == 0x80000000u && sb == -1) return 0x80000000;
                return static_cast<uint32_t>(sa / sb);
            case rv32i::Funct3_MULDIV::DIVU:   return (ub == 0) ? 0xFFFFFFFF : (ua / ub);
            case rv32i::Funct3_MULDIV::REM:
                if (sb == 0)                       return ua;
                if (ua == 0x80000000u && sb == -1) return 0;
                return static_cast<uint32_t>(sa % sb);
            case rv32i::Funct3_MULDIV::REMU:   return (ub == 0) ? ua : (ua % ub);
            default:                           return 0;
        }
    }

    static uint32_t fpu_compute(const FpuRs& u) {
        float a = rv32i::bits_to_float(u.s1.val);
        float b = rv32i::bits_to_float(u.s2.val);
        float c = rv32i::bits_to_float(u.s3.val);
        switch (u.r4op) {
            case 1: return rv32i::float_to_bits(std::fma(a, b, c));
            case 2: return rv32i::float_to_bits(std::fma(a, b, -c));
            case 3: return rv32i::float_to_bits(std::fma(-a, b, c));
            case 4: return rv32i::float_to_bits(std::fma(-a, b, -c));
        }
        switch (u.f7) {
            case rv32i::Funct7_FP::FADD_S:  return rv32i::float_to_bits(a + b);
            case rv32i::Funct7_FP::FSUB_S:  return rv32i::float_to_bits(a - b);
            case rv32i::Funct7_FP::FMUL_S:  return rv32i::float_to_bits(a * b);
            case rv32i::Funct7_FP::FDIV_S:  return rv32i::float_to_bits(a / b);
            case rv32i::Funct7_FP::FSQRT_S: return rv32i::float_to_bits(std::sqrt(a));
            case rv32i::Funct7_FP::FSGNJ_S: {
                uint32_t ab = u.s1.val, bb = u.s2.val, sign = 0;
                switch (u.f3) {
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
                switch (u.f3) {
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
                // s1 viene del banco ENTERO: entero crudo, no bits float
                return (u.rs2f == rv32i::Rs2_FCVT::WU)
                           ? rv32i::float_to_bits(static_cast<float>(u.s1.val))
                           : rv32i::float_to_bits(static_cast<float>(static_cast<int32_t>(u.s1.val)));
            case rv32i::Funct7_FP::FMV_X_W_FCLASS_S:
                return (u.f3 == rv32i::Funct3_FMV_FCLASS::FCLASS_S)
                           ? rv32i::fclass_s(a)
                           : u.s1.val;
            case rv32i::Funct7_FP::FMV_W_X:
                return u.s1.val;
            default:
                return 0;
        }
    }

    static bool branch_taken(uint8_t f3, uint32_t a_u, uint32_t b_u) {
        int32_t a = static_cast<int32_t>(a_u), b = static_cast<int32_t>(b_u);
        switch (f3) {
            case rv32i::Funct3_BRANCH::BEQ:  return a_u == b_u;
            case rv32i::Funct3_BRANCH::BNE:  return a_u != b_u;
            case rv32i::Funct3_BRANCH::BLT:  return a < b;
            case rv32i::Funct3_BRANCH::BGE:  return a >= b;
            case rv32i::Funct3_BRANCH::BLTU: return a_u < b_u;
            case rv32i::Funct3_BRANCH::BGEU: return a_u >= b_u;
            default:                         return false;
        }
    }

    // Load con extension de signo/cero segun f3 -- la memoria TLM es de
    // bytes, asi que LB/LH/desalineados funcionan nativamente (sin el
    // read-modify-write de la pista HLS).
    uint32_t lsu_load_value(uint32_t addr, uint8_t f3) {
        switch (f3) {
            case rv32i::Funct3_LOAD::LB:  return static_cast<uint32_t>(static_cast<int32_t>(static_cast<int8_t>(load(addr, 1))));
            case rv32i::Funct3_LOAD::LH:  return static_cast<uint32_t>(static_cast<int32_t>(static_cast<int16_t>(load(addr, 2))));
            case rv32i::Funct3_LOAD::LW:  return load(addr, 4);
            case rv32i::Funct3_LOAD::LBU: return load(addr, 1) & 0xFF;
            case rv32i::Funct3_LOAD::LHU: return load(addr, 2) & 0xFFFF;
            default:                      return 0;
        }
    }

    static unsigned store_len(uint8_t f3) {
        switch (f3) {
            case rv32i::Funct3_STORE::SB: return 1;
            case rv32i::Funct3_STORE::SH: return 2;
            default:                      return 4; // SW / FSW
        }
    }

    // ---- un tick = un ciclo (mismas 4 etapas que la pista HLS) ----
    void tick() {
        // Etapa 1: commit (retiro en orden desde la cabeza del ROB)
        if (rob_count > 0 && rob[rob_head].valid && rob[rob_head].ready) {
            RobEntry& h = rob[rob_head];
            if (h.is_store) {
                store(h.addr, h.sdata, store_len(h.mem_f3));
            } else if (h.dest_is_fp) {
                fregs[h.dest] = rv32i::bits_to_float(h.value);
                if (frat[h.dest].has_tag && frat[h.dest].tag == rob_head)
                    frat[h.dest].has_tag = false;
            } else if (h.dest != 0) {
                regs[h.dest] = h.value;
            }
            if (!h.dest_is_fp && h.dest != 0 &&
                rat[h.dest].has_tag && rat[h.dest].tag == rob_head)
                rat[h.dest].has_tag = false;
            if (trace) {
                if (h.is_store)
                    std::cout << "[" << cycle << "] COMMIT store" << std::endl;
                else if (h.dest_is_fp)
                    std::cout << "[" << cycle << "] COMMIT f" << +h.dest
                              << " = 0x" << std::hex << h.value << std::dec << std::endl;
                else
                    std::cout << "[" << cycle << "] COMMIT x" << +h.dest
                              << " = 0x" << std::hex << h.value << std::dec << std::endl;
            }
            n_commit++;
            h.valid = false;
            rob_head = (rob_head + 1) & (ROB_SZ - 1);
            rob_count--;
        }

        // Etapa 2: ejecucion + broadcast (CDB) por unidad
        for (int i = 0; i < N_ALU; i++) {
            if (alu_rs[i].busy && alu_rs[i].executing) {
                if (alu_rs[i].remaining > 0) alu_rs[i].remaining--;
                if (alu_rs[i].remaining == 0) {
                    uint32_t res = alu_compute(alu_rs[i].f3, alu_rs[i].alt,
                                               alu_rs[i].s1.val, alu_rs[i].s2.val);
                    complete_entry(alu_rs[i].rob_tag, res, "ALU");
                    alu_rs[i].busy = false; alu_rs[i].executing = false;
                }
            }
        }
        if (md_rs.busy && md_rs.executing) {
            if (md_rs.remaining > 0) md_rs.remaining--;
            if (md_rs.remaining == 0) {
                complete_entry(md_rs.rob_tag, md_compute(md_rs.f3, md_rs.s1.val, md_rs.s2.val), "MULDIV");
                md_rs.busy = false; md_rs.executing = false;
            }
        }
        if (fpu_rs.busy && fpu_rs.executing) {
            if (fpu_rs.remaining > 0) fpu_rs.remaining--;
            if (fpu_rs.remaining == 0) {
                complete_entry(fpu_rs.rob_tag, fpu_compute(fpu_rs), "FPU");
                fpu_rs.busy = false; fpu_rs.executing = false;
            }
        }
        if (br_rs.busy && br_rs.executing) {
            if (br_rs.remaining > 0) br_rs.remaining--;
            if (br_rs.remaining == 0) {
                uint32_t link = br_rs.br_pc + br_rs.size;
                uint32_t value = 0;
                if (br_rs.is_jalr) {
                    fetch_pc = (br_rs.s1.val + static_cast<uint32_t>(br_rs.imm)) & ~0x1u;
                    value = link;
                } else {
                    bool taken = branch_taken(br_rs.f3, br_rs.s1.val, br_rs.s2.val);
                    fetch_pc = taken ? (br_rs.br_pc + static_cast<uint32_t>(br_rs.imm)) : link;
                }
                complete_entry(br_rs.rob_tag, value, "BR");
                fetch_stalled = false; // fetch se reanuda en el destino correcto
                br_rs.busy = false; br_rs.executing = false;
            }
        }
        // LSU: un store "ejecuta" (direccion + dato) en cuanto puede -- la
        // escritura real por el Bus espera al commit. Un load solo ejecuta
        // en la cabeza del ROB (todo store anterior ya escribio memoria).
        if (lsu_rs.busy && lsu_rs.s1.ready && lsu_rs.s2.ready) {
            uint32_t addr = lsu_rs.s1.val + static_cast<uint32_t>(lsu_rs.imm);
            if (!lsu_rs.is_load) {
                RobEntry& e = rob[lsu_rs.rob_tag];
                e.addr = addr; e.sdata = lsu_rs.s2.val; e.mem_f3 = lsu_rs.f3;
                e.ready = true;
                record_complete(lsu_rs.rob_tag, "LSU");
                lsu_rs.busy = false;
            } else if (lsu_rs.rob_tag == rob_head && rob[rob_head].valid) {
                complete_entry(lsu_rs.rob_tag, lsu_load_value(addr, lsu_rs.f3), "LSU");
                lsu_rs.busy = false;
            }
        }
        // VEC: aritmetica con latencia fija; memoria vectorial resuelta
        // SOLO en la cabeza del ROB, para load Y store (mas conservador
        // que el store escalar, para no ampliar el ROB a 128 bits).
        if (vec_rs.busy && vec_rs.is_arith && vec_rs.executing) {
            if (vec_rs.remaining > 0) vec_rs.remaining--;
            if (vec_rs.remaining == 0) {
                vec_arith_compute(vec_rs);
                rob[vec_rs.rob_tag].ready = true;
                record_complete(vec_rs.rob_tag, "VEC");
                vec_rs.busy = false; vec_rs.executing = false;
            }
        }
        if (vec_rs.busy && (vec_rs.is_load || vec_rs.is_store) &&
            vec_rs.s1.ready && vec_rs.rob_tag == rob_head && rob[rob_head].valid) {
            uint32_t base = vec_rs.s1.val;
            // solo los primeros `vl` elementos (tail sin tocar; el store
            // tampoco escribe memoria mas alla de vl)
            if (vec_rs.is_load) {
                for (int lane = 0; lane < vec_rs.vl; lane++)
                    vregs[vec_rs.vd_or_vs3 * VEC_LANES + lane] = load(base + lane * 4, 4);
            } else {
                for (int lane = 0; lane < vec_rs.vl; lane++)
                    store(base + lane * 4, vregs[vec_rs.vd_or_vs3 * VEC_LANES + lane], 4);
            }
            rob[vec_rs.rob_tag].ready = true;
            record_complete(vec_rs.rob_tag, "VEC");
            vec_rs.busy = false;
        }

        // Etapa 3: issue (RS con operandos listos arranca ejecucion)
        for (int i = 0; i < N_ALU; i++) {
            if (alu_rs[i].busy && !alu_rs[i].executing && alu_rs[i].s1.ready && alu_rs[i].s2.ready) {
                alu_rs[i].executing = true;
                alu_rs[i].remaining = ALU_LAT;
            }
        }
        if (md_rs.busy && !md_rs.executing && md_rs.s1.ready && md_rs.s2.ready) {
            md_rs.executing = true;
            md_rs.remaining = (md_rs.f3 >= rv32i::Funct3_MULDIV::DIV) ? DIV_LAT : MUL_LAT;
        }
        if (fpu_rs.busy && !fpu_rs.executing &&
            fpu_rs.s1.ready && fpu_rs.s2.ready && fpu_rs.s3.ready) {
            fpu_rs.executing = true;
            if (fpu_rs.r4op != 0)                                    fpu_rs.remaining = FPU_LAT_FMA;
            else if (fpu_rs.f7 == rv32i::Funct7_FP::FDIV_S ||
                     fpu_rs.f7 == rv32i::Funct7_FP::FSQRT_S)         fpu_rs.remaining = FPU_LAT_DIV;
            else if (fpu_rs.f7 == rv32i::Funct7_FP::FADD_S ||
                     fpu_rs.f7 == rv32i::Funct7_FP::FSUB_S ||
                     fpu_rs.f7 == rv32i::Funct7_FP::FMUL_S)          fpu_rs.remaining = FPU_LAT_ADDMUL;
            else                                                     fpu_rs.remaining = FPU_LAT_MISC;
        }
        if (br_rs.busy && !br_rs.executing && br_rs.s1.ready && br_rs.s2.ready) {
            br_rs.executing = true;
            br_rs.remaining = BR_LAT;
        }
        if (vec_rs.busy && vec_rs.is_arith && !vec_rs.executing) {
            // sin operandos que esperar (lee vregs directo, sin RAT):
            // arranca en el primer ciclo de issue, igual que las demas
            vec_rs.executing = true;
            vec_rs.remaining = VEC_LAT;
        }

        // Etapa 4: fetch + dispatch (1 instruccion por ciclo, en orden)
        if (!fetch_done && !fetch_stalled && rob_count < ROB_SZ) {
            uint16_t half_lo = fetch16(fetch_pc);
            if (halted) return; // error de bus en el fetch
            if (half_lo == 0) {
                fetch_done = true; // convencion de fin de programa
            } else {
                uint32_t instr;
                uint8_t isize;
                if ((half_lo & 0x3) != 0x3) {
                    instr = rv32c::expand(half_lo);
                    isize = 2;
                } else {
                    uint16_t half_hi = fetch16(fetch_pc + 2);
                    instr = (static_cast<uint32_t>(half_hi) << 16) | half_lo;
                    isize = 4;
                }
                dispatch(instr, isize);
            }
        }

        if (fetch_done && rob_count == 0) halted = true;
    }

    // marca la entrada lista, difunde por el CDB y registra el ciclo
    void complete_entry(uint8_t tag, uint32_t value, const char* unit) {
        rob[tag].value = value;
        rob[tag].ready = true;
        cdb_broadcast(tag, value);
        record_complete(tag, unit);
    }

    void record_complete(uint8_t tag, const char* unit) {
        int d = rob[tag].disp_idx;
        if (d >= 0 && d < static_cast<int>(complete_cycle.size()))
            complete_cycle[d] = static_cast<int>(cycle);
        if (trace)
            std::cout << "[" << cycle << "] " << unit << " completa d" << d
                      << " (tag " << +tag << ")" << std::endl;
    }

    void dispatch(uint32_t instr, uint8_t isize) {
        uint32_t opc = rv32i::opcode(instr);
        uint8_t rd   = rv32i::rd(instr);
        uint8_t f3   = rv32i::funct3(instr);
        uint8_t rs1i = rv32i::rs1(instr);
        uint8_t rs2i = rv32i::rs2(instr);
        uint32_t f7  = rv32i::funct7(instr);

        bool can_dispatch = false;
        uint8_t new_tag = rob_tail;
        rob[new_tag].dest_is_fp = false; // default entero; los casos F lo activan
        uint32_t this_pc = fetch_pc;
        uint32_t next_fetch = this_pc + isize;

        if (opc == rv32i::Opcode::LUI || opc == rv32i::Opcode::AUIPC ||
            opc == rv32i::Opcode::JAL) {
            can_dispatch = true;
            RobEntry& e = rob[new_tag];
            e.valid = true; e.is_store = false; e.dest = rd; e.ready = true;
            if (opc == rv32i::Opcode::LUI)
                e.value = static_cast<uint32_t>(rv32i::get_imm_U(instr));
            else if (opc == rv32i::Opcode::AUIPC)
                e.value = this_pc + static_cast<uint32_t>(rv32i::get_imm_U(instr));
            else { // JAL (un C.JAL expandido enlaza a pc+2 via isize)
                e.value = this_pc + isize;
                next_fetch = this_pc + static_cast<uint32_t>(rv32i::get_imm_J(instr));
            }
        } else if (opc == rv32i::Opcode::OP && f7 == rv32i::Funct7::MULDIV) {
            if (!md_rs.busy) {
                can_dispatch = true;
                md_rs.busy = true; md_rs.executing = false;
                md_rs.f3 = f3; md_rs.rob_tag = new_tag;
                md_rs.s1 = read_operand(rs1i);
                md_rs.s2 = read_operand(rs2i);
                RobEntry& e = rob[new_tag];
                e.valid = true; e.is_store = false; e.dest = rd; e.ready = false;
            }
        } else if (opc == rv32i::Opcode::OP || opc == rv32i::Opcode::OP_IMM) {
            int free_alu = -1;
            for (int i = N_ALU - 1; i >= 0; i--)
                if (!alu_rs[i].busy) free_alu = i;
            if (free_alu >= 0) {
                can_dispatch = true;
                AluRs& rs = alu_rs[free_alu];
                rs.busy = true; rs.executing = false;
                rs.f3 = f3; rs.rob_tag = new_tag;
                rs.s1 = read_operand(rs1i);
                if (opc == rv32i::Opcode::OP) {
                    rs.alt = (f7 == rv32i::Funct7::ALT);
                    rs.s2 = read_operand(rs2i);
                } else {
                    rs.alt = (f3 == rv32i::Funct3_ALU::SRL_SRA) && ((instr >> 30) & 1);
                    rs.s2.ready = true; rs.s2.tag = 0;
                    if (f3 == rv32i::Funct3_ALU::SLL || f3 == rv32i::Funct3_ALU::SRL_SRA)
                        rs.s2.val = rv32i::get_shamt(instr);
                    else
                        rs.s2.val = static_cast<uint32_t>(rv32i::get_imm_I(instr));
                }
                RobEntry& e = rob[new_tag];
                e.valid = true; e.is_store = false; e.dest = rd; e.ready = false;
            }
        } else if ((opc == rv32i::Opcode::LOAD_FP || opc == rv32i::Opcode::STORE_FP) &&
                   f3 == RVV_WIDTH_32) {
            // vle32.v / vse32.v: MISMO opcode LOAD-FP/STORE-FP que FLW/FSW
            // escalar, pero width=110 (vectorial 32b) en vez de 010 --
            // asi se distinguen (codificacion verificada contra RVV v1.0).
            if (!vec_rs.busy) {
                can_dispatch = true;
                vec_rs.busy = true;
                vec_rs.is_load = (opc == rv32i::Opcode::LOAD_FP);
                vec_rs.is_store = !vec_rs.is_load;
                vec_rs.is_arith = false;
                vec_rs.vd_or_vs3 = rd; // mismo campo de bits que vd (load) o vs3 (store)
                vec_rs.rob_tag = new_tag;
                vec_rs.vl = csr_vl;             // largo vectorial vigente al despachar
                vec_rs.s1 = read_operand(rs1i); // direccion base, banco entero
                RobEntry& e = rob[new_tag];
                e.valid = true; e.is_store = false; e.dest_is_fp = false;
                e.dest = 0; e.ready = false;
            }
        } else if (opc == rv32i::Opcode::LOAD || opc == rv32i::Opcode::STORE ||
                   opc == rv32i::Opcode::LOAD_FP || opc == rv32i::Opcode::STORE_FP) {
            if (!lsu_rs.busy) {
                bool is_fp   = (opc == rv32i::Opcode::LOAD_FP || opc == rv32i::Opcode::STORE_FP);
                bool is_load = (opc == rv32i::Opcode::LOAD || opc == rv32i::Opcode::LOAD_FP);
                can_dispatch = true;
                lsu_rs.busy = true;
                lsu_rs.is_load = is_load;
                lsu_rs.f3 = f3; lsu_rs.rob_tag = new_tag;
                lsu_rs.s1 = read_operand(rs1i); // base SIEMPRE del banco entero
                if (is_load) {
                    lsu_rs.imm = rv32i::get_imm_I(instr);
                    lsu_rs.s2.ready = true; lsu_rs.s2.val = 0; lsu_rs.s2.tag = 0;
                } else {
                    lsu_rs.imm = rv32i::get_imm_S(instr);
                    lsu_rs.s2 = is_fp ? read_operand_fp(rs2i) : read_operand(rs2i);
                }
                RobEntry& e = rob[new_tag];
                e.valid = true; e.ready = false;
                e.is_store = !is_load;
                e.dest_is_fp = is_fp && is_load;
                e.dest = is_load ? rd : 0;
            }
        } else if (opc == rv32i::Opcode::FMADD || opc == rv32i::Opcode::FMSUB ||
                   opc == rv32i::Opcode::FNMSUB || opc == rv32i::Opcode::FNMADD) {
            if (!fpu_rs.busy) {
                can_dispatch = true;
                fpu_rs.busy = true; fpu_rs.executing = false;
                fpu_rs.r4op = (opc == rv32i::Opcode::FMADD)  ? 1 :
                              (opc == rv32i::Opcode::FMSUB)  ? 2 :
                              (opc == rv32i::Opcode::FNMSUB) ? 3 : 4;
                fpu_rs.f7 = 0; fpu_rs.f3 = f3; fpu_rs.rs2f = rs2i;
                fpu_rs.rob_tag = new_tag;
                fpu_rs.s1 = read_operand_fp(rs1i);
                fpu_rs.s2 = read_operand_fp(rs2i);
                fpu_rs.s3 = read_operand_fp(rv32i::rs3(instr));
                RobEntry& e = rob[new_tag];
                e.valid = true; e.is_store = false; e.ready = false;
                e.dest_is_fp = true; e.dest = rd;
            }
        } else if (opc == rv32i::Opcode::OP_FP) {
            if (!fpu_rs.busy) {
                can_dispatch = true;
                bool int_dest = (f7 == rv32i::Funct7_FP::FCMP_S ||
                                 f7 == rv32i::Funct7_FP::FCVT_W_S ||
                                 f7 == rv32i::Funct7_FP::FMV_X_W_FCLASS_S);
                bool int_src  = (f7 == rv32i::Funct7_FP::FCVT_S_W ||
                                 f7 == rv32i::Funct7_FP::FMV_W_X);
                bool needs_s2 = (f7 == rv32i::Funct7_FP::FADD_S ||
                                 f7 == rv32i::Funct7_FP::FSUB_S ||
                                 f7 == rv32i::Funct7_FP::FMUL_S ||
                                 f7 == rv32i::Funct7_FP::FDIV_S ||
                                 f7 == rv32i::Funct7_FP::FSGNJ_S ||
                                 f7 == rv32i::Funct7_FP::FMINMAX_S ||
                                 f7 == rv32i::Funct7_FP::FCMP_S);
                fpu_rs.busy = true; fpu_rs.executing = false;
                fpu_rs.r4op = 0; fpu_rs.f7 = f7; fpu_rs.f3 = f3; fpu_rs.rs2f = rs2i;
                fpu_rs.rob_tag = new_tag;
                fpu_rs.s1 = int_src ? read_operand(rs1i) : read_operand_fp(rs1i);
                if (needs_s2) {
                    fpu_rs.s2 = read_operand_fp(rs2i);
                } else {
                    fpu_rs.s2.ready = true; fpu_rs.s2.val = 0; fpu_rs.s2.tag = 0;
                }
                fpu_rs.s3.ready = true; fpu_rs.s3.val = 0; fpu_rs.s3.tag = 0;
                RobEntry& e = rob[new_tag];
                e.valid = true; e.is_store = false; e.ready = false;
                e.dest_is_fp = !int_dest; e.dest = rd;
            }
        } else if (opc == rv32i::Opcode::BRANCH || opc == rv32i::Opcode::JALR) {
            if (!br_rs.busy) {
                can_dispatch = true;
                br_rs.busy = true; br_rs.executing = false;
                br_rs.is_jalr = (opc == rv32i::Opcode::JALR);
                br_rs.f3 = f3; br_rs.rob_tag = new_tag;
                br_rs.br_pc = this_pc;
                br_rs.size = isize;
                br_rs.s1 = read_operand(rs1i);
                if (br_rs.is_jalr) {
                    br_rs.imm = rv32i::get_imm_I(instr);
                    br_rs.s2.ready = true; br_rs.s2.val = 0; br_rs.s2.tag = 0;
                } else {
                    br_rs.imm = rv32i::get_imm_B(instr);
                    br_rs.s2 = read_operand(rs2i);
                }
                RobEntry& e = rob[new_tag];
                e.valid = true; e.is_store = false; e.ready = false;
                e.dest = br_rs.is_jalr ? rd : 0;
                // sin especulacion: fetch detenido hasta resolver el destino
                fetch_stalled = true;
            }
        } else if (opc == RVV_OPCODE_OP_V && f3 == RVV_FUNCT3_OPCFG) {
            // ---- vsetvli / vsetivli / vsetvl (seccion 5 de la spec) ----
            // Se resuelve EN EL DISPATCH (como LUI/JAL): el dispatch es en
            // orden y este core no especula, asi que actualizar vtype/vl
            // aca es preciso y hace que toda vectorial posterior capture el
            // vl nuevo sin serializar la unidad VEC. Requiere el AVL ya
            // disponible: si el productor sigue en vuelo, se detiene el
            // dispatch y se reintenta el ciclo siguiente.
            bool is_vsetivli = (((instr >> 30) & 0x3) == 0x3);
            bool is_vsetvl   = (((instr >> 25) & 0x7F) == 0b1000000);
            Operand avl_op  = read_operand(rs1i);
            Operand vtyp_op = read_operand(rs2i);
            bool operands_ready = is_vsetivli
                                ? true
                                : (avl_op.ready && (!is_vsetvl || vtyp_op.ready));
            if (operands_ready) {
                can_dispatch = true;
                uint32_t vtype_req, avl;
                bool avl_is_max = false, keep_vl = false;
                if (is_vsetivli) {
                    vtype_req = (instr >> 20) & 0x3FF;  // zimm[9:0]
                    avl = rs1i;                          // uimm[4:0]
                } else if (is_vsetvl) {
                    vtype_req = vtyp_op.val;
                    if (rs1i == 0) { if (rd == 0) keep_vl = true; else avl_is_max = true; }
                    avl = avl_op.val;
                } else { // vsetvli
                    vtype_req = (instr >> 20) & 0x7FF;  // zimm[10:0]
                    if (rs1i == 0) { if (rd == 0) keep_vl = true; else avl_is_max = true; }
                    avl = avl_op.val;
                }
                uint32_t vsew  = (vtype_req >> 3) & 0x7;
                uint32_t vlmul = vtype_req & 0x7;
                bool supported = (vsew == RVV_VSEW_32) && (vlmul == RVV_VLMUL_1);
                uint32_t new_vl;
                if (!supported) {                 // configuracion no soportada
                    csr_vtype = (1u << RVV_VILL_BIT);
                    new_vl = 0;
                } else {
                    csr_vtype = vtype_req;
                    if (keep_vl)         new_vl = csr_vl;
                    else if (avl_is_max) new_vl = RVV_VLMAX;
                    else                 new_vl = (avl < RVV_VLMAX) ? avl : RVV_VLMAX;
                }
                csr_vl = new_vl;
                RobEntry& e = rob[new_tag];
                e.valid = true; e.is_store = false; e.dest_is_fp = false;
                e.dest = rd; e.value = new_vl; e.ready = true;
            }
        } else if (opc == RVV_OPCODE_OP_V) {
            // vadd.vv/vsub.vv/vmul.vv -- funct6/funct3 verificados contra
            // la seccion 19 de la especificacion oficial RVV v1.0.
            uint32_t f6 = (instr >> 26) & 0x3F;
            bool is_add = (f3 == RVV_FUNCT3_OPIVV && f6 == RVV_FUNCT6_VADD);
            bool is_sub = (f3 == RVV_FUNCT3_OPIVV && f6 == RVV_FUNCT6_VSUB);
            bool is_mul = (f3 == RVV_FUNCT3_OPMVV && f6 == RVV_FUNCT6_VMUL);
            if (is_add || is_sub || is_mul) {
                if (!vec_rs.busy) {
                    can_dispatch = true;
                    vec_rs.busy = true;
                    vec_rs.is_load = false; vec_rs.is_store = false; vec_rs.is_arith = true;
                    vec_rs.vd_or_vs3 = rd; vec_rs.vs1 = rs1i; vec_rs.vs2 = rs2i;
                    vec_rs.arith_op = is_add ? 0 : is_sub ? 1 : 2;
                    vec_rs.rob_tag = new_tag;
                    vec_rs.vl = csr_vl;       // largo vectorial vigente al despachar
                    vec_rs.executing = false; // arranca en la etapa 3, como el resto
                    RobEntry& e = rob[new_tag];
                    e.valid = true; e.is_store = false; e.dest_is_fp = false;
                    e.dest = 0; e.ready = false;
                }
                // si vec_rs.busy: can_dispatch queda false -> stall (unidad ocupada)
            } else {
                // variante RVV no soportada en este alcance minimo: no-op
                can_dispatch = true;
                RobEntry& e = rob[new_tag];
                e.valid = true; e.is_store = false; e.dest = 0; e.value = 0; e.ready = true;
            }
        } else {
            // opcode no soportado (SYSTEM/FENCE/...): retira como no-op
            can_dispatch = true;
            RobEntry& e = rob[new_tag];
            e.valid = true; e.is_store = false; e.dest = 0; e.value = 0; e.ready = true;
        }

        if (can_dispatch) {
            // renombrado (x0 nunca; f0 SI es un registro normal del banco F)
            uint8_t dest = rob[new_tag].dest;
            if (rob[new_tag].dest_is_fp) {
                frat[dest].has_tag = true;
                frat[dest].tag = new_tag;
            } else if (dest != 0) {
                rat[dest].has_tag = true;
                rat[dest].tag = new_tag;
            }
            rob[new_tag].disp_idx = n_disp;
            if (trace)
                std::cout << "[" << cycle << "] DISPATCH d" << n_disp
                          << " (pc=" << this_pc << ", tag " << +new_tag << ")" << std::endl;
            n_disp++;
            rob_tail = (rob_tail + 1) & (ROB_SZ - 1);
            rob_count++;
            fetch_pc = next_fetch;
        }
    }

    void run() {
        const uint64_t MAX_CYCLES = 100000; // guarda contra cuelgues
        while (!halted && cycle < MAX_CYCLES) {
            cycle++;
            tick();
            wait(sc_time(1, SC_NS)); // ciclo nominal (las latencias de
                                     // memoria se suman via b_transport)
        }
        finished.notify();
    }

    void dump_regs() {
        std::cout << "Registros enteros (OOO):" << std::endl;
        for (int i = 0; i < 32; i++) {
            std::cout << "x" << i << "=0x" << std::hex << regs[i] << std::dec
                      << ((i % 8 == 7) ? "\n" : " ");
        }
    }

    void dump_fregs() {
        std::cout << "Registros flotantes (OOO):" << std::endl;
        for (int i = 0; i < 32; i++) {
            std::cout << "f" << i << "=0x" << std::hex << rv32i::float_to_bits(fregs[i]) << std::dec
                      << ((i % 8 == 7) ? "\n" : " ");
        }
    }
};

#endif // PROCESSOR_OOO_H
