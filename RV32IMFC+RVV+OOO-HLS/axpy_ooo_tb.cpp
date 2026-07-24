// axpy_ooo_tb.cpp — Experimento V-3/V-4: AXPY entero (y = a*x + y) sobre
// el core OoO+RVV, en dos versiones que producen EXACTAMENTE el mismo
// resultado arquitectonico:
//   (1) escalar  : lazo lw/mul/add/sw clasico
//   (2) vectorial: lazo strip-mined con vsetvli + vle32/vmul.vx/vadd.vv/vse32
// Se mide: ciclos, instrucciones retiradas (commits), IPC, ciclos/elemento
// y el speedup vectorial. Ambas versiones se validan contra la referencia
// y[i] = a*x[i] + y0 calculada en el host. Exit code 0/1 (apto make check).
//
// Mismas convenciones que rv32_ooo_tb.cpp: encoders de rv32c_defs.h,
// codificacion RVV verificada contra la spec v1.0, halt por 0x0000.
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <vector>
#include "rv32_ooo.h"
#include "rv32i_defs.h"
#include "rv32c_defs.h"

using namespace rv32c;
namespace F3A = rv32i::Funct3_ALU;
namespace F3M = rv32i::Funct3_MULDIV;

// ---- Codificacion RVV (mismos campos que rv32_ooo_tb.cpp / spec v1.0) ----
static const uint32_t F3_OPIVV = 0b000, F3_OPIVX = 0b100;
static const uint32_t OPC_V = 0b1010111, OPC_LOAD_FP = 0b0000111, OPC_STORE_FP = 0b0100111;
static const uint32_t F6_VADD = 0b000000, F6_VMUL = 0b100101; // VMUL en OPMVV/OPMVX (f3=010/110)
static const uint32_t F3_OPMVX = 0b110;

static uint32_t enc_vec_mem(uint32_t opcode, uint32_t vd_or_vs3, uint32_t rs1) {
    // nf=000, mew=0, mop=00 (unit-stride), vm=1, lumop/sumop=00000, width=110 (32b)
    return (1u << 25) | (rs1 << 15) | (0b110u << 12) | (vd_or_vs3 << 7) | opcode;
}
static uint32_t enc_vsetvli(uint32_t rd, uint32_t rs1, uint32_t vtypei) {
    return ((vtypei & 0x7FF) << 20) | (rs1 << 15) | (0b111u << 12) | (rd << 7) | OPC_V;
}
static const uint32_t VTYPE_E32_M1 = (0b010u << 3) | 0b000u;
static uint32_t enc_op_v(uint32_t funct6, uint32_t vs2, uint32_t vs1_or_rs1,
                         uint32_t funct3, uint32_t vd) {
    return (funct6 << 26) | (1u << 25) | (vs2 << 20) | (vs1_or_rs1 << 15) |
           (funct3 << 12) | (vd << 7) | OPC_V;
}

// ---- Parametros del experimento ----
#ifndef N_ELEMS_P
#define N_ELEMS_P 1024
#endif
static const int      N_ELEMS = N_ELEMS_P;        // elementos del AXPY
static const uint32_t A_VAL   = 3;           // y = a*x + y
static const uint32_t X_BASE  = 4096;        // byte addr de x[] (dmem[1024])
static const uint32_t Y_BASE  = 8192;        // byte addr de y[] (dmem[2048])
static const uint32_t Y0      = 7;           // valor inicial de y[i]

// ---- Arnes: carga el programa, corre hasta halt, cuenta ciclos y commits ----
struct RunStats { long cycles; long commits; bool halted; };

static RunStats run_program(const std::vector<uint16_t>& prog,
                            ap_uint<32> dmem[OOO_DMEM_WORDS],
                            long max_cycles) {
    static ap_uint<32> imem[OOO_IMEM_WORDS];
    for (int i = 0; i < OOO_IMEM_WORDS; i++) imem[i] = 0;
    for (size_t i = 0; i < prog.size(); i++) {
        uint32_t w = imem[i / 2].to_uint();
        if ((i & 1) == 0) w = (w & 0xFFFF0000u) | prog[i];
        else              w = (w & 0x0000FFFFu) | (uint32_t(prog[i]) << 16);
        imem[i / 2] = w;
    }
    ap_uint<1> dv; ap_uint<3> dt; ap_uint<32> dp;
    ap_uint<1> a0d, a1d, mdd, fpd, lsd, brd, vcd;
    ap_uint<3> a0t, a1t, mdt, fpt, lst, brt, vct;
    ap_uint<1> cv, cfp; ap_uint<5> crd; ap_uint<32> cval;
    static ap_uint<32> vregs_out[OOO_VEC_REGFILE_LEN];
    ap_uint<1> halted;

    rv32_ooo_tick(1, imem, dmem, dv, dt, dp, a0d,a0t, a1d,a1t, mdd,mdt,
                  fpd,fpt, lsd,lst, brd,brt, vcd,vct, cv, cfp, crd, cval,
                  vregs_out, halted);
    RunStats st{0, 0, false};
    while (!halted && st.cycles < max_cycles) {
        st.cycles++;
        rv32_ooo_tick(0, imem, dmem, dv, dt, dp, a0d,a0t, a1d,a1t, mdd,mdt,
                      fpd,fpt, lsd,lst, brd,brt, vcd,vct, cv, cfp, crd, cval,
                      vregs_out, halted);
        if (cv) st.commits++;
    }
    st.halted = halted;
    return st;
}

static void init_dmem(ap_uint<32> dmem[OOO_DMEM_WORDS]) {
    for (int i = 0; i < OOO_DMEM_WORDS; i++) dmem[i] = 0;
    for (int i = 0; i < N_ELEMS; i++) {
        dmem[X_BASE / 4 + i] = (uint32_t)i;   // x[i] = i
        dmem[Y_BASE / 4 + i] = Y0;            // y[i] = 7
    }
}

static int check_result(const ap_uint<32> dmem[OOO_DMEM_WORDS], const char* tag) {
    int errs = 0;
    for (int i = 0; i < N_ELEMS; i++) {
        uint32_t exp = A_VAL * (uint32_t)i + Y0;
        uint32_t got = dmem[Y_BASE / 4 + i].to_uint();
        if (got != exp) {
            if (errs < 4)
                std::printf("FAIL  %s: y[%d] = %u, esperado %u\n", tag, i, got, exp);
            errs++;
        }
    }
    if (errs == 0) std::printf("OK    %s: %d elementos verificados (y[i] = %u*i + %u)\n",
                               tag, N_ELEMS, A_VAL, Y0);
    else std::printf("FAIL  %s: %d elementos incorrectos\n", tag, errs);
    return errs;
}

// ---- Programa 1: AXPY escalar ----
// x5=a  x10=&x  x11=&y  x12=n
// loop: lw x6,(x10); mul x6,x6,x5; lw x7,(x11); add x6,x6,x7; sw x6,(x11);
//       addi x10,4; addi x11,4; addi x12,-1; bne x12,x0,loop
static std::vector<uint16_t> build_axpy_scalar() {
    const uint32_t OP = rv32i::Opcode::OP, OP_IMM = rv32i::Opcode::OP_IMM;
    const uint32_t LOAD = rv32i::Opcode::LOAD, STORE = rv32i::Opcode::STORE;
    const uint32_t MULDIV = rv32i::Funct7::MULDIV;
    std::vector<uint16_t> p;
    auto push32 = [&](uint32_t w){ p.push_back(w & 0xFFFF); p.push_back(w >> 16); };

    push32(i_type(OP_IMM, 5, F3A::ADD_SUB, 0, A_VAL));            // x5 = a
    push32(u_type(rv32i::Opcode::LUI, 10, X_BASE >> 12));         // x10 = 0x1000
    push32(u_type(rv32i::Opcode::LUI, 11, Y_BASE >> 12));         // x11 = 0x2000
    push32(i_type(OP_IMM, 12, F3A::ADD_SUB, 0, N_ELEMS & 0x7FF)); // x12 = n (n<2048)
    // loop (9 instrucciones de 4 bytes = 36 bytes):
    push32(i_type(LOAD, 6, rv32i::Funct3_LOAD::LW, 10, 0));       // lw  x6,0(x10)
    push32(r_type(OP, 6, F3M::MUL, 6, 5, MULDIV));                // mul x6,x6,x5
    push32(i_type(LOAD, 7, rv32i::Funct3_LOAD::LW, 11, 0));       // lw  x7,0(x11)
    push32(r_type(OP, 6, F3A::ADD_SUB, 6, 7, 0));                 // add x6,x6,x7
    push32(s_type(STORE, rv32i::Funct3_STORE::SW, 11, 6, 0));     // sw  x6,0(x11)
    push32(i_type(OP_IMM, 10, F3A::ADD_SUB, 10, 4));              // x10 += 4
    push32(i_type(OP_IMM, 11, F3A::ADD_SUB, 11, 4));              // x11 += 4
    push32(i_type(OP_IMM, 12, F3A::ADD_SUB, 12, -1));             // x12 -= 1
    push32(b_type(rv32i::Funct3_BRANCH::BNE, 12, 0, -32));        // bne x12,x0,loop (byte 48 -> 16)
    p.push_back(0x0000);                                          // halt
    return p;
}

// ---- Programa 2: AXPY vectorial (strip-mined) ----
// x5=a  x10=&x  x11=&y  x12=n
// loop: vsetvli x13,x12,e32m1; vle32 v1,(x10); vle32 v2,(x11);
//       vmul.vx v3,v1,x5; vadd.vv v4,v3,v2; vse32 v4,(x11);
//       slli x14,x13,2; add x10,x10,x14; add x11,x11,x14;
//       sub x12,x12,x13; bne x12,x0,loop
static std::vector<uint16_t> build_axpy_vector() {
    const uint32_t OP = rv32i::Opcode::OP, OP_IMM = rv32i::Opcode::OP_IMM;
    std::vector<uint16_t> p;
    auto push32 = [&](uint32_t w){ p.push_back(w & 0xFFFF); p.push_back(w >> 16); };

    push32(i_type(OP_IMM, 5, F3A::ADD_SUB, 0, A_VAL));            // x5 = a
    push32(u_type(rv32i::Opcode::LUI, 10, X_BASE >> 12));         // x10 = &x
    push32(u_type(rv32i::Opcode::LUI, 11, Y_BASE >> 12));         // x11 = &y
    push32(i_type(OP_IMM, 12, F3A::ADD_SUB, 0, N_ELEMS & 0x7FF)); // x12 = n
    // loop (11 instrucciones de 4 bytes = 44 bytes):
    push32(enc_vsetvli(13, 12, VTYPE_E32_M1));                    // vl=min(n,VLMAX)
    push32(enc_vec_mem(OPC_LOAD_FP, 1, 10));                      // vle32.v v1,(x10)
    push32(enc_vec_mem(OPC_LOAD_FP, 2, 11));                      // vle32.v v2,(x11)
    push32(enc_op_v(F6_VMUL, 1, 5, F3_OPMVX, 3));                 // vmul.vx v3,v1,x5
    push32(enc_op_v(F6_VADD, 3, 2, F3_OPIVV, 4));                 // vadd.vv v4,v3,v2
    push32(enc_vec_mem(OPC_STORE_FP, 4, 11));                     // vse32.v v4,(x11)
    push32(i_type(OP_IMM, 14, 0b001, 13, 2));                     // slli x14,x13,2
    push32(r_type(OP, 10, F3A::ADD_SUB, 10, 14, 0));              // add x10,x10,x14
    push32(r_type(OP, 11, F3A::ADD_SUB, 11, 14, 0));              // add x11,x11,x14
    push32(r_type(OP, 12, F3A::ADD_SUB, 12, 13, rv32i::Funct7::ALT)); // sub x12,x12,x13
    push32(b_type(rv32i::Funct3_BRANCH::BNE, 12, 0, -40));        // bne x12,x0,loop (byte 56 -> 16)
    p.push_back(0x0000);                                          // halt
    return p;
}

int main() {
    static ap_uint<32> dmem[OOO_DMEM_WORDS];
    int fails = 0;
    const long MAXC = 500000;

    std::printf("================================================================\n");
    std::printf("  AXPY entero (y = %u*x + y), n=%d, sobre el core OoO+RVV\n", A_VAL, N_ELEMS);
    std::printf("  VLEN=128 (VLMAX=4 con e32), lanes=%d, ROB=%d\n", OOO_VEC_LANES, 8);
    std::printf("================================================================\n\n");

    init_dmem(dmem);
    RunStats s1 = run_program(build_axpy_scalar(), dmem, MAXC);
    if (!s1.halted) { std::printf("FAIL  escalar: no termino en %ld ciclos\n", MAXC); return 1; }
    fails += check_result(dmem, "escalar  ");

    init_dmem(dmem);
    RunStats s2 = run_program(build_axpy_vector(), dmem, MAXC);
    if (!s2.halted) { std::printf("FAIL  vectorial: no termino en %ld ciclos\n", MAXC); return 1; }
    fails += check_result(dmem, "vectorial");

    double ipc1 = (double)s1.commits / (double)s1.cycles;
    double ipc2 = (double)s2.commits / (double)s2.cycles;
    std::printf("\n%-11s %10s %10s %8s %12s\n", "version", "ciclos", "commits", "IPC", "ciclos/elem");
    std::printf("%-11s %10ld %10ld %8.3f %12.2f\n", "escalar",  s1.cycles, s1.commits, ipc1, (double)s1.cycles / N_ELEMS);
    std::printf("%-11s %10ld %10ld %8.3f %12.2f\n", "vectorial", s2.cycles, s2.commits, ipc2, (double)s2.cycles / N_ELEMS);
    std::printf("\nreduccion de instrucciones: %.2fx  (%ld -> %ld)\n",
                (double)s1.commits / (double)s2.commits, s1.commits, s2.commits);
    std::printf("speedup vectorial en ciclos: %.2fx  (%ld -> %ld)\n",
                (double)s1.cycles / (double)s2.cycles, s1.cycles, s2.cycles);

    if (fails == 0) std::printf("\nTodos los checks pasaron.\n");
    else            std::printf("\n%d check(s) fallaron.\n", fails);
    return fails == 0 ? 0 : 1;
}
