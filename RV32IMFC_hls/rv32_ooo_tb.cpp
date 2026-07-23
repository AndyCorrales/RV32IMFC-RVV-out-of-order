#include <cstdio>
#include <cstdint>
#include <vector>
#include "rv32_ooo.h"
#include "rv32i_defs.h"
#include "rv32c_defs.h" // encoders r_type/i_type/s_type/b_type/u_type/j_type

// Testbench del core OOO RV32IMFC. El programa (ensamblado a mano, en
// HALFWORDS para poder intercalar comprimidas de la extension C) ejercita:
//   - I+M: dependencias RAW via CDB, branch tomado, JAL con link, SW/LW
//     round-trip, LUI, MUL/DIV con independientes detras (evidencia OOO)
//   - F: FCVT.S.W (lee banco entero), FMUL, FMADD (TRES operandos rs3),
//     FSW/FLW round-trip por la LSU, FEQ (escribe banco entero),
//     FMV.X.W (bits crudos), FDIV largo con independiente detras
//     (evidencia OOO cruzando bancos)
//   - C: C.LI + C.ADDI comprimidas, con una instruccion de 32 bits
//     arrancando en pc%4==2 (straddle de palabra) entre ambas
//   - RVV (coprocesamiento): vle32.v x2, vadd.vv, vmul.vv, vse32.v --
//     con instrucciones escalares INDEPENDIENTES despachadas justo
//     despues de cada operacion vectorial (latencia 2 ciclos) y que
//     completan antes -- la evidencia de que el core escalar y la
//     unidad vectorial avanzan en paralelo, no que "RVV bloquea todo".
//
// El TB reconstruye el estado arquitectonico (x, f, y v) SOLO desde el
// stream de commit + el puerto vregs_out, sin backdoor al DUT.

using namespace rv32c;
namespace F3A = rv32i::Funct3_ALU;
namespace F3M = rv32i::Funct3_MULDIV;
namespace F7F = rv32i::Funct7_FP;

// Codificacion RVV -- mismos campos de bits que rv32_vector.cpp/rv32_ooo.cpp,
// verificados contra la especificacion oficial RVV v1.0.
static ap_uint<32> enc_vec_mem(uint32_t opcode, uint32_t vd_or_vs3, uint32_t rs1) {
    return (0u << 29) | (0u << 28) | (0u << 26) | (1u << 25) | (0u << 20) |
           (rs1 << 15) | (0b110u << 12) | (vd_or_vs3 << 7) | opcode;
}
static ap_uint<32> enc_op_v(uint32_t funct6, uint32_t vs2, uint32_t vs1, uint32_t funct3, uint32_t vd) {
    const uint32_t vm = 1;
    return (funct6 << 26) | (vm << 25) | (vs2 << 20) | (vs1 << 15) | (funct3 << 12) | (vd << 7) | 0b1010111u;
}
// vsetvli rd, rs1, vtypei -- instr[31]=0, zimm[10:0]=instr[30:20],
// rs1=AVL, funct3=111 (OPCFG). Formato de la seccion 5 de la spec.
static ap_uint<32> enc_vsetvli(uint32_t rd, uint32_t rs1, uint32_t vtypei) {
    return ((vtypei & 0x7FF) << 20) | (rs1 << 15) | (0b111u << 12) | (rd << 7) | 0b1010111u;
}
// vtype para SEW=32 (vsew=010 en bits[5:3]) y LMUL=1 (vlmul=000 en bits[2:0])
static const uint32_t VTYPE_E32_M1 = (0b010u << 3) | 0b000u;

int main() {
    ap_uint<32> imem[OOO_IMEM_WORDS] = {0};
    ap_uint<32> dmem[OOO_DMEM_WORDS] = {0};

    // datos de prueba para la seccion RVV -- direcciones 0/16, lejos de
    // 64/68 que usa la seccion escalar (dmem[16]/dmem[17]), sin colision
    dmem[0] = 10; dmem[1] = 20; dmem[2] = 30; dmem[3] = 40; // v1 tras vle32.v
    dmem[4] = 1;  dmem[5] = 2;  dmem[6] = 3;  dmem[7] = 4;  // v2 tras vle32.v

    const uint32_t OP     = rv32i::Opcode::OP;
    const uint32_t OP_IMM = rv32i::Opcode::OP_IMM;
    const uint32_t LOAD   = rv32i::Opcode::LOAD;
    const uint32_t STORE  = rv32i::Opcode::STORE;
    const uint32_t MULDIV = rv32i::Funct7::MULDIV;
    const uint32_t OP_FP  = rv32i::Opcode::OP_FP;
    const uint32_t rvv_f3_opivv  = 0b000;
    const uint32_t rvv_f3_opmvv  = 0b010;
    const uint32_t rvv_funct6_add = 0b000000;
    const uint32_t rvv_funct6_mul = 0b100101;

    std::vector<uint16_t> prog;
    auto push32 = [&](uint32_t w) { prog.push_back(w & 0xFFFF); prog.push_back(w >> 16); };
    auto push16 = [&](uint16_t h) { prog.push_back(h); };

    // ---- parte I+M (pcs 0..56) ----
    push32(i_type(OP_IMM, 1, F3A::ADD_SUB, 0, 5));         //   0: addi x1,x0,5      d0
    push32(i_type(OP_IMM, 2, F3A::ADD_SUB, 0, 10));        //   4: addi x2,x0,10     d1
    push32(r_type(OP, 3, F3M::MUL, 1, 2, MULDIV));         //   8: mul  x3,x1,x2     d2  (lat 3)
    push32(i_type(OP_IMM, 4, F3A::ADD_SUB, 0, 7));         //  12: addi x4,x0,7      d3  (OOO vs d2)
    push32(r_type(OP, 5, F3A::ADD_SUB, 3, 4, 0));          //  16: add  x5,x3,x4     d4  = 57
    push32(s_type(STORE, rv32i::Funct3_STORE::SW, 0, 5, 64)); // 20: sw x5,64(x0)    d5
    push32(i_type(LOAD, 6, rv32i::Funct3_LOAD::LW, 0, 64));   // 24: lw x6,64(x0)    d6
    push32(b_type(rv32i::Funct3_BRANCH::BEQ, 1, 1, 8));    //  28: beq  x1,x1,+8     d7  (tomado)
    push32(i_type(OP_IMM, 7, F3A::ADD_SUB, 0, 99));        //  32: addi x7,x0,99     (saltada)
    push32(i_type(OP_IMM, 7, F3A::ADD_SUB, 0, 1));         //  36: addi x7,x0,1      d8
    push32(j_type(8, 8));                                  //  40: jal  x8,+8        d9  (link 44)
    push32(i_type(OP_IMM, 9, F3A::ADD_SUB, 0, 88));        //  44: addi x9,x0,88     (saltada)
    push32(u_type(rv32i::Opcode::LUI, 10, 0x12345));       //  48: lui  x10,0x12345  d10
    push32(r_type(OP, 11, F3M::DIV, 3, 1, MULDIV));        //  52: div  x11,x3,x1    d11 (lat 8)
    push32(r_type(OP, 12, F3A::ADD_SUB, 2, 1, rv32i::Funct7::ALT)); // 56: sub x12,x2,x1 d12 (OOO vs d11)

    // ---- parte F (pcs 60..104) ----
    push32(i_type(OP_IMM, 20, F3A::ADD_SUB, 0, 3));        //  60: addi x20,x0,3     d13
    push32(r_type(OP_FP, 1, 0, 20, rv32i::Rs2_FCVT::W, F7F::FCVT_S_W)); // 64: fcvt.s.w f1,x20  d14 (3.0)
    push32(i_type(OP_IMM, 21, F3A::ADD_SUB, 0, 4));        //  68: addi x21,x0,4     d15
    push32(r_type(OP_FP, 2, 0, 21, rv32i::Rs2_FCVT::W, F7F::FCVT_S_W)); // 72: fcvt.s.w f2,x21  d16 (4.0)
    push32(r_type(OP_FP, 3, 0, 1, 2, F7F::FMUL_S));        //  76: fmul.s f3,f1,f2   d17 (12.0)
    push32(r_type(rv32i::Opcode::FMADD, 5, 0, 1, 2, (3u << 2))); // 80: fmadd.s f5,f1,f2,f3 d18 (3*4+12=24.0)
    push32(s_type(rv32i::Opcode::STORE_FP, rv32i::Funct3_FP_MEM::W, 0, 5, 68)); // 84: fsw f5,68(x0) d19
    push32(i_type(rv32i::Opcode::LOAD_FP, 6, rv32i::Funct3_FP_MEM::W, 0, 68));  // 88: flw f6,68(x0) d20
    push32(r_type(OP_FP, 22, rv32i::Funct3_FCMP::FEQ, 5, 6, F7F::FCMP_S)); // 92: feq.s x22,f5,f6 d21 (=1)
    push32(r_type(OP_FP, 23, 0, 3, 0, F7F::FMV_X_W_FCLASS_S)); // 96: fmv.x.w x23,f3 d22 (0x41400000)
    push32(r_type(OP_FP, 7, 0, 3, 1, F7F::FDIV_S));        // 100: fdiv.s f7,f3,f1   d23 (4.0, lat 8)
    push32(i_type(OP_IMM, 24, F3A::ADD_SUB, 0, 2));        // 104: addi x24,x0,2     d24 (OOO vs d23)

    // ---- parte C (pcs 108..114) ----
    push16(0x47A5);                                        // 108: c.li x15,9        d25 (16 bits)
    push32(i_type(OP_IMM, 16, F3A::ADD_SUB, 0, 21));       // 110: addi x16,x0,21    d26 (32 bits, pc%4==2: straddle)
    push16(0x078D);                                        // 114: c.addi x15,3      d27 -> x15=12

    // ---- parte RVV: coprocesamiento (pcs 116..164) ----
    // Todo programa RVV real arranca configurando el largo vectorial con
    // vsetvli: al reset vtype tiene vill y vl=0, asi que SIN este vsetvli
    // las vectoriales de abajo no procesarian ningun elemento.
    push32(enc_vsetvli(/*rd=*/19, /*rs1=*/0, VTYPE_E32_M1));            // 116: vsetvli x19,x0,e32,m1 d28 -> vl=VLMAX=4
    push32(enc_vec_mem(rv32i::Opcode::LOAD_FP, /*vd=*/1, /*rs1=*/0));    // 120: vle32.v v1,(x0)   d29 -> {10,20,30,40}
    push32(i_type(OP_IMM, 13, F3A::ADD_SUB, 0, 16));                    // 124: addi x13,x0,16    d30
    push32(enc_vec_mem(rv32i::Opcode::LOAD_FP, /*vd=*/2, /*rs1=*/13));  // 128: vle32.v v2,(x13)  d31 -> {1,2,3,4}
    push32(enc_op_v(rvv_funct6_add, 2, 1, rvv_f3_opivv, /*vd=*/3));     // 132: vadd.vv v3,v2,v1  d32 (lat 2) -> {11,22,33,44}
    push32(i_type(OP_IMM, 14, F3A::ADD_SUB, 0, 99));                    // 136: addi x14,x0,99    d33 (OOO vs d32)
    push32(enc_op_v(rvv_funct6_mul, 1, 1, rvv_f3_opmvv, /*vd=*/4));     // 140: vmul.vv v4,v1,v1  d34 (lat 2) -> {100,400,900,1600}
    push32(i_type(OP_IMM, 17, F3A::ADD_SUB, 0, 77));                    // 144: addi x17,x0,77    d35 (OOO vs d34)
    push32(enc_vec_mem(rv32i::Opcode::LOAD_FP, /*vd=*/5, /*rs1=*/0));   // 148: vle32.v v5,(x0)   d36 -> centinela {10,20,30,40}
    push32(i_type(OP_IMM, 18, F3A::ADD_SUB, 0, 32));                    // 152: addi x18,x0,32    d37
    push32(enc_vec_mem(rv32i::Opcode::STORE_FP, /*vs3=*/3, /*rs1=*/18)); // 156: vse32.v v3,(x18) d38 -> dmem[32..47]

    // ---- largo vectorial DINAMICO: vl=2 (AVL=2 < VLMAX=4) ----
    // Con vl=2, el vadd.vv solo debe tocar los 2 primeros elementos de v5
    // y dejar los otros 2 SIN TOCAR (tail-undisturbed): v5 quedara
    // {11,22, 30,40} -- los dos primeros calculados, los dos ultimos
    // sobrevivientes del centinela cargado arriba con vl=4.
    push32(i_type(OP_IMM, 26, F3A::ADD_SUB, 0, 2));                     // 160: addi x26,x0,2     d39 (AVL=2)
    push32(enc_vsetvli(/*rd=*/27, /*rs1=*/26, VTYPE_E32_M1));           // 164: vsetvli x27,x26,e32,m1 d40 -> vl=2
    push32(enc_op_v(rvv_funct6_add, 2, 1, rvv_f3_opivv, /*vd=*/5));     // 168: vadd.vv v5,v2,v1  d41 -> solo lanes 0..1

    push16(0x0000);                                        // 172: fin de programa

    for (size_t i = 0; i < prog.size(); i++) {
        uint32_t w = imem[i / 2].to_uint();
        if (i % 2 == 0) w = (w & 0xFFFF0000u) | prog[i];
        else            w = (w & 0x0000FFFFu) | (uint32_t(prog[i]) << 16);
        imem[i / 2] = w;
    }

    ap_uint<1> disp_valid; ap_uint<3> disp_tag; ap_uint<32> disp_pc;
    ap_uint<1> alu0_done;  ap_uint<3> alu0_tag;
    ap_uint<1> alu1_done;  ap_uint<3> alu1_tag;
    ap_uint<1> md_done;    ap_uint<3> md_tag;
    ap_uint<1> fpu_done;   ap_uint<3> fpu_tag;
    ap_uint<1> lsu_done;   ap_uint<3> lsu_tag;
    ap_uint<1> br_done;    ap_uint<3> br_tag;
    ap_uint<1> vec_done;   ap_uint<3> vec_tag;
    ap_uint<1> commit_valid; ap_uint<1> commit_is_fp;
    ap_uint<5> commit_rd; ap_uint<32> commit_value;
    ap_uint<32> vregs_out[OOO_VEC_REGFILE_LEN];
    ap_uint<1> halted;

    rv32_ooo_tick(1, imem, dmem,
                  disp_valid, disp_tag, disp_pc,
                  alu0_done, alu0_tag, alu1_done, alu1_tag,
                  md_done, md_tag, fpu_done, fpu_tag,
                  lsu_done, lsu_tag, br_done, br_tag, vec_done, vec_tag,
                  commit_valid, commit_is_fp, commit_rd, commit_value,
                  vregs_out, halted);

    uint32_t regs[32] = {0};   // ambos reconstruidos SOLO desde commits
    uint32_t fregs[32] = {0};
    int tag2disp[8];
    for (int i = 0; i < 8; i++) tag2disp[i] = -1;
    int complete_cycle[64];
    for (int i = 0; i < 64; i++) complete_cycle[i] = -1;
    int n_disp = 0, n_commit = 0;

    printf("ciclo | evento\n");
    printf("------+------------------------------------------------\n");

    int cycle = 0;
    const int MAX_CYCLES = 300;
    while (!halted && cycle < MAX_CYCLES) {
        cycle++;
        rv32_ooo_tick(0, imem, dmem,
                      disp_valid, disp_tag, disp_pc,
                      alu0_done, alu0_tag, alu1_done, alu1_tag,
                      md_done, md_tag, fpu_done, fpu_tag,
                      lsu_done, lsu_tag, br_done, br_tag, vec_done, vec_tag,
                      commit_valid, commit_is_fp, commit_rd, commit_value,
                      vregs_out, halted);

        if (disp_valid) {
            tag2disp[disp_tag.to_uint()] = n_disp;
            printf("%5d | DISPATCH d%-2d (pc=%3u, tag %d)\n",
                   cycle, n_disp, disp_pc.to_uint(), disp_tag.to_uint());
            n_disp++;
        }
        struct { ap_uint<1>* v; ap_uint<3>* t; const char* name; } evs[] = {
            {&alu0_done, &alu0_tag, "ALU0"}, {&alu1_done, &alu1_tag, "ALU1"},
            {&md_done, &md_tag, "MULDIV"}, {&fpu_done, &fpu_tag, "FPU"},
            {&lsu_done, &lsu_tag, "LSU"}, {&br_done, &br_tag, "BR"},
            {&vec_done, &vec_tag, "VEC"},
        };
        for (auto& e : evs) {
            if (*e.v) {
                int d = tag2disp[e.t->to_uint()];
                if (d >= 0 && d < 64) complete_cycle[d] = cycle;
                printf("%5d | %-6s completa d%-2d (tag %d)\n",
                       cycle, e.name, d, e.t->to_uint());
            }
        }
        if (commit_valid) {
            n_commit++;
            if (commit_is_fp) {
                fregs[commit_rd.to_uint()] = commit_value.to_uint();
                printf("%5d | COMMIT  f%-2u = 0x%08x\n",
                       cycle, commit_rd.to_uint(), commit_value.to_uint());
            } else {
                if (commit_rd != 0) regs[commit_rd.to_uint()] = commit_value.to_uint();
                printf("%5d | COMMIT  x%-2u = 0x%08x\n",
                       cycle, commit_rd.to_uint(), commit_value.to_uint());
            }
        }
    }
    printf("------+------------------------------------------------\n");
    printf("Halted en el ciclo %d (%d dispatches, %d commits)\n\n", cycle, n_disp, n_commit);

    bool ok = true;
    struct { int reg; uint32_t expect; const char* what; } xchecks[] = {
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
        {13, 16,         "addi (base del segundo vle32.v)"},
        {14, 99,         "addi independiente de vadd.vv"},
        {17, 77,         "addi independiente de vmul.vv"},
        {19, 4,          "vsetvli x19,x0 -> vl=VLMAX=4 (rs1=x0 pide el maximo)"},
        {26, 2,          "addi (AVL para el segundo vsetvli)"},
        {27, 2,          "vsetvli x27,x26 -> vl=min(AVL=2,VLMAX=4)=2 (largo dinamico)"},
        {18, 32,         "addi (direccion del vse32.v)"},
    };
    for (auto& c : xchecks) {
        if (regs[c.reg] != c.expect) {
            printf("FAIL  x%-2d = 0x%08x, esperado 0x%08x (%s)\n",
                   c.reg, regs[c.reg], c.expect, c.what);
            ok = false;
        } else {
            printf("OK    x%-2d = 0x%08x (%s)\n", c.reg, regs[c.reg], c.what);
        }
    }
    struct { int reg; uint32_t expect; const char* what; } fchecks[] = {
        {1, 0x40400000, "fcvt.s.w 3 -> 3.0"},
        {2, 0x40800000, "fcvt.s.w 4 -> 4.0"},
        {3, 0x41400000, "fmul.s 3*4 = 12.0"},
        {5, 0x41C00000, "fmadd.s 3*4+12 = 24.0 (rs3)"},
        {6, 0x41C00000, "flw round-trip del fsw"},
        {7, 0x40800000, "fdiv.s 12/3 = 4.0"},
    };
    for (auto& c : fchecks) {
        if (fregs[c.reg] != c.expect) {
            printf("FAIL  f%-2d = 0x%08x, esperado 0x%08x (%s)\n",
                   c.reg, fregs[c.reg], c.expect, c.what);
            ok = false;
        } else {
            printf("OK    f%-2d = 0x%08x (%s)\n", c.reg, fregs[c.reg], c.what);
        }
    }

    struct { int later, earlier; const char* what; } ooo[] = {
        {3, 2,   "d3 (addi) antes que d2 (mul)"},
        {12, 11, "d12 (sub) antes que d11 (div)"},
        {24, 23, "d24 (addi) antes que d23 (fdiv) -- OOO cruzando bancos"},
        {33, 32, "d33 (addi) antes que d32 (vadd.vv) -- coprocesamiento: escalar avanza mientras VEC ejecuta"},
        {35, 34, "d35 (addi) antes que d34 (vmul.vv) -- coprocesamiento"},
    };
    for (auto& c : ooo) {
        if (complete_cycle[c.later] > 0 && complete_cycle[c.earlier] > 0 &&
            complete_cycle[c.later] < complete_cycle[c.earlier]) {
            printf("OK    OOO: %s (ciclos %d < %d)\n", c.what,
                   complete_cycle[c.later], complete_cycle[c.earlier]);
        } else {
            printf("FAIL  OOO: %s NO se cumplio (ciclos %d vs %d)\n", c.what,
                   complete_cycle[c.later], complete_cycle[c.earlier]);
            ok = false;
        }
    }

    const int N_EXPECTED = 42;
    if (n_disp != N_EXPECTED) {
        printf("FAIL  dispatches: %d, esperados %d\n", n_disp, N_EXPECTED);
        ok = false;
    }
    if (n_commit != N_EXPECTED) {
        printf("FAIL  commits: %d, esperados %d\n", n_commit, N_EXPECTED);
        ok = false;
    }
    if (dmem[16] != 57) {
        printf("FAIL  dmem[16] = 0x%08x, esperado 57 (sw al commit)\n", dmem[16].to_uint());
        ok = false;
    } else {
        printf("OK    dmem[16] = 57 (sw escribio al commit)\n");
    }
    if (dmem[17] != 0x41C00000) {
        printf("FAIL  dmem[17] = 0x%08x, esperado 0x41C00000 (fsw al commit)\n", dmem[17].to_uint());
        ok = false;
    } else {
        printf("OK    dmem[17] = 0x41C00000 (fsw escribio 24.0f al commit)\n");
    }

    // banco vectorial: leido del puerto vregs_out (no es backdoor, es un
    // puerto de salida declarado del DUT, igual que r0_out..r3_out en
    // ooo_demo.cpp y vregs_out en rv32_vector.cpp)
    struct { int vreg, lane; uint32_t expect; } vchecks[] = {
        {1,0,10},{1,1,20},{1,2,30},{1,3,40},   // v1 (vle32.v)
        {2,0,1}, {2,1,2}, {2,2,3}, {2,3,4},    // v2 (vle32.v)
        {3,0,11},{3,1,22},{3,2,33},{3,3,44},   // v3 (vadd.vv)
        {4,0,100},{4,1,400},{4,2,900},{4,3,1600}, // v4 (vmul.vv)
        // v5: vadd.vv ejecutado con vl=2 -- lanes 0..1 calculados,
        // lanes 2..3 conservan el centinela cargado con vl=4 (tail
        // undisturbed). Esta es la evidencia del largo vectorial dinamico.
        {5,0,11},{5,1,22},{5,2,30},{5,3,40},
    };
    for (auto& c : vchecks) {
        uint32_t got = vregs_out[c.vreg * OOO_VEC_LANES + c.lane].to_uint();
        if (got != c.expect) {
            printf("FAIL  v%d[%d] = 0x%08x, esperado 0x%08x\n", c.vreg, c.lane, got, c.expect);
            ok = false;
        } else {
            printf("OK    v%d[%d] = 0x%08x\n", c.vreg, c.lane, got);
        }
    }
    uint32_t m32 = dmem[8].to_uint(), m33 = dmem[9].to_uint(), m34 = dmem[10].to_uint(), m35 = dmem[11].to_uint();
    if (m32 == 11 && m33 == 22 && m34 == 33 && m35 == 44) {
        printf("OK    dmem[32..47] = {11,22,33,44} (vse32.v round-trip, resuelto en cabeza del ROB)\n");
    } else {
        printf("FAIL  dmem[32..47] = {%u,%u,%u,%u}, esperado {11,22,33,44}\n", m32, m33, m34, m35);
        ok = false;
    }

    if (!ok) { printf("\nAl menos un check fallo.\n"); return 1; }
    printf("\nTodos los checks pasaron.\n");
    return 0;
}
