#include <cstdio>
#include <cstdint>
#include "rv32_vector.h"

// Testbench del decoder RVV minimo. Codifica las 5 instrucciones reales
// (vle32.v x2, vadd.vv, vsub.vv, vmul.vv, vse32.v) a mano, con los
// mismos campos de bits verificados contra la especificacion oficial
// (ver comentarios de rv32_vector.cpp), y corre el programa:
//
//   v1 <- mem[0..15]    (vle32.v v1, (0))   -- {10,20,30,40}
//   v2 <- mem[16..31]   (vle32.v v2, (16))  -- {1,2,3,4}
//   v3 <- v2 + v1       (vadd.vv v3,v2,v1)  -- {11,22,33,44}
//   v4 <- v2 - v1       (vsub.vv v4,v2,v1)  -- {-9,-18,-27,-36}
//   v5 <- v1 * v1       (vmul.vv v5,v1,v1)  -- {100,400,900,1600}
//   mem[32..47] <- v3   (vse32.v v3, (32))
//
// Verifica los 5 registros vectoriales resultantes Y el round-trip a
// memoria del store -- sin acceso backdoor al DUT, todo pasa por
// rv32_vector_step().

static ap_uint<32> enc_vec_mem(uint32_t opcode, uint32_t vd_or_vs3, uint32_t rs1) {
    // nf=000, mew=0, mop=00, vm=1 (unmasked), lumop/sumop=00000, width=110 (32b)
    return (0u << 29) | (0u << 28) | (0u << 26) | (1u << 25) | (0u << 20) |
           (rs1 << 15) | (0b110u << 12) | (vd_or_vs3 << 7) | opcode;
}

static ap_uint<32> enc_op_v(uint32_t funct6, uint32_t vs2, uint32_t vs1, uint32_t funct3, uint32_t vd) {
    const uint32_t vm = 1; // unmasked
    return (funct6 << 26) | (vm << 25) | (vs2 << 20) | (vs1 << 15) | (funct3 << 12) | (vd << 7) | 0b1010111u;
}

int main() {
    ap_uint<32> mem[VEC_MEM_WORDS] = {0};
    ap_uint<32> vregs_a[VEC_REGFILE_LEN] = {0};
    ap_uint<32> vregs_b[VEC_REGFILE_LEN] = {0};
    ap_uint<1>  halted;

    // datos de prueba en memoria (direcciones de BYTE 0 y 16, cada vector
    // ocupa 4 palabras = 16 bytes)
    mem[0] = 10; mem[1] = 20; mem[2] = 30; mem[3] = 40;   // v1
    mem[4] = 1;  mem[5] = 2;  mem[6] = 3;  mem[7] = 4;    // v2

    const uint32_t OPC_LOAD  = 0b0000111;
    const uint32_t OPC_STORE = 0b0100111;
    const uint32_t F3_OPIVV  = 0b000;
    const uint32_t F3_OPMVV  = 0b010;
    const uint32_t F6_ADD    = 0b000000;
    const uint32_t F6_SUB    = 0b000010;
    const uint32_t F6_MUL    = 0b100101;

    struct Step { ap_uint<32> instr; ap_uint<32> rs1_val; const char* what; };
    Step prog[] = {
        { enc_vec_mem(OPC_LOAD, /*vd=*/1, /*rs1=*/0),  0,  "vle32.v v1, (0)"   },
        { enc_vec_mem(OPC_LOAD, /*vd=*/2, /*rs1=*/0), 16,  "vle32.v v2, (16)"  },
        { enc_op_v(F6_ADD, /*vs2=*/2, /*vs1=*/1, F3_OPIVV, /*vd=*/3), 0, "vadd.vv v3,v2,v1" },
        { enc_op_v(F6_SUB, /*vs2=*/2, /*vs1=*/1, F3_OPIVV, /*vd=*/4), 0, "vsub.vv v4,v2,v1" },
        { enc_op_v(F6_MUL, /*vs2=*/1, /*vs1=*/1, F3_OPMVV, /*vd=*/5), 0, "vmul.vv v5,v1,v1" },
        { enc_vec_mem(OPC_STORE, /*vs3=*/3, /*rs1=*/0), 32, "vse32.v v3, (32)"  },
    };
    const int N = sizeof(prog) / sizeof(prog[0]);

    ap_uint<32>* cur_in = vregs_a;
    ap_uint<32>* cur_out = vregs_b;

    for (int i = 0; i < N; i++) {
        rv32_vector_step(prog[i].instr, prog[i].rs1_val, cur_in, cur_out, mem, halted);
        printf("[%d] %-20s halted=%d\n", i, prog[i].what, (int)halted);
        ap_uint<32>* tmp = cur_in; cur_in = cur_out; cur_out = tmp;
    }
    // tras el ultimo swap, el banco vigente es cur_in
    ap_uint<32>* final_regs = cur_in;

    bool ok = true;
    struct Check { int vreg; int lane; uint32_t expect; };
    Check checks[] = {
        {1,0,10},{1,1,20},{1,2,30},{1,3,40},
        {2,0,1}, {2,1,2}, {2,2,3}, {2,3,4},
        {3,0,11},{3,1,22},{3,2,33},{3,3,44},
        {4,0,(uint32_t)(1-10)},{4,1,(uint32_t)(2-20)},{4,2,(uint32_t)(3-30)},{4,3,(uint32_t)(4-40)},
        {5,0,100},{5,1,400},{5,2,900},{5,3,1600},
    };
    for (auto& c : checks) {
        uint32_t got = final_regs[c.vreg * VEC_LANES + c.lane].to_uint();
        if (got != c.expect) {
            printf("FAIL  v%d[%d] = 0x%08x, esperado 0x%08x\n", c.vreg, c.lane, got, c.expect);
            ok = false;
        } else {
            printf("OK    v%d[%d] = 0x%08x\n", c.vreg, c.lane, got);
        }
    }

    uint32_t m32 = mem[8].to_uint(), m33 = mem[9].to_uint(), m34 = mem[10].to_uint(), m35 = mem[11].to_uint();
    if (m32 == 11 && m33 == 22 && m34 == 33 && m35 == 44) {
        printf("OK    mem[32..47] = {11,22,33,44} (vse32.v round-trip)\n");
    } else {
        printf("FAIL  mem[32..47] = {%u,%u,%u,%u}, esperado {11,22,33,44}\n", m32, m33, m34, m35);
        ok = false;
    }

    if (!ok) { printf("\nAl menos un check fallo.\n"); return 1; }
    printf("\nTodos los checks pasaron.\n");
    return 0;
}
