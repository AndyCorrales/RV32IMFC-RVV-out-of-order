// axpy_soc_tlm.cpp — Experimento V-3/V-4 sobre el RISC-V SoC, pista TLM.
//
// El MISMO experimento AXPY de RV32IMFC+RVV+OOO-HLS/axpy_ooo_tb.cpp
// (autor: Daniel Chacon), corriendo sobre el modelo SystemC/TLM del SoC
// (TAGE + VIQ + interconnect con arbitro). Los programas y la medicion
// son identicos a la version HLS del SoC (RISCV-SoC/axpy_soc_tb.cpp):
// si la paridad ciclo a ciclo del proyecto se mantiene, los CUATRO
// numeros (ciclos y commits de cada version) deben salir IGUALES que
// alla. Ese es, de hecho, un check mas.
//
// Compilar:  g++ -std=c++17 -I src -o axpy_soc_tlm src/axpy_soc_tlm.cpp -lsystemc
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <vector>
#include <systemc.h>

#include "memory_map.h"
#include "processor_ooo.h"
#include "processor_vector_unit.h"
#include "processor_tick.h"
#include "processor_dispatch.h"
#include "bus.h"
#include "memory.h"
#include "uart.h"
#include "vector_unit.h"
#include "rv32c_defs.h"

using namespace rv32c;
namespace F3A = rv32i::Funct3_ALU;
namespace F3M = rv32i::Funct3_MULDIV;

// ---- Codificacion RVV (misma que el tb original / spec v1.0) ----
static const uint32_t F3_OPIVV = 0b000;
static const uint32_t OPC_V = 0b1010111, OPC_LOAD_FP = 0b0000111, OPC_STORE_FP = 0b0100111;
static const uint32_t F6_VADD = 0b000000, F6_VMUL = 0b100101;
static const uint32_t F3_OPMVX = 0b110;

static uint32_t enc_vec_mem(uint32_t opcode, uint32_t vd_or_vs3, uint32_t rs1) {
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

// ---- Parametros del experimento (identicos al original) ----
static const int      N_ELEMS = 1024;
static const uint32_t A_VAL   = 3;
static const uint32_t X_BASE  = 4096;
static const uint32_t Y_BASE  = 8192;
static const uint32_t Y0      = 7;

// ---- los dos programas: copiados VERBATIM del tb original ----
static std::vector<uint16_t> build_axpy_scalar() {
    const uint32_t OP = rv32i::Opcode::OP, OP_IMM = rv32i::Opcode::OP_IMM;
    const uint32_t LOAD = rv32i::Opcode::LOAD, STORE = rv32i::Opcode::STORE;
    const uint32_t MULDIV = rv32i::Funct7::MULDIV;
    std::vector<uint16_t> p;
    auto push32 = [&](uint32_t w){ p.push_back(w & 0xFFFF); p.push_back(w >> 16); };
    push32(i_type(OP_IMM, 5, F3A::ADD_SUB, 0, A_VAL));
    push32(u_type(rv32i::Opcode::LUI, 10, X_BASE >> 12));
    push32(u_type(rv32i::Opcode::LUI, 11, Y_BASE >> 12));
    push32(i_type(OP_IMM, 12, F3A::ADD_SUB, 0, N_ELEMS & 0x7FF));
    push32(i_type(LOAD, 6, rv32i::Funct3_LOAD::LW, 10, 0));
    push32(r_type(OP, 6, F3M::MUL, 6, 5, MULDIV));
    push32(i_type(LOAD, 7, rv32i::Funct3_LOAD::LW, 11, 0));
    push32(r_type(OP, 6, F3A::ADD_SUB, 6, 7, 0));
    push32(s_type(STORE, rv32i::Funct3_STORE::SW, 11, 6, 0));
    push32(i_type(OP_IMM, 10, F3A::ADD_SUB, 10, 4));
    push32(i_type(OP_IMM, 11, F3A::ADD_SUB, 11, 4));
    push32(i_type(OP_IMM, 12, F3A::ADD_SUB, 12, -1));
    push32(b_type(rv32i::Funct3_BRANCH::BNE, 12, 0, -32));
    p.push_back(0x0000);
    return p;
}
static std::vector<uint16_t> build_axpy_vector() {
    const uint32_t OP = rv32i::Opcode::OP, OP_IMM = rv32i::Opcode::OP_IMM;
    std::vector<uint16_t> p;
    auto push32 = [&](uint32_t w){ p.push_back(w & 0xFFFF); p.push_back(w >> 16); };
    push32(i_type(OP_IMM, 5, F3A::ADD_SUB, 0, A_VAL));
    push32(u_type(rv32i::Opcode::LUI, 10, X_BASE >> 12));
    push32(u_type(rv32i::Opcode::LUI, 11, Y_BASE >> 12));
    push32(i_type(OP_IMM, 12, F3A::ADD_SUB, 0, N_ELEMS & 0x7FF));
    push32(enc_vsetvli(13, 12, VTYPE_E32_M1));
    push32(enc_vec_mem(OPC_LOAD_FP, 1, 10));
    push32(enc_vec_mem(OPC_LOAD_FP, 2, 11));
    push32(enc_op_v(F6_VMUL, 1, 5, F3_OPMVX, 3));
    push32(enc_op_v(F6_VADD, 3, 2, F3_OPIVV, 4));
    push32(enc_vec_mem(OPC_STORE_FP, 4, 11));
    push32(i_type(OP_IMM, 14, 0b001, 13, 2));
    push32(r_type(OP, 10, F3A::ADD_SUB, 10, 14, 0));
    push32(r_type(OP, 11, F3A::ADD_SUB, 11, 14, 0));
    push32(r_type(OP, 12, F3A::ADD_SUB, 12, 13, rv32i::Funct7::ALT));
    push32(b_type(rv32i::Funct3_BRANCH::BNE, 12, 0, -40));
    p.push_back(0x0000);
    return p;
}

// ---- arnes ----
struct RunStats { long cycles; long commits; bool halted; };

SC_MODULE(AxpyTb) {
    SC_HAS_PROCESS(AxpyTb);
    ProcessorOOO& cpu;
    Memory&       mem;
    int           fails = 0;

    AxpyTb(sc_module_name n, ProcessorOOO& c, Memory& m) : sc_module(n), cpu(c), mem(m) {
        SC_THREAD(run);
    }

    void init_mem(const std::vector<uint16_t>& prog) {
        std::fill(mem.data.begin(), mem.data.end(), 0);
        std::memcpy(mem.data.data(), prog.data(), prog.size() * sizeof(uint16_t));
        for (int i = 0; i < N_ELEMS; i++) {
            uint32_t x = (uint32_t)i, y = Y0;
            std::memcpy(&mem.data[X_BASE + 4 * i], &x, 4);
            std::memcpy(&mem.data[Y_BASE + 4 * i], &y, 4);
        }
    }
    RunStats run_one(const std::vector<uint16_t>& prog) {
        cpu.reset_state();
        cpu.trace = false;
        init_mem(prog);
        bool ok = cpu.run_until_halt(500000);
        return RunStats{(long)cpu.cycle, (long)cpu.n_commit, ok};
    }
    int check_result(const char* tag) {
        int errs = 0;
        for (int i = 0; i < N_ELEMS; i++) {
            uint32_t exp = A_VAL * (uint32_t)i + Y0, got;
            std::memcpy(&got, &mem.data[Y_BASE + 4 * i], 4);
            if (got != exp) { if (errs < 4) std::printf("FAIL  %s: y[%d]=%u, esperado %u\n", tag, i, got, exp); errs++; }
        }
        if (errs == 0) std::printf("OK    %s: %d elementos verificados (y[i] = %u*i + %u)\n", tag, N_ELEMS, A_VAL, Y0);
        else std::printf("FAIL  %s: %d elementos incorrectos\n", tag, errs);
        return errs;
    }

    void run() {
        std::printf("================================================================\n");
        std::printf("  AXPY entero (y = %u*x + y), n=%d, sobre el RISC-V SoC (TLM)\n", A_VAL, N_ELEMS);
        std::printf("================================================================\n\n");

        RunStats s1 = run_one(build_axpy_scalar());
        if (!s1.halted) { std::printf("FAIL  escalar: no termino\n"); fails++; sc_stop(); return; }
        fails += check_result("escalar  ");

        RunStats s2 = run_one(build_axpy_vector());
        if (!s2.halted) { std::printf("FAIL  vectorial: no termino\n"); fails++; sc_stop(); return; }
        fails += check_result("vectorial");

        double ipc1 = (double)s1.commits / s1.cycles, ipc2 = (double)s2.commits / s2.cycles;
        std::printf("\n%-11s %10s %10s %8s %12s\n", "version", "ciclos", "commits", "IPC", "ciclos/elem");
        std::printf("%-11s %10ld %10ld %8.3f %12.2f\n", "escalar",  s1.cycles, s1.commits, ipc1, (double)s1.cycles / N_ELEMS);
        std::printf("%-11s %10ld %10ld %8.3f %12.2f\n", "vectorial", s2.cycles, s2.commits, ipc2, (double)s2.cycles / N_ELEMS);
        std::printf("\nreduccion de instrucciones: %.2fx  (%ld -> %ld)\n",
                    (double)s1.commits / s2.commits, s1.commits, s2.commits);
        std::printf("speedup vectorial en ciclos: %.2fx  (%ld -> %ld)\n",
                    (double)s1.cycles / s2.cycles, s1.cycles, s2.cycles);
        if (fails == 0) std::printf("\nTodos los checks pasaron.\n");
        else            std::printf("\n%d check(s) fallaron.\n", fails);
        sc_stop();
    }
};

int sc_main(int, char*[]) {
    Memory       memory("memory");
    Uart         uart("uart");
    Bus          bus("bus");
    ProcessorOOO cpu("cpu_ooo");
    VectorUnit   vu("vector_unit");
    AxpyTb       tb("axpy_tb", cpu, memory);

    cpu.init_socket.bind(bus.cpu_target);
    vu.init_socket.bind(bus.vector_target);
    bus.mem_initiator.bind(memory.socket);
    bus.uart_initiator.bind(uart.socket);

    sc_start();
    return tb.fails == 0 ? 0 : 1;
}
