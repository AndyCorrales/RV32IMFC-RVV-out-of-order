#ifndef PROCESSOR_H
#define PROCESSOR_H

#include <systemc.h>
#include <tlm.h>
#include <tlm_utils/simple_initiator_socket.h>
#include <array>
#include <cstring>
#include <iostream>
#include "rv32c_defs.h"
#include <vector>  
#include "rv32i_defs.h"
#include "immediates.h"
#include "fp_ops.h"
#include "rv32c_defs.h"

// CPU RV32I monociclo. INITIATOR puro hacia el Bus: toda direccion que
// maneja (fetch, load, store) es GLOBAL dentro del mapa de memoria.
SC_MODULE(Processor) {
    tlm_utils::simple_initiator_socket<Processor, 32> init_socket;

    std::array<uint32_t, 32> regs{};
    std::array<float, 32> fregs{}; // banco de registros f0-f31 (extension F, simple precision)
    uint32_t pc = 0;
    bool halted = false;
    sc_event finished;
// Contadores para IPC y CSRs rdcycle/rdinstret.
// cycle_count: incrementa una vez por instruccion (modelo funcional
//   monociclo — cada instruccion toma 1 ciclo).
// instr_count: incrementa solo para instrucciones que llegan a commit
//   (excluye halt por instruccion nula).
    uint64_t cycle_count  = 0;
    uint64_t instr_count  = 0;

    // --- Caracterizacion: histograma de instrucciones por clase ---
    enum InsnClass { IC_ALU, IC_ALUI, IC_LOAD, IC_STORE, IC_BRANCH, IC_JUMP,
                     IC_MUL, IC_DIV, IC_FP, IC_SYSTEM, IC_OTHER, IC_COUNT };
    uint64_t insn_hist[IC_COUNT] = {0};

    void classify_instr(uint32_t opcode, uint32_t f3, uint32_t f7) {
        switch (opcode) {
            case 0x33: insn_hist[(f7 == 0x01) ? ((f3 < 4) ? IC_MUL : IC_DIV)
                                              : IC_ALU]++; break;   // OP
            case 0x13: insn_hist[IC_ALUI]++;   break;               // OP-IMM
            case 0x37: case 0x17: insn_hist[IC_ALUI]++; break;      // LUI/AUIPC
            case 0x03: insn_hist[IC_LOAD]++;   break;               // LOAD
            case 0x23: insn_hist[IC_STORE]++;  break;               // STORE
            case 0x07: insn_hist[IC_LOAD]++;   break;               // LOAD-FP
            case 0x27: insn_hist[IC_STORE]++;  break;               // STORE-FP
            case 0x63: insn_hist[IC_BRANCH]++; break;               // BRANCH
            case 0x6F: case 0x67: insn_hist[IC_JUMP]++; break;      // JAL/JALR
            case 0x53: case 0x43: case 0x47:
            case 0x4B: case 0x4F: insn_hist[IC_FP]++; break;        // OP-FP/FMADD
            case 0x73: insn_hist[IC_SYSTEM]++; break;               // SYSTEM
            default:   insn_hist[IC_OTHER]++;  break;
        }
    }

    void dump_hist() const {
        const char* n[IC_COUNT] = {"ALU(reg)","ALU(imm)","LOAD","STORE",
            "BRANCH","JUMP","MUL","DIV","FP","SYSTEM","OTHER"};
        uint64_t tot = 0; for (int i = 0; i < IC_COUNT; i++) tot += insn_hist[i];
        std::cerr << "\n[Histograma de instrucciones] total=" << tot << "\n";
        for (int i = 0; i < IC_COUNT; i++)
            std::cerr << "  " << n[i] << " : " << insn_hist[i]
                      << " (" << (tot ? 100.0 * insn_hist[i] / tot : 0.0) << "%)\n";
    }

    SC_CTOR(Processor) : init_socket("init_socket") {
        SC_THREAD(run);
    }

    // Lectura/escritura generica de 'len' bytes en direccion GLOBAL via Bus.
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
            std::cerr << "[Processor] Error de bus en direccion 0x" << std::hex << addr << std::dec << std::endl;
            halted = true;
        }
    }

    // Extension C: toda instruccion se fetchea primero como halfword (16
    // bits), sin asumir alineacion a 4 bytes -- una instruccion normal de
    // 32 bits puede empezar en cualquier direccion PAR si la precede una
    // comprimida de 16 bits.
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

    void write_reg(uint32_t idx, uint32_t value) {
        if (idx != 0) regs[idx] = value;
    }

    void run() {
        static const int RING = 16;
        uint32_t ring_pc[RING] = {0};
        uint32_t ring_in[RING] = {0};
        int ring_i = 0;
        while (!halted) {
            cycle_count++;          // un ciclo por iteracion del loop principal
            uint16_t half = fetch16(pc);

            // Halfword nulo (memoria sin programa): fin de ejecucion. Se
            // chequea sobre el halfword crudo, no sobre la instruccion ya
            // expandida -- rv32c::expand() nunca devuelve 0 para una
            // comprimida invalida (devuelve rv32c::ILLEGAL), asi que este
            // chequeo sigue significando exclusivamente "no hay mas
            // programa", nunca "instruccion comprimida no reconocida".
            if (half == 0) {
                std::cerr << "\n[HALT-ZERO] el PC salto a memoria en ceros: PC=0x"
                          << std::hex << pc << std::dec << "\n";
                std::cerr << "Ultimas instrucciones ejecutadas (PC : instr):\n";
                for (int k = 0; k < RING; k++) {
                    int idx = (ring_i + k) % RING;
                    if (ring_pc[idx] || ring_in[idx])
                        std::cerr << "  0x" << std::hex << ring_pc[idx]
                                  << " : 0x" << ring_in[idx] << std::dec << "\n";
                }
                std::cerr << "ra(x1)=0x" << std::hex << regs[1]
                          << "  sp(x2)=0x" << regs[2]
                          << "  gp(x3)=0x" << regs[3] << std::dec << "\n";
                halted = true;
                break;
            }

            uint32_t instr;
            uint32_t instr_size;
            if ((half & 0x3) != 0x3) {
                // Extension C: instruccion comprimida de 16 bits. Se
                // expande a la instruccion de 32 bits equivalente y de
                // ahi en adelante corre exactamente el mismo decoder de
                // siempre -- el resto de esta funcion no sabe ni le
                // importa si la instruccion original era comprimida.
                instr = rv32c::expand(half);
                instr_size = 2;
            } else {
                uint16_t half_hi = fetch16(pc + 2);
                instr = (static_cast<uint32_t>(half_hi) << 16) | half;
                instr_size = 4;
            }

            uint32_t opcode = rv32i::opcode(instr);
            uint32_t rd     = rv32i::rd(instr);
            uint32_t f3     = rv32i::funct3(instr);
            uint32_t rs1i   = rv32i::rs1(instr);
            uint32_t rs2i   = rv32i::rs2(instr);
            uint32_t f7     = rv32i::funct7(instr);

            uint32_t next_pc = pc + instr_size;

            int32_t r1 = static_cast<int32_t>(regs[rs1i]);
            int32_t r2 = static_cast<int32_t>(regs[rs2i]);

            switch (opcode) {
                case rv32i::Opcode::OP: {
                    uint32_t result = 0;

                    if (f7 == rv32i::Funct7::MULDIV) {
                        // Extension M: mismo opcode OP, distinguida solo por
                        // funct7. Los productos de 64 bits se calculan en
                        // int64_t/uint64_t y se toma la mitad alta o baja
                        // segun corresponda.
                        int64_t  r1_64  = static_cast<int64_t>(r1);
                        int64_t  r2_64  = static_cast<int64_t>(r2);
                        uint64_t u1_64  = static_cast<uint64_t>(regs[rs1i]);
                        uint64_t u2_64  = static_cast<uint64_t>(regs[rs2i]);

                        switch (f3) {
                            case rv32i::Funct3_MULDIV::MUL:
                                result = static_cast<uint32_t>(regs[rs1i] * regs[rs2i]);
                                break;
                            case rv32i::Funct3_MULDIV::MULH:
                                result = static_cast<uint32_t>(static_cast<uint64_t>(r1_64 * r2_64) >> 32);
                                break;
                            case rv32i::Funct3_MULDIV::MULHSU:
                                result = static_cast<uint32_t>(static_cast<uint64_t>(r1_64 * static_cast<int64_t>(u2_64)) >> 32);
                                break;
                            case rv32i::Funct3_MULDIV::MULHU:
                                result = static_cast<uint32_t>((u1_64 * u2_64) >> 32);
                                break;
                            case rv32i::Funct3_MULDIV::DIV:
                                if (r2 == 0) {
                                    result = 0xFFFFFFFF; // division por cero: -1
                                } else if (regs[rs1i] == 0x80000000 && r2 == -1) {
                                    result = 0x80000000; // overflow: INT32_MIN / -1
                                } else {
                                    result = static_cast<uint32_t>(r1 / r2);
                                }
                                break;
                            case rv32i::Funct3_MULDIV::DIVU:
                                result = (regs[rs2i] == 0) ? 0xFFFFFFFF : (regs[rs1i] / regs[rs2i]);
                                break;
                            case rv32i::Funct3_MULDIV::REM:
                                if (r2 == 0) {
                                    result = regs[rs1i]; // division por cero: resto = dividendo
                                } else if (regs[rs1i] == 0x80000000 && r2 == -1) {
                                    result = 0; // overflow: resto = 0
                                } else {
                                    result = static_cast<uint32_t>(r1 % r2);
                                }
                                break;
                            case rv32i::Funct3_MULDIV::REMU:
                                result = (regs[rs2i] == 0) ? regs[rs1i] : (regs[rs1i] % regs[rs2i]);
                                break;
                        }
                    } else {
                        switch (f3) {
                            case rv32i::Funct3_ALU::ADD_SUB:
                                result = (f7 == rv32i::Funct7::ALT) ? (r1 - r2) : (r1 + r2);
                                break;
                            case rv32i::Funct3_ALU::SLL:
                                result = static_cast<uint32_t>(r1) << (r2 & 0x1F);
                                break;
                            case rv32i::Funct3_ALU::SLT:
                                result = (r1 < r2) ? 1 : 0;
                                break;
                            case rv32i::Funct3_ALU::SLTU:
                                result = (regs[rs1i] < regs[rs2i]) ? 1 : 0;
                                break;
                            case rv32i::Funct3_ALU::XOR:
                                result = regs[rs1i] ^ regs[rs2i];
                                break;
                            case rv32i::Funct3_ALU::SRL_SRA:
                                result = (f7 == rv32i::Funct7::ALT)
                                             ? static_cast<uint32_t>(r1 >> (r2 & 0x1F))
                                             : (regs[rs1i] >> (regs[rs2i] & 0x1F));
                                break;
                            case rv32i::Funct3_ALU::OR:
                                result = regs[rs1i] | regs[rs2i];
                                break;
                            case rv32i::Funct3_ALU::AND:
                                result = regs[rs1i] & regs[rs2i];
                                break;
                        }
                    }

                    write_reg(rd, result);
                    break;
                }

                case rv32i::Opcode::OP_IMM: {
                    int32_t imm = rv32i::get_imm_I(instr);
                    uint32_t shamt = rv32i::get_shamt(instr);
                    uint32_t result = 0;
                    switch (f3) {
                        case rv32i::Funct3_ALU::ADD_SUB:
                            result = r1 + imm;
                            break;
                        case rv32i::Funct3_ALU::SLL:
                            result = static_cast<uint32_t>(r1) << shamt;
                            break;
                        case rv32i::Funct3_ALU::SLT:
                            result = (r1 < imm) ? 1 : 0;
                            break;
                        case rv32i::Funct3_ALU::SLTU:
                            result = (regs[rs1i] < static_cast<uint32_t>(imm)) ? 1 : 0;
                            break;
                        case rv32i::Funct3_ALU::XOR:
                            result = regs[rs1i] ^ static_cast<uint32_t>(imm);
                            break;
                        case rv32i::Funct3_ALU::SRL_SRA:
                            result = (rv32i::funct7(instr) == rv32i::Funct7::ALT)
                                         ? static_cast<uint32_t>(r1 >> shamt)
                                         : (regs[rs1i] >> shamt);
                            break;
                        case rv32i::Funct3_ALU::OR:
                            result = regs[rs1i] | static_cast<uint32_t>(imm);
                            break;
                        case rv32i::Funct3_ALU::AND:
                            result = regs[rs1i] & static_cast<uint32_t>(imm);
                            break;
                    }
                    write_reg(rd, result);
                    break;
                }

                case rv32i::Opcode::LOAD: {
                    int32_t imm = rv32i::get_imm_I(instr);
                    uint32_t addr = regs[rs1i] + imm;
                    uint32_t result = 0;
                    switch (f3) {
                        case rv32i::Funct3_LOAD::LB:
                            result = static_cast<uint32_t>(static_cast<int32_t>(static_cast<int8_t>(load(addr, 1))));
                            break;
                        case rv32i::Funct3_LOAD::LH:
                            result = static_cast<uint32_t>(static_cast<int32_t>(static_cast<int16_t>(load(addr, 2))));
                            break;
                        case rv32i::Funct3_LOAD::LW:
                            result = load(addr, 4);
                            break;
                        case rv32i::Funct3_LOAD::LBU:
                            result = load(addr, 1) & 0xFF;
                            break;
                        case rv32i::Funct3_LOAD::LHU:
                            result = load(addr, 2) & 0xFFFF;
                            break;
                    }
                    write_reg(rd, result);
                    break;
                }

                case rv32i::Opcode::STORE: {
                    int32_t imm = rv32i::get_imm_S(instr);
                    uint32_t addr = regs[rs1i] + imm;
                    switch (f3) {
                        case rv32i::Funct3_STORE::SB: store(addr, regs[rs2i], 1); break;
                        case rv32i::Funct3_STORE::SH: store(addr, regs[rs2i], 2); break;
                        case rv32i::Funct3_STORE::SW: store(addr, regs[rs2i], 4); break;
                    }
                    break;
                }

                case rv32i::Opcode::BRANCH: {
                    int32_t imm = rv32i::get_imm_B(instr);
                    bool taken = false;
                    switch (f3) {
                        case rv32i::Funct3_BRANCH::BEQ:  taken = (r1 == r2); break;
                        case rv32i::Funct3_BRANCH::BNE:  taken = (r1 != r2); break;
                        case rv32i::Funct3_BRANCH::BLT:  taken = (r1 < r2); break;
                        case rv32i::Funct3_BRANCH::BGE:  taken = (r1 >= r2); break;
                        case rv32i::Funct3_BRANCH::BLTU: taken = (regs[rs1i] < regs[rs2i]); break;
                        case rv32i::Funct3_BRANCH::BGEU: taken = (regs[rs1i] >= regs[rs2i]); break;
                    }
                    if (taken) next_pc = pc + imm;
                    break;
                }

                case rv32i::Opcode::JAL: {
                    int32_t imm = rv32i::get_imm_J(instr);
                    write_reg(rd, pc + instr_size);
                    next_pc = pc + imm;
                    break;
                }

                case rv32i::Opcode::JALR: {
                    int32_t imm = rv32i::get_imm_I(instr);
                    uint32_t link = pc + instr_size;
                    next_pc = (regs[rs1i] + imm) & ~static_cast<uint32_t>(1);
                    write_reg(rd, link);
                    break;
                }

                case rv32i::Opcode::LUI:
                    write_reg(rd, static_cast<uint32_t>(rv32i::get_imm_U(instr)));
                    break;

                case rv32i::Opcode::AUIPC:
                    write_reg(rd, pc + static_cast<uint32_t>(rv32i::get_imm_U(instr)));
                    break;

                case rv32i::Opcode::LOAD_FP: {
                    // Unico ancho soportado: FLW (f3 == Funct3_FP_MEM::W).
                    int32_t imm = rv32i::get_imm_I(instr);
                    uint32_t addr = regs[rs1i] + imm;
                    fregs[rd] = rv32i::bits_to_float(load(addr, 4));
                    break;
                }

                case rv32i::Opcode::STORE_FP: {
                    // Unico ancho soportado: FSW (f3 == Funct3_FP_MEM::W).
                    int32_t imm = rv32i::get_imm_S(instr);
                    uint32_t addr = regs[rs1i] + imm;
                    store(addr, rv32i::float_to_bits(fregs[rs2i]), 4);
                    break;
                }

                // FMADD/FMSUB/FNMSUB/FNMADD: R4-type, un tercer operando
                // fuente (rs3) ademas de rs1/rs2. std::fma calcula
                // (a*b)+c con un unico redondeo final (fused), igual que
                // exige el ISA (a diferencia de hacer a*b y despues +c
                // por separado, que redondearia dos veces).
                case rv32i::Opcode::FMADD: {
                    uint32_t r3 = rv32i::rs3(instr);
                    fregs[rd] = std::fma(fregs[rs1i], fregs[rs2i], fregs[r3]);
                    break;
                }
                case rv32i::Opcode::FMSUB: {
                    uint32_t r3 = rv32i::rs3(instr);
                    fregs[rd] = std::fma(fregs[rs1i], fregs[rs2i], -fregs[r3]);
                    break;
                }
                case rv32i::Opcode::FNMSUB: {
                    uint32_t r3 = rv32i::rs3(instr);
                    fregs[rd] = std::fma(-fregs[rs1i], fregs[rs2i], fregs[r3]);
                    break;
                }
                case rv32i::Opcode::FNMADD: {
                    uint32_t r3 = rv32i::rs3(instr);
                    fregs[rd] = std::fma(-fregs[rs1i], fregs[rs2i], -fregs[r3]);
                    break;
                }

                case rv32i::Opcode::OP_FP: {
                    switch (f7) {
                        case rv32i::Funct7_FP::FADD_S:
                            fregs[rd] = fregs[rs1i] + fregs[rs2i];
                            break;
                        case rv32i::Funct7_FP::FSUB_S:
                            fregs[rd] = fregs[rs1i] - fregs[rs2i];
                            break;
                        case rv32i::Funct7_FP::FMUL_S:
                            fregs[rd] = fregs[rs1i] * fregs[rs2i];
                            break;
                        case rv32i::Funct7_FP::FDIV_S:
                            fregs[rd] = fregs[rs1i] / fregs[rs2i];
                            break;
                        case rv32i::Funct7_FP::FSQRT_S:
                            fregs[rd] = std::sqrt(fregs[rs1i]);
                            break;

                        case rv32i::Funct7_FP::FSGNJ_S: {
                            uint32_t a = rv32i::float_to_bits(fregs[rs1i]);
                            uint32_t b = rv32i::float_to_bits(fregs[rs2i]);
                            uint32_t sign = 0;
                            switch (f3) {
                                case rv32i::Funct3_FSGNJ::FSGNJ:  sign = b & 0x80000000; break;
                                case rv32i::Funct3_FSGNJ::FSGNJN: sign = (~b) & 0x80000000; break;
                                case rv32i::Funct3_FSGNJ::FSGNJX: sign = (a ^ b) & 0x80000000; break;
                            }
                            fregs[rd] = rv32i::bits_to_float((a & 0x7FFFFFFF) | sign);
                            break;
                        }

                        case rv32i::Funct7_FP::FMINMAX_S:
                            if (f3 == rv32i::Funct3_FMINMAX::FMIN)
                                fregs[rd] = std::fmin(fregs[rs1i], fregs[rs2i]);
                            else
                                fregs[rd] = std::fmax(fregs[rs1i], fregs[rs2i]);
                            break;

                        case rv32i::Funct7_FP::FCMP_S: {
                            // Los comparadores de C++ con NaN siempre dan
                            // false (IEEE-754), que es exactamente el
                            // resultado que exige el ISA para FEQ/FLT/FLE
                            // con NaN, sin necesitar codigo especial.
                            uint32_t result = 0;
                            switch (f3) {
                                case rv32i::Funct3_FCMP::FLE: result = (fregs[rs1i] <= fregs[rs2i]) ? 1 : 0; break;
                                case rv32i::Funct3_FCMP::FLT: result = (fregs[rs1i] <  fregs[rs2i]) ? 1 : 0; break;
                                case rv32i::Funct3_FCMP::FEQ: result = (fregs[rs1i] == fregs[rs2i]) ? 1 : 0; break;
                            }
                            write_reg(rd, result);
                            break;
                        }

                        case rv32i::Funct7_FP::FCVT_W_S: {
                            uint32_t r2 = rv32i::rs2(instr); // reusado como selector W/WU
                            uint32_t result = (r2 == rv32i::Rs2_FCVT::WU)
                                                  ? rv32i::fcvt_wu_s(fregs[rs1i])
                                                  : static_cast<uint32_t>(rv32i::fcvt_w_s(fregs[rs1i]));
                            write_reg(rd, result);
                            break;
                        }

                        case rv32i::Funct7_FP::FCVT_S_W: {
                            uint32_t r2 = rv32i::rs2(instr); // reusado como selector W/WU
                            fregs[rd] = (r2 == rv32i::Rs2_FCVT::WU)
                                            ? static_cast<float>(regs[rs1i])
                                            : static_cast<float>(static_cast<int32_t>(regs[rs1i]));
                            break;
                        }

                        case rv32i::Funct7_FP::FMV_X_W_FCLASS_S:
                            if (f3 == rv32i::Funct3_FMV_FCLASS::FCLASS_S)
                                write_reg(rd, rv32i::fclass_s(fregs[rs1i]));
                            else
                                write_reg(rd, rv32i::float_to_bits(fregs[rs1i]));
                            break;

                        case rv32i::Funct7_FP::FMV_W_X:
                            fregs[rd] = rv32i::bits_to_float(regs[rs1i]);
                            break;
                    }
                    break;
                }

                case 0x73: { // SYSTEM: ECALL, EBREAK, CSRs (opcode no definido en rv32i_defs.h)
                    uint32_t sys_f3   = (instr >> 12) & 0x7;
                    uint32_t csr_addr = (instr >> 20) & 0xFFF;
                    uint32_t sys_rd   = (instr >>  7) & 0x1F;

                    if (sys_f3 == 0b000) {
                        uint32_t funct12 = (instr >> 20) & 0xFFF;
                        if (funct12 == 1) {
                            // EBREAK
                            std::cerr << "[EBREAK] en PC=0x" << std::hex << pc << std::dec << "\n";
                            halted = true;
                            break;
                        }
                        // ECALL — ABI Linux rv32, syscall en a7 (x17)
                        uint32_t syscall_n = regs[17];
                        switch (syscall_n) {
                            case 64: {
                                // SYS_WRITE(fd, buf, count)
                                uint32_t fd    = regs[10];
                                uint32_t baddr = regs[11];
                                uint32_t count = regs[12];
                                if (fd == 1 || fd == 2) {
                                    std::vector<uint8_t> buf(count);
                                    tlm::tlm_generic_payload t;
                                    sc_time delay = SC_ZERO_TIME;
                                    t.set_command(tlm::TLM_READ_COMMAND);
                                    t.set_address(baddr);
                                    t.set_data_ptr(buf.data());
                                    t.set_data_length(count);
                                    t.set_streaming_width(count);
                                    t.set_byte_enable_ptr(nullptr);
                                    t.set_dmi_allowed(false);
                                    t.set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);
                                    init_socket->b_transport(t, delay);
                                    if (t.get_response_status() == tlm::TLM_OK_RESPONSE) {
                                        std::cout.write(reinterpret_cast<char*>(buf.data()), count);
                                        std::cout.flush();
                                        regs[10] = count;
                                    } else {
                                        regs[10] = static_cast<uint32_t>(-1);
                                    }
                                } else {
                                    regs[10] = static_cast<uint32_t>(-1);
                                }
                                break;
                            }
                            case 93:
                            case 94: {
                                // SYS_EXIT / SYS_EXIT_GROUP
                                uint32_t code = regs[10];
                                std::cout << "\n[ECALL exit(" << code << ")]\n";
                                std::cout << "──────────────────────────────────────────\n";
                                std::cout << "  Ciclos       : " << cycle_count << "\n";
                                std::cout << "  Instrucciones: " << instr_count << "\n";
                                if (cycle_count > 0) {
                                    uint64_t ipc_e = instr_count / cycle_count;
                                    uint64_t ipc_f = (instr_count * 1000 / cycle_count) % 1000;
                                    std::cout << "  IPC          : " << ipc_e << "." << ipc_f << "\n";
                                    std::cout << "  Ref Ibex     : 0.680\n";
                                }
                                std::cout << "──────────────────────────────────────────\n";
                                halted = true;
                                sc_stop();
                                break;
                            }
                            default:
                                regs[10] = static_cast<uint32_t>(-1);
                                break;
                        }
                    } else {
                        // CSRs: rdcycle, rdinstret, rdtime
                        uint32_t csr_val = 0;
                        switch (csr_addr) {
                            case 0xC00: csr_val = static_cast<uint32_t>(cycle_count);       break;
                            case 0xC80: csr_val = static_cast<uint32_t>(cycle_count >> 32); break;
                            case 0xC01: csr_val = static_cast<uint32_t>(cycle_count);       break;
                            case 0xC81: csr_val = static_cast<uint32_t>(cycle_count >> 32); break;
                            case 0xC02: csr_val = static_cast<uint32_t>(instr_count);       break;
                            case 0xC82: csr_val = static_cast<uint32_t>(instr_count >> 32); break;
                            default:    csr_val = 0; break;
                        }
                        if (sys_rd != 0) regs[sys_rd] = csr_val;
                    }
                    break;
                } // case SYSTEM

                default:
                    std::cerr << "[Processor] Opcode desconocido 0x" << std::hex << opcode
                              << " en PC=0x" << pc << std::dec << std::endl;
                    halted = true;
                    break;
            }

            ring_pc[ring_i] = pc; ring_in[ring_i] = instr; ring_i = (ring_i + 1) % RING;
            classify_instr(opcode, f3, f7);   // caracterizacion
            instr_count++;    // instruccion completada correctamente
            pc = next_pc;
        }
        dump_hist();
        finished.notify();
    }

    void dump_regs() const {
        for (int i = 0; i < 32; ++i) {
            std::cout << "x" << i << "=0x" << std::hex << regs[i] << std::dec;
            std::cout << ((i == 31) ? "\n" : " ");
        }
    }

    void dump_fregs() const {
        for (int i = 0; i < 32; ++i) {
            std::cout << "f" << i << "=" << fregs[i]
                       << "(0x" << std::hex << rv32i::float_to_bits(fregs[i]) << std::dec << ")";
            std::cout << ((i == 31) ? "\n" : " ");
        }
    }
};

#endif // PROCESSOR_H
