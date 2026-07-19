#include <systemc.h>
#include <tlm.h>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <vector>
#include <fstream>
#include <cstdint>

#include "rv32i_defs.h"
#include "memory_map.h"
#include "memory.h"
#include "bus.h"
#include "processor.h"
#include "vector_unit.h"

// ---------------------------------------------------------------------
// Codificadores de instruccion RV32I minimos, solo para construir el
// programa de prueba de este main.cpp (no forman parte del modelo).
// ---------------------------------------------------------------------
namespace asmenc {
using namespace rv32i;

uint32_t r_type(uint32_t opcode, uint32_t rd, uint32_t f3, uint32_t rs1, uint32_t rs2, uint32_t f7) {
    return (f7 << 25) | (rs2 << 20) | (rs1 << 15) | (f3 << 12) | (rd << 7) | opcode;
}
uint32_t i_type(uint32_t opcode, uint32_t rd, uint32_t f3, uint32_t rs1, int32_t imm) {
    return ((static_cast<uint32_t>(imm) & 0xFFF) << 20) | (rs1 << 15) | (f3 << 12) | (rd << 7) | opcode;
}
// Generico en el opcode (STORE y STORE_FP comparten formato S-type).
uint32_t s_type(uint32_t opcode, uint32_t f3, uint32_t rs1, uint32_t rs2, int32_t imm) {
    uint32_t u = static_cast<uint32_t>(imm) & 0xFFF;
    return ((u >> 5) << 25) | (rs2 << 20) | (rs1 << 15) | (f3 << 12) | ((u & 0x1F) << 7) | opcode;
}
uint32_t b_type(uint32_t f3, uint32_t rs1, uint32_t rs2, int32_t imm) {
    uint32_t u = static_cast<uint32_t>(imm) & 0x1FFF;
    return (((u >> 12) & 0x1) << 31) | (((u >> 5) & 0x3F) << 25) | (rs2 << 20) | (rs1 << 15) |
           (f3 << 12) | (((u >> 1) & 0xF) << 8) | (((u >> 11) & 0x1) << 7) | Opcode::BRANCH;
}
uint32_t u_type(uint32_t opcode, uint32_t rd, uint32_t imm_upper20) {
    return ((imm_upper20 << 12) & 0xFFFFF000) | (rd << 7) | opcode;
}
uint32_t j_type(uint32_t rd, int32_t imm) {
    uint32_t u = static_cast<uint32_t>(imm) & 0x1FFFFF;
    return (((u >> 20) & 0x1) << 31) | (((u >> 1) & 0x3FF) << 21) | (((u >> 11) & 0x1) << 20) |
           (((u >> 12) & 0xFF) << 12) | (rd << 7) | Opcode::JAL;
}
// R4-type (FMADD/FMSUB/FNMSUB/FNMADD): agrega rs3 y fmt (precision;
// 00=simple, la unica que soporta este modelo) en vez de funct7.
uint32_t r4_type(uint32_t opcode, uint32_t rd, uint32_t f3, uint32_t rs1, uint32_t rs2, uint32_t rs3) {
    constexpr uint32_t FMT_SINGLE = 0b00;
    return (rs3 << 27) | (FMT_SINGLE << 25) | (rs2 << 20) | (rs1 << 15) | (f3 << 12) | (rd << 7) | opcode;
}

// Pseudo-instrucciones de conveniencia.
uint32_t ADDI(uint32_t rd, uint32_t rs1, int32_t imm) { return i_type(Opcode::OP_IMM, rd, Funct3_ALU::ADD_SUB, rs1, imm); }
uint32_t ADD(uint32_t rd, uint32_t rs1, uint32_t rs2)  { return r_type(Opcode::OP, rd, Funct3_ALU::ADD_SUB, rs1, rs2, Funct7::NORMAL); }
uint32_t SUB(uint32_t rd, uint32_t rs1, uint32_t rs2)  { return r_type(Opcode::OP, rd, Funct3_ALU::ADD_SUB, rs1, rs2, Funct7::ALT); }
uint32_t AND(uint32_t rd, uint32_t rs1, uint32_t rs2)  { return r_type(Opcode::OP, rd, Funct3_ALU::AND, rs1, rs2, Funct7::NORMAL); }
uint32_t OR(uint32_t rd, uint32_t rs1, uint32_t rs2)   { return r_type(Opcode::OP, rd, Funct3_ALU::OR, rs1, rs2, Funct7::NORMAL); }
uint32_t XOR(uint32_t rd, uint32_t rs1, uint32_t rs2)  { return r_type(Opcode::OP, rd, Funct3_ALU::XOR, rs1, rs2, Funct7::NORMAL); }
uint32_t SLT(uint32_t rd, uint32_t rs1, uint32_t rs2)  { return r_type(Opcode::OP, rd, Funct3_ALU::SLT, rs1, rs2, Funct7::NORMAL); }
uint32_t SLTU(uint32_t rd, uint32_t rs1, uint32_t rs2) { return r_type(Opcode::OP, rd, Funct3_ALU::SLTU, rs1, rs2, Funct7::NORMAL); }
uint32_t SLL(uint32_t rd, uint32_t rs1, uint32_t rs2)  { return r_type(Opcode::OP, rd, Funct3_ALU::SLL, rs1, rs2, Funct7::NORMAL); }
uint32_t SRL(uint32_t rd, uint32_t rs1, uint32_t rs2)  { return r_type(Opcode::OP, rd, Funct3_ALU::SRL_SRA, rs1, rs2, Funct7::NORMAL); }
uint32_t SRA(uint32_t rd, uint32_t rs1, uint32_t rs2)  { return r_type(Opcode::OP, rd, Funct3_ALU::SRL_SRA, rs1, rs2, Funct7::ALT); }

// Extension M (mismo opcode OP, funct7=MULDIV).
uint32_t MUL(uint32_t rd, uint32_t rs1, uint32_t rs2)    { return r_type(Opcode::OP, rd, Funct3_MULDIV::MUL, rs1, rs2, Funct7::MULDIV); }
uint32_t MULH(uint32_t rd, uint32_t rs1, uint32_t rs2)   { return r_type(Opcode::OP, rd, Funct3_MULDIV::MULH, rs1, rs2, Funct7::MULDIV); }
uint32_t MULHSU(uint32_t rd, uint32_t rs1, uint32_t rs2) { return r_type(Opcode::OP, rd, Funct3_MULDIV::MULHSU, rs1, rs2, Funct7::MULDIV); }
uint32_t MULHU(uint32_t rd, uint32_t rs1, uint32_t rs2)  { return r_type(Opcode::OP, rd, Funct3_MULDIV::MULHU, rs1, rs2, Funct7::MULDIV); }
uint32_t DIV(uint32_t rd, uint32_t rs1, uint32_t rs2)    { return r_type(Opcode::OP, rd, Funct3_MULDIV::DIV, rs1, rs2, Funct7::MULDIV); }
uint32_t DIVU(uint32_t rd, uint32_t rs1, uint32_t rs2)   { return r_type(Opcode::OP, rd, Funct3_MULDIV::DIVU, rs1, rs2, Funct7::MULDIV); }
uint32_t REM(uint32_t rd, uint32_t rs1, uint32_t rs2)    { return r_type(Opcode::OP, rd, Funct3_MULDIV::REM, rs1, rs2, Funct7::MULDIV); }
uint32_t REMU(uint32_t rd, uint32_t rs1, uint32_t rs2)   { return r_type(Opcode::OP, rd, Funct3_MULDIV::REMU, rs1, rs2, Funct7::MULDIV); }

uint32_t LUI(uint32_t rd, uint32_t imm_upper20)        { return u_type(Opcode::LUI, rd, imm_upper20); }
uint32_t AUIPC(uint32_t rd, uint32_t imm_upper20)      { return u_type(Opcode::AUIPC, rd, imm_upper20); }
uint32_t LW(uint32_t rd, uint32_t rs1, int32_t imm)    { return i_type(Opcode::LOAD, rd, Funct3_LOAD::LW, rs1, imm); }
uint32_t LBU(uint32_t rd, uint32_t rs1, int32_t imm)   { return i_type(Opcode::LOAD, rd, Funct3_LOAD::LBU, rs1, imm); }
uint32_t LHU(uint32_t rd, uint32_t rs1, int32_t imm)   { return i_type(Opcode::LOAD, rd, Funct3_LOAD::LHU, rs1, imm); }
uint32_t SW(uint32_t rs1, uint32_t rs2, int32_t imm)   { return s_type(Opcode::STORE, Funct3_STORE::SW, rs1, rs2, imm); }
uint32_t SB(uint32_t rs1, uint32_t rs2, int32_t imm)   { return s_type(Opcode::STORE, Funct3_STORE::SB, rs1, rs2, imm); }
uint32_t SH(uint32_t rs1, uint32_t rs2, int32_t imm)   { return s_type(Opcode::STORE, Funct3_STORE::SH, rs1, rs2, imm); }
uint32_t BEQ(uint32_t rs1, uint32_t rs2, int32_t imm)  { return b_type(Funct3_BRANCH::BEQ, rs1, rs2, imm); }
uint32_t BNE(uint32_t rs1, uint32_t rs2, int32_t imm)  { return b_type(Funct3_BRANCH::BNE, rs1, rs2, imm); }
uint32_t BLT(uint32_t rs1, uint32_t rs2, int32_t imm)  { return b_type(Funct3_BRANCH::BLT, rs1, rs2, imm); }
uint32_t BGE(uint32_t rs1, uint32_t rs2, int32_t imm)  { return b_type(Funct3_BRANCH::BGE, rs1, rs2, imm); }
uint32_t JAL(uint32_t rd, int32_t imm)                 { return j_type(rd, imm); }
uint32_t JALR(uint32_t rd, uint32_t rs1, int32_t imm)  { return i_type(Opcode::JALR, rd, Funct3_ALU::ADD_SUB, rs1, imm); }

// Extension F (simple precision). rm (funct3 de las 5 operaciones
// aritmeticas base) siempre 0b000 = RNE: este modelo no implementa
// frm/fflags, ver fp_ops.h.
constexpr uint32_t RM_RNE = 0b000;

uint32_t FLW(uint32_t rd, uint32_t rs1, int32_t imm)   { return i_type(Opcode::LOAD_FP, rd, Funct3_FP_MEM::W, rs1, imm); }
uint32_t FSW(uint32_t rs1, uint32_t rs2, int32_t imm)  { return s_type(Opcode::STORE_FP, Funct3_FP_MEM::W, rs1, rs2, imm); }

uint32_t FADD_S(uint32_t rd, uint32_t rs1, uint32_t rs2)  { return r_type(Opcode::OP_FP, rd, RM_RNE, rs1, rs2, Funct7_FP::FADD_S); }
uint32_t FSUB_S(uint32_t rd, uint32_t rs1, uint32_t rs2)  { return r_type(Opcode::OP_FP, rd, RM_RNE, rs1, rs2, Funct7_FP::FSUB_S); }
uint32_t FMUL_S(uint32_t rd, uint32_t rs1, uint32_t rs2)  { return r_type(Opcode::OP_FP, rd, RM_RNE, rs1, rs2, Funct7_FP::FMUL_S); }
uint32_t FDIV_S(uint32_t rd, uint32_t rs1, uint32_t rs2)  { return r_type(Opcode::OP_FP, rd, RM_RNE, rs1, rs2, Funct7_FP::FDIV_S); }
uint32_t FSQRT_S(uint32_t rd, uint32_t rs1)               { return r_type(Opcode::OP_FP, rd, RM_RNE, rs1, 0, Funct7_FP::FSQRT_S); }

uint32_t FSGNJ_S(uint32_t rd, uint32_t rs1, uint32_t rs2)  { return r_type(Opcode::OP_FP, rd, Funct3_FSGNJ::FSGNJ, rs1, rs2, Funct7_FP::FSGNJ_S); }
uint32_t FSGNJN_S(uint32_t rd, uint32_t rs1, uint32_t rs2) { return r_type(Opcode::OP_FP, rd, Funct3_FSGNJ::FSGNJN, rs1, rs2, Funct7_FP::FSGNJ_S); }
uint32_t FSGNJX_S(uint32_t rd, uint32_t rs1, uint32_t rs2) { return r_type(Opcode::OP_FP, rd, Funct3_FSGNJ::FSGNJX, rs1, rs2, Funct7_FP::FSGNJ_S); }

uint32_t FMIN_S(uint32_t rd, uint32_t rs1, uint32_t rs2) { return r_type(Opcode::OP_FP, rd, Funct3_FMINMAX::FMIN, rs1, rs2, Funct7_FP::FMINMAX_S); }
uint32_t FMAX_S(uint32_t rd, uint32_t rs1, uint32_t rs2) { return r_type(Opcode::OP_FP, rd, Funct3_FMINMAX::FMAX, rs1, rs2, Funct7_FP::FMINMAX_S); }

uint32_t FEQ_S(uint32_t rd, uint32_t rs1, uint32_t rs2) { return r_type(Opcode::OP_FP, rd, Funct3_FCMP::FEQ, rs1, rs2, Funct7_FP::FCMP_S); }
uint32_t FLT_S(uint32_t rd, uint32_t rs1, uint32_t rs2) { return r_type(Opcode::OP_FP, rd, Funct3_FCMP::FLT, rs1, rs2, Funct7_FP::FCMP_S); }
uint32_t FLE_S(uint32_t rd, uint32_t rs1, uint32_t rs2) { return r_type(Opcode::OP_FP, rd, Funct3_FCMP::FLE, rs1, rs2, Funct7_FP::FCMP_S); }

uint32_t FCVT_W_S(uint32_t rd, uint32_t rs1)  { return r_type(Opcode::OP_FP, rd, RM_RNE, rs1, Rs2_FCVT::W, Funct7_FP::FCVT_W_S); }
uint32_t FCVT_WU_S(uint32_t rd, uint32_t rs1) { return r_type(Opcode::OP_FP, rd, RM_RNE, rs1, Rs2_FCVT::WU, Funct7_FP::FCVT_W_S); }
uint32_t FCVT_S_W(uint32_t rd, uint32_t rs1)  { return r_type(Opcode::OP_FP, rd, RM_RNE, rs1, Rs2_FCVT::W, Funct7_FP::FCVT_S_W); }
uint32_t FCVT_S_WU(uint32_t rd, uint32_t rs1) { return r_type(Opcode::OP_FP, rd, RM_RNE, rs1, Rs2_FCVT::WU, Funct7_FP::FCVT_S_W); }

uint32_t FMV_X_W(uint32_t rd, uint32_t rs1)  { return r_type(Opcode::OP_FP, rd, Funct3_FMV_FCLASS::FMV_X_W, rs1, 0, Funct7_FP::FMV_X_W_FCLASS_S); }
uint32_t FCLASS_S(uint32_t rd, uint32_t rs1) { return r_type(Opcode::OP_FP, rd, Funct3_FMV_FCLASS::FCLASS_S, rs1, 0, Funct7_FP::FMV_X_W_FCLASS_S); }
uint32_t FMV_W_X(uint32_t rd, uint32_t rs1)  { return r_type(Opcode::OP_FP, rd, RM_RNE, rs1, 0, Funct7_FP::FMV_W_X); }

uint32_t FMADD_S(uint32_t rd, uint32_t rs1, uint32_t rs2, uint32_t rs3)  { return r4_type(Opcode::FMADD, rd, RM_RNE, rs1, rs2, rs3); }
uint32_t FMSUB_S(uint32_t rd, uint32_t rs1, uint32_t rs2, uint32_t rs3)  { return r4_type(Opcode::FMSUB, rd, RM_RNE, rs1, rs2, rs3); }
uint32_t FNMSUB_S(uint32_t rd, uint32_t rs1, uint32_t rs2, uint32_t rs3) { return r4_type(Opcode::FNMSUB, rd, RM_RNE, rs1, rs2, rs3); }
uint32_t FNMADD_S(uint32_t rd, uint32_t rs1, uint32_t rs2, uint32_t rs3) { return r4_type(Opcode::FNMADD, rd, RM_RNE, rs1, rs2, rs3); }

} // namespace asmenc

// ---------------------------------------------------------------------
// Codificadores de la extension C (comprimidas, 16 bits). Cada uno
// construye el patron de bits exactamente inverso al que decodifica
// rv32c::expand() en rv32c_defs.h -- mismos desplazamientos, en la
// direccion opuesta. "_" en el nombre del parametro indica registro
// comprimido de 3 bits (x8-x15); creg_of() hace la conversion desde el
// numero de registro completo.
namespace cenc {
inline uint32_t creg_of(uint32_t full_reg) { return full_reg - 8; }

uint16_t C_ADDI4SPN(uint32_t rd_, uint32_t nzuimm) {
    uint16_t c10_7  = (nzuimm >> 6) & 0xF; // nzuimm[9:6]
    uint16_t c12_11 = (nzuimm >> 4) & 0x3; // nzuimm[5:4]
    uint16_t c5     = (nzuimm >> 3) & 0x1; // nzuimm[3]
    uint16_t c6     = (nzuimm >> 2) & 0x1; // nzuimm[2]
    return (0b000 << 13) | (c12_11 << 11) | (c10_7 << 7) | (c6 << 6) | (c5 << 5) | (rd_ << 2) | 0b00;
}
uint16_t C_LW(uint32_t rd_, uint32_t rs1_, uint32_t uimm) {
    uint16_t c12_10 = (uimm >> 3) & 0x7; // uimm[5:3]
    uint16_t c6     = (uimm >> 2) & 0x1; // uimm[2]
    uint16_t c5     = (uimm >> 6) & 0x1; // uimm[6]
    return (0b010 << 13) | (c12_10 << 10) | (rs1_ << 7) | (c6 << 6) | (c5 << 5) | (rd_ << 2) | 0b00;
}
uint16_t C_SW(uint32_t rs1_, uint32_t rs2_, uint32_t uimm) {
    uint16_t c12_10 = (uimm >> 3) & 0x7;
    uint16_t c6     = (uimm >> 2) & 0x1;
    uint16_t c5     = (uimm >> 6) & 0x1;
    return (0b110 << 13) | (c12_10 << 10) | (rs1_ << 7) | (c6 << 6) | (c5 << 5) | (rs2_ << 2) | 0b00;
}

uint16_t C_ADDI(uint32_t rd, int32_t imm) {
    uint32_t u = static_cast<uint32_t>(imm) & 0x3F;
    return (0b000 << 13) | (((u >> 5) & 0x1) << 12) | (rd << 7) | ((u & 0x1F) << 2) | 0b01;
}
uint16_t C_LI(uint32_t rd, int32_t imm) {
    uint32_t u = static_cast<uint32_t>(imm) & 0x3F;
    return (0b010 << 13) | (((u >> 5) & 0x1) << 12) | (rd << 7) | ((u & 0x1F) << 2) | 0b01;
}
uint16_t C_ADDI16SP(int32_t imm) {
    uint32_t u = static_cast<uint32_t>(imm);
    uint16_t c12 = (u >> 9) & 0x1, c6 = (u >> 4) & 0x1, c5 = (u >> 6) & 0x1;
    uint16_t c4  = (u >> 8) & 0x1, c3 = (u >> 7) & 0x1, c2 = (u >> 5) & 0x1;
    return (0b011 << 13) | (c12 << 12) | (2 << 7) | (c6 << 6) | (c5 << 5) | (c4 << 4) | (c3 << 3) | (c2 << 2) | 0b01;
}
uint16_t C_LUI(uint32_t rd, int32_t nzimm6) {
    uint32_t u = static_cast<uint32_t>(nzimm6) & 0x3F;
    return (0b011 << 13) | (((u >> 5) & 0x1) << 12) | (rd << 7) | ((u & 0x1F) << 2) | 0b01;
}
uint16_t C_SRLI(uint32_t rd_, uint32_t shamt) {
    return (0b100 << 13) | (((shamt >> 5) & 0x1) << 12) | (0b00 << 10) | (rd_ << 7) | ((shamt & 0x1F) << 2) | 0b01;
}
uint16_t C_SRAI(uint32_t rd_, uint32_t shamt) {
    return (0b100 << 13) | (((shamt >> 5) & 0x1) << 12) | (0b01 << 10) | (rd_ << 7) | ((shamt & 0x1F) << 2) | 0b01;
}
uint16_t C_ANDI(uint32_t rd_, int32_t imm) {
    uint32_t u = static_cast<uint32_t>(imm) & 0x3F;
    return (0b100 << 13) | (((u >> 5) & 0x1) << 12) | (0b10 << 10) | (rd_ << 7) | ((u & 0x1F) << 2) | 0b01;
}
uint16_t C_SUB(uint32_t rd_, uint32_t rs2_) { return (0b100011 << 10) | (rd_ << 7) | (0b00 << 5) | (rs2_ << 2) | 0b01; }
uint16_t C_XOR(uint32_t rd_, uint32_t rs2_) { return (0b100011 << 10) | (rd_ << 7) | (0b01 << 5) | (rs2_ << 2) | 0b01; }
uint16_t C_OR(uint32_t rd_, uint32_t rs2_)  { return (0b100011 << 10) | (rd_ << 7) | (0b10 << 5) | (rs2_ << 2) | 0b01; }
uint16_t C_AND(uint32_t rd_, uint32_t rs2_) { return (0b100011 << 10) | (rd_ << 7) | (0b11 << 5) | (rs2_ << 2) | 0b01; }

uint16_t c_jump_field(int32_t imm) {
    uint32_t u = static_cast<uint32_t>(imm);
    uint16_t c12=(u>>11)&1, c11=(u>>4)&1, c10=(u>>9)&1, c9=(u>>8)&1, c8=(u>>10)&1;
    uint16_t c7=(u>>6)&1, c6=(u>>7)&1, c5=(u>>3)&1, c4=(u>>2)&1, c3=(u>>1)&1, c2=(u>>5)&1;
    return (c12<<12)|(c11<<11)|(c10<<10)|(c9<<9)|(c8<<8)|(c7<<7)|(c6<<6)|(c5<<5)|(c4<<4)|(c3<<3)|(c2<<2);
}
uint16_t C_J(int32_t imm)   { return (0b101 << 13) | c_jump_field(imm) | 0b01; }
uint16_t C_JAL(int32_t imm) { return (0b001 << 13) | c_jump_field(imm) | 0b01; }

uint16_t c_branch_field(int32_t imm) {
    uint32_t u = static_cast<uint32_t>(imm);
    uint16_t c12=(u>>8)&1, c11=(u>>4)&1, c10=(u>>3)&1;
    uint16_t c6=(u>>7)&1, c5=(u>>6)&1, c4=(u>>2)&1, c3=(u>>1)&1, c2=(u>>5)&1;
    return (c12<<12)|(c11<<11)|(c10<<10)|(c6<<6)|(c5<<5)|(c4<<4)|(c3<<3)|(c2<<2);
}
uint16_t C_BEQZ(uint32_t rs1_, int32_t imm) { return (0b110 << 13) | c_branch_field(imm) | (rs1_ << 7) | 0b01; }
uint16_t C_BNEZ(uint32_t rs1_, int32_t imm) { return (0b111 << 13) | c_branch_field(imm) | (rs1_ << 7) | 0b01; }

uint16_t C_SLLI(uint32_t rd, uint32_t shamt) {
    return (0b000 << 13) | (((shamt >> 5) & 0x1) << 12) | (rd << 7) | ((shamt & 0x1F) << 2) | 0b10;
}
uint16_t C_LWSP(uint32_t rd, uint32_t uimm) {
    uint16_t c12=(uimm>>5)&1, c6=(uimm>>4)&1, c5=(uimm>>3)&1, c4=(uimm>>2)&1, c3=(uimm>>7)&1, c2=(uimm>>6)&1;
    return (0b010 << 13) | (c12<<12) | (rd << 7) | (c6<<6) | (c5<<5) | (c4<<4) | (c3<<3) | (c2<<2) | 0b10;
}
uint16_t C_SWSP(uint32_t rs2, uint32_t uimm) {
    uint16_t c12_9 = (uimm >> 2) & 0xF; // uimm[5:2]
    uint16_t c8_7  = (uimm >> 6) & 0x3; // uimm[7:6]
    return (0b110 << 13) | (c12_9 << 9) | (c8_7 << 7) | (rs2 << 2) | 0b10;
}
uint16_t C_JR(uint32_t rs1)              { return (0b1000 << 12) | (rs1 << 7) | (0 << 2) | 0b10; }
uint16_t C_MV(uint32_t rd, uint32_t rs2) { return (0b1000 << 12) | (rd << 7) | (rs2 << 2) | 0b10; }
uint16_t C_JALR(uint32_t rs1)            { return (0b1001 << 12) | (rs1 << 7) | (0 << 2) | 0b10; }
uint16_t C_ADD(uint32_t rd, uint32_t rs2){ return (0b1001 << 12) | (rd << 7) | (rs2 << 2) | 0b10; }

} // namespace cenc

// Direccion GLOBAL donde la CPU escribe el bloque de 16 bytes (128 bits)
// que luego lee la VectorUnit para el test end-to-end.
constexpr uint32_t VECTOR_TEST_ADDR = 0x2000;

// Ensambla el programa de prueba RV32I: ejercita R-type, I-type, LOAD,
// STORE, BRANCH, JAL, JALR, LUI y AUIPC, y ademas deja escrito en RAM el
// bloque de datos que usara la VectorUnit.
std::vector<uint8_t> build_test_program() {
    using namespace asmenc;
    std::vector<uint32_t> p;
    auto push = [&](uint32_t w) { p.push_back(w); };

    // --- Bloque 1: inmediatos y aritmetica registro-registro ---
    push(ADDI(1, 0, 10));      // x1 = 10
    push(ADDI(2, 0, 20));      // x2 = 20
    push(ADDI(3, 0, -5));      // x3 = -5
    push(ADDI(4, 0, 100));     // x4 = 100
    push(ADD(5, 1, 2));        // x5 = 30
    push(SUB(6, 2, 1));        // x6 = 10
    push(AND(7, 1, 2));        // x7 = 0
    push(OR(8, 1, 2));         // x8 = 30
    push(XOR(9, 1, 2));        // x9 = 30
    push(SLT(10, 1, 2));       // x10 = 1
    push(SLTU(11, 2, 1));      // x11 = 0
    push(ADDI(13, 0, 2));      // x13 = 2 (cantidad de shift)
    push(SLL(12, 1, 13));      // x12 = 40
    push(SRL(14, 2, 13));      // x14 = 5
    push(ADDI(15, 0, -16));    // x15 = -16
    push(SRA(16, 15, 13));     // x16 = -4

    // --- Bloque 2: LUI / AUIPC ---
    push(LUI(17, 0x12345));    // x17 = 0x12345000
    push(AUIPC(18, 0));        // x18 = pc actual

    // --- Bloque 3: STORE / LOAD sobre RAM (base en x19) ---
    push(LUI(19, 1));          // x19 = 0x1000
    push(SW(19, 1, 0));        // mem[0x1000] = x1 (10)
    push(SW(19, 2, 4));        // mem[0x1004] = x2 (20)
    push(LW(21, 19, 0));       // x21 = 10
    push(LW(22, 19, 4));       // x22 = 20
    push(SB(19, 1, 8));        // mem[0x1008] = byte bajo de x1
    push(LBU(23, 19, 8));      // x23 = 10
    push(SH(19, 2, 10));       // mem[0x100A..0x100B] = half bajo de x2
    push(LHU(24, 19, 10));     // x24 = 20

    // --- Bloque 4: branches (todas las condiciones) ---
    push(BEQ(1, 1, 8));        // taken -> salta la siguiente
    push(ADDI(25, 0, 999));    // (saltada)
    push(ADDI(25, 0, 111));    // x25 = 111
    push(BNE(1, 2, 8));        // taken -> salta la siguiente
    push(ADDI(26, 0, 999));    // (saltada)
    push(ADDI(26, 0, 222));    // x26 = 222
    push(BLT(2, 1, 8));        // no taken (20<10 falso)
    push(ADDI(27, 0, 55));     // x27 = 55 (se ejecuta)
    push(BGE(2, 1, 8));        // taken (20>=10)
    push(ADDI(28, 0, 999));    // (saltada)
    push(ADDI(28, 0, 88));     // x28 = 88

    // --- Bloque 5: JAL / JALR (llamada y retorno a "funcion") ---
    size_t idx_jal   = p.size(); push(0); // placeholder: JAL x1, <func>
    push(ADDI(31, 0, 7));                 // se ejecuta al volver (link = pc de aqui)
    size_t idx_skip  = p.size(); push(0); // placeholder: JAL x0, <end_func>
    size_t idx_func      = p.size(); push(ADDI(30, 0, 66)); // cuerpo de la "funcion"
    push(JALR(0, 1, 0));                                     // retorna a x1
    size_t idx_end_func   = p.size();

    p[idx_jal]  = JAL(1, static_cast<int32_t>((idx_func - idx_jal) * 4));
    p[idx_skip] = JAL(0, static_cast<int32_t>((idx_end_func - idx_skip) * 4));

    // --- Bloque 6: bloque de 128 bits para el test de VectorUnit ---
    push(LUI(9, VECTOR_TEST_ADDR >> 12)); // x9 = 0x2000 (base test vectorial)
    push(LUI(11, 0xAAAA0));               // x11 = 0xAAAA0000
    push(ADDI(12, 11, 1)); push(SW(9, 12, 0));  // mem[0x2000] = 0xAAAA0001
    push(ADDI(12, 11, 2)); push(SW(9, 12, 4));  // mem[0x2004] = 0xAAAA0002
    push(ADDI(12, 11, 3)); push(SW(9, 12, 8));  // mem[0x2008] = 0xAAAA0003
    push(ADDI(12, 11, 4)); push(SW(9, 12, 12)); // mem[0x200C] = 0xAAAA0004

    // --- Bloque 7: extension M (MUL/DIV). Va al final del programa a
    // proposito, para poder reutilizar libremente registros de bloques
    // anteriores (x2..x18) sin perder resultados que ya se verificaron
    // mientras se ejecutaban esos bloques. ---
    push(ADDI(2, 0, -100));    // x2 = -100 (operando A)
    push(ADDI(3, 0, 7));       // x3 = 7    (operando B)
    push(MUL(4, 2, 3));        // x4  = MUL(-100,7)    = -700
    push(MULH(5, 2, 3));       // x5  = MULH(-100,7)   = -1 (signo del producto)
    push(MULHSU(6, 2, 3));     // x6  = MULHSU(-100,7) = -1 (B se trata como no-signado, pero A negativo domina el signo)
    push(MULHU(7, 2, 3));      // x7  = MULHU(-100,7)  = mitad alta de 0xFFFFFF9C * 7 (sin signo)
    push(DIV(8, 2, 3));        // x8  = DIV(-100,7)    = -14 (trunca hacia cero)
    push(DIVU(9, 2, 3));       // x9  = DIVU(-100,7)   = 0xFFFFFF9C / 7 (sin signo)
    push(REM(10, 2, 3));       // x10 = REM(-100,7)    = -2
    push(REMU(11, 2, 3));      // x11 = REMU(-100,7)   = 0xFFFFFF9C % 7 (sin signo)

    push(ADDI(12, 0, 0));      // x12 = 0 (divisor nulo, para casos borde)
    push(DIV(13, 2, 12));      // x13 = DIV(-100,0)    = -1  (division por cero, ISA RISC-V)
    push(REM(14, 2, 12));      // x14 = REM(-100,0)    = -100 (resto = dividendo)

    push(LUI(15, 0x80000));    // x15 = 0x80000000 (INT32_MIN)
    push(ADDI(16, 0, -1));     // x16 = -1
    push(DIV(17, 15, 16));     // x17 = DIV(INT32_MIN,-1) = INT32_MIN (overflow, no trap)
    push(REM(18, 15, 16));     // x18 = REM(INT32_MIN,-1) = 0          (overflow, no trap)

    // --- Bloque 8: extension F (simple precision). Tambien al final,
    // reutiliza libremente x2..x13 y f1..f19. ---
    push(ADDI(2, 0, 10));       // x2 = 10
    push(FCVT_S_W(1, 2));       // f1 = 10.0
    push(ADDI(3, 0, 3));        // x3 = 3
    push(FCVT_S_W(2, 3));       // f2 = 3.0

    push(FADD_S(3, 1, 2));      // f3 = 13.0
    push(FSUB_S(4, 1, 2));      // f4 = 7.0
    push(FMUL_S(5, 1, 2));      // f5 = 30.0
    push(FDIV_S(6, 1, 2));      // f6 = 3.33333325 (10/3 en simple precision)
    push(FSQRT_S(7, 5));        // f7 = sqrt(30) = 5.4772258

    push(FMIN_S(8, 1, 2));      // f8 = 3.0
    push(FMAX_S(9, 1, 2));      // f9 = 10.0

    push(ADDI(4, 0, -1));       // x4 = -1
    push(FCVT_S_W(11, 4));      // f11 = -1.0
    push(FSGNJ_S(10, 1, 2));    // f10 = |f1| con signo de f2(+)  = +10.0
    push(FSGNJN_S(12, 1, 11));  // f12 = |f1| con signo de -f11(-,invertido=+) = +10.0
    push(FSGNJX_S(13, 1, 11));  // f13 = |f1| con signo(f1)^signo(f11) = +^- = -10.0

    push(FEQ_S(5, 1, 1));       // x5 = 1 (f1==f1)
    push(FLT_S(6, 2, 1));       // x6 = 1 (3.0 < 10.0)
    push(FLE_S(7, 1, 1));       // x7 = 1 (f1<=f1)

    push(FCVT_W_S(8, 6));       // x8 = round(3.33333325) = 3   (redondeo del FDIV_S de arriba, f6)
    push(FCVT_WU_S(9, 1));      // x9 = 10

    push(FMV_X_W(10, 1));       // x10 = bits crudos de 10.0f = 0x41200000
    push(FMV_W_X(14, 10));      // f14 = bits_to_float(x10) = 10.0 (ida y vuelta)

    push(FCLASS_S(11, 1));      // x11 = 0x40 (bit6: +normal)

    push(FMADD_S(15, 1, 2, 4));  // f15 = f1*f2+f4 = 30+7  = 37.0
    push(FMSUB_S(16, 1, 2, 4));  // f16 = f1*f2-f4 = 30-7  = 23.0
    push(FNMSUB_S(17, 1, 2, 4)); // f17 = -(f1*f2)+f4 = -30+7 = -23.0
    push(FNMADD_S(18, 1, 2, 4)); // f18 = -(f1*f2)-f4 = -30-7 = -37.0

    push(LUI(12, 1));            // x12 = 0x1000
    push(ADDI(12, 12, 0x10));    // x12 = 0x1010 (base test FLW/FSW)
    push(FSW(12, 1, 0));         // mem[0x1010..13] = bits de f1 (10.0)
    push(FLW(19, 12, 0));        // f19 = 10.0 (leido de vuelta desde RAM)
    push(FEQ_S(13, 1, 19));      // x13 = 1 (round-trip por memoria preserva el bit pattern)

    push(ADDI(2, 0, 0x400));     // x2 = 0x400 (SP valido para el bloque C de abajo)

    // --- Conversion a bytes: a partir de aca el programa mezcla
    // instrucciones de 32 bits (arriba, todo lo que ya se verifico) con
    // instrucciones comprimidas de 16 bits (extension C, abajo) -- por
    // eso el resto del ensamblado trabaja directo sobre un stream de
    // bytes en vez de un arreglo de palabras. ---
    std::vector<uint8_t> bytes;
    bytes.reserve(p.size() * 4 + 128);
    for (uint32_t w : p) {
        bytes.push_back(static_cast<uint8_t>(w));
        bytes.push_back(static_cast<uint8_t>(w >> 8));
        bytes.push_back(static_cast<uint8_t>(w >> 16));
        bytes.push_back(static_cast<uint8_t>(w >> 24));
    }
    auto push16 = [&](uint16_t h) {
        bytes.push_back(static_cast<uint8_t>(h));
        bytes.push_back(static_cast<uint8_t>(h >> 8));
    };

    // --- Bloque 9: extension C (comprimidas). Registros x8-x15 se usan
    // a proposito porque son los unicos direccionables por los formatos
    // comprimidos de 3 bits (rd'/rs1'/rs2'); C.LI/C.ADDI/C.MV/C.ADD/
    // C.SLLI/C.LUI usan el campo de registro completo (5 bits), pueden
    // apuntar a cualquier registro. ---
    using namespace cenc;

    push16(C_LI(8, 5));         // x8 = 5
    push16(C_LI(9, 3));         // x9 = 3
    push16(C_LI(10, 8));        // x10 = 8
    push16(C_ADD(8, 9));        // x8 = x8+x9 = 8
    push16(C_MV(11, 8));        // x11 = x8 = 8
    push16(C_SUB(creg_of(8), creg_of(9)));  // x8 = x8-x9 = 5
    push16(C_XOR(creg_of(10), creg_of(9))); // x10 = x10^x9 = 8^3 = 11
    push16(C_OR(creg_of(11), creg_of(8)));  // x11 = x11|x8 = 8|5 = 13
    push16(C_LI(12, 12));       // x12 = 12
    push16(C_AND(creg_of(12), creg_of(11))); // x12 = x12&x11 = 12&13 = 12
    push16(C_ANDI(creg_of(9), 1));  // x9 = x9&1 = 3&1 = 1
    push16(C_SRLI(creg_of(8), 1));  // x8 = x8>>1 (logico) = 5>>1 = 2
    push16(C_SRAI(creg_of(10), 1)); // x10 = x10>>1 (aritmetico) = 11>>1 = 5
    push16(C_SLLI(8, 2));       // x8 = x8<<2 = 2<<2 = 8
    push16(C_ADDI(8, 5));       // x8 = x8+5 = 13
    push16(C_LUI(16, 3));       // x16 = 3<<12 = 0x3000
    push16(C_ADDI16SP(16));     // x2 = x2+16 = 0x400+16 = 0x410
    push16(C_ADDI4SPN(creg_of(13), 4)); // x13 = x2+4 = 0x414
    push16(C_LUI(14, 1));       // x14 = 0x1000 (base para C.LW/C.SW)
    push16(C_SW(creg_of(14), creg_of(8), 0));  // mem[0x1000] = x8 (13)
    push16(C_LW(creg_of(15), creg_of(14), 0)); // x15 = mem[0x1000] = 13
    push16(C_SWSP(8, 0));       // mem[x2(0x410)] = x8 (13)
    push16(C_LWSP(17, 0));      // x17 = mem[0x410] = 13

    // C.BEQZ tomado. Nota: C.LI solo tiene un inmediato con signo de 6
    // bits (-32..31) -- los marcadores de "salteada"/"aterrizaje" tienen
    // que caber en ese rango (a diferencia de los ADDI de 32 bits usados
    // en los bloques anteriores, que sí admiten valores como 999).
    push16(C_LI(11, 0));                          // x11 = 0
    { size_t origin = bytes.size(); push16(0);
      push16(C_LI(19, 31));                       // (saltada)
      size_t target = bytes.size();
      bytes[origin] = static_cast<uint8_t>(C_BEQZ(creg_of(11), static_cast<int32_t>(target - origin)));
      bytes[origin + 1] = static_cast<uint8_t>(C_BEQZ(creg_of(11), static_cast<int32_t>(target - origin)) >> 8);
    }
    push16(C_LI(19, 17));                         // x19 = 17 (aterrizaje)

    // C.BNEZ tomado
    push16(C_LI(12, 1));                          // x12 = 1
    { size_t origin = bytes.size(); push16(0);
      push16(C_LI(21, 30));                       // (saltada)
      size_t target = bytes.size();
      uint16_t enc = C_BNEZ(creg_of(12), static_cast<int32_t>(target - origin));
      bytes[origin] = static_cast<uint8_t>(enc);
      bytes[origin + 1] = static_cast<uint8_t>(enc >> 8);
    }
    push16(C_LI(21, 21));                         // x21 = 21 (aterrizaje)

    // C.J incondicional
    { size_t origin = bytes.size(); push16(0);
      push16(C_LI(22, 29));                       // (saltada)
      size_t target = bytes.size();
      uint16_t enc = C_J(static_cast<int32_t>(target - origin));
      bytes[origin] = static_cast<uint8_t>(enc);
      bytes[origin + 1] = static_cast<uint8_t>(enc >> 8);
    }
    push16(C_LI(22, 19));                          // x22 = 19 (aterrizaje)

    // C.JAL / C.JR: llamada y retorno a "funcion", igual patron que el
    // bloque JAL/JALR de 32 bits mas arriba, pero con instrucciones
    // comprimidas.
    {
        size_t idx_jal      = bytes.size(); push16(0); // placeholder: C.JAL
        size_t idx_postcall = bytes.size(); push16(C_LI(23, 25)); // corre al volver
        size_t idx_skip      = bytes.size(); push16(0); // placeholder: C.J
        size_t idx_func       = bytes.size(); push16(C_LI(24, 26)); // cuerpo "funcion"
        push16(C_JR(1));                                              // retorna a x1
        size_t idx_end = bytes.size();

        uint16_t jal_enc = C_JAL(static_cast<int32_t>(idx_func - idx_jal));
        bytes[idx_jal] = static_cast<uint8_t>(jal_enc);
        bytes[idx_jal + 1] = static_cast<uint8_t>(jal_enc >> 8);

        uint16_t j_enc = C_J(static_cast<int32_t>(idx_end - idx_skip));
        bytes[idx_skip] = static_cast<uint8_t>(j_enc);
        bytes[idx_skip + 1] = static_cast<uint8_t>(j_enc >> 8);
        (void)idx_postcall;
    }

    // --- Fin de programa: palabra nula detiene la CPU (ver processor.h) ---
    bytes.push_back(0); bytes.push_back(0); bytes.push_back(0); bytes.push_back(0);

    return bytes;
}

// Testbench auxiliar: coordina el test de la VectorUnit una vez que la CPU
// termina de ejecutar (espera Processor::finished), dispara el load vectorial
// y compara el resultado contra lo que escribio la CPU en RAM.
SC_MODULE(Testbench) {
    SC_HAS_PROCESS(Testbench);

    Processor&   cpu;
    VectorUnit&  vu;
    Memory&      mem;

    Testbench(sc_module_name name, Processor& cpu_, VectorUnit& vu_, Memory& mem_)
        : sc_module(name), cpu(cpu_), vu(vu_), mem(mem_) {
        SC_THREAD(run);
    }

    void run() {
        wait(cpu.finished);

        std::cout << "\n=== Volcado de registros RV32I ===" << std::endl;
        cpu.dump_regs();

        std::cout << "\n=== Volcado de registros F (simple precision) ===" << std::endl;
        cpu.dump_fregs();

        // Dato escrito por la CPU, leido directamente de la memoria (backdoor,
        // solo para el print de verificacion del testbench).
        uint8_t cpu_bytes[VectorUnit::VLEN_BYTES];
        std::memcpy(cpu_bytes, &mem.data[VECTOR_TEST_ADDR - memory_map::RAM_BASE], VectorUnit::VLEN_BYTES);

        vu.vector_load_test(0, VECTOR_TEST_ADDR);

        std::cout << "\n=== Test end-to-end VectorUnit (CPU -> Bus -> Memory -> VectorUnit) ===" << std::endl;
        std::cout << "Dato escrito por la CPU en 0x" << std::hex << VECTOR_TEST_ADDR << std::dec << ": ";
        for (unsigned i = 0; i < VectorUnit::VLEN_BYTES; ++i)
            std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(cpu_bytes[i]) << " ";
        std::cout << std::dec << std::endl;

        std::cout << "Dato leido por la VectorUnit (v0):          ";
        vu.dump_vreg(0);

        bool match = std::memcmp(cpu_bytes, vu.vregs[0].data(), VectorUnit::VLEN_BYTES) == 0;
        std::cout << "Resultado: " << (match ? "COINCIDEN (OK)" : "NO COINCIDEN (FALLO)") << std::endl;

        sc_stop();
    }
};

// ─────────────────────────────────────────────────────────────────────────────
// ELF Loader: carga un binario ELF32 en la RAM del simulador.
// Parsea los segmentos PT_LOAD y los copia a memory.data[p_vaddr].
// Devuelve el entry point (e_entry) para inicializar el PC del procesador.
// ─────────────────────────────────────────────────────────────────────────────
uint32_t load_elf_file(const char* path, Memory& mem) {
    struct Ehdr {
        uint8_t  e_ident[16];
        uint16_t e_type, e_machine;
        uint32_t e_version, e_entry, e_phoff, e_shoff, e_flags;
        uint16_t e_ehsize, e_phentsize, e_phnum;
        uint16_t e_shentsize, e_shnum, e_shstrndx;
    };
    struct Phdr {
        uint32_t p_type, p_offset, p_vaddr, p_paddr;
        uint32_t p_filesz, p_memsz, p_flags, p_align;
    };
    constexpr uint32_t PT_LOAD = 1;

    std::ifstream f(path, std::ios::binary);
    if (!f) {
        std::cerr << "[ELF] No se pudo abrir: " << path << "\n";
        std::exit(1);
    }

    Ehdr ehdr;
    f.read(reinterpret_cast<char*>(&ehdr), sizeof(ehdr));
    if (!f || ehdr.e_ident[0] != 0x7F || ehdr.e_ident[1] != 'E' ||
        ehdr.e_ident[2] != 'L'  || ehdr.e_ident[3] != 'F') {
        std::cerr << "[ELF] Magia invalida — no es un ELF valido\n";
        std::exit(1);
    }
    if (ehdr.e_ident[4] != 1) {
        std::cerr << "[ELF] Solo se soporta ELF32\n";
        std::exit(1);
    }

    std::cout << "[ELF] Cargando: " << path << "\n";
    std::cout << "[ELF] Entry point: 0x" << std::hex << ehdr.e_entry << std::dec << "\n";

    for (uint16_t i = 0; i < ehdr.e_phnum; ++i) {
        Phdr phdr;
        f.seekg(ehdr.e_phoff + i * ehdr.e_phentsize);
        f.read(reinterpret_cast<char*>(&phdr), sizeof(phdr));
        if (phdr.p_type != PT_LOAD || phdr.p_filesz == 0) continue;

        uint32_t ram_offset = phdr.p_vaddr - memory_map::RAM_BASE;
        if (ram_offset + phdr.p_memsz > memory_map::RAM_SIZE) {
            std::cerr << "[ELF] Segmento 0x" << std::hex << phdr.p_vaddr
                      << " no cabe en RAM\n" << std::dec;
            std::exit(1);
        }

        f.seekg(phdr.p_offset);
        f.read(reinterpret_cast<char*>(mem.data.data() + ram_offset), phdr.p_filesz);

        if (phdr.p_memsz > phdr.p_filesz)
            std::memset(mem.data.data() + ram_offset + phdr.p_filesz,
                        0, phdr.p_memsz - phdr.p_filesz);

        std::cout << "[ELF]   PT_LOAD vaddr=0x" << std::hex << phdr.p_vaddr
                  << " filesz=" << phdr.p_filesz
                  << " memsz=" << phdr.p_memsz << std::dec << "\n";
    }
    return ehdr.e_entry;
}

// ─────────────────────────────────────────────────────────────────────────────
// sc_main: modo ELF si se pasa un argumento, modo test si no.
//   ./riscv_sim coremark.elf   → carga binario y corre hasta ECALL exit
//   ./riscv_sim                → corre el programa de prueba original
// ─────────────────────────────────────────────────────────────────────────────
int sc_main(int argc, char* argv[]) {
    Memory     memory("memory");
    Bus        bus("bus");
    Processor  cpu("cpu");
    VectorUnit vu("vector_unit");

    cpu.init_socket.bind(bus.cpu_target);
    vu.init_socket.bind(bus.vector_target);
    bus.mem_initiator.bind(memory.socket);

    if (argc >= 2) {
        // Modo ELF: cargar binario compilado
        uint32_t entry = load_elf_file(argv[1], memory);
        cpu.pc      = entry;
        cpu.regs[2] = memory_map::STACK_TOP;  // inicializar sp
        std::cout << "[SIM] Modo ELF — sp=0x" << std::hex
                  << memory_map::STACK_TOP << " pc=0x" << entry
                  << std::dec << "\n\n";
        sc_start();
    } else {
        // Modo test: programa de prueba original (sin cambios)
        Testbench tb("testbench", cpu, vu, memory);
        std::vector<uint8_t> program = build_test_program();
        std::memcpy(memory.data.data(), program.data(), program.size());
        sc_start();
    }

    return 0;
}
