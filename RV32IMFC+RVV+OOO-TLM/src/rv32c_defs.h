#ifndef RV32C_DEFS_H
#define RV32C_DEFS_H

#include <cstdint>
#include "rv32i_defs.h"

// Extension C (RVC): instrucciones comprimidas de 16 bits. Cada una es
// equivalente semantico de una instruccion RV32I/M/F estandar de 32 bits
// (mas compacta en el binario); en vez de duplicar la logica de
// ejecucion del switch de processor.h, este archivo solo EXPANDE el
// patron de 16 bits al patron de 32 bits equivalente, y el decoder
// principal ejecuta ese resultado exactamente igual que si hubiera
// venido de una instruccion normal. Esto es literalmente lo que hacen
// muchos decoders RVC reales.
namespace rv32c {

// Registro comprimido de 3 bits (campo rd'/rs1'/rs2') -> registro
// completo de 5 bits. La extension C solo permite que estos campos
// referencien x8-x15 (los "registros populares"), de ahi el +8.
inline uint32_t creg(uint32_t r3) { return r3 + 8; }

inline int32_t sext(uint32_t value, unsigned bits) {
    uint32_t shift = 32 - bits;
    return static_cast<int32_t>(value << shift) >> shift;
}

// --- Encoders minimos de RV32I, para construir la instruccion de 32
// bits equivalente (mismo patron que asmenc en main.cpp, pero vive aca
// porque el decoder principal necesita poder expandir en runtime, no
// solo al ensamblar un programa de prueba). ---
inline uint32_t r_type(uint32_t opcode, uint32_t rd, uint32_t f3, uint32_t rs1, uint32_t rs2, uint32_t f7) {
    return (f7 << 25) | (rs2 << 20) | (rs1 << 15) | (f3 << 12) | (rd << 7) | opcode;
}
inline uint32_t i_type(uint32_t opcode, uint32_t rd, uint32_t f3, uint32_t rs1, int32_t imm) {
    return ((static_cast<uint32_t>(imm) & 0xFFF) << 20) | (rs1 << 15) | (f3 << 12) | (rd << 7) | opcode;
}
inline uint32_t s_type(uint32_t opcode, uint32_t f3, uint32_t rs1, uint32_t rs2, int32_t imm) {
    uint32_t u = static_cast<uint32_t>(imm) & 0xFFF;
    return ((u >> 5) << 25) | (rs2 << 20) | (rs1 << 15) | (f3 << 12) | ((u & 0x1F) << 7) | opcode;
}
inline uint32_t b_type(uint32_t f3, uint32_t rs1, uint32_t rs2, int32_t imm) {
    uint32_t u = static_cast<uint32_t>(imm) & 0x1FFF;
    return (((u >> 12) & 0x1) << 31) | (((u >> 5) & 0x3F) << 25) | (rs2 << 20) | (rs1 << 15) |
           (f3 << 12) | (((u >> 1) & 0xF) << 8) | (((u >> 11) & 0x1) << 7) | rv32i::Opcode::BRANCH;
}
inline uint32_t u_type(uint32_t opcode, uint32_t rd, int32_t imm_upper20) {
    return ((static_cast<uint32_t>(imm_upper20) << 12) & 0xFFFFF000u) | (rd << 7) | opcode;
}
inline uint32_t j_type(uint32_t rd, int32_t imm) {
    uint32_t u = static_cast<uint32_t>(imm) & 0x1FFFFF;
    return (((u >> 20) & 0x1) << 31) | (((u >> 1) & 0x3FF) << 21) | (((u >> 11) & 0x1) << 20) |
           (((u >> 12) & 0xFF) << 12) | (rd << 7) | rv32i::Opcode::JAL;
}

// Marcador de "instruccion comprimida no reconocida / no soportada":
// deliberadamente NO es 0x00000000 (esa es la convencion de "fin de
// programa" en processor.h) -- decodifica a un opcode invalido real
// (0x7F), que cae en el 'default' del switch principal e imprime error,
// en vez de confundirse con el fin de programa silencioso.
constexpr uint32_t ILLEGAL = 0xFFFFFFFF;

// Expande una instruccion comprimida de 16 bits a su equivalente de 32
// bits. Precondicion: c[1:0] != 0b11 (si no, no es una instruccion
// comprimida, no llamar a esta funcion).
inline uint32_t expand(uint16_t c) {
    uint32_t quadrant = c & 0x3;
    uint32_t funct3   = (c >> 13) & 0x7;

    if (quadrant == 0b00) {
        uint32_t rd_rs2_ = rv32c::creg((c >> 2) & 0x7);
        uint32_t rs1_    = rv32c::creg((c >> 7) & 0x7);

        switch (funct3) {
            case 0b000: { // C.ADDI4SPN -> addi rd', x2, nzuimm
                uint32_t nzuimm = (((c >> 7) & 0x30)) | (((c >> 1) & 0x3C0)) |
                                  (((c >> 4) & 0x4))  | (((c >> 2) & 0x8));
                // nzuimm[9:6]=c[10:7] nzuimm[5:4]=c[12:11] nzuimm[3]=c[5] nzuimm[2]=c[6]
                if (nzuimm == 0) return ILLEGAL;
                return i_type(rv32i::Opcode::OP_IMM, rd_rs2_, rv32i::Funct3_ALU::ADD_SUB, 2,
                              static_cast<int32_t>(nzuimm));
            }
            case 0b010: { // C.LW -> lw rd', uimm(rs1')
                uint32_t uimm = (((c >> 7) & 0x38)) | (((c << 1) & 0x40)) | (((c >> 4) & 0x4));
                // uimm[5:3]=c[12:10] uimm[6]=c[5] uimm[2]=c[6]
                return i_type(rv32i::Opcode::LOAD, rd_rs2_, rv32i::Funct3_LOAD::LW, rs1_,
                              static_cast<int32_t>(uimm));
            }
            case 0b011: { // C.FLW -> flw rd', uimm(rs1')
                uint32_t uimm = (((c >> 7) & 0x38)) | (((c << 1) & 0x40)) | (((c >> 4) & 0x4));
                return i_type(rv32i::Opcode::LOAD_FP, rd_rs2_, rv32i::Funct3_FP_MEM::W, rs1_,
                              static_cast<int32_t>(uimm));
            }
            case 0b110: { // C.SW -> sw rs2', uimm(rs1')
                uint32_t uimm = (((c >> 7) & 0x38)) | (((c << 1) & 0x40)) | (((c >> 4) & 0x4));
                return s_type(rv32i::Opcode::STORE, rv32i::Funct3_STORE::SW, rs1_, rd_rs2_,
                              static_cast<int32_t>(uimm));
            }
            case 0b111: { // C.FSW -> fsw rs2', uimm(rs1')
                uint32_t uimm = (((c >> 7) & 0x38)) | (((c << 1) & 0x40)) | (((c >> 4) & 0x4));
                return s_type(rv32i::Opcode::STORE_FP, rv32i::Funct3_FP_MEM::W, rs1_, rd_rs2_,
                              static_cast<int32_t>(uimm));
            }
            default:
                return ILLEGAL;
        }
    }

    if (quadrant == 0b01) {
        uint32_t rd_rs1 = (c >> 7) & 0x1F;         // registro completo (CI)
        uint32_t rd_rs1_ = rv32c::creg((c >> 7) & 0x7); // registro comprimido (CA/CB)
        uint32_t rs2_     = rv32c::creg((c >> 2) & 0x7);

        switch (funct3) {
            case 0b000: { // C.NOP / C.ADDI -> addi rd, rd, imm
                int32_t imm = sext((((c >> 7) & 0x20)) | ((c >> 2) & 0x1F), 6);
                return i_type(rv32i::Opcode::OP_IMM, rd_rs1, rv32i::Funct3_ALU::ADD_SUB, rd_rs1, imm);
            }
            case 0b001: { // C.JAL -> jal x1, imm (RV32 unicamente)
                uint32_t u = 0;
                u |= ((c >> 1) & 0x800);  // imm[11] = c[12]
                u |= ((c >> 7) & 0x10);   // imm[4]  = c[11]
                u |= ((c >> 1) & 0x300);  // imm[9:8]= c[10:9]
                u |= ((c << 2) & 0x400);  // imm[10] = c[8]
                u |= ((c >> 1) & 0x40);   // imm[6]  = c[7]
                u |= ((c << 1) & 0x80);   // imm[7]  = c[6]
                u |= ((c >> 2) & 0xE);    // imm[3:1]= c[5:3]  (bits 3,2,1)
                u |= ((c << 3) & 0x20);   // imm[5]  = c[2]
                int32_t imm = sext(u, 12);
                return j_type(1, imm);
            }
            case 0b010: { // C.LI -> addi rd, x0, imm
                int32_t imm = sext((((c >> 7) & 0x20)) | ((c >> 2) & 0x1F), 6);
                return i_type(rv32i::Opcode::OP_IMM, rd_rs1, rv32i::Funct3_ALU::ADD_SUB, 0, imm);
            }
            case 0b011: {
                if (rd_rs1 == 2) { // C.ADDI16SP -> addi x2, x2, imm
                    uint32_t u = 0;
                    u |= ((c >> 3) & 0x200); // imm[9] = c[12]
                    u |= ((c >> 2) & 0x10);  // imm[4] = c[6]
                    u |= ((c << 1) & 0x40);  // imm[6] = c[5]
                    u |= ((c << 4) & 0x180); // imm[8:7]=c[4:3]
                    u |= ((c << 3) & 0x20);  // imm[5] = c[2]
                    int32_t imm = sext(u, 10);
                    if (imm == 0) return ILLEGAL;
                    return i_type(rv32i::Opcode::OP_IMM, 2, rv32i::Funct3_ALU::ADD_SUB, 2, imm);
                }
                if (rd_rs1 == 0) return ILLEGAL; // reservado
                // C.LUI -> lui rd, nzimm[17:12] con signo
                uint32_t raw6 = (((c >> 7) & 0x20)) | ((c >> 2) & 0x1F); // c[12]->bit5, c[6:2]->bits4:0
                int32_t nzimm = sext(raw6, 6);
                if (nzimm == 0) return ILLEGAL;
                return u_type(rv32i::Opcode::LUI, rd_rs1, nzimm);
            }
            case 0b100: {
                uint32_t funct2_hi = (c >> 10) & 0x3; // c[11:10]
                switch (funct2_hi) {
                    case 0b00: { // C.SRLI -> srli rd', rd', shamt
                        uint32_t shamt = (((c >> 7) & 0x20)) | ((c >> 2) & 0x1F);
                        return r_type(rv32i::Opcode::OP_IMM, rd_rs1_, rv32i::Funct3_ALU::SRL_SRA,
                                      rd_rs1_, shamt & 0x1F, rv32i::Funct7::NORMAL);
                    }
                    case 0b01: { // C.SRAI -> srai rd', rd', shamt
                        uint32_t shamt = (((c >> 7) & 0x20)) | ((c >> 2) & 0x1F);
                        return r_type(rv32i::Opcode::OP_IMM, rd_rs1_, rv32i::Funct3_ALU::SRL_SRA,
                                      rd_rs1_, shamt & 0x1F, rv32i::Funct7::ALT);
                    }
                    case 0b10: { // C.ANDI -> andi rd', rd', imm
                        int32_t imm = sext((((c >> 7) & 0x20)) | ((c >> 2) & 0x1F), 6);
                        return i_type(rv32i::Opcode::OP_IMM, rd_rs1_, rv32i::Funct3_ALU::AND, rd_rs1_, imm);
                    }
                    default: { // 0b11: C.SUB/C.XOR/C.OR/C.AND (bit12 debe ser 0 en RV32)
                        uint32_t funct2_lo = (c >> 5) & 0x3; // c[6:5]
                        switch (funct2_lo) {
                            case 0b00: // C.SUB
                                return r_type(rv32i::Opcode::OP, rd_rs1_, rv32i::Funct3_ALU::ADD_SUB,
                                              rd_rs1_, rs2_, rv32i::Funct7::ALT);
                            case 0b01: // C.XOR
                                return r_type(rv32i::Opcode::OP, rd_rs1_, rv32i::Funct3_ALU::XOR,
                                              rd_rs1_, rs2_, rv32i::Funct7::NORMAL);
                            case 0b10: // C.OR
                                return r_type(rv32i::Opcode::OP, rd_rs1_, rv32i::Funct3_ALU::OR,
                                              rd_rs1_, rs2_, rv32i::Funct7::NORMAL);
                            default: // C.AND
                                return r_type(rv32i::Opcode::OP, rd_rs1_, rv32i::Funct3_ALU::AND,
                                              rd_rs1_, rs2_, rv32i::Funct7::NORMAL);
                        }
                    }
                }
            }
            case 0b101: { // C.J -> jal x0, imm (misma disposicion de bits que C.JAL)
                uint32_t u = 0;
                u |= ((c >> 1) & 0x800);
                u |= ((c >> 7) & 0x10);
                u |= ((c >> 1) & 0x300);
                u |= ((c << 2) & 0x400);
                u |= ((c >> 1) & 0x40);
                u |= ((c << 1) & 0x80);
                u |= ((c >> 2) & 0xE);
                u |= ((c << 3) & 0x20);
                int32_t imm = sext(u, 12);
                return j_type(0, imm);
            }
            case 0b110: { // C.BEQZ -> beq rs1', x0, offset
                uint32_t u = 0;
                u |= ((c >> 4) & 0x100); // offset[8] = c[12]
                u |= ((c >> 7) & 0x18);  // offset[4:3] = c[11:10]
                u |= ((c << 1) & 0xC0);  // offset[7:6] = c[6:5]
                u |= ((c << 3) & 0x20);  // offset[5] = c[2]
                u |= ((c >> 2) & 0x6);   // offset[2:1] = c[4:3]
                int32_t imm = sext(u, 9);
                return b_type(rv32i::Funct3_BRANCH::BEQ, rd_rs1_, 0, imm);
            }
            case 0b111: { // C.BNEZ -> bne rs1', x0, offset
                uint32_t u = 0;
                u |= ((c >> 4) & 0x100);
                u |= ((c >> 7) & 0x18);
                u |= ((c << 1) & 0xC0);
                u |= ((c << 3) & 0x20);
                u |= ((c >> 2) & 0x6);
                int32_t imm = sext(u, 9);
                return b_type(rv32i::Funct3_BRANCH::BNE, rd_rs1_, 0, imm);
            }
        }
    }

    if (quadrant == 0b10) {
        uint32_t rd_rs1 = (c >> 7) & 0x1F;
        uint32_t rs2    = (c >> 2) & 0x1F;

        switch (funct3) {
            case 0b000: { // C.SLLI -> slli rd, rd, shamt
                uint32_t shamt = (((c >> 7) & 0x20)) | ((c >> 2) & 0x1F);
                if (rd_rs1 == 0) return ILLEGAL;
                return r_type(rv32i::Opcode::OP_IMM, rd_rs1, rv32i::Funct3_ALU::SLL,
                              rd_rs1, shamt & 0x1F, rv32i::Funct7::NORMAL);
            }
            case 0b010: { // C.LWSP -> lw rd, uimm(x2)
                // uimm[5]=c[12] uimm[4]=c[6] uimm[3]=c[5] uimm[2]=c[4] uimm[7]=c[3] uimm[6]=c[2]
                uint32_t uimm = (((c >> 7) & 0x20)) | (((c >> 2) & 0x18)) |
                                (((c << 4) & 0xC0))  | (((c >> 2) & 0x4));
                if (rd_rs1 == 0) return ILLEGAL;
                return i_type(rv32i::Opcode::LOAD, rd_rs1, rv32i::Funct3_LOAD::LW, 2,
                              static_cast<int32_t>(uimm));
            }
            case 0b011: { // C.FLWSP -> flw rd, uimm(x2)
                uint32_t uimm = (((c >> 7) & 0x20)) | (((c >> 2) & 0x18)) |
                                (((c << 4) & 0xC0))  | (((c >> 2) & 0x4));
                return i_type(rv32i::Opcode::LOAD_FP, rd_rs1, rv32i::Funct3_FP_MEM::W, 2,
                              static_cast<int32_t>(uimm));
            }
            case 0b100: {
                bool bit12 = (c >> 12) & 0x1;
                if (!bit12) {
                    if (rs2 == 0) { // C.JR -> jalr x0, rs1, 0
                        if (rd_rs1 == 0) return ILLEGAL;
                        return i_type(rv32i::Opcode::JALR, 0, rv32i::Funct3_ALU::ADD_SUB, rd_rs1, 0);
                    }
                    // C.MV -> add rd, x0, rs2
                    return r_type(rv32i::Opcode::OP, rd_rs1, rv32i::Funct3_ALU::ADD_SUB,
                                  0, rs2, rv32i::Funct7::NORMAL);
                } else {
                    if (rs2 == 0 && rd_rs1 == 0) return ILLEGAL; // C.EBREAK, no soportado
                    if (rs2 == 0) // C.JALR -> jalr x1, rs1, 0
                        return i_type(rv32i::Opcode::JALR, 1, rv32i::Funct3_ALU::ADD_SUB, rd_rs1, 0);
                    // C.ADD -> add rd, rd, rs2
                    return r_type(rv32i::Opcode::OP, rd_rs1, rv32i::Funct3_ALU::ADD_SUB,
                                  rd_rs1, rs2, rv32i::Funct7::NORMAL);
                }
            }
            case 0b110: { // C.SWSP -> sw rs2, uimm(x2)
                uint32_t uimm = (((c >> 7) & 0x3C)) | (((c >> 1) & 0xC0));
                // uimm[5:2]=c[12:9] uimm[7:6]=c[8:7]
                return s_type(rv32i::Opcode::STORE, rv32i::Funct3_STORE::SW, 2, rs2,
                              static_cast<int32_t>(uimm));
            }
            case 0b111: { // C.FSWSP -> fsw rs2, uimm(x2)
                uint32_t uimm = (((c >> 7) & 0x3C)) | (((c >> 1) & 0xC0));
                return s_type(rv32i::Opcode::STORE_FP, rv32i::Funct3_FP_MEM::W, 2, rs2,
                              static_cast<int32_t>(uimm));
            }
            default:
                return ILLEGAL;
        }
    }

    return ILLEGAL;
}

} // namespace rv32c

#endif // RV32C_DEFS_H
