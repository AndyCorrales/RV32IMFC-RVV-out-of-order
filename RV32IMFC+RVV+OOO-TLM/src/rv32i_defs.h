#ifndef RV32I_DEFS_H
#define RV32I_DEFS_H

#include <cstdint>

// Opcodes (bits [6:0]) y campos funct3/funct7 del ISA base RV32I.
namespace rv32i {

namespace Opcode {
constexpr uint32_t OP        = 0b0110011; // R-type: registro-registro
constexpr uint32_t OP_IMM    = 0b0010011; // I-type: aritmetica inmediata
constexpr uint32_t LOAD      = 0b0000011; // I-type: carga desde memoria
constexpr uint32_t STORE     = 0b0100011; // S-type: almacenamiento en memoria
constexpr uint32_t BRANCH    = 0b1100011; // B-type: saltos condicionales
constexpr uint32_t JAL       = 0b1101111; // J-type: salto incondicional con enlace
constexpr uint32_t JALR      = 0b1100111; // I-type: salto indirecto con enlace
constexpr uint32_t LUI       = 0b0110111; // U-type: carga inmediato en bits altos
constexpr uint32_t AUIPC     = 0b0010111; // U-type: PC + inmediato en bits altos

// Extension F (punto flotante simple precision).
constexpr uint32_t LOAD_FP   = 0b0000111; // I-type: FLW
constexpr uint32_t STORE_FP  = 0b0100111; // S-type: FSW
constexpr uint32_t FMADD     = 0b1000011; // R4-type: rd = (rs1*rs2)+rs3
constexpr uint32_t FMSUB     = 0b1000111; // R4-type: rd = (rs1*rs2)-rs3
constexpr uint32_t FNMSUB    = 0b1001011; // R4-type: rd = -(rs1*rs2)+rs3
constexpr uint32_t FNMADD    = 0b1001111; // R4-type: rd = -(rs1*rs2)-rs3
constexpr uint32_t OP_FP     = 0b1010011; // R-type FP: aritmetica/comparacion/conversion

// SYSTEM: ECALL/EBREAK (funct3=0) e instrucciones CSR (funct3=1..7) --
// el minimo de sistema para bare-metal (ver processor_ooo.h).
constexpr uint32_t SYSTEM    = 0b1110011;
constexpr uint32_t MISC_MEM  = 0b0001111; // FENCE / FENCE.I (no-op en este core)
} // namespace Opcode

// funct3 dentro de SYSTEM: 000 = ECALL/EBREAK (distinguidos por el
// inmediato de 12 bits [31:20]); el resto son las 6 formas de acceso a
// CSR (con registro o con inmediato de 5 bits, y write/set/clear).
namespace Funct3_SYSTEM {
constexpr uint32_t PRIV   = 0b000; // ECALL (imm=0) / EBREAK (imm=1) / MRET (imm=0x302)
constexpr uint32_t CSRRW  = 0b001;
constexpr uint32_t CSRRS  = 0b010;
constexpr uint32_t CSRRC  = 0b011;
constexpr uint32_t CSRRWI = 0b101;
constexpr uint32_t CSRRSI = 0b110;
constexpr uint32_t CSRRCI = 0b111;
} // namespace Funct3_SYSTEM

// Direcciones de CSR de modo maquina (subset minimo para bare-metal).
namespace CSR {
constexpr uint32_t MSTATUS = 0x300;
constexpr uint32_t MISA    = 0x301;
constexpr uint32_t MIE     = 0x304;
constexpr uint32_t MTVEC   = 0x305;
constexpr uint32_t MSCRATCH= 0x340;
constexpr uint32_t MEPC    = 0x341;
constexpr uint32_t MCAUSE  = 0x342;
constexpr uint32_t MTVAL   = 0x343;
constexpr uint32_t MIP     = 0x344;
constexpr uint32_t MHARTID = 0xF14;
// CSRs vectoriales (RVV): de solo lectura desde csrr -- los escribe la
// familia vsetvli/vsetivli/vsetvl, no csrw.
constexpr uint32_t VSTART  = 0x008; // read-write: primer elemento a ejecutar
constexpr uint32_t VL      = 0xC20;
constexpr uint32_t VTYPE   = 0xC21;
constexpr uint32_t VLENB   = 0xC22;
// mtime/mtimecmp: en hardware real son registros MMIO del CLINT, no CSRs.
// Aca se exponen como CSRs de M-mode para no necesitar un periferico
// aparte -- simplificacion documentada en el README.
constexpr uint32_t MTIME    = 0x7C0;
constexpr uint32_t MTIMECMP = 0x7C1;
} // namespace CSR

// funct3 para OP y OP_IMM (comparten codificacion base: la ALU hace lo
// mismo ya sea con dos registros o con un registro+inmediato).
namespace Funct3_ALU {
constexpr uint32_t ADD_SUB = 0b000;
constexpr uint32_t SLL     = 0b001;
constexpr uint32_t SLT     = 0b010;
constexpr uint32_t SLTU    = 0b011;
constexpr uint32_t XOR     = 0b100;
constexpr uint32_t SRL_SRA = 0b101;
constexpr uint32_t OR      = 0b110;
constexpr uint32_t AND     = 0b111;
} // namespace Funct3_ALU

// funct3 para LOAD
namespace Funct3_LOAD {
constexpr uint32_t LB  = 0b000;
constexpr uint32_t LH  = 0b001;
constexpr uint32_t LW  = 0b010;
constexpr uint32_t LBU = 0b100;
constexpr uint32_t LHU = 0b101;
} // namespace Funct3_LOAD

// funct3 para STORE
namespace Funct3_STORE {
constexpr uint32_t SB = 0b000;
constexpr uint32_t SH = 0b001;
constexpr uint32_t SW = 0b010;
} // namespace Funct3_STORE

// funct3 para BRANCH
namespace Funct3_BRANCH {
constexpr uint32_t BEQ  = 0b000;
constexpr uint32_t BNE  = 0b001;
constexpr uint32_t BLT  = 0b100;
constexpr uint32_t BGE  = 0b101;
constexpr uint32_t BLTU = 0b110;
constexpr uint32_t BGEU = 0b111;
} // namespace Funct3_BRANCH

// funct3 para OP con funct7=Funct7::MULDIV (extension M, ver mas abajo).
namespace Funct3_MULDIV {
constexpr uint32_t MUL    = 0b000;
constexpr uint32_t MULH   = 0b001;
constexpr uint32_t MULHSU = 0b010;
constexpr uint32_t MULHU  = 0b011;
constexpr uint32_t DIV    = 0b100;
constexpr uint32_t DIVU   = 0b101;
constexpr uint32_t REM    = 0b110;
constexpr uint32_t REMU   = 0b111;
} // namespace Funct3_MULDIV

// funct7 para OP: distingue variantes ADD/SUB, SRL/SRA, y el bloque
// completo de la extension M (MULDIV comparte el opcode OP, se distingue
// unicamente por este funct7; RISC-V la reservo asi a proposito para que
// un core sin M pueda decodificar el resto de OP sin ambiguedad).
namespace Funct7 {
constexpr uint32_t NORMAL = 0b0000000;
constexpr uint32_t ALT    = 0b0100000; // SUB, SRA
constexpr uint32_t MULDIV = 0b0000001; // MUL, MULH, MULHSU, MULHU, DIV, DIVU, REM, REMU
} // namespace Funct7

// funct3 para LOAD_FP/STORE_FP: unico ancho soportado (simple precision).
namespace Funct3_FP_MEM {
constexpr uint32_t W = 0b010; // FLW / FSW
} // namespace Funct3_FP_MEM

// funct7 para OP_FP: cada operacion (o grupo de operaciones emparentadas)
// tiene su propio valor de funct7. Dentro de FSGNJ/FMINMAX/FCMP, funct3
// selecciona la variante puntual (ver namespaces de abajo). Para
// FADD/FSUB/FMUL/FDIV/FSQRT, en cambio, funct3 codifica el modo de
// redondeo (rm) del ISA real; este modelo no implementa frm/fflags y
// siempre asume rm=000 (round-to-nearest-even, que es ademas el
// redondeo por defecto de la FPU nativa que usa C++).
namespace Funct7_FP {
constexpr uint32_t FADD_S    = 0b0000000;
constexpr uint32_t FSUB_S    = 0b0000100;
constexpr uint32_t FMUL_S    = 0b0001000;
constexpr uint32_t FDIV_S    = 0b0001100;
constexpr uint32_t FSQRT_S   = 0b0101100; // rs2 = 00000 (no usado)
constexpr uint32_t FSGNJ_S   = 0b0010000; // funct3: FSGNJ/FSGNJN/FSGNJX
constexpr uint32_t FMINMAX_S = 0b0010100; // funct3: FMIN/FMAX
constexpr uint32_t FCMP_S    = 0b1010000; // funct3: FLE/FLT/FEQ
constexpr uint32_t FCVT_W_S  = 0b1100000; // float->int, rs2: W/WU
constexpr uint32_t FCVT_S_W  = 0b1101000; // int->float, rs2: W/WU
constexpr uint32_t FMV_X_W_FCLASS_S = 0b1110000; // funct3: FMV.X.W/FCLASS.S
constexpr uint32_t FMV_W_X   = 0b1111000;
} // namespace Funct7_FP

namespace Funct3_FSGNJ {
constexpr uint32_t FSGNJ  = 0b000; // signo de rs2
constexpr uint32_t FSGNJN = 0b001; // signo invertido de rs2
constexpr uint32_t FSGNJX = 0b010; // signo(rs1) XOR signo(rs2)
} // namespace Funct3_FSGNJ

namespace Funct3_FMINMAX {
constexpr uint32_t FMIN = 0b000;
constexpr uint32_t FMAX = 0b001;
} // namespace Funct3_FMINMAX

namespace Funct3_FCMP {
constexpr uint32_t FLE = 0b000;
constexpr uint32_t FLT = 0b001;
constexpr uint32_t FEQ = 0b010;
} // namespace Funct3_FCMP

namespace Funct3_FMV_FCLASS {
constexpr uint32_t FMV_X_W  = 0b000;
constexpr uint32_t FCLASS_S = 0b001;
} // namespace Funct3_FMV_FCLASS

// Campo rs2 de FCVT.W.S/FCVT.WU.S/FCVT.S.W/FCVT.S.WU: no es un registro
// fuente real, se reusa esa posicion de bits para elegir con/sin signo.
namespace Rs2_FCVT {
constexpr uint32_t W  = 0b00000;
constexpr uint32_t WU = 0b00001;
} // namespace Rs2_FCVT

// Extraccion de campos de una instruccion de 32 bits.
inline uint32_t opcode(uint32_t instr) { return instr & 0x7F; }
inline uint32_t rd(uint32_t instr)     { return (instr >> 7) & 0x1F; }
inline uint32_t funct3(uint32_t instr) { return (instr >> 12) & 0x7; }
inline uint32_t rs1(uint32_t instr)    { return (instr >> 15) & 0x1F; }
inline uint32_t rs2(uint32_t instr)    { return (instr >> 20) & 0x1F; }
inline uint32_t funct7(uint32_t instr) { return (instr >> 25) & 0x7F; }

// Extraccion adicional para R4-type (FMADD/FMSUB/FNMSUB/FNMADD): rs3 es
// el tercer operando fuente, fmt selecciona precision (00=simple, la
// unica que implementa este modelo).
inline uint32_t rs3(uint32_t instr) { return (instr >> 27) & 0x1F; }
inline uint32_t fmt(uint32_t instr) { return (instr >> 25) & 0x3; }

} // namespace rv32i

#endif // RV32I_DEFS_H
