#ifndef IMMEDIATES_H
#define IMMEDIATES_H

#include <systemc.h>
#include <cstdint>

// Extraccion y extension de signo de los inmediatos de cada formato RV32I,
// usando sc_uint<N>::range()/bit() para el slicing de campos (en vez de
// shifts nativos sobre uint32_t): el hardware real hace exactamente este
// tipo de particionado bit a bit del bus de instruccion, y modelarlo asi
// deja mas clara la correspondencia con el ISA manual.
namespace rv32i {

// Extiende el signo de un valor de 'bits' bits contenido en un sc_uint<32>.
inline int32_t sign_extend(sc_dt::sc_uint<32> value, unsigned bits) {
    uint32_t shift = 32 - bits;
    return static_cast<int32_t>(value.to_uint() << shift) >> shift;
}

// I-type: usado por OP_IMM, LOAD, JALR. instr[31:20] con signo.
inline int32_t get_imm_I(sc_dt::sc_uint<32> instr) {
    return sign_extend(instr.range(31, 20), 12);
}

// S-type: usado por STORE. imm[11:5] en instr[31:25], imm[4:0] en instr[11:7].
// Partido en dos porque RISC-V mantiene rs1/rs2 en la misma posicion de
// bits en todos los formatos (R/I/S/B), asi el banco de registros se puede
// leer sin decodificar antes que tipo de instruccion es; el precio de esa
// simetria es que el inmediato queda fragmentado.
inline int32_t get_imm_S(sc_dt::sc_uint<32> instr) {
    sc_dt::sc_uint<12> imm;
    imm.range(11, 5) = instr.range(31, 25);
    imm.range(4, 0)  = instr.range(11, 7);
    return sign_extend(imm, 12);
}

// B-type: usado por BRANCH. Igual que S-type pero ademas reordenado
// (no solo fragmentado): imm[12|11|10:5|4:1], bit 0 siempre implicito en 0
// porque los branches saltan a direcciones alineadas a 2 bytes como minimo.
// El reordenamiento existe para maximizar cuantos bits coinciden
// posicionalmente con el formato J (ver get_imm_J).
inline int32_t get_imm_B(sc_dt::sc_uint<32> instr) {
    sc_dt::sc_uint<13> imm;
    imm.bit(12)      = instr.bit(31);
    imm.bit(11)      = instr.bit(7);
    imm.range(10, 5) = instr.range(30, 25);
    imm.range(4, 1)  = instr.range(11, 8);
    imm.bit(0)       = 0;
    return sign_extend(imm, 13);
}

// U-type: usado por LUI y AUIPC. instr[31:12] ya cae en los 20 bits altos
// del resultado, sin fragmentar ni reordenar: no necesita sign_extend.
inline int32_t get_imm_U(sc_dt::sc_uint<32> instr) {
    sc_dt::sc_uint<32> imm = 0;
    imm.range(31, 12) = instr.range(31, 12);
    return static_cast<int32_t>(imm.to_uint());
}

// J-type: usado por JAL. Mismo patron de reordenamiento retorcido que
// B-type pero para un rango mas amplio (21 bits, bit 0 implicito en 0).
inline int32_t get_imm_J(sc_dt::sc_uint<32> instr) {
    sc_dt::sc_uint<21> imm;
    imm.bit(20)       = instr.bit(31);
    imm.range(19, 12) = instr.range(19, 12);
    imm.bit(11)       = instr.bit(20);
    imm.range(10, 1)  = instr.range(30, 21);
    imm.bit(0)        = 0;
    return sign_extend(imm, 21);
}

// Cantidad de shift para SLLI/SRLI/SRAI: instr[24:20], sin signo (0-31,
// no necesita sign_extend). Coincide con los 5 bits bajos de get_imm_I(),
// pero se expone aparte para dejar explicito que en las variantes shift
// de OP_IMM el campo relevante es shamt, no un inmediato de 12 bits.
inline uint32_t get_shamt(sc_dt::sc_uint<32> instr) {
    return instr.range(24, 20).to_uint();
}

} // namespace rv32i

#endif // IMMEDIATES_H
