#ifndef IMMEDIATES_HLS_H
#define IMMEDIATES_HLS_H

#include <ap_int.h>
#include <cstdint>

// Espejo HLS-sintetizable de RV32IMFC_tlm/src/immediates.h: misma logica bit a bit,
// pero usando ap_uint<N> (biblioteca de Xilinx) en vez de sc_dt::sc_uint<N>
// (SystemC), para no depender de SystemC dentro del proyecto de Vitis HLS.
// Ambas APIs comparten el mismo metodo .range(hi,lo)/.bit(i), asi que la
// traduccion es casi mecanica; ver explain.md para la version comentada
// linea por linea de esta misma logica (seccion 2 y 3).
namespace rv32_hls {

inline int32_t sign_extend(ap_uint<32> value, unsigned bits) {
    uint32_t shift = 32 - bits;
    return static_cast<int32_t>(value.to_uint() << shift) >> shift;
}

inline int32_t get_imm_I(ap_uint<32> instr) {
    return sign_extend(instr.range(31, 20), 12);
}

inline int32_t get_imm_S(ap_uint<32> instr) {
    ap_uint<12> imm;
    imm.range(11, 5) = instr.range(31, 25);
    imm.range(4, 0)  = instr.range(11, 7);
    return sign_extend(imm, 12);
}

inline int32_t get_imm_B(ap_uint<32> instr) {
    ap_uint<13> imm;
    imm.bit(12)      = instr.bit(31);
    imm.bit(11)      = instr.bit(7);
    imm.range(10, 5) = instr.range(30, 25);
    imm.range(4, 1)  = instr.range(11, 8);
    imm.bit(0)       = 0;
    return sign_extend(imm, 13);
}

inline int32_t get_imm_U(ap_uint<32> instr) {
    ap_uint<32> imm = 0;
    imm.range(31, 12) = instr.range(31, 12);
    return static_cast<int32_t>(imm.to_uint());
}

inline int32_t get_imm_J(ap_uint<32> instr) {
    ap_uint<21> imm;
    imm.bit(20)       = instr.bit(31);
    imm.range(19, 12) = instr.range(19, 12);
    imm.bit(11)       = instr.bit(20);
    imm.range(10, 1)  = instr.range(30, 21);
    imm.bit(0)        = 0;
    return sign_extend(imm, 21);
}

inline uint32_t get_shamt(ap_uint<32> instr) {
    return instr.range(24, 20).to_uint();
}

} // namespace rv32_hls

#endif // IMMEDIATES_HLS_H
