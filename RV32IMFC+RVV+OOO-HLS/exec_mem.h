#ifndef EXEC_MEM_H
#define EXEC_MEM_H

// Acceso a la memoria de datos con los anchos de RV32I (byte/half/word,
// con y sin signo). La dmem es por palabras, asi que aca vive el
// desplazamiento y enmascarado dentro de la palabra.
#include "rv32_ooo.h"
#include "rv32i_defs.h"
#include "ooo_state.h"

static ap_uint<32> dmem_load(ap_uint<32> dmem[OOO_DMEM_WORDS], ap_uint<32> addr, ap_uint<3> f3) {
    ap_uint<32> word = dmem[(addr >> 2) & (OOO_DMEM_WORDS - 1)];
    ap_uint<5> shift = ap_uint<5>(addr.range(1, 0)) * 8;
    ap_uint<32> sh = word >> shift;
    switch (f3.to_uint()) {
        case rv32i::Funct3_LOAD::LB:
            return static_cast<uint32_t>(static_cast<int32_t>(static_cast<int8_t>(ap_uint<8>(sh.range(7, 0)).to_uint())));
        case rv32i::Funct3_LOAD::LH:
            return static_cast<uint32_t>(static_cast<int32_t>(static_cast<int16_t>(ap_uint<16>(sh.range(15, 0)).to_uint())));
        case rv32i::Funct3_LOAD::LW:  return word;
        case rv32i::Funct3_LOAD::LBU: return ap_uint<8>(sh.range(7, 0)).to_uint();
        case rv32i::Funct3_LOAD::LHU: return ap_uint<16>(sh.range(15, 0)).to_uint();
        default:                      return 0;
    }
}

static void dmem_store(ap_uint<32> dmem[OOO_DMEM_WORDS], ap_uint<32> addr, ap_uint<32> wdata, ap_uint<3> f3) {
    ap_uint<32> wa = (addr >> 2) & (OOO_DMEM_WORDS - 1);
    ap_uint<5> shift = ap_uint<5>(addr.range(1, 0)) * 8;
    switch (f3.to_uint()) {
        case rv32i::Funct3_STORE::SW:
            dmem[wa] = wdata;
            break;
        case rv32i::Funct3_STORE::SH: {
            ap_uint<32> word = dmem[wa];
            ap_uint<32> mask = ap_uint<32>(0xFFFF) << shift;
            dmem[wa] = (word & ~mask) | ((wdata & ap_uint<32>(0xFFFF)) << shift);
            break;
        }
        case rv32i::Funct3_STORE::SB: {
            ap_uint<32> word = dmem[wa];
            ap_uint<32> mask = ap_uint<32>(0xFF) << shift;
            dmem[wa] = (word & ~mask) | ((wdata & ap_uint<32>(0xFF)) << shift);
            break;
        }
    }
}

// Descarta todo lo que hay en vuelo y reinicia el frente del pipeline.
// En este core es SUFICIENTE para excepciones precisas: como no hay
// especulacion, un trap solo puede tomarse en la CABEZA del ROB, momento
// en que todo lo anterior ya committeo y todo lo posterior todavia no ha
// modificado el estado arquitectonico. Por eso un flush total es
// correcto y no hace falta maquinaria de recuperacion especulativa.

#endif // EXEC_MEM_H
