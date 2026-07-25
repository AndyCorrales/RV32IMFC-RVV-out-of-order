#ifndef AXI_INTERCONNECT_H
#define AXI_INTERCONNECT_H

// =====================================================================
// AXI INTERCONNECT + ARBITRO (bloque naranja del diagrama del SoC).
//
// Modela el punto donde los DOS maestros del SoC -- la LSU del core
// escalar y la VLSU del coprocesador vectorial -- comparten el camino a
// la memoria de datos de la FPGA. La memoria tiene UN puerto de datos:
// el arbitro concede UN acceso por ciclo, con prioridad fija
//
//     commit (stores en retiro)  >  VLSU  >  LSU escalar
//
// asi que un load escalar y un load vectorial en el mismo ciclo se
// serializan de verdad: el perdedor lo reintenta al ciclo siguiente.
// Por el mismo motivo la VLSU mueve UN ELEMENTO POR CICLO -- que es
// exactamente como se comporta un puerto de memoria arbitrado real, y
// no la version magica de 16 accesos simultaneos.
//
// El plano de CONTROL del SoC si es AXI de verdad: el top exporta un
// esclavo AXI4-Lite (s_axilite, ver soc_top.cpp) por el que el PS del
// Kria arranca el core y lee su estado. La memoria de instrucciones NO
// pasa por aqui: es una BRAM local del frontend (I-TCM), como muestra
// el diagrama (solo backend y coprocesador van al interconnect).
//
// Los helpers dmem_load/dmem_store hacen el acceso con los anchos de
// RV32I (byte/half/word); el desplazamiento dentro de la palabra vive
// aqui porque la memoria es por palabras.
// =====================================================================
#include "soc_top.h"
#include "rv32i_defs.h"
#include "soc_state.h"

// ---- el arbitro: una concesion por ciclo ----
static bool mem_port_used;   // lo resetea el tick al empezar cada ciclo

static bool axi_grant() {
#pragma HLS INLINE
    if (mem_port_used) return false;  // el puerto ya se uso este ciclo
    mem_port_used = true;
    return true;
}

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

#endif // AXI_INTERCONNECT_H
