#ifndef SOC_CSR_H
#define SOC_CSR_H

// Banco de CSRs de modo maquina: lectura y escritura. Es el punto unico
// por el que pasa todo acceso a estado privilegiado, lo que mantiene el
// orden de programa (las instrucciones CSR ejecutan en la cabeza del ROB).
#include "soc_top.h"
#include "rv32i_defs.h"
#include "soc_state.h"

// almacenamiento simple. Una direccion no soportada lee 0 / ignora la
// escritura (comportamiento aceptable para un core bare-metal minimo).
static ap_uint<32> read_csr(ap_uint<12> addr) {
    switch (addr.to_uint()) {
        case rv32i::CSR::MSTATUS:  return csr_mstatus;
        case rv32i::CSR::MISA:     return 0x40001104; // RV32 IMFC
        case rv32i::CSR::MIE:      return csr_mie;
        case rv32i::CSR::MTVEC:    return csr_mtvec;
        case rv32i::CSR::MSCRATCH: return csr_mscratch;
        case rv32i::CSR::MEPC:     return csr_mepc;
        case rv32i::CSR::MCAUSE:   return csr_mcause;
        case rv32i::CSR::MTVAL:    return csr_mtval;
        case rv32i::CSR::MIP:      return csr_mip;
        case rv32i::CSR::MHARTID:  return 0;
        case rv32i::CSR::MTIME:    return csr_mtime;
        case rv32i::CSR::MTIMECMP: return csr_mtimecmp;
        // vstart SI es read-write (seccion 3.7); vl/vtype/vlenb son de
        // solo lectura -- los escribe vsetvl*, no csrw.
        case rv32i::CSR::VSTART:   return csr_vstart;
        case rv32i::CSR::VL:       return csr_vl;
        case rv32i::CSR::VTYPE:    return csr_vtype;
        case rv32i::CSR::VLENB:    return OOO_VEC_LANES * 4; // VLEN/8 = 16 bytes
        default:                   return 0;
    }
}

static void write_csr(ap_uint<12> addr, ap_uint<32> v) {
    switch (addr.to_uint()) {
        case rv32i::CSR::MSTATUS:  csr_mstatus = v; break;
        case rv32i::CSR::MIE:      csr_mie = v; break;
        case rv32i::CSR::MTVEC:    csr_mtvec = v; break;
        case rv32i::CSR::MSCRATCH: csr_mscratch = v; break;
        case rv32i::CSR::MEPC:     csr_mepc = v; break;
        case rv32i::CSR::MCAUSE:   csr_mcause = v; break;
        case rv32i::CSR::MTVAL:    csr_mtval = v; break;
        case rv32i::CSR::MIP:      csr_mip = v; break;
        case rv32i::CSR::MTIMECMP: csr_mtimecmp = v; break;
        // vstart es escribible por software (permite reanudar a mano y
        // que una biblioteca de hilos guarde/restaure el estado vectorial)
        case rv32i::CSR::VSTART:   csr_vstart = v & ap_uint<32>(rvv::MAX_ELEMS - 1); break;
        default:                   break; // misa/mhartid read-only, resto ignorado
    }
}

// Lee un operando fuente en el dispatch: regfile directo si el RAT no
// tiene tag pendiente; bypass desde el ROB si el productor ya calculo;
// si no, queda esperando ese tag (lo despertara cdb_broadcast). x0
// siempre es 0 y nunca se renombra.

#endif // SOC_CSR_H
