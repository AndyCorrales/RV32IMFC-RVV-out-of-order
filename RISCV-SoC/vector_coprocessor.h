#ifndef VECTOR_COPROCESSOR_H
#define VECTOR_COPROCESSOR_H

// =====================================================================
// COPROCESADOR VECTORIAL (bloque derecho del diagrama del SoC).
//
//   Vector Instruction Queue (VIQ)  ->  Unidades funcionales
//        FIFO de 4 entradas             VALU / VMUL / VSLDU / VLSU
//
// El core escalar DESPACHA la instruccion vectorial a la VIQ (con su
// vl/vstart/sew capturados y su tag del ROB) y sigue: no espera a que
// el coprocesador termine. Ese es el desacople del diagrama -- hasta 4
// vectoriales en vuelo mientras el escalar avanza. El ROB comun es lo
// que mantiene el retiro en orden y los traps precisos.
//
// Dentro del coprocesador la ejecucion es EN ORDEN (una instruccion en
// ejecucion a la vez, sacada de la cabeza de la VIQ) y se rutea a una
// de cuatro unidades:
//   VALU  (lat 2): aritmetica/logica, comparaciones, mascaras,
//                  reducciones, widening/narrowing
//   VMUL  (lat 4): multiplicacion y division vectorial (tambien las
//                  formas widening vwmul*/vwmacc*)
//   VSLDU (lat 2): permutaciones -- slides, gather, compress
//   VLSU        : loads/stores vectoriales; ejecuta SOLO en la cabeza
//                 del ROB (memoria en orden) y accede a traves del
//                 arbitro del interconnect (ver axi_interconnect.h)
//
// El coprocesador NO ejecuta especulativamente: vregs no esta
// renombrado, asi que el dispatch retiene vectoriales mientras haya un
// branch en vuelo (branch_pending != 0, ver frontend_dispatch.h).
// =====================================================================
#include "soc_top.h"
#include "soc_state.h"
#include "exec_vector.h"

// clasificacion por unidad (decide la LATENCIA y, en el RTL, que
// hardware se instancia)
static bool vec_is_vmul(const VecRs& u) {
    ap_uint<6> f6 = u.funct6;
    bool muldiv = (u.funct3 == rvv::FUNCT3_OPMVV || u.funct3 == rvv::FUNCT3_OPMVX) &&
                  (f6 == rvv::F6_VMUL  || f6 == rvv::F6_VMULH   ||
                   f6 == rvv::F6_VMULHU|| f6 == rvv::F6_VMULHSU ||
                   f6 == rvv::F6_VDIVU || f6 == rvv::F6_VDIV    ||
                   f6 == rvv::F6_VREMU || f6 == rvv::F6_VREM);
    bool wmul = (u.vcat == rvv::VCAT_WIDE) &&
                (f6 == rvv::F6_VWMULU || f6 == rvv::F6_VWMULSU ||
                 f6 == rvv::F6_VWMUL  || f6 == rvv::F6_VWMACCU ||
                 f6 == rvv::F6_VWMACC || f6 == rvv::F6_VWMACCSU||
                 f6 == rvv::F6_VWMACCUS);
    return muldiv || wmul;
}

// Un paso del coprocesador (llamado una vez por tick del SoC).
// `vec_done/vec_tag` son las salidas de observabilidad del top.
static void vector_coprocessor_tick(ap_uint<32> dmem[OOO_DMEM_WORDS],
                                    ap_uint<1>& vec_done, ap_uint<3>& vec_tag) {
#pragma HLS INLINE
    // ---- despachar de la VIQ a la unidad (si esta libre) ----
    if (!vec_rs.busy && viq_count != 0) {
        vec_rs = viq[viq_head];
        viq_head = viq_head + 1;
        viq_count = viq_count - 1;
        vec_rs.busy = true;
        vec_rs.executing = false;
    }

    // ---- issue: la unidad arranca cuando sus operandos existen ----
    if (vec_rs.busy && vec_rs.is_arith && !vec_rs.executing && vec_rs.s1.ready) {
        // sin operandos que esperar (lee vregs directo, sin RAT): arranca
        // en el primer ciclo de issue disponible, igual que las demas
        vec_rs.executing = true;
        // latencia por UNIDAD: VMUL es mas cara que VALU/VSLDU
        vec_rs.remaining = vec_is_vmul(vec_rs)          ? VMUL_LAT :
                           (vec_rs.vcat == rvv::VCAT_PERM) ? VSLDU_LAT :
                                                             VALU_LAT;
    }

    // ---- VALU / VMUL / VSLDU: comparten el datapath por elemento ----
    if (vec_rs.busy && vec_rs.is_arith && vec_rs.executing) {
        if (vec_rs.remaining > 0) vec_rs.remaining = vec_rs.remaining - 1;
        if (vec_rs.remaining == 0) {
            ap_uint<32> xres = vec_arith_compute(vec_rs, vregs);
            // vcpop.m / vfirst.m escriben un registro ENTERO: el valor
            // viaja por el ROB y el CDB como el de cualquier unidad.
            if (vec_rs.vcat == rvv::VCAT_XRES) {
                rob[vec_rs.rob_tag].value = xres;
                cdb_broadcast(vec_rs.rob_tag, xres);
            }
            rob[vec_rs.rob_tag].ready = true;
            vec_done = 1; vec_tag = vec_rs.rob_tag;
            vec_rs.busy = false;
            vec_rs.executing = false;
            csr_vstart = 0;   // Fase 6: toda vectorial que COMPLETA resetea vstart
        }
    }

    // ---- VLSU: memoria vectorial, a traves del ARBITRO del interconnect ----
    // Ejecuta SOLO en la cabeza del ROB (memoria en orden, igual que el
    // load escalar) y mueve UN ELEMENTO POR CICLO: cada acceso pide el
    // puerto de datos al arbitro (axi_grant) y compite con la LSU
    // escalar. `elem` y `fld` llevan el progreso entre ciclos -- la
    // instruccion permanece en la unidad hasta terminar su ultimo
    // elemento. Los cuatro modos de direccionamiento solo cambian COMO
    // se calcula la direccion:
    //
    //   unit-stride : base + (e*nf + f)*eew      -- elementos contiguos
    //   strided     : base + e*paso   + f*eew    -- paso en BYTES de x[rs2]
    //   indexado    : base + vs2[e]   + f*eew    -- offsets desde un vector
    //
    if (vec_rs.busy && (vec_rs.is_load || vec_rs.is_store) &&
        vec_rs.s1.ready && vec_rs.s2.ready &&
        vec_rs.rob_tag == rob_head && rob[rob_head].valid) {
        const ap_uint<32> base   = vec_rs.s1.val;
        const ap_uint<32> stride = vec_rs.s2.val;
        const uint32_t sb    = vec_rs.sew_b.to_uint();   // ancho del DATO
        const uint32_t ib    = vec_rs.idx_b.to_uint();   // ancho del INDICE
        const uint32_t nfld  = vec_rs.nf.to_uint() + 1;  // campos por segmento
        const uint32_t f     = vec_rs.fld.to_uint();     // campo en curso
        const uint32_t e     = vec_rs.elem.to_uint();    // elemento en curso
        const uint32_t emask = (sb == 4) ? 0xFFFFFFFFu : ((1u << (sb * 8)) - 1);
        const bool indexed   = (vec_rs.mop == rvv::MOP_IDX_UNORD ||
                                vec_rs.mop == rvv::MOP_IDX_ORD);
        bool done = false;

        if (vec_rs.lsmode == rvv::LSM_WHOLE) {
            // ---- vl<nf>r.v / vs<nf>r.v: registros COMPLETOS ----
            // Ignora vl, vtype y la mascara (por eso sirve para cambios de
            // contexto). UNA PALABRA por ciclo a traves del arbitro; `elem`
            // recorre las palabras del registro y `fld` los registros.
            if (axi_grant()) {
                uint32_t vi = (vec_rs.vd_or_vs3.to_uint() + f) & 31;
                ap_uint<32> a = base + (f * OOO_VEC_LANES + e) * 4;
                uint32_t wi = (a >> 2) & (OOO_DMEM_WORDS - 1);
                if (vec_rs.is_load) vregs[vi * OOO_VEC_LANES + e] = dmem[wi];
                else                dmem[wi] = vregs[vi * OOO_VEC_LANES + e];
                if (e + 1 < (uint32_t)OOO_VEC_LANES) {
                    vec_rs.elem = e + 1;
                } else if (f + 1 < nfld) {
                    vec_rs.fld = f + 1; vec_rs.elem = 0;
                } else done = true;
            }
        } else if (vec_rs.lsmode == rvv::LSM_MASK) {
            // ---- vlm.v / vsm.v: la MASCARA como bytes, ceil(vl/8) ----
            uint32_t evl = (vec_rs.vl.to_uint() + 7) / 8;
            if (e >= evl) {
                done = true;   // vl=0: no hay nada que mover
            } else if (axi_grant()) {
                ap_uint<32> ba = base + e;
                uint32_t wi = (ba >> 2) & (OOO_DMEM_WORDS - 1);
                int shift = (ba & 3) * 8;
                if (vec_rs.is_load) {
                    ap_uint<32> v = (dmem[wi] >> shift) & ap_uint<32>(0xFF);
                    vreg_set(vregs, vec_rs.vd_or_vs3.to_uint(), e, 1, v);
                } else {
                    ap_uint<32> v = vreg_get(vregs, vec_rs.vd_or_vs3.to_uint(), e, 1);
                    dmem[wi] = (dmem[wi] & ap_uint<32>(~(0xFFu << shift))) |
                               ((v & ap_uint<32>(0xFF)) << shift);
                }
                if (e + 1 < evl) vec_rs.elem = e + 1; else done = true;
            }
        } else {
            // ---- unit-stride / strided / indexado, con o sin segmentos ----
            uint32_t vl_eff = vec_rs.vl.to_uint();

            if (vec_rs.lsmode == rvv::LSM_FOF && e == 0 && f == 0) {
                // ---- fault-only-first: se resuelve ANTES de mover nada ----
                // Solo el elemento 0 puede atrapar; si falla uno posterior
                // se RECORTA vl y el software reintenta. El chequeo es
                // solo de rangos (no hay MMU): no gasta el puerto.
                uint32_t trim = vl_eff;
                for (int k = 0; k < rvv::MAX_ELEMS; k++) {
#pragma HLS UNROLL
                    if (k < (int)vl_eff) {
                        ap_uint<32> ba = base + k * sb;
                        bool fault = (ba.to_uint() >= OOO_DMEM_WORDS * 4u);
                        if (fault && (uint32_t)k < trim) trim = k;
                    }
                }
                if (trim == 0 && vl_eff > 0) {
                    rob[vec_rs.rob_tag].takes_trap = true;
                    rob[vec_rs.rob_tag].cause = 5;  // load access fault
                    vl_eff = 0;
                } else if (trim < vl_eff) {
                    csr_vl = trim;      // recorte visible al software
                    vl_eff = trim;
                }
                vec_rs.vl = vl_eff;     // el resto de los ciclos usa el recortado
            }

            uint32_t vst = vec_rs.vstart.to_uint();  // Fase 6
            if (e >= vl_eff) {
                // no quedan elementos en este campo del segmento
                if (f + 1 < nfld) { vec_rs.fld = f + 1; vec_rs.elem = 0; }
                else done = true;
            } else if (e < vst || !vec_elem_active(vec_rs, (int)e, vregs)) {
                vec_rs.elem = e + 1;   // inactivo: avanza sin gastar puerto
            } else if (axi_grant()) {
                // --- generador de direcciones, comun a los tres modos ---
                ap_uint<32> off;
                if (indexed)
                    off = vreg_get(vregs, vec_rs.vs2.to_uint(), (int)e, ib);
                else if (vec_rs.mop == rvv::MOP_STRIDED)
                    off = ap_uint<32>(e * stride.to_uint());
                else
                    off = ap_uint<32>(e * nfld * sb);
                ap_uint<32> ba = base + off + ap_uint<32>(f * sb);
                uint32_t wi = (ba >> 2) & (OOO_DMEM_WORDS - 1);
                int shift = (ba & 3) * 8;
                uint32_t vreg = (vec_rs.vd_or_vs3.to_uint() + f) & 31;
                if (vec_rs.is_load) {
                    ap_uint<32> v = (dmem[wi] >> shift) & ap_uint<32>(emask);
                    vreg_set(vregs, vreg, (int)e, sb, v);
                } else {
                    ap_uint<32> v = vreg_get(vregs, vreg, (int)e, sb);
                    if (sb == 4) dmem[wi] = v;
                    else dmem[wi] = (dmem[wi] & ap_uint<32>(~(emask << shift))) |
                                    ((v & ap_uint<32>(emask)) << shift);
                }
                vec_rs.elem = e + 1;
            }
            // si el arbitro nego el puerto, la unidad reintenta el mismo
            // elemento el ciclo siguiente: eso ES la contencion del bus
        }

        if (done) {
            rob[vec_rs.rob_tag].ready = true;
            vec_done = 1; vec_tag = vec_rs.rob_tag;
            vec_rs.busy = false;
            // Fase 6: toda vectorial que COMPLETA resetea vstart; si
            // ATRAPO, vstart queda apuntando al elemento culpable.
            if (!rob[vec_rs.rob_tag].takes_trap) csr_vstart = 0;
        }
    }
}

#endif // VECTOR_COPROCESSOR_H
