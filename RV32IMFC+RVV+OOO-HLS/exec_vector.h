#ifndef EXEC_VECTOR_H
#define EXEC_VECTOR_H

// Unidad de ejecucion VECTORIAL. Contiene el acceso al banco con ancho de
// elemento variable (incluido el acceso a PARES de registros para los
// grupos EMUL=2 de widening/narrowing), la ALU vectorial en todas sus
// variantes, y vec_arith_compute, que despacha por categoria (VCAT_*).
#include "rv32_ooo.h"
#include "rv32i_defs.h"
#include "ooo_state.h"
#include "exec_scalar.h"

static ap_uint<32> vreg_get(ap_uint<32> vregs[OOO_VEC_REGFILE_LEN],
                            uint32_t vreg, int idx, uint32_t sew_b) {
    int bit_off = idx * sew_b * 8;
    ap_uint<32> w = vregs[vreg * OOO_VEC_LANES + (bit_off / 32)];
    ap_uint<32> raw = w >> (bit_off % 32);
    if (sew_b == 1) return raw & ap_uint<32>(0xFF);
    if (sew_b == 2) return raw & ap_uint<32>(0xFFFF);
    return raw;
}
static void vreg_set(ap_uint<32> vregs[OOO_VEC_REGFILE_LEN],
                     uint32_t vreg, int idx, uint32_t sew_b, ap_uint<32> val) {
    int bit_off = idx * sew_b * 8;
    int wi = vreg * OOO_VEC_LANES + (bit_off / 32);
    int shift = bit_off % 32;
    if (sew_b == 4) { vregs[wi] = val; return; }
    uint32_t field = (1u << (sew_b * 8)) - 1;
    vregs[wi] = (vregs[wi] & ap_uint<32>(~(field << shift))) |
                ((val & ap_uint<32>(field)) << shift);
}
// ---- Fase 4d: acceso a un GRUPO de dos registros (EMUL=2) ----
// Un operando widening/narrowing de 2*SEW ocupa el par (vreg, vreg+1). Con
// elementos de `wide_b` bytes caben VLEN/(wide_b*8) por registro, asi que
// el elemento `idx` vive en el registro vreg + idx/por_reg. El
// almacenamiento no cambia: es el MISMO banco plano, solo cambia el indice
// -- exactamente la misma idea que se uso para el EEW variable en la Fase 2.
static ap_uint<32> vreg_get_pair(ap_uint<32> vregs[OOO_VEC_REGFILE_LEN],
                                 uint32_t vreg, int idx, uint32_t wide_b) {
    int por_reg = rvv::VLEN_BITS / (wide_b * 8);
    return vreg_get(vregs, (vreg + (idx / por_reg)) & 31, idx % por_reg, wide_b);
}
static void vreg_set_pair(ap_uint<32> vregs[OOO_VEC_REGFILE_LEN],
                          uint32_t vreg, int idx, uint32_t wide_b, ap_uint<32> val) {
    int por_reg = rvv::VLEN_BITS / (wide_b * 8);
    vreg_set(vregs, (vreg + (idx / por_reg)) & 31, idx % por_reg, wide_b, val);
}
// extension de signo desde el ancho SEW hacia 32 bits
static int32_t vsext(uint32_t v, uint32_t sew_b) {
    if (sew_b == 1) return static_cast<int32_t>(static_cast<int8_t>(v));
    if (sew_b == 2) return static_cast<int32_t>(static_cast<int16_t>(v));
    return static_cast<int32_t>(v);
}

// ALU vectorial de un elemento, parametrizada por SEW. `a` = elemento de
// vs2, `b` = elemento de vs1 (.vv) o el escalar difundido (.vx/.vi).
// Orden de operandos del ISA: "vop.vv vd, vs2, vs1".
static ap_uint<32> vec_alu(ap_uint<3> f3, ap_uint<6> f6, ap_uint<32> a, ap_uint<32> b,
                           uint32_t sew_b) {
    const int sew_bits = sew_b * 8;
    int32_t  sa = vsext(a.to_uint(), sew_b);
    int32_t  sb = vsext(b.to_uint(), sew_b);
    uint32_t ua = a.to_uint(), ub = b.to_uint();
    ap_uint<5> sh = b & ap_uint<32>(sew_bits - 1); // shift acotado a log2(SEW)
    // rangos del ancho SEW, para las operaciones saturantes
    uint32_t sew_maskv = (sew_b == 4) ? 0xFFFFFFFFu : ((1u << sew_bits) - 1);
    int32_t  sew_max_s = (sew_b == 4) ? 0x7FFFFFFF : int32_t((1u << (sew_bits - 1)) - 1);
    int32_t  sew_min_s = (sew_b == 4) ? int32_t(0x80000000) : -int32_t(1u << (sew_bits - 1));
    bool is_m = (f3 == rvv::FUNCT3_OPMVV || f3 == rvv::FUNCT3_OPMVX);
    if (!is_m) {
        switch (f6.to_uint()) {
            case rvv::F6_VADD:  return a + b;
            case rvv::F6_VSUB:  return a - b;
            case rvv::F6_VRSUB: return b - a;
            case rvv::F6_VMINU: return (ua < ub) ? a : b;
            case rvv::F6_VMIN:  return (sa < sb) ? a : b;
            case rvv::F6_VMAXU: return (ua > ub) ? a : b;
            case rvv::F6_VMAX:  return (sa > sb) ? a : b;
            case rvv::F6_VAND:  return a & b;
            case rvv::F6_VOR:   return a | b;
            case rvv::F6_VXOR:  return a ^ b;
            case rvv::F6_VSLL:  return a << sh;
            case rvv::F6_VSRL:  return a >> sh;
            case rvv::F6_VSRA:  return ap_uint<32>(static_cast<uint32_t>(sa >> sh.to_uint()));
            // --- saturantes: se pegan al extremo representable de SEW ---
            case rvv::F6_VSADDU: { uint32_t r = (ua + ub) & sew_maskv;
                                   return ap_uint<32>((r < ua) ? sew_maskv : r); }
            case rvv::F6_VSADD:  { int32_t r = sa + sb;
                                   if (r > sew_max_s) return ap_uint<32>(uint32_t(sew_max_s));
                                   if (r < sew_min_s) return ap_uint<32>(uint32_t(sew_min_s) & sew_maskv);
                                   return ap_uint<32>(uint32_t(r) & sew_maskv); }
            case rvv::F6_VSSUBU: return ap_uint<32>((ua < ub) ? 0u : ((ua - ub) & sew_maskv));
            case rvv::F6_VSSUB:  { int32_t r = sa - sb;
                                   if (r > sew_max_s) return ap_uint<32>(uint32_t(sew_max_s));
                                   if (r < sew_min_s) return ap_uint<32>(uint32_t(sew_min_s) & sew_maskv);
                                   return ap_uint<32>(uint32_t(r) & sew_maskv); }
            default:            return 0;
        }
    }
    // La parte alta de vmulh* y los casos borde de division dependen del
    // ancho SEW, no de 32 bits fijos.
    int64_t  a64 = sa, b64 = sb;
    uint64_t ua64 = ua, ub64 = ub;
    uint32_t sew_mask = (sew_b == 4) ? 0xFFFFFFFFu : ((1u << sew_bits) - 1);
    uint32_t sew_min  = (sew_b == 4) ? 0x80000000u : (1u << (sew_bits - 1));
    switch (f6.to_uint()) {
        case rvv::F6_VMUL:    return ap_uint<32>((ua * ub) & sew_mask);
        case rvv::F6_VMULH:   return ap_uint<32>(static_cast<uint32_t>(static_cast<uint64_t>(a64 * b64) >> sew_bits) & sew_mask);
        case rvv::F6_VMULHU:  return ap_uint<32>(static_cast<uint32_t>((ua64 * ub64) >> sew_bits) & sew_mask);
        case rvv::F6_VMULHSU: return ap_uint<32>(static_cast<uint32_t>(static_cast<uint64_t>(a64 * static_cast<int64_t>(ub64)) >> sew_bits) & sew_mask);
        case rvv::F6_VDIVU:   return (ub == 0) ? ap_uint<32>(sew_mask) : ap_uint<32>((ua / ub) & sew_mask);
        case rvv::F6_VDIV:
            if (sb == 0)                   return sew_mask;
            if (ua == sew_min && sb == -1) return sew_min;
            return ap_uint<32>(static_cast<uint32_t>(sa / sb) & sew_mask);
        // --- promediados: (a op b)>>1 con redondeo (vxrm=rnu por defecto).
        //     Se calculan en 64 bits para no perder el bit desplazado. ---
        case rvv::F6_VAADDU: { uint64_t t = uint64_t(ua) + ub;         return ap_uint<32>(uint32_t((t >> 1) + (t & 1)) & sew_mask); }
        case rvv::F6_VAADD:  { int64_t  t = int64_t(sa) + sb;          return ap_uint<32>(uint32_t((t >> 1) + (t & 1)) & sew_mask); }
        case rvv::F6_VASUBU: { int64_t  t = int64_t(ua) - int64_t(ub); return ap_uint<32>(uint32_t((t >> 1) + (t & 1)) & sew_mask); }
        case rvv::F6_VASUB:  { int64_t  t = int64_t(sa) - sb;          return ap_uint<32>(uint32_t((t >> 1) + (t & 1)) & sew_mask); }
        case rvv::F6_VREMU:   return (ub == 0) ? a : ap_uint<32>((ua % ub) & sew_mask);
        case rvv::F6_VREM:
            if (sb == 0)                   return a;
            if (ua == sew_min && sb == -1) return 0;
            return ap_uint<32>(static_cast<uint32_t>(sa % sb) & sew_mask);
        default:              return 0;
    }
}

// ---- acceso a REGISTROS DE MASCARA ----
// Un registro de mascara guarda un bit por elemento: el bit del elemento
// i vive en el bit i del registro, independiente de SEW (spec 4.5).
static bool vmask_get(ap_uint<32> vregs[OOO_VEC_REGFILE_LEN], uint32_t vreg, int idx) {
    return ((vregs[vreg * OOO_VEC_LANES + (idx / 32)].to_uint() >> (idx % 32)) & 1u) != 0;
}
static void vmask_set(ap_uint<32> vregs[OOO_VEC_REGFILE_LEN], uint32_t vreg, int idx, bool v) {
    int wi = vreg * OOO_VEC_LANES + (idx / 32);
    ap_uint<32> bit = ap_uint<32>(1) << (idx % 32);
    vregs[wi] = v ? (vregs[wi] | bit) : (vregs[wi] & ~bit);
}

// Comparacion de un elemento (OPIV*): devuelve el BIT de la mascara.
static bool vec_cmp(ap_uint<6> f6, ap_uint<32> a, ap_uint<32> b, uint32_t sew_b) {
    uint32_t ua = a.to_uint(), ub = b.to_uint();
    int32_t sa = vsext(ua, sew_b), sb = vsext(ub, sew_b);
    switch (f6.to_uint()) {
        case rvv::F6_VMSEQ:  return ua == ub;
        case rvv::F6_VMSNE:  return ua != ub;
        case rvv::F6_VMSLTU: return ua <  ub;
        case rvv::F6_VMSLT:  return sa <  sb;
        case rvv::F6_VMSLEU: return ua <= ub;
        case rvv::F6_VMSLE:  return sa <= sb;
        case rvv::F6_VMSGTU: return ua >  ub;
        case rvv::F6_VMSGT:  return sa >  sb;
        default:             return false;
    }
}

// ---- Fase 4d: widening (2*SEW = SEW op SEW) ----
// `a` es vs2[e] y `b` es vs1[e] o el escalar, igual que en vec_alu. En las
// formas ".w" `a` YA viene ancho (2*SEW). `dw` es el vd previo, que solo
// usan los multiply-accumulate. Todo se calcula en 64 bits y se recorta al
// final: asi ningun producto de 2*SEW se pierde por el camino.
static ap_uint<32> vec_wide_alu(ap_uint<6> f6, ap_uint<32> a, ap_uint<32> b,
                                uint32_t sb, ap_uint<32> dw) {
    const uint32_t wb    = sb * 2;                       // ancho del destino
    const uint32_t wmask = (wb == 4) ? 0xFFFFFFFFu : ((1u << (wb * 8)) - 1);
    const uint64_t ua = a.to_uint(), ub = b.to_uint();   // ya vienen enmascarados
    const int64_t  sa = vsext(a.to_uint(), sb);          // vs2 con signo (ancho SEW)
    const int64_t  sbv= vsext(b.to_uint(), sb);          // vs1 con signo (ancho SEW)
    const uint64_t uaw = ua;                             // vs2 ancho, sin signo
    const int64_t  saw = vsext(a.to_uint(), wb);         // vs2 ancho, con signo
    const int64_t  sdw = vsext(dw.to_uint(), wb);        // acumulador con signo
    const uint64_t udw = dw.to_uint();
    uint64_t r;
    switch (f6.to_uint()) {
        // sumas y restas: la clave es COMO se extiende cada fuente
        case rvv::F6_VWADDU:   r = ua  + ub;                    break;
        case rvv::F6_VWADD:    r = (uint64_t)(sa + sbv);        break;
        case rvv::F6_VWSUBU:   r = ua  - ub;                    break;
        case rvv::F6_VWSUB:    r = (uint64_t)(sa - sbv);        break;
        // formas ".w": el primer operando ya es de 2*SEW
        case rvv::F6_VWADDU_W: r = uaw + ub;                    break;
        case rvv::F6_VWADD_W:  r = (uint64_t)(saw + sbv);       break;
        case rvv::F6_VWSUBU_W: r = uaw - ub;                    break;
        case rvv::F6_VWSUB_W:  r = (uint64_t)(saw - sbv);       break;
        // productos: SEW x SEW -> 2*SEW, sin perder bits altos
        case rvv::F6_VWMULU:   r = ua * ub;                     break;
        case rvv::F6_VWMUL:    r = (uint64_t)(sa * sbv);        break;
        case rvv::F6_VWMULSU:  r = (uint64_t)(sa * (int64_t)ub); break; // vs2 con signo
        // multiply-accumulate: acumulan SOBRE vd (que ya es ancho)
        case rvv::F6_VWMACCU:  r = udw + ua * ub;               break;
        case rvv::F6_VWMACC:   r = (uint64_t)(sdw + sa * sbv);  break;
        case rvv::F6_VWMACCSU: r = (uint64_t)(sdw + sa * (int64_t)ub); break;
        case rvv::F6_VWMACCUS: r = (uint64_t)(sdw + (int64_t)ua * sbv); break;
        default:               r = 0;                           break;
    }
    return ap_uint<32>((uint32_t)(r & wmask));
}

// ---- Fase 4d: narrowing (SEW = 2*SEW op SEW) ----
// `aw` es vs2[e] con ancho 2*SEW; `b` es el desplazamiento (vs1/escalar/imm).
// vnsrl/vnsra solo truncan; vnclip ademas REDONDEA y SATURA al rango SEW,
// que es la diferencia util: sirve para bajar de precision sin envolver.
static ap_uint<32> vec_narrow_alu(ap_uint<6> f6, ap_uint<32> aw, ap_uint<32> b,
                                  uint32_t sb) {
    const uint32_t wb    = sb * 2;
    const uint32_t nmask = (sb == 4) ? 0xFFFFFFFFu : ((1u << (sb * 8)) - 1);
    // el desplazamiento se acota a log2(2*SEW) bits, como manda la spec
    const uint32_t sh    = b.to_uint() & (wb * 8 - 1);
    const uint64_t uaw   = aw.to_uint();
    const int64_t  saw   = vsext(aw.to_uint(), wb);
    switch (f6.to_uint()) {
        case rvv::F6_VNSRL: return ap_uint<32>((uint32_t)((uaw >> sh) & nmask));
        case rvv::F6_VNSRA: return ap_uint<32>((uint32_t)(((uint64_t)(saw >> sh)) & nmask));
        case rvv::F6_VNCLIPU: {
            // redondeo rnu: se suma el bit que se va a descartar
            uint64_t r = (sh == 0) ? uaw : ((uaw >> sh) + ((uaw >> (sh - 1)) & 1));
            return ap_uint<32>((uint32_t)(r > nmask ? nmask : r)); // satura arriba
        }
        case rvv::F6_VNCLIP: {
            int64_t r = (sh == 0) ? saw : ((saw >> sh) + ((saw >> (sh - 1)) & 1));
            const int64_t hi = (int64_t)(nmask >> 1);       // maximo con signo de SEW
            const int64_t lo = -hi - 1;                     // minimo con signo de SEW
            if (r > hi) r = hi;
            if (r < lo) r = lo;
            return ap_uint<32>((uint32_t)((uint64_t)r & nmask));
        }
        default: return 0;
    }
}

// Logica bit a bit entre registros de mascara (OPMVV).
static bool vec_mask_logic(ap_uint<6> f6, bool a, bool b) {
    switch (f6.to_uint()) {
        case rvv::F6_VMANDNOT: return a && !b;
        case rvv::F6_VMAND:    return a &&  b;
        case rvv::F6_VMOR:     return a ||  b;
        case rvv::F6_VMXOR:    return a !=  b;
        case rvv::F6_VMORNOT:  return a || !b;
        case rvv::F6_VMNAND:   return !(a &&  b);
        case rvv::F6_VMNOR:    return !(a ||  b);
        case rvv::F6_VMXNOR:   return a ==  b;
        default:               return false;
    }
}

// Predicado por elemento: activo si vm=1 (sin mascara) o si el bit
// correspondiente de v0 esta en 1 (spec 4.5: el bit del elemento i vive
// en el bit i del registro de mascara).
static bool vec_elem_active(const VecRs& u, int lane, ap_uint<32> vregs[OOO_VEC_REGFILE_LEN]) {
    if (u.vm) return true;
    return ((vregs[0].to_uint() >> lane) & 1u) != 0;
}

// Solo los primeros `vl` elementos y solo los ACTIVOS por mascara; el
// resto (tail e inactivos) queda sin tocar -- politicas tail-undisturbed
// y mask-undisturbed.
// Devuelve el resultado ESCALAR de vcpop.m / vfirst.m (VCAT_XRES); para
// el resto devuelve 0.
static ap_uint<32> vec_arith_compute(const VecRs& u, ap_uint<32> vregs[OOO_VEC_REGFILE_LEN]) {
    uint32_t sb = u.sew_b.to_uint();
    bool scalar_form = (u.funct3 == rvv::FUNCT3_OPIVX ||
                        u.funct3 == rvv::FUNCT3_OPIVI ||
                        u.funct3 == rvv::FUNCT3_OPMVX);
    uint32_t sew_mask = (sb == 4) ? 0xFFFFFFFFu : ((1u << (sb * 8)) - 1);

    // Fase 6: toda vectorial arranca en el elemento `vstart`, dejando los
    // anteriores SIN TOCAR (seccion 3.7). Es lo que permite reanudar una
    // instruccion a mitad de camino despues de un trap.
    const uint32_t v0_ = u.vstart.to_uint();

    if (u.vcat == rvv::VCAT_WIDE || u.vcat == rvv::VCAT_NARROW) {
        // ---- Fase 4d: widening / narrowing ----
        // El operando o el destino de 2*SEW ocupa un PAR de registros
        // (EMUL=2), asi que se accede con vreg_get_pair/vreg_set_pair.
        const uint32_t wb = sb * 2;
        for (int e = 0; e < rvv::MAX_ELEMS; e++) {
#pragma HLS UNROLL
            if (e >= (int)v0_ && e < (int)u.vl.to_uint() && vec_elem_active(u, e, vregs)) {
                // el segundo operando SIEMPRE es de ancho SEW
                ap_uint<32> b = scalar_form ? (u.s1.val & ap_uint<32>(sew_mask))
                                            : vreg_get(vregs, u.vs1.to_uint(), e, sb);
                if (u.vcat == rvv::VCAT_WIDE) {
                    // las formas ".w" leen vs2 ya ancho, del par
                    bool src_wide = (u.funct6 >= rvv::F6_VWADDU_W &&
                                     u.funct6 <= rvv::F6_VWSUB_W);
                    ap_uint<32> a = src_wide
                        ? vreg_get_pair(vregs, u.vs2.to_uint(), e, wb)
                        : vreg_get(vregs, u.vs2.to_uint(), e, sb);
                    // los multiply-accumulate necesitan el vd previo (ancho)
                    ap_uint<32> dw = vreg_get_pair(vregs, u.vd_or_vs3.to_uint(), e, wb);
                    vreg_set_pair(vregs, u.vd_or_vs3.to_uint(), e, wb,
                                  vec_wide_alu(u.funct6, a, b, sb, dw));
                } else {
                    // narrowing: vs2 es ancho, el destino es estrecho
                    ap_uint<32> a = vreg_get_pair(vregs, u.vs2.to_uint(), e, wb);
                    vreg_set(vregs, u.vd_or_vs3.to_uint(), e, sb,
                             vec_narrow_alu(u.funct6, a, b, sb));
                }
            }
        }
        return 0;
    }

    if (u.vcat == rvv::VCAT_ALU || u.vcat == rvv::VCAT_CMP || u.vcat == rvv::VCAT_MERGE) {
        for (int e = 0; e < rvv::MAX_ELEMS; e++) {
#pragma HLS UNROLL
            // vmerge/vmv no se predica: el bit de v0 SELECCIONA la fuente
            bool go = (e >= (int)v0_) && (e < u.vl.to_uint()) &&
                      (u.vcat == rvv::VCAT_MERGE || vec_elem_active(u, e, vregs));
            if (go) {
                ap_uint<32> a = vreg_get(vregs, u.vs2.to_uint(), e, sb);
                ap_uint<32> b = scalar_form ? (u.s1.val & ap_uint<32>(sew_mask))
                                            : vreg_get(vregs, u.vs1.to_uint(), e, sb);
                if (u.vcat == rvv::VCAT_ALU) {
                    vreg_set(vregs, u.vd_or_vs3.to_uint(), e, sb,
                             vec_alu(u.funct3, u.funct6, a, b, sb));
                } else if (u.vcat == rvv::VCAT_CMP) {
                    vmask_set(vregs, u.vd_or_vs3.to_uint(), e, vec_cmp(u.funct6, a, b, sb));
                } else {
                    bool take_b = u.vm ? true : vmask_get(vregs, 0, e);
                    vreg_set(vregs, u.vd_or_vs3.to_uint(), e, sb, take_b ? b : a);
                }
            }
        }
        return 0;
    }

    if (u.vcat == rvv::VCAT_MLOG) { // logica entre mascaras, sin predicado
        for (int e = 0; e < rvv::MAX_ELEMS; e++) {
#pragma HLS UNROLL
            if (e < u.vl.to_uint())
                vmask_set(vregs, u.vd_or_vs3.to_uint(), e,
                          vec_mask_logic(u.funct6, vmask_get(vregs, u.vs2.to_uint(), e),
                                                    vmask_get(vregs, u.vs1.to_uint(), e)));
        }
        return 0;
    }

    if (u.vcat == rvv::VCAT_VID) { // vid.v / viota.m
        uint32_t running = 0;
        for (int e = 0; e < rvv::MAX_ELEMS; e++) {
            if (e >= u.vl.to_uint()) continue;
            if (u.vs1_field == rvv::VS1_VID) {
                if (vec_elem_active(u, e, vregs))
                    vreg_set(vregs, u.vd_or_vs3.to_uint(), e, sb, e);
            } else { // viota.m: prefijo de bits activos ANTES del elemento
                if (vec_elem_active(u, e, vregs))
                    vreg_set(vregs, u.vd_or_vs3.to_uint(), e, sb, running);
                if (vmask_get(vregs, u.vs2.to_uint(), e)) running++;
            }
        }
        return 0;
    }

    if (u.vcat == rvv::VCAT_RED) { // reduccion: pliega vs2 sobre vs1[0] -> vd[0]
        uint32_t sew_mask = (sb == 4) ? 0xFFFFFFFFu : ((1u << (sb * 8)) - 1);
        uint32_t acc = vreg_get(vregs, u.vs1.to_uint(), 0, sb).to_uint();
        for (int e = 0; e < rvv::MAX_ELEMS; e++) {
            if (e >= u.vl.to_uint()) continue;
            if (!vec_elem_active(u, e, vregs)) continue;   // los inactivos no participan
            uint32_t x = vreg_get(vregs, u.vs2.to_uint(), e, sb).to_uint();
            int32_t sacc = vsext(acc, sb), sx = vsext(x, sb);
            switch (u.funct6.to_uint()) {
                case rvv::F6_VREDSUM:  acc = acc + x;                break;
                case rvv::F6_VREDAND:  acc = acc & x;                break;
                case rvv::F6_VREDOR:   acc = acc | x;                break;
                case rvv::F6_VREDXOR:  acc = acc ^ x;                break;
                case rvv::F6_VREDMINU: acc = (x < acc) ? x : acc;    break;
                case rvv::F6_VREDMIN:  acc = (sx < sacc) ? x : acc;  break;
                case rvv::F6_VREDMAXU: acc = (x > acc) ? x : acc;    break;
                case rvv::F6_VREDMAX:  acc = (sx > sacc) ? x : acc;  break;
                default: break;
            }
            acc &= sew_mask;
        }
        if (u.vl.to_uint() > 0) vreg_set(vregs, u.vd_or_vs3.to_uint(), 0, sb, acc);
        return 0;
    }

    if (u.vcat == rvv::VCAT_PERM) { // slides / gather / compress
        // Elementos que REALMENTE caben con este SEW (VLEN/SEW): leer mas
        // alla se saldria del registro y pisaria el vecino.
        const int n_elems = rvv::VLEN_BITS / (sb * 8);
        // Se lee la fuente ENTERA antes de escribir: vslideup y vcompress
        // pueden tener vd y vs2 solapados.
        ap_uint<32> src[rvv::MAX_ELEMS];
        for (int e = 0; e < rvv::MAX_ELEMS; e++)
            src[e] = (e < n_elems) ? vreg_get(vregs, u.vs2.to_uint(), e, sb) : ap_uint<32>(0);
        uint32_t off = scalar_form ? u.s1.val.to_uint() : 0;

        if (u.funct6 == rvv::F6_VCOMPRESS) {
            // empaqueta los elementos cuyo bit en vs1 esta en 1, en
            // posiciones CONSECUTIVAS de vd (vs1 ES la mascara)
            int dst = 0;
            for (int e = 0; e < rvv::MAX_ELEMS; e++) {
                if (e >= n_elems || e >= (int)u.vl.to_uint()) break;
                if (vmask_get(vregs, u.vs1.to_uint(), e))
                    vreg_set(vregs, u.vd_or_vs3.to_uint(), dst++, sb, src[e]);
            }
            return 0;
        }
        for (int e = 0; e < rvv::MAX_ELEMS; e++) {
            if (e >= (int)u.vl.to_uint()) continue;
            if (!vec_elem_active(u, e, vregs)) continue;
            ap_uint<32> v = 0;
            bool escribir = true;
            if (u.funct6 == rvv::F6_VRGATHER) {
                uint32_t idx = scalar_form ? off
                             : vreg_get(vregs, u.vs1.to_uint(), e, sb).to_uint();
                v = (idx < uint32_t(n_elems)) ? src[idx] : ap_uint<32>(0);
            } else if (u.funct6 == rvv::F6_VSLIDEUP) {
                if (u.funct3 == rvv::FUNCT3_OPMVX) {        // vslide1up
                    v = (e == 0) ? u.s1.val : src[e - 1];
                } else if (uint32_t(e) < off) {
                    escribir = false;                       // los de abajo, sin tocar
                } else {
                    v = src[e - off];
                }
            } else if (u.funct6 == rvv::F6_VSLIDEDOWN) {
                if (u.funct3 == rvv::FUNCT3_OPMVX) {        // vslide1down
                    v = (e == (int)u.vl.to_uint() - 1) ? u.s1.val : src[e + 1];
                } else {
                    uint32_t si = e + off;
                    v = (si < uint32_t(n_elems)) ? src[si] : ap_uint<32>(0);
                }
            }
            if (escribir) vreg_set(vregs, u.vd_or_vs3.to_uint(), e, sb, v);
        }
        return 0;
    }

    if (u.vcat == rvv::VCAT_XRES) { // vcpop.m / vfirst.m -> registro entero
        uint32_t count = 0;
        int32_t  first = -1;
        for (int e = 0; e < rvv::MAX_ELEMS; e++) {
            if (e >= u.vl.to_uint()) continue;
            if (!vec_elem_active(u, e, vregs)) continue;
            if (vmask_get(vregs, u.vs2.to_uint(), e)) {
                count++;
                if (first < 0) first = e;
            }
        }
        return (u.vs1_field == rvv::VS1_VCPOP) ? ap_uint<32>(count)
                                               : ap_uint<32>(static_cast<uint32_t>(first));
    }
    return 0;
}


#endif // EXEC_VECTOR_H
