#ifndef VECTOR_ALU_H
#define VECTOR_ALU_H

// =====================================================================
// ALU VECTORIAL: la aritmetica de UN elemento, parametrizada por SEW.
//
// Son funciones PURAS -- no tocan el estado del procesador, solo
// transforman valores. Por eso viven fuera de la clase: se leen, se
// prueban y se razonan aisladas, que es justo lo que uno quiere de la
// parte donde vive la semantica del ISA.
//
//   vsext           extension de signo desde el ancho SEW
//   vec_alu         aritmetica/logica de ancho fijo (incluye las
//                   saturantes y promediadas de la Fase 4c)
//   vec_cmp         comparaciones que producen un bit de mascara
//   vec_wide_alu    widening  (2*SEW = SEW op SEW), Fase 4d
//   vec_narrow_alu  narrowing (SEW = 2*SEW op SEW), Fase 4d
//   vec_mask_logic  logica bit a bit entre registros de mascara
//
// Lo que necesita el BANCO vectorial (vreg_get/set, vec_elem_active,
// vec_arith_compute) no esta aca: depende del estado y sigue en la clase.
// =====================================================================
#include <cstdint>
#include "rvv_encoding.h"

static int32_t vsext(uint32_t v, uint8_t sew_b) {
    if (sew_b == 1) return static_cast<int32_t>(static_cast<int8_t>(v));
    if (sew_b == 2) return static_cast<int32_t>(static_cast<int16_t>(v));
    return static_cast<int32_t>(v);
}

// ALU vectorial de un elemento, parametrizada por SEW. `a` = elemento
// de vs2, `b` = elemento de vs1 (.vv) o el escalar difundido
// (.vx/.vi). Orden de operandos del ISA: "vop.vv vd, vs2, vs1".
static uint32_t vec_alu(uint8_t f3, uint8_t f6, uint32_t a, uint32_t b, uint8_t sew_b) {
    const int  sew_bits = sew_b * 8;
    int32_t  sa = vsext(a, sew_b), sb = vsext(b, sew_b);
    uint32_t sh = b & (sew_bits - 1); // shift acotado a log2(SEW) bits
    // rangos del ancho SEW, para las operaciones saturantes
    uint32_t sew_mask  = (sew_b == 4) ? 0xFFFFFFFFu : ((1u << sew_bits) - 1);
    int32_t  sew_max_s = (sew_b == 4) ? 0x7FFFFFFF : int32_t((1u << (sew_bits - 1)) - 1);
    int32_t  sew_min_s = (sew_b == 4) ? int32_t(0x80000000) : -int32_t(1u << (sew_bits - 1));
    bool is_m = (f3 == RVV_FUNCT3_OPMVV || f3 == RVV_FUNCT3_OPMVX);
    if (!is_m) { // familias OPIVV / OPIVX / OPIVI
        switch (f6) {
            case RVV_F6_VADD:  return a + b;
            case RVV_F6_VSUB:  return a - b;
            case RVV_F6_VRSUB: return b - a; // invertido, por definicion
            case RVV_F6_VMINU: return (a < b) ? a : b;   // ya truncados a SEW
            case RVV_F6_VMIN:  return (sa < sb) ? a : b;
            case RVV_F6_VMAXU: return (a > b) ? a : b;
            case RVV_F6_VMAX:  return (sa > sb) ? a : b;
            case RVV_F6_VAND:  return a & b;
            case RVV_F6_VOR:   return a | b;
            case RVV_F6_VXOR:  return a ^ b;
            case RVV_F6_VSLL:  return a << sh;
            case RVV_F6_VSRL:  return a >> sh;                        // a ya viene sin signo
            case RVV_F6_VSRA:  return static_cast<uint32_t>(sa >> sh); // desplaza el valor con signo de SEW
            // --- saturantes: en vez de envolver, se pegan al extremo
            //     representable del ancho SEW (seccion 12 de la spec) ---
            case RVV_F6_VSADDU: {
                uint32_t r = (a + b) & sew_mask;
                return (r < a) ? sew_mask : r;            // hubo acarreo -> maximo
            }
            case RVV_F6_VSADD: {
                int32_t r = sa + sb;
                if (r > sew_max_s) return uint32_t(sew_max_s);
                if (r < sew_min_s) return uint32_t(sew_min_s) & sew_mask;
                return uint32_t(r) & sew_mask;
            }
            case RVV_F6_VSSUBU: return (a < b) ? 0u : ((a - b) & sew_mask);
            case RVV_F6_VSSUB: {
                int32_t r = sa - sb;
                if (r > sew_max_s) return uint32_t(sew_max_s);
                if (r < sew_min_s) return uint32_t(sew_min_s) & sew_mask;
                return uint32_t(r) & sew_mask;
            }
            default:           return 0;
        }
    }
    // familias OPMVV / OPMVX -- mismos casos borde que la extension M
    // escalar (division por cero, overflow INT32_MIN/-1)
    // La parte alta de vmulh* y los casos borde de division dependen
    // del ancho SEW, no de 32 bits fijos.
    int64_t  a64 = sa, b64 = sb;
    uint64_t ua64 = a, ub64 = b;
    uint32_t sew_min  = (sew_b == 4) ? 0x80000000u : (1u << (sew_bits - 1)); // INT_MIN de SEW
    switch (f6) {
        case RVV_F6_VMUL:    return (a * b) & sew_mask;
        case RVV_F6_VMULH:   return static_cast<uint32_t>(static_cast<uint64_t>(a64 * b64) >> sew_bits) & sew_mask;
        case RVV_F6_VMULHU:  return static_cast<uint32_t>((ua64 * ub64) >> sew_bits) & sew_mask;
        case RVV_F6_VMULHSU: return static_cast<uint32_t>(static_cast<uint64_t>(a64 * static_cast<int64_t>(ub64)) >> sew_bits) & sew_mask;
        case RVV_F6_VDIVU:   return (b == 0) ? sew_mask : ((a / b) & sew_mask);
        case RVV_F6_VDIV:
            if (sb == 0)                    return sew_mask;              // -1 en SEW
            if (a == sew_min && sb == -1)   return sew_min;               // overflow
            return static_cast<uint32_t>(sa / sb) & sew_mask;
        // --- promediados: (a op b) >> 1 con redondeo al par (vxrm=0
        //     por defecto). Se calculan en 64 bits para no perder el
        //     bit que se va en el desplazamiento. ---
        case RVV_F6_VAADDU: { uint64_t t = uint64_t(a) + b;              return uint32_t((t >> 1) + (t & 1)) & sew_mask; }
        case RVV_F6_VAADD:  { int64_t  t = int64_t(sa) + sb;             return uint32_t((t >> 1) + (t & 1)) & sew_mask; }
        case RVV_F6_VASUBU: { int64_t  t = int64_t(a) - int64_t(b);      return uint32_t((t >> 1) + (t & 1)) & sew_mask; }
        case RVV_F6_VASUB:  { int64_t  t = int64_t(sa) - sb;             return uint32_t((t >> 1) + (t & 1)) & sew_mask; }
        case RVV_F6_VREMU:   return (b == 0) ? a : ((a % b) & sew_mask);
        case RVV_F6_VREM:
            if (sb == 0)                    return a;
            if (a == sew_min && sb == -1)   return 0;
            return static_cast<uint32_t>(sa % sb) & sew_mask;
        default:             return 0;
    }
}

// Comparacion de un elemento (familia OPIV*): devuelve el BIT que va
// al registro de mascara destino.
static bool vec_cmp(uint8_t f6, uint32_t a, uint32_t b, uint8_t sew_b) {
    int32_t sa = vsext(a, sew_b), sb = vsext(b, sew_b);
    switch (f6) {
        case RVV_F6_VMSEQ:  return a == b;
        case RVV_F6_VMSNE:  return a != b;
        case RVV_F6_VMSLTU: return a <  b;
        case RVV_F6_VMSLT:  return sa <  sb;
        case RVV_F6_VMSLEU: return a <= b;
        case RVV_F6_VMSLE:  return sa <= sb;
        case RVV_F6_VMSGTU: return a >  b;
        case RVV_F6_VMSGT:  return sa >  sb;
        default:            return false;
    }
}

// Combina el acumulador con un elemento, segun la reduccion pedida.
static uint32_t vec_red_step(uint8_t f6, uint32_t acc, uint32_t x, uint8_t sew_b) {
    int32_t sa = vsext(acc, sew_b), sx = vsext(x, sew_b);
    switch (f6) {
        case RVV_F6_VREDSUM:  return acc + x;
        case RVV_F6_VREDAND:  return acc & x;
        case RVV_F6_VREDOR:   return acc | x;
        case RVV_F6_VREDXOR:  return acc ^ x;
        case RVV_F6_VREDMINU: return (x < acc) ? x : acc;
        case RVV_F6_VREDMIN:  return (sx < sa) ? x : acc;
        case RVV_F6_VREDMAXU: return (x > acc) ? x : acc;
        case RVV_F6_VREDMAX:  return (sx > sa) ? x : acc;
        default:              return acc;
    }
}

// Logica bit a bit entre registros de mascara (familia OPMVV).
// ---- Fase 4d: widening (2*SEW = SEW op SEW) ----
// `a` es vs2[e] y `b` es vs1[e] o el escalar, igual que en vec_alu. En
// las formas ".w" `a` YA viene ancho (2*SEW). `dw` es el vd previo, que
// solo usan los multiply-accumulate. Todo se calcula en 64 bits y se
// recorta al final: asi ningun producto de 2*SEW se pierde por el camino.
static uint32_t vec_wide_alu(uint8_t f6, uint32_t a, uint32_t b,
                             uint8_t sb, uint32_t dw) {
    const uint8_t  wb    = sb * 2;
    const uint32_t wmask = (wb == 4) ? 0xFFFFFFFFu : ((1u << (wb * 8)) - 1);
    const uint64_t ua = a, ub = b;                 // ya vienen enmascarados
    const int64_t  sa = vsext(a, sb);              // vs2 con signo (ancho SEW)
    const int64_t  sbv= vsext(b, sb);              // vs1 con signo (ancho SEW)
    const int64_t  saw= vsext(a, wb);              // vs2 ancho, con signo
    const int64_t  sdw= vsext(dw, wb);             // acumulador con signo
    const uint64_t udw= dw;
    uint64_t r;
    switch (f6) {
        // sumas y restas: la clave es COMO se extiende cada fuente
        case RVV_F6_VWADDU:   r = ua + ub;                          break;
        case RVV_F6_VWADD:    r = (uint64_t)(sa + sbv);              break;
        case RVV_F6_VWSUBU:   r = ua - ub;                          break;
        case RVV_F6_VWSUB:    r = (uint64_t)(sa - sbv);              break;
        // formas ".w": el primer operando ya es de 2*SEW
        case RVV_F6_VWADDU_W: r = ua + ub;                          break;
        case RVV_F6_VWADD_W:  r = (uint64_t)(saw + sbv);             break;
        case RVV_F6_VWSUBU_W: r = ua - ub;                          break;
        case RVV_F6_VWSUB_W:  r = (uint64_t)(saw - sbv);             break;
        // productos: SEW x SEW -> 2*SEW, sin perder los bits altos
        case RVV_F6_VWMULU:   r = ua * ub;                          break;
        case RVV_F6_VWMUL:    r = (uint64_t)(sa * sbv);              break;
        case RVV_F6_VWMULSU:  r = (uint64_t)(sa * (int64_t)ub);      break;
        // multiply-accumulate: acumulan SOBRE vd (que ya es ancho)
        case RVV_F6_VWMACCU:  r = udw + ua * ub;                    break;
        case RVV_F6_VWMACC:   r = (uint64_t)(sdw + sa * sbv);        break;
        case RVV_F6_VWMACCSU: r = (uint64_t)(sdw + sa * (int64_t)ub);break;
        case RVV_F6_VWMACCUS: r = (uint64_t)(sdw + (int64_t)ua * sbv);break;
        default:              r = 0;                                break;
    }
    return (uint32_t)(r & wmask);
}

// ---- Fase 4d: narrowing (SEW = 2*SEW op SEW) ----
// `aw` es vs2[e] con ancho 2*SEW; `b` es el desplazamiento. vnsrl/vnsra
// solo truncan; vnclip ademas REDONDEA y SATURA al rango SEW, que es la
// diferencia util: baja de precision sin envolver.
static uint32_t vec_narrow_alu(uint8_t f6, uint32_t aw, uint32_t b, uint8_t sb) {
    const uint8_t  wb    = sb * 2;
    const uint32_t nmask = (sb == 4) ? 0xFFFFFFFFu : ((1u << (sb * 8)) - 1);
    // el desplazamiento se acota a log2(2*SEW) bits, como manda la spec
    const uint32_t sh    = b & (wb * 8 - 1);
    const uint64_t uaw   = aw;
    const int64_t  saw   = vsext(aw, wb);
    switch (f6) {
        case RVV_F6_VNSRL: return (uint32_t)((uaw >> sh) & nmask);
        case RVV_F6_VNSRA: return (uint32_t)(((uint64_t)(saw >> sh)) & nmask);
        case RVV_F6_VNCLIPU: {
            // redondeo rnu: se suma el bit que se va a descartar
            uint64_t r = (sh == 0) ? uaw : ((uaw >> sh) + ((uaw >> (sh - 1)) & 1));
            return (uint32_t)(r > nmask ? nmask : r);   // satura arriba
        }
        case RVV_F6_VNCLIP: {
            int64_t r = (sh == 0) ? saw : ((saw >> sh) + ((saw >> (sh - 1)) & 1));
            const int64_t hi = (int64_t)(nmask >> 1);   // maximo con signo de SEW
            const int64_t lo = -hi - 1;                 // minimo con signo de SEW
            if (r > hi) r = hi;
            if (r < lo) r = lo;
            return (uint32_t)((uint64_t)r & nmask);
        }
        default: return 0;
    }
}

static bool vec_mask_logic(uint8_t f6, bool a, bool b) {
    switch (f6) {
        case RVV_F6_VMANDNOT: return a && !b;
        case RVV_F6_VMAND:    return a &&  b;
        case RVV_F6_VMOR:     return a ||  b;
        case RVV_F6_VMXOR:    return a !=  b;
        case RVV_F6_VMORNOT:  return a || !b;
        case RVV_F6_VMNAND:   return !(a &&  b);
        case RVV_F6_VMNOR:    return !(a ||  b);
        case RVV_F6_VMXNOR:   return a ==  b;
        default:              return false;
    }
}

#endif // VECTOR_ALU_H
