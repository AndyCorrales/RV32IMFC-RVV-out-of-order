#ifndef PROCESSOR_VECTOR_UNIT_H
#define PROCESSOR_VECTOR_UNIT_H

// =====================================================================
// UNIDAD VECTORIAL de ProcessorOOO: ejecuta una instruccion RVV entera
// sobre el banco de registros vectoriales.
//
// Despacha por CATEGORIA (VCAT_*), que es lo que decide QUE destino se
// escribe y como se recorren los elementos:
//   VCAT_ALU/CMP/MERGE  elemento a elemento (destino vectorial o mascara)
//   VCAT_WIDE/NARROW    grupos EMUL=2: el operando o el destino de 2*SEW
//                       ocupa un PAR de registros
//   VCAT_MLOG           logica entre registros de mascara
//   VCAT_VID            vid.v / viota.m
//   VCAT_RED            reducciones: pliegan vs2 sobre vs1[0] -> vd[0]
//   VCAT_PERM           slides, gather y compress
//   VCAT_XRES           vcpop.m / vfirst.m, que escriben un registro ENTERO
//
// Todas arrancan en el elemento `vstart` (Fase 6) y respetan la mascara.
// La aritmetica de UN elemento vive en vector_alu.h (funciones puras).
// =====================================================================
#include "processor_ooo.h"

inline uint32_t ProcessorOOO::vec_arith_compute(const VecRs& u) {
        bool scalar_form = (u.funct3 == RVV_FUNCT3_OPIVX ||
                            u.funct3 == RVV_FUNCT3_OPIVI ||
                            u.funct3 == RVV_FUNCT3_OPMVX);
        uint32_t sew_mask = (u.sew_b == 4) ? 0xFFFFFFFFu : ((1u << (u.sew_b * 8)) - 1);

        // Fase 6: toda vectorial arranca en el elemento `vstart`, dejando
        // los anteriores SIN TOCAR (seccion 3.7). Es lo que permite
        // reanudar una instruccion a mitad de camino tras un trap.
        const int v0_ = u.vstart;

        if (u.vcat == VCAT_WIDE || u.vcat == VCAT_NARROW) {
            // ---- Fase 4d: widening / narrowing ----
            // El operando o el destino de 2*SEW ocupa un PAR de registros
            // (EMUL=2), asi que se accede con vreg_get_pair/vreg_set_pair.
            const uint8_t wb = u.sew_b * 2;
            for (int e = v0_; e < VEC_MAX_ELEMS; e++) {
                if (e >= u.vl) continue;
                if (!vec_elem_active(u, e)) continue;
                // el segundo operando SIEMPRE es de ancho SEW
                uint32_t b = scalar_form ? (u.s1.val & sew_mask)
                                         : vreg_get(u.vs1, e, u.sew_b);
                if (u.vcat == VCAT_WIDE) {
                    // las formas ".w" leen vs2 ya ancho, del par
                    bool src_wide = (u.funct6 >= RVV_F6_VWADDU_W &&
                                     u.funct6 <= RVV_F6_VWSUB_W);
                    uint32_t a  = src_wide ? vreg_get_pair(u.vs2, e, wb)
                                           : vreg_get(u.vs2, e, u.sew_b);
                    // los multiply-accumulate necesitan el vd previo (ancho)
                    uint32_t dw = vreg_get_pair(u.vd_or_vs3, e, wb);
                    vreg_set_pair(u.vd_or_vs3, e, wb,
                                  vec_wide_alu(u.funct6, a, b, u.sew_b, dw));
                } else {
                    // narrowing: vs2 es ancho, el destino es estrecho
                    uint32_t a = vreg_get_pair(u.vs2, e, wb);
                    vreg_set(u.vd_or_vs3, e, u.sew_b,
                             vec_narrow_alu(u.funct6, a, b, u.sew_b));
                }
            }
            return 0;
        }

        // --- categorias que recorren elementos de forma "normal" ---
        if (u.vcat == VCAT_ALU || u.vcat == VCAT_CMP || u.vcat == VCAT_MERGE) {
            for (int e = v0_; e < VEC_MAX_ELEMS; e++) {
                if (e >= u.vl) continue;
                // vmerge/vmv NO se predica por v0 en el sentido habitual:
                // el bit de v0 SELECCIONA la fuente en vez de inhibir la
                // escritura, asi que se procesan todos los elementos.
                if (u.vcat != VCAT_MERGE && !vec_elem_active(u, e)) continue;
                uint32_t a = vreg_get(u.vs2, e, u.sew_b);
                uint32_t b = scalar_form ? (u.s1.val & sew_mask)
                                         : vreg_get(u.vs1, e, u.sew_b);
                if (u.vcat == VCAT_ALU) {
                    vreg_set(u.vd_or_vs3, e, u.sew_b,
                             vec_alu(u.funct3, u.funct6, a, b, u.sew_b));
                } else if (u.vcat == VCAT_CMP) {
                    vmask_set(u.vd_or_vs3, e, vec_cmp(u.funct6, a, b, u.sew_b));
                } else { // VCAT_MERGE
                    // vm=1 -> vmv.v.* (copia la fuente); vm=0 -> vmerge
                    bool take_b = u.vm ? true : vmask_get(0, e);
                    vreg_set(u.vd_or_vs3, e, u.sew_b, take_b ? b : a);
                }
            }
            return 0;
        }

        // --- logica entre mascaras: opera sobre BITS, sin predicado ---
        if (u.vcat == VCAT_MLOG) {
            for (int e = 0; e < VEC_MAX_ELEMS; e++) {
                if (e >= u.vl) continue;
                vmask_set(u.vd_or_vs3, e,
                          vec_mask_logic(u.funct6, vmask_get(u.vs2, e), vmask_get(u.vs1, e)));
            }
            return 0;
        }

        // --- vid.v (indice del elemento) y viota.m (prefijo de la mascara) ---
        if (u.vcat == VCAT_VID) {
            uint32_t running = 0;
            for (int e = 0; e < VEC_MAX_ELEMS; e++) {
                if (e >= u.vl) continue;
                if (u.vs1_field == RVV_VS1_VID) {
                    if (vec_elem_active(u, e)) vreg_set(u.vd_or_vs3, e, u.sew_b, e);
                } else { // viota.m: cuenta de bits activos ANTES de este elemento
                    if (vec_elem_active(u, e)) vreg_set(u.vd_or_vs3, e, u.sew_b, running);
                    if (vmask_get(u.vs2, e)) running++;
                }
            }
            return 0;
        }

        // --- permutaciones: mueven elementos entre posiciones ---
        // Todas leen la fuente ENTERA antes de escribir: vslideup y
        // vcompress pueden tener vd y vs2 solapados, y escribir en el
        // lugar corromperia elementos que todavia falta leer.
        if (u.vcat == VCAT_PERM) {
            // Elementos que REALMENTE caben en un registro con este SEW
            // (VLEN/SEW). Leer mas alla se saldria del registro y pisaria
            // el vecino, porque el banco es un arreglo plano.
            const int n_elems = VEC_VLEN_BITS / (u.sew_b * 8);
            uint32_t src[VEC_MAX_ELEMS] = {0};
            for (int e = 0; e < n_elems; e++) src[e] = vreg_get(u.vs2, e, u.sew_b);
            bool scalar_form = (u.funct3 == RVV_FUNCT3_OPIVX ||
                                u.funct3 == RVV_FUNCT3_OPIVI ||
                                u.funct3 == RVV_FUNCT3_OPMVX);
            uint32_t off = scalar_form ? u.s1.val : 0;

            if (u.funct6 == RVV_F6_VCOMPRESS) {
                // vcompress.vm: empaqueta los elementos de vs2 cuyo bit de
                // mascara en vs1 esta en 1, en posiciones CONSECUTIVAS de
                // vd. No usa el predicado normal: vs1 ES la mascara.
                int dst = 0;
                for (int e = 0; e < n_elems; e++) {
                    if (e >= u.vl) break;
                    if (vmask_get(u.vs1, e)) vreg_set(u.vd_or_vs3, dst++, u.sew_b, src[e]);
                }
                return 0;
            }

            for (int e = 0; e < VEC_MAX_ELEMS; e++) {
                if (e >= u.vl) continue;
                if (!vec_elem_active(u, e)) continue;
                uint32_t v = 0;
                if (u.funct6 == RVV_F6_VRGATHER) {
                    // vd[i] = vs2[indice]; fuera de rango -> 0 (spec 16.4)
                    uint32_t idx = scalar_form ? off : vreg_get(u.vs1, e, u.sew_b);
                    v = (idx < uint32_t(n_elems)) ? src[idx] : 0;
                } else if (u.funct6 == RVV_F6_VSLIDEUP) {
                    if (u.funct3 == RVV_FUNCT3_OPMVX) {   // vslide1up
                        v = (e == 0) ? u.s1.val : src[e - 1];
                    } else {                               // vslideup.vx/.vi
                        if (uint32_t(e) < off) continue;   // los de abajo, sin tocar
                        v = src[e - off];
                    }
                } else if (u.funct6 == RVV_F6_VSLIDEDOWN) {
                    if (u.funct3 == RVV_FUNCT3_OPMVX) {   // vslide1down
                        v = (e == u.vl - 1) ? u.s1.val : src[e + 1];
                    } else {                               // vslidedown.vx/.vi
                        uint32_t si = e + off;
                        v = (si < uint32_t(n_elems)) ? src[si] : 0;
                    }
                }
                vreg_set(u.vd_or_vs3, e, u.sew_b, v);
            }
            return 0;
        }

        // --- reduccion: pliega los elementos activos de vs2 sobre el
        //     acumulador inicial vs1[0], y escribe SOLO vd[0] ---
        if (u.vcat == VCAT_RED) {
            uint32_t sew_mask = (u.sew_b == 4) ? 0xFFFFFFFFu : ((1u << (u.sew_b * 8)) - 1);
            uint32_t acc = vreg_get(u.vs1, 0, u.sew_b); // valor escalar inicial
            for (int e = 0; e < VEC_MAX_ELEMS; e++) {
                if (e >= u.vl) continue;
                if (!vec_elem_active(u, e)) continue;   // los inactivos no participan
                acc = vec_red_step(u.funct6, acc, vreg_get(u.vs2, e, u.sew_b), u.sew_b) & sew_mask;
            }
            if (u.vl > 0) vreg_set(u.vd_or_vs3, 0, u.sew_b, acc);
            return 0;
        }

        // --- vcpop.m / vfirst.m: resultado a un registro ENTERO ---
        if (u.vcat == VCAT_XRES) {
            uint32_t count = 0;
            int32_t  first = -1;
            for (int e = 0; e < VEC_MAX_ELEMS; e++) {
                if (e >= u.vl) continue;
                if (!vec_elem_active(u, e)) continue;   // los inactivos no cuentan
                if (vmask_get(u.vs2, e)) {
                    count++;
                    if (first < 0) first = e;
                }
            }
            return (u.vs1_field == RVV_VS1_VCPOP) ? count
                                                  : static_cast<uint32_t>(first);
        }
        return 0;
    }

#endif // PROCESSOR_VECTOR_UNIT_H
