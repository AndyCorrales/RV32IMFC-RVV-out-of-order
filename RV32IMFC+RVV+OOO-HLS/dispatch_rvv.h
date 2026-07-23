#ifndef DISPATCH_RVV_H
#define DISPATCH_RVV_H

// =====================================================================
// DISPATCH de las instrucciones con opcode OP-V (0b1010111).
//
// Cubre las dos familias que comparten ese opcode:
//   - vsetvli / vsetivli / vsetvl  (funct3 = 111), que se resuelven en el
//     propio dispatch porque fijan vtype/vl para todo lo que sigue;
//   - la aritmetica vectorial completa, despachada por categoria
//     (VCAT_*): ALU, comparaciones, merge, logica de mascaras, unary,
//     reducciones, permutaciones y los grupos EMUL=2 de widening y
//     narrowing.
//
// Vive aparte porque OP-V NO colisiona con ningun otro opcode, asi que
// puede resolverse antes que el resto de la cadena sin alterar el orden.
// Ojo: los load/store vectoriales NO estan aca -- comparten los opcodes
// LOAD-FP/STORE-FP con FLW/FSW y su orden relativo si importa, por eso
// siguen en dispatch.h.
// =====================================================================
#include "rv32_ooo.h"
#include "rv32i_defs.h"
#include "ooo_state.h"
#include "ooo_csr.h"
#include "exec_scalar.h"
#include "exec_vector.h"
#include "immediates_hls.h"

using namespace rv32_hls;

static bool dispatch_rvv(ap_uint<32> instr, ap_uint<3> new_tag,
                         ap_uint<32> this_pc) {
#pragma HLS INLINE
    uint32_t iw  = instr.to_uint();
    ap_uint<5> rd  = rv32i::rd(iw);
    ap_uint<3> f3  = rv32i::funct3(iw);
    ap_uint<5> rs1 = rv32i::rs1(iw);
    ap_uint<5> rs2 = rv32i::rs2(iw);
    bool can_dispatch = false;

    if (f3 == rvv::FUNCT3_OPCFG) {
        // ---- vsetvli / vsetivli / vsetvl (seccion 5 de la spec) ----
        //   vsetvli  : instr[31]=0      , vtypei=instr[30:20], AVL=x[rs1]
        //   vsetivli : instr[31:30]=11  , vtypei=instr[29:20], AVL=uimm(rs1)
        //   vsetvl   : instr[31:25]=1000000, vtype=x[rs2]    , AVL=x[rs1]
        //
        // Se resuelve EN EL DISPATCH (como LUI/AUIPC/JAL): el
        // dispatch es en orden y este core no especula, asi que
        // actualizar vtype/vl aca es preciso -- y ademas garantiza
        // que toda instruccion vectorial despachada despues capture
        // el vl nuevo, sin serializar la unidad VEC.
        bool is_vsetivli = (((iw >> 30) & 0x3) == 0x3);
        bool is_vsetvl   = (((iw >> 25) & 0x7F) == 0b1000000);

        // Como vsetvl* se resuelve en el dispatch, necesita el
        // valor de x[rs1] (el AVL) YA disponible. Si el productor
        // sigue en vuelo, se detiene el dispatch y se reintenta el
        // ciclo siguiente -- un stall en orden, correcto y simple
        // (vset* es de por si semi-serializante en implementaciones
        // reales, y ocurre una vez por bucle, no en el camino
        // critico del rendimiento).
        Operand avl_op  = read_operand(rs1);
        Operand vtyp_op = read_operand(rs2);
        bool operands_ready = is_vsetivli
                            ? true
                            : (avl_op.ready && (!is_vsetvl || vtyp_op.ready));

        // ...y ademas hay que esperar a que NO quede ninguna
        // instruccion CSR en vuelo. Un `csrr x, vl` lee el banco de
        // CSRs en la CABEZA del ROB, mientras que vset* lo escribe
        // en el DISPATCH: si se dejara pasar, un vset* posterior en
        // el programa pisaria vl ANTES de que el csrr anterior
        // alcanzara a leerlo (un WAR entre dos etapas distintas del
        // pipeline). Esperar a que sys_rs se libere cierra
        // exactamente esa ventana, y no cuesta nada en la practica
        // porque leer vl explicitamente es raro.
        if (!operands_ready || sys_rs.busy) {
            can_dispatch = false; // stall: falta el AVL, o hay un CSR en vuelo
        } else {
        can_dispatch = true;

        uint32_t vtype_req;
        uint32_t avl;
        bool avl_is_max = false;   // rs1==x0 && rd!=x0 -> vl = VLMAX
        bool keep_vl    = false;   // rs1==x0 && rd==x0 -> conserva vl
        if (is_vsetivli) {
            vtype_req = (iw >> 20) & 0x3FF;   // zimm[9:0]
            avl = rs1.to_uint();              // uimm[4:0], sin caso especial
        } else if (is_vsetvl) {
            vtype_req = vtyp_op.val.to_uint(); // x[rs2]
            if (rs1 == 0) { if (rd == 0) keep_vl = true; else avl_is_max = true; }
            avl = avl_op.val.to_uint();
        } else { // vsetvli
            vtype_req = (iw >> 20) & 0x7FF;   // zimm[10:0]
            if (rs1 == 0) { if (rd == 0) keep_vl = true; else avl_is_max = true; }
            avl = avl_op.val.to_uint();
        }

        // Este core solo implementa SEW=32 / LMUL=1. Cualquier otra
        // configuracion es "no soportada" -> se activa vill y vl=0,
        // exactamente como manda la spec (seccion 3.4.4).
        uint32_t vsew  = (vtype_req >> 3) & 0x7;
        uint32_t vlmul = vtype_req & 0x7;
        // Zve32x exige EEW 8, 16 y 32; LMUL!=1 sigue fuera de
        // alcance. VLMAX = LMUL*VLEN/SEW = VLEN/SEW con LMUL=1.
        bool sew_ok = (vsew == rvv::VSEW_8 || vsew == rvv::VSEW_16 ||
                       vsew == rvv::VSEW_32);
        bool supported = sew_ok && (vlmul == rvv::VLMUL_1);
        uint32_t vlmax = supported ? (rvv::VLEN_BITS / (8u << vsew)) : 0;

        ap_uint<32> new_vl;
        if (!supported) {
            csr_vtype = (ap_uint<32>(1) << rvv::VILL_BIT); // vill, resto en cero
            new_vl = 0;
        } else {
            csr_vtype = vtype_req;
            if (keep_vl)          new_vl = csr_vl;   // vtype cambia, vl se conserva
            else if (avl_is_max)  new_vl = vlmax;
            else                  new_vl = (avl < vlmax) ? ap_uint<32>(avl)
                                                         : ap_uint<32>(vlmax);
        }
        csr_vl = new_vl;
        csr_vstart = 0;   // "all vector instructions, including
                          //  vset{i}vl{i}, reset the vstart CSR to zero

        // vsetvl* escribe en rd el vl efectivamente concedido
        RobEntry& e = rob[new_tag];
        e.valid = true; e.is_store = false; e.dest_is_fp = false;
        e.dest = rd; e.value = new_vl; e.ready = true;
        } // fin de operands_ready
} else {
        // vadd.vv/vsub.vv/vmul.vv -- funct6/funct3 verificados
        // contra la seccion 19 de la especificacion oficial RVV
        // v1.0 (misma tabla que rv32_vector.cpp).
        uint32_t f6 = (iw >> 26) & 0x3F;
        bool vm = ((iw >> 25) & 1) != 0;
        bool is_iv = (f3 == rvv::FUNCT3_OPIVV || f3 == rvv::FUNCT3_OPIVX ||
                      f3 == rvv::FUNCT3_OPIVI);
        bool is_mv = (f3 == rvv::FUNCT3_OPMVV || f3 == rvv::FUNCT3_OPMVX);
        bool supported = false;
        uint8_t vcat = rvv::VCAT_ALU;
        if (is_iv) {
            supported = (f6 == rvv::F6_VADD  || f6 == rvv::F6_VSUB  ||
                         f6 == rvv::F6_VRSUB || f6 == rvv::F6_VMINU ||
                         f6 == rvv::F6_VMIN  || f6 == rvv::F6_VMAXU ||
                         f6 == rvv::F6_VMAX  || f6 == rvv::F6_VAND  ||
                         f6 == rvv::F6_VOR   || f6 == rvv::F6_VXOR  ||
                         f6 == rvv::F6_VSLL  || f6 == rvv::F6_VSRL  ||
                         f6 == rvv::F6_VSRA);
            if (f6 == rvv::F6_VSUB  && f3 == rvv::FUNCT3_OPIVI) supported = false;
            if (f6 == rvv::F6_VRSUB && f3 == rvv::FUNCT3_OPIVV) supported = false;
            // Fase 4c: saturantes (vssubu/vssub no tienen forma .vi)
            if (f6 == rvv::F6_VSADDU || f6 == rvv::F6_VSADD) supported = true;
            if ((f6 == rvv::F6_VSSUBU || f6 == rvv::F6_VSSUB) &&
                f3 != rvv::FUNCT3_OPIVI) supported = true;
            // Fase 4b: permutaciones con forma entera (slide no tiene .vv)
            if (f6 == rvv::F6_VRGATHER ||
                ((f6 == rvv::F6_VSLIDEUP || f6 == rvv::F6_VSLIDEDOWN) &&
                 f3 != rvv::FUNCT3_OPIVV)) {
                supported = true; vcat = rvv::VCAT_PERM;
            }
            // Fase 3: comparaciones -> escriben una MASCARA
            if (f6 >= rvv::F6_VMSEQ && f6 <= rvv::F6_VMSGT) {
                supported = true; vcat = rvv::VCAT_CMP;
                if ((f6 == rvv::F6_VMSGT || f6 == rvv::F6_VMSGTU) &&
                    f3 == rvv::FUNCT3_OPIVV) supported = false;
                if ((f6 == rvv::F6_VMSLTU || f6 == rvv::F6_VMSLT) &&
                    f3 == rvv::FUNCT3_OPIVI) supported = false;
            }
            // Fase 3: vmerge (vm=0) / vmv.v.* (vm=1)
            if (f6 == rvv::F6_VMERGE) { supported = true; vcat = rvv::VCAT_MERGE; }
            // Fase 4d: narrowing (.wv/.wx/.wi) -- vs2 es de 2*SEW
            if (f6 == rvv::F6_VNSRL || f6 == rvv::F6_VNSRA ||
                f6 == rvv::F6_VNCLIPU || f6 == rvv::F6_VNCLIP) {
                supported = true; vcat = rvv::VCAT_NARROW;
            }
        } else if (is_mv) {
            supported = (f6 == rvv::F6_VMUL   || f6 == rvv::F6_VMULH  ||
                         f6 == rvv::F6_VMULHU || f6 == rvv::F6_VMULHSU||
                         f6 == rvv::F6_VDIVU  || f6 == rvv::F6_VDIV   ||
                         f6 == rvv::F6_VREMU  || f6 == rvv::F6_VREM);
            // Fase 4a: reducciones (forma .vs = OPMVV)
            if (f3 == rvv::FUNCT3_OPMVV && f6 <= rvv::F6_VREDMAX) {
                supported = true; vcat = rvv::VCAT_RED;
            }
            // Fase 4d: widening -- el destino ocupa un PAR de
            // registros (EMUL=2). Cubre sumas/restas (incluidas las
            // formas .w con el primer operando ya ancho), productos
            // y multiply-accumulate.
            if ((f6 >= rvv::F6_VWADDU && f6 <= rvv::F6_VWSUB_W) ||
                f6 == rvv::F6_VWMULU  || f6 == rvv::F6_VWMULSU ||
                f6 == rvv::F6_VWMUL   || f6 == rvv::F6_VWMACCU ||
                f6 == rvv::F6_VWMACC  || f6 == rvv::F6_VWMACCSU) {
                supported = true; vcat = rvv::VCAT_WIDE;
            }
            // vwmaccus SOLO existe en forma .vx (ver tabla seccion 19)
            if (f6 == rvv::F6_VWMACCUS && f3 == rvv::FUNCT3_OPMVX) {
                supported = true; vcat = rvv::VCAT_WIDE;
            }
            // Fase 4b: vslide1up/vslide1down (.vx) y vcompress (.vm)
            if (f3 == rvv::FUNCT3_OPMVX &&
                (f6 == rvv::F6_VSLIDEUP || f6 == rvv::F6_VSLIDEDOWN)) {
                supported = true; vcat = rvv::VCAT_PERM;
            }
            if (f3 == rvv::FUNCT3_OPMVV && f6 == rvv::F6_VCOMPRESS) {
                supported = true; vcat = rvv::VCAT_PERM;
            }
            // Fase 4c: promediados (formas .vv y .vx)
            if (f6 == rvv::F6_VAADDU || f6 == rvv::F6_VAADD ||
                f6 == rvv::F6_VASUBU || f6 == rvv::F6_VASUB) {
                supported = true; vcat = rvv::VCAT_ALU;
            }
            // Fase 3: logica entre mascaras (forma .mm = OPMVV)
            if (f3 == rvv::FUNCT3_OPMVV && f6 >= rvv::F6_VMANDNOT && f6 <= rvv::F6_VMXNOR) {
                supported = true; vcat = rvv::VCAT_MLOG;
            }
            // Fase 3: grupos unary seleccionados por el campo vs1
            if (f3 == rvv::FUNCT3_OPMVV && f6 == rvv::F6_VWXUNARY0 &&
                (rs1 == rvv::VS1_VCPOP || rs1 == rvv::VS1_VFIRST)) {
                supported = true; vcat = rvv::VCAT_XRES;
            }
            if (f3 == rvv::FUNCT3_OPMVV && f6 == rvv::F6_VMUNARY0 &&
                (rs1 == rvv::VS1_VID || rs1 == rvv::VS1_VIOTA)) {
                supported = true; vcat = rvv::VCAT_VID;
            }
        }

        // ---- Fase 4d: restricciones de los grupos EMUL=2 ----
        // Zve32x tiene ELEN=32, asi que un destino de 2*SEW solo
        // cabe si SEW<=16. Ensanchar desde SEW=32 pediria 64 bits:
        // fuera del perfil -> instruccion ilegal, no resultado malo.
        // Ademas el grupo de dos registros debe empezar en un
        // registro PAR (la spec pide especificadores legales para
        // el EMUL): en widening lo es vd, en narrowing vs2.
        {
            uint32_t sew_b_now = (8u << ((csr_vtype.to_uint() >> 3) & 0x7)) / 8u;
            if (vcat == rvv::VCAT_WIDE &&
                (sew_b_now > 2 || (rd & 1) != 0))       supported = false;
            if (vcat == rvv::VCAT_NARROW &&
                (sew_b_now > 2 || (rs2 & 1) != 0))      supported = false;
        }

        // ---- Fase 6: instrucciones que no admiten vstart != 0 ----
        // Reducciones, permutaciones, logica de mascaras y los
        // grupos unary se ejecutan de forma ATOMICA en este core:
        // nunca puede quedar un vstart intermedio en ellas. La spec
        // (seccion 3.7) permite explicitamente levantar instruccion
        // ilegal "with a value of vstart that the implementation can
        // never produce", que es justo este caso.
        if (csr_vstart != 0 &&
            (vcat == rvv::VCAT_RED  || vcat == rvv::VCAT_PERM ||
             vcat == rvv::VCAT_MLOG || vcat == rvv::VCAT_XRES ||
             vcat == rvv::VCAT_VID)) supported = false;

        if (supported) {
        // Una vectorial captura vl/vstart EN EL DISPATCH, pero un
        // `csrw vstart` los escribe en la CABEZA del ROB. Si se
        // dejara pasar, la vectorial leeria el vstart VIEJO (un RAW
        // entre dos etapas distintas del pipeline). Es la simetrica
        // del stall que ya hace vset*, y por eso la cura es la misma.
            if (!vec_rs.busy && !sys_rs.busy) {
                can_dispatch = true;
                vec_rs.busy = true;
                vec_rs.is_load = false; vec_rs.is_store = false; vec_rs.is_arith = true;
                vec_rs.vd_or_vs3 = rd;
                vec_rs.vs1 = rs1; vec_rs.vs2 = rs2; // mismos bits que rs1/rs2
                vec_rs.funct6 = f6; vec_rs.funct3 = f3; vec_rs.vm = vm;
                vec_rs.vcat = vcat; vec_rs.vs1_field = rs1;
                vec_rs.rob_tag = new_tag;
                vec_rs.vl = csr_vl;       // largo vectorial vigente al despachar
                vec_rs.vstart = csr_vstart; // Fase 6: primer elemento a calcular
                // SEW vigente segun vtype (8<<vsew = SEW en bits)
                vec_rs.sew_b = (8u << ((csr_vtype.to_uint() >> 3) & 0x7)) / 8u;
                vec_rs.executing = false; // arranca en la etapa 3, como el resto de las unidades
                // operando escalar de las formas .vx / .vi
                if (f3 == rvv::FUNCT3_OPIVX || f3 == rvv::FUNCT3_OPMVX) {
                    vec_rs.s1 = read_operand(rs1); // x[rs1], espera al CDB si hace falta
                } else if (f3 == rvv::FUNCT3_OPIVI) {
                    int32_t simm = static_cast<int32_t>(rs1.to_uint() << 27) >> 27;
                    vec_rs.s1.ready = true; vec_rs.s1.val = static_cast<uint32_t>(simm);
                    vec_rs.s1.tag = 0;
                } else {
                    vec_rs.s1.ready = true; vec_rs.s1.val = 0; vec_rs.s1.tag = 0;
                }
                RobEntry& e = rob[new_tag];
                e.valid = true; e.is_store = false; e.dest_is_fp = false;
                // vcpop.m / vfirst.m escriben el registro ENTERO rd
                e.dest = (vcat == rvv::VCAT_XRES) ? rd : ap_uint<5>(0);
                e.ready = false;
            }
            // si vec_rs.busy, can_dispatch queda false: stall correcto (unidad ocupada)
        } else {
            // variante RVV fuera de alcance -> instruccion ilegal
            can_dispatch = true;
            RobEntry& e = rob[new_tag];
            e.valid = true; e.is_store = false; e.takes_trap = true;
            e.pc = this_pc; e.cause = 2;
            e.dest = 0; e.value = 0; e.ready = true;
        }
    }
    return can_dispatch;
}

#endif // DISPATCH_RVV_H
