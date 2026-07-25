#ifndef FRONTEND_DISPATCH_H
#define FRONTEND_DISPATCH_H

// =====================================================================
// DECODIFICADOR Y DISPATCH (etapa 4 del pipeline).
//
// Una sola instruccion por ciclo, en orden de programa. Decide a que
// estacion de reserva va, lee sus operandos (renombrados) y reserva la
// entrada del ROB. Devuelve `true` si la instruccion pudo despacharse;
// `false` significa stall estructural (la unidad que necesita esta
// ocupada) y el fetch la reintenta el ciclo siguiente.
//
// Esta separado del tick porque es, con diferencia, la parte mas grande
// y la que mas crece al agregar instrucciones: aca vive TODA la tabla de
// decodificacion de RV32IMFC + RVV.
// =====================================================================
#include "soc_top.h"
#include "rv32i_defs.h"
#include "soc_state.h"
#include "soc_csr.h"
#include "backend.h"
#include "exec_vector.h"
#include "immediates_hls.h"
#include "vector_dispatch.h"

using namespace rv32_hls;

// `next_fetch` es de entrada/salida: JAL y los saltos resueltos en el
// dispatch lo reescriben para redirigir el fetch.
static bool dispatch_decode(ap_uint<32> instr, ap_uint<3> isize,
                            ap_uint<32> this_pc, ap_uint<3> new_tag,
                            ap_uint<32>& next_fetch) {
#pragma HLS INLINE
    uint32_t iw  = instr.to_uint();
    uint32_t opc = rv32i::opcode(iw);
    ap_uint<5> rd  = rv32i::rd(iw);
    ap_uint<3> f3  = rv32i::funct3(iw);
    ap_uint<5> rs1 = rv32i::rs1(iw);
    ap_uint<5> rs2 = rv32i::rs2(iw);
    uint32_t   f7  = rv32i::funct7(iw);
    bool can_dispatch = false;

        // OP-V se resuelve aparte (ver dispatch_rvv.h): no colisiona con
        // ningun otro opcode, asi que sacarlo al frente no altera el orden
        // de la cadena, que en el resto SI es significativo.
        if (opc == rvv::OPCODE_OP_V) {
            return dispatch_rvv(instr, new_tag, this_pc);
        }

        if (opc == rv32i::Opcode::LUI || opc == rv32i::Opcode::AUIPC ||
            opc == rv32i::Opcode::JAL) {
            // resuelven completos en el dispatch: valor conocido sin
            // leer registros (JAL ademas redirige el fetch aca mismo)
            can_dispatch = true;
            RobEntry& e = rob[new_tag];
            e.valid = true; e.is_store = false; e.dest = rd; e.ready = true;
            if (opc == rv32i::Opcode::LUI)
                e.value = ap_uint<32>(static_cast<uint32_t>(get_imm_U(instr)));
            else if (opc == rv32i::Opcode::AUIPC)
                e.value = this_pc + ap_uint<32>(static_cast<uint32_t>(get_imm_U(instr)));
            else { // JAL (un C.JAL expandido enlaza a pc+2 via isize)
                e.value = this_pc + isize;
                next_fetch = this_pc + ap_uint<32>(static_cast<uint32_t>(get_imm_J(instr)));
            }
        } else if (opc == rv32i::Opcode::OP && f7 == rv32i::Funct7::MULDIV) {
            if (!md_rs.busy) {
                can_dispatch = true;
                md_rs.busy = true; md_rs.executing = false;
                md_rs.f3 = f3; md_rs.rob_tag = new_tag;
                md_rs.s1 = read_operand(rs1);
                md_rs.s2 = read_operand(rs2);
                RobEntry& e = rob[new_tag];
                e.valid = true; e.is_store = false; e.dest = rd; e.ready = false;
            }
        } else if (opc == rv32i::Opcode::OP || opc == rv32i::Opcode::OP_IMM) {
            int free_alu = -1;
            for (int i = N_ALU - 1; i >= 0; i--) {
                if (!alu_rs[i].busy) free_alu = i;
            }
            if (free_alu >= 0) {
                can_dispatch = true;
                AluRs& rs = alu_rs[free_alu];
                rs.busy = true; rs.executing = false;
                rs.f3 = f3; rs.rob_tag = new_tag;
                rs.s1 = read_operand(rs1);
                if (opc == rv32i::Opcode::OP) {
                    rs.alt = (f7 == rv32i::Funct7::ALT);
                    rs.s2 = read_operand(rs2);
                } else {
                    // OP_IMM: el inmediato entra como operando ya listo;
                    // SRAI se distingue por el bit 30 (extension de funct7
                    // dentro del campo de inmediato, igual que en el ISA)
                    rs.alt = (f3 == rv32i::Funct3_ALU::SRL_SRA) && ((iw >> 30) & 1);
                    rs.s2.ready = true;
                    rs.s2.tag = 0;
                    if (f3 == rv32i::Funct3_ALU::SLL || f3 == rv32i::Funct3_ALU::SRL_SRA)
                        rs.s2.val = get_shamt(instr);
                    else
                        rs.s2.val = ap_uint<32>(static_cast<uint32_t>(get_imm_I(instr)));
                }
                RobEntry& e = rob[new_tag];
                e.valid = true; e.is_store = false; e.dest = rd; e.ready = false;
            }
        } else if ((opc == rv32i::Opcode::LOAD_FP || opc == rv32i::Opcode::STORE_FP) &&
                   (f3 == rvv::WIDTH_64 || (((iw >> 28) & 1) != 0))) {
            // EEW=64 (width=111) o mew=1 (>=128 bits): un load/store
            // vectorial legal en la spec, pero FUERA de Zve32x, que
            // llega hasta 32 bits. La spec (seccion 7.3) pide que un
            // ancho no soportado levante instruccion ilegal en vez de
            // ejecutarse mal en silencio -- que es justamente lo que
            // permite a un runtime detectar el perfil de la maquina.
            can_dispatch = true;
            RobEntry& e = rob[new_tag];
            e.valid = true; e.is_store = false; e.takes_trap = true;
            e.pc = this_pc; e.cause = 2;
            e.dest = 0; e.value = 0; e.ready = true;
        } else if ((opc == rv32i::Opcode::LOAD_FP || opc == rv32i::Opcode::STORE_FP) &&
                   (f3 == rvv::WIDTH_8 || f3 == rvv::WIDTH_16 || f3 == rvv::WIDTH_32) &&
                   (((iw >> 26) & 0x3) == rvv::MOP_UNIT) &&
                   !(((iw >> 20) & 0x1F) == rvv::LUMOP_UNIT  ||
                     ((iw >> 20) & 0x1F) == rvv::LUMOP_WHOLE ||
                     ((iw >> 20) & 0x1F) == rvv::LUMOP_MASK  ||
                     (((iw >> 20) & 0x1F) == rvv::LUMOP_FOF &&
                      opc == rv32i::Opcode::LOAD_FP))) {
            // lumop/sumop reservado (tablas 11 y 12). Ojo: el
            // fault-only-first SOLO existe como load -- no hay "store
            // que falla solo en el primer elemento", porque un store ya
            // escribio memoria cuando se descubre el fallo.
            can_dispatch = true;
            RobEntry& e = rob[new_tag];
            e.valid = true; e.is_store = false; e.takes_trap = true;
            e.pc = this_pc; e.cause = 2;
            e.dest = 0; e.value = 0; e.ready = true;
        } else if ((opc == rv32i::Opcode::LOAD_FP || opc == rv32i::Opcode::STORE_FP) &&
                   (f3 == rvv::WIDTH_8 || f3 == rvv::WIDTH_16 || f3 == rvv::WIDTH_32)) {
            // vle32.v / vse32.v: MISMO opcode LOAD-FP/STORE-FP que
            // FLW/FSW escalar, pero width=110 (32b vectorial) en vez
            // de width=010 (FLW/FSW) -- asi se distinguen (ver
            // rv32_ooo.h y rv32_vector.cpp para el detalle de bits,
            // verificado contra la especificacion oficial RVV v1.0).
            // Una vectorial captura vl/vstart EN EL DISPATCH, pero un
            // `csrw vstart` los escribe en la CABEZA del ROB. Si se
            // dejara pasar, la vectorial leeria el vstart VIEJO (un RAW
            // entre dos etapas distintas del pipeline). Es la simetrica
            // del stall que ya hace vset*, y por eso la cura es la misma.
// vstart != 0 apaga el desacople de la VIQ: cada vectorial que
            // completa pone vstart=0, y una posterior encolada antes
            // capturaria el valor viejo (mismo patron CSR de siempre).
                        if (!viq_full() && !sys_rs.busy && branch_pending == 0 &&
                (csr_vstart == 0 || (viq_count == 0 && !vec_rs.busy))) {
                VecRs vq = VecRs();  // se ENCOLA en la VIQ del coprocesador
                can_dispatch = true;
                vq.busy = true;
                vq.is_load = (opc == rv32i::Opcode::LOAD_FP);
                vq.is_store = !vq.is_load;
                vq.is_arith = false;
                vq.vd_or_vs3 = rd; // mismo campo de bits que vd (load) o vs3 (store)
                vq.vm = ((iw >> 25) & 1) != 0; // bit vm, igual que en aritmetica
                vq.rob_tag = new_tag;
                vq.vl = csr_vl;    // largo vectorial vigente en este punto del programa
                vq.vstart = csr_vstart; // Fase 6: primer elemento a mover
                // ---- Fase 5: modo de direccionamiento (seccion 7.2) ----
                // mop  = instr[27:26]  : unit-stride / strided / indexado
                // nf   = instr[31:29]  : campos por segmento, menos uno
                // lumop= instr[24:20]  : variantes del unit-stride
                uint32_t mop   = (iw >> 26) & 0x3;
                uint32_t lumop = (iw >> 20) & 0x1F;
                vq.mop = mop;
                vq.nf  = (iw >> 29) & 0x7;
                vq.fld = 0;
                bool unit = (mop == rvv::MOP_UNIT);
                vq.lsmode =
                    (unit && lumop == rvv::LUMOP_MASK)  ? rvv::LSM_MASK  :
                    (unit && lumop == rvv::LUMOP_WHOLE) ? rvv::LSM_WHOLE :
                    (unit && lumop == rvv::LUMOP_FOF)   ? rvv::LSM_FOF   :
                                                          rvv::LSM_NORMAL;
                // Ancho de elemento: unit-stride y strided lo toman del
                // campo `width` de la instruccion. El INDEXADO no: ahi
                // `width` es el ancho del INDICE y el dato usa el SEW de
                // vtype (seccion 7.3). Es la unica forma que mezcla dos
                // anchos en una misma instruccion.
                ap_uint<3> w_b = (f3 == rvv::WIDTH_8)  ? ap_uint<3>(1) :
                                 (f3 == rvv::WIDTH_16) ? ap_uint<3>(2) : ap_uint<3>(4);
                bool indexed = (mop == rvv::MOP_IDX_UNORD || mop == rvv::MOP_IDX_ORD);
                vq.idx_b = w_b;
                vq.sew_b = indexed
                    ? ap_uint<3>((8u << ((csr_vtype.to_uint() >> 3) & 0x7)) / 8u)
                    : w_b;
                vq.vs2 = rs2;              // indices (indexado) -- banco vectorial
                vq.s1 = read_operand(rs1); // direccion base, banco entero
                if (mop == rvv::MOP_STRIDED) {
                    vq.s2 = read_operand(rs2); // paso en BYTES, banco entero
                } else {
                    vq.s2.ready = true; vq.s2.val = 0; vq.s2.tag = 0;
                }
                RobEntry& e = rob[new_tag];
                e.valid = true; e.is_store = false; e.dest_is_fp = false;
                e.dest = 0; e.ready = false;
                viq_push(vq);
            }        } else if (opc == rv32i::Opcode::LOAD || opc == rv32i::Opcode::STORE ||
                   opc == rv32i::Opcode::LOAD_FP || opc == rv32i::Opcode::STORE_FP) {
            // FLW/FSW comparten la LSU: mismo f3 (0b010) que LW/SW, la
            // unica diferencia es a que banco va/viene el dato
            if (!lsu_rs.busy) {
                bool is_fp   = (opc == rv32i::Opcode::LOAD_FP || opc == rv32i::Opcode::STORE_FP);
                bool is_load = (opc == rv32i::Opcode::LOAD || opc == rv32i::Opcode::LOAD_FP);
                can_dispatch = true;
                lsu_rs.busy = true;
                lsu_rs.is_load = is_load;
                lsu_rs.f3 = f3; lsu_rs.rob_tag = new_tag;
                lsu_rs.s1 = read_operand(rs1); // base SIEMPRE del banco entero
                if (is_load) {
                    lsu_rs.imm = get_imm_I(instr);
                    lsu_rs.s2.ready = true; lsu_rs.s2.val = 0; lsu_rs.s2.tag = 0;
                } else {
                    lsu_rs.imm = get_imm_S(instr);
                    lsu_rs.s2 = is_fp ? read_operand_fp(rs2) : read_operand(rs2);
                }
                RobEntry& e = rob[new_tag];
                e.valid = true; e.ready = false;
                e.is_store = !is_load;
                e.dest_is_fp = is_fp && is_load;
                e.dest = is_load ? rd : ap_uint<5>(0);
            }
        } else if (opc == rv32i::Opcode::FMADD || opc == rv32i::Opcode::FMSUB ||
                   opc == rv32i::Opcode::FNMSUB || opc == rv32i::Opcode::FNMADD) {
            // familia R4: TRES operandos fuente del banco F (rs3 vive en
            // los bits [31:27], donde un R-type normal tendria funct7)
            if (!fpu_rs.busy) {
                can_dispatch = true;
                fpu_rs.busy = true; fpu_rs.executing = false;
                fpu_rs.r4op = (opc == rv32i::Opcode::FMADD)  ? 1 :
                              (opc == rv32i::Opcode::FMSUB)  ? 2 :
                              (opc == rv32i::Opcode::FNMSUB) ? 3 : 4;
                fpu_rs.f7 = 0; fpu_rs.f3 = f3; fpu_rs.rs2f = rs2;
                fpu_rs.rob_tag = new_tag;
                fpu_rs.s1 = read_operand_fp(rs1);
                fpu_rs.s2 = read_operand_fp(rs2);
                fpu_rs.s3 = read_operand_fp(rv32i::rs3(iw));
                RobEntry& e = rob[new_tag];
                e.valid = true; e.is_store = false; e.ready = false;
                e.dest_is_fp = true; e.dest = rd;
            }
        } else if (opc == rv32i::Opcode::OP_FP) {
            if (!fpu_rs.busy) {
                can_dispatch = true;
                // tres familias segun banco fuente/destino (ver
                // explain.md seccion 1): FCMP/FCVT.W/FMV.X/FCLASS
                // escriben el banco ENTERO; FCVT.S.W/FMV.W.X leen del
                // banco ENTERO; el resto es F->F puro
                bool int_dest = (f7 == rv32i::Funct7_FP::FCMP_S ||
                                 f7 == rv32i::Funct7_FP::FCVT_W_S ||
                                 f7 == rv32i::Funct7_FP::FMV_X_W_FCLASS_S);
                bool int_src  = (f7 == rv32i::Funct7_FP::FCVT_S_W ||
                                 f7 == rv32i::Funct7_FP::FMV_W_X);
                bool needs_s2 = (f7 == rv32i::Funct7_FP::FADD_S ||
                                 f7 == rv32i::Funct7_FP::FSUB_S ||
                                 f7 == rv32i::Funct7_FP::FMUL_S ||
                                 f7 == rv32i::Funct7_FP::FDIV_S ||
                                 f7 == rv32i::Funct7_FP::FSGNJ_S ||
                                 f7 == rv32i::Funct7_FP::FMINMAX_S ||
                                 f7 == rv32i::Funct7_FP::FCMP_S);
                fpu_rs.busy = true; fpu_rs.executing = false;
                fpu_rs.r4op = 0; fpu_rs.f7 = f7; fpu_rs.f3 = f3; fpu_rs.rs2f = rs2;
                fpu_rs.rob_tag = new_tag;
                fpu_rs.s1 = int_src ? read_operand(rs1) : read_operand_fp(rs1);
                if (needs_s2) {
                    fpu_rs.s2 = read_operand_fp(rs2);
                } else {
                    fpu_rs.s2.ready = true; fpu_rs.s2.val = 0; fpu_rs.s2.tag = 0;
                }
                fpu_rs.s3.ready = true; fpu_rs.s3.val = 0; fpu_rs.s3.tag = 0;
                RobEntry& e = rob[new_tag];
                e.valid = true; e.is_store = false; e.ready = false;
                e.dest_is_fp = !int_dest; e.dest = rd;
            }
        } else if (opc == rv32i::Opcode::BRANCH || opc == rv32i::Opcode::JALR) {
            if (!br_rs.busy) {
                can_dispatch = true;
                br_rs.busy = true; br_rs.executing = false;
                br_rs.is_jalr = (opc == rv32i::Opcode::JALR);
                br_rs.f3 = f3; br_rs.rob_tag = new_tag;
                br_rs.br_pc = this_pc;
                br_rs.size = isize; // link/fall-through correctos con C
                br_rs.s1 = read_operand(rs1);
                RobEntry& e = rob[new_tag];
                e.valid = true; e.is_store = false; e.ready = false;
                e.dest = br_rs.is_jalr ? rd : ap_uint<5>(0);
                if (br_rs.is_jalr) {
                    br_rs.imm = get_imm_I(instr);
                    br_rs.s2.ready = true; br_rs.s2.val = 0; br_rs.s2.tag = 0;
                    // JALR: el destino sale de un REGISTRO, y sin BTB/RAS no
                    // hay de donde predecirlo -> este si detiene el fetch
                    // hasta resolverse (decision documentada en el README).
                    fetch_stalled = true;
                } else {
                    br_rs.imm = get_imm_B(instr);
                    br_rs.s2 = read_operand(rs2);
                    // ---- ESPECULACION: TAGE decide el camino del fetch ----
                    // El branch condicional ya NO detiene el pipeline. Se
                    // predice tomado/no-tomado; el destino de un branch es
                    // pc+imm (viene EN la instruccion), asi que no hace
                    // falta BTB. La verdad se comprueba en la unidad BR y
                    // se salda en el COMMIT.
                    bool pred = tage_predict(this_pc);
                    e.is_branch  = true;
                    e.pred_taken = pred;
                    e.ghr_snap   = tage_ghr;   // para el update y el repair
                    if (pred)
                        next_fetch = this_pc +
                            ap_uint<32>(static_cast<uint32_t>(get_imm_B(instr)));
                    // historia global: se actualiza ESPECULATIVAMENTE con la
                    // prediccion (asi los branches siguientes predicen con
                    // el contexto nuevo); el commit la repara si fallo.
                    tage_ghr = (tage_ghr << 1) | ap_uint<16>(pred ? 1 : 0);
                    branch_pending = branch_pending + 1;
                }
            }
        } else if (opc == rv32i::Opcode::SYSTEM) {
            // bare-metal (items 2-5): ECALL/EBREAK (funct3=0) e
            // instrucciones CSR (funct3=1..7).
            if (f3 == rv32i::Funct3_SYSTEM::PRIV) {
                uint32_t imm12 = (iw >> 20) & 0xFFF;
                if (imm12 == 0x000 || imm12 == 0x001) {
                    // ECALL (causa 11 = "environment call from M-mode")
                    // / EBREAK (causa 3). Se marca en el ROB y el trap
                    // se toma al retirarse -> excepcion precisa.
                    can_dispatch = true;
                    RobEntry& e = rob[new_tag];
                    e.valid = true; e.is_store = false; e.takes_trap = true;
                    e.pc = this_pc;
                    e.cause = (imm12 == 0x000) ? ap_uint<5>(11) : ap_uint<5>(3);
                    e.dest = 0; e.value = 0; e.ready = true;
                } else if (imm12 == 0x302) {
                    // MRET: retorno desde el handler (pc <- mepc)
                    can_dispatch = true;
                    RobEntry& e = rob[new_tag];
                    e.valid = true; e.is_store = false; e.is_mret = true;
                    e.pc = this_pc; e.dest = 0; e.value = 0; e.ready = true;
                } else {
                    // WFI y otros privilegiados: no-op
                    can_dispatch = true;
                    RobEntry& e = rob[new_tag];
                    e.valid = true; e.is_store = false; e.dest = 0; e.value = 0; e.ready = true;
                }
            } else {
                // CSRRW/CSRRS/CSRRC (y variantes con inmediato de 5 bits)
                if (!sys_rs.busy) {
                    can_dispatch = true;
                    bool is_imm = (f3 == rv32i::Funct3_SYSTEM::CSRRWI ||
                                   f3 == rv32i::Funct3_SYSTEM::CSRRSI ||
                                   f3 == rv32i::Funct3_SYSTEM::CSRRCI);
                    sys_rs.busy = true;
                    sys_rs.f3 = f3;
                    sys_rs.csr_addr = (iw >> 20) & 0xFFF;
                    sys_rs.rob_tag = new_tag;
                    if (is_imm) {
                        // el "fuente" es el campo rs1 usado como zimm[4:0]
                        sys_rs.s1.ready = true; sys_rs.s1.val = rs1; sys_rs.s1.tag = 0;
                    } else {
                        sys_rs.s1 = read_operand(rs1);
                    }
                    RobEntry& e = rob[new_tag];
                    e.valid = true; e.is_store = false; e.dest_is_fp = false;
                    e.dest = rd; e.ready = false;
                }
                // si sys_rs.busy: can_dispatch false -> stall (serializa CSR)
            }
        } else if (opc == rv32i::Opcode::MISC_MEM) {
            // FENCE / FENCE.I: sin caches ni reordenamiento de memoria
            // real observable, se reconoce como no-op (el crt0 la emite).
            can_dispatch = true;
            RobEntry& e = rob[new_tag];
            e.valid = true; e.is_store = false; e.dest = 0; e.value = 0; e.ready = true;
        } else {
            // Opcode no soportado -> EXCEPCION DE INSTRUCCION ILEGAL
            // (causa 2). Antes se retiraba como no-op silencioso, lo
            // que hacia que un binario incompatible diera resultados
            // incorrectos SIN AVISAR. Ahora falla de forma visible y,
            // si hay handler, este puede emular la instruccion.
            can_dispatch = true;
            RobEntry& e = rob[new_tag];
            e.valid = true; e.is_store = false; e.takes_trap = true;
            e.pc = this_pc; e.cause = 2;
            e.dest = 0; e.value = 0; e.ready = true;
        }
    return can_dispatch;
}

#endif // FRONTEND_DISPATCH_H
