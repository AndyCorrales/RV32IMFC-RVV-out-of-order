#ifndef PROCESSOR_DISPATCH_H
#define PROCESSOR_DISPATCH_H

// =====================================================================
// DECODIFICADOR Y DISPATCH (etapa 4) de ProcessorOOO.
//
// Una instruccion por ciclo, en orden de programa: decide a que estacion
// de reserva va, lee sus operandos (renombrados) y reserva la entrada del
// ROB. Aca vive TODA la tabla de decodificacion de RV32IMFC + RVV, que es
// la parte que mas crece al agregar instrucciones.
//
// El ORDEN de la cadena de if/else importa: las vectoriales de memoria
// deben comprobarse ANTES que FLW/FSW, porque comparten los opcodes
// LOAD-FP y STORE-FP y solo las distingue el campo `width`.
// =====================================================================
#include "processor_ooo.h"

inline void ProcessorOOO::dispatch(uint32_t instr, uint8_t isize) {
        uint32_t opc = rv32i::opcode(instr);
        uint8_t rd   = rv32i::rd(instr);
        uint8_t f3   = rv32i::funct3(instr);
        uint8_t rs1i = rv32i::rs1(instr);
        uint8_t rs2i = rv32i::rs2(instr);
        uint32_t f7  = rv32i::funct7(instr);

        bool can_dispatch = false;
        uint8_t new_tag = rob_tail;
        rob[new_tag].dest_is_fp = false; // default entero; los casos F lo activan
        rob[new_tag].is_branch = false;  // solo el branch condicional lo activa
        rob[new_tag].br_mispred = false;
        rob[new_tag].takes_trap = false;
        rob[new_tag].is_mret    = false;
        uint32_t this_pc = fetch_pc;
        uint32_t next_fetch = this_pc + isize;
        // El pc se guarda para TODA instruccion: una interrupcion puede
        // tomarse con cualquiera en la cabeza del ROB, y mepc debe apuntar
        // a ella para que el MRET retome exactamente ahi.
        rob[new_tag].pc = this_pc;

        if (opc == rv32i::Opcode::LUI || opc == rv32i::Opcode::AUIPC ||
            opc == rv32i::Opcode::JAL) {
            can_dispatch = true;
            RobEntry& e = rob[new_tag];
            e.valid = true; e.is_store = false; e.dest = rd; e.ready = true;
            if (opc == rv32i::Opcode::LUI)
                e.value = static_cast<uint32_t>(rv32i::get_imm_U(instr));
            else if (opc == rv32i::Opcode::AUIPC)
                e.value = this_pc + static_cast<uint32_t>(rv32i::get_imm_U(instr));
            else { // JAL (un C.JAL expandido enlaza a pc+2 via isize)
                e.value = this_pc + isize;
                next_fetch = this_pc + static_cast<uint32_t>(rv32i::get_imm_J(instr));
            }
        } else if (opc == rv32i::Opcode::OP && f7 == rv32i::Funct7::MULDIV) {
            if (!md_rs.busy) {
                can_dispatch = true;
                md_rs.busy = true; md_rs.executing = false;
                md_rs.f3 = f3; md_rs.rob_tag = new_tag;
                md_rs.s1 = read_operand(rs1i);
                md_rs.s2 = read_operand(rs2i);
                RobEntry& e = rob[new_tag];
                e.valid = true; e.is_store = false; e.dest = rd; e.ready = false;
            }
        } else if (opc == rv32i::Opcode::OP || opc == rv32i::Opcode::OP_IMM) {
            int free_alu = -1;
            for (int i = N_ALU - 1; i >= 0; i--)
                if (!alu_rs[i].busy) free_alu = i;
            if (free_alu >= 0) {
                can_dispatch = true;
                AluRs& rs = alu_rs[free_alu];
                rs.busy = true; rs.executing = false;
                rs.f3 = f3; rs.rob_tag = new_tag;
                rs.s1 = read_operand(rs1i);
                if (opc == rv32i::Opcode::OP) {
                    rs.alt = (f7 == rv32i::Funct7::ALT);
                    rs.s2 = read_operand(rs2i);
                } else {
                    rs.alt = (f3 == rv32i::Funct3_ALU::SRL_SRA) && ((instr >> 30) & 1);
                    rs.s2.ready = true; rs.s2.tag = 0;
                    if (f3 == rv32i::Funct3_ALU::SLL || f3 == rv32i::Funct3_ALU::SRL_SRA)
                        rs.s2.val = rv32i::get_shamt(instr);
                    else
                        rs.s2.val = static_cast<uint32_t>(rv32i::get_imm_I(instr));
                }
                RobEntry& e = rob[new_tag];
                e.valid = true; e.is_store = false; e.dest = rd; e.ready = false;
            }
        } else if ((opc == rv32i::Opcode::LOAD_FP || opc == rv32i::Opcode::STORE_FP) &&
                   (f3 == RVV_WIDTH_64 || (((instr >> 28) & 1) != 0))) {
            // EEW=64 (width=111) o mew=1 (>=128 bits): load/store vectorial
            // legal en la spec pero FUERA de Zve32x, que llega hasta 32
            // bits. La seccion 7.3 pide que un ancho no soportado levante
            // instruccion ilegal en vez de ejecutarse mal en silencio --
            // que es lo que permite a un runtime detectar el perfil real.
            can_dispatch = true;
            RobEntry& e = rob[new_tag];
            e.valid = true; e.is_store = false; e.takes_trap = true;
            e.pc = this_pc; e.cause = 2;
            e.dest = 0; e.value = 0; e.ready = true;
        } else if ((opc == rv32i::Opcode::LOAD_FP || opc == rv32i::Opcode::STORE_FP) &&
                   (f3 == RVV_WIDTH_8 || f3 == RVV_WIDTH_16 || f3 == RVV_WIDTH_32) &&
                   (((instr >> 26) & 0x3) == RVV_MOP_UNIT) &&
                   !(((instr >> 20) & 0x1F) == RVV_LUMOP_UNIT  ||
                     ((instr >> 20) & 0x1F) == RVV_LUMOP_WHOLE ||
                     ((instr >> 20) & 0x1F) == RVV_LUMOP_MASK  ||
                     (((instr >> 20) & 0x1F) == RVV_LUMOP_FOF &&
                      opc == rv32i::Opcode::LOAD_FP))) {
            // lumop/sumop reservado (tablas 11 y 12). Ojo: el
            // fault-only-first SOLO existe como load -- no hay "store que
            // falla solo en el primer elemento", porque un store ya
            // escribio memoria cuando se descubre el fallo.
            can_dispatch = true;
            RobEntry& e = rob[new_tag];
            e.valid = true; e.is_store = false; e.takes_trap = true;
            e.pc = this_pc; e.cause = 2;
            e.dest = 0; e.value = 0; e.ready = true;
        } else if ((opc == rv32i::Opcode::LOAD_FP || opc == rv32i::Opcode::STORE_FP) &&
                   (f3 == RVV_WIDTH_8 || f3 == RVV_WIDTH_16 || f3 == RVV_WIDTH_32)) {
            // vle32.v / vse32.v: MISMO opcode LOAD-FP/STORE-FP que FLW/FSW
            // escalar, pero width=110 (vectorial 32b) en vez de 010 --
            // asi se distinguen (codificacion verificada contra RVV v1.0).
            // Una vectorial captura vl/vstart EN EL DISPATCH, pero un
            // `csrw vstart` los escribe en la CABEZA del ROB. Si se dejara
            // pasar, la vectorial leeria el vstart VIEJO (un RAW entre dos
            // etapas distintas del pipeline). Es la simetrica del stall que
            // ya hace vset*, y la cura es la misma.
            if (!viq_full() && !sys_rs.busy && branch_pending == 0 &&
                (csr_vstart == 0 || (viq_count == 0 && !vec_rs.busy))) {
                VecRs vq = VecRs();  // se ENCOLA en la VIQ del coprocesador
                can_dispatch = true;
                vq.busy = true;
                vq.is_load = (opc == rv32i::Opcode::LOAD_FP);
                vq.is_store = !vq.is_load;
                vq.is_arith = false;
                vq.vd_or_vs3 = rd; // mismo campo de bits que vd (load) o vs3 (store)
                vq.vm = ((instr >> 25) & 1) != 0; // bit vm, igual que en aritmetica
                vq.rob_tag = new_tag;
                vq.vl = csr_vl;             // largo vectorial vigente al despachar
                vq.vstart = (uint8_t)csr_vstart; // Fase 6: primer elemento a mover
                // ---- Fase 5: modo de direccionamiento (seccion 7.2) ----
                // mop  = instr[27:26] : unit-stride / strided / indexado
                // nf   = instr[31:29] : campos por segmento, menos uno
                // lumop= instr[24:20] : variantes del unit-stride
                uint32_t mop   = (instr >> 26) & 0x3;
                uint32_t lumop = (instr >> 20) & 0x1F;
                vq.mop = static_cast<uint8_t>(mop);
                vq.nf  = static_cast<uint8_t>((instr >> 29) & 0x7);
                vq.fld = 0;
                bool unit = (mop == RVV_MOP_UNIT);
                vq.lsmode = (unit && lumop == RVV_LUMOP_MASK)  ? LSM_MASK  :
                                (unit && lumop == RVV_LUMOP_WHOLE) ? LSM_WHOLE :
                                (unit && lumop == RVV_LUMOP_FOF)   ? LSM_FOF   :
                                                                     LSM_NORMAL;
                // Ancho de elemento: unit-stride y strided lo toman del
                // campo `width` de la instruccion. El INDEXADO no: ahi
                // `width` es el ancho del INDICE y el dato usa el SEW de
                // vtype (seccion 7.3). Es la unica forma que mezcla dos
                // anchos distintos en una misma instruccion.
                uint8_t w_b = (f3 == RVV_WIDTH_8) ? 1 : (f3 == RVV_WIDTH_16) ? 2 : 4;
                bool indexed = (mop == RVV_MOP_IDX_UNORD || mop == RVV_MOP_IDX_ORD);
                vq.idx_b = w_b;
                vq.sew_b = indexed ? cur_sew_bytes() : w_b;
                vq.vs2 = rs2i;              // indices (indexado) -- banco vectorial
                vq.s1 = read_operand(rs1i); // direccion base, banco entero
                if (mop == RVV_MOP_STRIDED) {
                    vq.s2 = read_operand(rs2i); // paso en BYTES, banco entero
                } else {
                    vq.s2.ready = true; vq.s2.val = 0; vq.s2.tag = 0;
                }
                RobEntry& e = rob[new_tag];
                e.valid = true; e.is_store = false; e.dest_is_fp = false;
                e.dest = 0; e.ready = false;
                viq_push(vq);
            }        } else if (opc == rv32i::Opcode::LOAD || opc == rv32i::Opcode::STORE ||
                   opc == rv32i::Opcode::LOAD_FP || opc == rv32i::Opcode::STORE_FP) {
            if (!lsu_rs.busy) {
                bool is_fp   = (opc == rv32i::Opcode::LOAD_FP || opc == rv32i::Opcode::STORE_FP);
                bool is_load = (opc == rv32i::Opcode::LOAD || opc == rv32i::Opcode::LOAD_FP);
                can_dispatch = true;
                lsu_rs.busy = true;
                lsu_rs.is_load = is_load;
                lsu_rs.f3 = f3; lsu_rs.rob_tag = new_tag;
                lsu_rs.s1 = read_operand(rs1i); // base SIEMPRE del banco entero
                if (is_load) {
                    lsu_rs.imm = rv32i::get_imm_I(instr);
                    lsu_rs.s2.ready = true; lsu_rs.s2.val = 0; lsu_rs.s2.tag = 0;
                } else {
                    lsu_rs.imm = rv32i::get_imm_S(instr);
                    lsu_rs.s2 = is_fp ? read_operand_fp(rs2i) : read_operand(rs2i);
                }
                RobEntry& e = rob[new_tag];
                e.valid = true; e.ready = false;
                e.is_store = !is_load;
                e.dest_is_fp = is_fp && is_load;
                e.dest = is_load ? rd : 0;
            }
        } else if (opc == rv32i::Opcode::FMADD || opc == rv32i::Opcode::FMSUB ||
                   opc == rv32i::Opcode::FNMSUB || opc == rv32i::Opcode::FNMADD) {
            if (!fpu_rs.busy) {
                can_dispatch = true;
                fpu_rs.busy = true; fpu_rs.executing = false;
                fpu_rs.r4op = (opc == rv32i::Opcode::FMADD)  ? 1 :
                              (opc == rv32i::Opcode::FMSUB)  ? 2 :
                              (opc == rv32i::Opcode::FNMSUB) ? 3 : 4;
                fpu_rs.f7 = 0; fpu_rs.f3 = f3; fpu_rs.rs2f = rs2i;
                fpu_rs.rob_tag = new_tag;
                fpu_rs.s1 = read_operand_fp(rs1i);
                fpu_rs.s2 = read_operand_fp(rs2i);
                fpu_rs.s3 = read_operand_fp(rv32i::rs3(instr));
                RobEntry& e = rob[new_tag];
                e.valid = true; e.is_store = false; e.ready = false;
                e.dest_is_fp = true; e.dest = rd;
            }
        } else if (opc == rv32i::Opcode::OP_FP) {
            if (!fpu_rs.busy) {
                can_dispatch = true;
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
                fpu_rs.r4op = 0; fpu_rs.f7 = f7; fpu_rs.f3 = f3; fpu_rs.rs2f = rs2i;
                fpu_rs.rob_tag = new_tag;
                fpu_rs.s1 = int_src ? read_operand(rs1i) : read_operand_fp(rs1i);
                if (needs_s2) {
                    fpu_rs.s2 = read_operand_fp(rs2i);
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
                br_rs.size = isize;
                br_rs.s1 = read_operand(rs1i);
                RobEntry& e = rob[new_tag];
                e.valid = true; e.is_store = false; e.ready = false;
                e.dest = br_rs.is_jalr ? rd : 0;
                if (br_rs.is_jalr) {
                    br_rs.imm = rv32i::get_imm_I(instr);
                    br_rs.s2.ready = true; br_rs.s2.val = 0; br_rs.s2.tag = 0;
                    // JALR: el destino sale de un REGISTRO y sin BTB/RAS no
                    // hay de donde predecirlo -> este SI detiene el fetch.
                    fetch_stalled = true;
                } else {
                    br_rs.imm = rv32i::get_imm_B(instr);
                    br_rs.s2 = read_operand(rs2i);
                    // ---- ESPECULACION: TAGE decide el camino del fetch ----
                    // El destino de un branch es pc+imm (viene EN la
                    // instruccion): no hace falta BTB. La verdad se
                    // comprueba en la unidad BR y se salda en el COMMIT.
                    bool pred = tage.predict(this_pc);
                    e.is_branch  = true;
                    e.pred_taken = pred;
                    e.ghr_snap   = tage.ghr;
                    if (pred)
                        next_fetch = this_pc +
                            static_cast<uint32_t>(rv32i::get_imm_B(instr));
                    // GHR especulativo (el commit lo repara si fallo)
                    tage.ghr = uint16_t((tage.ghr << 1) | (pred ? 1 : 0));
                    branch_pending++;
                }
            }
        } else if (opc == RVV_OPCODE_OP_V && f3 == RVV_FUNCT3_OPCFG) {
            // ---- vsetvli / vsetivli / vsetvl (seccion 5 de la spec) ----
            // Se resuelve EN EL DISPATCH (como LUI/JAL): el dispatch es en
            // orden y este core no especula, asi que actualizar vtype/vl
            // aca es preciso y hace que toda vectorial posterior capture el
            // vl nuevo sin serializar la unidad VEC. Requiere el AVL ya
            // disponible: si el productor sigue en vuelo, se detiene el
            // dispatch y se reintenta el ciclo siguiente.
            bool is_vsetivli = (((instr >> 30) & 0x3) == 0x3);
            bool is_vsetvl   = (((instr >> 25) & 0x7F) == 0b1000000);
            Operand avl_op  = read_operand(rs1i);
            Operand vtyp_op = read_operand(rs2i);
            bool operands_ready = is_vsetivli
                                ? true
                                : (avl_op.ready && (!is_vsetvl || vtyp_op.ready));
            // ...y ademas hay que esperar a que NO quede ninguna instruccion
            // CSR en vuelo. Un `csrr x, vl` lee el banco de CSRs en la
            // CABEZA del ROB, mientras que vset* lo escribe en el DISPATCH:
            // si se dejara pasar, un vset* posterior en el programa pisaria
            // vl ANTES de que el csrr anterior alcanzara a leerlo (un WAR
            // entre dos etapas distintas del pipeline). Esperar a que
            // sys_rs se libere cierra exactamente esa ventana, y no cuesta
            // nada en la practica porque leer vl explicitamente es raro.
            // ...y sin BRANCH especulado en vuelo: vset* escribe vtype/vl
            // EN EL DISPATCH -- estado arquitectonico que un flush no
            // podria deshacer si el branch anterior fallo.
            if (operands_ready && !sys_rs.busy && branch_pending == 0) {
                can_dispatch = true;
                uint32_t vtype_req, avl;
                bool avl_is_max = false, keep_vl = false;
                if (is_vsetivli) {
                    vtype_req = (instr >> 20) & 0x3FF;  // zimm[9:0]
                    avl = rs1i;                          // uimm[4:0]
                } else if (is_vsetvl) {
                    vtype_req = vtyp_op.val;
                    if (rs1i == 0) { if (rd == 0) keep_vl = true; else avl_is_max = true; }
                    avl = avl_op.val;
                } else { // vsetvli
                    vtype_req = (instr >> 20) & 0x7FF;  // zimm[10:0]
                    if (rs1i == 0) { if (rd == 0) keep_vl = true; else avl_is_max = true; }
                    avl = avl_op.val;
                }
                uint32_t vsew  = (vtype_req >> 3) & 0x7;
                uint32_t vlmul = vtype_req & 0x7;
                // Zve32x exige EEW de 8, 16 y 32; LMUL!=1 sigue fuera de
                // alcance. VLMAX = LMUL*VLEN/SEW = VLEN/SEW con LMUL=1.
                bool sew_ok = (vsew == RVV_VSEW_8 || vsew == RVV_VSEW_16 ||
                               vsew == RVV_VSEW_32);
                bool supported = sew_ok && (vlmul == RVV_VLMUL_1);
                uint32_t vlmax = 0;
                if (supported) vlmax = VEC_VLEN_BITS / (8u << vsew); // 8<<vsew = SEW en bits
                uint32_t new_vl;
                if (!supported) {                 // configuracion no soportada
                    csr_vtype = (1u << RVV_VILL_BIT);
                    new_vl = 0;
                } else {
                    csr_vtype = vtype_req;
                    if (keep_vl)         new_vl = csr_vl;
                    else if (avl_is_max) new_vl = vlmax;
                    else                 new_vl = (avl < vlmax) ? avl : vlmax;
                }
                csr_vl = new_vl;
                csr_vstart = 0;   // "all vector instructions, including
                                  //  vset{i}vl{i}, reset the vstart CSR to zero"
                RobEntry& e = rob[new_tag];
                e.valid = true; e.is_store = false; e.dest_is_fp = false;
                e.dest = rd; e.value = new_vl; e.ready = true;
            }
        } else if (opc == RVV_OPCODE_OP_V) {
            // Aritmetica vectorial entera. funct3 elige la FORMA del
            // segundo operando (.vv = vector, .vx = escalar de x[rs1],
            // .vi = inmediato de 5 bits con signo) y la familia
            // (OPIV* enteras / OPMV* multiplicacion-division); funct6
            // elige la operacion. Todos los valores verificados contra la
            // tabla de la seccion 19 de la spec RVV v1.0.
            uint32_t f6 = (instr >> 26) & 0x3F;
            bool vm = ((instr >> 25) & 1) != 0; // 1 = sin mascara
            bool is_iv = (f3 == RVV_FUNCT3_OPIVV || f3 == RVV_FUNCT3_OPIVX ||
                          f3 == RVV_FUNCT3_OPIVI);
            bool is_mv = (f3 == RVV_FUNCT3_OPMVV || f3 == RVV_FUNCT3_OPMVX);

            bool supported = false;
            uint8_t vcat = VCAT_ALU;
            if (is_iv) {
                supported = (f6 == RVV_F6_VADD  || f6 == RVV_F6_VSUB  ||
                             f6 == RVV_F6_VRSUB || f6 == RVV_F6_VMINU ||
                             f6 == RVV_F6_VMIN  || f6 == RVV_F6_VMAXU ||
                             f6 == RVV_F6_VMAX  || f6 == RVV_F6_VAND  ||
                             f6 == RVV_F6_VOR   || f6 == RVV_F6_VXOR  ||
                             f6 == RVV_F6_VSLL  || f6 == RVV_F6_VSRL  ||
                             f6 == RVV_F6_VSRA);
                // vsub no tiene forma .vi, y vrsub no tiene forma .vv
                if (f6 == RVV_F6_VSUB  && f3 == RVV_FUNCT3_OPIVI) supported = false;
                if (f6 == RVV_F6_VRSUB && f3 == RVV_FUNCT3_OPIVV) supported = false;
                // Fase 4c: saturantes. vssubu/vssub no tienen forma .vi.
                if (f6 == RVV_F6_VSADDU || f6 == RVV_F6_VSADD) supported = true;
                if ((f6 == RVV_F6_VSSUBU || f6 == RVV_F6_VSSUB) &&
                    f3 != RVV_FUNCT3_OPIVI) supported = true;
                // Fase 3: comparaciones -> escriben una MASCARA
                if (f6 >= RVV_F6_VMSEQ && f6 <= RVV_F6_VMSGT) {
                    supported = true; vcat = VCAT_CMP;
                    // vmsgt/vmsgtu solo existen en forma .vx/.vi
                    if ((f6 == RVV_F6_VMSGT || f6 == RVV_F6_VMSGTU) &&
                        f3 == RVV_FUNCT3_OPIVV) supported = false;
                    // vmsltu/vmslt no tienen forma .vi
                    if ((f6 == RVV_F6_VMSLTU || f6 == RVV_F6_VMSLT) &&
                        f3 == RVV_FUNCT3_OPIVI) supported = false;
                }
                // Fase 3: vmerge (vm=0) / vmv.v.* (vm=1)
                if (f6 == RVV_F6_VMERGE) { supported = true; vcat = VCAT_MERGE; }
                // Fase 4d: narrowing (.wv/.wx/.wi) -- vs2 es de 2*SEW
                if (f6 == RVV_F6_VNSRL || f6 == RVV_F6_VNSRA ||
                    f6 == RVV_F6_VNCLIPU || f6 == RVV_F6_VNCLIP) {
                    supported = true; vcat = VCAT_NARROW;
                }
                // Fase 4b: permutaciones con forma entera
                if (f6 == RVV_F6_VRGATHER ||
                    ((f6 == RVV_F6_VSLIDEUP || f6 == RVV_F6_VSLIDEDOWN) &&
                     f3 != RVV_FUNCT3_OPIVV)) {  // slide no tiene forma .vv
                    supported = true; vcat = VCAT_PERM;
                }
            } else if (is_mv) {
                supported = (f6 == RVV_F6_VMUL   || f6 == RVV_F6_VMULH  ||
                             f6 == RVV_F6_VMULHU || f6 == RVV_F6_VMULHSU||
                             f6 == RVV_F6_VDIVU  || f6 == RVV_F6_VDIV   ||
                             f6 == RVV_F6_VREMU  || f6 == RVV_F6_VREM);
                // Fase 4b: vslide1up/vslide1down (.vx) y vcompress (.vm)
                if (f3 == RVV_FUNCT3_OPMVX &&
                    (f6 == RVV_F6_VSLIDEUP || f6 == RVV_F6_VSLIDEDOWN)) {
                    supported = true; vcat = VCAT_PERM;
                }
                if (f3 == RVV_FUNCT3_OPMVV && f6 == RVV_F6_VCOMPRESS) {
                    supported = true; vcat = VCAT_PERM;
                }
                // Fase 4c: promediados (formas .vv y .vx)
                if (f6 == RVV_F6_VAADDU || f6 == RVV_F6_VAADD ||
                    f6 == RVV_F6_VASUBU || f6 == RVV_F6_VASUB) {
                    supported = true; vcat = VCAT_ALU;
                }
                // Fase 4a: reducciones (solo forma .vs = OPMVV)
                if (f3 == RVV_FUNCT3_OPMVV && f6 <= RVV_F6_VREDMAX) {
                    supported = true; vcat = VCAT_RED;
                }
                // Fase 4d: widening -- el destino ocupa un PAR de registros
                // (EMUL=2). Cubre sumas/restas (incluidas las formas .w con
                // el primer operando ya ancho), productos y multiply-accumulate.
                if ((f6 >= RVV_F6_VWADDU && f6 <= RVV_F6_VWSUB_W) ||
                    f6 == RVV_F6_VWMULU  || f6 == RVV_F6_VWMULSU ||
                    f6 == RVV_F6_VWMUL   || f6 == RVV_F6_VWMACCU ||
                    f6 == RVV_F6_VWMACC  || f6 == RVV_F6_VWMACCSU) {
                    supported = true; vcat = VCAT_WIDE;
                }
                // vwmaccus SOLO existe en forma .vx (ver tabla seccion 19)
                if (f6 == RVV_F6_VWMACCUS && f3 == RVV_FUNCT3_OPMVX) {
                    supported = true; vcat = VCAT_WIDE;
                }
                // Fase 3: logica entre mascaras (solo forma .mm = OPMVV)
                if (f3 == RVV_FUNCT3_OPMVV && f6 >= RVV_F6_VMANDNOT && f6 <= RVV_F6_VMXNOR) {
                    supported = true; vcat = VCAT_MLOG;
                }
                // Fase 3: grupos unary seleccionados por el campo vs1
                if (f3 == RVV_FUNCT3_OPMVV && f6 == RVV_F6_VWXUNARY0 &&
                    (rs1i == RVV_VS1_VCPOP || rs1i == RVV_VS1_VFIRST)) {
                    supported = true; vcat = VCAT_XRES;
                }
                if (f3 == RVV_FUNCT3_OPMVV && f6 == RVV_F6_VMUNARY0 &&
                    (rs1i == RVV_VS1_VID || rs1i == RVV_VS1_VIOTA)) {
                    supported = true; vcat = VCAT_VID;
                }
            }

            // ---- Fase 4d: restricciones de los grupos EMUL=2 ----
            // Zve32x tiene ELEN=32, asi que un destino de 2*SEW solo cabe si
            // SEW<=16. Ensanchar desde SEW=32 pediria 64 bits: fuera del
            // perfil -> instruccion ilegal, no un resultado malo en silencio.
            // Ademas el grupo de dos registros debe empezar en un registro
            // PAR (la spec pide especificadores legales para el EMUL): en
            // widening lo es vd, en narrowing vs2.
            {
                uint8_t sew_now = cur_sew_bytes();
                if (vcat == VCAT_WIDE   && (sew_now > 2 || (rd    & 1))) supported = false;
                if (vcat == VCAT_NARROW && (sew_now > 2 || (rs2i  & 1))) supported = false;
            }
            // ---- Fase 6: instrucciones que no admiten vstart != 0 ----
            // Reducciones, permutaciones, logica de mascaras y los grupos
            // unary se ejecutan de forma ATOMICA en este core: nunca puede
            // quedar un vstart intermedio en ellas. La spec (seccion 3.7)
            // permite explicitamente levantar instruccion ilegal "with a
            // value of vstart that the implementation can never produce".
            if (csr_vstart != 0 &&
                (vcat == VCAT_RED  || vcat == VCAT_PERM || vcat == VCAT_MLOG ||
                 vcat == VCAT_XRES || vcat == VCAT_VID)) supported = false;

            if (supported) {
                // mismo stall que en el camino de memoria: la vectorial lee
                // vl/vstart en el DISPATCH y un csrw los escribe en la
                // cabeza del ROB.
                if (!viq_full() && !sys_rs.busy && branch_pending == 0 &&
                    (csr_vstart == 0 || (viq_count == 0 && !vec_rs.busy))) {
                    VecRs vq = VecRs();  // a la VIQ del coprocesador
                    can_dispatch = true;
                    vq.busy = true;
                    vq.is_load = false; vq.is_store = false; vq.is_arith = true;
                    vq.vd_or_vs3 = rd; vq.vs1 = rs1i; vq.vs2 = rs2i;
                    vq.funct6 = f6; vq.funct3 = f3; vq.vm = vm;
                    vq.vcat = vcat; vq.vs1_field = rs1i;
                    vq.rob_tag = new_tag;
                    vq.vl = csr_vl;          // largo vectorial vigente al despachar
                    vq.vstart = (uint8_t)csr_vstart; // Fase 6
                    vq.sew_b = cur_sew_bytes(); // ancho de elemento vigente
                    vq.executing = false; // arranca en la etapa 3, como el resto
                    // operando escalar de las formas .vx / .vi
                    if (f3 == RVV_FUNCT3_OPIVX || f3 == RVV_FUNCT3_OPMVX) {
                        vq.s1 = read_operand(rs1i); // x[rs1], espera al CDB si hace falta
                    } else if (f3 == RVV_FUNCT3_OPIVI) {
                        // imm[4:0] con signo, en el mismo campo que rs1
                        int32_t simm = static_cast<int32_t>(rs1i << 27) >> 27;
                        vq.s1.ready = true; vq.s1.val = static_cast<uint32_t>(simm);
                        vq.s1.tag = 0;
                    } else {
                        vq.s1.ready = true; vq.s1.val = 0; vq.s1.tag = 0;
                    }
                    RobEntry& e = rob[new_tag];
                    e.valid = true; e.is_store = false; e.dest_is_fp = false;
                    // vcpop.m / vfirst.m escriben el registro ENTERO rd;
                    // el resto de las vectoriales no escriben banco escalar.
                    e.dest = (vcat == VCAT_XRES) ? rd : 0;
                    e.ready = false;
                    viq_push(vq);
                }                // si vec_rs.busy: can_dispatch queda false -> stall (unidad ocupada)
            } else {
                // variante RVV fuera de alcance -> instruccion ilegal
                can_dispatch = true;
                RobEntry& e = rob[new_tag];
                e.valid = true; e.is_store = false;
                e.takes_trap = true; e.cause = 2;
                e.dest = 0; e.value = 0; e.ready = true;
            }
        } else if (opc == rv32i::Opcode::SYSTEM) {
            if (f3 == rv32i::Funct3_SYSTEM::PRIV) {
                uint32_t imm12 = (instr >> 20) & 0xFFF;
                if (imm12 == 0x000 || imm12 == 0x001) {
                    // ECALL (causa 11 desde M, 8 desde U) / EBREAK (causa 3)
                    can_dispatch = true;
                    RobEntry& e = rob[new_tag];
                    e.valid = true; e.is_store = false; e.takes_trap = true;
                    e.cause = (imm12 == 0x000) ? 11 : 3;
                    e.dest = 0; e.value = 0; e.ready = true;
                } else if (imm12 == 0x302) {
                    // MRET: retorno desde el handler (pc <- mepc)
                    can_dispatch = true;
                    RobEntry& e = rob[new_tag];
                    e.valid = true; e.is_store = false; e.is_mret = true;
                    e.dest = 0; e.value = 0; e.ready = true;
                } else {
                    // WFI y otros privilegiados: no-op
                    can_dispatch = true;
                    RobEntry& e = rob[new_tag];
                    e.valid = true; e.is_store = false; e.dest = 0; e.value = 0; e.ready = true;
                }
            } else if (!sys_rs.busy) {
                // CSRRW/CSRRS/CSRRC y variantes con inmediato de 5 bits
                can_dispatch = true;
                bool is_imm = (f3 == rv32i::Funct3_SYSTEM::CSRRWI ||
                               f3 == rv32i::Funct3_SYSTEM::CSRRSI ||
                               f3 == rv32i::Funct3_SYSTEM::CSRRCI);
                sys_rs.busy = true;
                sys_rs.f3 = f3;
                sys_rs.csr_addr = (instr >> 20) & 0xFFF;
                sys_rs.rob_tag = new_tag;
                if (is_imm) { sys_rs.s1.ready = true; sys_rs.s1.val = rs1i; sys_rs.s1.tag = 0; }
                else        { sys_rs.s1 = read_operand(rs1i); }
                RobEntry& e = rob[new_tag];
                e.valid = true; e.is_store = false; e.dest_is_fp = false;
                e.dest = rd; e.ready = false;
            }
            // si sys_rs.busy: can_dispatch false -> stall (serializa CSR)
        } else if (opc == rv32i::Opcode::MISC_MEM) {
            // FENCE / FENCE.I: sin caches ni reordenamiento observable, se
            // reconoce como no-op (el crt0 la emite).
            can_dispatch = true;
            RobEntry& e = rob[new_tag];
            e.valid = true; e.is_store = false; e.dest = 0; e.value = 0; e.ready = true;
        } else {
            // Opcode no soportado -> EXCEPCION DE INSTRUCCION ILEGAL
            // (causa 2). Antes se retiraba como no-op silencioso, lo que
            // hacia que un binario incompatible diera resultados
            // incorrectos SIN AVISAR.
            can_dispatch = true;
            RobEntry& e = rob[new_tag];
            e.takes_trap = true; e.cause = 2;
            e.valid = true; e.is_store = false; e.dest = 0; e.value = 0; e.ready = true;
        }

        if (can_dispatch) {
            // renombrado (x0 nunca; f0 SI es un registro normal del banco F)
            uint8_t dest = rob[new_tag].dest;
            if (rob[new_tag].dest_is_fp) {
                frat[dest].has_tag = true;
                frat[dest].tag = new_tag;
            } else if (dest != 0) {
                rat[dest].has_tag = true;
                rat[dest].tag = new_tag;
            }
            rob[new_tag].disp_idx = n_disp;
            if (n_disp >= 0 && n_disp < 64) pc_of_disp[n_disp] = this_pc;
            if (trace)
                std::cout << "[" << cycle << "] DISPATCH d" << n_disp
                          << " (pc=" << this_pc << ", tag " << +new_tag << ")" << std::endl;
            n_disp++;
            rob_tail = (rob_tail + 1) & (ROB_SZ - 1);
            rob_count++;
            fetch_pc = next_fetch;
        }
    }

#endif // PROCESSOR_DISPATCH_H
