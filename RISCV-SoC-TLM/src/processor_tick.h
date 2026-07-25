#ifndef PROCESSOR_TICK_H
#define PROCESSOR_TICK_H

// =====================================================================
// EL TICK de ProcessorOOO: un ciclo = las cuatro etapas del pipeline, en
// el mismo orden que la pista HLS (rv32_ooo.cpp), que es lo que mantiene
// la paridad ciclo a ciclo entre los dos modelos.
//
//   Etapa 1  COMMIT     retiro en orden desde la cabeza del ROB; aca se
//                       toman traps e interrupciones -- por eso son PRECISOS
//   Etapa 2  EJECUCION  cada unidad avanza y difunde por el CDB
//   Etapa 3  ISSUE      una RS con operandos listos arranca
//   Etapa 4  DISPATCH   fetch + decodificacion (ver processor_dispatch.h)
//
// El orden importa: commit va PRIMERO para que la entrada del ROB que se
// libera pueda reusarse en el dispatch del mismo ciclo.
// =====================================================================
#include "processor_ooo.h"

inline void ProcessorOOO::tick() {
        mem_port_used = false;  // el arbitro del interconnect concede de nuevo

        // El timer avanza un tick por ciclo y levanta mip.MTIP al vencer.
        csr_mtime++;
        if (csr_mtime >= csr_mtimecmp) csr_mip |= (1u << 7);
        else                           csr_mip &= ~(1u << 7);

        // ---- Interrupcion de timer ----
        // Una interrupcion es un TRAP tomado en el mismo punto preciso que
        // una excepcion (la cabeza del ROB); solo cambia el disparador y
        // que el bit 31 de mcause la marca como asincrona.
        bool irq_en = (csr_mstatus & (1u << 3)) != 0;          // mstatus.MIE
        bool irq_tm = ((csr_mie & (1u << 7)) != 0) &&          // mie.MTIE
                      ((csr_mip & (1u << 7)) != 0);            // mip.MTIP
        if (irq_en && irq_tm && csr_mtvec != 0 &&
            rob_count > 0 && rob[rob_head].valid && rob[rob_head].ready) {
            csr_mepc   = rob[rob_head].pc;   // instruccion que aun no ejecuto
            csr_mcause = 0x80000000u | 7u;   // asincrona, timer de M-mode
            csr_mstatus = (csr_mstatus & ~((1u << 7) | (1u << 3) | (3u << 11)))
                        | (irq_en ? (1u << 7) : 0) | (uint32_t(cur_priv) << 11);
            cur_priv   = 3;
            fetch_pc   = csr_mtvec & ~3u;
            fetch_done = false;
            pipeline_flush();
            return;
        }

        // Etapa 1: commit (retiro en orden desde la cabeza del ROB)
        if (rob_count > 0 && rob[rob_head].valid && rob[rob_head].ready) {
            RobEntry& h = rob[rob_head];
            if (h.is_branch) {
                // ---- resolucion del branch ESPECULADO, en el commit ----
                // El predictor entrena aqui (con la verdad, en orden) y aqui
                // se salda la apuesta del fetch: mispredict = redirigir +
                // flush, el mecanismo de los traps reutilizado.
                tage.update(h.pc, h.ghr_snap, h.br_taken_real, h.br_mispred);
                if (branch_pending > 0) branch_pending--;
                stat_branches++;   // branches RETIRADOS (arquitectonicos)
                n_commit++;
                if (h.br_mispred) {
                    stat_mispredicts++;
                    tage.ghr = uint16_t((h.ghr_snap << 1) | (h.br_taken_real ? 1 : 0));
                    fetch_pc = h.br_target;
                    fetch_done = false;
                    pipeline_flush();
                } else {
                    h.valid = false;
                    rob_head = (rob_head + 1) & (ROB_SZ - 1);
                    rob_count--;
                }
                return;
            }
            if (h.takes_trap || h.is_mret) {
                // EXCEPCION PRECISA (o retorno del handler)
                bool redirect = false; uint32_t target = 0;
                if (h.is_mret) {
                    bool mpie = (csr_mstatus & (1u << 7)) != 0;
                    uint8_t mpp = (csr_mstatus >> 11) & 3;
                    csr_mstatus = (csr_mstatus & ~((1u << 3) | (3u << 11)))
                                | (mpie ? (1u << 3) : 0) | (1u << 7);
                    cur_priv = mpp;
                    target = csr_mepc; redirect = true;
                } else if (csr_mtvec != 0) {
                    csr_mepc = h.pc;
                    // ECALL reporta causa 8 desde U-mode y 11 desde M-mode
                    csr_mcause = (h.cause == 11 && cur_priv == 0) ? 8u : uint32_t(h.cause);
                    bool mie_prev = (csr_mstatus & (1u << 3)) != 0;
                    csr_mstatus = (csr_mstatus & ~((1u << 7) | (1u << 3) | (3u << 11)))
                                | (mie_prev ? (1u << 7) : 0) | (uint32_t(cur_priv) << 11);
                    cur_priv = 3;
                    target = csr_mtvec & ~3u; redirect = true;
                } else {
                    // sin handler instalado: se detiene (equivale al exit
                    // de un runtime bare-metal)
                    ecall_halt = true;
                }
                if (trace) std::cout << "[" << cycle << "] TRAP causa=" << +h.cause
                                     << (redirect ? " -> handler" : " -> halt") << std::endl;
                n_commit++;
                if (redirect) {
                    fetch_pc = target; fetch_done = false;
                    pipeline_flush();  // deja el ROB vacio: sin contabilidad extra
                } else {
                    h.valid = false;
                    rob_head = (rob_head + 1) & (ROB_SZ - 1);
                    rob_count--;
                }
                if (ecall_halt) halted = true;
                return;
            } else if (h.is_store) {
                axi_grant();  // el commit gana el arbitro, pero CONSUME el puerto
                store(h.addr, h.sdata, store_len(h.mem_f3));
            } else if (h.dest_is_fp) {
                fregs[h.dest] = rv32i::bits_to_float(h.value);
                if (frat[h.dest].has_tag && frat[h.dest].tag == rob_head)
                    frat[h.dest].has_tag = false;
            } else if (h.dest != 0) {
                regs[h.dest] = h.value;
            }
            if (!h.dest_is_fp && h.dest != 0 &&
                rat[h.dest].has_tag && rat[h.dest].tag == rob_head)
                rat[h.dest].has_tag = false;
            if (trace) {
                if (h.is_store)
                    std::cout << "[" << cycle << "] COMMIT store" << std::endl;
                else if (h.dest_is_fp)
                    std::cout << "[" << cycle << "] COMMIT f" << +h.dest
                              << " = 0x" << std::hex << h.value << std::dec << std::endl;
                else
                    std::cout << "[" << cycle << "] COMMIT x" << +h.dest
                              << " = 0x" << std::hex << h.value << std::dec << std::endl;
            }
            n_commit++;
            h.valid = false;
            rob_head = (rob_head + 1) & (ROB_SZ - 1);
            rob_count--;
        }

        // Etapa 2: ejecucion + broadcast (CDB) por unidad
        for (int i = 0; i < N_ALU; i++) {
            if (alu_rs[i].busy && alu_rs[i].executing) {
                if (alu_rs[i].remaining > 0) alu_rs[i].remaining--;
                if (alu_rs[i].remaining == 0) {
                    uint32_t res = alu_compute(alu_rs[i].f3, alu_rs[i].alt,
                                               alu_rs[i].s1.val, alu_rs[i].s2.val);
                    complete_entry(alu_rs[i].rob_tag, res, "ALU");
                    alu_rs[i].busy = false; alu_rs[i].executing = false;
                }
            }
        }
        if (md_rs.busy && md_rs.executing) {
            if (md_rs.remaining > 0) md_rs.remaining--;
            if (md_rs.remaining == 0) {
                complete_entry(md_rs.rob_tag, md_compute(md_rs.f3, md_rs.s1.val, md_rs.s2.val), "MULDIV");
                md_rs.busy = false; md_rs.executing = false;
            }
        }
        if (fpu_rs.busy && fpu_rs.executing) {
            if (fpu_rs.remaining > 0) fpu_rs.remaining--;
            if (fpu_rs.remaining == 0) {
                complete_entry(fpu_rs.rob_tag, fpu_compute(fpu_rs), "FPU");
                fpu_rs.busy = false; fpu_rs.executing = false;
            }
        }
        if (br_rs.busy && br_rs.executing) {
            if (br_rs.remaining > 0) br_rs.remaining--;
            if (br_rs.remaining == 0) {
                uint32_t link = br_rs.br_pc + br_rs.size;
                uint32_t value = 0;
                if (br_rs.is_jalr) {
                    // JALR: el fetch estaba detenido; se calcula el destino
                    // real y se reanuda (esquema sin especulacion).
                    fetch_pc = (br_rs.s1.val + static_cast<uint32_t>(br_rs.imm)) & ~0x1u;
                    value = link;
                    fetch_stalled = false;
                } else {
                    // Branch ESPECULADO: aqui solo se comprueba la
                    // prediccion y se ANOTA el veredicto en el ROB. La
                    // redireccion (si toca) ocurre en el COMMIT -- el mismo
                    // punto preciso de los traps, por la misma razon.
                    bool taken = branch_taken(br_rs.f3, br_rs.s1.val, br_rs.s2.val);
                    RobEntry& b = rob[br_rs.rob_tag];
                    b.br_taken_real = taken;
                    b.br_target = taken ? (br_rs.br_pc + static_cast<uint32_t>(br_rs.imm))
                                        : link;
                    b.br_mispred = (taken != b.pred_taken);
                }
                complete_entry(br_rs.rob_tag, value, "BR");
                br_rs.busy = false; br_rs.executing = false;
            }
        }
        // LSU: un store "ejecuta" (direccion + dato) en cuanto puede -- la
        // escritura real por el Bus espera al commit. Un load solo ejecuta
        // en la cabeza del ROB (todo store anterior ya escribio memoria).
        if (lsu_rs.busy && lsu_rs.s1.ready && lsu_rs.s2.ready) {
            uint32_t addr = lsu_rs.s1.val + static_cast<uint32_t>(lsu_rs.imm);
            if (!lsu_rs.is_load) {
                RobEntry& e = rob[lsu_rs.rob_tag];
                e.addr = addr; e.sdata = lsu_rs.s2.val; e.mem_f3 = lsu_rs.f3;
                e.ready = true;
                record_complete(lsu_rs.rob_tag, "LSU");
                lsu_rs.busy = false;
            } else if (lsu_rs.rob_tag == rob_head && rob[rob_head].valid &&
                       axi_grant()) {  // el interconnect concede el puerto
                complete_entry(lsu_rs.rob_tag, lsu_load_value(addr, lsu_rs.f3), "LSU");
                lsu_rs.busy = false;
            }
        }
        // VEC: aritmetica con latencia fija; memoria vectorial resuelta
        // SOLO en la cabeza del ROB, para load Y store (mas conservador
        // que el store escalar, para no ampliar el ROB a 128 bits).
        // ---- coprocesador: pop de la VIQ + issue con latencia por unidad ----
        if (!vec_rs.busy && viq_count != 0) {
            vec_rs = viq[viq_head];
            viq_head = (viq_head + 1) & (VIQ_SZ - 1);
            viq_count--;
            vec_rs.busy = true;
            vec_rs.executing = false;
        }
        if (vec_rs.busy && vec_rs.is_arith && !vec_rs.executing && vec_rs.s1.ready) {
            vec_rs.executing = true;
            // VALU / VMUL / VSLDU: cada unidad con su latencia
            const uint8_t f6 = vec_rs.funct6;
            const bool is_mv = (vec_rs.funct3 == RVV_FUNCT3_OPMVV ||
                                vec_rs.funct3 == RVV_FUNCT3_OPMVX);
            const bool vmul =
                (is_mv && (f6 == RVV_F6_VMUL  || f6 == RVV_F6_VMULH   ||
                           f6 == RVV_F6_VMULHU|| f6 == RVV_F6_VMULHSU ||
                           f6 == RVV_F6_VDIVU || f6 == RVV_F6_VDIV    ||
                           f6 == RVV_F6_VREMU || f6 == RVV_F6_VREM)) ||
                (vec_rs.vcat == VCAT_WIDE &&
                 (f6 == RVV_F6_VWMULU || f6 == RVV_F6_VWMULSU ||
                  f6 == RVV_F6_VWMUL  || f6 == RVV_F6_VWMACCU ||
                  f6 == RVV_F6_VWMACC || f6 == RVV_F6_VWMACCSU||
                  f6 == RVV_F6_VWMACCUS));
            vec_rs.remaining = vmul ? VMUL_LAT :
                               (vec_rs.vcat == VCAT_PERM) ? VSLDU_LAT : VALU_LAT;
        }

        if (vec_rs.busy && vec_rs.is_arith && vec_rs.executing) {
            if (vec_rs.remaining > 0) vec_rs.remaining--;
            if (vec_rs.remaining == 0) {
                uint32_t xres = vec_arith_compute(vec_rs);
                // vcpop.m / vfirst.m escriben un registro ENTERO: el valor
                // viaja por el ROB y el CDB como el de cualquier unidad.
                if (vec_rs.vcat == VCAT_XRES) {
                    complete_entry(vec_rs.rob_tag, xres, "VEC");
                } else {
                    rob[vec_rs.rob_tag].ready = true;
                    record_complete(vec_rs.rob_tag, "VEC");
                }
                vec_rs.busy = false; vec_rs.executing = false;
                csr_vstart = 0; // Fase 6: toda vectorial que COMPLETA resetea vstart
            }
        }
        // ---- VLSU: memoria vectorial, a traves del ARBITRO del interconnect ----
        // Ejecuta SOLO en la cabeza del ROB (memoria en orden) y mueve UN
        // ELEMENTO POR CICLO: cada acceso pide el puerto al arbitro
        // (axi_grant) y compite con la LSU escalar y los stores en retiro.
        // `elem` y `fld` llevan el progreso entre ciclos. Espejo exacto de
        // la VLSU de la pista HLS (RISCV-SoC/vector_coprocessor.h).
        if (vec_rs.busy && (vec_rs.is_load || vec_rs.is_store) &&
            vec_rs.s1.ready && vec_rs.s2.ready &&
            vec_rs.rob_tag == rob_head && rob[rob_head].valid) {
            const uint32_t base   = vec_rs.s1.val;
            const uint32_t stride = vec_rs.s2.val;
            const uint8_t  sb     = vec_rs.sew_b;
            const uint8_t  ib     = vec_rs.idx_b;
            const int      nfld   = vec_rs.nf + 1;
            const int      f      = vec_rs.fld;
            const int      e      = vec_rs.elem;
            const bool indexed = (vec_rs.mop == RVV_MOP_IDX_UNORD ||
                                  vec_rs.mop == RVV_MOP_IDX_ORD);
            bool done = false;

            if (vec_rs.lsmode == LSM_WHOLE) {
                // vl<nf>r.v / vs<nf>r.v: UNA PALABRA por ciclo; `elem`
                // recorre las palabras y `fld` los registros.
                if (axi_grant()) {
                    const uint8_t vi = (vec_rs.vd_or_vs3 + f) & 31;
                    uint32_t a = base + (f * VEC_LANES + e) * 4;
                    if (vec_rs.is_load) vregs[vi * VEC_LANES + e] = load(a, 4);
                    else                store(a, vregs[vi * VEC_LANES + e], 4);
                    if (e + 1 < VEC_LANES)      vec_rs.elem = uint8_t(e + 1);
                    else if (f + 1 < nfld)    { vec_rs.fld = uint8_t(f + 1); vec_rs.elem = 0; }
                    else done = true;
                }
            } else if (vec_rs.lsmode == LSM_MASK) {
                // vlm.v / vsm.v: la MASCARA como bytes, ceil(vl/8)
                const int evl = (vec_rs.vl + 7) / 8;
                if (e >= evl) {
                    done = true;   // vl=0: nada que mover
                } else if (axi_grant()) {
                    if (vec_rs.is_load)
                        vreg_set(vec_rs.vd_or_vs3, e, 1, load(base + e, 1));
                    else
                        store(base + e, vreg_get(vec_rs.vd_or_vs3, e, 1), 1);
                    if (e + 1 < evl) vec_rs.elem = uint8_t(e + 1); else done = true;
                }
            } else {
                // unit-stride / strided / indexado, con o sin segmentos
                int vl_eff = vec_rs.vl;

                if (vec_rs.lsmode == LSM_FOF && e == 0 && f == 0) {
                    // fault-only-first: se resuelve ANTES de mover nada.
                    // Solo chequeo de rangos (no hay MMU): no gasta puerto.
                    const uint32_t ram_end = memory_map::RAM_BASE + memory_map::RAM_SIZE;
                    int trim = vl_eff;
                    for (int k = 0; k < vl_eff; k++)
                        if (base + k * sb >= ram_end) { trim = k; break; }
                    if (trim == 0 && vl_eff > 0) {
                        rob[vec_rs.rob_tag].takes_trap = true;
                        rob[vec_rs.rob_tag].cause = 5;  // load access fault
                        vl_eff = 0;
                    } else if (trim < vl_eff) {
                        csr_vl = static_cast<uint32_t>(trim);
                        vl_eff = trim;
                    }
                    vec_rs.vl = uint8_t(vl_eff);
                }

                const int vst = vec_rs.vstart;   // Fase 6
                if (e >= vl_eff) {
                    if (f + 1 < nfld) { vec_rs.fld = uint8_t(f + 1); vec_rs.elem = 0; }
                    else done = true;
                } else if (e < vst || !vec_elem_active(vec_rs, e)) {
                    vec_rs.elem = uint8_t(e + 1);  // inactivo: no gasta puerto
                } else if (axi_grant()) {
                    uint32_t off;
                    if (indexed)                       off = vreg_get(vec_rs.vs2, e, ib);
                    else if (vec_rs.mop == RVV_MOP_STRIDED) off = uint32_t(e) * stride;
                    else                               off = uint32_t(e * nfld * sb);
                    const uint32_t a = base + off + uint32_t(f * sb);
                    const uint8_t vreg = (vec_rs.vd_or_vs3 + f) & 31;
                    if (vec_rs.is_load) vreg_set(vreg, e, sb, load(a, sb));
                    else                store(a, vreg_get(vreg, e, sb), sb);
                    vec_rs.elem = uint8_t(e + 1);
                }
                // puerta negada por el arbitro: se reintenta el mismo
                // elemento el ciclo siguiente (contencion real del bus)
            }

            if (done) {
                rob[vec_rs.rob_tag].ready = true;
                record_complete(vec_rs.rob_tag, "VEC");
                vec_rs.busy = false;
                if (!rob[vec_rs.rob_tag].takes_trap) csr_vstart = 0;
            }
        }

        // SYS (instrucciones CSR): ejecuta SOLO en la cabeza del ROB, para
        // que el acceso al banco de CSRs quede en orden de programa.
        if (sys_rs.busy && sys_rs.s1.ready &&
            sys_rs.rob_tag == rob_head && rob[rob_head].valid) {
            uint32_t old_v = read_csr(sys_rs.csr_addr);
            uint32_t src   = sys_rs.s1.val;
            uint32_t newv;
            switch (sys_rs.f3) {
                case rv32i::Funct3_SYSTEM::CSRRW:
                case rv32i::Funct3_SYSTEM::CSRRWI: newv = src;          break;
                case rv32i::Funct3_SYSTEM::CSRRS:
                case rv32i::Funct3_SYSTEM::CSRRSI: newv = old_v | src;  break;
                case rv32i::Funct3_SYSTEM::CSRRC:
                case rv32i::Funct3_SYSTEM::CSRRCI: newv = old_v & ~src; break;
                default:                           newv = old_v;        break;
            }
            write_csr(sys_rs.csr_addr, newv);
            complete_entry(sys_rs.rob_tag, old_v, "SYS"); // rd recibe el valor viejo
            sys_rs.busy = false;
        }

        // Etapa 3: issue (RS con operandos listos arranca ejecucion)
        for (int i = 0; i < N_ALU; i++) {
            if (alu_rs[i].busy && !alu_rs[i].executing && alu_rs[i].s1.ready && alu_rs[i].s2.ready) {
                alu_rs[i].executing = true;
                alu_rs[i].remaining = ALU_LAT;
            }
        }
        if (md_rs.busy && !md_rs.executing && md_rs.s1.ready && md_rs.s2.ready) {
            md_rs.executing = true;
            md_rs.remaining = (md_rs.f3 >= rv32i::Funct3_MULDIV::DIV) ? DIV_LAT : MUL_LAT;
        }
        if (fpu_rs.busy && !fpu_rs.executing &&
            fpu_rs.s1.ready && fpu_rs.s2.ready && fpu_rs.s3.ready) {
            fpu_rs.executing = true;
            if (fpu_rs.r4op != 0)                                    fpu_rs.remaining = FPU_LAT_FMA;
            else if (fpu_rs.f7 == rv32i::Funct7_FP::FDIV_S ||
                     fpu_rs.f7 == rv32i::Funct7_FP::FSQRT_S)         fpu_rs.remaining = FPU_LAT_DIV;
            else if (fpu_rs.f7 == rv32i::Funct7_FP::FADD_S ||
                     fpu_rs.f7 == rv32i::Funct7_FP::FSUB_S ||
                     fpu_rs.f7 == rv32i::Funct7_FP::FMUL_S)          fpu_rs.remaining = FPU_LAT_ADDMUL;
            else                                                     fpu_rs.remaining = FPU_LAT_MISC;
        }
        if (br_rs.busy && !br_rs.executing && br_rs.s1.ready && br_rs.s2.ready) {
            br_rs.executing = true;
            br_rs.remaining = BR_LAT;
        }

        // Etapa 4: fetch + dispatch (1 instruccion por ciclo, en orden)
        if (!fetch_done && !fetch_stalled && rob_count < ROB_SZ) {
            uint16_t half_lo = fetch16(fetch_pc);
            if (halted) return; // error de bus en el fetch
            if (half_lo == 0) {
                fetch_done = true; // convencion de fin de programa
            } else {
                uint32_t instr;
                uint8_t isize;
                if ((half_lo & 0x3) != 0x3) {
                    instr = rv32c::expand(half_lo);
                    isize = 2;
                } else {
                    uint16_t half_hi = fetch16(fetch_pc + 2);
                    instr = (static_cast<uint32_t>(half_hi) << 16) | half_lo;
                    isize = 4;
                }
                dispatch(instr, isize);
            }
        }

        if (fetch_done && rob_count == 0) halted = true;
    }

#endif // PROCESSOR_TICK_H
