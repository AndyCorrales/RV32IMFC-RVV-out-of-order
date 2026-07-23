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
                    fetch_pc = (br_rs.s1.val + static_cast<uint32_t>(br_rs.imm)) & ~0x1u;
                    value = link;
                } else {
                    bool taken = branch_taken(br_rs.f3, br_rs.s1.val, br_rs.s2.val);
                    fetch_pc = taken ? (br_rs.br_pc + static_cast<uint32_t>(br_rs.imm)) : link;
                }
                complete_entry(br_rs.rob_tag, value, "BR");
                fetch_stalled = false; // fetch se reanuda en el destino correcto
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
            } else if (lsu_rs.rob_tag == rob_head && rob[rob_head].valid) {
                complete_entry(lsu_rs.rob_tag, lsu_load_value(addr, lsu_rs.f3), "LSU");
                lsu_rs.busy = false;
            }
        }
        // VEC: aritmetica con latencia fija; memoria vectorial resuelta
        // SOLO en la cabeza del ROB, para load Y store (mas conservador
        // que el store escalar, para no ampliar el ROB a 128 bits).
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
        if (vec_rs.busy && (vec_rs.is_load || vec_rs.is_store) &&
            vec_rs.s1.ready && vec_rs.s2.ready &&
            vec_rs.rob_tag == rob_head && rob[rob_head].valid) {
            // ========== unidad de memoria vectorial (seccion 7) ==========
            // Los cuatro modos de direccionamiento solo cambian COMO se
            // calcula la direccion de cada elemento; el resto (mascara, vl,
            // ancho de elemento, transaccion TLM) es identico:
            //
            //   unit-stride : base + (e*nf + f)*eew    -- elementos contiguos
            //   strided     : base + e*paso  + f*eew   -- paso en BYTES de x[rs2]
            //   indexado    : base + vs2[e]  + f*eew   -- offsets de un vector
            //
            // donde `f` es el campo dentro del segmento y `eew` el ancho del
            // dato. Un solo generador de direcciones cubre los tres.
            const uint32_t base   = vec_rs.s1.val;
            const uint32_t stride = vec_rs.s2.val;
            const uint8_t  sb     = vec_rs.sew_b;      // ancho del DATO
            const uint8_t  ib     = vec_rs.idx_b;      // ancho del INDICE
            const int      nfld   = vec_rs.nf + 1;     // campos por segmento
            const int      f      = vec_rs.fld;        // campo de ESTE ciclo
            const bool indexed = (vec_rs.mop == RVV_MOP_IDX_UNORD ||
                                  vec_rs.mop == RVV_MOP_IDX_ORD);
            bool done = true;   // la instruccion termina en este ciclo?

            if (vec_rs.lsmode == LSM_WHOLE) {
                // ---- vl<nf>r.v / vs<nf>r.v: registros COMPLETOS ----
                // Mueve nf registros enteros (VLEN bits cada uno) sin mirar
                // vl, ni vtype, ni la mascara. Por eso es lo que usa un
                // cambio de contexto o un spill del compilador: no depende
                // de como este configurada la unidad en ese momento.
                // UN REGISTRO POR CICLO, por la misma razon que los
                // segmentos: mover los 8 registros de golpe serian 32
                // accesos a memoria en un solo ciclo, muy por encima de los
                // puertos de un BRAM en la pista HLS. Se reusa el contador
                // `fld` como indice de registro, y asi las dos pistas
                // siguen coincidiendo ciclo a ciclo.
                const uint8_t vi = (vec_rs.vd_or_vs3 + f) & 31;
                for (int w = 0; w < VEC_LANES; w++) {
                    uint32_t a = base + (f * VEC_LANES + w) * 4;
                    if (vec_rs.is_load) vregs[vi * VEC_LANES + w] = load(a, 4);
                    else                store(a, vregs[vi * VEC_LANES + w], 4);
                }
                if (f + 1 < nfld) { vec_rs.fld = static_cast<uint8_t>(f + 1); done = false; }
            } else if (vec_rs.lsmode == LSM_MASK) {
                // ---- vlm.v / vsm.v: mueve la MASCARA como bytes ----
                // El largo efectivo es ceil(vl/8) BYTES (un bit por
                // elemento), siempre sin mascara. Sirve para guardar o
                // restaurar un v0 sin tener que reconfigurar vl a byte.
                const int evl = (vec_rs.vl + 7) / 8;
                for (int b = 0; b < evl; b++) {
                    if (vec_rs.is_load)
                        vreg_set(vec_rs.vd_or_vs3, b, 1, load(base + b, 1));
                    else
                        store(base + b, vreg_get(vec_rs.vd_or_vs3, b, 1), 1);
                }
            } else {
                int vl_eff = vec_rs.vl;

                if (vec_rs.lsmode == LSM_FOF) {
                    // ---- fault-only-first (vle<EEW>ff.v) ----
                    // Solo el elemento 0 puede provocar un trap. Si falla
                    // uno POSTERIOR no se atrapa: se RECORTA vl a los
                    // elementos que si se pudieron leer, y el software
                    // reintenta desde ahi. Es lo que permite recorrer un
                    // string terminado en NUL sin arriesgarse a leer una
                    // pagina que no existe.
                    //
                    // Aca el unico "fallo" posible es salirse del rango de
                    // la RAM (no hay MMU ni PMP), asi que esa es la
                    // condicion que se comprueba.
                    const uint32_t ram_end = memory_map::RAM_BASE + memory_map::RAM_SIZE;
                    int trim = vl_eff;
                    for (int e = 0; e < vl_eff; e++)
                        if (base + e * sb >= ram_end) { trim = e; break; }
                    if (trim == 0 && vl_eff > 0) {
                        // El elemento 0 SI atrapa: no hay nada que recortar.
                        // Causa 5 = load access fault (mismo camino de trap
                        // preciso que usa la instruccion ilegal).
                        rob[vec_rs.rob_tag].takes_trap = true;
                        rob[vec_rs.rob_tag].cause = 5; // el pc ya lo puso el dispatch
                        vl_eff = 0;                    // no se escribe NINGUN elemento
                    } else if (trim < vl_eff) {
                        // vl recortado y visible al software. Es seguro
                        // escribirlo aca: hay UNA sola estacion vectorial,
                        // asi que ninguna instruccion vectorial posterior
                        // pudo capturar todavia el vl viejo (esta bloqueada
                        // en el dispatch), y un `csrr vl` ejecuta en la
                        // cabeza del ROB, o sea despues de esta.
                        csr_vl = static_cast<uint32_t>(trim);
                        vl_eff = trim;
                    }
                }

                for (int e = vec_rs.vstart; e < vl_eff; e++) {   // Fase 6
                    if (!vec_elem_active(vec_rs, e)) continue;
                    // --- generador de direcciones, comun a los tres modos ---
                    uint32_t off;
                    if (indexed)
                        // los indices son offsets en BYTES, con su propio
                        // ancho (ib), extendidos con ceros a XLEN
                        off = vreg_get(vec_rs.vs2, e, ib);
                    else if (vec_rs.mop == RVV_MOP_STRIDED)
                        off = static_cast<uint32_t>(e) * stride; // paso con signo: envuelve
                    else
                        off = static_cast<uint32_t>(e * nfld * sb); // unit-stride
                    const uint32_t a = base + off + static_cast<uint32_t>(f * sb);
                    // campo f del segmento -> registro vd+f (con LMUL=1 los
                    // grupos son de un registro, asi que van consecutivos)
                    const uint8_t vreg = (vec_rs.vd_or_vs3 + f) & 31;
                    if (vec_rs.is_load) vreg_set(vreg, e, sb, load(a, sb));
                    else                store(a, vreg_get(vreg, e, sb), sb);
                }
                // Segmentos: UN CAMPO POR CICLO. Un vlseg3e32.v con vl=4
                // mueve 12 elementos; hacerlos todos en un ciclo
                // multiplicaria por nf los puertos de memoria del RTL de la
                // pista HLS. Iterar deja el costo en 16 accesos por ciclo
                // (igual que un load simple) y modela mejor el hardware --
                // y mantiene la paridad ciclo a ciclo entre las dos pistas.
                if (f + 1 < nfld) { vec_rs.fld = static_cast<uint8_t>(f + 1); done = false; }
            }

            if (done) {
                rob[vec_rs.rob_tag].ready = true;
                record_complete(vec_rs.rob_tag, "VEC");
                vec_rs.busy = false;
                // Fase 6: toda vectorial que COMPLETA resetea vstart a 0. Si
                // en cambio ATRAPO (fault-only-first en el elemento 0),
                // vstart queda apuntando al elemento que fallo, para reanudar.
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
        if (vec_rs.busy && vec_rs.is_arith && !vec_rs.executing && vec_rs.s1.ready) {
            // sin operandos que esperar (lee vregs directo, sin RAT):
            // arranca en el primer ciclo de issue, igual que las demas
            vec_rs.executing = true;
            vec_rs.remaining = VEC_LAT;
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
