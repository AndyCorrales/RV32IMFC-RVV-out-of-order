// =====================================================================
// Core RV32IMFC + RVV out-of-order (Tomasulo) -- EL TICK.
//
// Este archivo contiene UNICAMENTE el lazo principal: las cuatro etapas
// del pipeline. Todo lo demas vive en headers, incluidos aca para formar
// una sola unidad de traduccion (el estado es `static`, asi que partirlo
// en varios .cpp le daria a cada uno su propia copia del ROB):
//
//   ooo_config.h    parametros (latencias, tamanos)
//   rvv_encoding.h  codificacion RVV 1.0 verificada contra la spec
//   ooo_state.h     structs y registros del procesador
//   ooo_csr.h       banco de CSRs
//   exec_scalar.h   CDB, operandos, ALU, mul/div, FPU, branch
//   exec_vector.h   banco vectorial y ALU vectorial (incl. EMUL=2)
//   exec_mem.h      acceso a la memoria de datos
// =====================================================================
#include "ooo_config.h"
#include "rvv_encoding.h"
#include "ooo_state.h"
#include "ooo_csr.h"
#include "exec_scalar.h"
#include "exec_vector.h"
#include "exec_mem.h"
#include "rv32c_defs.h"  // rv32c::expand, para las instrucciones comprimidas
#include "dispatch.h"    // etapa 4: la tabla de decodificacion completa

static void pipeline_flush() {
    for (int i = 0; i < ROB_SZ; i++) {
#pragma HLS UNROLL
        rob[i].valid = false; rob[i].ready = false;
    }
    rob_head = 0; rob_tail = 0; rob_count = 0;
    for (int i = 0; i < 32; i++) {
#pragma HLS UNROLL
        rat[i].has_tag = false;
        frat[i].has_tag = false;
    }
    for (int i = 0; i < N_ALU; i++) {
#pragma HLS UNROLL
        alu_rs[i].busy = false; alu_rs[i].executing = false;
    }
    md_rs.busy = false;  md_rs.executing = false;
    fpu_rs.busy = false; fpu_rs.executing = false;
    lsu_rs.busy = false;
    br_rs.busy = false;  br_rs.executing = false;
    vec_rs.busy = false; vec_rs.executing = false;
    sys_rs.busy = false;
    fetch_stalled = false;
}

// ================= el tick =================

void rv32_ooo_tick(
    ap_uint<1>  reset,
    ap_uint<32> imem[OOO_IMEM_WORDS],
    ap_uint<32> dmem[OOO_DMEM_WORDS],
    ap_uint<1>&  disp_valid,
    ap_uint<3>&  disp_tag,
    ap_uint<32>& disp_pc,
    ap_uint<1>& alu0_done, ap_uint<3>& alu0_tag,
    ap_uint<1>& alu1_done, ap_uint<3>& alu1_tag,
    ap_uint<1>& md_done,   ap_uint<3>& md_tag,
    ap_uint<1>& fpu_done,  ap_uint<3>& fpu_tag,
    ap_uint<1>& lsu_done,  ap_uint<3>& lsu_tag,
    ap_uint<1>& br_done,   ap_uint<3>& br_tag,
    ap_uint<1>& vec_done,  ap_uint<3>& vec_tag,
    ap_uint<1>&  commit_valid,
    ap_uint<1>&  commit_is_fp,
    ap_uint<5>&  commit_rd,
    ap_uint<32>& commit_value,
    ap_uint<32>  vregs_out[OOO_VEC_REGFILE_LEN],
    ap_uint<1>&  halted)
{
#pragma HLS INTERFACE bram port=imem
#pragma HLS INTERFACE bram port=dmem
#pragma HLS INTERFACE bram port=vregs_out
#pragma HLS INTERFACE ap_none port=reset
#pragma HLS INTERFACE ap_none port=disp_valid
#pragma HLS INTERFACE ap_none port=disp_tag
#pragma HLS INTERFACE ap_none port=disp_pc
#pragma HLS INTERFACE ap_none port=alu0_done
#pragma HLS INTERFACE ap_none port=alu0_tag
#pragma HLS INTERFACE ap_none port=alu1_done
#pragma HLS INTERFACE ap_none port=alu1_tag
#pragma HLS INTERFACE ap_none port=md_done
#pragma HLS INTERFACE ap_none port=md_tag
#pragma HLS INTERFACE ap_none port=fpu_done
#pragma HLS INTERFACE ap_none port=fpu_tag
#pragma HLS INTERFACE ap_none port=lsu_done
#pragma HLS INTERFACE ap_none port=lsu_tag
#pragma HLS INTERFACE ap_none port=br_done
#pragma HLS INTERFACE ap_none port=br_tag
#pragma HLS INTERFACE ap_none port=vec_done
#pragma HLS INTERFACE ap_none port=vec_tag
#pragma HLS INTERFACE ap_none port=commit_valid
#pragma HLS INTERFACE ap_none port=commit_is_fp
#pragma HLS INTERFACE ap_none port=commit_rd
#pragma HLS INTERFACE ap_none port=commit_value
#pragma HLS INTERFACE ap_none port=halted
#pragma HLS INTERFACE s_axilite port=return bundle=control

    // --- timer: avanza un tick por ciclo y levanta mip.MTIP al vencer ---
    csr_mtime = csr_mtime + 1;
    if (csr_mtime >= csr_mtimecmp) csr_mip = csr_mip | ap_uint<32>(1u << 7); // MTIP
    else                           csr_mip = csr_mip & ap_uint<32>(~(1u << 7));

    disp_valid = 0;  disp_tag = 0;  disp_pc = 0;
    alu0_done = 0;   alu0_tag = 0;
    alu1_done = 0;   alu1_tag = 0;
    md_done = 0;     md_tag = 0;
    fpu_done = 0;    fpu_tag = 0;
    lsu_done = 0;    lsu_tag = 0;
    br_done = 0;     br_tag = 0;
    vec_done = 0;    vec_tag = 0;
    commit_valid = 0; commit_is_fp = 0; commit_rd = 0; commit_value = 0;
    halted = 0;

    if (reset) {
        for (int i = 0; i < 32; i++) {
#pragma HLS UNROLL
            regfile[i] = 0;
            fregs[i] = 0;
            rat[i].has_tag = false;
            rat[i].tag = 0;
            frat[i].has_tag = false;
            frat[i].tag = 0;
        }
        for (int i = 0; i < ROB_SZ; i++) {
#pragma HLS UNROLL
            rob[i].valid = false; rob[i].ready = false; rob[i].is_store = false;
            rob[i].dest_is_fp = false;
            rob[i].dest = 0; rob[i].value = 0; rob[i].addr = 0; rob[i].sdata = 0;
            rob[i].mem_f3 = 0;
        }
        rob_head = 0; rob_tail = 0; rob_count = 0;
        for (int i = 0; i < N_ALU; i++) {
#pragma HLS UNROLL
            alu_rs[i].busy = false; alu_rs[i].executing = false;
        }
        md_rs.busy = false;  md_rs.executing = false;
        fpu_rs.busy = false; fpu_rs.executing = false;
        lsu_rs.busy = false;
        br_rs.busy = false;  br_rs.executing = false;
        vec_rs.busy = false; vec_rs.executing = false;
        sys_rs.busy = false;
        csr_mstatus = 0; csr_mie = 0; csr_mtvec = 0; csr_mscratch = 0;
        csr_mepc = 0; csr_mcause = 0; csr_mtval = 0; csr_mip = 0;
        // Estado vectorial al reset (spec 3.11): vtype con vill activo y
        // vl=0, es decir "no hay configuracion vectorial valida todavia".
        // Consecuencia: una instruccion vectorial ANTES del primer
        // vsetvli opera sobre 0 elementos (no hace nada), igual que
        // manda la definicion de `body` con vl=0.
        csr_vtype = (ap_uint<32>(1) << rvv::VILL_BIT);
        csr_vl = 0; csr_vstart = 0;
        ecall_halt = false;
        csr_mtime = 0; csr_mtimecmp = 0xFFFFFFFF; // sin interrupcion programada
        cur_priv = 3;                              // arranca en M-mode
        for (int i = 0; i < OOO_VEC_REGFILE_LEN; i++) {
            vregs[i] = 0;
        }
        fetch_pc = 0; fetch_stalled = false; fetch_done = false;
        for (int i = 0; i < OOO_VEC_REGFILE_LEN; i++) {
            vregs_out[i] = vregs[i];
        }
        return;
    }

    // ---- Interrupcion de timer ----------------------------------------
    // Una interrupcion es un TRAP tomado en el mismo punto preciso que una
    // excepcion (la cabeza del ROB), con dos diferencias: la dispara una
    // condicion externa en vez de una instruccion, y el bit 31 de mcause
    // marca que es asincrona. Toda la maquinaria (mepc/mcause/mtvec/flush/
    // MRET) es la misma que ya usan las excepciones.
    bool irq_enabled = (csr_mstatus & ap_uint<32>(1u << 3)) != 0;   // mstatus.MIE
    bool irq_timer   = ((csr_mie & ap_uint<32>(1u << 7)) != 0) &&   // mie.MTIE
                       ((csr_mip & ap_uint<32>(1u << 7)) != 0);     // mip.MTIP
    if (irq_enabled && irq_timer && csr_mtvec != 0 &&
        rob_count > 0 && rob[rob_head].valid && rob[rob_head].ready) {
        // mepc apunta a la instruccion que AUN NO se ejecuto: al volver con
        // MRET el programa retoma exactamente ahi.
        csr_mepc   = rob[rob_head].pc;
        csr_mcause = ap_uint<32>(0x80000000u | 7u); // bit31=asincrona, 7=timer M
        // al entrar al handler se deshabilitan interrupciones y se guarda
        // el estado previo (mstatus.MPIE <- MIE, MIE <- 0, MPP <- privilegio)
        csr_mstatus = (csr_mstatus & ap_uint<32>(~((1u << 7) | (1u << 3) | (3u << 11))))
                    | ap_uint<32>((irq_enabled ? (1u << 7) : 0) | (uint32_t(cur_priv) << 11));
        cur_priv   = 3;                       // el handler corre en M-mode
        fetch_pc   = csr_mtvec & ap_uint<32>(~3u);
        fetch_done = false;
        pipeline_flush();
        for (int i = 0; i < OOO_VEC_REGFILE_LEN; i++) vregs_out[i] = vregs[i];
        halted = 0;
        return;
    }

    // ---- Etapa 1: commit (retiro en orden desde la cabeza del ROB) ----
    if (rob_count > 0 && rob[rob_head].valid && rob[rob_head].ready) {
        RobEntry& h = rob[rob_head];
        if (h.takes_trap || h.is_mret) {
            // EXCEPCION PRECISA. Al llegar a la cabeza del ROB, todo lo
            // anterior ya committeo (retiro en orden) y nada posterior
            // modifico el estado arquitectonico todavia, asi que basta con
            // descartar lo que hay en vuelo: no hace falta recuperacion
            // especulativa (este core no especula).
            bool redirect = false;
            ap_uint<32> target = 0;
            if (h.is_mret) {
                // MRET: restaura interrupciones y privilegio previos, y
                // vuelve al pc guardado en mepc.
                bool mpie = (csr_mstatus & ap_uint<32>(1u << 7)) != 0;
                ap_uint<2> mpp = (csr_mstatus >> 11) & ap_uint<32>(3);
                csr_mstatus = (csr_mstatus & ap_uint<32>(~((1u << 3) | (3u << 11))))
                            | ap_uint<32>((mpie ? (1u << 3) : 0)) // MIE <- MPIE
                            | ap_uint<32>(1u << 7);               // MPIE <- 1
                cur_priv = mpp;
                target = csr_mepc; redirect = true;
            } else if (csr_mtvec != 0) {
                // hay handler instalado: se salta a el guardando el estado
                csr_mepc   = h.pc;      // direccion de la instruccion que atrapo
                // ECALL reporta una causa distinta segun el modo: 8 desde
                // U-mode, 11 desde M-mode (la spec los separa a proposito).
                // ECALL reporta causa distinta segun el modo (8 desde U,
                // 11 desde M); el resto de las causas pasan tal cual.
                csr_mcause = (h.cause == 11 && cur_priv == 0) ? ap_uint<32>(8) : ap_uint<32>(h.cause);
                // guarda el estado previo y entra al handler en M-mode
                bool mie_prev = (csr_mstatus & ap_uint<32>(1u << 3)) != 0;
                csr_mstatus = (csr_mstatus & ap_uint<32>(~((1u << 7) | (1u << 3) | (3u << 11))))
                            | ap_uint<32>((mie_prev ? (1u << 7) : 0) | (uint32_t(cur_priv) << 11));
                cur_priv = 3;
                target = csr_mtvec & ap_uint<32>(~3u); // modo directo
                redirect = true;
            } else {
                // sin mtvec instalado no hay a donde saltar: se detiene el
                // programa (convencion del simulador, documentada en el
                // README: equivale al "exit" de un runtime bare-metal).
                ecall_halt = true;
            }
            commit_valid = 1; commit_is_fp = 0; commit_rd = 0; commit_value = 0;
            if (redirect) {
                fetch_pc = target;
                fetch_done = false;
                // OJO: el flush ya deja el ROB vacio (head/tail/count en 0),
                // asi que NO debe hacerse ademas la contabilidad normal de
                // retiro -- restarle 1 a un contador ya en cero lo desborda.
                pipeline_flush();
            } else {
                h.valid = false;
                rob_head = rob_head + 1;
                rob_count = rob_count - 1;
            }
            for (int i = 0; i < OOO_VEC_REGFILE_LEN; i++) vregs_out[i] = vregs[i];
            halted = ecall_halt ? ap_uint<1>(1) : ap_uint<1>(0);
            return;
        } else if (h.is_store) {
            // UART mapeado en memoria: un store a UART_TX_ADDR imprime el
            // byte bajo en vez de escribir memoria. Es la salida que
            // necesita un putchar/printf de runtime C.
            if (h.addr == UART_TX_ADDR) {
                // Modelo de simulacion del UART: en csim imprime el byte.
                // En SINTESIS este bloque desaparece -- en hardware real la
                // direccion mapea a un periferico UART externo (por AXI),
                // no a esta memoria. Documentado en el README.
#ifndef __SYNTHESIS__
                printf("%c", static_cast<char>(h.sdata.to_uint() & 0xFF));
                fflush(stdout);
#endif
            } else {
                // el store escribe memoria recien al retirarse: garantiza el
                // orden de programa en memoria sin necesitar disambiguation
                dmem_store(dmem, h.addr, h.sdata, h.mem_f3);
            }
        } else if (h.dest_is_fp) {
            fregs[h.dest] = h.value; // f0 es escribible (no hay f0=0 en el ISA)
            if (frat[h.dest].has_tag && frat[h.dest].tag == rob_head) {
                frat[h.dest].has_tag = false;
            }
        } else if (h.dest != 0) {
            regfile[h.dest] = h.value;
        }
        if (!h.dest_is_fp && h.dest != 0 &&
            rat[h.dest].has_tag && rat[h.dest].tag == rob_head) {
            rat[h.dest].has_tag = false;
        }
        commit_valid = 1;
        commit_is_fp = h.dest_is_fp ? 1 : 0;
        commit_rd = h.is_store ? ap_uint<5>(0) : h.dest;
        commit_value = h.value;
        h.valid = false;
        rob_head = rob_head + 1;
        rob_count = rob_count - 1;
    }

    // ---- Etapa 2: ejecucion + broadcast (CDB) por unidad ----
    for (int i = 0; i < N_ALU; i++) {
#pragma HLS UNROLL
        if (alu_rs[i].busy && alu_rs[i].executing) {
            if (alu_rs[i].remaining > 0) alu_rs[i].remaining = alu_rs[i].remaining - 1;
            if (alu_rs[i].remaining == 0) {
                ap_uint<32> res = alu_compute(alu_rs[i].f3, alu_rs[i].alt,
                                              alu_rs[i].s1.val, alu_rs[i].s2.val);
                rob[alu_rs[i].rob_tag].value = res;
                rob[alu_rs[i].rob_tag].ready = true;
                cdb_broadcast(alu_rs[i].rob_tag, res);
                if (i == 0) { alu0_done = 1; alu0_tag = alu_rs[i].rob_tag; }
                else        { alu1_done = 1; alu1_tag = alu_rs[i].rob_tag; }
                alu_rs[i].busy = false;
                alu_rs[i].executing = false;
            }
        }
    }
    if (md_rs.busy && md_rs.executing) {
        if (md_rs.remaining > 0) md_rs.remaining = md_rs.remaining - 1;
        if (md_rs.remaining == 0) {
            ap_uint<32> res = md_compute(md_rs.f3, md_rs.s1.val, md_rs.s2.val);
            rob[md_rs.rob_tag].value = res;
            rob[md_rs.rob_tag].ready = true;
            cdb_broadcast(md_rs.rob_tag, res);
            md_done = 1; md_tag = md_rs.rob_tag;
            md_rs.busy = false;
            md_rs.executing = false;
        }
    }
    if (fpu_rs.busy && fpu_rs.executing) {
        if (fpu_rs.remaining > 0) fpu_rs.remaining = fpu_rs.remaining - 1;
        if (fpu_rs.remaining == 0) {
            ap_uint<32> res = fpu_compute(fpu_rs);
            rob[fpu_rs.rob_tag].value = res;
            rob[fpu_rs.rob_tag].ready = true;
            cdb_broadcast(fpu_rs.rob_tag, res);
            fpu_done = 1; fpu_tag = fpu_rs.rob_tag;
            fpu_rs.busy = false;
            fpu_rs.executing = false;
        }
    }
    if (br_rs.busy && br_rs.executing) {
        if (br_rs.remaining > 0) br_rs.remaining = br_rs.remaining - 1;
        if (br_rs.remaining == 0) {
            ap_uint<32> link = br_rs.br_pc + br_rs.size; // pc+2 si vino comprimida
            if (br_rs.is_jalr) {
                fetch_pc = (br_rs.s1.val + ap_uint<32>(br_rs.imm)) & ap_uint<32>(~1u);
                rob[br_rs.rob_tag].value = link;
            } else {
                bool taken = branch_taken(br_rs.f3, br_rs.s1.val, br_rs.s2.val);
                fetch_pc = taken ? ap_uint<32>(br_rs.br_pc + ap_uint<32>(br_rs.imm)) : link;
                rob[br_rs.rob_tag].value = 0;
            }
            rob[br_rs.rob_tag].ready = true;
            cdb_broadcast(br_rs.rob_tag, rob[br_rs.rob_tag].value);
            br_done = 1; br_tag = br_rs.rob_tag;
            fetch_stalled = false; // se reanuda el fetch en el destino correcto
            br_rs.busy = false;
            br_rs.executing = false;
        }
    }
    // LSU: un store "ejecuta" (calcula direccion y captura el dato) en
    // cuanto tiene los operandos -- la escritura real espera al commit.
    // Un load solo ejecuta cuando su entrada es la cabeza del ROB (todo
    // store anterior ya committeo su escritura): memoria en orden.
    if (lsu_rs.busy && lsu_rs.s1.ready && lsu_rs.s2.ready) {
        ap_uint<32> addr = lsu_rs.s1.val + ap_uint<32>(lsu_rs.imm);
        if (!lsu_rs.is_load) {
            RobEntry& e = rob[lsu_rs.rob_tag];
            e.addr = addr;
            e.sdata = lsu_rs.s2.val;
            e.mem_f3 = lsu_rs.f3;
            e.ready = true;
            lsu_done = 1; lsu_tag = lsu_rs.rob_tag;
            lsu_rs.busy = false;
        } else if (lsu_rs.rob_tag == rob_head && rob[rob_head].valid) {
            ap_uint<32> res = dmem_load(dmem, addr, lsu_rs.f3);
            rob[lsu_rs.rob_tag].value = res;
            rob[lsu_rs.rob_tag].ready = true;
            cdb_broadcast(lsu_rs.rob_tag, res);
            lsu_done = 1; lsu_tag = lsu_rs.rob_tag;
            lsu_rs.busy = false;
        }
    }
    // VEC: aritmetica con latencia fija (como la ALU); memoria vectorial
    // resuelta SOLO en la cabeza del ROB -- para load Y store (mas
    // conservador que el store escalar, para no tener que ampliar el ROB
    // a 128 bits para cargar los 4 lanes de un store vectorial).
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
    if (vec_rs.busy && (vec_rs.is_load || vec_rs.is_store) &&
        vec_rs.s1.ready && vec_rs.s2.ready &&
        vec_rs.rob_tag == rob_head && rob[rob_head].valid) {
        // ============ unidad de memoria vectorial (seccion 7) ============
        // Los cuatro modos de direccionamiento solo cambian COMO se calcula
        // la direccion de cada elemento; el resto (mascara, vl, ancho de
        // elemento, lectura/escritura de la dmem) es identico:
        //
        //   unit-stride : base + (e*nf + f)*eew      -- elementos contiguos
        //   strided     : base + e*paso   + f*eew    -- paso en BYTES de x[rs2]
        //   indexado    : base + vs2[e]   + f*eew    -- offsets desde un vector
        //
        // donde `f` es el campo dentro del segmento (nf campos) y `eew` el
        // ancho del dato. Un solo generador de direcciones cubre los tres.
        ap_uint<32> base   = vec_rs.s1.val;
        ap_uint<32> stride = vec_rs.s2.val;
        uint32_t sb    = vec_rs.sew_b.to_uint();   // ancho del DATO en bytes
        uint32_t ib    = vec_rs.idx_b.to_uint();   // ancho del INDICE en bytes
        uint32_t nfld  = vec_rs.nf.to_uint() + 1;  // campos por segmento
        uint32_t f     = vec_rs.fld.to_uint();     // campo de ESTE ciclo
        uint32_t emask = (sb == 4) ? 0xFFFFFFFFu : ((1u << (sb * 8)) - 1);
        bool indexed   = (vec_rs.mop == rvv::MOP_IDX_UNORD ||
                          vec_rs.mop == rvv::MOP_IDX_ORD);
        bool done      = true;   // la instruccion termina en este ciclo?

        if (vec_rs.lsmode == rvv::LSM_WHOLE) {
            // ---- vl<nf>r.v / vs<nf>r.v: registros COMPLETOS ----
            // Mueve nf registros enteros (VLEN bits cada uno) sin mirar vl,
            // ni vtype, ni la mascara. Por eso es lo que usa un cambio de
            // contexto o un spill del compilador: no depende de como este
            // configurada la unidad vectorial en ese momento.
            // UN REGISTRO POR CICLO, por la misma razon que los segmentos:
            // mover los 8 registros de golpe serian 32 accesos a la dmem en
            // un solo ciclo, muy por encima de los puertos que tiene un
            // BRAM. Se reusa el contador `fld` como indice de registro.
            uint32_t vi = (vec_rs.vd_or_vs3.to_uint() + f) & 31;
            for (int w = 0; w < OOO_VEC_LANES; w++) {
#pragma HLS UNROLL
                ap_uint<32> a = base + (f * OOO_VEC_LANES + w) * 4;
                uint32_t wi = (a >> 2) & (OOO_DMEM_WORDS - 1);
                if (vec_rs.is_load) vregs[vi * OOO_VEC_LANES + w] = dmem[wi];
                else                dmem[wi] = vregs[vi * OOO_VEC_LANES + w];
            }
            if (f + 1 < nfld) { vec_rs.fld = f + 1; done = false; }
        } else if (vec_rs.lsmode == rvv::LSM_MASK) {
            // ---- vlm.v / vsm.v: mueve la MASCARA como bytes ----
            // El largo efectivo es ceil(vl/8) BYTES (un bit por elemento),
            // siempre sin mascara. Sirve para guardar/cargar un v0 sin
            // tener que reconfigurar vl a byte.
            uint32_t evl = (vec_rs.vl.to_uint() + 7) / 8;
            for (int b = 0; b < rvv::VLEN_BITS / 8; b++) {
#pragma HLS UNROLL
                if (b < (int)evl) {
                    ap_uint<32> ba = base + b;
                    uint32_t wi = (ba >> 2) & (OOO_DMEM_WORDS - 1);
                    int shift = (ba & 3) * 8;
                    if (vec_rs.is_load) {
                        ap_uint<32> v = (dmem[wi] >> shift) & ap_uint<32>(0xFF);
                        vreg_set(vregs, vec_rs.vd_or_vs3.to_uint(), b, 1, v);
                    } else {
                        ap_uint<32> v = vreg_get(vregs, vec_rs.vd_or_vs3.to_uint(), b, 1);
                        dmem[wi] = (dmem[wi] & ap_uint<32>(~(0xFFu << shift))) |
                                   ((v & ap_uint<32>(0xFF)) << shift);
                    }
                }
            }
        } else {
            // ---- unit-stride / strided / indexado, con o sin segmentos ----
            uint32_t vl_eff = vec_rs.vl.to_uint();

            if (vec_rs.lsmode == rvv::LSM_FOF) {
                // ---- fault-only-first (vle<EEW>ff.v) ----
                // Solo el elemento 0 puede provocar un trap. Si falla uno
                // POSTERIOR no se atrapa: se RECORTA vl a los elementos que
                // si se pudieron leer, y el software reintenta desde ahi.
                // Es lo que permite recorrer un string terminado en NUL sin
                // arriesgarse a leer una pagina que no existe.
                //
                // En este core el unico "fallo" posible es salirse del rango
                // fisico de la dmem (no hay MMU ni PMP), asi que esa es la
                // condicion que se comprueba.
                uint32_t trim = vl_eff;
                for (int e = 0; e < rvv::MAX_ELEMS; e++) {
#pragma HLS UNROLL
                    if (e < (int)vl_eff) {
                        ap_uint<32> ba = base + e * sb;
                        bool fault = (ba.to_uint() >= OOO_DMEM_WORDS * 4u);
                        if (fault && (uint32_t)e < trim) trim = e;
                    }
                }
                if (trim == 0 && vl_eff > 0) {
                    // El elemento 0 SI atrapa: no hay nada que recortar.
                    // Causa 5 = load access fault (mismo camino de trap
                    // preciso que usa la instruccion ilegal).
                    rob[vec_rs.rob_tag].takes_trap = true;
                    rob[vec_rs.rob_tag].cause = 5;  // el pc ya lo puso el dispatch
                    vl_eff = 0;      // no se escribe NINGUN elemento
                } else if (trim < vl_eff) {
                    // vl recortado y visible al software. Es seguro
                    // escribirlo aca: hay UNA sola estacion vectorial, asi
                    // que ninguna instruccion vectorial posterior pudo haber
                    // capturado todavia el vl viejo (esta bloqueada en el
                    // dispatch), y un `csrr vl` ejecuta en la cabeza del ROB,
                    // o sea despues de esta.
                    csr_vl = trim;
                    vl_eff = trim;
                }
            }

            const uint32_t vst = vec_rs.vstart.to_uint(); // Fase 6
            for (int e = 0; e < rvv::MAX_ELEMS; e++) {
#pragma HLS UNROLL
                if (e >= (int)vst && e < (int)vl_eff &&
                    vec_elem_active(vec_rs, e, vregs)) {
                    // --- generador de direcciones, comun a los tres modos ---
                    ap_uint<32> off;
                    if (indexed) {
                        // los indices son offsets en BYTES, con su propio
                        // ancho (ib), extendidos con ceros a XLEN
                        off = vreg_get(vregs, vec_rs.vs2.to_uint(), e, ib);
                    } else if (vec_rs.mop == rvv::MOP_STRIDED) {
                        off = ap_uint<32>(e * stride.to_uint()); // paso con signo: envuelve
                    } else {
                        off = ap_uint<32>(e * nfld * sb);        // unit-stride
                    }
                    ap_uint<32> ba = base + off + ap_uint<32>(f * sb);
                    uint32_t wi = (ba >> 2) & (OOO_DMEM_WORDS - 1);
                    int shift = (ba & 3) * 8;
                    // campo f del segmento -> registro vd+f (con LMUL=1 los
                    // grupos son de un registro, asi que van consecutivos)
                    uint32_t vreg = (vec_rs.vd_or_vs3.to_uint() + f) & 31;
                    if (vec_rs.is_load) {
                        ap_uint<32> v = (dmem[wi] >> shift) & ap_uint<32>(emask);
                        vreg_set(vregs, vreg, e, sb, v);
                    } else {
                        ap_uint<32> v = vreg_get(vregs, vreg, e, sb);
                        if (sb == 4) dmem[wi] = v;
                        else dmem[wi] = (dmem[wi] & ap_uint<32>(~(emask << shift))) |
                                        ((v & ap_uint<32>(emask)) << shift);
                    }
                }
            }
            // Segmentos: UN CAMPO POR CICLO. Un vlseg3e32.v con vl=4 mueve
            // 12 elementos; hacerlos todos en un ciclo multiplicaria por nf
            // los puertos de memoria del RTL. Iterar deja el costo en 16
            // accesos por ciclo (igual que un load simple) y modela mejor
            // lo que hace el hardware de verdad.
            if (f + 1 < nfld) { vec_rs.fld = f + 1; done = false; }
        }

        if (done) {
            rob[vec_rs.rob_tag].ready = true;
            vec_done = 1; vec_tag = vec_rs.rob_tag;
            vec_rs.busy = false;
            // Fase 6: toda vectorial que COMPLETA resetea vstart a 0. Si en
            // cambio ATRAPO (fault-only-first en el elemento 0), vstart se
            // deja apuntando al elemento que fallo, para poder reanudar.
            if (!rob[vec_rs.rob_tag].takes_trap) csr_vstart = 0;
        }
    }
    // SYS (instrucciones CSR): ejecuta SOLO en la cabeza del ROB, para
    // que el acceso al banco de CSRs sea en orden de programa. Lee el
    // CSR viejo (va a rd via el CDB), calcula el nuevo (write/set/clear) y
    // lo escribe. El destino rd=0 en las variantes "csrw" no importa (el
    // ROB no escribe x0 en el commit).
    if (sys_rs.busy && sys_rs.s1.ready &&
        sys_rs.rob_tag == rob_head && rob[rob_head].valid) {
        ap_uint<32> old = read_csr(sys_rs.csr_addr);
        ap_uint<32> src = sys_rs.s1.val;
        ap_uint<32> newv;
        switch (sys_rs.f3.to_uint()) {
            case rv32i::Funct3_SYSTEM::CSRRW:
            case rv32i::Funct3_SYSTEM::CSRRWI: newv = src;        break;
            case rv32i::Funct3_SYSTEM::CSRRS:
            case rv32i::Funct3_SYSTEM::CSRRSI: newv = old | src;  break;
            case rv32i::Funct3_SYSTEM::CSRRC:
            case rv32i::Funct3_SYSTEM::CSRRCI: newv = old & ~src; break;
            default:                           newv = old;        break;
        }
        write_csr(sys_rs.csr_addr, newv);
        rob[sys_rs.rob_tag].value = old;
        rob[sys_rs.rob_tag].ready = true;
        cdb_broadcast(sys_rs.rob_tag, old);
        sys_rs.busy = false;
    }

    // ---- Etapa 3: issue (RS con operandos listos arranca ejecucion) ----
    for (int i = 0; i < N_ALU; i++) {
#pragma HLS UNROLL
        if (alu_rs[i].busy && !alu_rs[i].executing && alu_rs[i].s1.ready && alu_rs[i].s2.ready) {
            alu_rs[i].executing = true;
            alu_rs[i].remaining = ALU_LAT;
        }
    }
    if (md_rs.busy && !md_rs.executing && md_rs.s1.ready && md_rs.s2.ready) {
        md_rs.executing = true;
        bool is_div = (md_rs.f3.to_uint() >= rv32i::Funct3_MULDIV::DIV);
        md_rs.remaining = is_div ? DIV_LAT : MUL_LAT;
    }
    if (fpu_rs.busy && !fpu_rs.executing &&
        fpu_rs.s1.ready && fpu_rs.s2.ready && fpu_rs.s3.ready) {
        fpu_rs.executing = true;
        if (fpu_rs.r4op != 0) {
            fpu_rs.remaining = FPU_LAT_FMA;
        } else if (fpu_rs.f7 == rv32i::Funct7_FP::FDIV_S ||
                   fpu_rs.f7 == rv32i::Funct7_FP::FSQRT_S) {
            fpu_rs.remaining = FPU_LAT_DIV;
        } else if (fpu_rs.f7 == rv32i::Funct7_FP::FADD_S ||
                   fpu_rs.f7 == rv32i::Funct7_FP::FSUB_S ||
                   fpu_rs.f7 == rv32i::Funct7_FP::FMUL_S) {
            fpu_rs.remaining = FPU_LAT_ADDMUL;
        } else {
            fpu_rs.remaining = FPU_LAT_MISC;
        }
    }
    if (br_rs.busy && !br_rs.executing && br_rs.s1.ready && br_rs.s2.ready) {
        br_rs.executing = true;
        br_rs.remaining = BR_LAT;
    }
    if (vec_rs.busy && vec_rs.is_arith && !vec_rs.executing && vec_rs.s1.ready) {
        // sin operandos que esperar (lee vregs directo, sin RAT): arranca
        // en el primer ciclo de issue disponible, igual que las demas
        vec_rs.executing = true;
        vec_rs.remaining = VEC_LAT;
    }

    // ---- Etapa 4: fetch + dispatch (1 instruccion por ciclo, en orden) ----
    if (!fetch_done && !fetch_stalled && rob_count < ROB_SZ) {
        // Fetch de 16 bits (extension C): se leen las dos palabras que
        // pueden contener la instruccion (la de pc y la siguiente, por si
        // una de 32 bits arranca en la mitad alta) y se seleccionan los
        // halfwords segun pc[1]. Mismo criterio que el modelo TLM: bits
        // [1:0] != 11 => comprimida, se expande con rv32c::expand() y pc
        // avanza 2; cualquier alineacion PAR es valida (a diferencia del
        // core escalar rv32_core.cpp, que exige mitad baja alineada).
        ap_uint<32> w0 = imem[(fetch_pc >> 2) & (OOO_IMEM_WORDS - 1)];
        ap_uint<32> w1 = imem[((fetch_pc >> 2) + 1) & (OOO_IMEM_WORDS - 1)];
        bool upper = (fetch_pc & 0x2) != 0;
        ap_uint<16> half_lo = upper ? ap_uint<16>(w0.range(31, 16)) : ap_uint<16>(w0.range(15, 0));
        ap_uint<16> half_hi = upper ? ap_uint<16>(w1.range(15, 0))  : ap_uint<16>(w0.range(31, 16));
        if (half_lo == 0) {
            fetch_done = true; // convencion de fin de programa (halfword 0)
        } else {
            ap_uint<32> instr;
            ap_uint<3>  isize;
            if ((half_lo & 0x3) != 0x3) {
                instr = rv32c::expand(half_lo.to_uint());
                isize = 2;
            } else {
                instr = (ap_uint<32>(half_hi) << 16) | ap_uint<32>(half_lo);
                isize = 4;
            }
            uint32_t iw = instr.to_uint();
            ap_uint<3> new_tag = rob_tail;
            rob[new_tag].dest_is_fp = false; // default entero; los casos F lo activan
            rob[new_tag].takes_trap = false; // lo activan ECALL/EBREAK e ilegal
            rob[new_tag].is_mret  = false;   // solo MRET lo activa
            ap_uint<32> this_pc = fetch_pc;           // pc de ESTA instruccion
            ap_uint<32> next_fetch = this_pc + isize; // se pisa si JAL redirige
            // El pc se guarda para TODA instruccion, no solo las de
            // sistema: una INTERRUPCION puede tomarse con cualquier
            // instruccion en la cabeza del ROB, y mepc debe apuntar a
            // ella para que el MRET retome exactamente ahi.
            rob[new_tag].pc = this_pc;
            (void)iw;

            bool can_dispatch =
                dispatch_decode(instr, isize, this_pc, new_tag, next_fetch);


            if (can_dispatch) {
                // renombrado: el destino arquitectonico pasa a apuntar a la
                // entrada nueva del ROB (x0 nunca se renombra; f0 SI, es un
                // registro normal en el banco F)
                ap_uint<5> dest = rob[new_tag].dest;
                if (rob[new_tag].dest_is_fp) {
                    frat[dest].has_tag = true;
                    frat[dest].tag = new_tag;
                } else if (dest != 0) {
                    rat[dest].has_tag = true;
                    rat[dest].tag = new_tag;
                }
                disp_valid = 1;
                disp_tag = new_tag;
                disp_pc = this_pc;
                rob_tail = rob_tail + 1;
                rob_count = rob_count + 1;
                fetch_pc = next_fetch;
            }
        }
    }

    halted = (ecall_halt || (fetch_done && rob_count == 0)) ? ap_uint<1>(1) : ap_uint<1>(0);

    for (int i = 0; i < OOO_VEC_REGFILE_LEN; i++) {
        vregs_out[i] = vregs[i];
    }
}
