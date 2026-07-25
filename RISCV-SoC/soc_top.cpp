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
#include "soc_config.h"
#include "rvv_encoding.h"
#include "soc_state.h"
#include "soc_csr.h"
#include "backend.h"
#include "exec_vector.h"
#include "axi_interconnect.h"
#include "rv32c_defs.h"  // rv32c::expand, para las instrucciones comprimidas
#include "tage.h"        // el predictor de saltos del frontend
#include "vector_coprocessor.h" // VIQ + VALU/VMUL/VSLDU/VLSU
#include "frontend_dispatch.h"    // etapa 4: la tabla de decodificacion completa

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
    branch_pending = 0;   // los branches en vuelo se descartaron con el ROB
    viq_reset();          // la cola del coprocesador tambien era especulable
}

// ================= el tick =================

void riscv_soc_tick(
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
        tage_reset();                  // predictor y GHR en frio
        viq_reset();
        branch_pending = 0;
        stat_branches = 0; stat_mispredicts = 0;
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

    mem_port_used = false;  // el arbitro del interconnect concede de nuevo

    // ---- Etapa 1: commit (retiro en orden desde la cabeza del ROB) ----
    if (rob_count > 0 && rob[rob_head].valid && rob[rob_head].ready) {
        RobEntry& h = rob[rob_head];
        if (h.is_branch) {
            // ---- resolucion del branch ESPECULADO, en el commit ----
            // El predictor se entrena aqui (con la verdad y en orden de
            // programa) y aqui se salda la apuesta del fetch. Este es el
            // mismo punto preciso donde se toman los traps -- y un
            // mispredict usa EXACTAMENTE su mecanismo: redirigir + flush.
            tage_update(h.pc, h.ghr_snap, h.br_taken_real, h.br_mispred);
            if (branch_pending > 0) branch_pending = branch_pending - 1;
            // los contadores son de branches RETIRADOS (arquitectonicos):
            // los especulados que un flush descarto no cuentan
            stat_branches = stat_branches + 1;
            commit_valid = 1; commit_is_fp = 0; commit_rd = 0; commit_value = 0;
            if (h.br_mispred) {
                stat_mispredicts = stat_mispredicts + 1;
                // reparar la historia global: lo especulado tras este
                // branch nunca ocurrio; la historia real es snapshot+real
                tage_ghr = (h.ghr_snap << 1) |
                           ap_uint<16>(h.br_taken_real ? 1 : 0);
                fetch_pc = h.br_target;   // el camino CORRECTO
                fetch_done = false;
                pipeline_flush();         // descarta todo lo especulado
            } else {
                h.valid = false;
                rob_head = rob_head + 1;
                rob_count = rob_count - 1;
            }
            for (int i = 0; i < OOO_VEC_REGFILE_LEN; i++) vregs_out[i] = vregs[i];
            halted = ecall_halt ? ap_uint<1>(1) : ap_uint<1>(0);
            return;
        }
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
                // orden de programa en memoria sin necesitar disambiguation.
                // Tambien pasa por el ARBITRO: el commit va primero en el
                // tick, asi que siempre gana -- pero consume el puerto, y
                // la VLSU/LSU de este ciclo lo veran ocupado (1 acceso/ciclo
                // de verdad, tambien contra los stores en retiro).
                axi_grant();
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
                // JALR sigue el esquema viejo: el fetch estaba detenido,
                // aqui se calcula el destino real y se reanuda.
                fetch_pc = (br_rs.s1.val + ap_uint<32>(br_rs.imm)) & ap_uint<32>(~1u);
                rob[br_rs.rob_tag].value = link;
                fetch_stalled = false;
            } else {
                // Branch condicional ESPECULADO: aqui solo se comprueba la
                // prediccion y se ANOTA el veredicto en el ROB. La
                // redireccion (si hizo falta) ocurre en el COMMIT -- el
                // mismo punto preciso donde se toman los traps, y por la
                // misma razon: nada posterior ha tocado estado.
                bool taken = branch_taken(br_rs.f3, br_rs.s1.val, br_rs.s2.val);
                RobEntry& b = rob[br_rs.rob_tag];
                b.br_taken_real = taken;
                b.br_target = taken ? ap_uint<32>(br_rs.br_pc + ap_uint<32>(br_rs.imm))
                                    : link;
                b.br_mispred = (taken != b.pred_taken);
                b.value = 0;
            }
            rob[br_rs.rob_tag].ready = true;
            cdb_broadcast(br_rs.rob_tag, rob[br_rs.rob_tag].value);
            br_done = 1; br_tag = br_rs.rob_tag;
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
        } else if (lsu_rs.rob_tag == rob_head && rob[rob_head].valid &&
                   axi_grant()) {  // el interconnect concede el puerto
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
    // a 128 bits para cargar los 4 lanes de un store vectorial).    // ---- coprocesador vectorial: VIQ + VALU/VMUL/VSLDU/VLSU ----
    // (bloque derecho del diagrama; ver vector_coprocessor.h)
    vector_coprocessor_tick(dmem, vec_done, vec_tag);
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
            rob[new_tag].is_branch = false;  // solo el branch condicional
            rob[new_tag].br_mispred = false;
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


// ---- observabilidad del predictor (solo csim: el testbench los lee) ----
// En hardware estos contadores se expondrian como registros del esclavo
// AXI4-Lite de control; para la verificacion alcanza con leerlos directo.
#ifndef __SYNTHESIS__
unsigned int soc_stat_branches()    { return stat_branches.to_uint(); }
unsigned int soc_stat_mispredicts() { return stat_mispredicts.to_uint(); }
#endif
