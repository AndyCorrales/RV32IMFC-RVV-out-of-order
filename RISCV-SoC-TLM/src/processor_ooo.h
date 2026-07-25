#ifndef PROCESSOR_OOO_H
#define PROCESSOR_OOO_H

#include <systemc.h>
#include <tlm.h>
#include <tlm_utils/simple_initiator_socket.h>
#include <array>
#include <cstring>
#include <cmath>
#include <iostream>
#include <vector>

#include "rv32i_defs.h"
#include "immediates.h"
#include "fp_ops.h"
#include "rv32c_defs.h"
#include "rvv_encoding.h"  // codificacion RVV 1.0 (constantes puras)
#include "vector_alu.h"    // ALU vectorial: funciones puras por elemento
#include "tage.h"          // predictor de saltos del frontend (espejo de la pista HLS)

// CPU RV32IMFC con ejecucion FUERA DE ORDEN (Tomasulo-lite). Mismo rol de
// initiator TLM-2.0 que Processor (processor.h) -- mismo socket, mismo
// bus_access de 9 pasos, mismo fetch de 16 bits -- pero internamente el
// modelo es de microarquitectura por ciclos:
//
//   - RAT (enteros) + FRAT (flotantes) apuntando a un ROB unificado de 8
//     entradas con retiro en orden.
//   - Unidades funcionales con reservation station propia: 2x ALU (lat 1),
//     1x MUL/DIV (lat 3/8), 1x FPU (F completa, familia FMADD con rs3;
//     lat 3/8/4/2 segun op), 1x LSU, 1x branch, 1x VEC (coprocesamiento
//     vectorial RVV, ver abajo). LUI/AUIPC/JAL resuelven en el dispatch.
//   - CDB: cada unidad difunde (tag, valor) al completar y despierta a
//     las RS en espera. Los valores F viajan como bits IEEE-754 crudos.
//
// Coprocesamiento vectorial (RVV, unidad VecRs): mismas 5 instrucciones
// y misma codificacion de bits verificada contra la especificacion
// oficial RVV v1.0 que la pista HLS (rv32_ooo.cpp) -- vle32.v/vse32.v
// (memoria unit-stride) + vadd.vv/vsub.vv/vmul.vv (aritmetica
// vector-vector), MAS vsetvli/vsetivli/vsetvl con vl DINAMICO
// (vl = min(AVL,VLMAX)) y tail-undisturbed. Unica configuracion
// soportada: SEW=32/LMUL=1/VLEN=128 -> VLMAX=4 (otra activa vill). Sin
// mascara. Se despachan desde el MISMO decoder OOO que el resto del
// programa: un vadd.vv puede estar ejecutando mientras instrucciones
// escalares independientes completan fuera de orden alrededor suyo --
// eso es coprocesamiento con un core OOO, no dos decoders separados. El
// banco vectorial (vregs) no se renombra via el ROB: una sola RS VEC
// serializa las instrucciones vectoriales entre si (como la LSU). La
// distincion con FLW/FSW escalar es el campo width (010=escalar,
// 110=vectorial de 32b). vle32.v/vse32.v se resuelven en la cabeza del
// ROB, con acceso al Bus real (respetando la convencion TLM).
//
// Decisiones de alcance (mismas que la pista HLS, rv32_ooo.h):
//   - SIN especulacion de saltos: fetch detenido hasta resolver
//     BRANCH/JALR (JAL redirige en el dispatch). Sin squash/recovery.
//   - Memoria EN ORDEN: un acceso en vuelo; un load ejecuta solo en la
//     cabeza del ROB; un store escribe por el Bus recien al commit.
//   - Sin CSRs de F (rm=RNE fijo). RVV minimo (ver arriba), sin el resto
//     del ISA vectorial.
//
// Convenciones TLM del proyecto respetadas: sin sc_signal, todo acceso a
// memoria via b_transport bloqueante por el Bus (fetch de 16 bits, loads
// de 1/2/4 bytes -- aca la memoria es de bytes, asi que LB/LH/desalineados
// funcionan nativamente, sin el read-modify-write de la pista HLS). El
// "ciclo" es un tick del bucle de run(): el tiempo simulado avanza 1ns
// nominal por tick MAS las latencias reales que el Bus/Memory anotan en
// b_transport (estilo LT: el tiempo se cobra donde se genera).
//
// Para la evidencia de reordenamiento, el modulo registra en que ciclo
// completo cada instruccion (indexado por orden de dispatch) en
// complete_cycle[], y cuenta dispatches/commits -- el testbench
// (main_ooo.cpp) verifica con eso que una instruccion posterior completo
// antes que una anterior, sin acceso backdoor al estado interno.
SC_MODULE(ProcessorOOO) {
    tlm_utils::simple_initiator_socket<ProcessorOOO, 32> init_socket;

    static const int ROB_SZ = 8;
    static const int N_ALU  = 2;
    static const int ALU_LAT = 1, MUL_LAT = 3, DIV_LAT = 8, BR_LAT = 1;
    static const int FPU_LAT_ADDMUL = 3, FPU_LAT_DIV = 8, FPU_LAT_FMA = 4, FPU_LAT_MISC = 2;
// latencias del COPROCESADOR, por unidad funcional (ver el diagrama):
    static const int VALU_LAT  = 2; // aritmetica/logica/mascaras/reducciones
    static const int VMUL_LAT  = 4; // multiplicacion/division vectorial
    static const int VSLDU_LAT = 2; // permutaciones (slide/gather/compress)

    // ---- coprocesamiento vectorial (RVV): VLEN=128/SEW=32/LMUL=1 -> 4 lanes,
    // mismos valores y misma codificacion de bits que la pista HLS
    // (rv32_ooo.cpp / rv32_vector.cpp), verificados contra la
    // especificacion oficial RVV v1.0 ----
    static const int VEC_NUM_VREGS = 32;
    static const int VEC_LANES     = 4;
    static const int VEC_REGFILE_LEN = VEC_NUM_VREGS * VEC_LANES;
    // opcodes/campos RVV (no viven en rv32i_defs.h: ese header es
    // compartido con Processor in-order, no hace falta tocarlo)
    // (la codificacion RVV pura vive ahora en rvv_encoding.h)
    // VLEN = VEC_LANES*32 = 128 bits. Con EEW variable, el numero de
    // elementos por registro depende de SEW: 128/8=16, 128/16=8, 128/32=4.
    // El ALMACENAMIENTO no cambia (siempre 4 palabras por registro): lo
    // que cambia es como se indexan los elementos dentro de esos bits.
    // Como 8/16/32 dividen exacto a 32, ningun elemento cruza palabra.
    static const int VEC_VLEN_BITS  = VEC_LANES * 32;   // 128
    static const int VEC_MAX_ELEMS  = VEC_VLEN_BITS / 8; // 16 (peor caso, SEW=8)
    // (los modos de load/store tambien estan en rvv_encoding.h)

    // ---- estado arquitectonico ----
    std::array<uint32_t, 32> regs{};
    std::array<float, 32> fregs{};
    std::array<uint32_t, VEC_REGFILE_LEN> vregs{}; // banco vectorial (sin renombrar)
    // CSRs vectoriales: los escribe la familia vsetvl*, no csrw.
    uint32_t csr_vtype = (1u << RVV_VILL_BIT); // al reset: vill activo
    uint32_t csr_vl = 0;                       // ...y vl=0 (spec 3.11)
    uint32_t csr_vstart = 0;                   // Fase 6: primer elemento a ejecutar
    // --- CSRs de modo maquina (bare-metal) ---
    uint32_t csr_mstatus = 0, csr_mie = 0, csr_mtvec = 0, csr_mscratch = 0;
    uint32_t csr_mepc = 0, csr_mcause = 0, csr_mtval = 0, csr_mip = 0;
    uint32_t csr_mtime = 0, csr_mtimecmp = 0xFFFFFFFF; // timer
    uint8_t  cur_priv = 3;                             // 3 = M-mode, 0 = U-mode
    bool     ecall_halt = false;
    bool halted = false;
    sc_event finished;

    // ---- observabilidad para el testbench ----
    bool trace = true;
    uint64_t cycle = 0;
    int n_disp = 0, n_commit = 0;
    std::array<int, 64> complete_cycle; // por indice de dispatch
    std::array<uint32_t, 64> pc_of_disp; // pc de cada dispatch: los checks de
                                         // orden van POR PC (con TAGE hay
                                         // dispatches especulativos descartados)

    struct Operand { bool ready; uint32_t val; uint8_t tag; };
    struct RatEntry { bool has_tag; uint8_t tag; };
    struct RobEntry {
        bool valid, ready, is_store, dest_is_fp;
        bool takes_trap;  // al retirarse toma un TRAP (causa en `cause`)
        bool is_mret;     // MRET: retorno desde el handler
        // ---- especulacion de branches (frontend TAGE) ----
        bool     is_branch;     // branch CONDICIONAL especulado
        bool     pred_taken;    // que predijo TAGE en el dispatch
        bool     br_taken_real; // el resultado real (lo pone la unidad BR)
        bool     br_mispred;    // real != predicho -> el commit redirige+flush
        uint16_t ghr_snap;      // historia global AL PREDECIR
        uint32_t br_target;     // destino REAL (adonde ir si hubo mispredict)
        uint8_t dest;
        uint32_t value;   // para F: bits IEEE-754 crudos
        uint32_t addr, sdata; // solo stores
        uint8_t mem_f3;
        uint32_t pc;      // pc de la instruccion (va a mepc si hay trap)
        uint8_t cause;    // codigo de causa del trap (mcause)
        int disp_idx;     // orden de dispatch (para complete_cycle[])
    };
    struct AluRs {
        bool busy, executing; uint8_t remaining;
        uint8_t f3; bool alt; uint8_t rob_tag;
        Operand s1, s2;
    };
    struct MdRs {
        bool busy, executing; uint8_t remaining;
        uint8_t f3; uint8_t rob_tag;
        Operand s1, s2;
    };
    struct FpuRs {
        bool busy, executing; uint8_t remaining;
        uint8_t r4op;  // 0=OP_FP; 1..4 = FMADD/FMSUB/FNMSUB/FNMADD
        uint8_t f7, f3, rs2f, rob_tag;
        Operand s1, s2, s3; // s3: tercer operando de la familia R4
    };
    struct LsuRs {
        bool busy, is_load;
        uint8_t f3, rob_tag;
        int32_t imm;
        Operand s1, s2; // s1: base (banco entero); s2: dato del store
    };
    struct BrRs {
        bool busy, executing; uint8_t remaining;
        bool is_jalr;
        uint8_t f3, rob_tag, size; // size: 2 o 4 (extension C)
        uint32_t br_pc;
        int32_t imm;
        Operand s1, s2;
    };
    // Unidad de SISTEMA: una sola reservation station para las
    // instrucciones CSR, que ejecuta SOLO en la cabeza del ROB -- asi el
    // acceso a los CSRs queda en orden de programa. ECALL/EBREAK/MRET no
    // pasan por aca: se marcan en el ROB y se resuelven al retirarse.
    struct SysRs {
        bool busy;
        uint8_t f3;        // CSRRW/S/C y variantes con inmediato
        uint16_t csr_addr; // instr[31:20]
        uint8_t rob_tag;
        Operand s1;        // x[rs1] o zimm de las variantes con inmediato
    };

    // Unidad vectorial (RVV): una sola reservation station, como la LSU --
    // serializa las instrucciones vectoriales ENTRE SI (no hay VRAT que
    // las renombre), pero se solapan libremente con instrucciones
    // escalares independientes que completen fuera de orden alrededor
    // suyo. Eso es coprocesamiento con un core OOO, no "otro decoder".
    struct VecRs {
        bool busy, executing; uint8_t remaining;
        bool is_load, is_store, is_arith;
        uint8_t vd_or_vs3, vs1, vs2; // mismos bits que rd/rs1/rs2
        uint8_t funct6, funct3;      // operacion y forma (.vv/.vx/.vi)
        bool    vm;                  // 1 = sin mascara; 0 = predicado por v0
        uint8_t rob_tag;
        // s1 tiene doble uso: direccion base (memoria) u operando ESCALAR
        // de las formas .vx/.vi (aritmetica). En ambos casos pasa por el
        // mecanismo normal de operandos, asi que la unidad espera al CDB
        // si el productor sigue en vuelo.
        Operand s1;
        Operand s2;                  // paso en bytes (rs2) -- solo strided
        uint8_t vl;                  // largo vectorial capturado en el dispatch
        uint8_t sew_b;               // ancho de elemento en BYTES (1, 2 o 4)
        uint8_t vcat;                // categoria (VCAT_*): que destino escribe
        uint8_t vstart;              // primer elemento a ejecutar (CSR vstart, seccion 3.7)
        uint8_t elem;                // elemento en curso: la VLSU mueve UNO
                                     // por ciclo (puerto de memoria arbitrado)
        uint8_t vs1_field;           // campo vs1 crudo (selector de los grupos unary)
        // ---- Fase 5: modo de direccionamiento del acceso a memoria ----
        uint8_t mop;                 // 00 unit-stride, 01/11 indexado, 10 strided
        uint8_t nf;                  // campos por segmento MENOS UNO
        uint8_t fld;                 // campo que toca procesar en este ciclo
        uint8_t lsmode;              // LSM_*: normal / mascara / reg. completo / fof
        uint8_t idx_b;               // ancho del INDICE en bytes (solo indexado)
    };

    // ---- estado de la microarquitectura ----
    std::array<RatEntry, 32> rat{};
    std::array<RatEntry, 32> frat{};
    std::array<RobEntry, ROB_SZ> rob{};
    uint8_t rob_head = 0, rob_tail = 0, rob_count = 0;
    std::array<AluRs, N_ALU> alu_rs{};
    MdRs  md_rs{};
    FpuRs fpu_rs{};
    LsuRs lsu_rs{};
    BrRs  br_rs{};
    VecRs vec_rs{};
    SysRs sys_rs{};
    uint32_t fetch_pc = 0;
    bool fetch_stalled = false, fetch_done = false;   // JALR aun detiene el
                                                      // fetch; los branches ya NO (TAGE)
    // ---- frontend especulativo ----
    Tage     tage;
    uint8_t  branch_pending = 0;       // branches condicionales en vuelo
    uint32_t stat_branches = 0, stat_mispredicts = 0; // retirados (suite E)
    // ---- arbitro del interconnect: UNA concesion de memoria por ciclo ----
    bool mem_port_used = false;
    bool axi_grant() {
        if (mem_port_used) return false;
        mem_port_used = true;
        return true;
    }
    // ---- Vector Instruction Queue del coprocesador (FIFO de 4) ----
    static const int VIQ_SZ = 4;
    VecRs   viq[VIQ_SZ];
    uint8_t viq_head = 0, viq_tail = 0, viq_count = 0;
    void viq_reset() { viq_head = 0; viq_tail = 0; viq_count = 0; }
    bool viq_full() const { return viq_count == VIQ_SZ; }
    void viq_push(const VecRs& v) {
        viq[viq_tail] = v;
        viq_tail = (viq_tail + 1) & (VIQ_SZ - 1);
        viq_count++;
    }

    // El core NO arranca por su cuenta: el testbench lo resetea y lo corre
    // una vez por suite (ver main.cpp). Por eso run_until_halt() es un
    // metodo normal y no un SC_THREAD -- los wait() que hace por dentro
    // suspenden al proceso del testbench que lo invoca, que es valido en
    // SystemC y permite ejecutar varios programas en una sola simulacion.
    SC_CTOR(ProcessorOOO) : init_socket("init_socket") {
        reset_state();
    }

    // Deja el core como recien salido del reset. Necesario para poder
    // correr varios binarios distintos en la misma simulacion.
    void reset_state() {
        tage.reset(); branch_pending = 0;
        stat_branches = 0; stat_mispredicts = 0;
        viq_reset(); mem_port_used = false;
        regs.fill(0); fregs.fill(0.0f); vregs.fill(0);
        csr_vtype = (1u << RVV_VILL_BIT); csr_vl = 0; csr_vstart = 0;
        csr_mstatus = 0; csr_mie = 0; csr_mtvec = 0; csr_mscratch = 0;
        csr_mepc = 0; csr_mcause = 0; csr_mtval = 0; csr_mip = 0;
        csr_mtime = 0; csr_mtimecmp = 0xFFFFFFFF;
        cur_priv = 3; ecall_halt = false;
        for (auto& r : rat)  { r.has_tag = false; r.tag = 0; }
        for (auto& r : frat) { r.has_tag = false; r.tag = 0; }
        for (auto& e : rob)  { e.valid = false; e.ready = false; e.takes_trap = false;
                               e.is_mret = false; e.is_store = false; e.dest_is_fp = false; }
        rob_head = 0; rob_tail = 0; rob_count = 0;
        for (auto& a : alu_rs) { a.busy = false; a.executing = false; }
        md_rs.busy = false;  md_rs.executing = false;
        fpu_rs.busy = false; fpu_rs.executing = false;
        lsu_rs.busy = false;
        br_rs.busy = false;  br_rs.executing = false;
        vec_rs.busy = false; vec_rs.executing = false;
        sys_rs.busy = false;
        fetch_pc = 0; fetch_stalled = false; fetch_done = false;
        halted = false; cycle = 0; n_disp = 0; n_commit = 0;
        pc_of_disp.fill(0xFFFFFFFFu);
        complete_cycle.fill(-1);
    }

    // ---- acceso al Bus: identico a Processor (processor.h) ----
    void bus_access(tlm::tlm_command cmd, uint32_t addr, uint8_t* data, unsigned int len) {
        tlm::tlm_generic_payload trans;
        sc_time delay = SC_ZERO_TIME;

        trans.set_command(cmd);
        trans.set_address(addr);
        trans.set_data_ptr(data);
        trans.set_data_length(len);
        trans.set_streaming_width(len);
        trans.set_byte_enable_ptr(nullptr);
        trans.set_dmi_allowed(false);
        trans.set_response_status(tlm::TLM_INCOMPLETE_RESPONSE);

        init_socket->b_transport(trans, delay);
        wait(delay);

        if (trans.get_response_status() != tlm::TLM_OK_RESPONSE) {
            std::cerr << "[ProcessorOOO] Error de bus en direccion 0x" << std::hex << addr << std::dec << std::endl;
            halted = true;
        }
    }

    uint16_t fetch16(uint32_t addr) {
        uint16_t half = 0;
        bus_access(tlm::TLM_READ_COMMAND, addr, reinterpret_cast<uint8_t*>(&half), 2);
        return half;
    }

    uint32_t load(uint32_t addr, unsigned int len) {
        uint32_t value = 0;
        bus_access(tlm::TLM_READ_COMMAND, addr, reinterpret_cast<uint8_t*>(&value), len);
        return value;
    }

    void store(uint32_t addr, uint32_t value, unsigned int len) {
        bus_access(tlm::TLM_WRITE_COMMAND, addr, reinterpret_cast<uint8_t*>(&value), len);
    }

    // ---- helpers combinacionales (mismos que la pista HLS rv32_ooo.cpp) ----

    void cdb_broadcast(uint8_t tag, uint32_t value) {
        for (int i = 0; i < N_ALU; i++) {
            if (alu_rs[i].busy && !alu_rs[i].s1.ready && alu_rs[i].s1.tag == tag) {
                alu_rs[i].s1.ready = true; alu_rs[i].s1.val = value;
            }
            if (alu_rs[i].busy && !alu_rs[i].s2.ready && alu_rs[i].s2.tag == tag) {
                alu_rs[i].s2.ready = true; alu_rs[i].s2.val = value;
            }
        }
        if (md_rs.busy && !md_rs.s1.ready && md_rs.s1.tag == tag) { md_rs.s1.ready = true; md_rs.s1.val = value; }
        if (md_rs.busy && !md_rs.s2.ready && md_rs.s2.tag == tag) { md_rs.s2.ready = true; md_rs.s2.val = value; }
        if (fpu_rs.busy && !fpu_rs.s1.ready && fpu_rs.s1.tag == tag) { fpu_rs.s1.ready = true; fpu_rs.s1.val = value; }
        if (fpu_rs.busy && !fpu_rs.s2.ready && fpu_rs.s2.tag == tag) { fpu_rs.s2.ready = true; fpu_rs.s2.val = value; }
        if (fpu_rs.busy && !fpu_rs.s3.ready && fpu_rs.s3.tag == tag) { fpu_rs.s3.ready = true; fpu_rs.s3.val = value; }
        if (lsu_rs.busy && !lsu_rs.s1.ready && lsu_rs.s1.tag == tag) { lsu_rs.s1.ready = true; lsu_rs.s1.val = value; }
        if (lsu_rs.busy && !lsu_rs.s2.ready && lsu_rs.s2.tag == tag) { lsu_rs.s2.ready = true; lsu_rs.s2.val = value; }
        if (br_rs.busy && !br_rs.s1.ready && br_rs.s1.tag == tag) { br_rs.s1.ready = true; br_rs.s1.val = value; }
        if (br_rs.busy && !br_rs.s2.ready && br_rs.s2.tag == tag) { br_rs.s2.ready = true; br_rs.s2.val = value; }
        if (vec_rs.busy && !vec_rs.s1.ready && vec_rs.s1.tag == tag) { vec_rs.s1.ready = true; vec_rs.s1.val = value; }
        if (vec_rs.busy && !vec_rs.s2.ready && vec_rs.s2.tag == tag) { vec_rs.s2.ready = true; vec_rs.s2.val = value; }
        // ...y las instrucciones que ESPERAN EN LA VIQ tambien escuchan
        // el CDB: pueden haberse encolado con un operando en vuelo.
        for (int q = 0; q < VIQ_SZ; q++) {
            if (!viq[q].s1.ready && viq[q].s1.tag == tag) { viq[q].s1.ready = true; viq[q].s1.val = value; }
            if (!viq[q].s2.ready && viq[q].s2.tag == tag) { viq[q].s2.ready = true; viq[q].s2.val = value; }
        }
        if (sys_rs.busy && !sys_rs.s1.ready && sys_rs.s1.tag == tag) { sys_rs.s1.ready = true; sys_rs.s1.val = value; }
    }

    // Aritmetica RVV vector-vector -- misma semantica que rv32_vector.cpp
    // (elemento a elemento sobre los VEC_LANES lanes).
    // ---- acceso a elementos con ancho SEW variable ----
    // El registro vectorial son VEC_LANES palabras de 32 bits (128 bits).
    // El elemento `idx` de ancho `sew_b` bytes vive en el bit
    // idx*sew_b*8 de esa tira; como 8/16/32 dividen exacto a 32, nunca
    // cruza el limite de palabra.
    uint32_t vreg_get(uint8_t vreg, int idx, uint8_t sew_b) const {
        int bit_off = idx * sew_b * 8;
        uint32_t w = vregs[vreg * VEC_LANES + (bit_off / 32)];
        uint32_t raw = w >> (bit_off % 32);
        if (sew_b == 1) return raw & 0xFFu;
        if (sew_b == 2) return raw & 0xFFFFu;
        return raw;
    }
    void vreg_set(uint8_t vreg, int idx, uint8_t sew_b, uint32_t val) {
        int bit_off = idx * sew_b * 8;
        int wi = vreg * VEC_LANES + (bit_off / 32);
        int shift = bit_off % 32;
        if (sew_b == 4) { vregs[wi] = val; return; } // 1u<<32 seria UB
        uint32_t field = (1u << (sew_b * 8)) - 1;
        vregs[wi] = (vregs[wi] & ~(field << shift)) | ((val & field) << shift);
    }
    // ---- banco de CSRs -------------------------------------------------
    // Subset minimo de modo maquina: almacenamiento simple, sin logica
    // WARL. Alcanza para que el crt0 de un binario compilado configure
    // mtvec/stack y llegue a main. misa/mhartid son de solo lectura.
    uint32_t read_csr(uint16_t addr) const {
        switch (addr) {
            case rv32i::CSR::MSTATUS:  return csr_mstatus;
            case rv32i::CSR::MISA:     return 0x40001104; // RV32 IMFC
            case rv32i::CSR::MIE:      return csr_mie;
            case rv32i::CSR::MTVEC:    return csr_mtvec;
            case rv32i::CSR::MSCRATCH: return csr_mscratch;
            case rv32i::CSR::MEPC:     return csr_mepc;
            case rv32i::CSR::MCAUSE:   return csr_mcause;
            case rv32i::CSR::MTVAL:    return csr_mtval;
            case rv32i::CSR::MIP:      return csr_mip;
            case rv32i::CSR::MHARTID:  return 0;
            case rv32i::CSR::MTIME:    return csr_mtime;
            case rv32i::CSR::MTIMECMP: return csr_mtimecmp;
            case rv32i::CSR::VSTART:   return csr_vstart;
            case rv32i::CSR::VL:       return csr_vl;
            case rv32i::CSR::VTYPE:    return csr_vtype;
            case rv32i::CSR::VLENB:    return VEC_LANES * 4;
            default:                   return 0;
        }
    }
    void write_csr(uint16_t addr, uint32_t v) {
        switch (addr) {
            // vstart es read-write (seccion 3.7): permite reanudar a mano
            // y que una biblioteca de hilos guarde/restaure el estado.
            case rv32i::CSR::VSTART:   csr_vstart = v & (VEC_MAX_ELEMS - 1); break;
            case rv32i::CSR::MSTATUS:  csr_mstatus = v; break;
            case rv32i::CSR::MIE:      csr_mie = v; break;
            case rv32i::CSR::MTVEC:    csr_mtvec = v; break;
            case rv32i::CSR::MSCRATCH: csr_mscratch = v; break;
            case rv32i::CSR::MEPC:     csr_mepc = v; break;
            case rv32i::CSR::MCAUSE:   csr_mcause = v; break;
            case rv32i::CSR::MTVAL:    csr_mtval = v; break;
            case rv32i::CSR::MIP:      csr_mip = v; break;
            case rv32i::CSR::MTIMECMP: csr_mtimecmp = v; break;
            default: break; // misa/mhartid/vl/vtype: solo lectura
        }
    }

    // Descarta todo lo que hay en vuelo y reinicia el frente del pipeline.
    // Es SUFICIENTE para excepciones precisas: como este core no especula,
    // un trap solo puede tomarse en la CABEZA del ROB, donde todo lo
    // anterior ya committeo y nada posterior modifico el estado
    // arquitectonico. Por eso no hace falta recuperacion especulativa.
    void pipeline_flush() {
        for (int i = 0; i < ROB_SZ; i++) { rob[i].valid = false; rob[i].ready = false; }
        rob_head = 0; rob_tail = 0; rob_count = 0;
        for (int i = 0; i < 32; i++) { rat[i].has_tag = false; frat[i].has_tag = false; }
        for (int i = 0; i < N_ALU; i++) { alu_rs[i].busy = false; alu_rs[i].executing = false; }
        md_rs.busy = false;  md_rs.executing = false;
        fpu_rs.busy = false; fpu_rs.executing = false;
        lsu_rs.busy = false;
        br_rs.busy = false;  br_rs.executing = false;
        vec_rs.busy = false; vec_rs.executing = false;
        sys_rs.busy = false;
        fetch_stalled = false;
        branch_pending = 0;   // los branches en vuelo se descartaron con el ROB
        viq_reset();          // la cola del coprocesador tambien era descartable
    }

    // ---- acceso a REGISTROS DE MASCARA ----
    // Un registro de mascara guarda un bit por elemento: el bit del
    // elemento i vive en el bit i del registro, independiente de SEW
    // (spec 4.5). Con VLEN=128 y hasta 16 elementos, todos los bits caben
    // en la primera palabra.
    bool vmask_get(uint8_t vreg, int idx) const {
        return (vregs[vreg * VEC_LANES + (idx / 32)] >> (idx % 32)) & 1u;
    }
    void vmask_set(uint8_t vreg, int idx, bool v) {
        uint32_t& w = vregs[vreg * VEC_LANES + (idx / 32)];
        uint32_t bit = 1u << (idx % 32);
        w = v ? (w | bit) : (w & ~bit);
    }

    // SEW vigente (en bytes) segun vtype. 8<<vsew = SEW en bits.
    uint8_t cur_sew_bytes() const {
        uint32_t vsew = (csr_vtype >> 3) & 0x7;
        return static_cast<uint8_t>((8u << vsew) / 8u); // 1, 2 o 4
    }

    // extension de signo desde el ancho SEW hacia 32 bits
    // ---- Fase 4d: acceso a un GRUPO de dos registros (EMUL=2) ----
    // Un operando widening/narrowing de 2*SEW ocupa el par (vreg, vreg+1).
    // Con elementos de `wide_b` bytes caben VLEN/(wide_b*8) por registro,
    // asi que el elemento `idx` vive en vreg + idx/por_reg. El
    // almacenamiento NO cambia: es el mismo banco plano, solo cambia el
    // indice -- la misma idea que el EEW variable de la Fase 2.
    uint32_t vreg_get_pair(uint8_t vreg, int idx, uint8_t wide_b) const {
        int por_reg = VEC_VLEN_BITS / (wide_b * 8);
        return vreg_get((vreg + idx / por_reg) & 31, idx % por_reg, wide_b);
    }
    void vreg_set_pair(uint8_t vreg, int idx, uint8_t wide_b, uint32_t val) {
        int por_reg = VEC_VLEN_BITS / (wide_b * 8);
        vreg_set((vreg + idx / por_reg) & 31, idx % por_reg, wide_b, val);
    }

    // (la ALU vectorial pura vive ahora en vector_alu.h)

    // Predicado por elemento: activo si la instruccion es sin mascara
    // (vm=1) o si el bit correspondiente de v0 esta en 1. La spec (4.5)
    // define que el bit de mascara del elemento i vive en el bit i del
    // registro de mascara, independiente de SEW/LMUL.
    bool vec_elem_active(const VecRs& u, int lane) const {
        if (u.vm) return true;
        return (vregs[0 * VEC_LANES + 0] >> lane) & 1u; // v0.mask[lane]
    }

    // Solo los primeros `vl` elementos y solo los ACTIVOS segun mascara;
    // el resto (tail e inactivos) se deja sin tocar -- politicas
    // tail-undisturbed / mask-undisturbed, que la spec permite usar
    // siempre en implementaciones simples.
    // Devuelve el resultado ESCALAR de vcpop.m / vfirst.m (categoria
    // VCAT_XRES); para el resto devuelve 0.
    // ---- definidas FUERA DE LINEA, para no inflar este archivo ----
    //   processor_vector_unit.h  ejecucion de una instruccion RVV completa
    //   processor_tick.h         las cuatro etapas del pipeline
    //   processor_dispatch.h     la tabla de decodificacion RV32IMFC + RVV
    uint32_t vec_arith_compute(const VecRs& u);
    void     tick();
    void     dispatch(uint32_t instr, uint8_t isize);

    Operand read_operand(uint8_t reg) {
        Operand op;
        if (reg == 0) { op.ready = true; op.val = 0; op.tag = 0; return op; }
        if (!rat[reg].has_tag) { op.ready = true; op.val = regs[reg]; op.tag = 0; return op; }
        uint8_t t = rat[reg].tag;
        if (rob[t].ready) { op.ready = true; op.val = rob[t].value; op.tag = 0; }
        else              { op.ready = false; op.val = 0; op.tag = t; }
        return op;
    }

    // f0 es un registro normal (se renombra y escribe como cualquier otro)
    Operand read_operand_fp(uint8_t reg) {
        Operand op;
        if (!frat[reg].has_tag) {
            op.ready = true; op.val = rv32i::float_to_bits(fregs[reg]); op.tag = 0;
            return op;
        }
        uint8_t t = frat[reg].tag;
        if (rob[t].ready) { op.ready = true; op.val = rob[t].value; op.tag = 0; }
        else              { op.ready = false; op.val = 0; op.tag = t; }
        return op;
    }

    static uint32_t alu_compute(uint8_t f3, bool alt, uint32_t a_u, uint32_t b_u) {
        int32_t a = static_cast<int32_t>(a_u);
        int32_t b = static_cast<int32_t>(b_u);
        uint32_t sh = b_u & 0x1F;
        switch (f3) {
            case rv32i::Funct3_ALU::ADD_SUB: return alt ? static_cast<uint32_t>(a - b) : static_cast<uint32_t>(a + b);
            case rv32i::Funct3_ALU::SLL:     return a_u << sh;
            case rv32i::Funct3_ALU::SLT:     return (a < b) ? 1 : 0;
            case rv32i::Funct3_ALU::SLTU:    return (a_u < b_u) ? 1 : 0;
            case rv32i::Funct3_ALU::XOR:     return a_u ^ b_u;
            case rv32i::Funct3_ALU::SRL_SRA: return alt ? static_cast<uint32_t>(a >> sh) : (a_u >> sh);
            case rv32i::Funct3_ALU::OR:      return a_u | b_u;
            case rv32i::Funct3_ALU::AND:     return a_u & b_u;
            default:                         return 0;
        }
    }

    static uint32_t md_compute(uint8_t f3, uint32_t ua, uint32_t ub) {
        int32_t  sa = static_cast<int32_t>(ua), sb = static_cast<int32_t>(ub);
        int64_t  a64 = sa, b64 = sb;
        uint64_t ua64 = ua, ub64 = ub;
        switch (f3) {
            case rv32i::Funct3_MULDIV::MUL:    return ua * ub;
            case rv32i::Funct3_MULDIV::MULH:   return static_cast<uint32_t>(static_cast<uint64_t>(a64 * b64) >> 32);
            case rv32i::Funct3_MULDIV::MULHSU: return static_cast<uint32_t>(static_cast<uint64_t>(a64 * static_cast<int64_t>(ub64)) >> 32);
            case rv32i::Funct3_MULDIV::MULHU:  return static_cast<uint32_t>((ua64 * ub64) >> 32);
            case rv32i::Funct3_MULDIV::DIV:
                if (sb == 0)                       return 0xFFFFFFFF;
                if (ua == 0x80000000u && sb == -1) return 0x80000000;
                return static_cast<uint32_t>(sa / sb);
            case rv32i::Funct3_MULDIV::DIVU:   return (ub == 0) ? 0xFFFFFFFF : (ua / ub);
            case rv32i::Funct3_MULDIV::REM:
                if (sb == 0)                       return ua;
                if (ua == 0x80000000u && sb == -1) return 0;
                return static_cast<uint32_t>(sa % sb);
            case rv32i::Funct3_MULDIV::REMU:   return (ub == 0) ? ua : (ua % ub);
            default:                           return 0;
        }
    }

    static uint32_t fpu_compute(const FpuRs& u) {
        float a = rv32i::bits_to_float(u.s1.val);
        float b = rv32i::bits_to_float(u.s2.val);
        float c = rv32i::bits_to_float(u.s3.val);
        switch (u.r4op) {
            case 1: return rv32i::float_to_bits(std::fma(a, b, c));
            case 2: return rv32i::float_to_bits(std::fma(a, b, -c));
            case 3: return rv32i::float_to_bits(std::fma(-a, b, c));
            case 4: return rv32i::float_to_bits(std::fma(-a, b, -c));
        }
        switch (u.f7) {
            case rv32i::Funct7_FP::FADD_S:  return rv32i::float_to_bits(a + b);
            case rv32i::Funct7_FP::FSUB_S:  return rv32i::float_to_bits(a - b);
            case rv32i::Funct7_FP::FMUL_S:  return rv32i::float_to_bits(a * b);
            case rv32i::Funct7_FP::FDIV_S:  return rv32i::float_to_bits(a / b);
            case rv32i::Funct7_FP::FSQRT_S: return rv32i::float_to_bits(std::sqrt(a));
            case rv32i::Funct7_FP::FSGNJ_S: {
                uint32_t ab = u.s1.val, bb = u.s2.val, sign = 0;
                switch (u.f3) {
                    case rv32i::Funct3_FSGNJ::FSGNJ:  sign = bb & 0x80000000u; break;
                    case rv32i::Funct3_FSGNJ::FSGNJN: sign = (~bb) & 0x80000000u; break;
                    case rv32i::Funct3_FSGNJ::FSGNJX: sign = (ab ^ bb) & 0x80000000u; break;
                }
                return (ab & 0x7FFFFFFFu) | sign;
            }
            case rv32i::Funct7_FP::FMINMAX_S:
                return rv32i::float_to_bits(
                    (u.f3 == rv32i::Funct3_FMINMAX::FMIN) ? std::fmin(a, b) : std::fmax(a, b));
            case rv32i::Funct7_FP::FCMP_S:
                switch (u.f3) {
                    case rv32i::Funct3_FCMP::FLE: return (a <= b) ? 1 : 0;
                    case rv32i::Funct3_FCMP::FLT: return (a <  b) ? 1 : 0;
                    case rv32i::Funct3_FCMP::FEQ: return (a == b) ? 1 : 0;
                }
                return 0;
            case rv32i::Funct7_FP::FCVT_W_S:
                return (u.rs2f == rv32i::Rs2_FCVT::WU)
                           ? rv32i::fcvt_wu_s(a)
                           : static_cast<uint32_t>(rv32i::fcvt_w_s(a));
            case rv32i::Funct7_FP::FCVT_S_W:
                // s1 viene del banco ENTERO: entero crudo, no bits float
                return (u.rs2f == rv32i::Rs2_FCVT::WU)
                           ? rv32i::float_to_bits(static_cast<float>(u.s1.val))
                           : rv32i::float_to_bits(static_cast<float>(static_cast<int32_t>(u.s1.val)));
            case rv32i::Funct7_FP::FMV_X_W_FCLASS_S:
                return (u.f3 == rv32i::Funct3_FMV_FCLASS::FCLASS_S)
                           ? rv32i::fclass_s(a)
                           : u.s1.val;
            case rv32i::Funct7_FP::FMV_W_X:
                return u.s1.val;
            default:
                return 0;
        }
    }

    static bool branch_taken(uint8_t f3, uint32_t a_u, uint32_t b_u) {
        int32_t a = static_cast<int32_t>(a_u), b = static_cast<int32_t>(b_u);
        switch (f3) {
            case rv32i::Funct3_BRANCH::BEQ:  return a_u == b_u;
            case rv32i::Funct3_BRANCH::BNE:  return a_u != b_u;
            case rv32i::Funct3_BRANCH::BLT:  return a < b;
            case rv32i::Funct3_BRANCH::BGE:  return a >= b;
            case rv32i::Funct3_BRANCH::BLTU: return a_u < b_u;
            case rv32i::Funct3_BRANCH::BGEU: return a_u >= b_u;
            default:                         return false;
        }
    }

    // Load con extension de signo/cero segun f3 -- la memoria TLM es de
    // bytes, asi que LB/LH/desalineados funcionan nativamente (sin el
    // read-modify-write de la pista HLS).
    uint32_t lsu_load_value(uint32_t addr, uint8_t f3) {
        switch (f3) {
            case rv32i::Funct3_LOAD::LB:  return static_cast<uint32_t>(static_cast<int32_t>(static_cast<int8_t>(load(addr, 1))));
            case rv32i::Funct3_LOAD::LH:  return static_cast<uint32_t>(static_cast<int32_t>(static_cast<int16_t>(load(addr, 2))));
            case rv32i::Funct3_LOAD::LW:  return load(addr, 4);
            case rv32i::Funct3_LOAD::LBU: return load(addr, 1) & 0xFF;
            case rv32i::Funct3_LOAD::LHU: return load(addr, 2) & 0xFFFF;
            default:                      return 0;
        }
    }

    static unsigned store_len(uint8_t f3) {
        switch (f3) {
            case rv32i::Funct3_STORE::SB: return 1;
            case rv32i::Funct3_STORE::SH: return 2;
            default:                      return 4; // SW / FSW
        }
    }

    // ---- un tick = un ciclo (mismas 4 etapas que la pista HLS) ----
    // marca la entrada lista, difunde por el CDB y registra el ciclo
    void complete_entry(uint8_t tag, uint32_t value, const char* unit) {
        rob[tag].value = value;
        rob[tag].ready = true;
        cdb_broadcast(tag, value);
        record_complete(tag, unit);
    }

    void record_complete(uint8_t tag, const char* unit) {
        int d = rob[tag].disp_idx;
        if (d >= 0 && d < static_cast<int>(complete_cycle.size()))
            complete_cycle[d] = static_cast<int>(cycle);
        if (trace)
            std::cout << "[" << cycle << "] " << unit << " completa d" << d
                      << " (tag " << +tag << ")" << std::endl;
    }

    // Corre hasta que el programa se detiene solo. Devuelve true si
    // termino, false si se agoto el limite de ciclos (guarda contra
    // cuelgues). Se invoca desde el SC_THREAD del testbench.
    bool run_until_halt(uint64_t max_cycles = 100000) {
        while (!halted && cycle < max_cycles) {
            cycle++;
            tick();
            wait(sc_time(1, SC_NS)); // ciclo nominal (las latencias de
                                     // memoria se suman via b_transport)
        }
        finished.notify();
        return halted;
    }

    void dump_regs() {
        std::cout << "Registros enteros (OOO):" << std::endl;
        for (int i = 0; i < 32; i++) {
            std::cout << "x" << i << "=0x" << std::hex << regs[i] << std::dec
                      << ((i % 8 == 7) ? "\n" : " ");
        }
    }

    void dump_fregs() {
        std::cout << "Registros flotantes (OOO):" << std::endl;
        for (int i = 0; i < 32; i++) {
            std::cout << "f" << i << "=0x" << std::hex << rv32i::float_to_bits(fregs[i]) << std::dec
                      << ((i % 8 == 7) ? "\n" : " ");
        }
    }
};

#endif // PROCESSOR_OOO_H
