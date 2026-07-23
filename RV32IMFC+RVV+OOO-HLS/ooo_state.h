#ifndef OOO_STATE_H
#define OOO_STATE_H

// El ESTADO del procesador: los structs de cada estacion de reserva y del
// ROB, y las variables que en el RTL son registros. Todo es `static` a
// proposito -- el diseno es una sola unidad de traduccion, asi que las
// etapas del pipeline los ven directamente sin pasarlos por parametros.
#include "rv32_ooo.h"
#include "rv32i_defs.h"
#include "ooo_config.h"
#include "rvv_encoding.h"

// ================= estado persistente (registros en RTL) =================

struct RatEntry {
    bool       has_tag;
    ap_uint<3> tag;
};

struct RobEntry {
    bool        valid;
    bool        ready;      // listo para retirarse
    bool        is_store;   // el commit debe escribir memoria
    bool        dest_is_fp; // el commit escribe f[dest] (f0 SI es escribible)
    bool        takes_trap; // al retirarse toma un TRAP (la causa esta en `cause`):
                            // 2 = instruccion ilegal, 3 = EBREAK, 11/8 = ECALL
    bool        is_mret;    // MRET: al retirarse vuelve desde el handler
    ap_uint<32> pc;         // pc de la instruccion (va a mepc si hay trap)
    ap_uint<5>  cause;      // codigo de causa del trap (mcause)
    ap_uint<5>  dest;       // registro destino (entero: 0 = no escribe)
    ap_uint<32> value;      // para F: bits IEEE-754 crudos
    ap_uint<32> addr;      // solo stores
    ap_uint<32> sdata;     // solo stores
    ap_uint<3>  mem_f3;    // solo stores
};

struct Operand {
    bool        ready;
    ap_uint<32> val;
    ap_uint<3>  tag; // tag ROB esperado cuando !ready
};

struct AluRs {
    bool        busy, executing;
    ap_uint<1>  remaining;
    ap_uint<3>  f3;
    bool        alt;      // SUB/SRA(I)
    ap_uint<3>  rob_tag;
    Operand     s1, s2;
};

struct MdRs {
    bool        busy, executing;
    ap_uint<4>  remaining;
    ap_uint<3>  f3;
    ap_uint<3>  rob_tag;
    Operand     s1, s2;
};

struct LsuRs {
    bool        busy;
    bool        is_load;
    ap_uint<3>  f3;
    ap_uint<3>  rob_tag;
    int32_t     imm;
    Operand     s1;  // base de direccion
    Operand     s2;  // dato del store (loads: ready=true)
};

struct BrRs {
    bool        busy, executing;
    ap_uint<1>  remaining;
    bool        is_jalr;
    ap_uint<3>  f3;
    ap_uint<3>  rob_tag;
    ap_uint<32> br_pc;
    ap_uint<3>  size;   // 2 o 4 (extension C): link y fall-through = pc+size
    int32_t     imm;
    Operand     s1, s2;
};

struct FpuRs {
    bool        busy, executing;
    ap_uint<4>  remaining;
    ap_uint<3>  r4op;   // 0=OP_FP; 1..4 = FMADD/FMSUB/FNMSUB/FNMADD
    ap_uint<7>  f7;
    ap_uint<3>  f3;     // selector dentro del grupo (sgnj/minmax/cmp/mv-class)
    ap_uint<5>  rs2f;   // campo rs2 crudo (selector W/WU de FCVT)
    ap_uint<3>  rob_tag;
    Operand     s1, s2, s3; // s3: tercer operando de la familia R4
};

// Unidad de sistema (bare-metal, items 2-5): una sola reservation
// station para las instrucciones CSR, que ejecuta SOLO en la cabeza del
// ROB (como un load) -- asi el acceso a CSR es en orden de programa y ve
// el estado arquitectonico correcto, sin necesitar el mecanismo de traps
// precisas (item 6, fuera de alcance). ECALL/EBREAK no pasan por aca:
// se marcan en el ROB (is_ecall) y detienen el programa al retirarse.
struct SysRs {
    bool        busy;
    ap_uint<3>  f3;        // CSRRW/S/C y variantes con inmediato
    ap_uint<12> csr_addr;  // instr[31:20]
    ap_uint<3>  rob_tag;
    Operand     s1;        // valor de rs1 (CSRRW/S/C) o zimm (variantes I, ready=true)
};

// Unidad vectorial (RVV): una sola reservation station, como la LSU --
// serializa las instrucciones vectoriales ENTRE SI (no hay VRAT que las
// renombre), pero pueden solaparse libremente con instrucciones
// escalares independientes que completen fuera de orden alrededor
// suyo -- eso es lo que hace esto "coprocesamiento con un core OOO" y
// no solo "otro decoder mas". Ver rv32_ooo.h para el detalle completo.
struct VecRs {
    bool        busy, executing;
    ap_uint<2>  remaining;
    bool        is_load, is_store, is_arith;
    ap_uint<5>  vd_or_vs3;
    ap_uint<5>  vs1, vs2;   // solo aritmetica (vs1/vs2 en los mismos bits que rs1/rs2)
    ap_uint<6>  funct6;     // operacion
    ap_uint<3>  funct3;     // familia y forma (.vv/.vx/.vi)
    bool        vm;         // 1 = sin mascara; 0 = predicado por v0
    ap_uint<3>  rob_tag;
    Operand     s1;         // direccion base (rs1, banco entero) -- solo memoria
    Operand     s2;         // paso en bytes (rs2, banco entero) -- solo strided
    // ---- Fase 5: modo de direccionamiento del acceso a memoria ----
    ap_uint<2>  mop;      // 00 unit-stride, 01/11 indexado, 10 strided
    ap_uint<3>  nf;       // campos por segmento MENOS UNO (nf=0 -> 1 campo)
    ap_uint<3>  fld;      // campo que toca procesar en este ciclo (0..nf)
    ap_uint<2>  lsmode;   // LSM_*: normal / mascara / registro completo / fof
    ap_uint<3>  idx_b;    // ancho del INDICE en bytes (solo indexado)
    // `vl` vigente CAPTURADO EN EL DISPATCH: como el dispatch es en orden
    // de programa y este core no especula (toda instruccion despachada
    // committea), capturar aca el largo vectorial es preciso y ademas
    // permite que la unidad VEC siga solapandose con el core escalar.
    ap_uint<5>  vl;      // hasta 16 elementos (SEW=8)
    ap_uint<3>  sew_b;   // ancho de elemento en BYTES (1, 2 o 4)
    ap_uint<4>  vcat;      // categoria (VCAT_*): que destino escribe
    ap_uint<5>  vstart;    // primer elemento a ejecutar (CSR vstart, seccion 3.7)
    ap_uint<5>  vs1_field; // campo vs1 crudo (selector de los grupos unary)
};

static ap_uint<32> regfile[32];
static ap_uint<32> fregs[32];   // banco F como bits IEEE-754 crudos
static RatEntry    rat[32];
static RatEntry    frat[32];    // RAT del banco flotante (mismo ROB)
static RobEntry    rob[ROB_SZ];
static ap_uint<3>  rob_head, rob_tail;
static ap_uint<4>  rob_count;
static AluRs       alu_rs[N_ALU];
static MdRs        md_rs;
static FpuRs       fpu_rs;
static LsuRs       lsu_rs;
static BrRs        br_rs;
static VecRs       vec_rs;
static SysRs       sys_rs;
static ap_uint<32> vregs[OOO_VEC_REGFILE_LEN]; // banco vectorial, sin renombrar
// Banco de CSRs de modo maquina (subset minimo para bare-metal): cada
// uno es solo almacenamiento leible/escribible, sin logica WARL. Alcanza
// para que el crt0 de un binario compilado configure mtvec/stack y llegue
// a main. mhartid es read-only (0 = unico hart), misa reporta RV32IMFC.
static ap_uint<32> csr_mstatus, csr_mie, csr_mtvec, csr_mscratch;
static ap_uint<32> csr_mepc, csr_mcause, csr_mtval, csr_mip;
// CSRs vectoriales (RVV): vtype y vl, escritos por vsetvli/vsetivli/vsetvl
// y leibles con csrr (direcciones 0xC21/0xC20 de la spec).
static ap_uint<32> csr_vtype, csr_vl;
static ap_uint<32> csr_vstart;   // Fase 6: primer elemento a ejecutar (seccion 3.7)
static bool        ecall_halt; // se activa al retirar un ECALL/EBREAK
// --- soporte de INTERRUPCIONES (timer) y MODOS DE PRIVILEGIO ---
static ap_uint<32> csr_mtime;    // contador libre, avanza un tick por ciclo
static ap_uint<32> csr_mtimecmp; // umbral: mtime >= mtimecmp -> interrupcion
static ap_uint<2>  cur_priv;     // 3 = M-mode, 0 = U-mode
static ap_uint<32> fetch_pc;
static bool        fetch_stalled; // esperando resolucion de BRANCH/JALR
static bool        fetch_done;    // se encontro la palabra 0 (fin de programa)


#endif // OOO_STATE_H
