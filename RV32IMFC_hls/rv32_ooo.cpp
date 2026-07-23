#include "rv32_ooo.h"
#include "rv32i_defs.h"
#include "immediates_hls.h"
#include "fp_ops.h"
#include "rv32c_defs.h"
#include <cmath>

using namespace rv32_hls; // get_imm_I/S/B/U/J, get_shamt

// ---- latencias de las unidades funcionales (en ticks) ----
static const int ALU_LAT   = 1;
static const int MUL_LAT   = 3;
static const int DIV_LAT   = 8;
static const int BR_LAT    = 1;
static const int FPU_LAT_ADDMUL = 3; // FADD/FSUB/FMUL
static const int FPU_LAT_DIV    = 8; // FDIV/FSQRT
static const int FPU_LAT_FMA    = 4; // familia FMADD (R4)
static const int FPU_LAT_MISC   = 2; // sgnj/minmax/cmp/cvt/mv/class

static const int ROB_SZ  = 8;
static const int N_ALU   = 2;
static const int VEC_LAT = 3; // vadd.vv/vsub.vv/vmul.vv

// ---- RVV: opcodes/campos verificados contra la especificacion oficial
// (RISC-V "V" Vector Extension v1.0), mismos valores que rv32_vector.cpp.
// No se agregan a rv32i_defs.h a proposito (ese header es compartido con
// rv32_core.cpp/rv32_vector.cpp, no hace falta tocarlo). ----
namespace rvv {
    constexpr uint32_t OPCODE_OP_V = 0b1010111;
    constexpr uint32_t WIDTH_32    = 0b110; // distingue vle32.v/vse32.v de FLW/FSW (width=010)
    constexpr uint32_t FUNCT3_OPIVV = 0b000;
    constexpr uint32_t FUNCT3_OPMVV = 0b010;
    constexpr uint32_t FUNCT3_OPCFG = 0b111; // vsetvli / vsetivli / vsetvl
    constexpr uint32_t FUNCT6_VADD  = 0b000000;
    constexpr uint32_t FUNCT6_VSUB  = 0b000010;
    constexpr uint32_t FUNCT6_VMUL  = 0b100101;

    // Campos de vtype (seccion 3.4 de la spec): vlmul[2:0], vsew[5:3],
    // vta[6], vma[7], vill[XLEN-1]. Este core solo soporta la
    // configuracion SEW=32 / LMUL=1 -> VLMAX = VLEN/SEW = 128/32 = 4.
    constexpr uint32_t VSEW_32   = 0b010; // vsew: 000=8,001=16,010=32,011=64
    constexpr uint32_t VLMUL_1   = 0b000; // vlmul: 000=1, 001=2, ... 111=1/2
    constexpr uint32_t VILL_BIT  = 31;    // XLEN-1 en RV32
    constexpr uint32_t VLMAX     = OOO_VEC_LANES; // 4 elementos
}

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
    bool        is_ecall;   // ECALL/EBREAK: al retirarse detiene el programa
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
    ap_uint<2>  arith_op;   // 0=vadd.vv, 1=vsub.vv, 2=vmul.vv
    ap_uint<3>  rob_tag;
    Operand     s1;         // direccion base (rs1, banco entero) -- solo memoria
    // `vl` vigente CAPTURADO EN EL DISPATCH: como el dispatch es en orden
    // de programa y este core no especula (toda instruccion despachada
    // committea), capturar aca el largo vectorial es preciso y ademas
    // permite que la unidad VEC siga solapandose con el core escalar.
    ap_uint<3>  vl;
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
static bool        ecall_halt; // se activa al retirar un ECALL/EBREAK
static ap_uint<32> fetch_pc;
static bool        fetch_stalled; // esperando resolucion de BRANCH/JALR
static bool        fetch_done;    // se encontro la palabra 0 (fin de programa)

// ================= helpers combinacionales =================

// Despierta a toda RS que esperara `tag` -- el rol del CDB. Se llama una
// vez por cada unidad que completa en el tick (bus de resultado por
// unidad, sin arbitraje de un CDB unico -- simplificacion documentada en
// rv32_ooo.h).
static void cdb_broadcast(ap_uint<3> tag, ap_uint<32> value) {
    for (int i = 0; i < N_ALU; i++) {
#pragma HLS UNROLL
        if (alu_rs[i].busy && !alu_rs[i].s1.ready && alu_rs[i].s1.tag == tag) {
            alu_rs[i].s1.ready = true; alu_rs[i].s1.val = value;
        }
        if (alu_rs[i].busy && !alu_rs[i].s2.ready && alu_rs[i].s2.tag == tag) {
            alu_rs[i].s2.ready = true; alu_rs[i].s2.val = value;
        }
    }
    if (md_rs.busy && !md_rs.s1.ready && md_rs.s1.tag == tag) {
        md_rs.s1.ready = true; md_rs.s1.val = value;
    }
    if (md_rs.busy && !md_rs.s2.ready && md_rs.s2.tag == tag) {
        md_rs.s2.ready = true; md_rs.s2.val = value;
    }
    if (lsu_rs.busy && !lsu_rs.s1.ready && lsu_rs.s1.tag == tag) {
        lsu_rs.s1.ready = true; lsu_rs.s1.val = value;
    }
    if (lsu_rs.busy && !lsu_rs.s2.ready && lsu_rs.s2.tag == tag) {
        lsu_rs.s2.ready = true; lsu_rs.s2.val = value;
    }
    if (br_rs.busy && !br_rs.s1.ready && br_rs.s1.tag == tag) {
        br_rs.s1.ready = true; br_rs.s1.val = value;
    }
    if (br_rs.busy && !br_rs.s2.ready && br_rs.s2.tag == tag) {
        br_rs.s2.ready = true; br_rs.s2.val = value;
    }
    if (fpu_rs.busy && !fpu_rs.s1.ready && fpu_rs.s1.tag == tag) {
        fpu_rs.s1.ready = true; fpu_rs.s1.val = value;
    }
    if (fpu_rs.busy && !fpu_rs.s2.ready && fpu_rs.s2.tag == tag) {
        fpu_rs.s2.ready = true; fpu_rs.s2.val = value;
    }
    if (fpu_rs.busy && !fpu_rs.s3.ready && fpu_rs.s3.tag == tag) {
        fpu_rs.s3.ready = true; fpu_rs.s3.val = value;
    }
    if (vec_rs.busy && !vec_rs.s1.ready && vec_rs.s1.tag == tag) {
        vec_rs.s1.ready = true; vec_rs.s1.val = value;
    }
    if (sys_rs.busy && !sys_rs.s1.ready && sys_rs.s1.tag == tag) {
        sys_rs.s1.ready = true; sys_rs.s1.val = value;
    }
}

// Lectura/escritura del banco de CSRs (subset minimo de modo maquina).
// misa/mhartid son de solo lectura (valores fijos); el resto es
// almacenamiento simple. Una direccion no soportada lee 0 / ignora la
// escritura (comportamiento aceptable para un core bare-metal minimo).
static ap_uint<32> read_csr(ap_uint<12> addr) {
    switch (addr.to_uint()) {
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
        // CSRs vectoriales de solo lectura (los escribe vsetvl*, no csrw)
        case rv32i::CSR::VL:       return csr_vl;
        case rv32i::CSR::VTYPE:    return csr_vtype;
        case rv32i::CSR::VLENB:    return OOO_VEC_LANES * 4; // VLEN/8 = 16 bytes
        default:                   return 0;
    }
}

static void write_csr(ap_uint<12> addr, ap_uint<32> v) {
    switch (addr.to_uint()) {
        case rv32i::CSR::MSTATUS:  csr_mstatus = v; break;
        case rv32i::CSR::MIE:      csr_mie = v; break;
        case rv32i::CSR::MTVEC:    csr_mtvec = v; break;
        case rv32i::CSR::MSCRATCH: csr_mscratch = v; break;
        case rv32i::CSR::MEPC:     csr_mepc = v; break;
        case rv32i::CSR::MCAUSE:   csr_mcause = v; break;
        case rv32i::CSR::MTVAL:    csr_mtval = v; break;
        case rv32i::CSR::MIP:      csr_mip = v; break;
        default:                   break; // misa/mhartid read-only, resto ignorado
    }
}

// Lee un operando fuente en el dispatch: regfile directo si el RAT no
// tiene tag pendiente; bypass desde el ROB si el productor ya calculo;
// si no, queda esperando ese tag (lo despertara cdb_broadcast). x0
// siempre es 0 y nunca se renombra.
static Operand read_operand(ap_uint<5> reg) {
    Operand op;
    if (reg == 0) {
        op.ready = true; op.val = 0; op.tag = 0;
        return op;
    }
    if (!rat[reg].has_tag) {
        op.ready = true; op.val = regfile[reg]; op.tag = 0;
        return op;
    }
    ap_uint<3> t = rat[reg].tag;
    if (rob[t].ready) {
        op.ready = true; op.val = rob[t].value; op.tag = 0;
    } else {
        op.ready = false; op.val = 0; op.tag = t;
    }
    return op;
}

// Version F de read_operand: usa FRAT/fregs. A diferencia de x0, f0 es
// un registro normal (se renombra y escribe como cualquier otro). Los
// tags son del MISMO ROB, asi que el wakeup por CDB es identico.
static Operand read_operand_fp(ap_uint<5> reg) {
    Operand op;
    if (!frat[reg].has_tag) {
        op.ready = true; op.val = fregs[reg]; op.tag = 0;
        return op;
    }
    ap_uint<3> t = frat[reg].tag;
    if (rob[t].ready) {
        op.ready = true; op.val = rob[t].value; op.tag = 0;
    } else {
        op.ready = false; op.val = 0; op.tag = t;
    }
    return op;
}

// ALU entera RV32I -- identica semantica al bloque OP/OP_IMM de
// rv32_core.cpp, factorizada como funcion pura (fase 1 del plan:
// separar decode de execute).
static ap_uint<32> alu_compute(ap_uint<3> f3, bool alt, ap_uint<32> a_u, ap_uint<32> b_u) {
    int32_t a = static_cast<int32_t>(a_u.to_uint());
    int32_t b = static_cast<int32_t>(b_u.to_uint());
    ap_uint<5> sh = b_u.range(4, 0);
    switch (f3.to_uint()) {
        case rv32i::Funct3_ALU::ADD_SUB: return alt ? ap_uint<32>(a - b) : ap_uint<32>(a + b);
        case rv32i::Funct3_ALU::SLL:     return a_u << sh;
        case rv32i::Funct3_ALU::SLT:     return (a < b) ? 1 : 0;
        case rv32i::Funct3_ALU::SLTU:    return (a_u < b_u) ? 1 : 0;
        case rv32i::Funct3_ALU::XOR:     return a_u ^ b_u;
        case rv32i::Funct3_ALU::SRL_SRA: return alt ? ap_uint<32>(a >> sh.to_uint()) : ap_uint<32>(a_u >> sh);
        case rv32i::Funct3_ALU::OR:      return a_u | b_u;
        case rv32i::Funct3_ALU::AND:     return a_u & b_u;
        default:                         return 0;
    }
}

// Extension M completa, con los mismos casos borde del ISA que
// rv32_core.cpp (division por cero, overflow INT32_MIN/-1).
static ap_uint<32> md_compute(ap_uint<3> f3, ap_uint<32> a_u, ap_uint<32> b_u) {
    uint32_t ua = a_u.to_uint(), ub = b_u.to_uint();
    int32_t  sa = static_cast<int32_t>(ua), sb = static_cast<int32_t>(ub);
    int64_t  a64 = sa, b64 = sb;
    uint64_t ua64 = ua, ub64 = ub;
    switch (f3.to_uint()) {
        case rv32i::Funct3_MULDIV::MUL:
            return ua * ub;
        case rv32i::Funct3_MULDIV::MULH:
            return static_cast<uint32_t>(static_cast<uint64_t>(a64 * b64) >> 32);
        case rv32i::Funct3_MULDIV::MULHSU:
            return static_cast<uint32_t>(static_cast<uint64_t>(a64 * static_cast<int64_t>(ub64)) >> 32);
        case rv32i::Funct3_MULDIV::MULHU:
            return static_cast<uint32_t>((ua64 * ub64) >> 32);
        case rv32i::Funct3_MULDIV::DIV:
            if (sb == 0)                          return 0xFFFFFFFF;
            if (ua == 0x80000000u && sb == -1)    return 0x80000000;
            return static_cast<uint32_t>(sa / sb);
        case rv32i::Funct3_MULDIV::DIVU:
            return (ub == 0) ? 0xFFFFFFFF : (ua / ub);
        case rv32i::Funct3_MULDIV::REM:
            if (sb == 0)                          return ua;
            if (ua == 0x80000000u && sb == -1)    return 0;
            return static_cast<uint32_t>(sa % sb);
        case rv32i::Funct3_MULDIV::REMU:
            return (ub == 0) ? ua : (ua % ub);
        default:
            return 0;
    }
}

// Extension F completa -- misma semantica que el bloque OP_FP/R4 de
// rv32_core.cpp (fp_ops.h para reinterpretaciones/saturacion/FCLASS),
// factorizada como funcion pura de la reservation station de la FPU.
// Los operandos entran/salen como bits IEEE-754 crudos.
static ap_uint<32> fpu_compute(const FpuRs& u) {
    float a = rv32i::bits_to_float(u.s1.val.to_uint());
    float b = rv32i::bits_to_float(u.s2.val.to_uint());
    float c = rv32i::bits_to_float(u.s3.val.to_uint());
    switch (u.r4op.to_uint()) {
        case 1: return rv32i::float_to_bits(std::fma(a, b, c));   // FMADD
        case 2: return rv32i::float_to_bits(std::fma(a, b, -c));  // FMSUB
        case 3: return rv32i::float_to_bits(std::fma(-a, b, c));  // FNMSUB
        case 4: return rv32i::float_to_bits(std::fma(-a, b, -c)); // FNMADD
    }
    switch (u.f7.to_uint()) {
        case rv32i::Funct7_FP::FADD_S:  return rv32i::float_to_bits(a + b);
        case rv32i::Funct7_FP::FSUB_S:  return rv32i::float_to_bits(a - b);
        case rv32i::Funct7_FP::FMUL_S:  return rv32i::float_to_bits(a * b);
        case rv32i::Funct7_FP::FDIV_S:  return rv32i::float_to_bits(a / b);
        case rv32i::Funct7_FP::FSQRT_S: return rv32i::float_to_bits(std::sqrt(a));
        case rv32i::Funct7_FP::FSGNJ_S: {
            uint32_t ab = u.s1.val.to_uint();
            uint32_t bb = u.s2.val.to_uint();
            uint32_t sign = 0;
            switch (u.f3.to_uint()) {
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
            switch (u.f3.to_uint()) {
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
            // s1 viene del banco ENTERO: es un entero crudo, no bits float
            return (u.rs2f == rv32i::Rs2_FCVT::WU)
                       ? rv32i::float_to_bits(static_cast<float>(u.s1.val.to_uint()))
                       : rv32i::float_to_bits(static_cast<float>(static_cast<int32_t>(u.s1.val.to_uint())));
        case rv32i::Funct7_FP::FMV_X_W_FCLASS_S:
            return (u.f3 == rv32i::Funct3_FMV_FCLASS::FCLASS_S)
                       ? ap_uint<32>(rv32i::fclass_s(a))
                       : u.s1.val; // FMV.X.W: bits crudos, sin conversion
        case rv32i::Funct7_FP::FMV_W_X:
            return u.s1.val;       // FMV.W.X: bits crudos desde el banco entero
        default:
            return 0;
    }
}

// Aritmetica RVV vector-vector -- misma semantica que rv32_vector.cpp,
// pero limitada a los primeros `vl` elementos (el largo vectorial
// dinamico que fijo el ultimo vsetvli). Los elementos con indice >= vl
// son el "tail" y se dejan SIN TOCAR (politica tail-undisturbed, vta=0):
// la spec permite explicitamente que una implementacion simple ignore
// vta y use siempre undisturbed (nota de la seccion 3.4.3).
static void vec_arith_compute(const VecRs& u, ap_uint<32> vregs[OOO_VEC_REGFILE_LEN]) {
    for (int lane = 0; lane < OOO_VEC_LANES; lane++) {
#pragma HLS UNROLL
        if (lane < u.vl.to_uint()) {
            ap_uint<32> a = vregs[u.vs2.to_uint() * OOO_VEC_LANES + lane];
            ap_uint<32> b = vregs[u.vs1.to_uint() * OOO_VEC_LANES + lane];
            ap_uint<32> r;
            if (u.arith_op == 0)      r = a + b; // vadd.vv
            else if (u.arith_op == 1) r = a - b; // vsub.vv
            else                      r = a * b; // vmul.vv
            vregs[u.vd_or_vs3.to_uint() * OOO_VEC_LANES + lane] = r;
        }
    }
}

static bool branch_taken(ap_uint<3> f3, ap_uint<32> a_u, ap_uint<32> b_u) {
    int32_t a = static_cast<int32_t>(a_u.to_uint());
    int32_t b = static_cast<int32_t>(b_u.to_uint());
    switch (f3.to_uint()) {
        case rv32i::Funct3_BRANCH::BEQ:  return a_u == b_u;
        case rv32i::Funct3_BRANCH::BNE:  return a_u != b_u;
        case rv32i::Funct3_BRANCH::BLT:  return a < b;
        case rv32i::Funct3_BRANCH::BGE:  return a >= b;
        case rv32i::Funct3_BRANCH::BLTU: return a_u < b_u;
        case rv32i::Funct3_BRANCH::BGEU: return a_u >= b_u;
        default:                         return false;
    }
}

// Acceso a memoria de datos direccionada por palabra -- misma tecnica
// (y misma limitacion de no cruzar palabra) que mem_load/mem_store de
// rv32_core.cpp, sobre el arreglo dmem local de esta pista.
static ap_uint<32> dmem_load(ap_uint<32> dmem[OOO_DMEM_WORDS], ap_uint<32> addr, ap_uint<3> f3) {
    ap_uint<32> word = dmem[(addr >> 2) & (OOO_DMEM_WORDS - 1)];
    ap_uint<5> shift = ap_uint<5>(addr.range(1, 0)) * 8;
    ap_uint<32> sh = word >> shift;
    switch (f3.to_uint()) {
        case rv32i::Funct3_LOAD::LB:
            return static_cast<uint32_t>(static_cast<int32_t>(static_cast<int8_t>(ap_uint<8>(sh.range(7, 0)).to_uint())));
        case rv32i::Funct3_LOAD::LH:
            return static_cast<uint32_t>(static_cast<int32_t>(static_cast<int16_t>(ap_uint<16>(sh.range(15, 0)).to_uint())));
        case rv32i::Funct3_LOAD::LW:  return word;
        case rv32i::Funct3_LOAD::LBU: return ap_uint<8>(sh.range(7, 0)).to_uint();
        case rv32i::Funct3_LOAD::LHU: return ap_uint<16>(sh.range(15, 0)).to_uint();
        default:                      return 0;
    }
}

static void dmem_store(ap_uint<32> dmem[OOO_DMEM_WORDS], ap_uint<32> addr, ap_uint<32> wdata, ap_uint<3> f3) {
    ap_uint<32> wa = (addr >> 2) & (OOO_DMEM_WORDS - 1);
    ap_uint<5> shift = ap_uint<5>(addr.range(1, 0)) * 8;
    switch (f3.to_uint()) {
        case rv32i::Funct3_STORE::SW:
            dmem[wa] = wdata;
            break;
        case rv32i::Funct3_STORE::SH: {
            ap_uint<32> word = dmem[wa];
            ap_uint<32> mask = ap_uint<32>(0xFFFF) << shift;
            dmem[wa] = (word & ~mask) | ((wdata & ap_uint<32>(0xFFFF)) << shift);
            break;
        }
        case rv32i::Funct3_STORE::SB: {
            ap_uint<32> word = dmem[wa];
            ap_uint<32> mask = ap_uint<32>(0xFF) << shift;
            dmem[wa] = (word & ~mask) | ((wdata & ap_uint<32>(0xFF)) << shift);
            break;
        }
    }
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
        csr_vl = 0;
        ecall_halt = false;
        for (int i = 0; i < OOO_VEC_REGFILE_LEN; i++) {
            vregs[i] = 0;
        }
        fetch_pc = 0; fetch_stalled = false; fetch_done = false;
        for (int i = 0; i < OOO_VEC_REGFILE_LEN; i++) {
            vregs_out[i] = vregs[i];
        }
        return;
    }

    // ---- Etapa 1: commit (retiro en orden desde la cabeza del ROB) ----
    if (rob_count > 0 && rob[rob_head].valid && rob[rob_head].ready) {
        RobEntry& h = rob[rob_head];
        if (h.is_ecall) {
            // ECALL/EBREAK: al llegar a la cabeza del ROB, todo lo anterior
            // ya committeo (retiro en orden) -- detener el programa es una
            // "excepcion precisa" gratis, sin squash: lo posterior en el
            // ROB simplemente se descarta al no seguir committeando.
            ecall_halt = true;
        } else if (h.is_store) {
            // el store escribe memoria recien al retirarse: garantiza el
            // orden de programa en memoria sin necesitar disambiguation
            dmem_store(dmem, h.addr, h.sdata, h.mem_f3);
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
            vec_arith_compute(vec_rs, vregs);
            rob[vec_rs.rob_tag].ready = true;
            vec_done = 1; vec_tag = vec_rs.rob_tag;
            vec_rs.busy = false;
            vec_rs.executing = false;
        }
    }
    if (vec_rs.busy && (vec_rs.is_load || vec_rs.is_store) &&
        vec_rs.s1.ready && vec_rs.rob_tag == rob_head && rob[rob_head].valid) {
        ap_uint<32> word_addr = vec_rs.s1.val >> 2;
        // Solo se mueven los primeros `vl` elementos: un vle32.v deja el
        // tail del registro sin tocar, y un vse32.v no escribe memoria mas
        // alla de vl (la spec es explicita en que el store NO debe tocar
        // memoria fuera del body).
        if (vec_rs.is_load) {
            for (int lane = 0; lane < OOO_VEC_LANES; lane++) {
#pragma HLS UNROLL
                if (lane < vec_rs.vl.to_uint())
                    vregs[vec_rs.vd_or_vs3.to_uint() * OOO_VEC_LANES + lane] =
                        dmem[(word_addr + lane) & (OOO_DMEM_WORDS - 1)];
            }
        } else {
            for (int lane = 0; lane < OOO_VEC_LANES; lane++) {
#pragma HLS UNROLL
                if (lane < vec_rs.vl.to_uint())
                    dmem[(word_addr + lane) & (OOO_DMEM_WORDS - 1)] =
                        vregs[vec_rs.vd_or_vs3.to_uint() * OOO_VEC_LANES + lane];
            }
        }
        rob[vec_rs.rob_tag].ready = true;
        vec_done = 1; vec_tag = vec_rs.rob_tag;
        vec_rs.busy = false;
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
    if (vec_rs.busy && vec_rs.is_arith && !vec_rs.executing) {
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
            uint32_t iw  = instr.to_uint();
            uint32_t opc = rv32i::opcode(iw);
            ap_uint<5> rd  = rv32i::rd(iw);
            ap_uint<3> f3  = rv32i::funct3(iw);
            ap_uint<5> rs1 = rv32i::rs1(iw);
            ap_uint<5> rs2 = rv32i::rs2(iw);
            uint32_t   f7  = rv32i::funct7(iw);

            bool can_dispatch = false;
            ap_uint<3> new_tag = rob_tail;
            rob[new_tag].dest_is_fp = false; // default entero; los casos F lo activan
            rob[new_tag].is_ecall = false;   // solo ECALL/EBREAK lo activan
            ap_uint<32> this_pc = fetch_pc;           // pc de ESTA instruccion
            ap_uint<32> next_fetch = this_pc + isize; // se pisa si JAL redirige

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
                       f3 == rvv::WIDTH_32) {
                // vle32.v / vse32.v: MISMO opcode LOAD-FP/STORE-FP que
                // FLW/FSW escalar, pero width=110 (32b vectorial) en vez
                // de width=010 (FLW/FSW) -- asi se distinguen (ver
                // rv32_ooo.h y rv32_vector.cpp para el detalle de bits,
                // verificado contra la especificacion oficial RVV v1.0).
                if (!vec_rs.busy) {
                    can_dispatch = true;
                    vec_rs.busy = true;
                    vec_rs.is_load = (opc == rv32i::Opcode::LOAD_FP);
                    vec_rs.is_store = !vec_rs.is_load;
                    vec_rs.is_arith = false;
                    vec_rs.vd_or_vs3 = rd; // mismo campo de bits que vd (load) o vs3 (store)
                    vec_rs.rob_tag = new_tag;
                    vec_rs.vl = csr_vl;    // largo vectorial vigente en este punto del programa
                    vec_rs.s1 = read_operand(rs1); // direccion base, banco entero
                    RobEntry& e = rob[new_tag];
                    e.valid = true; e.is_store = false; e.dest_is_fp = false;
                    e.dest = 0; e.ready = false;
                }
            } else if (opc == rv32i::Opcode::LOAD || opc == rv32i::Opcode::STORE ||
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
                    if (br_rs.is_jalr) {
                        br_rs.imm = get_imm_I(instr);
                        br_rs.s2.ready = true; br_rs.s2.val = 0; br_rs.s2.tag = 0;
                    } else {
                        br_rs.imm = get_imm_B(instr);
                        br_rs.s2 = read_operand(rs2);
                    }
                    RobEntry& e = rob[new_tag];
                    e.valid = true; e.is_store = false; e.ready = false;
                    e.dest = br_rs.is_jalr ? rd : ap_uint<5>(0);
                    // sin especulacion: el fetch queda detenido hasta que
                    // la unidad de branch resuelva el destino real
                    fetch_stalled = true;
                }
            } else if (opc == rvv::OPCODE_OP_V && f3 == rvv::FUNCT3_OPCFG) {
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

                if (!operands_ready) {
                    can_dispatch = false; // stall: el AVL todavia no existe
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
                bool supported = (vsew == rvv::VSEW_32) && (vlmul == rvv::VLMUL_1);

                ap_uint<32> new_vl;
                if (!supported) {
                    csr_vtype = (ap_uint<32>(1) << rvv::VILL_BIT); // vill, resto en cero
                    new_vl = 0;
                } else {
                    csr_vtype = vtype_req;
                    if (keep_vl)          new_vl = csr_vl;                 // vtype cambia, vl se conserva
                    else if (avl_is_max)  new_vl = rvv::VLMAX;
                    else                  new_vl = (avl < rvv::VLMAX) ? ap_uint<32>(avl)
                                                                      : ap_uint<32>(rvv::VLMAX);
                }
                csr_vl = new_vl;

                // vsetvl* escribe en rd el vl efectivamente concedido
                RobEntry& e = rob[new_tag];
                e.valid = true; e.is_store = false; e.dest_is_fp = false;
                e.dest = rd; e.value = new_vl; e.ready = true;
                } // fin de operands_ready
            } else if (opc == rvv::OPCODE_OP_V) {
                // vadd.vv/vsub.vv/vmul.vv -- funct6/funct3 verificados
                // contra la seccion 19 de la especificacion oficial RVV
                // v1.0 (misma tabla que rv32_vector.cpp).
                uint32_t f6 = (iw >> 26) & 0x3F;
                bool is_add = (f3 == rvv::FUNCT3_OPIVV && f6 == rvv::FUNCT6_VADD);
                bool is_sub = (f3 == rvv::FUNCT3_OPIVV && f6 == rvv::FUNCT6_VSUB);
                bool is_mul = (f3 == rvv::FUNCT3_OPMVV && f6 == rvv::FUNCT6_VMUL);
                if (is_add || is_sub || is_mul) {
                    if (!vec_rs.busy) {
                        can_dispatch = true;
                        vec_rs.busy = true;
                        vec_rs.is_load = false; vec_rs.is_store = false; vec_rs.is_arith = true;
                        vec_rs.vd_or_vs3 = rd;
                        vec_rs.vs1 = rs1; vec_rs.vs2 = rs2; // mismos bits que rs1/rs2
                        vec_rs.arith_op = is_add ? ap_uint<2>(0) : is_sub ? ap_uint<2>(1) : ap_uint<2>(2);
                        vec_rs.rob_tag = new_tag;
                        vec_rs.vl = csr_vl;       // largo vectorial vigente al despachar
                        vec_rs.executing = false; // arranca en la etapa 3, como el resto de las unidades
                        RobEntry& e = rob[new_tag];
                        e.valid = true; e.is_store = false; e.dest_is_fp = false;
                        e.dest = 0; e.ready = false;
                    }
                    // si vec_rs.busy, can_dispatch queda false: stall correcto (unidad ocupada)
                } else {
                    // variante RVV no soportada en este alcance minimo: no-op
                    can_dispatch = true;
                    RobEntry& e = rob[new_tag];
                    e.valid = true; e.is_store = false; e.dest = 0; e.value = 0; e.ready = true;
                }
            } else if (opc == rv32i::Opcode::SYSTEM) {
                // bare-metal (items 2-5): ECALL/EBREAK (funct3=0) e
                // instrucciones CSR (funct3=1..7).
                if (f3 == rv32i::Funct3_SYSTEM::PRIV) {
                    uint32_t imm12 = (iw >> 20) & 0xFFF;
                    if (imm12 == 0x000 || imm12 == 0x001) {
                        // ECALL / EBREAK: se marca en el ROB y detiene el
                        // programa al retirarse (excepcion precisa gratis,
                        // ver Etapa 1). MRET (0x302) y otros privilegiados
                        // caen en el no-op de abajo (traps = item 6, futuro).
                        can_dispatch = true;
                        RobEntry& e = rob[new_tag];
                        e.valid = true; e.is_store = false; e.is_ecall = true;
                        e.dest = 0; e.value = 0; e.ready = true;
                    } else {
                        // MRET/WFI/FENCE-privilegiado: no-op (sin traps)
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
                // opcode realmente no soportado: retira como no-op para no
                // trabar el pipeline
                can_dispatch = true;
                RobEntry& e = rob[new_tag];
                e.valid = true; e.is_store = false; e.dest = 0; e.value = 0; e.ready = true;
            }

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
