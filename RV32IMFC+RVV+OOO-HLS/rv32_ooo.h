#ifndef RV32_OOO_H
#define RV32_OOO_H

#include <ap_int.h>

// Core RV32IMFC con ejecucion fuera de orden (Tomasulo-lite), sintetizable
// con Vitis HLS. Evolucion del mecanismo probado en ooo_demo.cpp,
// generalizado a un procesador real:
//
//   - Fetch real de 16 bits (extension C): la unidad de fetch lee el
//     halfword en pc (cualquier direccion PAR), expande comprimidas con
//     rv32c::expand() y avanza pc en 2 o 4. A diferencia del core escalar
//     rv32_core.cpp (que exige la comprimida en la mitad baja de una
//     palabra alineada), esta pista soporta cualquier alineacion de 2
//     bytes, igual que el modelo TLM.
//   - 32 registros enteros + RAT, y 32 registros flotantes + FRAT
//     (extension F). Ambos RAT apuntan al MISMO ROB unificado de 8
//     entradas: una dependencia float->int (p.ej. FEQ que escribe x-reg)
//     o int->float (FCVT.S.W que lee x-reg) usa exactamente el mismo
//     mecanismo de tags que una dependencia entera.
//   - ROB de 8 entradas, retiro estrictamente en orden.
//   - Unidades funcionales con reservation station propia:
//       2x ALU     (lat. 1)              -- OP/OP_IMM enteras
//       1x MUL/DIV (MUL lat. 3, DIV 8)   -- extension M completa
//       1x FPU     (ADD/SUB/MUL lat. 3, DIV/SQRT 8, FMADD* 4, resto 2)
//                  -- extension F completa, incluida la familia R4
//                  (FMADD/FMSUB/FNMSUB/FNMADD) con TERCER operando rs3
//       1x LSU     (LW/LH/LB/SW/SH/SB + FLW/FSW, memoria en orden)
//       1x BR      (BRANCH/JALR; JAL/LUI/AUIPC resuelven en dispatch)
//       1x VEC     (RVV minimo: vle32.v/vse32.v + vadd.vv/vsub.vv/vmul.vv,
//                  ver detalle abajo) -- coprocesamiento vectorial real,
//                  integrado al mismo mecanismo OOO
//   - CDB: al completar, cada unidad difunde (tag, valor) y despierta a
//     cualquier RS que esperara ese tag (los valores F viajan como bits
//     IEEE-754 crudos, igual que en el bus real).
//
// Coprocesamiento vectorial (RVV, integrado con rv32_vector.cpp):
//   Mismas 5 instrucciones y misma codificacion de bits verificada
//   contra la especificacion oficial que en rv32_vector.cpp (memoria
//   unit-stride + aritmetica vector-vector), pero ahora despachadas
//   desde el MISMO decoder OOO que el resto del programa -- un
//   `vadd.vv` puede estar ejecutando en la unidad VEC mientras
//   instrucciones escalares independientes (ALU/MUL/FPU) completan
//   fuera de orden alrededor suyo, que es la definicion real de
//   coprocesamiento con un core OOO. Simplificaciones adicionales
//   (documentadas en LIMITACIONES.md): el banco vectorial NO se
//   renombra via el ROB (no hay VRAT) -- una sola reservation station
//   VEC (como la LSU) serializa las instrucciones vectoriales ENTRE SI
//   en orden de programa (misma logica que "memoria en orden"), aunque
//   pueden solaparse libremente con instrucciones escalares
//   independientes. `vle32.v`/`vse32.v` se resuelven recien en la
//   cabeza del ROB (igual que un load escalar, pero tambien para el
//   store -- mas conservador que el store escalar, para no tener que
//   ampliar el ROB a 128 bits). La distincion con FLW/FSW escalar (que
//   comparte el mismo opcode LOAD-FP/STORE-FP) es el campo `width`:
//   0b010 = escalar F, 0b110 = vectorial unit-stride de 32 bits.
//
// Decisiones de alcance (deliberadas, para acotar verificacion):
//   - SIN especulacion de saltos (fetch se detiene hasta resolver
//     BRANCH/JALR; JAL redirige en el dispatch). Sin squash/recovery.
//   - Memoria EN ORDEN: un acceso en vuelo; un load ejecuta solo en la
//     cabeza del ROB; un store escribe memoria recien al commit.
//   - Sin CSRs de F (frm/fflags): rm=RNE fijo, igual que el resto del
//     proyecto.
//   - RVV: vsetvli/vsetivli/vsetvl implementadas con vl DINAMICO
//     (vl = min(AVL, VLMAX)) y politica tail-undisturbed; unica
//     configuracion soportada SEW=32/LMUL=1 con VLEN=128 -> VLMAX=4
//     (otra configuracion activa vill y deja vl=0). Sin mascara, y solo
//     las 5 instrucciones de datos de rv32_vector.cpp. Al reset
//     vtype.vill=1 y vl=0, asi que un programa RVV real debe empezar
//     con vsetvli, igual que en hardware de verdad.
//   - Convencion de fin de programa: halfword 0x0000 detiene el fetch;
//     `halted` cuando ademas el ROB esta vacio.
//
// Un tick = un ciclo de reloj. El estado persiste en variables static.
// Salidas: eventos de dispatch/completion/commit para que el testbench
// reconstruya el estado arquitectonico SOLO desde el retiro (sin
// backdoor) y verifique el reordenamiento desde afuera del DUT.
// `commit_is_fp` distingue si el retiro escribe el banco entero o el
// flotante (el valor flotante se reporta como bits IEEE-754). El banco
// vectorial se expone como puerto de salida `vregs_out` (no pasa por el
// stream de commit, ver nota de alcance arriba).

// Memoria de instrucciones y datos (en palabras de 32 bits). Se
// dimensionaron para que quepan los programas de prueba completos
// (I+M+F+C+RVV con EEW variable) y un binario bare-metal chico.
// 16384 palabras = 64KB cada una. Dimensionadas para que quepa un
// binario con la biblioteca C completa (el printf de newlib solo ya son
// ~54KB de codigo).
#define OOO_IMEM_WORDS 16384
#define OOO_DMEM_WORDS 16384
#define OOO_VEC_NUM_VREGS 32
#define OOO_VEC_LANES 4
#define OOO_VEC_REGFILE_LEN (OOO_VEC_NUM_VREGS * OOO_VEC_LANES)

void rv32_ooo_tick(
    ap_uint<1>  reset,
    ap_uint<32> imem[OOO_IMEM_WORDS],
    ap_uint<32> dmem[OOO_DMEM_WORDS],
    // evento de dispatch (para que el TB mapee tag ROB -> instruccion)
    ap_uint<1>&  disp_valid,
    ap_uint<3>&  disp_tag,
    ap_uint<32>& disp_pc,
    // eventos de completion, uno por unidad funcional
    ap_uint<1>& alu0_done, ap_uint<3>& alu0_tag,
    ap_uint<1>& alu1_done, ap_uint<3>& alu1_tag,
    ap_uint<1>& md_done,   ap_uint<3>& md_tag,
    ap_uint<1>& fpu_done,  ap_uint<3>& fpu_tag,
    ap_uint<1>& lsu_done,  ap_uint<3>& lsu_tag,
    ap_uint<1>& br_done,   ap_uint<3>& br_tag,
    ap_uint<1>& vec_done,  ap_uint<3>& vec_tag,
    // stream de commit (retiro en orden)
    ap_uint<1>&  commit_valid,
    ap_uint<1>&  commit_is_fp, // 1: escribe f[rd]; 0: escribe x[rd] (rd=0 => nada)
    ap_uint<5>&  commit_rd,
    ap_uint<32>& commit_value,
    // banco vectorial (estado completo, actualizado cada ciclo)
    ap_uint<32>  vregs_out[OOO_VEC_REGFILE_LEN],
    ap_uint<1>&  halted
);

#endif // RV32_OOO_H
