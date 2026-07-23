#ifndef RVV_ENCODING_H
#define RVV_ENCODING_H

// =====================================================================
// Codificacion de la extension vectorial RVV 1.0: opcodes, funct3,
// funct6, campos de vtype y modos de direccionamiento de memoria.
//
// TODO lo de este archivo esta verificado contra la especificacion
// oficial; cada bloque cita su seccion. No hay logica: solo constantes.
// Estan a nivel de archivo (no dentro de la clase) para que las use
// tambien vector_alu.h sin arrastrar el procesador entero.
//
// OJO: VEC_VLEN_BITS y VEC_MAX_ELEMS NO estan aca -- se derivan de
// VEC_LANES, que es una propiedad de ESTA implementacion (su VLEN), no
// de la codificacion del ISA. Siguen dentro de la clase.
// =====================================================================
#include <cstdint>

static const uint32_t RVV_OPCODE_OP_V = 0b1010111;
static const uint32_t RVV_WIDTH_32    = 0b110; // distingue vle32.v/vse32.v de FLW/FSW (width=010)
// funct3 = familia/forma del operando (seccion 5 de la spec)
static const uint32_t RVV_FUNCT3_OPIVV = 0b000; // vector-vector, enteras
static const uint32_t RVV_FUNCT3_OPMVV = 0b010; // vector-vector, "mask/mul"
static const uint32_t RVV_FUNCT3_OPIVI = 0b011; // vector-inmediato (5 bits con signo)
static const uint32_t RVV_FUNCT3_OPIVX = 0b100; // vector-escalar (x[rs1])
static const uint32_t RVV_FUNCT3_OPMVX = 0b110; // vector-escalar, familia M
static const uint32_t RVV_FUNCT3_OPCFG = 0b111; // vsetvli / vsetivli / vsetvl

// funct6 -- valores tomados de la tabla "Vector Instruction Listing"
// (seccion 19 de la spec RVV v1.0), no de memoria.
// Familia OPIVV / OPIVX / OPIVI:
static const uint32_t RVV_F6_VADD  = 0b000000;
static const uint32_t RVV_F6_VSUB  = 0b000010;
static const uint32_t RVV_F6_VRSUB = 0b000011; // solo .vx/.vi: vd = escalar - vs2
static const uint32_t RVV_F6_VMINU = 0b000100;
static const uint32_t RVV_F6_VMIN  = 0b000101;
static const uint32_t RVV_F6_VMAXU = 0b000110;
static const uint32_t RVV_F6_VMAX  = 0b000111;
static const uint32_t RVV_F6_VAND  = 0b001001;
static const uint32_t RVV_F6_VOR   = 0b001010;
static const uint32_t RVV_F6_VXOR  = 0b001011;
static const uint32_t RVV_F6_VSLL  = 0b100101;
static const uint32_t RVV_F6_VSRL  = 0b101000;
static const uint32_t RVV_F6_VSRA  = 0b101001;
// Familia OPMVV / OPMVX (multiplicacion y division):
static const uint32_t RVV_F6_VDIVU  = 0b100000;
static const uint32_t RVV_F6_VDIV   = 0b100001;
static const uint32_t RVV_F6_VREMU  = 0b100010;
static const uint32_t RVV_F6_VREM   = 0b100011;
static const uint32_t RVV_F6_VMULHU = 0b100100;
static const uint32_t RVV_F6_VMUL   = 0b100101;
static const uint32_t RVV_F6_VMULHSU= 0b100110;
static const uint32_t RVV_F6_VMULH  = 0b100111;
// --- Fase 3: comparaciones (OPIV*) que ESCRIBEN UNA MASCARA ---
static const uint32_t RVV_F6_VMSEQ  = 0b011000;
static const uint32_t RVV_F6_VMSNE  = 0b011001;
static const uint32_t RVV_F6_VMSLTU = 0b011010;
static const uint32_t RVV_F6_VMSLT  = 0b011011;
static const uint32_t RVV_F6_VMSLEU = 0b011100;
static const uint32_t RVV_F6_VMSLE  = 0b011101;
static const uint32_t RVV_F6_VMSGTU = 0b011110; // solo .vx/.vi
static const uint32_t RVV_F6_VMSGT  = 0b011111; // solo .vx/.vi
// --- vmerge (vm=0) / vmv.v.* (vm=1), mismo funct6 en OPIV* ---
static const uint32_t RVV_F6_VMERGE = 0b010111;
// --- logica de mascaras (OPMVV). Mismo rango de funct6 que las
//     comparaciones, pero distinto funct3: no hay ambiguedad. ---
static const uint32_t RVV_F6_VMANDNOT = 0b011000;
static const uint32_t RVV_F6_VMAND    = 0b011001;
static const uint32_t RVV_F6_VMOR     = 0b011010;
static const uint32_t RVV_F6_VMXOR    = 0b011011;
static const uint32_t RVV_F6_VMORNOT  = 0b011100;
static const uint32_t RVV_F6_VMNAND   = 0b011101;
static const uint32_t RVV_F6_VMNOR    = 0b011110;
static const uint32_t RVV_F6_VMXNOR   = 0b011111;
// --- grupos "unary" codificados en el campo vs1 (tablas 21 y 27) ---
// --- Fase 4a: reducciones (OPMVV). El acumulador inicial es vs1[0] y
//     el resultado va a vd[0]; solo participan los elementos activos. ---
static const uint32_t RVV_F6_VREDSUM  = 0b000000;
static const uint32_t RVV_F6_VREDAND  = 0b000001;
static const uint32_t RVV_F6_VREDOR   = 0b000010;
static const uint32_t RVV_F6_VREDXOR  = 0b000011;
static const uint32_t RVV_F6_VREDMINU = 0b000100;
static const uint32_t RVV_F6_VREDMIN  = 0b000101;
static const uint32_t RVV_F6_VREDMAXU = 0b000110;
static const uint32_t RVV_F6_VREDMAX  = 0b000111;
static const uint32_t RVV_F6_VWXUNARY0 = 0b010000; // vcpop.m / vfirst.m
static const uint32_t RVV_F6_VMUNARY0  = 0b010100; // vmsbf/vmsof/vmsif/viota/vid
static const uint32_t RVV_VS1_VCPOP  = 0b10000;
static const uint32_t RVV_VS1_VFIRST = 0b10001;
static const uint32_t RVV_VS1_VMSBF  = 0b00001;
static const uint32_t RVV_VS1_VMSOF  = 0b00010;
static const uint32_t RVV_VS1_VMSIF  = 0b00011;
static const uint32_t RVV_VS1_VIOTA  = 0b10000;
static const uint32_t RVV_VS1_VID    = 0b10001;
// categorias de operacion vectorial (que tipo de destino escriben)
static const uint8_t VCAT_ALU   = 0; // elementos normales
static const uint8_t VCAT_CMP   = 1; // bits de mascara
static const uint8_t VCAT_MERGE = 2; // elementos, seleccionados por v0
static const uint8_t VCAT_MLOG  = 3; // logica bit a bit entre mascaras
static const uint8_t VCAT_VID   = 4; // vid.v / viota.m -> elementos
static const uint8_t VCAT_XRES  = 5; // vcpop.m / vfirst.m -> registro entero
static const uint8_t VCAT_RED   = 6; // reduccion -> escribe solo vd[0]
static const uint8_t VCAT_PERM  = 7; // permutacion (slide/gather/compress)
static const uint8_t VCAT_WIDE  = 8; // widening: destino 2*SEW en un PAR de registros
static const uint8_t VCAT_NARROW= 9; // narrowing: fuente vs2 de 2*SEW, destino SEW

// --- Fase 4d: widening (OPMVV/OPMVX) --- tabla de la seccion 19.
// Las formas ".w" (110100..110111) toman el PRIMER operando ya ancho.
static const uint32_t RVV_F6_VWADDU   = 0b110000;
static const uint32_t RVV_F6_VWADD    = 0b110001;
static const uint32_t RVV_F6_VWSUBU   = 0b110010;
static const uint32_t RVV_F6_VWSUB    = 0b110011;
static const uint32_t RVV_F6_VWADDU_W = 0b110100;
static const uint32_t RVV_F6_VWADD_W  = 0b110101;
static const uint32_t RVV_F6_VWSUBU_W = 0b110110;
static const uint32_t RVV_F6_VWSUB_W  = 0b110111;
static const uint32_t RVV_F6_VWMULU   = 0b111000;
static const uint32_t RVV_F6_VWMULSU  = 0b111010;
static const uint32_t RVV_F6_VWMUL    = 0b111011;
static const uint32_t RVV_F6_VWMACCU  = 0b111100;
static const uint32_t RVV_F6_VWMACC   = 0b111101;
static const uint32_t RVV_F6_VWMACCUS = 0b111110; // solo .vx
static const uint32_t RVV_F6_VWMACCSU = 0b111111;
// --- Fase 4d: narrowing (OPIVV/OPIVX/OPIVI) ---
static const uint32_t RVV_F6_VNSRL    = 0b101100;
static const uint32_t RVV_F6_VNSRA    = 0b101101;
static const uint32_t RVV_F6_VNCLIPU  = 0b101110;
static const uint32_t RVV_F6_VNCLIP   = 0b101111;
// --- Fase 4b: permutaciones ---
// OPIVX/OPIVI: vslideup(001110) / vslidedown(001111) / vrgather(001100)
// OPMVX:       vslide1up(001110) / vslide1down(001111)
// OPMVV:       vcompress(010111)
static const uint32_t RVV_F6_VRGATHER   = 0b001100;
static const uint32_t RVV_F6_VSLIDEUP   = 0b001110;
static const uint32_t RVV_F6_VSLIDEDOWN = 0b001111;
static const uint32_t RVV_F6_VCOMPRESS  = 0b010111;
// --- Fase 4c: punto fijo. Saturantes (OPIV*) y promediados (OPMV*) ---
static const uint32_t RVV_F6_VSADDU = 0b100000;
static const uint32_t RVV_F6_VSADD  = 0b100001;
static const uint32_t RVV_F6_VSSUBU = 0b100010;
static const uint32_t RVV_F6_VSSUB  = 0b100011;
static const uint32_t RVV_F6_VAADDU = 0b001000;
static const uint32_t RVV_F6_VAADD  = 0b001001;
static const uint32_t RVV_F6_VASUBU = 0b001010;
static const uint32_t RVV_F6_VASUB  = 0b001011;
// Campos de vtype (spec 3.4): vlmul[2:0], vsew[5:3], vta[6], vma[7],
// vill[XLEN-1]. Solo se soporta SEW=32 / LMUL=1 -> VLMAX = 128/32 = 4.
static const uint32_t RVV_VSEW_8   = 0b000;
static const uint32_t RVV_VSEW_16  = 0b001;
static const uint32_t RVV_VSEW_32  = 0b010;
static const uint32_t RVV_VLMUL_1  = 0b000;
static const uint32_t RVV_VILL_BIT = 31;
// width de los load/store vectoriales (seccion 7): 000=8b, 101=16b,
// 110=32b. Distinguen ademas de FLW/FSW escalar (width=010).
static const uint32_t RVV_WIDTH_8  = 0b000;
static const uint32_t RVV_WIDTH_16 = 0b101;
static const uint32_t RVV_WIDTH_64 = 0b111; // EEW=64: FUERA de Zve32x -> ilegal

// ---- Fase 5: modos de direccionamiento de memoria (seccion 7.2) ----
// mop[1:0] = instr[27:26]. Tablas 9 (loads) y 10 (stores) de la spec.
static const uint32_t RVV_MOP_UNIT      = 0b00; // unit-stride  vle<EEW>.v
static const uint32_t RVV_MOP_IDX_UNORD = 0b01; // indexado sin orden
static const uint32_t RVV_MOP_STRIDED   = 0b10; // paso constante  vlse<EEW>.v
static const uint32_t RVV_MOP_IDX_ORD   = 0b11; // indexado ordenado
// lumop/sumop = instr[24:20]. Tablas 11 y 12: variantes del unit-stride.
static const uint32_t RVV_LUMOP_UNIT  = 0b00000; // vle<EEW>.v / vse<EEW>.v
static const uint32_t RVV_LUMOP_WHOLE = 0b01000; // vl<nf>r.v / vs<nf>r.v
static const uint32_t RVV_LUMOP_MASK  = 0b01011; // vlm.v / vsm.v  (EEW=8)
static const uint32_t RVV_LUMOP_FOF   = 0b10000; // vle<EEW>ff.v  (solo load)
// Modo interno de la unidad de memoria vectorial (que camino toma).
static const uint8_t LSM_NORMAL = 0; // unit-stride / strided / indexado
static const uint8_t LSM_MASK   = 1; // vlm.v / vsm.v
static const uint8_t LSM_WHOLE  = 2; // registro completo
static const uint8_t LSM_FOF    = 3; // fault-only-first

#endif // RVV_ENCODING_H
