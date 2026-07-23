#ifndef RVV_ENCODING_H
#define RVV_ENCODING_H

// Codificacion de la extension vectorial RVV 1.0: opcodes, funct3, funct6,
// modos de direccionamiento y campos de vtype. TODO lo de este archivo esta
// verificado contra la especificacion oficial (las secciones estan citadas
// en cada bloque). No hay logica: solo constantes.
#include "rv32_ooo.h"
#include "rv32i_defs.h"

namespace rvv {
    constexpr uint32_t OPCODE_OP_V = 0b1010111;
    constexpr uint32_t WIDTH_32    = 0b110; // distingue vle32.v/vse32.v de FLW/FSW (width=010)
    // funct3 = familia y FORMA del segundo operando (seccion 5)
    constexpr uint32_t FUNCT3_OPIVV = 0b000; // vector-vector, enteras
    constexpr uint32_t FUNCT3_OPMVV = 0b010; // vector-vector, mul/div
    constexpr uint32_t FUNCT3_OPIVI = 0b011; // vector-inmediato (5 bits con signo)
    constexpr uint32_t FUNCT3_OPIVX = 0b100; // vector-escalar (x[rs1])
    constexpr uint32_t FUNCT3_OPMVX = 0b110; // vector-escalar, familia mul/div
    constexpr uint32_t FUNCT3_OPCFG = 0b111; // vsetvli / vsetivli / vsetvl
    // funct6 -- tabla "Vector Instruction Listing" (seccion 19)
    constexpr uint32_t F6_VADD   = 0b000000;
    constexpr uint32_t F6_VSUB   = 0b000010;
    constexpr uint32_t F6_VRSUB  = 0b000011; // solo .vx/.vi
    constexpr uint32_t F6_VMINU  = 0b000100;
    constexpr uint32_t F6_VMIN   = 0b000101;
    constexpr uint32_t F6_VMAXU  = 0b000110;
    constexpr uint32_t F6_VMAX   = 0b000111;
    constexpr uint32_t F6_VAND   = 0b001001;
    constexpr uint32_t F6_VOR    = 0b001010;
    constexpr uint32_t F6_VXOR   = 0b001011;
    constexpr uint32_t F6_VSLL   = 0b100101;
    constexpr uint32_t F6_VSRL   = 0b101000;
    constexpr uint32_t F6_VSRA   = 0b101001;
    constexpr uint32_t F6_VDIVU  = 0b100000;
    constexpr uint32_t F6_VDIV   = 0b100001;
    constexpr uint32_t F6_VREMU  = 0b100010;
    constexpr uint32_t F6_VREM   = 0b100011;
    constexpr uint32_t F6_VMULHU = 0b100100;
    constexpr uint32_t F6_VMUL   = 0b100101;
    constexpr uint32_t F6_VMULHSU= 0b100110;
    constexpr uint32_t F6_VMULH  = 0b100111;
    // --- Fase 3: comparaciones (OPIV*) que ESCRIBEN UNA MASCARA ---
    constexpr uint32_t F6_VMSEQ  = 0b011000;
    constexpr uint32_t F6_VMSNE  = 0b011001;
    constexpr uint32_t F6_VMSLTU = 0b011010;
    constexpr uint32_t F6_VMSLT  = 0b011011;
    constexpr uint32_t F6_VMSLEU = 0b011100;
    constexpr uint32_t F6_VMSLE  = 0b011101;
    constexpr uint32_t F6_VMSGTU = 0b011110; // solo .vx/.vi
    constexpr uint32_t F6_VMSGT  = 0b011111; // solo .vx/.vi
    constexpr uint32_t F6_VMERGE = 0b010111; // vmerge (vm=0) / vmv.v.* (vm=1)
    // --- logica de mascaras (OPMVV): mismo rango de funct6, distinto funct3 ---
    constexpr uint32_t F6_VMANDNOT = 0b011000;
    constexpr uint32_t F6_VMAND    = 0b011001;
    constexpr uint32_t F6_VMOR     = 0b011010;
    constexpr uint32_t F6_VMXOR    = 0b011011;
    constexpr uint32_t F6_VMORNOT  = 0b011100;
    constexpr uint32_t F6_VMNAND   = 0b011101;
    constexpr uint32_t F6_VMNOR    = 0b011110;
    constexpr uint32_t F6_VMXNOR   = 0b011111;
    // --- grupos "unary" codificados en el campo vs1 (tablas 21 y 27) ---
    // --- Fase 4a: reducciones (OPMVV): acumulador inicial vs1[0] -> vd[0] ---
    constexpr uint32_t F6_VREDSUM  = 0b000000;
    constexpr uint32_t F6_VREDAND  = 0b000001;
    constexpr uint32_t F6_VREDOR   = 0b000010;
    constexpr uint32_t F6_VREDXOR  = 0b000011;
    constexpr uint32_t F6_VREDMINU = 0b000100;
    constexpr uint32_t F6_VREDMIN  = 0b000101;
    constexpr uint32_t F6_VREDMAXU = 0b000110;
    constexpr uint32_t F6_VREDMAX  = 0b000111;
    // --- Fase 4b: permutaciones ---
    constexpr uint32_t F6_VRGATHER   = 0b001100;
    constexpr uint32_t F6_VSLIDEUP   = 0b001110; // .vx/.vi; en OPMVX = vslide1up
    constexpr uint32_t F6_VSLIDEDOWN = 0b001111; // .vx/.vi; en OPMVX = vslide1down
    constexpr uint32_t F6_VCOMPRESS  = 0b010111;
    // --- Fase 4c: punto fijo saturante (OPIV*) y promediado (OPMV*) ---
    constexpr uint32_t F6_VSADDU = 0b100000;
    constexpr uint32_t F6_VSADD  = 0b100001;
    constexpr uint32_t F6_VSSUBU = 0b100010;
    constexpr uint32_t F6_VSSUB  = 0b100011;
    constexpr uint32_t F6_VAADDU = 0b001000;
    constexpr uint32_t F6_VAADD  = 0b001001;
    constexpr uint32_t F6_VASUBU = 0b001010;
    constexpr uint32_t F6_VASUB  = 0b001011;
    constexpr uint32_t F6_VWXUNARY0 = 0b010000; // vcpop.m / vfirst.m
    constexpr uint32_t F6_VMUNARY0  = 0b010100; // viota.m / vid.v
    constexpr uint32_t VS1_VCPOP  = 0b10000;
    constexpr uint32_t VS1_VFIRST = 0b10001;
    constexpr uint32_t VS1_VIOTA  = 0b10000;
    constexpr uint32_t VS1_VID    = 0b10001;
    // categorias de operacion vectorial (que destino escriben)
    constexpr uint8_t VCAT_ALU   = 0; // elementos normales
    constexpr uint8_t VCAT_CMP   = 1; // bits de mascara
    constexpr uint8_t VCAT_MERGE = 2; // elementos, seleccionados por v0
    constexpr uint8_t VCAT_MLOG  = 3; // logica bit a bit entre mascaras
    constexpr uint8_t VCAT_VID   = 4; // vid.v / viota.m
    constexpr uint8_t VCAT_XRES  = 5; // vcpop.m / vfirst.m -> registro entero
    constexpr uint8_t VCAT_RED   = 6; // reduccion -> escribe solo vd[0]
    constexpr uint8_t VCAT_PERM  = 7; // permutacion (slide/gather/compress)
    constexpr uint8_t VCAT_WIDE  = 8; // widening: destino 2*SEW en un PAR de registros
    constexpr uint8_t VCAT_NARROW= 9; // narrowing: fuente vs2 de 2*SEW, destino SEW

    // --- Fase 4d: widening (OPMVV/OPMVX) --- tabla de la seccion 19.
    // Las formas ".w" (110100..110111) toman el PRIMER operando ya ancho.
    constexpr uint32_t F6_VWADDU   = 0b110000;
    constexpr uint32_t F6_VWADD    = 0b110001;
    constexpr uint32_t F6_VWSUBU   = 0b110010;
    constexpr uint32_t F6_VWSUB    = 0b110011;
    constexpr uint32_t F6_VWADDU_W = 0b110100;
    constexpr uint32_t F6_VWADD_W  = 0b110101;
    constexpr uint32_t F6_VWSUBU_W = 0b110110;
    constexpr uint32_t F6_VWSUB_W  = 0b110111;
    constexpr uint32_t F6_VWMULU   = 0b111000;
    constexpr uint32_t F6_VWMULSU  = 0b111010;
    constexpr uint32_t F6_VWMUL    = 0b111011;
    constexpr uint32_t F6_VWMACCU  = 0b111100;
    constexpr uint32_t F6_VWMACC   = 0b111101;
    constexpr uint32_t F6_VWMACCUS = 0b111110; // solo .vx
    constexpr uint32_t F6_VWMACCSU = 0b111111;
    // --- Fase 4d: narrowing (OPIVV/OPIVX/OPIVI) ---
    constexpr uint32_t F6_VNSRL    = 0b101100;
    constexpr uint32_t F6_VNSRA    = 0b101101;
    constexpr uint32_t F6_VNCLIPU  = 0b101110;
    constexpr uint32_t F6_VNCLIP   = 0b101111;

    // Campos de vtype (seccion 3.4 de la spec): vlmul[2:0], vsew[5:3],
    // vta[6], vma[7], vill[XLEN-1]. Este core solo soporta la
    // configuracion SEW=32 / LMUL=1 -> VLMAX = VLEN/SEW = 128/32 = 4.
    constexpr uint32_t VSEW_8    = 0b000;
    constexpr uint32_t VSEW_16   = 0b001;
    constexpr uint32_t VSEW_32   = 0b010; // vsew: 000=8,001=16,010=32,011=64
    constexpr uint32_t VLMUL_1   = 0b000; // vlmul: 000=1, 001=2, ... 111=1/2
    constexpr uint32_t VILL_BIT  = 31;    // XLEN-1 en RV32
    // VLEN = OOO_VEC_LANES*32 = 128 bits. Con EEW variable el numero de
    // elementos depende de SEW (128/8=16, 128/16=8, 128/32=4). El
    // ALMACENAMIENTO no cambia (4 palabras por registro): cambia como se
    // indexan los elementos. Como 8/16/32 dividen exacto a 32, ningun
    // elemento cruza el limite de palabra.
    constexpr int VLEN_BITS  = OOO_VEC_LANES * 32;   // 128
    constexpr int MAX_ELEMS  = VLEN_BITS / 8;        // 16 (peor caso, SEW=8)
    // width de los load/store vectoriales (seccion 7)
    constexpr uint32_t WIDTH_8   = 0b000;
    constexpr uint32_t WIDTH_16  = 0b101;
    constexpr uint32_t WIDTH_64  = 0b111; // EEW=64: FUERA de Zve32x -> ilegal

    // ---- Fase 5: modos de direccionamiento de memoria (seccion 7.2) ----
    // mop[1:0] = instr[27:26]. Tablas 9 (loads) y 10 (stores) de la spec.
    constexpr uint32_t MOP_UNIT      = 0b00; // unit-stride  vle<EEW>.v
    constexpr uint32_t MOP_IDX_UNORD = 0b01; // indexado sin orden  vluxei<EEW>.v
    constexpr uint32_t MOP_STRIDED   = 0b10; // paso constante  vlse<EEW>.v
    constexpr uint32_t MOP_IDX_ORD   = 0b11; // indexado ordenado  vloxei<EEW>.v
    // lumop/sumop = instr[24:20]. Tablas 11 y 12: variantes del unit-stride.
    constexpr uint32_t LUMOP_UNIT  = 0b00000; // vle<EEW>.v / vse<EEW>.v
    constexpr uint32_t LUMOP_WHOLE = 0b01000; // vl<nf>r.v / vs<nf>r.v
    constexpr uint32_t LUMOP_MASK  = 0b01011; // vlm.v / vsm.v  (EEW=8)
    constexpr uint32_t LUMOP_FOF   = 0b10000; // vle<EEW>ff.v  (solo load)
    // Modo interno de la unidad de memoria vectorial (que camino toma).
    constexpr uint8_t LSM_NORMAL = 0; // unit-stride / strided / indexado
    constexpr uint8_t LSM_MASK   = 1; // vlm.v / vsm.v
    constexpr uint8_t LSM_WHOLE  = 2; // registro completo
    constexpr uint8_t LSM_FOF    = 3; // fault-only-first
}


#endif // RVV_ENCODING_H
