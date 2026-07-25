# RISC-V SoC — pista TLM (SystemC)

Espejo **ciclo a ciclo** de `RISCV-SoC/` (la pista HLS): el mismo
diagrama del SoC — core escalar OoO con **TAGE**, **coprocesador
vectorial** con Vector Instruction Queue y unidades VALU/VMUL/VLSU/VSLDU,
e **interconnect con árbitro** (una concesión de memoria por ciclo) —
modelado en SystemC/TLM-2.0. Deriva de `RV32IMFC+RVV+OOO-TLM/`.

## El mapa: bloque del diagrama → archivo

| Bloque del diagrama | Archivo | Notas |
|---|---|---|
| Frontend: Fetch/Decode | `processor_dispatch.h` | fetch + tabla de decodificación RV32IMFC+RVV |
| Frontend: **TAGE** | `tage.h` | mismos parámetros que HLS: bimodal 512×2b + 3 tablas {8b tag, 3b ctr, 2b u}, historias 4/8/16, GHR 16b |
| Backend: Issue/Rename + ALU/MUL/FPU/LSU | `processor_ooo.h` (estado, CDB) + `processor_tick.h` (ejecución) | RS distribuidas estilo Tomasulo |
| **Vector instruction Queue** | `processor_ooo.h` (FIFO de 4) | el CDB también despierta operandos en cola |
| Unidades VALU/VMUL/VLSU/VSLDU | `processor_tick.h` + `processor_vector_unit.h` + `vector_alu.h` | latencias VALU 2 / VMUL 4 / VSLDU 2; VLSU un elemento/ciclo |
| **AXI4 Interconnect + Árbitro** | `bus.h` (ruteo por dirección) + `axi_grant()` en `processor_ooo.h` | **ventaja TLM**: el Bus ya es un interconnect real con mapa de direcciones y periférico UART como target |
| Memoria del FPGA | `memory.h` (target TLM) | unificada, cargada con los ELF reales |

## Ventaja propia de esta pista

En TLM el interconnect **no es un modelo dentro del core**: es un módulo
`Bus` de verdad con sockets TLM-2.0, decodificación de direcciones y dos
targets (Memoria y UART). El árbitro (`axi_grant()`, una concesión por
ciclo, prioridad commit > VLSU > LSU) gobierna cuándo el procesador emite
la transacción — la estructura del SoC del diagrama existe como jerarquía
de módulos SystemC.

## Verificación

**219 checks, 0 fallos** (las 8 suites). Paridad medida con la pista HLS
en la suite E: **241 ciclos, 64 branches, 6 mispredicts (90.6 %)** —
números **idénticos** en ambas pistas. La paridad ciclo a ciclo del
proyecto sobrevivió a TAGE, a la VIQ y al árbitro.

```bash
g++ -std=c++17 -I. -o soc src/main.cpp -lsystemc && ./soc
```

## Limitaciones

Las mismas de `RISCV-SoC/` (ver su README): JALR sin especular, issue
queue como RS distribuidas, coprocesador no especulativo, GHR no reparado
en flush por trap, y las heredadas de la base (`LMUL≠1`, sin FP
vectorial/SEW=64, `vxrm` fijo, sin S-mode/MMU).
