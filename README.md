# RV32IMFC-RVV-out-of-order

Base de un proyecto de maestría hacia un core **RV32IMFC + RVV
out-of-order**. El repo tiene dos versiones **independientes y
autocontenidas** (cada una copia lo que necesita de la otra, sin rutas
relativas cruzadas):

- **[`RV32IMFC_tlm/`](RV32IMFC_tlm/)**: modelo de verificación funcional
  en SystemC/TLM-2.0 — el que se explica en detalle más abajo y en
  `explain.md`.
- **[`RV32IMFC_hls/`](RV32IMFC_hls/)**: prototipo sintetizable (Vitis
  HLS) de RV32IMFC con maestro AXI4 Full, para obtener métricas reales de
  hardware (Kria KV260).

> Cycle-accurate-ish RV32IMFC RISC-V core in SystemC/TLM-2.0 (Loosely-Timed),
> with a shared Bus, integer/M/F/C extensions verified end-to-end, and an
> early RVV (Vector) skeleton — plus a separate, synthesizable RV32IMFC
> HLS prototype targeting a Kria KV260.

## Estado actual

- ✅ **RV32I** completo: R-type, I-type, Load/Store (1/2/4 bytes, LE,
  desalineado), Branch, JAL/JALR, LUI/AUIPC.
- ✅ **Extensión M**: MUL, MULH, MULHSU, MULHU, DIV, DIVU, REM, REMU —
  incluye los casos borde de división por cero y overflow (`INT32_MIN /
  -1`) según el manual RISC-V.
- ✅ **Extensión F** (simple precisión): FLW/FSW, FADD.S/FSUB.S/FMUL.S/
  FDIV.S/FSQRT.S, FSGNJ.S/FSGNJN.S/FSGNJX.S, FMIN.S/FMAX.S, FEQ.S/FLT.S/
  FLE.S, FCVT.W.S/FCVT.WU.S/FCVT.S.W/FCVT.S.WU, FMV.X.W/FMV.W.X,
  FCLASS.S, y la familia FMADD.S/FMSUB.S/FNMSUB.S/FNMADD.S.
- ✅ **Extensión C** (comprimidas): decoder completo (expansión a
  instrucciones de 32 bits equivalentes) — C.ADDI4SPN, C.LW/C.SW/C.FLW/
  C.FSW, C.ADDI, C.JAL, C.LI, C.ADDI16SP, C.LUI, C.SRLI/C.SRAI/C.ANDI,
  C.SUB/C.XOR/C.OR/C.AND, C.J, C.BEQZ/C.BNEZ, C.SLLI, C.LWSP/C.SWSP,
  C.JR/C.MV/C.JALR/C.ADD. El fetch ahora es de 16 bits (no 32), con
  `pc` avanzando en 2 o 4 según corresponda. **RV32IMFC completo** en la
  versión TLM.
- ✅ **Ejecución fuera de orden (`ProcessorOOO`)**: Tomasulo-lite (RAT+FRAT
  → ROB unificado de 8, 2×ALU, MUL/DIV, FPU con familia FMADD, LSU,
  branch sin especulación, **y unidad vectorial RVV**), como `SC_MODULE`
  alternativo que corre sobre la misma topología TLM (`Bus`/`Memory`).
  Verificado con el mismo programa RV32IMFC+RVV que la pista HLS,
  evidencia de reordenamiento y de coprocesamiento incluida (ver sección
  abajo). No reemplaza a `Processor` (in-order) — son dos módulos
  separados, mismo socket TLM.
- ✅ **Coprocesamiento vectorial RVV (integrado al OOO)**: la unidad
  `VecRs` de `ProcessorOOO` decodifica y ejecuta 5 instrucciones RVV
  reales (`vle32.v`/`vse32.v` + `vadd.vv`/`vsub.vv`/`vmul.vv`,
  codificación verificada contra la especificación oficial RVV v1.0)
  como una reservation station más del pipeline OOO — no es un módulo
  separado. Misma pieza en la pista HLS (`rv32_ooo.cpp`), mismos ciclos.
- 🚧 **`VectorUnit`** (esqueleto standalone, previo a la integración):
  initiator independiente conectado al Bus, banco de 32 registros
  vectoriales y test end-to-end — reemplazado en el pipeline por la
  unidad `VecRs` de arriba, se conserva por compatibilidad del testbench
  in-order. `vtype`/`vsetvli`, máscara y el resto del ISA vectorial
  siguen pendientes.
- ⬜ CSRs, ECALL/EBREAK, loader de ELF, periféricos — no empezado (ver
  `explain.md` para el detalle de qué falta para correr binarios
  bare-metal reales).

Todo lo anterior está compilado, ejecutado y verificado (a mano, contra
Python) — no es solo código teórico.

## Arquitectura

```
Processor (Initiator) ──┐
                         ├──> Bus (Target + Initiator) ──> Memory (Target)
VectorUnit (Initiator) ──┘
```

- **`Processor`**: CPU RV32IMF monociclo, `simple_initiator_socket`,
  comunicación 100% vía `b_transport` bloqueante (estilo Loosely-Timed).
- **`Bus`**: decodifica direcciones globales y rutea hacia el
  periférico correcto; expone un `target_socket` por cada iniciador del
  sistema (CPU, VectorUnit).
- **`Memory`**: 64KB, recibe direcciones locales, soporta accesos de
  1/2/4 bytes en Little Endian, incluyendo desalineados.
- **`VectorUnit`**: esqueleto de RVV — banco de registros vectoriales +
  socket propio hacia el Bus.

Sin `sc_signal` en ningún módulo (pureza TLM-2.0).

## Síntesis a FPGA (Kria KV260)

`RV32IMFC_tlm/` es un modelo de **verificación funcional**, no es
sintetizable tal cual (usa sockets, `tlm_generic_payload`, I/O de C++,
etc.). Para obtener métricas reales de hardware (utilización de
recursos, frecuencia máxima) está `RV32IMFC_hls/`: el decode+execute de
RV32IMFC reescrito para Vitis HLS, con maestro AXI4 Full, apuntando al
Kria KV260. Ver [`RV32IMFC_hls/README.md`](RV32IMFC_hls/README.md) para
cómo correrlo y qué números extraer del reporte de síntesis (incluye la
limitación de alineación propia de esa pista para instrucciones
comprimidas — distinta de la limitación de acceso a memoria que ya
tenía).

**Nota de independencia**: las dos carpetas son autocontenidas — no hay
`#include`s cruzados entre ellas. `RV32IMFC_hls/rv32i_defs.h`,
`RV32IMFC_hls/fp_ops.h` y `RV32IMFC_hls/rv32c_defs.h` son *copias* de
sus equivalentes en `RV32IMFC_tlm/src/` (mismo contenido, mantenerlas
sincronizadas a mano si alguna cambia).

## Compilar y correr (RV32IMFC_tlm)

Requiere SystemC 2.3.4 (con TLM-2.0 incluido) — en Ubuntu/Debian:

```bash
sudo apt-get update && sudo apt-get install -y libsystemc-dev
```

```bash
g++ -std=c++17 -o riscv_sim RV32IMFC_tlm/src/main.cpp -lsystemc -lpthread
LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu ./riscv_sim
```

Esto corre el programa de prueba embebido en `main.cpp` (ejercita las
tres extensiones) e imprime el volcado de registros enteros y de punto
flotante, más el resultado del test end-to-end de la VectorUnit.

### Core RV32IMFC fuera de orden (`ProcessorOOO`)

Además del modelo in-order de arriba, `RV32IMFC_tlm/src/processor_ooo.h`
implementa el **mismo mecanismo Tomasulo-lite** (RAT+FRAT → ROB unificado
de 8 entradas, 2×ALU, MUL/DIV, FPU con familia FMADD, LSU, branch sin
especulación, **y unidad vectorial RVV**) que la pista HLS
(`RV32IMFC_hls/rv32_ooo.cpp`), pero como `SC_MODULE` initiator TLM-2.0
puro — mismo socket, mismo `bus_access` de 9 pasos, mismo fetch de 16
bits por el Bus. Corre el **mismo programa** (I+M+F+C+RVV) que verifica
la pista HLS, con los mismos resultados esperados y los mismos números de
ciclo — correr ambos y obtener el mismo estado arquitectónico es la
verificación cruzada TLM↔HLS.

```bash
g++ -std=c++17 -o riscv_ooo_sim RV32IMFC_tlm/src/main_ooo.cpp -lsystemc -lpthread
LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu ./riscv_ooo_sim
```

Imprime la traza de dispatch/completion/commit ciclo a ciclo y termina
con "Todos los checks pasaron." — incluida la evidencia de
reordenamiento: una instrucción despachada después de un `mul`/`div`
largo completa su ejecución antes (verificado también cruzando bancos
entero↔flotante, con un `addi` completando antes que un `fdiv`), **y de
coprocesamiento vectorial**: un `addi` escalar completa mientras un
`vadd.vv`/`vmul.vv` todavía está ejecutando en la unidad vectorial (el
core escalar avanza en paralelo, no bloqueado por RVV).

**Coprocesamiento vectorial (RVV)**: la unidad `VecRs` incorpora las 5
instrucciones RVV (`vle32.v`/`vse32.v` + `vadd.vv`/`vsub.vv`/`vmul.vv`,
codificación verificada contra la especificación oficial RVV v1.0) como
una reservation station más del mismo pipeline OOO — no un módulo
aparte. `vle32.v`/`vse32.v` se distinguen de `FLW`/`FSW` escalar por el
campo `width` y acceden a memoria por el Bus real (respetando TLM). El
banco vectorial (`vregs`) es un miembro público, verificado directo por
el testbench igual que `regs`/`fregs`.

**Nota de diseño importante**: a diferencia de la pista HLS (que tiene
`imem`/`dmem` físicamente separados), acá instrucciones y datos
comparten la misma RAM — las direcciones de scratch del programa de
prueba se eligieron deliberadamente lejos del código para no
auto-modificarlo en ejecución (ver comentario en `main_ooo.cpp`).

## Estructura del código

```
RV32IMFC_tlm/src/
├── rv32i_defs.h    # Opcodes, funct3/funct7 (base + M + F)
├── immediates.h    # Extraccion de inmediatos por formato (I/S/B/U/J)
├── fp_ops.h        # Semantica de F: bits<->float, FCVT saturado, FCLASS
├── rv32c_defs.h    # Extension C: expande comprimidas de 16 bits a 32 bits
├── memory_map.h    # Mapa de direcciones (RAM, rango reservado RVV)
├── memory.h        # Memoria TLM-2.0 (target)
├── bus.h           # Bus TLM-2.0 (target + initiator, decodifica direcciones)
├── processor.h      # CPU: fetch de 16 bits + decoder + registros enteros/flotantes
├── vector_unit.h   # Esqueleto RVV (initiator independiente)
├── processor_ooo.h # CPU RV32IMFC fuera de orden (Tomasulo-lite), initiator TLM alternativo
├── main.cpp        # Ensamblador de prueba + integracion + testbench (Processor in-order)
└── main_ooo.cpp    # Testbench de ProcessorOOO (mismo programa que RV32IMFC_hls/rv32_ooo_tb.cpp)

RV32IMFC_hls/
├── rv32i_defs.h         # Copia de RV32IMFC_tlm/src/rv32i_defs.h (autocontenido)
├── fp_ops.h             # Copia de RV32IMFC_tlm/src/fp_ops.h (ya portable, sin cambios)
├── rv32c_defs.h         # Copia de RV32IMFC_tlm/src/rv32c_defs.h (ya portable, sin cambios)
├── rv32_core.h/.cpp     # Decode+execute RV32IMFC sintetizable (Vitis HLS, AXI4 Full)
├── immediates_hls.h     # Espejo de immediates.h usando ap_uint en vez de sc_uint
├── rv32_core_tb.cpp     # Testbench de C-simulation
├── run_hls.tcl          # Script de Vitis HLS (csim + csynth)
├── vivado/
│   └── run_vivado.tcl   # Sintesis + implementacion en Vivado (numeros post-P&R)
└── README.md            # Como correrlo y que reportar
```

## Documentación

- **`context.md`**: historial de decisiones de diseño (por qué Bus, por
  qué VLEN=128, convenciones de estilo, etc.).
- **`explain.md`**: explicación línea por línea de cada archivo, con
  ejemplos numéricos y trazas end-to-end — el lugar para entender *cómo*
  funciona el código, no solo *qué* hace.
- **[`LIMITACIONES.md`](LIMITACIONES.md)**: limitaciones de alcance de
  ambas pistas (HLS/TLM) y de sus cores in-order/fuera de orden, con
  justificación técnica de cada una y qué haría falta para levantarlas
  — la referencia directa para la sección de limitaciones del artículo.

## Contexto

Proyecto de trabajo de posgrado (Maestría en Ingeniería Electrónica,
TEC), y base de un proyecto de equipo mayor cuyo objetivo final es un
core **RV32IMFC + RVV out-of-order**.
