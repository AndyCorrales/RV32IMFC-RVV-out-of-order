# RISC-V SoC — RV32IMFC OoO + TAGE + coprocesador RVV + interconnect con árbitro

Esta carpeta implementa **el diagrama del SoC** — cada bloque del dibujo
tiene su archivo, y cada archivo es un bloque del dibujo. Deriva de la
pista `RV32IMFC+RVV+OOO-HLS/` (las 6 fases de RVV + bare-metal completo)
y le **agrega los tres bloques que faltaban**: el predictor TAGE con
especulación real, el coprocesador vectorial desacoplado con su cola, y
el interconnect con árbitro hacia la memoria.

## El mapa: bloque del diagrama → archivo

```
┌─────────────────────────────── RISC-V SoC ────────────────────────────┐
│  ┌───────────── Core Escalar OoO ─────────────┐  ┌─ Coproc. vectorial ─┐
│  │  Frontend               Backend            │  │  Vector instruction │
│  │  ┌──────────┐           ┌───────────────┐  │  │  Queue (VIQ)        │
│  │  │ Fetch    │──────────▶│ Issue/Rename  │──┼──┼─▶ (FIFO de 4)       │
│  │  │ Decode   │           │ ALU/MUL/FPU   │  │  │        │            │
│  │  │ TAGE     │           │ LSU           │  │  │        ▼            │
│  │  └──────────┘           └───────┬───────┘  │  │  Unidades:          │
│  └───────────────────────────────  │  ────────┘  │  VALU/VMUL          │
│                                    ▼             │  VLSU/VSLDU ────┐   │
│                    ┌────────────────────────┐    └─────────────────┼───┘
│                    │ AXI4-Lite ctrl + árbitro│◀───────────────────┘
│                    └───────────┬────────────┘
│                                ▼
│                        Memoria de la FPGA (BRAM)
└───────────────────────────────────────────────────────────────────────┘
```

| Bloque del diagrama | Archivo | Qué contiene |
|---|---|---|
| Frontend: **Fetch, Decode** | `frontend_dispatch.h` | fetch (con expansión C), decodificación completa RV32IMFC, dispatch a las estaciones |
| Frontend: **TAGE** | `tage.h` | predictor real: bimodal (512×2b) + 3 tablas etiquetadas con historias 4/8/16 (geométricas), GHR de 16 bits |
| Backend: **Issue Queue / Rename** | `backend.h` + `soc_state.h` | RAT/FRAT (rename), estaciones de reserva (la issue queue, distribuida al estilo Tomasulo), CDB |
| Backend: **ALU/MUL/FPU/LSU** | `backend.h` | las unidades escalares y la lectura de operandos |
| **Vector instruction Queue** | `soc_state.h` (estado) + `vector_coprocessor.h` | FIFO de 4: el escalar encola y sigue; el coprocesador saca de la cabeza |
| Unidades **VALU/VMUL/VLSU/VSLDU** | `vector_coprocessor.h` + `exec_vector.h` | ruteo por unidad con latencias distintas (VALU 2, VMUL 4, VSLDU 2); VLSU un elemento/ciclo por el árbitro |
| **AXI4 Interconnect + Árbitro** | `axi_interconnect.h` | un puerto de datos, **una concesión por ciclo**, prioridad commit > VLSU > LSU; control del SoC por **AXI4-Lite** real (`s_axilite`) |
| **Memoria del FPGA** | puerto `dmem` (BRAM) | la memoria de datos on-chip; `imem` es una BRAM local del frontend (I-TCM) |
| (todo junto) | `soc_top.cpp` | el tick del SoC: commit → ejecución → issue → fetch/dispatch + coprocesador |

Comparten base con la pista anterior: `soc_config.h` (parámetros),
`rvv_encoding.h` (codificación RVV verificada contra la spec),
`soc_csr.h` (CSRs), `vector_dispatch.h` (decodificador OP-V),
`exec_vector.h` (datapath por elemento), y el ISA (`rv32i_defs.h`,
`rv32c_defs.h`, `fp_ops.h`, `immediates_hls.h`).

## Qué cambia respecto a `RV32IMFC+RVV+OOO-HLS/`

### 1. TAGE + especulación (el core ya NO se detiene en los branches)

- El branch condicional **se predice** (`tage_predict`) y el fetch sigue
  por el camino predicho. El destino no necesita BTB: viene en la
  instrucción (`pc+imm`).
- La unidad BR solo **comprueba** la predicción y anota el veredicto en
  el ROB. La resolución es **en el commit**: si falló, se redirige el
  fetch y se hace `pipeline_flush()` — exactamente el mecanismo de los
  traps precisos, reutilizado. Un mispredict es "un trap barato".
- El predictor **entrena en el commit** (con la verdad, en orden), y el
  GHR se repara con el snapshot guardado en el ROB.
- Especulan solo las unidades **renombradas** (ALU/MUL/FPU/LSU): escriben
  el ROB y el flush las descarta. Lo que escribe estado directo — `vset*`
  (vtype/vl en el dispatch) y el coprocesador entero (vregs sin
  renombrar) — **espera** a que no haya branch en vuelo
  (`branch_pending`).
- **JALR no se especula** (sin BTB/RAS el destino es incognoscible): ese
  sí detiene el fetch, como antes.

Resultado medido (suite E): lazo + branch alternante T,N,T,N…
**64 branches, 6 mispredicts (90.6 % de acierto)**. El branch alternante
es el caso que un bimodal puro falla ~50 % para siempre; TAGE lo aprende
con un bit de historia — esa es la razón de ser del predictor.

### 2. Coprocesador vectorial desacoplado (VIQ + 4 unidades)

- El dispatch escalar **encola** la vectorial en la VIQ (con su
  `vl`/`vstart`/`sew` capturados y su tag del ROB) **y sigue**: hasta 4
  vectoriales en vuelo mientras el escalar avanza. Antes había una sola
  estación: la segunda vectorial bloqueaba el dispatch entero.
- Dentro del coprocesador la ejecución es en orden (cabeza de la cola) y
  se rutea por unidad: **VALU** (lat 2) aritmética/máscaras/reducciones/
  widening, **VMUL** (lat 4) mul/div vectorial, **VSLDU** (lat 2)
  permutaciones, **VLSU** memoria.
- Las entradas en cola **escuchan el CDB** (pueden encolarse con un
  operando escalar en vuelo).
- El ROB común sigue mandando: retiro en orden, traps precisos.

**Bug real que apareció al desacoplar**: la segunda vectorial se
encolaba capturando el `vstart` viejo antes de que la primera lo
resetease al completar — el mismo patrón lee-en-dispatch /
escribe-al-completar de los CSR. Cura: con `vstart≠0` el desacople se
apaga (solo se encola con el coprocesador vacío). `vstart≠0` solo ocurre
tras un `csrw` o un trap: el stall ahí no cuesta nada.

### 3. Interconnect con árbitro (la memoria deja de ser mágica)

- **Un puerto de datos, una concesión por ciclo** (`axi_grant()`), con
  prioridad fija: stores en retiro > VLSU > LSU escalar. Un load escalar
  y uno vectorial en el mismo ciclo ahora **se serializan de verdad**.
- La **VLSU mueve un elemento por ciclo** a través del árbitro (antes:
  16 accesos simultáneos imposibles en un puerto real). Los elementos
  inactivos por máscara avanzan sin gastar puerto.
- El plano de **control es AXI4-Lite real** (`s_axilite`): así el PS del
  Kria arranca el core y lee su estado. La memoria de instrucciones es
  una BRAM local del frontend (I-TCM) — como en el diagrama, el fetch no
  pasa por el interconnect.

## Verificación

**8 suites, 218 checks, 0 fallos** — todas las de la pista base (ISA+RVV
fases 1-6, excepciones, UART/timer/M-U, printf de newlib) **corriendo
ahora bajo especulación**, más la suite E del predictor. Que el `printf`
de newlib complete entero (~5000 ciclos, decenas de branches reales)
sobre un frontend especulativo es el stress-test más fuerte del flush.

**Síntesis (csynth, Kria KV260):** **135.86 MHz** estimados, *loop
constraints* satisfechos, 60 677 LUT (51 %), 23 153 FF, 42 DSP, 8 BRAM.
Curiosidad honesta: el Fmax **subió** respecto a la pista base (128.25 →
135.86 MHz) — serializar la VLSU a un elemento por ciclo eliminó el
multiplexado ancho de los 16 accesos simultáneos, que pesaba en el camino
crítico. El costo fue área (+2 500 LUT por TAGE/VIQ/árbitro) y ciclos de
memoria vectorial.

```bash
# rapido (g++):
g++ -std=c++14 -I. -I$XILINX_VITIS/include -o soc soc_top.cpp soc_tb.cpp && ./soc
# flujo completo (csim + csynth contra el Kria KV260):
vitis_hls -f run_hls.tcl
```

## Limitaciones honestas (para la defensa)

- **Issue queue** = estaciones de reserva distribuidas (Tomasulo), no
  una cola unificada — mismo concepto, otra organización; el README lo
  dice y el diagrama lo agrupa como "Issue Queue/Rename".
- **JALR no especulado** (falta BTB/RAS — trabajo futuro natural).
- El coprocesador **no ejecuta especulativamente** (vregs sin renombrar);
  el desacople es de latencia, no de control.
- El dato va por **BRAM on-chip arbitrada** ("Memoria del FPGA"); el AXI
  real del top es el **control AXI4-Lite**. Un maestro AXI4 hacia DDR
  (`m_axi`) es la extensión natural si se quisiera memoria externa.
- Un flush por trap/interrupción no repara el GHR de los branches
  especulados descartados: el predictor puede quedar momentáneamente
  contaminado. Eso afecta la *exactitud* (rendimiento), jamás la
  *corrección* — el predictor solo predice.
- Se heredan las de la pista base: `LMUL≠1`, sin FP vectorial ni SEW=64
  (fuera de Zve32x), `vxrm` fijo en `rnu`, sin S-mode/MMU.
