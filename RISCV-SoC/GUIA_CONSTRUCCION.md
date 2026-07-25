# Guía de construcción — cómo se arma este SoC, archivo por archivo, en orden

Este documento es distinto de [EXPLICACION_COMPLETA.md](EXPLICACION_COMPLETA.md)
(que explica *qué es* cada cosa). Aquí se responde: **en qué orden se
construye, cuál fue la base, por qué cada archivo va donde va, y cómo se
logra armar cada pieza sobre la anterior.**

Hay dos "órdenes" y conviene no confundirlos:

- **El orden histórico** (lo que realmente pasó): se empezó con un core
  escalar en orden, se le puso Tomasulo, luego RVV, luego bare-metal, se
  refactorizó en headers, y al final se agregaron los bloques del SoC.
- **El orden de dependencias** (cómo se arma hoy, limpio): de los cimientos
  hacia arriba, cada archivo apoyándose solo en los anteriores.

Este documento sigue el **orden de dependencias**, porque es el que sirve
para *reconstruir* el proyecto o para entender por qué un archivo existe
antes que otro. En cada nivel marco cuándo, históricamente, apareció esa
pieza.

```
┌─ NIVEL 0 · cimientos (no dependen de nada) ────────────────┐
│  soc_top.h   rv32i_defs.h   fp_ops.h   immediates_hls.h     │
├─ NIVEL 1 · el ISA comprimido ──────────────────────────────┤
│  rv32c_defs.h                                               │
├─ NIVEL 2 · parámetros y codificación ──────────────────────┤
│  soc_config.h        rvv_encoding.h                         │
├─ NIVEL 3 · el estado (los registros del procesador) ───────┤
│  soc_state.h                                                │
├─ NIVEL 4 · lo que opera sobre el estado ───────────────────┤
│  soc_csr.h   backend.h   axi_interconnect.h   tage.h        │
├─ NIVEL 5 · la unidad vectorial ────────────────────────────┤
│  exec_vector.h   →   vector_coprocessor.h                   │
├─ NIVEL 6 · los decodificadores ────────────────────────────┤
│  vector_dispatch.h   →   frontend_dispatch.h                │
├─ NIVEL 7 · el lazo principal ──────────────────────────────┤
│  soc_top.cpp  (une todo: las 4 etapas del tick)             │
└────────────────────────────────────────────────────────────┘
```

**Regla de oro de todo el proyecto:** *nunca construir dos cosas sin
verificar la anterior.* Cada nivel se compila y se prueba antes de subir
al siguiente. Por eso hay una suite de tests que creció con el código.

---

## ¿Cuál fue la base? ¿Cómo se inició?

**La base fue un core escalar RV32I en orden.** Antes de cualquier cosa
vectorial o fuera de orden, existió un procesador que ejecutaba una
instrucción a la vez, en orden de programa. Eso permitió acertar el
**ISA** (decodificación, inmediatos, la ALU) sin la complejidad de
Tomasulo encima.

> **Método, no anécdota:** nunca se depura el ISA y la microarquitectura
> al mismo tiempo. Primero un core simple que ejecuta bien las
> instrucciones; después se le cambia el motor por debajo. Si algo falla
> en la etapa 2, ya sabés que el ISA era correcto.

De ese core escalar sobreviven **tal cual** los archivos del Nivel 0-1
(las definiciones del ISA): son independientes de *cómo* se ejecuta, así
que se reutilizaron sin tocar en el core OoO, en el core con RVV, y en el
SoC.

---

# NIVEL 0 — Los cimientos

Son archivos que **no dependen de nada** y que todo lo demás usa. Se
crean primero porque son el vocabulario común.

### `soc_top.h` (130 líneas) — el contrato con el mundo exterior

Lo primero que se define es **la interfaz del procesador**: qué entra y
qué sale. Aquí viven los tamaños del sistema y la firma de la función top:

```c
#define OOO_IMEM_WORDS 16384      // memoria de instrucciones (64 KB)
#define OOO_DMEM_WORDS 16384      // memoria de datos
#define OOO_VEC_LANES  4          // VLEN=128 / 32 = 4 palabras por vreg

void riscv_soc_tick(reset, imem, dmem, ...salidas..., halted);
```

**Por qué primero:** en HLS, la firma de la función top *es* la interfaz
de hardware (los puertos del chip). Definirla primero fija el contrato:
`imem`/`dmem` serán puertos BRAM, las señales de observación serán cables,
el `return` será el control AXI-Lite. Todo lo demás se construye para
llenar esta función.

### `rv32i_defs.h` (206 líneas) — el diccionario del ISA

Constantes para **cada opcode, funct3 y funct7** de RV32I, más funciones
que extraen los campos de una instrucción de 32 bits:

```c
namespace rv32i {
    namespace Opcode { constexpr uint32_t OP = 0b0110011, LOAD = ...; }
    ap_uint<5> rd(uint32_t iw)     { return (iw >> 7) & 0x1F; }
    ap_uint<3> funct3(uint32_t iw) { return (iw >> 12) & 0x7; }
}
```

**Por qué:** es el diccionario que traduce bits ↔ significado. Sin esto,
el decodificador serían números mágicos. Se hizo temprano y **no cambió
nunca** — el ISA es un estándar fijo.

### `fp_ops.h` (74 líneas) — la aritmética de punto flotante

Las operaciones IEEE-754 (suma, mul, div, sqrt, conversiones) como
funciones puras sobre `float`. En simulación usa el `float` nativo de C++;
en síntesis, Vitis las mapea a IP de punto flotante de la FPGA.

**Por qué separado:** es matemática pura, sin estado. Aislarla la hace
verificable sola y reutilizable por la FPU escalar y (a futuro) la
vectorial.

### `immediates_hls.h` (63 líneas) — armar los inmediatos

Las instrucciones RISC-V guardan las constantes ("inmediatos") **troceadas
y reordenadas** entre los bits, distinto en cada formato. Estas funciones
las reensamblan: `get_imm_I`, `get_imm_S`, `get_imm_B`, `get_imm_U`,
`get_imm_J`.

**Por qué:** es un detalle mecánico pero delicado (un bit mal puesto
rompe todos los saltos). Encapsularlo evita repetir el rearmado en cada
sitio del decodificador.

---

# NIVEL 1 — El ISA comprimido

### `rv32c_defs.h` (291 líneas) — la extensión C

Depende de `rv32i_defs.h`. Contiene `rv32c::expand()`: toma una
instrucción comprimida de **16 bits** y devuelve su equivalente de 32
bits.

**Por qué en su propio nivel:** la extensión C es un *traductor* que se
apoya en las definiciones de RV32I (expande *hacia* instrucciones RV32I).
Va después de `rv32i_defs.h` y antes que el fetch, que es su único
cliente. La decisión de diseño —expandir en el fetch para que el resto del
pipeline solo vea 32 bits— hace que este archivo sea el **único** que sabe
de instrucciones de 16 bits.

---

# NIVEL 2 — Parámetros y codificación

Aquí se define *cómo se configura* la máquina, antes de declarar su estado.

### `soc_config.h` (41 líneas) — las perillas del diseño

Las **latencias** de cada unidad y los **tamaños** de las estructuras:

```c
static const int ROB_SZ = 8;      // 8 entradas en el ROB
static const int N_ALU  = 2;      // 2 ALUs
static const int DIV_LAT = 8;     // la división tarda 8 ciclos
static const int VMUL_LAT = 4;    // el multiplicador vectorial, 4
```

**Por qué antes del estado:** el estado (Nivel 3) necesita estos números
para dimensionarse — `rob[ROB_SZ]` no se puede declarar sin `ROB_SZ`. Es
el archivo que se toca para *explorar el diseño* (más ALUs, ROB más
profundo) sin tocar la lógica.

### `rvv_encoding.h` (166 líneas) — la codificación vectorial

Todos los opcodes, funct6, funct3 y campos de `vtype` de RVV, verificados
contra la especificación oficial. **Solo constantes, sin lógica.**

**Por qué aquí:** es el `rv32i_defs.h` del mundo vectorial. Va en este
nivel porque lo usan tanto el estado (para dimensionar) como la unidad
vectorial y su decodificador. Se construyó **incrementalmente, fase por
fase**: cada fase de RVV agregó sus constantes aquí antes de implementar
la lógica.

---

# NIVEL 3 — El estado (el punto de inflexión del diseño)

### `soc_state.h` (217 líneas) — los registros del procesador

Este es **el archivo más importante para entender la microarquitectura**,
porque define *qué recuerda* el procesador entre ciclos. Depende de
`soc_config.h` y `rvv_encoding.h`.

```c
// bancos de registros arquitectónicos
static ap_uint<32> regfile[32];    // enteros
static ap_uint<32> fregs[32];      // flotantes
static ap_uint<32> vregs[32*4];    // vectoriales (plano)

// la maquinaria Tomasulo
static RobEntry rob[ROB_SZ];       // el reorder buffer
static RatEntry rat[32], frat[32]; // renombrado
static AluRs    alu_rs[N_ALU];     // estaciones de reserva...
static VecRs    vec_rs;

// el frontend especulativo (SoC)
static Tage     tage;
static ap_uint<4> branch_pending;

// el coprocesador (SoC): la cola vectorial
static VecRs    viq[VIQ_SZ];       // Vector Instruction Queue

// el árbitro del interconnect (SoC)
static bool     mem_port_used;
```

**Por qué aquí, y por qué importa el orden:** todo lo que ejecuta (Niveles
4-7) *opera sobre este estado*. Definirlo primero significa que las
funciones de ejecución no pasan el estado por parámetros — lo ven directo,
porque es `static` (una sola unidad de traducción). Ese es el pilar que
permite partir el código en headers sin que se rompa: **hay un solo ROB,
un solo banco, compartido por todos los archivos.**

> **Cómo se construye en la práctica:** este archivo *creció* con el
> proyecto. Primero solo tenía `regfile`, `rob`, `rat`, `alu_rs`. F agregó
> `fregs`/`frat`. RVV agregó `vregs`/`vec_rs`. Bare-metal agregó los CSRs.
> El SoC agregó `tage`, `branch_pending`, `viq`, `mem_port_used`. Cada
> struct nuevo (`RobEntry`, `VecRs`…) se extendió con campos a medida que
> las features los pedían — por ejemplo, `RobEntry.is_branch` y
> `ghr_snap` aparecieron cuando se agregó la especulación.

---

# NIVEL 4 — Lo que opera sobre el estado (funciones puras y semi-puras)

Con el estado definido, se construyen las unidades que lo leen y
modifican. Los cuatro archivos de este nivel son **independientes entre
sí** (solo dependen del estado), así que se pueden construir en cualquier
orden — pero conceptualmente van juntos.

### `soc_csr.h` (60 líneas) — el banco de CSRs

`read_csr()` y `write_csr()`: el punto único por donde pasa todo acceso a
los registros de control (mstatus, mtvec, mepc, vl, vstart…).

**Por qué:** centralizar los CSRs en un solo lugar mantiene el orden de
programa (las instrucciones CSR ejecutan en la cabeza del ROB). Se
construyó con el bare-metal.

### `backend.h` (263 líneas) — las unidades escalares y el bus de datos

El más denso de este nivel. Contiene:

- `cdb_broadcast()`: el Common Data Bus — despierta a toda RS (y a la VIQ)
  que esperaba un tag.
- `read_operand()` / `read_operand_fp()`: los **tres casos** del
  renombrado (banco / bypass del ROB / esperar al CDB).
- `alu_compute()`, `md_compute()` (mul/div), `fpu_compute()` (la FPU),
  `branch_taken()`.

**Por qué se construye aquí y en este orden interno:** `read_operand` es
el mecanismo central de Tomasulo; todo lo que despacha lo usa. `cdb_broadcast`
es su contraparte (uno lee tags, el otro los resuelve). Fue de lo primero
que se escribió al pasar de en-orden a fuera-de-orden.

### `axi_interconnect.h` (89 líneas) — memoria de datos + árbitro

`dmem_load`/`dmem_store` (acceso con los anchos de RV32I) y `axi_grant()`
(el árbitro: una concesión de memoria por ciclo). También vive aquí
`pipeline_flush()`.

**Por qué juntos:** el acceso a memoria y quién lo controla son la misma
responsabilidad. `dmem_load/store` venían del core base; `axi_grant` es la
adición del SoC — se metió en el mismo archivo porque es el guardián del
mismo recurso.

### `tage.h` (136 líneas) — el predictor de saltos

Depende solo de `soc_top.h`. Es autocontenido: sus propias tablas,
`predict()`, `update()`, `reset()`.

**Por qué tan independiente:** un predictor es un módulo cerrado —
entra un `pc`, sale una predicción; entra un resultado, aprende. No
necesita el resto del estado del procesador. Por eso es de los archivos
más fáciles de razonar y probar aislado. Se agregó **al final**, con el
SoC, pero encaja en el Nivel 4 por sus dependencias.

---

# NIVEL 5 — La unidad vectorial

### `exec_vector.h` (453 líneas) — el datapath vectorial

El archivo más grande. Depende de `backend.h` (usa la extensión de signo
y helpers). Contiene:

- `vreg_get`/`vreg_set` (y las variantes `_pair` para EMUL=2): el acceso
  al banco plano con ancho de elemento variable — **la idea que sostiene
  todo RVV**.
- Las ALUs vectoriales: `vec_alu`, `vec_cmp`, `vec_wide_alu`,
  `vec_narrow_alu`, `vec_mask_logic`.
- `vec_arith_compute`: el despachador por categoría (VCAT_*).

**Por qué después de `backend.h`:** reutiliza la aritmética escalar (por
ejemplo, la extensión de signo) y sigue el mismo patrón que la ALU
escalar. **Cómo se construyó:** una fase de RVV a la vez. Primero
`vreg_get/set` + `vec_alu` (fases 1-2). Luego `vec_cmp` y las máscaras
(fase 3). Luego reducciones/permutaciones (fase 4). Luego
`vec_wide_alu`/`vec_narrow_alu` + los accesores `_pair` (fase 4d). Cada
una sumó código aquí y una suite de tests.

### `vector_coprocessor.h` (236 líneas) — VIQ + las unidades del SoC

Depende de `exec_vector.h`. Envuelve el datapath vectorial en el
**coprocesador desacoplado**: saca instrucciones de la VIQ, las rutea a
VALU/VMUL/VSLDU/VLSU con sus latencias, y corre la VLSU un elemento por
ciclo a través del árbitro.

**Por qué encima de `exec_vector.h`:** el coprocesador es la *organización*
(cola, unidades, latencias, árbitro); el datapath es el *cálculo*. Se
construye el cálculo primero, se lo envuelve después. Es la adición del
SoC sobre la unidad vectorial base.

---

# NIVEL 6 — Los decodificadores

Con toda la maquinaria lista, se construye lo que *decide qué hacer* con
cada instrucción.

### `vector_dispatch.h` (321 líneas) — decodificar RVV

Depende de csr + backend + exec_vector. Decodifica el opcode OP-V:
`vsetvli` y toda la aritmética vectorial. Encola en la VIQ.

**Por qué antes que el frontend escalar:** porque el frontend lo
*incluye* y lo llama. OP-V no colisiona con ningún otro opcode, así que se
resuelve aparte y primero.

### `frontend_dispatch.h` (390 líneas) — el decodificador principal

Depende de todo lo anterior, incluido `vector_dispatch.h`. Es la **tabla
de decodificación completa** de RV32IMFC: mira el opcode y despacha a la
estación correcta, leyendo operandos (renombrado) y reservando la entrada
del ROB. Aquí vive la predicción de branches (llama a `tage.predict`).

**Por qué es el penúltimo:** el dispatch necesita que *todas* las unidades
y el estado existan para poder repartir el trabajo. Es el archivo que más
creció al agregar instrucciones — por eso se separó del tick.

> **Detalle de orden que importa:** los load/store vectoriales comparten
> opcode con FLW/FSW escalar; solo los distingue el campo `width`. Por eso
> están en `frontend_dispatch.h` (no en `vector_dispatch.h`) y su posición
> en la cadena de `if/else` es *semántica*: deben comprobarse antes que
> FLW/FSW.

---

# NIVEL 7 — El lazo principal

### `soc_top.cpp` (582 líneas) — el tick

Incluye **todos** los headers y contiene solo el lazo principal: las
cuatro etapas del pipeline, en orden:

```
Etapa 1: COMMIT     – retiro en orden; aquí se toman traps, interrupciones
                      y se resuelven los branches especulados
Etapa 2: EJECUCIÓN  – cada unidad avanza; el coprocesador da un paso
Etapa 3: ISSUE      – una RS con operandos listos arranca
Etapa 4: FETCH+DISPATCH – llama a frontend_dispatch
```

**Por qué es el último y por qué el orden de las etapas es al revés del
flujo:** commit va **primero** para que la entrada del ROB que se libera
pueda reusarse en el dispatch del mismo ciclo. El tick es el director de
orquesta: no calcula casi nada él mismo, solo llama a las piezas de los
niveles anteriores en la secuencia correcta.

**Por qué es una sola unidad de traducción:** el `.cpp` incluye los `.h`
en vez de compilarlos por separado, porque el estado es `static`. Si
fueran unidades de traducción distintas, cada una tendría su propia copia
del ROB. Un solo `.cpp` que incluye todo → un solo procesador.

---

# El resumen: cómo reconstruirlo desde cero

Si tuvieras que rehacer este proyecto, este es el orden y por qué:

| # | Construye | Apoyándote en | Verifica con |
|---|---|---|---|
| 1 | ISA: `rv32i_defs`, `immediates`, `fp_ops`, `soc_top.h` | nada | que compile; decodificar a mano |
| 2 | Extensión C: `rv32c_defs` | ISA | expandir instrucciones conocidas |
| 3 | Config + estado: `soc_config`, `soc_state` | ISA | que el ROB/bancos existan |
| 4 | **Core escalar en orden** (ALU, LSU, fetch/decode básico) | estado | un programa entero suma bien |
| 5 | **Tomasulo**: `backend` (CDB, read_operand), commit del ROB | estado | una división no bloquea sumas |
| 6 | M y F: mul/div y FPU en `backend` | Tomasulo | mul/fdiv correctos, OoO cruzando bancos |
| 7 | **Bare-metal**: `soc_csr`, traps, timer, M/U | commit | un ELF real con handler + MRET |
| 8 | **RVV** fase por fase: `rvv_encoding`, `exec_vector`, `vector_dispatch` | estado + backend | una suite nueva por fase |
| 9 | Refactor: partir el `.cpp` gigante en headers | todo verde | que los tests sigan idénticos |
| 10 | **SoC**: `tage`, `vector_coprocessor`, árbitro en `axi_interconnect` | la base estable | 8 suites + paridad con TLM |

Los pasos **4-9** son la carpeta base (`RV32IMFC+RVV+OOO-HLS`). El paso
**10** es lo que convierte esa base en este SoC. En cada paso, *nada sube
de nivel hasta que el nivel actual pasa sus tests* — esa disciplina es la
razón de que los bugs se cazaran temprano y de que un refactor de 2364
líneas fuera seguro.

---

*Para entender qué hace cada pieza conceptualmente, ver
[EXPLICACION_COMPLETA.md](EXPLICACION_COMPLETA.md). Para los resultados
medidos, ver [RESULTADOS.md](../RESULTADOS.md).*
