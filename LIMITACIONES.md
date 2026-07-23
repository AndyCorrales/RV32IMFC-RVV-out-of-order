# Limitaciones conocidas

Este documento centraliza las limitaciones de alcance de las dos pistas
del proyecto (`RV32IMFC_hls/` y `RV32IMFC_tlm/`), separadas por pista y
por core (in-order / fuera de orden). La mayoría son decisiones de
diseño deliberadas para acotar el trabajo dentro del tiempo del curso,
no bugs — donde aplica, se indica la justificación técnica y qué haría
falta para levantarlas.

---

## `RV32IMFC_hls/` — pista de síntesis (Vitis HLS)

### Core escalar in-order (`rv32_core.cpp`)

- **Acceso a memoria de datos que cruza el límite de una palabra AXI no
  se resuelve correctamente** (p. ej. `LW` en una dirección que no es
  múltiplo de 4 y cuyos 4 bytes caen en dos palabras distintas). La
  memoria es un maestro AXI4 Full direccionado por palabra de 32 bits
  (`mem_load`/`mem_store`), no un arreglo de bytes — un acceso de un
  solo `mem[word_addr]` no alcanza a leer/escribir el byte que cae en
  la palabra siguiente.
  - *Justificación*: el ISA de RISC-V no exige soporte de hardware para
    accesos desalineados (es *implementation-defined*); muchos cores
    embebidos reales tampoco lo soportan y dependen del compilador o de
    un trap por software.
  - *Cómo se arreglaría*: detectar `(addr & 3) + len > 4`, leer/escribir
    las dos palabras AXI involucradas y combinar/enmascarar los bytes
    de cada una — la misma técnica que ya se usó para resolver el
    straddle de instrucciones comprimidas en `rv32_ooo.cpp` (ver abajo).
- **`SB`/`SH` hacen *read-modify-write* de la palabra completa** (no hay
  señal de byte-enable a este nivel de abstracción con un puntero HLS
  simple) — un ciclo extra de lectura por cada store parcial.
- **Limitación de alineación de comprimidas (extensión C)**: `instr`
  siempre entra como una palabra de 32 bits ya alineada a 4 bytes; si es
  comprimida, se usa solo su mitad baja de 16 bits. Una instrucción
  comprimida que caiga en la mitad **alta** de una palabra (o
  intercalada en cualquier dirección par) no está cubierta. El modelo
  TLM sí soporta cualquier alineación de 2 bytes porque su fetch es un
  acceso de bus real de 16 bits, no un parámetro de entrada fijo.
  - *Ya resuelto en el core OOO de esta misma pista* (ver abajo) — el
    fetch de `rv32_ooo.cpp` lee dos palabras y selecciona por `pc[1]`,
    soportando cualquier alineación par.
- **Es una función de un solo paso** (decode+execute de una instrucción
  por llamada, sin loop de fetch interno). Para correr un programa
  completo en hardware real hace falta un wrapper externo que invoque
  la función repetidamente, alimentando `pc`/`instr` y realimentando
  `next_pc`/`regs_out` — no existe todavía (ver también la sección de
  "próximo paso" en `RV32IMFC_hls/README.md` sobre integración con el
  Zynq PS del Kria).

### Core fuera de orden (`rv32_ooo.cpp`)

Documentadas también como comentario de cabecera en `rv32_ooo.h`.

- **Sin especulación de saltos**: el fetch se detiene al despachar
  `BRANCH`/`JALR` hasta que la unidad de branch resuelve destino real
  (`JAL` redirige en el propio dispatch, con destino conocido
  estáticamente). Nunca hay instrucciones en vuelo por un camino
  equivocado, así que no hace falta lógica de *squash*/*recovery* — pero
  tampoco hay ganancia de ILP a través de saltos no resueltos.
- **Memoria en orden y conservadora**: una sola reservation station de
  LSU (un acceso en vuelo a la vez); un *load* solo ejecuta cuando su
  entrada llega a la cabeza del ROB (garantiza que todo *store* anterior
  ya escribió memoria); un *store* calcula dirección/dato en cuanto
  puede pero solo escribe memoria al retirarse (commit). No hay
  desambiguación de memoria (*memory disambiguation*) — sería necesaria
  para permitir loads/stores independientes fuera de orden entre sí.
- **Hereda la limitación de acceso a memoria que cruza palabra AXI** del
  core escalar (mismo `dmem_load`/`dmem_store`, mismo alcance).
- **CDB modelado como un bus de resultado por unidad, no un único CDB
  arbitrado**: cada unidad funcional difunde su propio `(tag, valor)` de
  forma independiente en el ciclo que completa. Simplificación
  documentada — un CDB único requeriría arbitraje explícito si dos
  unidades completan el mismo ciclo (acá no puede pasar porque cada una
  tiene su propia línea de broadcast).
- **Tamaños fijos y pequeños**: ROB de 8 entradas, 2 reservation
  stations de ALU, 1 de MUL/DIV, 1 de FPU, 1 de LSU, 1 de branch, 1 de
  VEC — no configurable en tiempo de síntesis, elegido para acotar el
  trabajo de verificación dentro del tiempo disponible.
- **Solo RV32IMFC entero/flotante de simple precisión**: sin `frm`/
  `fflags` (CSR de modo de redondeo/excepciones — `rm=RNE` fijo, igual
  que el resto del proyecto), sin doble precisión (D).
- **Opcodes no soportados** (`SYSTEM`/`FENCE`/CSRs) se retiran como
  *no-op* silencioso en vez de disparar una excepción real — no hay
  infraestructura de traps.
- Mismo requisito de wrapper externo que el core escalar para correr en
  hardware real (acá además el "tick" representa un ciclo de la
  microarquitectura, no un paso completo de instrucción).

**RVV integrado (unidad `VecRs`)** — a diferencia de `rv32_vector.cpp`
(ver abajo), esto SÍ corre junto al resto del pipeline OOO:

- **Banco vectorial sin renombrar** (sin VRAT): una sola reservation
  station VEC serializa las instrucciones vectoriales *entre sí* en
  orden de programa (misma lógica que "memoria en orden" de la LSU),
  aunque se solapan libremente con instrucciones escalares
  independientes — esa es la evidencia de coprocesamiento real
  verificada en el testbench (un `addi` completa mientras `vadd.vv`
  sigue ejecutando).
- **`vle32.v`/`vse32.v` se resuelven solo en la cabeza del ROB** — más
  conservador que un store escalar (que calcula dirección/dato antes
  pero escribe en el commit): se aplicó la misma regla a ambos para no
  tener que ampliar el ROB a 128 bits (4 lanes) solo para los stores
  vectoriales.
- **`vsetvli`/`vsetivli`/`vsetvl` implementadas** con **`vl` dinámico**
  (`vl = min(AVL, VLMAX)`), `vtype` real y política *tail-undisturbed*:
  las vectoriales operan solo sobre los primeros `vl` elementos y dejan
  el resto del registro sin tocar. Al reset `vtype.vill=1` y `vl=0`
  (spec 3.11), así que un programa RVV real debe empezar con `vsetvli`
  igual que en hardware de verdad. Se resuelve en el *dispatch* (como
  `LUI`/`JAL`): es preciso porque el dispatch es en orden y este core no
  especula, y evita serializar la unidad VEC. Si su AVL todavía está en
  vuelo, el dispatch se detiene un ciclo.
  **Lo que sigue faltando**: solo se acepta `SEW=32`/`LMUL=1` (cualquier
  otra configuración activa `vill` y deja `vl=0`, que es el
  comportamiento correcto) — sin anchos de elemento variables (8/16/64)
  ni agrupación de registros (LMUL≠1).
- **Solo 5 instrucciones de datos** (`vle32.v`/`vse32.v` +
  `vadd.vv`/`vsub.vv`/`vmul.vv`), **sin máscara** (`vm=1` siempre) — ver
  el detalle completo abajo.
- **La distinción con `FLW`/`FSW` escalar es el campo `width`**
  (`010`=escalar, `110`=vectorial) — ambos opcodes (`LOAD-FP`/
  `STORE-FP`) se comparten con la extensión F, el decoder los separa
  antes de rutear a la LSU o a la unidad VEC.

### Decoder RVV mínimo standalone (`rv32_vector.cpp`)

- **No integrado a ningún core** (a diferencia de la unidad `VecRs` de
  `rv32_ooo.cpp` de arriba): es un módulo separado que decodifica
  instrucciones vectoriales reales por su cuenta, pero el operando
  escalar `rs1` (dirección base) entra ya resuelto como parámetro — no
  hay ningún banco de registros enteros en este archivo.
- **Sin `vsetvli`** (a diferencia del core OOO integrado, que sí lo
  tiene): `SEW=32`/`LMUL=1`/`VLMAX=4` están fijos y no hay `vtype`/`vl`
  — este módulo quedó como el prototipo previo a la integración.
- **Sin máscara** (`vm=1` siempre) — `v0` no se usa como registro de
  máscara, ninguna instrucción predicada.
- **Solo 5 instrucciones**: `vle32.v`/`vse32.v` (memoria unit-stride,
  sin segmentos ni acceso strided/indexed) y `vadd.vv`/`vsub.vv`/
  `vmul.vv` (aritmética vector-vector). Sin reducción, sin permutación,
  sin punto flotante vectorial, sin el resto de la extensión M
  vectorial (división, widening, multiply-add).
  - *Justificación*: cobertura mínima pero verificada de las dos
    categorías estructurales del ISA (memoria + aritmética), priorizada
    sobre cobertura amplia dado el tiempo disponible.
  - *Codificación de bits verificada contra la especificación oficial*
    (RVV v1.0, secciones 7 y 19) — no es una codificación inventada ni
    aproximada.

### Comunes a toda la pista HLS

- **No hay integración física con el Kria KV260 todavía**: lo que existe
  es síntesis *out-of-context* (`run_hls.tcl`/`run_hls_ooo_core.tcl`/
  `run_hls_ooo.tcl`) — métricas reales de recursos/Fmax del bloque
  aislado, pero sin Block Design con el Zynq PS, sin bitstream cargado
  en la placa. El flujo completo (IP export → Block Design → bitstream →
  prueba en hardware) está documentado como próximo paso pero no
  ejecutado.
- **Post-P&R real**: el core OOO+RVV+bare-metal **sí** se llevó hasta
  place & route en Vivado (`run_hls_ooo_impl.tcl`, `export_design -flow
  impl`) — hay números post-implementación (7603 LUT, 4829 FF, 21 BRAM,
  ≈134.6 MHz, cierra timing a 100 MHz), documentados en
  `RV32IMFC_hls/README.md`. Los otros cores (escalar, `ooo_demo`,
  `rv32_vector`) siguen reportando solo estimados de HLS
  (`csynth_design`); correr su P&R es directo pero no se hizo.
- Sin CSRs, `ECALL`/`EBREAK`, `FENCE`, loader de ELF, ni periférico
  UART — no se pueden correr binarios reales compilados con un
  toolchain (`riscv32-unknown-elf-gcc`), solo los programas de prueba
  ensamblados a mano en los testbenches.

---

## `RV32IMFC_tlm/` — pista de verificación funcional (SystemC/TLM-2.0)

### Core in-order (`Processor`, `processor.h`)

- Sin CSRs, `ECALL`/`EBREAK`, `FENCE`, loader de ELF, ni periférico
  UART — el programa entra por un `memcpy` "backdoor" a la RAM antes de
  `sc_start()`, no por un binario real.
- Sin `frm`/`fflags` de la extensión F (`rm=RNE` fijo), sin doble
  precisión (D).
- No es sintetizable tal cual (usa sockets TLM, `tlm_generic_payload`,
  I/O de C++) — es exclusivamente un modelo de verificación funcional;
  para métricas de hardware está la pista HLS, en paralelo.

### Core fuera de orden (`ProcessorOOO`, `processor_ooo.h`)

Mismas decisiones de alcance que `rv32_ooo.cpp` (documentadas también
como comentario de cabecera en `processor_ooo.h`), portadas 1:1 para
que ambas pistas sean comparables:

- Sin especulación de saltos (fetch detenido hasta resolver
  `BRANCH`/`JALR`).
- Memoria en orden: un acceso en vuelo, *load* solo en la cabeza del
  ROB, *store* escribe por el Bus recién al commit. Sin desambiguación
  de memoria.
- ROB de 8 entradas, 2×ALU, 1×MUL/DIV, 1×FPU, 1×LSU, 1×branch, 1×VEC —
  mismos tamaños fijos que la pista HLS (a propósito, para que el mismo
  programa de prueba dé el mismo resultado ciclo a ciclo en ambas
  pistas, como efectivamente se verificó — incluida la sección RVV).
- Sin `frm`/`fflags`, sin D.
- **RVV integrado** (unidad `VecRs`): exactamente las mismas
  capacidades y simplificaciones que la pista HLS — banco vectorial sin
  renombrar, una sola RS que serializa las vectoriales entre sí,
  `vle32.v`/`vse32.v` resueltas en la cabeza del ROB, `vsetvli` con `vl`
  dinámico y *tail-undisturbed*, única configuración soportada
  `SEW=32`/`LMUL=1` (`VLEN=128`), sin máscara, 5 instrucciones de datos.
  A diferencia de HLS, la memoria vectorial pasa por el Bus real
  (`b_transport`), respetando la convención TLM. El esqueleto
  independiente `VectorUnit` (ver abajo) sigue existiendo pero no se usa
  en este pipeline.
- Opcodes no soportados se retiran como *no-op*, sin traps reales.

**Ventaja sobre la pista HLS** (vale la pena tenerlo presente, no es
simétrico en todo): la memoria TLM es un `std::vector<uint8_t>` real, así
que `LB`/`LH`/`LW`/`SB`/`SH`/`SW` desalineados **y que cruzan el límite
de una palabra** funcionan correctamente de forma nativa, tanto en el
core in-order como en el OOO — a diferencia de la pista HLS (ver
arriba), donde esa limitación sigue abierta en ambos cores.

**Nota de diseño verificada en la práctica**: instrucciones y datos
comparten la misma RAM (no hay `imem`/`dmem` separados como en HLS) — el
testbench de `ProcessorOOO` (`main_ooo.cpp`) tuvo que elegir direcciones
de *scratch* lejos del código a propósito para no auto-modificar el
programa en ejecución (un bug real que apareció y se corrigió durante el
desarrollo, documentado en el comentario de esa sección del archivo).

### Comunes a toda la pista TLM

- **RVV**: `VectorUnit` (`vector_unit.h`) es solo un esqueleto —
  banco de 32 registros vectoriales (VLEN=128 bits) y un método de
  prueba (`vector_load_test`) que valida el camino CPU → Bus → Memory →
  VectorUnit end-to-end. No hay decoder de instrucciones vectoriales
  reales (`vadd.vv`, `vle32.v`, etc.), y no está conectado al pipeline
  de `Processor` ni de `ProcessorOOO`.
- Sin CSRs, `ECALL`/`EBREAK`, `FENCE`, loader de ELF, ni UART (igual que
  arriba) — necesarios para correr binarios *bare-metal* reales.

---

## Comunes a todo el proyecto (ambas pistas, ambos cores)

- **RVV real**: **ambos cores OOO** (HLS `rv32_ooo.cpp` y TLM
  `processor_ooo.h`) tienen RVV **integrado** como coprocesamiento real
  verificado (unidad `VecRs`) — con los mismos números de ciclo en las
  dos pistas. Sigue faltando en los cores in-order (HLS `rv32_core.cpp`
  y TLM `Processor`), donde no aporta al objetivo (el tema pide OOO).
  `vsetvli` con `vl` dinámico ya está en ambas pistas; lo que sigue
  pendiente es SEW/LMUL variables, máscara, y el resto de las categorías
  del ISA (reducción, permutación, punto flotante vectorial) hacia el
  objetivo final del equipo (RV32IMFC + RVV out-of-order).
- **Bare-metal** (correr binarios compilados reales, no solo programas
  ensamblados a mano):
  - **Hecho en el HLS OOO** (`rv32_ooo.cpp`, items 1-5): loader de ELF32
    (`rv32_ooo_baremetal_tb.cpp`), opcode `SYSTEM`, `ECALL`/`EBREAK`
    (halt preciso en el commit), instrucciones CSR (`CSRRW`/`S`/`C` +
    inmediato) y banco de CSRs de modo máquina, más `FENCE` como no-op.
    Verificado ejecutando un binario ELF real de
    `riscv64-unknown-elf-gcc` (loop + acceso a CSR + `ecall`), y sigue
    sintetizando (135.86 MHz).
  - **Falta (item 6): traps precisas** — el `ECALL`-como-halt funciona
    sin ellas, pero un manejo de excepciones/interrupciones *reanudable*
    (guardar `mepc`/`mcause`, saltar a `mtvec`, volver con `MRET`)
    requiere el *squash* del ROB que el diseño evita a propósito. Es el
    componente estructuralmente más difícil en un core OOO (mismo motivo
    por el que no hay especulación de branches).
  - **Falta también**: periférico UART mapeado en memoria (para `printf`
    real), y el mecanismo está solo en el HLS OOO — no portado a las
    otras tres combinaciones (in-order de ambas pistas, TLM OOO).
- **Doble precisión (D)** y **CSRs de F** (`frm`/`fflags`) fuera de
  alcance en las cuatro combinaciones — siempre se asume `rm=RNE`.
