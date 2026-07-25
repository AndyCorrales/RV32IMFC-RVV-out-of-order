# De cero a un SoC RISC-V vectorial fuera de orden — la explicación completa

Este documento explica **todo** el trabajo de la pista HLS, empezando por
lo más básico (¿qué es un RISC-V?) y llegando hasta el SoC completo
(fuera de orden + vectorial + especulación + AXI). Está pensado para
leerse de corrido: cada parte se apoya en la anterior.

- **Parte I** — Fundamentos: qué es RISC-V y qué significa "hacer" un procesador
- **Parte II** — Las extensiones I, M, F, C
- **Parte III** — Ejecución fuera de orden (Tomasulo): el corazón
- **Parte IV** — RVV: la extensión vectorial
- **Parte V** — Bare-metal: correr software de verdad
- **Parte VI** — El SoC: TAGE, coprocesador desacoplado, interconnect AXI
- **Parte VII** — Cómo se construyó todo, en orden, y cómo se verificó

---

# Parte I — Fundamentos

## 1. ¿Qué es RISC-V?

**RISC-V es una ISA** (Instruction Set Architecture): el *contrato* entre
el software y el hardware. Define qué instrucciones existen, qué
registros hay y qué hace cada bit — **pero no dice cómo construir el
chip**. Es como las reglas del ajedrez: definen el juego, no la estrategia
ni el material del tablero.

Dos cosas la hacen especial:

- **Es abierta y libre.** Cualquiera puede implementarla sin pagar
  licencia (a diferencia de x86 de Intel o ARM). Por eso hay cientos de
  cores RISC-V, desde microcontroladores hasta supercomputadoras.
- **Es RISC** (*Reduced Instruction Set Computer*): pocas instrucciones,
  simples y regulares. Cada instrucción hace *una* cosa. Lo contrario es
  CISC (x86), donde una sola instrucción puede leer memoria, operar y
  escribir de vuelta. RISC apuesta a que instrucciones simples +
  compilador inteligente > instrucciones complejas.

### La base: RV32I

El núcleo mínimo se llama **RV32I**: enteros de **32 bits** (`RV32`),
extensión base de enteros (`I`). Tiene:

- **32 registros** de 32 bits (`x0`–`x31`). `x0` está cableado a **cero**
  (leerlo da 0, escribirlo no hace nada — un truco que simplifica el ISA).
- Un **contador de programa** (`pc`): la dirección de la instrucción
  actual.
- Unas **47 instrucciones**: sumar, restar, AND/OR/XOR, desplazar,
  comparar, cargar/guardar memoria, saltar.

Eso es *todo*. Con eso ya se puede correr cualquier programa (más lento
que con extensiones, pero completo — es Turing-completo).

### Los formatos de instrucción (la regularidad de RISC)

Toda instrucción RV32I son **32 bits**, y hay solo 6 formatos. Lo
genial: **los campos están casi siempre en el mismo lugar**, así que el
decodificador es trivial en hardware. Ejemplo del formato R (registro-
registro, como `add x1, x2, x3`):

```
 31        25 24    20 19    15 14  12 11     7 6       0
┌────────────┬────────┬────────┬──────┬────────┬─────────┐
│  funct7    │  rs2   │  rs1   │funct3│   rd   │ opcode  │
└────────────┴────────┴────────┴──────┴────────┴─────────┘
   7 bits      5 bits   5 bits  3 bits  5 bits   7 bits
```

- `opcode` (7 bits): la familia de la instrucción (OP, LOAD, BRANCH…).
- `rd` (5 bits): registro destino (5 bits porque 2⁵ = 32 registros).
- `rs1`, `rs2`: registros fuente.
- `funct3`, `funct7`: refinan la operación (ADD vs SUB vs XOR…).

En el código esto vive en [rv32i_defs.h](rv32i_defs.h): constantes para
cada opcode/funct, y funciones que extraen los campos:

```c
rv32i::opcode(iw)   // los bits [6:0]
rv32i::rd(iw)       // los bits [11:7]
rv32i::funct3(iw)   // los bits [14:12]
```

## 2. ¿Qué significa "hacer" un procesador?

Un procesador es una máquina que repite tres pasos para siempre:

```
   ┌──────────────────────────────────────────┐
   │  1. FETCH   – leer la instrucción en pc   │
   │  2. DECODE  – entender qué pide            │
   │  3. EXECUTE – hacerlo, actualizar pc       │
   └──────────────────────────────────────────┘
              ↑____________________│
```

"Hacer un procesador" es construir el hardware que ejecuta ese ciclo.
Hay capas, de lo abstracto a lo físico:

```
ISA (el contrato: RV32I)
  └─ Microarquitectura (CÓMO lo implemento: en orden, fuera de orden…)
       └─ RTL (Verilog/VHDL: los registros y la lógica, ciclo a ciclo)
            └─ Compuertas (AND/OR/flip-flops)
                 └─ Transistores / celdas de la FPGA
```

**La misma ISA se puede implementar de mil formas.** Un core simple
ejecuta una instrucción a la vez, en orden. Uno avanzado ejecuta varias
a la vez, fuera de orden, especulando. Ambos cumplen el mismo contrato
RV32I — un programa no nota la diferencia salvo en la velocidad. Este
proyecto implementa la versión avanzada (fuera de orden).

## 3. ¿Por qué HLS y no Verilog?

Normalmente el RTL se escribe en **Verilog/VHDL**. Este proyecto usa
**HLS (High-Level Synthesis)**: se escribe el hardware en **C++**, y la
herramienta (Vitis HLS de AMD) lo traduce a Verilog sintetizable para la
FPGA.

**La ventaja:** el mismo C++ se puede
- **simular como software** (rápido, con `printf`, con depurador), y
- **sintetizar como hardware** (para el chip real).

Un core fuera de orden tiene *mucha* lógica de control (el ROB, el
renombrado, las estaciones de reserva). Describir eso en Verilog es lento
y propenso a errores; en C++ es legible y se verifica en segundos.

**El costo:** no todo C++ vale. Nada de `new`/`delete`, ni recursión, ni
lazos de largo desconocido. Cada arreglo se vuelve una **memoria física**
(BRAM) o **registros**. Por eso el código usa:

- **Arreglos estáticos** (`static ap_uint<32> regfile[32]`) → registros
  que sobreviven entre ciclos.
- **`ap_uint<N>`**: entero de **exactamente N bits**. `ap_uint<3>` son 3
  cables reales, no 32. Los tags del ROB son `ap_uint<3>` porque el ROB
  tiene 8 entradas (2³ = 8).
- **`#pragma HLS`**: directivas para guiar a la herramienta (por ejemplo
  `UNROLL` = "desenrolla este lazo en hardware paralelo").

### El modelo de ejecución: un "tick" = un ciclo de reloj

La función principal es:

```c
void riscv_soc_tick(reset, imem, dmem, ...salidas..., halted);
```

**Cada llamada simula un ciclo de reloj.** El testbench la llama en un
lazo hasta que `halted` se activa. En hardware, esa función *es* la lógica
que se ejecuta en cada flanco de reloj. Todo el estado del procesador son
variables `static` → en el chip son registros.

---

# Parte II — Las extensiones I, M, F, C

RISC-V es modular: la base `I` más extensiones opcionales, cada una una
letra. Este proyecto implementa **RV32IMFC**:

| Letra | Qué agrega | Por qué |
|---|---|---|
| **I** | enteros base (la base obligatoria) | correr cualquier cosa |
| **M** | multiplicación y división | sin M, multiplicar cuesta ~32 sumas |
| **F** | punto flotante de 32 bits (float) | cálculo científico/señales |
| **C** | instrucciones comprimidas de 16 bits | código más chico (ahorra memoria) |

## 4. I — la base entera

- **ALU** (unidad aritmético-lógica): suma, resta, lógica, desplazamientos,
  comparaciones. En [backend.h](backend.h): `alu_compute()`.
- **Loads/stores**: leer y escribir memoria (byte, media palabra, palabra,
  con y sin signo). En [axi_interconnect.h](axi_interconnect.h):
  `dmem_load()`/`dmem_store()`.
- **Saltos**: condicionales (`beq`, `bne`, `blt`…) e incondicionales
  (`jal`, `jalr`). Son la parte difícil, porque cambian el `pc` y rompen
  el flujo — de ahí sale toda la complejidad de la Parte III y VI.

Este core tiene **2 ALUs** porque las instrucciones enteras son las más
comunes: tener dos permite ejecutar dos sumas independientes a la vez.

## 5. M — multiplicación y división

Van en una **unidad propia** (`md_rs` en [soc_state.h](soc_state.h),
`md_compute()` en [backend.h](backend.h)) con latencias distintas:
**mul = 3 ciclos, div = 8 ciclos**. Esto es exactamente lo que justifica
el fuera de orden: una división de 8 ciclos **no debe bloquear** a las
sumas que no dependen de ella.

## 6. F — punto flotante (IEEE-754)

El estándar IEEE-754 guarda un `float` como signo + exponente + mantisa.
La extensión F agrega:

- Un **banco de registros separado**: `fregs[32]` — 32 registros de punto
  flotante, distintos de los enteros. Guardan los bits IEEE crudos.
- Una **FPU** que hace add/sub/mul/div/sqrt, la familia **FMA**
  (multiplicar-y-sumar en una instrucción, con 3 operandos),
  comparaciones, conversiones int↔float, y manipulación de signo.
  Latencias: add/mul 3, div/sqrt 8, FMA 4, el resto 2.

Detalle fino: algunas instrucciones **cruzan bancos**. `FCVT.W.S`
convierte float→entero (lee F, escribe entero). `FMV.W.X` mueve bits de
entero a float. El decodificador ([frontend_dispatch.h](frontend_dispatch.h))
decide qué banco de renombrado usar según el caso.

## 7. C — instrucciones comprimidas

Muchas instrucciones comunes (cargar una constante chica, sumar, saltar
cerca) se pueden codificar en **16 bits** en vez de 32. Eso reduce el
tamaño del código ~30 %, que en un sistema embebido importa.

La idea de implementación es elegante: **la extensión C se paga una sola
vez, en el fetch**. El frontend lee 16 bits; si los 2 bits bajos ≠ `11`,
es comprimida: se **expande** a su equivalente de 32 bits con
`rv32c::expand()` ([rv32c_defs.h](rv32c_defs.h)) y el `pc` avanza 2 en vez
de 4. **El resto del pipeline solo ve instrucciones de 32 bits** — nada
más cambia.

---

# Parte III — Ejecución fuera de orden (el corazón)

Esta es la parte más importante del proyecto. Es lo que distingue un core
"de verdad" de un modelo de juguete.

## 8. El problema

En un pipeline **en orden**, si una instrucción lenta (una división de 8
ciclos) está adelante, **todo lo que viene detrás espera**, aunque no
dependa de ella:

```
div  x11, x3, x1     ← tarda 8 ciclos
add  x12, x2, x1     ← NO depende de la división, pero espera igual
```

**Fuera de orden**: las instrucciones independientes **adelantan** a las
lentas. El `add` se ejecuta mientras la división sigue en curso. Cuando
termina, ambas escriben sus resultados.

Pero hay una regla de oro: **el resultado visible debe ser como si todo
se hubiera ejecutado en orden**. Si el programa dice "primero div, luego
add", el mundo exterior debe ver ese orden. Ejecutar desordenado por
dentro, parecer ordenado por fuera. Eso lo resuelve el algoritmo de
**Tomasulo**.

## 9. Tomasulo: las piezas

Todas viven en [soc_state.h](soc_state.h):

| Pieza | Qué es | En este core |
|---|---|---|
| **ROB** (Reorder Buffer) | cola circular; cada instrucción reserva una entrada al despacharse y se **retira en orden de programa** | 8 entradas, tags de 3 bits |
| **RAT** (Register Alias Table) | por cada registro: ¿su valor está en el banco, o lo está produciendo una entrada del ROB? | `rat[32]` enteros, `frat[32]` flotantes |
| **Estaciones de reserva** (RS) | salas de espera por unidad: guardan la operación y sus operandos (el valor, o el *tag* de quién lo producirá) | 2 ALU, 1 MUL/DIV, 1 FPU, 1 LSU, 1 branch, 1 vectorial, 1 sistema |
| **CDB** (Common Data Bus) | cuando una unidad termina, difunde `(tag, valor)`; toda RS que esperaba ese tag lo captura | `cdb_broadcast()` en [backend.h](backend.h) |

## 10. La vida de una instrucción, paso a paso

Tomemos `add x12, x2, x1`:

**1. Dispatch** (etapa 4 del tick, [frontend_dispatch.h](frontend_dispatch.h)):
se decodifica, se reserva la entrada `rob_tail` del ROB, y se leen los
operandos con `read_operand()`. Aquí ocurre la magia — hay **tres casos**:

```c
static Operand read_operand(ap_uint<5> reg) {
    if (reg == 0) { ... return 0; }             // x0 siempre es 0
    if (!rat[reg].has_tag)                        // (a) nadie lo produce
        return { ready, regfile[reg] };           //     → valor del banco
    ap_uint<3> t = rat[reg].tag;                  //     lo produce el tag t
    if (rob[t].ready)                             // (b) ya terminó, sin retirar
        return { ready, rob[t].value };           //     → bypass del ROB
    else                                          // (c) todavía en vuelo
        return { not_ready, tag = t };            //     → esperar el CDB
}
```

Esos tres casos **son** Tomasulo: banco / adelanto desde el ROB / esperar
en el bus. Luego se **renombra** el destino: `rat[x12] = {tag = esta
entrada}`. Desde ahora, quien pida `x12` recibirá el tag, no el valor
viejo.

**2. Issue** (etapa 3): cuando la RS tiene *todos* sus operandos, arranca
la unidad. Cada una con su latencia (ALU 1, mul 3, div 8…).

**3. Ejecución + broadcast** (etapa 2): al terminar, el resultado va a la
entrada del ROB y se **difunde por el CDB**. Toda RS que esperaba ese tag
lo captura al vuelo — sin pasar por el banco de registros.

**4. Commit** (etapa 1): cuando la instrucción llega a la **cabeza** del
ROB y está lista, escribe el banco (o la memoria, si es store) y se
retira. **Aquí se recupera el orden de programa**: aunque el `add` haya
*terminado* antes que el `div`, se *retira* después, porque está detrás en
el ROB.

**La frase para la defensa:** *"El ROB desacopla ejecutar de completar:
ejecuto en el orden que los datos permiten, pero hago visible el estado en
orden de programa. Por eso las excepciones son precisas."*

## 11. Los stores esperan al commit

Un store **no escribe memoria al ejecutarse**, sino al **retirarse**
(guarda dirección y dato en su entrada del ROB). Así nunca escribe memoria
"de más" si algo se cancela. Coherente con el punto siguiente.

## 12. La decisión que define el proyecto (versión base)

En la carpeta base (`RV32IMFC+RVV+OOO-HLS`), el core **no especula**:
cuando el dispatch encuentra un salto, el fetch **se detiene** hasta que
se resuelve el destino. Costo: se pierden ciclos en cada salto. Ganancia
**enorme**: *toda instrucción despachada va a completarse*, nunca hay que
deshacer nada. Eso vuelve **baratísimas** las excepciones precisas (Parte
V) — un simple `pipeline_flush()`.

El SoC (esta carpeta) **sí especula**, con TAGE — pero reutiliza
exactamente ese mismo `pipeline_flush()` para recuperarse de un fallo de
predicción (Parte VI). La decisión de diseño se pagó tres veces:
excepciones, interrupciones y `vstart`.

---

# Parte IV — RVV: la extensión vectorial

## 13. ¿Qué es un procesador vectorial?

Un core normal (**escalar**) opera de a un dato: `add` suma dos números.
Un core **vectorial** opera sobre un *arreglo* de datos con una sola
instrucción: `vadd.vv` suma dos vectores de, digamos, 4 números de golpe.
Es el mismo principio que hace rápidas a las GPUs.

**RVV** (RISC-V Vector) es la extensión vectorial oficial. Su gran idea es
la **longitud de vector agnóstica** (*vector-length agnostic*): el mismo
binario corre en hardware con vectores chicos o grandes, sin recompilar.
El software pregunta "¿cuántos elementos por iteración puedo hacer?" y el
hardware responde.

### Los parámetros de esta implementación

- **VLEN = 128 bits**: cada registro vectorial tiene 128 bits.
- **32 registros vectoriales** (`v0`–`v31`).
- **SEW** (*Selected Element Width*): el ancho de cada elemento, 8/16/32
  bits. Con SEW=32 caben 4 elementos por registro (128/32); con SEW=8,
  16 elementos.
- **Perfil objetivo: Zve32x** — el perfil embebido más chico de RVV
  (enteros hasta 32 bits, sin flotante vectorial).

## 14. La idea que sostiene todo: el banco es plano

```c
static ap_uint<32> vregs[32 * 4];   // 32 registros × 4 palabras de 32b
```

El banco vectorial es **un arreglo plano de palabras**. El "ancho de
elemento" **no cambia el almacenamiento** — cambia **cómo se indexa**:

```c
// exec_vector.h
static ap_uint<32> vreg_get(ap_uint<32> vregs[], uint32_t vreg,
                            int idx, uint32_t sew_b) {
    int bit_off = idx * sew_b * 8;                          // ¿en qué bit?
    ap_uint<32> w = vregs[vreg * OOO_VEC_LANES + (bit_off/32)]; // palabra
    ap_uint<32> raw = w >> (bit_off % 32);                  // desplazar
    if (sew_b == 1) return raw & 0xFF;                      // enmascarar
    if (sew_b == 2) return raw & 0xFFFF;
    return raw;
}
```

**Cinco líneas** que sostienen SEW variable, EEW variable, y (con
`vreg_get_pair`, que suma `idx/por_reg` al número de registro) los grupos
de dos registros del widening. El almacenamiento nunca cambia; el índice
sí. Esta única idea se reutiliza en tres fases distintas.

## 15. `vsetvli` — cómo se programa RVV

RVV se usa así: *"tengo N elementos que procesar; dime cuántos hago por
iteración"*. La instrucción `vsetvli`:

- fija `vtype` (SEW, LMUL, políticas),
- calcula `vl = min(AVL, VLMAX)` donde `VLMAX = VLEN/SEW`,
- devuelve `vl`.

El software hace **strip-mining**: un lazo que procesa `vl` elementos por
vuelta y avanza los punteros, hasta agotar el arreglo. El AXPY de los
resultados es exactamente esto.

**Decisión de diseño**: `vsetvli` se resuelve **en el dispatch** (como
`LUI`/`JAL`), porque el dispatch es en orden. Así toda vectorial
despachada después captura el `vl` nuevo sin serializar la unidad. Cada
vectorial captura su `vl`/`vstart`/`sew` al despacharse.

## 16. Las 6 fases de RVV (cómo se construyó por partes)

El proyecto implementó RVV incrementalmente, cada fase con su suite de
tests:

| Fase | Qué entró |
|---|---|
| **1-2** | `vsetvli`, aritmética entera (`vadd`/`vsub`/`vmul`/`vdiv`/lógicas/shifts), formas `.vv`/`.vx`/`.vi`, EEW variable en memoria |
| **3** | Comparaciones a máscara (`vmseq`…), `vmerge`/`vmv`, lógica de máscaras, `vcpop`/`vfirst`/`viota`/`vid`, predicación por `v0` |
| **4** | Reducciones (`vredsum`…), permutaciones (`vslideup`/`vrgather`/`vcompress`), punto fijo saturante (`vsaddu`) y promediado (`vaadd`) |
| **4d** | Widening (`vwadd`/`vwmul`/`vwmacc`) y narrowing (`vnsrl`/`vnclip`) — el resultado ocupa un **par** de registros (EMUL=2) |
| **5** | Modos de memoria: strided, indexado (gather/scatter), segmentado, fault-only-first, registro completo, máscara |
| **6** | `vstart`: reanudar una vectorial a mitad de camino tras un trap |

### Cómo se ejecuta una vectorial (por categoría)

El decodificador ([vector_dispatch.h](vector_dispatch.h)) clasifica cada
instrucción en una **categoría** (VCAT_*) que dice qué destino escribe.
`vec_arith_compute()` en [exec_vector.h](exec_vector.h) despacha por
categoría. El lazo sobre los elementos usa `#pragma HLS UNROLL`: los 16
elementos (peor caso) se procesan **en paralelo** en hardware, como
*lanes* de verdad:

```c
for (int e = 0; e < rvv::MAX_ELEMS; e++) {
#pragma HLS UNROLL
    bool go = (e >= vstart) && (e < vl) && vec_elem_active(u, e, vregs);
    if (go) {
        ap_uint<32> a = vreg_get(vregs, u.vs2, e, sb);
        ap_uint<32> b = scalar_form ? u.s1.val : vreg_get(vregs, u.vs1, e, sb);
        vreg_set(vregs, u.vd, e, sb, vec_alu(u.funct3, u.funct6, a, b, sb));
    }
}
```

Cuatro condiciones por elemento, **cada una de una fase distinta**: `e >=
vstart` (fase 6), `e < vl` (fase 1), la máscara (fase 3), el ancho `sb`
(fase 2). Ese fragmento resume el proyecto entero en cuatro líneas.

---

# Parte V — Bare-metal: correr software de verdad

**Bare-metal** = correr programas compilados con gcc **sin sistema
operativo**, hablándole directo al hardware. El core ejecuta binarios ELF
reales: el `crt0` configura el stack, `main()` corre, y `printf` de la
biblioteca C real (newlib) sale por un UART. Demostrado en las suites B/C/D.

Las piezas (en [soc_csr.h](soc_csr.h) y el tick):

- **Carga de ELF**: el testbench parsea el binario y copia cada segmento a
  memoria. Como HLS es **Harvard** (memoria de instrucciones y datos
  separadas), cada segmento se copia a ambas.
- **CSRs** (*Control and Status Registers*): `mstatus`, `mtvec`, `mepc`,
  `mcause`, `mie`, `mip`, `mtime`… los registros de control de modo
  máquina. Las instrucciones CSR ejecutan solo en la cabeza del ROB (orden
  de programa).
- **Excepciones precisas**: ECALL/EBREAK/ilegal marcan su entrada del ROB.
  Al llegar al commit: se guarda el `pc` en `mepc`, la causa en `mcause`,
  se salta a `mtvec`, y **`pipeline_flush()`**. Como nada posterior tocó
  el estado, es *precisa*. El handler retorna con `MRET` y el programa
  sigue → *reanudable*.
- **Interrupciones de timer**: `mtime` avanza cada ciclo; al vencer,
  levanta una interrupción tomada por el mismo camino de trap.
- **Modos M/U**: máquina y usuario. Un ECALL desde U reporta causa 8;
  desde M, causa 11 — la prueba de que los privilegios funcionan.

**La frase para la defensa:** *"No especular hizo el bare-metal casi
gratis: como todo lo despachado se completa, un trap preciso es solo un
flush en el commit. Las interrupciones reutilizan el mismo camino."*

---

# Parte VI — El SoC: los tres bloques del diagrama

La carpeta base ya era un RV32IMFC + OoO + RVV + bare-metal completo. El
**SoC** (esta carpeta) le agrega los tres bloques que faltaban para
coincidir con el diagrama de arquitectura: **TAGE**, **coprocesador
desacoplado**, e **interconnect con árbitro**.

## 17. TAGE — el predictor de saltos

Sin predicción, el fetch se detiene en cada salto. **TAGE** (*TAgged
GEometric*) predice si un branch se toma o no, y el fetch **sigue** por el
camino predicho. En [tage.h](tage.h):

- un **bimodal** base (512 contadores de 2 bits, indexado por `pc`),
- **3 tablas etiquetadas** (128 entradas), cada una indexada con una
  longitud distinta de *historia global* (4/8/16 bits, geométricas),
- gana la tabla con match de historia **más larga**.

La integración es lo bonito: se **predice** en el dispatch; la unidad de
branch **comprueba** la predicción; la resolución es **en el commit** — si
falló, se redirige el fetch y se hace `pipeline_flush()`, exactamente el
mecanismo de los traps. **Un mispredict es "un trap barato".** El
predictor **entrena en el commit** (con la verdad, en orden). Resultado
medido: 90.6 % de acierto en un patrón que un bimodal falla al 50 %.

## 18. El coprocesador vectorial desacoplado

En la base, la unidad vectorial era una estación más dentro del core. En
el SoC ([vector_coprocessor.h](vector_coprocessor.h)) es un **coprocesador
de verdad**:

- una **Vector Instruction Queue** (FIFO de 4): el core escalar
  **encola** la instrucción vectorial y **sigue** — hasta 4 en vuelo
  mientras el escalar avanza;
- **cuatro unidades** con latencias propias: VALU (2), VMUL (4), VSLDU (2)
  permutaciones, VLSU memoria;
- el ROB común mantiene el retiro en orden y los traps precisos.

## 19. El interconnect con árbitro

En la base, la memoria vectorial hacía 16 accesos "mágicos" en un ciclo
—imposible en un puerto real—. El SoC ([axi_interconnect.h](axi_interconnect.h))
modela un **árbitro**: **una concesión de memoria por ciclo**, con
prioridad `commit > VLSU > LSU`. La VLSU pasa a **un elemento por ciclo**.
El control del SoC es **AXI4-Lite real** (`s_axilite`), por donde el PS
del Kria arranca el core.

Esto hizo los números *realistas*: el speedup vectorial del AXPY bajó de
2.66× (memoria ideal) a 1.66× (memoria arbitrada) — y comparar ambos es
análisis del bueno (ver [RESULTADOS.md](../RESULTADOS.md)).

---

# Parte VII — Cómo se construyó todo, y cómo se verificó

## 20. La cronología (el orden importa)

1. **Core en orden** — el ISA escalar correcto sin la complejidad del OoO.
   *Método: nunca depurar ISA y microarquitectura a la vez.*
2. **Esqueleto Tomasulo, solo enteros** — aquí se tomaron las decisiones
   estructurales (ROB de 8, no especular, stores al commit) que no
   cambiaron más.
3. **F y C encima** — segundo banco pero el mismo ROB; C resuelto en el
   fetch.
4. **Bare-metal ANTES que RVV** — el camino de trap que se construyó aquí
   se reutilizó después para las ilegales de RVV y para `vstart`. El orden
   inverso habría duplicado trabajo.
5. **RVV por fases (1→6)** — cada fase con su suite de tests y su csynth al
   cerrar; por eso los bugs se cazaron fase a fase.
6. **La reestructuración** — con todo verde, se partió el archivo de 2364
   líneas en headers. *Un refactor mecánico solo es seguro con una red de
   tests completa debajo.*
7. **El SoC** — TAGE + coprocesador + árbitro sobre la base estable, sin
   tocar el ISA (los 214 checks de la base siguen pasando).

## 21. La verificación (por qué se puede confiar)

- **8 suites, 218/219 checks** sin *backdoors*: el estado se verifica
  desde lo que el programa deja en memoria o desde las salidas de
  observación del core.
- **Los checks distinguen**: cada uno falla si el comportamiento correcto
  y el incorrecto darían resultados distintos (saturar vs envolver, causa
  8 vs 11, elemento intacto vs pisado).
- **csim + csynth**: la C-simulación verifica la función; la C-síntesis
  produce el RTL y confirma frecuencia/recursos/timing.
- **Paridad HLS ↔ TLM**: las dos pistas dan los mismos ciclos; convertido
  en regresión ejecutable (`make check-axpy-parity`). Esta paridad cazó
  dos bugs reales del SoC.

## 22. Las historias de bugs (oro para la defensa)

Contar un bug bien entendido vale más que una feature. Los reales:

1. **La familia de hazards CSR** (cuatro, una sola causa): estado que se
   *lee en una etapa* del pipeline y se *escribe en otra*. `vsetvli`
   escribe `vl` en el dispatch, `csrr` lo lee en la cabeza del ROB → un
   `vsetvli` posterior pisaba `vl` antes de tiempo. La cura (esperar a que
   no haya CSR en vuelo) elimina la **clase** entera, no el caso.
2. **Overflow de ancho silencioso**: `remaining` en 2 bits no aguantaba
   `VMUL_LAT=4` (se envolvía a 0). El multiplicador vectorial completaba en
   1 ciclo. Lo cazó la **paridad con TLM** (que usa un tipo más ancho).
3. **Gate de `vset*` perdido**: un parche a medias dejó a `vsetvli`
   despachándose especulativamente bajo un branch — un fallo de
   *corrección*. También lo delató la paridad de ciclos.
4. **Permutaciones fuera de rango**: leían 16 elementos sin mirar SEW; con
   SEW=32 solo hay 4 → se salían del registro y pisaban el vecino.

## 23. Estructura de archivos (una unidad de traducción)

Todo el estado es `static`; partirlo en varios `.cpp` le daría a cada uno
su propia copia del ROB. Por eso es **una sola unidad de traducción**: el
`.cpp` incluye los headers.

```
soc_top.cpp             las 4 etapas del pipeline (el tick)
├── soc_config.h        latencias, ROB_SZ, tamaños
├── rvv_encoding.h      codificación RVV verificada contra la spec
├── soc_state.h         structs de RS/ROB + registros (static) + VIQ
├── soc_csr.h           banco de CSRs
├── backend.h           CDB, operandos, ALU, mul/div, FPU, branch
├── exec_vector.h       banco vectorial + ALU vectorial (incl. EMUL=2)
├── axi_interconnect.h  memoria de datos + árbitro
├── tage.h              el predictor de saltos
├── vector_coprocessor.h VIQ + VALU/VMUL/VSLDU/VLSU
├── frontend_dispatch.h decodificador escalar (I/M/F/C/SYSTEM)
└── vector_dispatch.h   decodificador RVV (opcode OP-V)
```

## 24. Limitaciones (dilas tú antes de que te las pregunten)

- **Zve32x incompleto en un punto**: falta `LMUL≠1` (agrupar registros).
  Todo lo demás de las 6 fases está.
- **No es conformidad certificada**: eso exige pasar los RISC-V
  Architecture Compatibility Tests. La frase correcta es *"subconjunto de
  RVV 1.0 orientado a Zve32x, con codificación y semántica verificadas
  contra la especificación oficial"*.
- **Fuera de alcance a propósito**: `SEW=64` y flotante vectorial (serían
  Zve64/Zve32f), MMU/modo S (solo para un SO), `vxrm` fijo en `rnu`.
- **SoC**: JALR no se especula (falta BTB/RAS), el coprocesador no ejecuta
  especulativamente (vregs sin renombrar), el dato va por BRAM arbitrada
  (el AXI del top es el control AXI4-Lite).

## 25. En una frase

> *Un RISC-V RV32IMFC con extensión vectorial RVV y ejecución fuera de
> orden tipo Tomasulo, escrito en C++ sintetizable para FPGA, que corre
> software bare-metal real (binarios de gcc con printf de newlib), con un
> frontend especulativo TAGE, un coprocesador vectorial desacoplado y un
> interconnect AXI con árbitro; sintetiza a 135.86 MHz en la Kria KV260 y
> está verificado con 218 checks en 7+1 suites, con paridad ciclo a ciclo
> demostrada contra un modelo TLM independiente.*

---

*Documento de referencia de la pista HLS. Para los números medidos, ver
[RESULTADOS.md](../RESULTADOS.md). Para el detalle del diagrama →
archivo, ver [README.md](README.md).*
