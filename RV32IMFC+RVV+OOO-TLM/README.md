# RV32IMFC + RVV + OOO — pista TLM (línea de trabajo hacia Zve32x)

Copia **autocontenida** del core RV32IMFC fuera de orden con
coprocesamiento vectorial en SystemC/TLM-2.0, extraída de
`RV32IMFC_tlm/` para trabajar sobre ella sin poner en riesgo la versión
ya verificada del entregable.

**Punto de partida** (estado al copiar, verificado en esta carpeta:
61 checks OK, 0 fallos):
- `ProcessorOOO`: RV32IMFC completo con ejecución fuera de orden
  (RAT+FRAT → ROB unificado de 8, 2×ALU, MUL/DIV, FPU, LSU, branch),
  como initiator TLM-2.0 puro sobre la topología `Bus`/`Memory`.
- RVV integrado (unidad `VecRs`): `vsetvli`/`vsetivli`/`vsetvl` con `vl`
  dinámico y *tail-undisturbed*, `vle32.v`/`vse32.v`,
  `vadd.vv`/`vsub.vv`/`vmul.vv`. La memoria vectorial pasa por el Bus
  real (`b_transport`), respetando la convención TLM del proyecto.
- Verificación cruzada con la pista HLS: el mismo programa produce el
  mismo estado arquitectónico y los mismos números de ciclo.

## Objetivo de esta línea: `Zve32x`

Mismo objetivo y misma tabla de requisitos que la pista HLS — ver
[`../RV32IMFC+RVV+OOO-HLS/README.md`](../RV32IMFC+RVV+OOO-HLS/README.md).
La idea es mantener **paridad** entre ambas pistas: lo que se agregue
acá se agrega allá y se verifica que sigan coincidiendo, que es lo que
da confianza en que la microarquitectura está bien especificada.

Ventaja de prototipar acá primero: no hay costo de síntesis ni tiempos
de `csynth`, el ciclo de edición-compilación-prueba es de segundos, y
la memoria es de bytes (no de palabras AXI), lo que hace más simple
experimentar con anchos de elemento (EEW 8/16/32).

## Estructura

Un solo archivo por cosa — sin versiones duplicadas:

| Archivo | Qué es |
|---|---|
| `src/processor_ooo.h` | **El core** |
| `src/main.cpp` | **Testbench único**: 4 suites en secuencia |
| `src/bus.h`, `memory.h`, `uart.h`, `memory_map.h` | Topología TLM (Bus + RAM + UART) |
| `src/rv32i_defs.h`, `immediates.h`, `fp_ops.h`, `rv32c_defs.h` | Dependencias del ISA |
| `src/trap_elf.h`, `full_elf.h`, `printf_elf.h` | Los **mismos** binarios que la pista HLS |
| `src/vector_unit.h` | Esqueleto RVV previo (no se usa en el pipeline) |

### Las 4 suites del testbench

| Suite | Qué verifica |
|---|---|
| **A. ISA + RVV** | I+M+F+C y RVV Fases 1-3, con evidencia de OOO y coprocesamiento |
| **B. Excepciones** | ELF real: handler → `MRET` → **el programa continúa** |
| **C. Sistema** | ELF real: UART (periférico TLM), timer, modos M/U |
| **D. `printf`** | ELF real con **newlib** (~54 KB) |

## Cómo correr

Requiere SystemC 2.3.4 (`libsystemc-dev`):

```bash
g++ -std=c++17 -o riscv_ooo_sim src/main.cpp -lsystemc -lpthread
LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu ./riscv_ooo_sim
```

Los binarios ELF se regeneran desde
[`../RV32IMFC+RVV+OOO-HLS/baremetal/`](../RV32IMFC+RVV+OOO-HLS/baremetal/)
con `sh build.sh`, y se copian a `src/` (las dos pistas son
autocontenidas, misma convención que el resto del proyecto).

---

# ESTADO ACTUAL: Fases 1-6 completas (hacia Zve32x)

Última actualización tras completar las **Fases 4d y 6**. Ambas pistas (HLS y
TLM) están en paridad y verificadas.

## Resumen de verificación

| | TLM | HLS |
|---|---|---|
| Checks | **215 OK / 0 FAIL** (7 suites) | **214 OK / 0 FAIL** (7 suites) |
| Bare-metal (ELF real de gcc) | — | ✅ pasa |
| csim en Vitis | — | ✅ pasa |
| Síntesis | — | ✅ **128.25 MHz**, *loop constraints* satisfechos |
| Recursos (estimado HLS) | — | 58083 LUT (49 %), 16565 FF, 42 DSP, 6 BRAM |

El Fmax bajó de 135.86 a **128.25 MHz** a lo largo de las fases (~5 %),
mientras el área subió a 49 % de LUTs. El camino crítico lo sigue
dominando la FPU escalar, no la unidad vectorial: lo que creció con RVV
es el área, no el retardo.

## ✅ Fase 1 — familias de operaciones, formas escalares y máscara

- **Máscara real**: bit `vm`; con `vm=0` la operación queda predicada por
  `v0` (política *mask-undisturbed*: los elementos inactivos no se tocan).
- **13 ops OPIV\***: `vadd` `vsub` `vrsub` `vminu` `vmin` `vmaxu` `vmax`
  `vand` `vor` `vxor` `vsll` `vsrl` `vsra`.
- **8 ops OPMV\***: `vmul` `vmulh` `vmulhu` `vmulhsu` `vdivu` `vdiv`
  `vremu` `vrem` (con los casos borde del ISA: división por cero y
  overflow `INT_MIN/-1`).
- **Formas vector-escalar**: `.vx` (desde `x[rs1]`, esperando por el CDB
  si el productor sigue en vuelo) y `.vi` (inmediato de 5 bits con signo).

## ✅ Fase 2 — ancho de elemento variable (EEW 8/16/32)

Es el requisito que bloqueaba `Zve32x`. Decisión de diseño que lo hizo
acotado: **el almacenamiento no cambió** (un registro sigue siendo 4
palabras de 32 bits = 128 bits); lo que cambió es cómo se *indexan* los
elementos dentro de esos bits (`vreg_get`/`vreg_set`). Como 8/16/32
dividen exacto a 32, ningún elemento cruza el límite de palabra.

- `vsetvli e8` → `VLMAX = 128/8 = 16`; `e16` → 8; `e32` → 4.
- `vle8.v`/`vle16.v`/`vle32.v` según el campo `width` (el EEW del acceso
  lo fija la instrucción, no `vtype`).
- Aritmética con SEW correcto: extensión de signo del ancho real, shift
  acotado a `log2(SEW)`, parte alta de `vmulh*` desde `sew_bits`, casos
  borde de división con el `INT_MIN` de cada ancho.

## ✅ Fase 3 — comparaciones, merge, lógica de máscaras y grupos unary

- **Comparaciones que escriben máscaras**: `vmseq` `vmsne` `vmsltu`
  `vmslt` `vmsleu` `vmsle` `vmsgtu` `vmsgt` (con las restricciones de
  forma del ISA: `vmsgt*` solo `.vx`/`.vi`, `vmslt*` sin `.vi`).
- **`vmerge` / `vmv.v.*`**: mismo `funct6`, distinguidos por `vm` — con
  `vm=0` el bit de `v0` *selecciona la fuente* en vez de inhibir la
  escritura.
- **Lógica entre máscaras**: `vmand` `vmor` `vmxor` `vmandnot` `vmornot`
  `vmnand` `vmnor` `vmxnor`.
- **Grupos unary** (seleccionados por el campo `vs1`): `vid.v`,
  `viota.m`, `vcpop.m` y `vfirst.m` — los dos últimos escriben un
  **registro entero**, así que su resultado viaja por el ROB y el CDB
  como el de cualquier otra unidad.


## ✅ Fase 4 — reducciones, permutaciones y punto fijo

- **Reducciones** (`.vs`, OPMVV): `vredsum` `vredand` `vredor` `vredxor`
  `vredminu` `vredmin` `vredmaxu` `vredmax`. El acumulador inicial sale de
  `vs1[0]` y el resultado va **solo a `vd[0]`**; los elementos inactivos
  por máscara no participan.
- **Permutaciones**: `vslideup`/`vslidedown` (`.vx`/`.vi`),
  `vslide1up`/`vslide1down` (`.vx`), `vrgather` y `vcompress`. Se lee el
  registro fuente **entero antes de escribir**, porque `vslideup` y
  `vcompress` pueden tener `vd` y `vs2` solapados y escribir en el lugar
  corrompería elementos que aún falta leer.
- **Punto fijo**: saturantes `vsaddu` `vsadd` `vssubu` `vssub` (se pegan
  al extremo representable del ancho SEW en vez de envolver) y
  promediados `vaaddu` `vaadd` `vasubu` `vasub` (con redondeo `rnu`,
  calculados en 64 bits para no perder el bit que se va en el
  desplazamiento).

**Un bug real que encontró esta fase**: el código de permutaciones leía
los 16 elementos posibles sin importar el SEW — con `SEW=32` solo hay 4
por registro, así que leer el elemento 15 **se salía del registro y
pisaba el vecino** (el banco es un arreglo plano). Ahora el número de
elementos se calcula como `VLEN/SEW`.

**Y la excepción de instrucción ilegal se ganó el sueldo**: al portar,
una instrucción quedó fuera del decodificador por un parche mal aplicado.
En vez de dar resultados silenciosamente incorrectos, el programa
**se detuvo con un trap visible** — exactamente para lo que se agregó.

**Falta de la Fase 4**: widening/narrowing (`vwadd`/`vwmul`/`vnsrl`/
`vnclip`), que es lo único estructural que queda porque el destino ocupa
`EMUL=2` (pares de registros).


## ✅ Fase 5 — los modos de direccionamiento de memoria

Hasta aquí los load/store vectoriales solo sabían leer elementos
**contiguos**. La Fase 5 agrega los cuatro modos de la sección 7 de la
spec, más las variantes del unit-stride:

| Instrucción | Qué hace | Para qué sirve |
|---|---|---|
| `vlse32.v` / `vsse32.v` | **strided**: paso arbitrario en bytes desde `x[rs2]` | recorrer una columna de una matriz, o un campo de un arreglo de structs |
| `vluxei32.v` / `vsuxei32.v` (y las `vlox`/`vsox` ordenadas) | **indexado**: los offsets salen de un registro **vectorial** | gather/scatter — indexación indirecta, tablas de dispersión |
| `vlseg<nf>e32.v` | **segmentado**: `nf` campos por elemento a `nf` registros | desintercalar un arreglo de structs en vectores separados |
| `vle32ff.v` | **fault-only-first** | recorrer un string terminado en NUL sin arriesgar leer de más |
| `vl1r.v` / `vs1r.v` | **registro completo**, ignora `vl` y `vtype` | cambio de contexto y spills del compilador |
| `vlm.v` / `vsm.v` | mueve la **máscara** como bytes (`ceil(vl/8)`) | guardar y restaurar `v0` sin reconfigurar `vl` |

Los tres modos de direccionamiento comparten **un solo generador de
direcciones**; lo único que cambia es de dónde sale el desplazamiento:

```
unit-stride : base + (e*nf + f)*eew      elementos contiguos
strided     : base + e*paso  + f*eew     paso en BYTES de x[rs2]
indexado    : base + vs2[e]  + f*eew     offsets desde un vector
```

**El indexado es la única forma que mezcla dos anchos**: `width` codifica
el ancho del **índice**, y el dato usa el `SEW` de `vtype` (sección 7.3).
Confundirlos es el error fácil de esta fase.

**Segmentos: un campo por ciclo.** Un `vlseg3e32.v` con `vl=4` mueve 12
elementos. Hacerlos todos en un ciclo multiplicaría por `nf` los puertos
de memoria del RTL; iterar sobre los campos deja el costo en 16 accesos
por ciclo (igual que un load simple) y modela mejor lo que hace el
hardware real. Ambas pistas lo hacen igual, así que la paridad ciclo a
ciclo se mantiene.

**`vle32ff.v` recorta `vl` en vez de atrapar.** Si falla un elemento
posterior al 0, no hay trap: `vl` se recorta a los elementos que sí se
pudieron leer y el software reintenta desde ahí. Solo el elemento 0
atrapa (causa 5, *load access fault*), por el mismo camino de trap
preciso que ya usaba la instrucción ilegal. En este core el único
"fallo" posible es salirse del rango físico de memoria — no hay MMU ni
PMP —, así que esa es la condición que se comprueba.

**`EEW=64` ahora es instrucción ilegal**, igual que `mew=1` y los
`lumop`/`sumop` reservados. `Zve32x` llega hasta 32 bits, y la spec pide
que un ancho no soportado atrape en vez de ejecutarse mal en silencio:
es lo que le permite a un runtime detectar el perfil real de la máquina.

### Un bug de verdad que destapó esta fase

`vsetvli` resuelve **en el dispatch** (para que toda vectorial despachada
después vea el `vl` nuevo sin serializar la unidad VEC), pero `csrr x, vl`
lee el banco de CSRs **en la cabeza del ROB**. Con esas dos etapas
distintas, un `vsetvli` *posterior* en el programa alcanzaba a pisar `vl`
**antes** de que un `csrr` *anterior* lo leyera — un WAR entre dos etapas
del pipeline. Se detectó justamente al leer el `vl` recortado por el
fault-only-first, que es el primer caso del proyecto donde el software
necesita leer `vl` explícitamente.

El arreglo es detener el dispatch de `vset*` mientras haya una
instrucción CSR en vuelo. Cierra exactamente esa ventana y no cuesta
nada en la práctica, porque leer `vl` a mano es raro.


## ✅ Fase 4d — widening / narrowing (grupos `EMUL=2`)

Es el primer sitio donde **una instrucción toca dos registros vectoriales
a la vez**. Un resultado de `2*SEW` no cabe en un registro, así que ocupa
el par `(vd, vd+1)`:

| | Qué hace |
|---|---|
| `vwaddu` `vwadd` `vwsubu` `vwsub` (`.vv`/`.vx`) | `2*SEW = SEW ± SEW` |
| `vwaddu.w` `vwadd.w` `vwsubu.w` `vwsub.w` | `2*SEW = 2*SEW ± SEW` (el primer operando **ya viene ancho**) |
| `vwmulu` `vwmul` `vwmulsu` | producto sin perder los bits altos |
| `vwmaccu` `vwmacc` `vwmaccsu` `vwmaccus` | multiply-accumulate **sobre** `vd` |
| `vnsrl` `vnsra` (`.wv`/`.wx`/`.wi`) | `SEW = 2*SEW >> SEW`, truncando |
| `vnclipu` `vnclip` | igual pero **redondeando y saturando** al rango `SEW` |

**El almacenamiento no cambió.** El banco sigue siendo el mismo arreglo
plano; solo cambia cómo se indexa: con elementos de `wide_b` bytes caben
`VLEN/(wide_b*8)` por registro, así que el elemento `idx` vive en
`vd + idx/por_reg`. Es exactamente la misma idea que se usó para el EEW
variable en la Fase 2 — dos accesores nuevos de seis líneas
(`vreg_get_pair`/`vreg_set_pair`) y nada más.

**`Zve32x` acota esto fuerte**: `ELEN=32`, así que ensanchar solo es legal
desde `SEW ≤ 16`. Ensanchar desde `SEW=32` pediría 64 bits → **instrucción
ilegal**, igual que `EEW=64`. También se exige que el grupo de dos
registros empiece en un registro **par**.

Un caso que la prueba distingue bien: `vwaddu.vx` y `vwadd.vx` con el
*mismo* escalar `0xFFFFFFFF` dan resultados distintos (`0x10000` vs `0`),
porque uno lo toma como `0xFFFF` sin signo y el otro como `-1`. Si la
extensión de signo estuviera mal, las dos darían lo mismo.

## ✅ Fase 6 — `vstart`

`vstart` (CSR `0x008`, read-write) dice **por cuál elemento empieza** una
instrucción vectorial. Los anteriores quedan sin tocar, y al **completar**
se resetea a 0 — incluidos los `vset{i}vl{i}`, como pide la sección 3.7.
Si en cambio la instrucción **atrapa**, `vstart` queda apuntando al
elemento que falló, y por eso la vectorial es reanudable: el handler
retorna con `MRET` y la instrucción sigue donde se quedó.

**Reducciones, permutaciones, lógica de máscaras y los grupos unary
levantan instrucción ilegal si `vstart≠0`.** No es un atajo: este core las
ejecuta de forma **atómica**, así que nunca puede *producir* un `vstart`
intermedio en ellas, y la spec lo permite explícitamente —

> *"Implementations are permitted to raise illegal instruction exceptions
> when attempting to execute a vector instruction with a value of `vstart`
> that the implementation can never produce when executing that same
> instruction with the same `vtype` setting."* (sección 3.7)

### El tercer bug de la misma familia

Ya van tres, y todos son la misma forma: **estado arquitectónico que se
lee en una etapa del pipeline y se escribe en otra.**

1. Fase 5: `vsetvli` escribe `vl` en el *dispatch*, `csrr` lo lee en la
   *cabeza del ROB* → un `vsetvli` posterior pisaba `vl` antes de que un
   `csrr` anterior lo leyera (WAR).
2. Fase 6: `csrw vstart` escribe en la *cabeza del ROB*, la vectorial
   siguiente lee `vstart` en el *dispatch* → leía el valor viejo (RAW).

La cura es la misma en los dos sentidos: el dispatch de `vset*` espera a
que no haya CSR en vuelo, y el dispatch de cualquier vectorial también.
Cuesta nada porque tocar estos CSR a mano es raro, y elimina la clase
entera de bugs en vez de un caso.

## ✅ Bare-metal COMPLETO — paridad con la pista HLS

Esta pista ejecuta **los mismos binarios ELF** que la de HLS (mismo UART
en `0x20000000`), así que correrlos en ambas y obtener el mismo resultado
—y los **mismos números de ciclo**— es la verificación cruzada TLM↔HLS.

| Componente | Estado |
|---|---|
| Loader de ELF32 | ✅ carga `PT_LOAD` en la RAM (memoria **unificada**: no hace falta duplicar como en HLS) |
| Opcode `SYSTEM`, `FENCE` | ✅ |
| Instrucciones y banco de CSRs | ✅ unidad `SysRs` que ejecuta en la cabeza del ROB |
| Excepciones precisas y reanudables | ✅ `ECALL`/`EBREAK` → `mtvec` → handler → `MRET` → continúa |
| Interrupciones de timer | ✅ `mtime`/`mtimecmp`, `mie.MTIE`, `mstatus.MIE` |
| Modos de privilegio M/U | ✅ `mstatus.MPP`/`MPIE`; causa 8 desde U-mode, 11 desde M |
| Excepción de instrucción ilegal | ✅ causa 2 en vez de *no-op silencioso* |
| **UART** | ✅ **periférico TLM real colgado del Bus** (ver abajo) |
| `printf` de la biblioteca C | ✅ binario con newlib (~54 KB) |

### Una ventaja de esta pista: el UART es un periférico de verdad

En HLS el UART es un caso especial dentro del camino de store del core.
Acá es un **módulo TLM-2.0 target** (`uart.h`) colgado del Bus: el
procesador solo emite un store a `0x20000000` y es el **Bus** quien
decodifica la dirección y rutea la transacción, sin que el core sepa que
del otro lado hay algo distinto de una memoria. Eso es exactamente para
lo que se diseñó el Bus con mapa de direcciones, y hace la topología más
fiel a un SoC real:

```
ProcessorOOO ──┐
               ├──> Bus ──┬──> Memory (RAM 64KB)
VectorUnit ────┘          └──> Uart   (0x20000000)
```

### Detalle que apareció al portar

`full.c` escribía sus resultados en `0xA0`-`0xA8`, **dentro de su propia
imagen** (el binario son 214 bytes). En HLS eso no molestaba porque
`imem` y `dmem` están separadas; acá la memoria es unificada y el
programa **se auto-modificaba**. Se movieron los resultados a `0x400` y
ahora el mismo binario sirve en las dos pistas — un ejemplo concreto de
por qué mantener ambas pistas en paridad encuentra problemas reales.

## ⬜ Fases pendientes hacia `Zve32x`

| Fase | Contenido | Esfuerzo estimado |
|---|---|---|
| **4** | Solo queda **widening/narrowing** (`vwadd`/`vwmul`/`vnsrl`/`vnclip`): el destino ocupa `EMUL=2`, o sea pares de registros | ~1 día |
| **6** | `vstart` (reinicio de una vectorial a mitad de camino). Las **traps precisas** que este ítem también exigía ya están en **ambas** pistas | ~1 semana |

**Otras limitaciones que siguen abiertas**: `LMUL≠1` (agrupación de
registros) y `SEW=64`; sin punto flotante vectorial (no lo exige
`Zve32x`, sí `Zve32f`).

## Nota de honestidad sobre la conformidad

Esto **no es todavía una implementación conforme de `Zve32x`**. La
conformidad se declara solo tras pasar los RISC-V Architecture
Compatibility Tests, y además falta `LMUL≠1`. Lo correcto al
describir este trabajo es *"subconjunto de RVV 1.0 orientado a Zve32x,
con codificación y semántica verificadas contra la especificación
oficial"*.
