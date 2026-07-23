# RV32IMFC + RVV + OOO — pista HLS (línea de trabajo hacia Zve32x)

Copia **autocontenida** del core RV32IMFC fuera de orden con
coprocesamiento vectorial, extraída de `RV32IMFC_hls/` para trabajar
sobre ella sin poner en riesgo la versión ya verificada y sintetizada
del entregable.

**Punto de partida** (estado al copiar, verificado en esta carpeta):
- RV32IMFC completo (I+M+F+C) con ejecución fuera de orden
  (RAT+FRAT → ROB unificado de 8, 2×ALU, MUL/DIV, FPU, LSU, branch).
- RVV integrado como unidad `VecRs`: `vsetvli`/`vsetivli`/`vsetvl` con
  `vl` dinámico y *tail-undisturbed*, `vle32.v`/`vse32.v`,
  `vadd.vv`/`vsub.vv`/`vmul.vv`.
- Bare-metal: loader de ELF32, opcode `SYSTEM`, `ECALL`, CSRs de modo
  máquina — corre un binario real de `riscv64-unknown-elf-gcc`.
- csim + csynth OK: **128.25 MHz**, todos los *loop constraints*
  satisfechos.

## Objetivo de esta línea: `Zve32x`

`Zve32x` es el perfil vectorial **más pequeño** del estándar RVV 1.0
(sección 18.2). Exige, además de lo que ya hay:

| Requisito | Estado |
|---|---|
| `vsetvli`/`vsetivli`/`vsetvl` | ✅ ya |
| VLEN mínimo 32 (acá 128) | ✅ ya |
| **EEW 8, 16 y 32** | ❌ solo 32 — implica rediseñar el datapath |
| **Set entero completo** (~150+ formas `.vv`/`.vx`/`.vi`) | ❌ 5 instrucciones |
| **Máscara** (`vm=0` con `v0`) + instrucciones de máscara | ❌ |
| Reducciones y permutaciones | ❌ |
| Punto fijo saturante (`vsadd`/`vsmul`/`vnclip`…) | ❌ |
| Todos los modos de load/store (strided, indexed, segmentado, fault-only-first, whole-register) | ❌ solo unit-stride |
| **`vstart`** (reinicio a mitad de una vectorial) | ❌ pendiente |
| **Traps precisas** (obligatorio en todo Zve\*) | ✅ **hechas** (ver bare-metal abajo) |

**Nota de honestidad**: la conformidad real con `Zve32x` se declara solo
tras pasar los RISC-V Architecture Compatibility Tests. Mientras eso no
ocurra, lo correcto es describir el trabajo como *"subconjunto de RVV
1.0 orientado a Zve32x"*, no como una implementación conforme.

## Estructura

Un solo archivo por cosa — sin versiones duplicadas:

| Archivo | Qué es |
|---|---|
| `rv32_ooo.cpp` / `.h` | **El core** (única versión) |
| `rv32_ooo_tb.cpp` | **Testbench único**: corre las 4 suites en secuencia |
| `run_hls.tcl` | **Script único**: C-simulation + C-synthesis |
| `run_hls_impl.tcl` | Implementación post-P&R en Vivado (flujo distinto) |
| `rv32i_defs.h`, `immediates_hls.h`, `fp_ops.h`, `rv32c_defs.h` | Dependencias del ISA |
| `trap_elf.h`, `full_elf.h`, `printf_elf.h` | Binarios ELF embebidos (datos de prueba) |
| `baremetal/` | Fuentes de esos binarios + `build.sh` que los regenera |

### Las 4 suites del testbench

| Suite | Qué verifica |
|---|---|
| **A. ISA + RVV** | I+M+F+C y RVV Fases 1-3 (máscara, EEW 8/16/32, comparaciones, merge, lógica de máscaras, `vid`/`viota`/`vcpop`/`vfirst`), con evidencia de ejecución fuera de orden y coprocesamiento |
| **B. Excepciones** | ELF real: handler en `mtvec` → `ECALL` → `MRET` → **el programa continúa** |
| **C. Sistema** | ELF real: UART, interrupción de timer, modos M/U |
| **D. `printf`** | ELF real enlazado contra **newlib** (~54 KB) |

## Cómo correr

```bash
vitis_hls -f run_hls.tcl        # csim (4 suites) + csynth
vitis_hls -f run_hls_impl.tcl   # implementación post-P&R en Vivado
sh baremetal/build.sh           # regenera los 3 binarios ELF
```

Verificación rápida sin Vitis (compilador nativo):
```bash
g++ -std=c++17 -Wno-unknown-pragmas -I <ruta-Vitis>/include \
    rv32_ooo.cpp rv32_ooo_tb.cpp -o test && ./test
```

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

## ✅ Bare-metal COMPLETO — en AMBAS pistas

> La pista TLM tiene ahora las mismas capacidades y ejecuta **los mismos
> binarios ELF** (mismo UART en `0x20000000`), obteniendo los **mismos
> números de ciclo**. Allá el UART es además un periférico TLM real
> colgado del Bus. Ver
> [`../RV32IMFC+RVV+OOO-TLM/README.md`](../RV32IMFC+RVV+OOO-TLM/README.md).


Todo verificado ejecutando **binarios ELF reales compilados con
`riscv64-unknown-elf-gcc`**, sin backdoor: los resultados salen de que el
programa realmente corrió.

| Componente | Estado |
|---|---|
| Loader de ELF32 | ✅ parsea `PT_LOAD` (carga a `imem` y `dmem`: el binario usa espacio unificado, el core es Harvard) |
| Opcode `SYSTEM`, `FENCE` | ✅ |
| Instrucciones y banco de CSRs | ✅ `CSRRW`/`S`/`C` + inmediato; `mstatus`/`mtvec`/`mepc`/`mcause`/`mie`/`mip`/`mscratch`/… |
| **Excepciones precisas y reanudables** | ✅ `ECALL`/`EBREAK` → `mtvec`, handler, `MRET` → el programa **continúa** |
| **Interrupciones de timer** | ✅ `mtime`/`mtimecmp`, `mie.MTIE`, `mstatus.MIE`, `mcause=0x80000007` |
| **Modos de privilegio M/U** | ✅ `mstatus.MPP`/`MPIE`; `ECALL` reporta causa 8 desde U-mode y 11 desde M-mode |
| **UART mapeado en memoria** | ✅ store a `0x20000000` (región MMIO) → salida por consola; misma dirección que la pista TLM, para que ambas ejecuten los mismos binarios |
| **Excepción de instrucción ilegal** (causa 2) | ✅ un opcode no soportado **atrapa** en vez de ser un *no-op silencioso* |
| **`printf` de la biblioteca C real** | ✅ binario enlazado contra **newlib** (~54 KB) corriendo sobre el core |

### Por qué esto costó mucho menos de lo estimado

Se estimaron semanas y tomó horas, por una razón **arquitectónica** que
vale la pena en la defensa: como este core **no especula**, un trap solo
puede tomarse en la **cabeza del ROB** — donde todo lo anterior ya
committeó y nada posterior tocó el estado arquitectónico. Basta con un
*flush* total (`pipeline_flush()`); no hace falta la maquinaria de
recuperación especulativa que encarece esto en cores con predicción de
saltos.

Y una vez hecho eso, **las interrupciones salieron casi gratis**: una
interrupción es el mismo trap tomado en el mismo punto preciso, con otro
código de causa (bit 31 = asíncrona). Lo único que hubo que agregar fue
el *disparador* (contadores `mtime`/`mtimecmp` y los bits de habilitación).

En resumen: **la decisión de no especular, que limita el ILP, abarata
todo lo que se resuelve en el commit.** Es un trade-off real y medible,
no una excusa.

Todo esto lo verifican las suites B, C y D del testbench único
(`vitis_hls -f run_hls.tcl`). Las fuentes de los binarios están en
`baremetal/` (`trap.c`, `full.c`, `printf_demo.c` + `syscalls.c`) y se
regeneran con `sh baremetal/build.sh`.

### Notas de alcance honestas

- **UART**: el modelo imprime por consola en `csim` y está excluido de la
  síntesis (`#ifndef __SYNTHESIS__`). En hardware real esa dirección debe
  mapear a un periférico UART externo por AXI — el core solo genera el
  acceso.
- **`mtime`/`mtimecmp`**: en hardware real son registros MMIO del CLINT;
  acá se exponen como CSRs de M-mode para no necesitar otro periférico.
- **Sin `mtvec` instalado** un `ECALL` detiene el core (convención del
  simulador, equivale al `exit` de un runtime) en vez de saltar a 0.
- **Memoria**: `imem`/`dmem` son de **64 KB** cada una, dimensionadas para
  que quepa un binario con biblioteca C (el `printf` de newlib son ~54 KB
  de código él solo).
- **Arranque**: el core resetea en `pc=0`, pero el *entry point* de un ELF
  puede estar en otra dirección. El loader instala un **stub de arranque**
  (`jal` al entry) en la dirección 0 — lo mismo que hace el vector de
  reset de un core real.
- **Toolchain**: para `printf` se usa el compilador que trae **Vivado**,
  que sí incluye biblioteca C para `rv32imfc`; el paquete
  `gcc-riscv64-unknown-elf` de Ubuntu viene sin libc.
- **Falta**: modo supervisor (S) y memoria virtual (solo hacen falta para
  correr un SO, no para bare-metal), interrupciones externas/software
  (solo está la de timer), chequeo de privilegio en el acceso a CSRs, y
  excepciones por acceso desalineado.

## ⬜ Fases pendientes hacia `Zve32x`

**Las seis fases están completas.** Lo único que queda abierto hacia
`Zve32x` es:

| Qué falta | Por qué | Esfuerzo estimado |
|---|---|---|
| **`LMUL≠1`** | Agrupar 2/4/8 registros como uno. No es una fase: es un cambio transversal a cómo se indexa el banco (la Fase 4d ya abrió el camino con los grupos `EMUL=2`) | ~1 día |

Fuera de alcance **a propósito**: `SEW=64` y punto flotante vectorial —
`Zve32x` no los exige (eso ya sería `Zve32f`). El `vxrm` está fijo en
`rnu`, el modo de redondeo por defecto.

## Nota de honestidad sobre la conformidad

Esto **no es todavía una implementación conforme de `Zve32x`**. La
conformidad se declara solo tras pasar los RISC-V Architecture
Compatibility Tests, y además falta `LMUL≠1`. Lo correcto al
describir este trabajo es *"subconjunto de RVV 1.0 orientado a Zve32x,
con codificación y semántica verificadas contra la especificación
oficial"*.
