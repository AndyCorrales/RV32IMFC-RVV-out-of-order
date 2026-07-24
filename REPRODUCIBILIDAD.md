# Reproducibilidad — compilar y correr todo

Este documento describe cómo reproducir **todas** las pruebas del
proyecto: CoreMark en el modelo TLM, los baselines escalares AXPY/GEMM,
y los testbenches HLS (core escalar, demo OoO y core OoO Tomasulo-lite).

## Requisitos

| Herramienta | Verificado con | Uso |
|---|---|---|
| g++ (C++17) | 13+ | simulador TLM y C-sim de HLS |
| SystemC | 3.0.1-Accellera | simulador TLM (`-lsystemc`) |
| riscv64-unknown-elf-gcc | 14.2.0 | binarios bare-metal (CoreMark, tests) |
| git | — | clona CoreMark y los tipos ap_int si faltan |
| Vitis HLS (opcional) | 2024.1 | solo para síntesis; la C-sim NO lo requiere |

> La C-sim de HLS corre con g++ puro usando los tipos `ap_int`
> open-source de Xilinx (`HLS_arbitrary_Precision_Types`), que el
> Makefile clona automáticamente a `third_party/`. No hace falta Vitis
> para reproducir la verificación funcional del OoO.

## Todo de una vez

Desde la **raíz del repo**:

```bash
make check
```

Corre, en orden: (1) CoreMark rápido (10 iter) en el TLM y verifica los
CRC golden; (2) los tres testbenches HLS. Termina con
`TODAS las pruebas de reproducibilidad pasaron (TLM + HLS).`
Cualquier fallo detiene el make con código de error (apto para CI).

## Resultados esperados

**CoreMark (TLM)** — los CRC deben coincidir con la referencia publicada
por EEMBC/Ibex para esta configuración (semillas 0/0/0x66, 2K perf run):

```
seedcrc          : 0xe9f5
[0]crclist       : 0xe714
[0]crcmatrix     : 0x1fd7
[0]crcstate      : 0x8e3a
Correct operation validated.        (con ITERATIONS=1000 y CPU_HZ=25 MHz)
```

Con 1000 iteraciones: ~313.8 M ciclos → ~313 803 ciclos/iteración →
**≈ 3.19 CoreMark/MHz** con IPC = 1.00 (modelo funcional monociclo: el
número mide conteo de instrucciones del ISA, no microarquitectura).

**Core OoO (HLS C-sim)** — `make check-ooo` debe terminar en
`Todos los checks pasaron.` / `PASS: core OoO.` El testbench verifica:
28 dispatches = 28 commits (halted ~ciclo 53), estado arquitectónico
reconstruido SOLO del stream de commit, stores a memoria al commit, y
evidencia de ejecución fuera de orden con ciclos medidos:

```
OK  OOO: d3 (addi) antes que d2 (mul)    (ciclos 6 < 7)
OK  OOO: d12 (sub) antes que d11 (div)   (ciclos 17 < 23)
OK  OOO: d24 (addi) antes que d23 (fdiv) (ciclos 42 < 48)  <- cruza bancos int/float
```

**AXPY / GEMM (TLM)** — cada uno imprime ciclos, instrucciones y
`validacion : 0 errores -> PASS` (resultado exacto en FP32 contra
referencia entera independiente).

## Objetivos del Makefile

| Objetivo | Qué hace |
|---|---|
| `make check` | Todo: CRC CoreMark + 3 TB de HLS (pass/fail) |
| `make check-coremark` | Solo regresión golden de CRC (CoreMark 10 iter) |
| `make check-ooo` | Solo el core OoO (`rv32_ooo_tb`) |
| `make check-hls` | Los 3 testbenches HLS |
| `make run-coremark` | CoreMark 1000 iteraciones (~1 min de simulación) |
| `make run-ooo` | Traza completa dispatch/completion/commit del OoO |
| `make run-axpy` / `run-gemm` | Baselines escalares FP32 |
| `make coremark-valid` | Variante VALIDATION_RUN (otras semillas) |
| `make coremark-noc` | Variante sin extensión C (`rv32im`) |
| `make sim` | Recompila solo el simulador TLM |
| `make clean` | Limpia binarios |

## Notas de build (por qué estos flags)

- **`-nostdlib -lgcc`**: bare-metal sin libc (el toolchain no trae
  libc/libgloss para el multilib rv32imc/ilp32); `libgcc` aporta la
  división de 64 bits y soft-float.
- **`-fno-builtin`**: imprescindible. A `-O2`, GCC reconoce el bucle del
  `memset` propio y lo reemplaza por una llamada a `memset` → recursión
  infinita que desborda el stack sobre el código. `mem_stubs.c` además
  blinda `mem*` con `optimize("no-tree-loop-distribute-patterns")`.
- **`-DSEED_METHOD=SEED_VOLATILE`**: en bare-metal no hay `argv`; con
  `SEED_ARG` CoreMark correría 0 iteraciones.
- El warning del linker `LOAD segment with RWX permissions` es
  inofensivo para un binario bare-metal cargado por el simulador.
- La regla de CoreMark de "mínimo 10 s" es para hardware real; en
  simulación se satisface con `CPU_HZ=25 MHz` (en
  `baremetal/core_portme.c`) e `ITERATIONS=1000`, o se ignora: los CRC
  validan la correctitud con cualquier número de iteraciones.

## Caracterización de instrucciones

`src/processor.h` incluye un histograma por clase (ALU/LOAD/STORE/
BRANCH/JUMP/MUL/DIV/FP/...) que se imprime al final de cada simulación
(`dump_hist()`), útil para reportar el *mix* de instrucciones de
CoreMark/AXPY/GEMM en la sección de análisis.

## Síntesis (flujo oficial, requiere Vitis HLS)

```bash
cd RV32IMFC_hls
vitis_hls -f run_hls_ooo_core.tcl    # core OoO
vitis_hls -f run_hls.tcl             # core escalar
```
