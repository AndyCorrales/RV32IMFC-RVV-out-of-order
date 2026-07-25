# Resultados — RISC-V SoC (RV32IMFC + OoO + RVV)

Resultados **medidos** de las dos pistas nuevas del SoC (`RISCV-SoC/` en
HLS y `RISCV-SoC-TLM/` en SystemC/TLM), reproducibles con `make check`.
Todos los números de esta página salen de correr los testbenches; ninguno
está estimado a mano salvo donde se indica.

Reproducir todo:

```bash
make check-soc          # SoC HLS: 8 suites + AXPY
make check-soc-tlm      # SoC TLM: 8 suites + AXPY
make check-axpy-parity  # paridad de ciclos HLS vs TLM (diff exacto)
```

---

## 1. Verificación funcional — testbenches (8 suites)

Cada suite corre programas ensamblados a mano (o binarios ELF reales de
gcc) y verifica el estado observado sin *backdoors*. Un check falla si el
comportamiento correcto y el incorrecto darían resultados distintos.

| Suite | Qué verifica | Checks |
|---|---:|
| **A** — ISA + RVV Fases 1-3 | I+M+F+C escalar; aritmética vectorial, EEW variable, máscaras, comparaciones, merge | 136 |
| **A2** — RVV Fase 4 | reducciones, permutaciones, punto fijo saturante/promediado | 23 |
| **A3** — RVV Fase 5 | strided, indexado (gather/scatter), segmentado, fault-only-first, registro completo, máscara | 30 |
| **A4** — RVV Fase 4d + 6 | widening/narrowing (EMUL=2), `vstart` | 17 |
| **E** — Predictor TAGE | el lazo corre bajo especulación; el branch alternante se aprende | 4 |
| **B** — Excepciones | trap preciso y **reanudable** con ELF real (handler + MRET) | 4 |
| **C** — Sistema | UART + interrupción de timer + modos M/U (ECALL causa 8 vs 11) | 3 |
| **D** — printf | `printf` de newlib completo sobre el core (~5300 ciclos) | 1 |

| Pista | Total | Fallos |
|---|---:|---:|
| **SoC HLS** (`soc_tb`) | **218** | **0** |
| **SoC TLM** (`main`) | **219** | **0** |

La diferencia de 1 check (137 vs 136 en la suite A) es una comprobación
extra que la pista TLM traía desde antes; no es una funcionalidad que le
falte a HLS. **Ambas pistas pasan todo.**

---

## 2. Predictor de saltos (TAGE) — suite E

Programa: lazo de 32 iteraciones con un branch **alternante** (tomado,
no-tomado, tomado…) anidado. El alternante es el caso que clava a un
predictor bimodal puro en ~50 % de acierto para siempre; TAGE lo aprende
con un bit de historia.

| Métrica | Valor |
|---|---:|
| Branches condicionales retirados | 64 |
| Mispredicts | 6 |
| **Acierto** | **90.6 %** |
| Ciclos totales | 241 |

Idéntico en HLS y TLM.

---

## 3. Experimento AXPY (V-3/V-4) — escalar vs vectorial

`y[i] = a·x[i] + y[i]`, enteros, n = 1024, dos versiones que producen el
mismo resultado arquitectónico: una escalar (lw/mul/add/sw) y una
vectorial strip-mined (`vsetvli` + `vle32`/`vmul.vx`/`vadd.vv`/`vse32`).
Experimento original de Daniel Chacón, portado al SoC sin tocar la lógica
de medición.

### Sobre el SoC (memoria arbitrada, latencias reales)

| Versión | Ciclos | Instr. retiradas | IPC | Ciclos/elemento |
|---|---:|---:|---:|---:|
| escalar | 12 297 | 9 220 | 0.750 | 12.01 |
| vectorial | 7 430 | 2 820 | 0.380 | **7.26** |

- **Reducción de instrucciones**: 3.27× (9 220 → 2 820) — cada
  instrucción vectorial hace el trabajo de 4 elementos.
- **Speedup en ciclos**: **1.66×** (12 297 → 7 430).
- Números **idénticos** en HLS y TLM (ver §5).

### Comparación con el core estable (memoria idealizada)

| | Core estable | SoC | Por qué cambia |
|---|---:|---:|---|
| escalar (ciclos) | 12 294 | 12 297 | ~igual: lo que TAGE ahorra en el branch lo consume el árbitro en la contención load/store |
| vectorial (ciclos) | 4 615 | 7 430 | la VLSU del SoC mueve **un elemento por ciclo** por el puerto arbitrado; el core estable modelaba 16 accesos simultáneos (irreal) y el VMUL costaba 1 ciclo en vez de sus 4 reales |
| **speedup** | **2.66×** | **1.66×** | el 2.66× era con memoria mágica; 1.66× es con memoria y latencias realistas |

**Cómo leer estos dos números juntos** (esto es análisis, no un
retroceso): AXPY hace **3 accesos a memoria por elemento** (2 loads + 1
store). Con un puerto de 1 acceso/ciclo, el piso físico es **3
ciclos/elemento** — ninguna implementación puede bajar de ahí con este
bus. El vectorial está en 7.26 (piso + overhead de vsetvli/branch/VMUL);
con VLEN mayor ese overhead se amortiza. El 1.66× medido contra ese techo
real es el resultado honesto; el 2.66× ideal marca el límite superior si
el ancho de banda de memoria no fuera la restricción.

---

## 4. Síntesis (Vitis HLS csynth, AMD Kria KV260, xck26-sfvc784-2LV-c)

| Métrica | Core estable | **SoC** |
|---|---:|---:|
| Fmax estimado | 128.25 MHz | **135.86 MHz** |
| LUT | 58 061 (49 %) | 60 676 (51 %) |
| FF | 16 494 (7 %) | 23 160 (9 %) |
| DSP | 42 (3 %) | 42 (3 %) |
| BRAM | 6 (2 %) | 8 (2 %) |
| Loop constraints | satisfechos | satisfechos |

**El Fmax subió** (+6 %) al agregar TAGE + VIQ + árbitro. No es casualidad:
serializar la VLSU a un elemento por ciclo eliminó el multiplexor ancho de
los 16 accesos simultáneos, que estaba en el camino crítico. Se pagó en
**área** (+2 600 LUT, +6 600 FF por el predictor, la cola y el árbitro),
no en frecuencia.

---

## 5. Paridad ciclo a ciclo HLS ↔ TLM

Las dos pistas modelan la **misma** microarquitectura; deben dar los
mismos ciclos. `make check-axpy-parity` lo comprueba como regresión
ejecutable (`diff` exacto). Barrido del AXPY vectorial:

| n | HLS (ciclos) | TLM (ciclos) | ¿iguales? |
|---:|---:|---:|:---:|
| 4 | 35 | 35 | ✅ |
| 16 | 122 | 122 | ✅ |
| 64 | 470 | 470 | ✅ |
| 256 | 1 862 | 1 862 | ✅ |
| 1024 | 7 430 | 7 430 | ✅ |

**Idénticos en todos los tamaños.** Esta paridad no es cosmética: al
portar el AXPY, la exigencia de que los ciclos coincidieran **destapó dos
bugs reales** en el SoC HLS —

1. un overflow de ancho silencioso (`remaining` en 2 bits no aguantaba
   `VMUL_LAT=4`, y el multiplicador vectorial completaba en 1 ciclo);
2. un gate perdido que dejaba a `vsetvli` despacharse especulativamente
   bajo un branch (un fallo de **corrección**, no solo de rendimiento).

Con ambos corregidos las pistas coinciden al ciclo. El modelo TLM
funcionó como oráculo del RTL — que es exactamente el argumento
metodológico de mantener dos pistas.

---

## 6. Cómo interpretar los resultados (resumen para la defensa)

- **IPC escalar 0.750** = 75 % del techo estructural (dispatch de 1
  instr/ciclo). Sólido para un ROB de 8 en un kernel con load-use, `mul`
  de 3 ciclos y un branch cada 9 instrucciones.
- **El IPC vectorial (0.380) es la métrica equivocada** para código
  vectorial: cada instrucción hace 4 elementos. La métrica correcta es
  ciclos/elemento (12.01 → 7.26) o el speedup (1.66×).
- **TAGE 90.6 %** en un patrón que un bimodal falla al 50 %, con solo 3
  tablas de 128 entradas.
- **Paridad HLS/TLM exacta**, convertida en regresión y con dos bugs
  cazados en el camino.

*Última actualización: números medidos el 2026-07-24.*
