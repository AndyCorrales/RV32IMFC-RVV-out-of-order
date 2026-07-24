# ==============================================================
#  Makefile raiz — RV32IMFC-RVV-out-of-order
#  Reproducibilidad completa: TLM (CoreMark + baselines) y HLS
#  (core escalar, demo OoO y core OoO Tomasulo-lite) sin Vitis.
#
#  Colocar en la RAIZ del repo. Objetivos principales:
#    make check          -> TODO: CRC golden de CoreMark + 3 TB de HLS
#    make check-ooo      -> solo el core OoO (rv32_ooo_tb)
#    make check-hls      -> los 3 testbenches HLS (escalar, demo, OoO)
#    make check-coremark -> regresion CRC golden en el TLM
#    make run-coremark   -> CoreMark completo (1000 iter) en el TLM
#    make run-axpy/run-gemm -> baselines escalares FP32 en el TLM
#
#  La C-sim de HLS corre con g++ usando los tipos ap_int open-source
#  de Xilinx (HLS_arbitrary_Precision_Types) — no requiere Vitis.
#  El flujo oficial de sintesis sigue siendo run_hls_ooo_core.tcl.
# ==============================================================

TLM    := RV32IMFC_tlm
HLS    := RV32IMFC_hls
RVVHLS := RV32IMFC+RVV+OOO-HLS
RVVTLM := RV32IMFC+RVV+OOO-TLM

# --- Toolchains ---
CC   := riscv64-unknown-elf-gcc
CXX  := g++
SIM  := $(TLM)/riscv_sim

# --- Flags bare-metal ---
ARCH     ?= rv32imc
FP_ARCH  := rv32imfc
ABI      := ilp32
CFLAGS   := -march=$(ARCH) -mabi=$(ABI) -O2 -nostdlib -fno-builtin
LDFLAGS  := -lgcc
LDSCRIPT := $(TLM)/baremetal/link.ld
INC      := -I $(TLM)/baremetal
BM       := $(TLM)/baremetal/crt0.S $(TLM)/baremetal/syscall_stubs.c \
            $(TLM)/baremetal/mem_stubs.c

# --- CoreMark ---
CM_SRC   := $(TLM)/coremark_src
CM_DEFS  := -DPERFORMANCE_RUN=1 -DSEED_METHOD=SEED_VOLATILE -DMEM_METHOD=MEM_STACK
CM_ITER  ?= 1000
CM_PORT  := $(TLM)/baremetal/core_portme.c
CM_FILES := $(CM_SRC)/core_main.c $(CM_SRC)/core_list_join.c \
            $(CM_SRC)/core_matrix.c $(CM_SRC)/core_state.c $(CM_SRC)/core_util.c

# --- Tipos ap_int open-source (para C-sim de HLS sin Vitis) ---
AP_TYPES := third_party/ap_types
AP_INC   := -I $(AP_TYPES)/include
HLS_STD  := -std=c++14 -O2

.PHONY: all sim clean check check-coremark check-hls check-ooo \
        coremark coremark-fast coremark-valid coremark-noc run-coremark \
        axpy gemm run-axpy run-gemm ooo-tb demo-tb core-tb \
        rvv-ooo-tb run-rvv-ooo check-rvv-ooo sim-rvv run-rvv-tlm check-rvv-tlm \
        axpy-ooo run-axpy-ooo check-axpy-ooo \
        run-ooo run-demo run-core

all: sim coremark ooo-tb

# ===================== TLM: simulador =====================
sim: $(SIM)
$(SIM): $(TLM)/src/main.cpp $(wildcard $(TLM)/src/*.h)
	$(CXX) -std=c++17 -O2 -o $(SIM) $(TLM)/src/main.cpp -lsystemc -lpthread

$(CM_SRC):
	git clone --depth 1 https://github.com/eembc/coremark.git $(CM_SRC)

# ===================== TLM: CoreMark ======================
coremark: $(CM_SRC)
	$(CC) $(CFLAGS) -T $(LDSCRIPT) -I $(CM_SRC) $(INC) \
	    -DITERATIONS=$(CM_ITER) $(CM_DEFS) \
	    $(BM) $(CM_PORT) $(CM_FILES) -o $(TLM)/coremark.elf $(LDFLAGS)

coremark-fast:
	$(MAKE) coremark CM_ITER=10

coremark-valid: $(CM_SRC)
	$(CC) $(CFLAGS) -T $(LDSCRIPT) -I $(CM_SRC) $(INC) \
	    -DITERATIONS=$(CM_ITER) -DVALIDATION_RUN=1 \
	    -DSEED_METHOD=SEED_VOLATILE -DMEM_METHOD=MEM_STACK \
	    $(BM) $(CM_PORT) $(CM_FILES) -o $(TLM)/coremark_valid.elf $(LDFLAGS)

coremark-noc:
	$(MAKE) coremark ARCH=rv32im

run-coremark: coremark $(SIM)
	cd $(TLM) && ./riscv_sim coremark.elf

# Regresion golden: los CRC deben ser 0xe714 / 0x1fd7 / 0x8e3a.
check-coremark: $(SIM)
	$(MAKE) coremark-fast
	@echo "=== [TLM] CoreMark: verificando CRC golden ==="
	@cd $(TLM) && ./riscv_sim coremark.elf > /tmp/cm_check.log 2>&1 ; \
	 grep -E "crclist|crcmatrix|crcstate" /tmp/cm_check.log ; \
	 if grep -q "0xe714" /tmp/cm_check.log && \
	    grep -q "0x1fd7" /tmp/cm_check.log && \
	    grep -q "0x8e3a" /tmp/cm_check.log ; then \
	     echo "PASS: CRC de CoreMark coinciden con la referencia." ; \
	 else \
	     echo "FAIL: CRC de CoreMark NO coinciden." ; exit 1 ; \
	 fi

# =============== TLM: baselines escalares =================
axpy: $(TLM)/tests/axpy.c
	$(CC) -march=$(FP_ARCH) -mabi=$(ABI) -O2 -nostdlib -fno-builtin \
	    -T $(LDSCRIPT) $(INC) $(BM) $(TLM)/tests/axpy.c \
	    -o $(TLM)/axpy.elf $(LDFLAGS)
gemm: $(TLM)/tests/gemm.c
	$(CC) -march=$(FP_ARCH) -mabi=$(ABI) -O2 -nostdlib -fno-builtin \
	    -T $(LDSCRIPT) $(INC) $(BM) $(TLM)/tests/gemm.c \
	    -o $(TLM)/gemm.elf $(LDFLAGS)

run-axpy: axpy $(SIM)
	cd $(TLM) && ./riscv_sim axpy.elf
run-gemm: gemm $(SIM)
	cd $(TLM) && ./riscv_sim gemm.elf

# ========== HLS: C-sim reproducible sin Vitis =============
$(AP_TYPES):
	mkdir -p third_party
	git clone --depth 1 \
	    https://github.com/Xilinx/HLS_arbitrary_Precision_Types.git $(AP_TYPES)

# Core OoO Tomasulo-lite (RV32IMFC): el testbench autoverifica resultados
# arquitectonicos, evidencia OOO (completions adelantadas) y stores al commit.
ooo-tb: $(AP_TYPES)
	$(CXX) $(HLS_STD) $(AP_INC) -o $(HLS)/rv32_ooo_tb \
	    $(HLS)/rv32_ooo.cpp $(HLS)/rv32_ooo_tb.cpp

# Demo del mecanismo OoO (ROB + 2 FUs con latencias distintas).
demo-tb: $(AP_TYPES)
	$(CXX) $(HLS_STD) $(AP_INC) -o $(HLS)/ooo_demo_tb \
	    $(HLS)/ooo_demo.cpp $(HLS)/ooo_demo_tb.cpp

# Core escalar HLS de referencia.
core-tb: $(AP_TYPES)
	$(CXX) $(HLS_STD) $(AP_INC) -o $(HLS)/rv32_core_tb \
	    $(HLS)/rv32_core.cpp $(HLS)/rv32_core_tb.cpp

run-ooo: ooo-tb
	$(HLS)/rv32_ooo_tb
run-demo: demo-tb
	$(HLS)/ooo_demo_tb
run-core: core-tb
	$(HLS)/rv32_core_tb

# El exit code de cada TB decide PASS/FAIL.
check-ooo: ooo-tb
	@echo "=== [HLS] rv32_ooo_tb (core OoO) ==="
	@$(HLS)/rv32_ooo_tb > /tmp/ooo_tb.log 2>&1 && \
	    { tail -1 /tmp/ooo_tb.log ; echo "PASS: core OoO." ; } || \
	    { tail -20 /tmp/ooo_tb.log ; echo "FAIL: core OoO." ; exit 1 ; }

check-hls: ooo-tb demo-tb core-tb
	@echo "=== [HLS] rv32_core_tb (escalar) ==="
	@$(HLS)/rv32_core_tb > /tmp/core_tb.log 2>&1 && echo "PASS: escalar HLS." || \
	    { tail -20 /tmp/core_tb.log ; echo "FAIL: escalar HLS." ; exit 1 ; }
	@echo "=== [HLS] ooo_demo_tb (mecanismo OoO) ==="
	@$(HLS)/ooo_demo_tb > /tmp/demo_tb.log 2>&1 && echo "PASS: demo OoO." || \
	    { tail -20 /tmp/demo_tb.log ; echo "FAIL: demo OoO." ; exit 1 ; }
	@$(MAKE) --no-print-directory check-ooo

# ===== OoO + RVV: pista HLS (C-sim con g++, sin Vitis) =====
rvv-ooo-tb: $(AP_TYPES)
	$(CXX) $(HLS_STD) $(AP_INC) -o $(RVVHLS)/rvv_ooo_tb \
	    $(RVVHLS)/rv32_ooo.cpp $(RVVHLS)/rv32_ooo_tb.cpp

run-rvv-ooo: rvv-ooo-tb
	$(RVVHLS)/rvv_ooo_tb

check-rvv-ooo: rvv-ooo-tb
	@echo "=== [HLS] OoO+RVV (ISA+RVV fases 1-4 + suites ELF/UART/newlib) ==="
	@$(RVVHLS)/rvv_ooo_tb > /tmp/rvv_ooo.log 2>&1 && \
	    { grep -c "^OK" /tmp/rvv_ooo.log | xargs -I{} echo "{} checks OK" ; \
	      echo "PASS: OoO+RVV HLS." ; } || \
	    { tail -25 /tmp/rvv_ooo.log ; echo "FAIL: OoO+RVV HLS." ; exit 1 ; }

# ===== OoO + RVV: pista TLM (SystemC) =====
sim-rvv: $(RVVTLM)/riscv_rvv_sim
$(RVVTLM)/riscv_rvv_sim: $(RVVTLM)/src/main.cpp $(wildcard $(RVVTLM)/src/*.h)
	$(CXX) -std=c++17 -O2 -o $(RVVTLM)/riscv_rvv_sim \
	    $(RVVTLM)/src/main.cpp -lsystemc -lpthread

run-rvv-tlm: sim-rvv
	$(RVVTLM)/riscv_rvv_sim

check-rvv-tlm: sim-rvv
	@echo "=== [TLM] OoO+RVV (verificacion cruzada TLM<->HLS) ==="
	@$(RVVTLM)/riscv_rvv_sim > /tmp/rvv_tlm.log 2>&1 && \
	    { grep -c "^OK" /tmp/rvv_tlm.log | xargs -I{} echo "{} checks OK" ; \
	      echo "PASS: OoO+RVV TLM." ; } || \
	    { tail -25 /tmp/rvv_tlm.log ; echo "FAIL: OoO+RVV TLM." ; exit 1 ; }

# ===== Experimento V-3/V-4: AXPY escalar vs vectorial en el OoO =====
axpy-ooo: $(AP_TYPES)
	$(CXX) $(HLS_STD) -I $(RVVHLS) $(AP_INC) -o $(RVVHLS)/axpy_ooo_tb \
	    $(RVVHLS)/axpy_ooo_tb.cpp $(RVVHLS)/rv32_ooo.cpp

run-axpy-ooo: axpy-ooo
	$(RVVHLS)/axpy_ooo_tb

check-axpy-ooo: axpy-ooo
	@echo "=== [HLS] AXPY escalar vs vectorial sobre el OoO (V-3/V-4) ==="
	@$(RVVHLS)/axpy_ooo_tb > /tmp/axpy_ooo.log 2>&1 && \
	    { grep -E "IPC|speedup|reduccion|escalar |vectorial " /tmp/axpy_ooo.log ; \
	      echo "PASS: AXPY OoO." ; } || \
	    { tail -20 /tmp/axpy_ooo.log ; echo "FAIL: AXPY OoO." ; exit 1 ; }

# =================== TODO junto ===========================
check: check-coremark check-hls check-rvv-ooo check-rvv-tlm check-axpy-ooo
	@echo ""
	@echo "TODAS las pruebas de reproducibilidad pasaron (TLM + HLS)."

clean:
	rm -f $(SIM) $(TLM)/*.elf $(HLS)/rv32_ooo_tb $(HLS)/ooo_demo_tb \
	      $(HLS)/rv32_core_tb $(RVVHLS)/rvv_ooo_tb $(RVVHLS)/axpy_ooo_tb $(RVVTLM)/riscv_rvv_sim \
	      /tmp/cm_check.log /tmp/ooo_tb.log /tmp/demo_tb.log /tmp/core_tb.log \
	      /tmp/rvv_ooo.log /tmp/rvv_tlm.log
