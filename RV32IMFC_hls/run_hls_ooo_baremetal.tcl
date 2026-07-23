# Script de Vitis HLS para el test BARE-METAL del core OOO (rv32_ooo.cpp).
# Uso: vitis_hls -f run_hls_ooo_baremetal.tcl
#
# Mismo top (rv32_ooo_tick) que run_hls_ooo_core.tcl, pero con el
# testbench de bare-metal: carga un binario ELF32 REAL compilado con
# riscv64-unknown-elf-gcc (embebido en bm_elf.h), lo corre hasta su
# propio ECALL, y verifica los resultados (loop + acceso a CSR).
#
# Demuestra los items 1-5 del plan bare-metal (loader de ELF, opcode
# SYSTEM, ECALL, instrucciones CSR, banco de CSRs) sobre el core
# sintetizable. El item 6 (traps precisas) queda documentado como futuro.

open_project -reset rv32_ooo_bm_proj
set_top rv32_ooo_tick

add_files rv32_ooo.cpp
add_files -tb rv32_ooo_baremetal_tb.cpp
add_files -tb bm_elf.h

open_solution -reset "solution1"

set_part {xck26-sfvc784-2LV-c}
create_clock -period 10 -name default

csim_design
csynth_design

puts "C-simulation (binario ELF real) y C-synthesis terminados."
exit
