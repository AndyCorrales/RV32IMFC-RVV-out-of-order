# Script de Vitis HLS para el RISC-V SoC (core escalar OoO + TAGE +
# coprocesador vectorial con VIQ + interconnect con arbitro).
# Uso: vitis_hls -f run_hls.tcl
#
# Corre C-simulation (las 8 suites del testbench: ISA+RVV, fases 4/5/6,
# TAGE, excepciones, sistema y printf de newlib) y despues C-synthesis
# contra la parte del Kria KV260.

open_project -reset riscv_soc_proj
set_top riscv_soc_tick

add_files soc_top.cpp
add_files -tb soc_tb.cpp
add_files -tb trap_elf.h
add_files -tb full_elf.h
add_files -tb printf_elf.h

open_solution -reset "solution1"
set_part {xck26-sfvc784-2LV-c}
create_clock -period 10 -name default

csim_design
csynth_design

puts "C-simulation y C-synthesis terminados."
puts "Reporte: riscv_soc_proj/solution1/syn/report/riscv_soc_tick_csynth.rpt"
exit
