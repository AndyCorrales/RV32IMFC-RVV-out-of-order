# Script UNICO de Vitis HLS para el core RV32IMFC + RVV + OOO.
# Uso: vitis_hls -f run_hls.tcl
#
# Corre C-simulation (las cuatro suites del testbench unico:
# ISA+RVV, excepciones, sistema y printf de newlib) y despues
# C-synthesis contra la parte del Kria KV260.
#
# Para la implementacion post-P&R en Vivado, ver run_hls_impl.tcl.

open_project -reset rv32_ooo_proj
set_top rv32_ooo_tick

add_files rv32_ooo.cpp
add_files -tb rv32_ooo_tb.cpp
add_files -tb trap_elf.h
add_files -tb full_elf.h
add_files -tb printf_elf.h

open_solution -reset "solution1"
set_part {xck26-sfvc784-2LV-c}
create_clock -period 10 -name default

csim_design
csynth_design

puts "C-simulation y C-synthesis terminados."
puts "Reporte: rv32_ooo_proj/solution1/syn/report/rv32_ooo_tick_csynth.rpt"
exit
