# Script de Vitis HLS para el decoder RVV minimo (rv32_vector.cpp).
# Uso: vitis_hls -f run_hls_vector.tcl
#
# Proyecto separado de rv32_core_proj / rv32_ooo_proj -- no los toca.
# Genera "rv32_vector_proj", corre csim + csynth, reporte en
# rv32_vector_proj/solution1/syn/report/rv32_vector_step_csynth.rpt

open_project -reset rv32_vector_proj
set_top rv32_vector_step

add_files rv32_vector.cpp
add_files -tb rv32_vector_tb.cpp

open_solution -reset "solution1"

set_part {xck26-sfvc784-2LV-c}
create_clock -period 10 -name default

csim_design
csynth_design

puts "C-simulation y C-synthesis del decoder RVV terminados."
puts "Reporte: rv32_vector_proj/solution1/syn/report/rv32_vector_step_csynth.rpt"

exit
