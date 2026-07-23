# Implementacion post-P&R (Vivado) del core OOO+RVV+bare-metal, via
# Vitis HLS export_design -flow impl. A diferencia de csynth_design (que
# da ESTIMADOS), esto corre sintesis + place & route reales en Vivado y
# reporta utilizacion/timing POST-IMPLEMENTACION -- los numeros que
# conviene citar en "Resultados y Analisis" del articulo.
#
# export_design maneja automaticamente la IP de punto flotante que
# instancia el RTL generado (faddfsub/fdiv/fmul/fsqrt/...), cosa que un
# flujo Vivado a mano sobre el RTL crudo dejaria como caja negra.
#
# Precondicion: haber corrido run_hls_ooo_core.tcl (deja el RTL en
# rv32_ooo_proj/solution1/syn/verilog/). Uso:
#   vitis_hls -f run_hls_ooo_impl.tcl
#
# El reporte post-impl queda en:
#   rv32_ooo_proj/solution1/impl/report/verilog/

open_project rv32_ooo_proj
open_solution solution1
set_part {xck26-sfvc784-2LV-c}

# -flow impl: corre Vivado synth + place & route (out-of-context) y
# genera los reportes de utilizacion/timing reales.
export_design -flow impl -rtl verilog -format ip_catalog

puts "Implementacion post-P&R terminada."
puts "Reportes en rv32_ooo_proj/solution1/impl/report/verilog/"
exit
