# ============================================================================
# batch_run_defect.do — 缺陷检测模块仿真 (defect_analyzer + defect_distance_calc)
# 用法 (工程根目录下): vsim -c -do "do sim/batch_run_defect.do"
# ============================================================================

if {[file exists work]} { vdel -lib work -all }
vlib work
vmap work work

puts ">>> Compiling DUT..."
vlog -work work -novopt rtl/defect_analyzer.v
vlog -work work -novopt rtl/defect_distance_calc.v

puts ">>> Compiling TB..."
vlog -work work -novopt sim/tb_defect_analyzer.v
vlog -work work -novopt sim/tb_defect_distance_calc.v

puts ">>> Running tb_defect_distance_calc..."
vsim -t 1ns -L work work.tb_defect_distance_calc
onfinish stop
run -all

puts ">>> Running tb_defect_analyzer..."
vsim -t 1ns -L work work.tb_defect_analyzer
onfinish stop
run -all

quit -f
