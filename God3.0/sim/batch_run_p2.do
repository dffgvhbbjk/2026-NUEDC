# ============================================================================
# batch_run_p2.do — P2 新模块 (wave_smooth + wave_uart_export) 仿真批处理脚本
# 用法 (工程根目录下): vsim -c -do "do sim/batch_run_p2.do"
# ============================================================================

# ---- 1. 创建 work 库 ----
if {[file exists work]} { vdel -lib work -all }
vlib work
vmap work work

# ---- 2. 编译 DUT ----
puts ">>> Compiling DUT..."
vlog -work work -novopt rtl/wave_smooth.v
vlog -work work -novopt rtl/uart_tx.v
vlog -work work -novopt rtl/wave_uart_export.v

# ---- 3. 编译 TB ----
puts ">>> Compiling TB..."
vlog -work work -novopt sim/tb_wave_smooth.v
vlog -work work -novopt sim/tb_wave_uart_export.v

# ---- 4. 依次运行两个 TB ----
# onfinish stop: 防止 TB 的 $finish 直接退出 vsim, 否则第二个 TB 不会运行
puts ">>> Running tb_wave_smooth..."
vsim -t 1ns -L work work.tb_wave_smooth
onfinish stop
run -all
quit -sim

puts ">>> Running tb_wave_uart_export..."
vsim -t 1ns -L work work.tb_wave_uart_export
onfinish stop
run -all

quit -f
